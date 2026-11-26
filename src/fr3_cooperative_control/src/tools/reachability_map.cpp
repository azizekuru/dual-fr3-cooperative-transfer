// Offline reachability map (no Gazebo, no move_group): for every arm, cell and grasp family,
// can the arm hold the box there with joint margin, how dexterous is it (Jacobian
// manipulability), and can it leave the cell with the box (lift 8 cm, retreat 12/8/6 cm)?
// Uses the same robot model, joint limits and margins as the transfer task.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "fr3_cooperative_control/arm_motion.hpp"
#include "fr3_cooperative_control/common.hpp"
#include "fr3_cooperative_control/world_model.hpp"

namespace fc = fr3_cooperative_control;

namespace
{
const std::vector<double> kReady{0, -0.785, 0, -2.356, 0, 1.571, 0.785};

struct Grasp
{
  std::string name;
  Eigen::Quaterniond hand;  // TCP orientation (TCP at the box centre)
  Eigen::Vector3d out;      // horizontal direction the hand leaves the cell
  // Fixed moves (relative, world frame) before the retreat along `out`.
  std::vector<Eigen::Vector3d> pre{Eigen::Vector3d(0, 0, 0.08)};
};

using fc::handFrame;  // hand z = approach, hand y = finger opening axis

struct Cell
{
  std::string id, surface;  // e.g. "shelf_2/L2/x+0.00/d0.03", "shelf_2"
  int level = 0;
  double dx = 0, inset = 0;
  fc::Cell cell;
  double inward = 0;  // 0 for table cells
};

struct Result
{
  int ik_ok = 0, seeds = 0;
  double best_margin = -1, best_manip = 0;
  double exit_d = 0, exit_margin = -1;
};

double manipulability(const moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * group, const moveit::core::LinkModel * tcp)
{
  Eigen::MatrixXd jacobian;
  if (!state.getJacobian(group, tcp, Eigen::Vector3d::Zero(), jacobian)) {return 0.0;}
  return std::sqrt(std::max(0.0, (jacobian * jacobian.transpose()).determinant()));
}

class Mapper
{
public:
  Mapper(rclcpp::Node::SharedPtr node, const moveit::core::RobotModelConstPtr & model)
  : node_(node), model_(model), scene_(model), world_(model, fc::TaskConfig::fromParameters(*node))
  {
    node->get_parameter_or("seeds", seeds_, seeds_);
    node->get_parameter_or("exit_checks", exit_checks_, exit_checks_);
    // Defaults match the task (planning probes 0.07 rad, executed motions 0.05 rad);
    // 0/0 gives the pure kinematic + collision map.
    node->get_parameter_or("plan_margin", plan_margin_, plan_margin_);
    node->get_parameter_or("exec_margin", exec_margin_, exec_margin_);
    node->get_parameter_or("x_offsets", x_offsets_, x_offsets_);
    node->get_parameter_or("insets", insets_, insets_);
    node->get_parameter_or("include_table", include_table_, include_table_);
    node->get_parameter_or("grasp_set", grasp_set_, grasp_set_);
    node->get_parameter_or("check_collisions", check_collisions_, check_collisions_);
    node->get_parameter_or("report_contacts", report_contacts_, report_contacts_);
    auto & state = scene_.getCurrentStateNonConst();
    state.setToDefaultValues();
    state.setJointGroupPositions("arm_1", kReady);
    state.setJointGroupPositions("arm_2", kReady);
    state.update();
    // Box collisions stay fully checked: the box is evaluated 1.5 mm above its surface.
  }

  std::vector<Cell> cells() const
  {
    std::vector<Cell> out;
    for (const auto & shelf : world_.shelves()) {
      for (int level = 1; level <= static_cast<int>(shelf.boards.size()); ++level) {
        for (double dx : x_offsets_) {
          for (double inset : insets_) {
            Cell c;
            char id[64];
            std::snprintf(id, sizeof(id), "%s/L%d/x%+.2f/d%.2f", shelf.name.c_str(), level, dx,
              inset);
            c.id = id;c.surface = shelf.name;c.level = level;c.dx = dx;c.inset = inset;
            c.cell = world_.shelfCell(shelf.name, level, dx, inset);c.inward = shelf.inward();
            out.push_back(c);
          }
        }
      }
    }
    for (double x : {-0.15, 0.0, 0.15}) {
      for (double y : {-0.15, 0.05, 0.25}) {
        if (!include_table_) {break;}
        Cell c;
        char id[64];
        std::snprintf(id, sizeof(id), "table/x%+.2f/y%+.2f", x, y);
        c.id = id;c.surface = "table";c.dx = x;c.inset = y;
        c.cell = world_.tableCell(x, y);
        out.push_back(c);
      }
    }
    return out;
  }

  std::vector<Grasp> grasps(const Cell & c) const
  {
    const Eigen::Vector3d ex = Eigen::Vector3d::UnitX(), ey = Eigen::Vector3d::UnitY();
    const Eigen::Vector3d down = -Eigen::Vector3d::UnitZ();
    if (c.inward != 0 && grasp_set_ == "extended") {
      // Front side grasps with the hand pitched down/up (fingers stay on the box's x faces),
      // and "inner" grasps: approach from the shelf centre towards the post, fingers on the
      // y faces, leaving towards the centre first.
      const Eigen::Vector3d in = c.inward * ey;
      const auto pitched = [&down](const Eigen::Vector3d & dir, double deg) {
          const double a = deg * M_PI / 180.0;
          return Eigen::Vector3d(std::cos(a) * dir + std::sin(a) * down);
        };
      std::vector<Grasp> out;
      for (const auto & [name, deg] : std::vector<std::pair<std::string, double>>{
          {"side", 0}, {"side_dn20", 20}, {"side_dn35", 35}, {"side_up20", -20}})
      {
        out.push_back({name, handFrame(pitched(in, deg), ex), -in});
      }
      if (c.dx != 0) {
        const Eigen::Vector3d toward_post = (c.dx > 0 ? 1.0 : -1.0) * ex;
        for (const auto & [name, deg] : std::vector<std::pair<std::string, double>>{
            {"inner", 0}, {"inner_dn20", 20}})
        {
          Grasp g{name, handFrame(pitched(toward_post, deg), ey), -in};
          g.pre = {Eigen::Vector3d(0, 0, 0.04), -0.06 * toward_post};
          out.push_back(g);
        }
      }
      return out;
    }
    if (c.inward != 0) {
      const Eigen::Vector3d in = c.inward * ey;
      return {{"side", handFrame(in, ex), -in}, {"top_x", handFrame(down, ex), -in},
        {"top_y", handFrame(down, ey), -in}};
    }
    // Table: leave upwards (out = zero); side grasps from either robot side.
    const Eigen::Vector3d none = Eigen::Vector3d::Zero();
    return {{"top_x", handFrame(down, ex), none}, {"top_y", handFrame(down, ey), none},
      {"side_+y", handFrame(ey, ex), none}, {"side_-y", handFrame(-ey, ex), none}};
  }

  Result evaluate(const fc::ArmNames & arm, const Cell & c, const Grasp & g)
  {
    // The fingers hold the box's side faces; pitching the hand turns it about the finger axis,
    // so the box itself stays level: attach it with its world orientation (identity).
    attachBox(arm, g.hand.inverse());
    const auto * group = model_->getJointModelGroup(arm.group);
    const auto * tcp = model_->getLinkModel(arm.tcp);
    const auto valid = [this](double margin) {
        return [this, margin](moveit::core::RobotState * s, const moveit::core::JointModelGroup * j,
                 const double * v) {
                 s->setJointGroupPositions(j, v);s->update();
                 return s->satisfiesBounds(j) && fc::ArmMotion::jointLimitMargin(*s, j) >= margin &&
                        (!check_collisions_ || !scene_.isStateColliding(*s));
               };
      };
    const auto plan_valid = valid(plan_margin_);
    const auto exec_valid = valid(exec_margin_);
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.linear() = g.hand.toRotationMatrix();
    target.translation() = c.cell.center + Eigen::Vector3d(0, 0, 0.0015);
    Result r;r.seeds = seeds_;
    std::vector<moveit::core::RobotState> solutions;
    for (int i = 0; i < seeds_; ++i) {
      auto state = scene_.getCurrentState();
      if (i > 0) {state.setToRandomPositions(group);}
      if (!state.setFromIK(group, target, arm.tcp, 0.05, plan_valid)) {continue;}
      ++r.ik_ok;
      r.best_margin = std::max(r.best_margin, fc::ArmMotion::jointLimitMargin(state, group));
      r.best_manip = std::max(r.best_manip, manipulability(state, group, tcp));
      if (static_cast<int>(solutions.size()) < exit_checks_) {solutions.push_back(state);}
    }
    if (r.ik_ok == 0 && report_contacts_) {reportContacts(arm, c, g, target, group);}
    // Exit: lift 8 cm, then retreat horizontally (shelf cells) as far as possible.
    for (auto & start : solutions) {
      for (double d : {0.12, 0.08, 0.06}) {
        if (d <= r.exit_d) {break;}
        if (c.inward == 0 && d < 0.12) {break;}  // table: lift only
        EigenSTL::vector_Isometry3d waypoints;
        Eigen::Isometry3d step = target;
        for (const auto & move : g.pre) {step.translation() += move;waypoints.push_back(step);}
        if (c.inward != 0) {
          step.translation() += d * g.out;
          waypoints.push_back(step);
        }
        auto state = start;
        std::vector<moveit::core::RobotStatePtr> path;
        const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
          &state, group, path, tcp, waypoints, true, moveit::core::MaxEEFStep(0.002),
          moveit::core::CartesianPrecision{}, exec_valid, kinematics::KinematicsQueryOptions());
        // 2 mm steps: a larger joint change means the IK switched configuration mid-path.
        if (fraction < 0.999 || path.empty() || fc::ArmMotion::maxJointJump(path, group) > 0.05) {
          continue;
        }
        double margin = std::numeric_limits<double>::infinity();
        for (const auto & p : path) {margin = std::min(margin, fc::ArmMotion::jointLimitMargin(*p, group));}
        r.exit_d = c.inward == 0 ? 0.08 : d;r.exit_margin = std::max(r.exit_margin, margin);
        break;
      }
    }
    return r;
  }

private:
  // Diagnosis for NO_IK: solve without collisions and list what the holding posture touches.
  void reportContacts(const fc::ArmNames & arm, const Cell & c, const Grasp & g,
    const Eigen::Isometry3d & target, const moveit::core::JointModelGroup * group)
  {
    std::map<std::string, int> pairs;int solved = 0;
    for (int i = 0; i < 10; ++i) {
      auto state = scene_.getCurrentState();
      if (i > 0) {state.setToRandomPositions(group);}
      if (!state.setFromIK(group, target, arm.tcp, 0.05)) {continue;}
      ++solved;state.update();
      collision_detection::CollisionRequest request;request.contacts = true;request.max_contacts = 10;
      collision_detection::CollisionResult result;
      scene_.checkCollision(request, result, state);
      for (const auto & contact : result.contacts) {
        ++pairs[contact.first.first + " <-> " + contact.first.second];
      }
    }
    std::printf("NO_IK %s %s %s: %d collision-free-less solutions, contacts:", arm.side.c_str(),
      c.id.c_str(), g.name.c_str(), solved);
    for (const auto & [pair, count] : pairs) {std::printf(" [%d] %s;", count, pair.c_str());}
    std::printf("\n");
  }

  void attachBox(const fc::ArmNames & arm, const Eigen::Quaterniond & box_in_tcp)
  {
    moveit_msgs::msg::AttachedCollisionObject msg;
    if (!attached_to_.empty()) {
      msg.object.id = "box";msg.object.operation = msg.object.REMOVE;
      msg.link_name = fc::ArmNames(attached_to_).tcp;
      scene_.processAttachedCollisionObjectMsg(msg);
    }
    msg = moveit_msgs::msg::AttachedCollisionObject();
    msg.link_name = arm.tcp;
    msg.touch_links = {arm.left_finger, arm.right_finger, arm.hand};
    msg.object.id = "box";msg.object.header.frame_id = arm.tcp;
    msg.object.operation = msg.object.ADD;
    shape_msgs::msg::SolidPrimitive box;box.type = box.BOX;
    const double size = world_.task().box_size;
    box.dimensions = {size, size, size};
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = box_in_tcp.w();pose.orientation.x = box_in_tcp.x();
    pose.orientation.y = box_in_tcp.y();pose.orientation.z = box_in_tcp.z();
    msg.object.primitives.push_back(box);msg.object.primitive_poses.push_back(pose);
    scene_.processAttachedCollisionObjectMsg(msg);
    attached_to_ = arm.side;
  }

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr model_;
  planning_scene::PlanningScene scene_;
  fc::WorldModel world_;
  int seeds_ = 24, exit_checks_ = 6;
  double plan_margin_ = 0.07, exec_margin_ = 0.05;
  std::vector<double> x_offsets_{-0.15, 0.0, 0.15}, insets_{0.03, 0.10};
  bool include_table_ = true;
  std::string grasp_set_ = "basic";
  bool check_collisions_ = true;  // debug: false separates reach limits from collisions
  bool report_contacts_ = false;  // debug: list the contacts that block NO_IK combinations
  std::string attached_to_;
};

const char * statusOf(const Result & r)
{
  if (r.ik_ok == 0) {return "NO_IK";}
  if (r.exit_d == 0) {return "NO_EXIT";}
  return "OK";
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("reachability_map",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  robot_model_loader::RobotModelLoader loader(node, "robot_description");
  const auto model = loader.getModel();
  if (!model) {RCLCPP_FATAL(node->get_logger(), "Robot model unavailable");return 1;}
  std::string output = "reachability.csv";
  node->get_parameter_or("output", output, output);
  Mapper mapper(node, model);
  const auto cells = mapper.cells();
  std::ofstream csv(output);
  csv << "arm,cell,surface,level,dx_or_x,inset_or_y,grasp,status,ik_ok,seeds,best_margin_rad,"
    "best_manipulability,exit_retreat_m,exit_margin_rad\n";
  // Summary: best status per (arm, cell) over grasps.
  std::map<std::string, std::map<std::string, std::string>> best;
  const auto started = std::chrono::steady_clock::now();
  for (const std::string side : {"left", "right"}) {
    const fc::ArmNames arm(side);
    bool own_shelf_only = false;
    node->get_parameter_or("own_shelf_only", own_shelf_only, own_shelf_only);
    for (const auto & c : cells) {
      // left arm base is at y < 0 (shelf_1, inward -1), right arm at y > 0 (shelf_2).
      if (own_shelf_only && c.inward != 0 && (side == "left") != (c.inward < 0)) {continue;}
      std::string summary = "--";
      double best_score = -1;
      for (const auto & g : mapper.grasps(c)) {
        const Result r = mapper.evaluate(arm, c, g);
        char line[256];
        std::snprintf(line, sizeof(line), "%s,%s,%s,%d,%.2f,%.2f,%s,%s,%d,%d,%.4f,%.4f,%.2f,%.4f\n",
          side.c_str(), c.id.c_str(), c.surface.c_str(), c.level, c.dx, c.inset, g.name.c_str(),
          statusOf(r), r.ik_ok, r.seeds, r.best_margin, r.best_manip, r.exit_d, r.exit_margin);
        csv << line;
        if (std::string(statusOf(r)) == "OK") {
          const double score = r.exit_d + r.exit_margin;  // longer exit, then larger margin
          if (score > best_score) {
            best_score = score;
            char text[32];std::snprintf(text, sizeof(text), "%s %.0f", g.name.c_str(), r.exit_d * 100);
            summary = text;
          }
        } else if (summary == "--" && r.ik_ok > 0) {
          summary = "noexit";
        }
      }
      best[side][c.id] = summary;
    }
    RCLCPP_INFO(node->get_logger(), "%s arm done (%.0f s)", side.c_str(),
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
  }
  csv.close();
  std::printf("\nBest grasp per cell (grasp + retreat cm; noexit = holdable but cannot leave; -- = unreachable)\n");
  for (const std::string side : {"left", "right"}) {
    std::printf("\n=== %s arm ===\n", side.c_str());
    for (const auto & c : cells) {
      if (best[side].count(c.id)) {std::printf("  %-28s %s\n", c.id.c_str(), best[side][c.id].c_str());}
    }
  }
  std::printf("\nCSV: %s\n", output.c_str());
  rclcpp::shutdown();
  return 0;
}
