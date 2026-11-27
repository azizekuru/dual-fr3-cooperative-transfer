// Offline geometric screening. Never sends controller or MoveGroup goals.
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{
void fingers(moveit::core::RobotState & state, const std::string & prefix, double opening)
{
  state.setVariablePosition(prefix + "_finger_joint1", opening);
  state.setVariablePosition(prefix + "_finger_joint2", opening);
  state.update();
}

moveit_msgs::msg::CollisionObject cubeObject(
  const std::string & id, double side, double x, double y, double z)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;object.header.frame_id = "world";
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive box;box.type = box.BOX;box.dimensions = {side, side, side};
  object.primitives.push_back(box);
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;pose.position.y = y;pose.position.z = z;pose.orientation.w = 1;
  object.primitive_poses.push_back(pose);
  return object;
}

bool completePath(
  moveit::core::RobotState start, const moveit::core::JointModelGroup * group,
  const std::string & tcp, const EigenSTL::vector_Isometry3d & waypoints,
  const moveit::core::GroupStateValidityCallbackFn & valid, double & fraction)
{
  std::vector<moveit::core::RobotStatePtr> path;
  fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
    &start, group, path, start.getLinkModel(tcp), waypoints, true,
    moveit::core::MaxEEFStep(0.002), moveit::core::CartesianPrecision{}, valid,
    kinematics::KinematicsQueryOptions());
  if (fraction < 0.999 || path.size() < 2) {return false;}
  for (size_t i = 1; i < path.size(); ++i) {
    for (const auto & name : group->getVariableNames()) {
      if (std::abs(path[i]->getVariablePosition(name) -
        path[i - 1]->getVariablePosition(name)) > 0.05) {return false;}
    }
  }
  return true;
}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("compartment_reach_check",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try {
    robot_model_loader::RobotModelLoader loader(node, "robot_description");
    auto model = loader.getModel();
    if (!model) {throw std::runtime_error("Robot model unavailable");}
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    // Old SRDF sampled 'Never' pairs do not establish clearance for these poses.
    for (const auto & obstacle : {"shelf_1_link", "shelf_2_link", "table_link"}) {
      for (const auto & link : model->getLinkModelNames()) {
        if (link.find("fr3v2") != std::string::npos) {
          scene->getAllowedCollisionMatrixNonConst().setEntry(obstacle, link, false);
        }
      }
    }
    auto & initial = scene->getCurrentStateNonConst();
    initial.setToDefaultValues();
    const std::vector<double> ready = {0, -0.785, 0, -2.356, 0, 1.571, 0.785};
    initial.setJointGroupPositions("arm_1", ready);
    initial.setJointGroupPositions("arm_2", ready);
    fingers(initial, "left_fr3v2", 0.04);fingers(initial, "right_fr3v2", 0.04);
    const auto targets = node->get_parameter("targets").as_double_array();
    if (targets.size() != 48) {throw std::runtime_error("Expected sixteen xyz targets");}
    const double side = node->get_parameter("cube_side").as_double();
    const double lift = node->get_parameter("lift").as_double();
    const double withdraw = node->get_parameter("withdraw").as_double();
    const bool occupied = node->get_parameter("occupied_cells").as_bool();
    const auto occupied_shelves = node->get_parameter("initially_occupied_shelves").as_integer_array();
    const auto populated = [&occupied_shelves, occupied](size_t cell) {
        const int shelf = cell < 8 ? 1 : 2;
        return occupied && std::find(occupied_shelves.begin(), occupied_shelves.end(), shelf) !=
               occupied_shelves.end();
      };
    for (size_t cell = 0; occupied && cell < 16; ++cell) {
      if (!populated(cell)) {continue;}
      if (!scene->processCollisionObjectMsg(cubeObject("cell_cube_" + std::to_string(cell),
          side, targets[cell * 3], targets[cell * 3 + 1], targets[cell * 3 + 2])))
      {
        throw std::runtime_error("Cannot populate neighbouring cells");
      }
    }
    RCLCPP_INFO(node->get_logger(), "SCREENING SETUP: occupied_cells=%d cube_side=%.3f m",
      occupied, side);
    int passed = 0;
    for (size_t cell = 0; cell < 16; ++cell) {
      const bool left = cell < 8;
      const std::string arm = left ? "arm_1" : "arm_2";
      const std::string prefix = left ? "left_fr3v2" : "right_fr3v2";
      const std::string tcp = prefix + "_hand_tcp";
      const std::string object_id = "cell_cube_" + std::to_string(cell);
      const auto * group = model->getJointModelGroup(arm);
      const Eigen::Quaterniond nominal = left ? Eigen::Quaterniond(0.5, 0.5, 0.5, -0.5) :
        Eigen::Quaterniond(0.5, -0.5, -0.5, -0.5);
      Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
      target.translation() = Eigen::Vector3d(targets[cell * 3], targets[cell * 3 + 1],
        targets[cell * 3 + 2]);
      auto entry_scene = scene->diff();
      if (!populated(cell) && !entry_scene->processCollisionObjectMsg(cubeObject(object_id, side,
          target.translation().x(), target.translation().y(), target.translation().z())))
      {
        throw std::runtime_error("Cannot add target cube");
      }
      // Target remains a world object during entry: no target/finger exceptions.
      auto closure_scene = entry_scene->diff();
      for (const auto & finger : {prefix + "_leftfinger", prefix + "_rightfinger"}) {
        closure_scene->getAllowedCollisionMatrixNonConst().setEntry(object_id, finger, true);
      }
      auto local = entry_scene;
      std::string phase = "ik", blocked_by = "none";
      const auto valid = [&local, &blocked_by](moveit::core::RobotState * state,
        const moveit::core::JointModelGroup * joints, const double * values) {
          state->setJointGroupPositions(joints, values);state->update();
          for (const auto & name : joints->getVariableNames()) {
            const auto & bound = state->getRobotModel()->getVariableBounds(name);
            const double value = state->getVariablePosition(name);
            if (value < bound.min_position_ + 0.05 || value > bound.max_position_ - 0.05) {
              blocked_by = "joint_margin:" + name;return false;
            }
          }
          // Include gripper joints even though IK only solves the seven arm joints.
          if (!local->isStateColliding(*state)) {return true;}
          collision_detection::CollisionRequest request;
          request.contacts = true;request.max_contacts = 1;
          collision_detection::CollisionResult result;
          local->checkCollision(request, result, *state);
          if (!result.contacts.empty()) {
            const auto & pair = result.contacts.begin()->first;
            blocked_by = pair.first + ":" + pair.second;
          }
          return false;
        };
      bool ik = false, entered = false, closed = false, escaped = false;
      int attempts = 0, selected_roll = -1;
      double best_entry = 0, best_exit = 0;
      for (; attempts < 32 && !escaped; ++attempts) {
        local = entry_scene;phase = "ik";
        const int roll = attempts % 2;
        const Eigen::Quaterniond q = nominal * Eigen::Quaterniond(
          Eigen::AngleAxisd(roll * std::acos(-1.0), Eigen::Vector3d::UnitZ()));
        target.linear() = q.toRotationMatrix();
        auto grasp = entry_scene->getCurrentState();
        if (attempts >= 2) {grasp.setToRandomPositions(group);}
        if (!grasp.setFromIK(group, target, tcp, 0.08, valid)) {continue;}
        ik = true;phase = "open_entry";
        auto outside = target;outside.translation().y() += left ? withdraw : -withdraw;
        double fraction = 0;
        // Reverse from the open grasp state to preserve one IK branch. Reversing
        // this collision-checked path gives the geometrically equivalent entry.
        const bool entry_ok = completePath(grasp, group, tcp, {outside}, valid, fraction);
        best_entry = std::max(best_entry, fraction);
        if (!entry_ok) {continue;}
        entered = true;phase = "closure";local = closure_scene;
        auto held = grasp;
        bool closure_ok = true;
        for (int step = 0; step <= 10; ++step) {
          fingers(held, prefix, 0.04 - step * 0.002);
          std::vector<double> joints;held.copyJointGroupPositions(group, joints);
          if (!valid(&held, group, joints.data())) {closure_ok = false;break;}
        }
        if (!closure_ok) {continue;}
        closed = true;phase = "held_exit";
        local = closure_scene->diff();
        moveit_msgs::msg::AttachedCollisionObject attached;
        attached.link_name = tcp;
        attached.touch_links = {prefix + "_leftfinger", prefix + "_rightfinger"};
        attached.object.id = object_id;attached.object.header.frame_id = tcp;
        attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        shape_msgs::msg::SolidPrimitive primitive;primitive.type = primitive.BOX;
        primitive.dimensions = {side, side, side};attached.object.primitives.push_back(primitive);
        geometry_msgs::msg::Pose relative;
        relative.orientation.w = q.w();relative.orientation.x = -q.x();
        relative.orientation.y = -q.y();relative.orientation.z = -q.z();
        attached.object.primitive_poses.push_back(relative);
        // Install at the solved grasp state, not the ready state; attach removes
        // the target world object while the other fifteen cubes remain obstacles.
        local->setCurrentState(held);
        if (!local->processAttachedCollisionObjectMsg(attached)) {
          throw std::runtime_error("Cannot attach selected cube");
        }
        if (local->getWorld()->hasObject(object_id)) {
          throw std::runtime_error("Selected cube remained in world after attachment");
        }
        for (size_t neighbour = 0; occupied && neighbour < 16; ++neighbour) {
          if (neighbour != cell && populated(neighbour) &&
            !local->getWorld()->hasObject("cell_cube_" + std::to_string(neighbour)))
          {
            throw std::runtime_error("Neighbour disappeared during attachment");
          }
        }
        held = local->getCurrentState();
        std::vector<double> joints;held.copyJointGroupPositions(group, joints);
        if (!valid(&held, group, joints.data())) {continue;}
        auto raised = target;raised.translation().z() += lift;
        auto exit = raised;exit.translation().y() += left ? withdraw : -withdraw;
        escaped = completePath(held, group, tcp, {raised, exit}, valid, fraction);
        best_exit = std::max(best_exit, fraction);
        if (escaped) {selected_roll = roll;}
      }
      passed += escaped;
      RCLCPP_INFO(node->get_logger(),
        "CELL PROBE: shelf=%d row=%zu col=%zu ik=%d open_entry=%d closure=%d "
        "local_lift_exit=%d attempts=%d roll_pi=%d entry_fraction=%.3f exit_fraction=%.3f "
        "last_phase=%s blocked_by=%s", left ? 1 : 2, (cell % 8) / 2 + 1,
        cell % 2 + 1, ik, entered, closed, escaped, attempts, selected_roll, best_entry, best_exit,
        escaped ? "passed" : phase.c_str(), escaped ? "none" : blocked_by.c_str());
    }
    RCLCPP_INFO(node->get_logger(),
      "COMPARTMENT SCREENING: %d/16 open-entry/closure/held-exit paths; "
      "occupied_cells=%d; NOT full pick/place validation", passed, occupied);
    rclcpp::shutdown();return passed == 16 ? 0 : 2;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();return 1;
  }
}
