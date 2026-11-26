// Right arm: parks the left arm, picks the box from the table hand-off cell and places it in
// the target shelf cell. Attachment affects planning only; every step is verified physically.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fr3_cooperative_control/right_transfer.hpp"
#include "fr3_cooperative_control/arm_motion.hpp"
#include "fr3_cooperative_control/box_tracker.hpp"
#include "fr3_cooperative_control/box_weld.hpp"
#include "fr3_cooperative_control/common.hpp"
#include "fr3_cooperative_control/contact_monitor.hpp"
#include "fr3_cooperative_control/controllers.hpp"
#include "fr3_cooperative_control/gripper.hpp"
#include "fr3_cooperative_control/world_model.hpp"

using namespace std::chrono_literals;
namespace fc = fr3_cooperative_control;
using fc::LinearMode;
using fc::Pose;
using fc::Wall;
using fc::point;
using fc::require;
using fc::rotation;
using Group = fc::Group;

namespace
{
double jointLimitMargin(
  const moveit::core::RobotState & state, const moveit::core::JointModelGroup * group)
{
  return fc::ArmMotion::jointLimitMargin(state, group);
}
fc::ContactPolicy rightPolicy(const fc::TaskConfig & task)
{
  fc::ContactPolicy policy;
  // The target board is the only surface the carried box may rest on while seating.
  policy.support_collision =
    task.target_shelf + "_level_" + std::to_string(task.target_level) + "_collision";
  policy.seat_allows_support_only = true;
  policy.log_each_unexpected = true;
  return policy;
}
fc::MotionProfile rightProfile()
{
  fc::MotionProfile profile;
  profile.fine_scale_threshold = 0.02;
  profile.coarse_position_tolerance = 0.0005;
  profile.coarse_angle_tolerance = 0.005;
  profile.fine_angle_tolerance = 0.0015;
  return profile;
}

class RightTablePick
{
public:
  explicit RightTablePick(rclcpp::Node::SharedPtr node)
  : node_(node), names_("right"), task_(fc::TaskConfig::fromParameters(*node)),
    box_(node, task_.box_size, task_.gazebo_z_offset), gripper_(node, names_),
    contacts_(node, names_, rightPolicy(task_)),
    motion_(node, names_, rightProfile(), [this] {safe();}, nullptr)
  {
    fc::setSpeedScale(task_.speed_scale);
    if (task_.rigid_grasp) {
      left_weld_ = std::make_unique<fc::BoxWeld>(node_, "left");
      right_weld_ = std::make_unique<fc::BoxWeld>(node_, "right");
    }
    std::string shelf_topic = "/placement/shelf2_contacts";
    node_->get_parameter_or<std::string>("target_contacts_topic", shelf_topic, shelf_topic);
    contacts_.addSurface("TABLE", "/placement/table_contacts", "", false);
    contacts_.addSurface("SHELF", shelf_topic, rightPolicy(task_).support_collision, true,
      [this] {return monitor_shelf_.load();});
  }

  void run(const std::string & stage, bool left_placed)
  {
    require(stage == "pick" || stage == "shelf_hold" || stage == "shelf_place" ||
      stage == "left_park",
        "Invalid right-arm stage");
    // Rigid-grasp test mode: without the left stage nobody has released the spawn welds.
    if (right_weld_ && !left_placed) {left_weld_->release(true);right_weld_->release(true);}
    monitor_shelf_ = stage == "shelf_place";
    const bool ready = fc::waitForControllers(node_,
      {"arm_1_controller", "arm_2_controller", "right_gripper_controller"}, 90s);
    require(ready && gripper_.waitForServer(20s), "Right arm/gripper not ready");
    motion_.connect();
    world_ = std::make_unique<fc::WorldModel>(arm().getRobotModel(), task_);
    cell_ = world_->targetCell();
    inward_ = world_->targetShelf().inward();
    Group left(node_, "arm_1");
    arm().setEndEffectorLink(tcp());arm().setPoseReferenceFrame("world");
    arm().setPlanningTime(10);arm().setNumPlanningAttempts(3);
    arm().setGoalPositionTolerance(0.0005);arm().setGoalOrientationTolerance(0.001);
    arm().setMaxVelocityScalingFactor(fc::speed(0.40));arm().setMaxAccelerationScalingFactor(fc::speed(0.40));
    require(arm().startStateMonitor(15) && left.startStateMonitor(15), "Robot state unavailable");
    require(motion_.scene().getAttachedObjects({"target_box"}).empty(),
        "Box still attached; restart simulation");
    const fc::Cell handoff = world_->handoffCell();
    const auto table_box = box_.stable();
    require((point(table_box) - handoff.center).norm() < 0.006,
      "Box not resting at the table pickup location");
    waitTableSupport();addBox(table_box);configureContacts(false);
    RCLCPP_INFO(node_->get_logger(), "LEFT PARK START: box collisions enabled for left fingers");
    if (left_placed) {
      parkLeftAfterPlacement(left);
    } else {
      park(left);
    }
    waitTableSupport();
    require((point(box_.fresh()) - point(table_box)).norm() < 0.001,
      "Box moved while parking the left arm");
    RCLCPP_INFO(node_->get_logger(), "LEFT PARK PASSED");
    if (stage == "left_park") {return;}
    gripper_.command(0.04, false);
    park(arm());
    const auto initial = box_.stable();
    require((point(initial) - point(table_box)).norm() < 0.001, "Box moved while parking arms");
    safe();addBox(initial);
    Pose pick = initial;const double half = std::sqrt(0.5);
    pick.orientation.x = half;pick.orientation.y = half;
    pick.orientation.z = 0;pick.orientation.w = 0;
    Pose pre = pick;pre.position.z += 0.12;
    const auto approach = planPickupApproach(pre, pick);
    require(arm().execute(approach) == moveit::core::MoveItErrorCode::SUCCESS,
      "Right pickup approach execution failed");
    motion_.waitAt(&pre, 0.001, 0.01, true);
    configureContacts(true);
    motion_.linear(pick, 0.1, LinearMode::pickup);motion_.waitAt(&pick, 0.0005, 0.005);
    safe();waitTableSupport();
    RCLCPP_INFO(node_->get_logger(), "RIGHT PICK TARGET SETTLED: closing right gripper");
    gripper_.command(0.018, true);
    if (right_weld_) {right_weld_->attach();}  // test mode: rigid for the transfer motions
    const auto grasped = box_.stable();const auto tcp_pose = arm().getCurrentPose(tcp()).pose;
    const Eigen::Vector3d reference =
      rotation(tcp_pose).inverse() * (point(grasped) - point(tcp_pose));
    const Eigen::Quaterniond reference_angle = rotation(tcp_pose).inverse() * rotation(grasped);
    addBox(grasped);
    motion_.attachBox();
    Pose lifted = pick;lifted.position.z += 0.08;
    motion_.linear(lifted, 0.1, LinearMode::pickup);motion_.waitAt(&lifted, 0.001, 0.01);safe();
    const auto lifted_box = box_.stable();
    const double rise = lifted_box.position.z - initial.position.z;
    RCLCPP_INFO(node_->get_logger(), "RIGHT PHYSICAL LIFT: %.3f mm", rise * 1000);
    require(std::abs(rise - 0.08) < 0.005 &&
      (point(lifted_box).head<2>() - point(initial).head<2>()).norm() < 0.005,
      "Physical box did not follow right-arm lift");
    const auto sim_start = node_->now();const auto wall_start = Wall::now();
    double drift = 0, angle = 0;
    while ((node_->now() - sim_start).seconds() < 3) {
      require(Wall::now() - wall_start < 30s, "Right hold clock timed out");safe();
      const auto box = box_.fresh();const auto hand = arm().getCurrentPose(tcp()).pose;
      drift = std::max(drift,
          (rotation(hand).inverse() * (point(box) - point(hand)) - reference).norm());
      angle = std::max(angle,
          reference_angle.angularDistance(rotation(hand).inverse() * rotation(box)));
      require(drift < 0.005 && angle < 0.0873, "Right grasp slipped or rotated");
      require(welded() || contacts_.fingersFresh(1s),
        "Missing current physical right-finger contact");
      std::this_thread::sleep_for(100ms);
    }
    require(!onTable(), "Box still has fresh table contact after right lift");
    RCLCPP_INFO(node_->get_logger(),
      "RIGHT PICK TEST PASSED: lift=%.3f mm drift=%.3f mm angle=%.3f deg; right gripper stays closed",
      rise * 1000, drift * 1000, angle * 180 / M_PI);
    if (stage == "pick") {return;}
    contacts_.setCarrying(true);
    configureContacts(true, false);
    if (stage == "shelf_place") {
      raiseForShelfTransfer();
      holdAt("RIGHT CLEARANCE", reference, reference_angle, drift, angle, 1.0);
      placeOnShelf(reference, reference_angle);
      return;
    }
    Pose high = arm().getCurrentPose(tcp()).pose;
    high.position.z += 0.10;
    RCLCPP_INFO(node_->get_logger(), "RIGHT CLEARANCE START: 100 mm additional vertical lift");
    motion_.linear(high);motion_.waitAt(&high, 0.001, 0.01);
    holdAt("RIGHT CLEARANCE", reference, reference_angle, drift, angle);
    Pose shelf = high;
    shelf.orientation.x = -0.5;shelf.orientation.y = -0.5;
    shelf.orientation.z = -0.5;shelf.orientation.w = 0.5;
    const Eigen::Vector3d desired_center = frontCenter();
    const Eigen::Vector3d target = desired_center - rotation(shelf) * reference;
    shelf.position.x = target.x();shelf.position.y = target.y();shelf.position.z = target.z();
    arm().setMaxVelocityScalingFactor(fc::speed(0.10));arm().setMaxAccelerationScalingFactor(fc::speed(0.10));
    RCLCPP_INFO(node_->get_logger(),
        "RIGHT SHELF TRANSFER START: box target=(%.3f, %.3f, %.4f)", desired_center.x(),
        desired_center.y(), desired_center.z());
    arm().setPoseTarget(shelf);executePlan();arm().clearPoseTargets();
    motion_.waitAt(&shelf, 0.0015, 0.005);
    holdAt("RIGHT SHELF FRONT", reference, reference_angle, drift, angle);
    const auto shelf_box = box_.fresh();
    const double center_error = (point(shelf_box) - desired_center).norm();
    const double face_tilt = std::acos(std::clamp(
      rotation(shelf_box).toRotationMatrix().row(2).cwiseAbs().maxCoeff(), 0.0, 1.0));
    require(center_error < 0.01 && face_tilt < 0.0873, "Box not held level at shelf-front target");
    RCLCPP_INFO(node_->get_logger(),
      "RIGHT SHELF HOLD PASSED: center_error=%.3f mm drift=%.3f mm angle=%.3f deg tilt=%.3f deg; no release",
      center_error * 1000, drift * 1000, angle * 180 / M_PI, face_tilt * 180 / M_PI);
  }

private:
  Group & arm() {return motion_.group();}
  // Test mode: while welded Gazebo reports no finger contacts, so finger checks are skipped.
  bool welded() const {return right_weld_ && right_weld_->attached();}
  const std::string & tcp() const {return names_.tcp;}
  void addBox(const Pose & box) {motion_.addBox(box, task_.box_size);}
  void syncHeldBox(const Pose & box, const Pose & tcp_pose)
  {
    motion_.syncHeldBox(box, tcp_pose, task_.box_size);
  }
  void executePlan() {motion_.executePlan(fc::PlanOptions{});}

  // Target-cell geometry. `inward_` points from the robots into the target shelf.
  // In front of the shelf at the approach height (12 cm out, 8 cm above the resting centre).
  Eigen::Vector3d frontCenter() const
  {
    return cell_.center + Eigen::Vector3d(0, -inward_ * 0.12, 0.08);
  }
  // Inside the shelf at the approach height, above the resting centre.
  Eigen::Vector3d insertCenter() const {return cell_.center + Eigen::Vector3d(0, 0, 0.08);}
  // y limit along the shelf direction, `back` metres in front of the cell centre.
  double limitBefore(double back) const {return cell_.center.y() - inward_ * back;}

  Eigen::Quaterniond nearestCubeRotation(const Eigen::Quaterniond & current)
  {
    std::array<int, 3> order = {0, 1, 2};double best_error = 10;
    Eigen::Quaterniond best = current;
    do {
      for (int signs = 0; signs < 8; ++signs) {
        Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
        for (int column = 0; column < 3; ++column) {
          matrix(order[column], column) = signs & (1 << column) ? -1.0 : 1.0;
        }
        if (matrix.col(0).cross(matrix.col(1)).dot(matrix.col(2)) < 0.5) {continue;}
        Eigen::Quaterniond q(matrix);const double error = q.angularDistance(current);
        if (error < best_error) {best_error = error;best = q;}
      }
    } while (std::next_permutation(order.begin(), order.end()));
    return best;
  }
  struct ShelfTransfer
  {
    Group::Plan turn;
    moveit_msgs::msg::RobotTrajectory advance;
    Pose midpoint;
  };

  void raiseForShelfTransfer()
  {
    safe();
    planning_scene::PlanningScene scene(arm().getRobotModel());
    require(scene.setPlanningSceneMsg(motion_.sceneSnapshot()),
        "Clearance lift scene import failed");
    auto start = scene.getCurrentState();
    start.update();
    const auto * group = start.getJointModelGroup(names_.group);
    const auto & tcp_pose = start.getGlobalLinkTransform(tcp());
    const Eigen::Quaterniond orientation(tcp_pose.rotation());
    Pose target;
    target.position.x = tcp_pose.translation().x();
    target.position.y = tcp_pose.translation().y();
    target.orientation.w = orientation.w();
    target.orientation.x = orientation.x();
    target.orientation.y = orientation.y();
    target.orientation.z = orientation.z();
    for (const double rise : {0.10, 0.08, 0.06, 0.04}) {
      target.position.z = tcp_pose.translation().z() + rise;
      std::vector<moveit::core::RobotStatePtr> path;
      if (!motion_.checkedFinePath(scene, start, target, path)) {
        RCLCPP_INFO(node_->get_logger(), "RIGHT CLEARANCE CANDIDATE REJECTED: %.0f mm, path",
          rise * 1000);
        continue;
      }
      double margin = std::numeric_limits<double>::infinity();
      robot_trajectory::RobotTrajectory trajectory(arm().getRobotModel(), group);
      for (const auto & waypoint : path) {
        margin = std::min(margin, jointLimitMargin(*waypoint, group));
        trajectory.addSuffixWayPoint(*waypoint, 0.0);
      }
      if (margin < 0.05) {
        RCLCPP_INFO(node_->get_logger(),
          "RIGHT CLEARANCE CANDIDATE REJECTED: %.0f mm, joint margin=%.4f rad",
          rise * 1000, margin);
        continue;
      }
      trajectory_processing::TimeOptimalTrajectoryGeneration timing(0.00001, 0.005, 0.000001);
      if (!timing.computeTimeStamps(trajectory, fc::speed(0.10), fc::speed(0.10))) {continue;}
      bool valid = true;
      for (size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
        const auto & waypoint = trajectory.getWayPoint(i);
        margin = std::min(margin, jointLimitMargin(waypoint, group));
        if (margin < 0.05 || scene.isStateColliding(waypoint, names_.group)) {
          valid = false;
          break;
        }
      }
      const auto & end = trajectory.getLastWayPoint().getGlobalLinkTransform(tcp());
      if (!valid || (end.translation() - point(target)).norm() >= 0.0005 ||
        Eigen::Quaterniond(end.rotation()).angularDistance(orientation) >= 0.005)
      {
        continue;
      }
      moveit_msgs::msg::RobotTrajectory motion;
      trajectory.getRobotTrajectoryMsg(motion);
      RCLCPP_INFO(node_->get_logger(), "RIGHT CLEARANCE START: %.0f mm, joint margin=%.4f rad",
        rise * 1000, margin);
      safe();
      require(arm().execute(motion) == moveit::core::MoveItErrorCode::SUCCESS,
        "Clearance lift execution failed");
      motion_.waitAt(&target, 0.001, 0.01);
      auto actual = arm().getCurrentState(2.0);
      require(actual && jointLimitMargin(*actual, group) >= 0.05,
        "Measured clearance posture too close to joint limit");
      return;
    }
    throw std::runtime_error("No additional lift with enough joint margin");
  }

  bool checkShelfTransfer(
    const moveit_msgs::msg::RobotTrajectory & msg, const planning_scene::PlanningScene & scene,
    const Eigen::Vector3d & relative, const Eigen::Quaterniond & relative_angle,
    double y_limit, const char * phase)
  {
    const auto & trajectory = msg.joint_trajectory;
    if (trajectory.points.empty()) {return false;}
    auto state = scene.getCurrentState();
    const auto * group = state.getJointModelGroup(names_.group);
    double min_margin = std::numeric_limits<double>::infinity();
    double max_reach = -std::numeric_limits<double>::infinity();  // along inward_
    double min_bottom = std::numeric_limits<double>::infinity();
    for (const auto & waypoint : trajectory.points) {
      state.setVariablePositions(trajectory.joint_names, waypoint.positions);
      state.update();
      min_margin = std::min(min_margin, jointLimitMargin(state, group));
      const auto & tcp_pose = state.getGlobalLinkTransform(tcp());
      const Eigen::Vector3d center = tcp_pose * relative;
      const Eigen::Matrix3d rotation = tcp_pose.linear() * relative_angle.toRotationMatrix();
      max_reach = std::max(max_reach, inward_ * center.y());
      min_bottom = std::min(min_bottom,
        center.z() - box_.half() * rotation.row(2).cwiseAbs().sum());
      if (!state.satisfiesBounds(group) || scene.isStateColliding(state, names_.group)) {
        return false;
      }
    }
    const double floor = cell_.surface + 0.0175;  // carried box stays above the target board
    const bool accepted = min_margin >= 0.05 && max_reach <= inward_ * y_limit &&
      min_bottom >= floor;
    RCLCPP_INFO(node_->get_logger(),
      "SHELF TRANSFER %s: phase=%s joint margin=%.4f rad max_box_y=%.4f m min_box_bottom=%.4f m",
      accepted ? "PLAN" : "REJECTED", phase, min_margin, inward_ * max_reach, min_bottom);
    return accepted;
  }

  Eigen::Quaterniond chooseShelfApproach(
    const Eigen::Vector3d & relative,
    const Eigen::Quaterniond & relative_angle, ShelfTransfer & transfer)
  {
    auto scene = std::make_shared<planning_scene::PlanningScene>(arm().getRobotModel());
    require(scene->setPlanningSceneMsg(motion_.sceneSnapshot()), "Shelf probe scene import failed");
    auto seating_scene = scene->diff();
    seating_scene->getAllowedCollisionMatrixNonConst().setEntry(
      "target_box", world_->targetShelf().link(), true);
    const auto * group = scene->getRobotModel()->getJointModelGroup(names_.group);
    const auto valid = [&seating_scene, this](moveit::core::RobotState * state,
      const moveit::core::JointModelGroup * joints, const double * values) {
        state->setJointGroupPositions(joints, values);
        state->update();
        return jointLimitMargin(*state, joints) >= 0.05 &&
               !seating_scene->isStateColliding(*state, names_.group);
      };
    const double half = std::sqrt(0.5);
    const std::array<Eigen::Quaterniond, 3> nominal = {
      Eigen::Quaterniond(0.5, -0.5, -0.5, -0.5),
      Eigen::Quaterniond(0.5, -0.5, 0.5, 0.5), Eigen::Quaterniond(0, half, half, 0)};
    // Midpoints in front of the shelf, relative to the target cell (dx, back, up).
    const std::array<Eigen::Vector3d, 6> midpoint_offsets = {
      Eigen::Vector3d(0.0, 0.32, 0.1075), Eigen::Vector3d(0.10, 0.32, 0.1075),
      Eigen::Vector3d(-0.10, 0.32, 0.1075), Eigen::Vector3d(0.0, 0.40, 0.1575),
      Eigen::Vector3d(0.10, 0.40, 0.1575), Eigen::Vector3d(-0.10, 0.40, 0.1575)};
    // Planning only: the held box has not moved yet. If the first search finds no
    // accepted approach, search once more with fresh random IK seeds.
    for (int pass = 0; pass < 2; ++pass) {
      if (pass > 0) {
        RCLCPP_WARN(node_->get_logger(),
          "SHELF APPROACH SEARCH RETRY: no accepted plan in pass 1, "
          "searching again (planning only)");
      }
      const auto deadline = Wall::now() + 35s;
      // Visit every hand orientation before spending another seed on the first one.
      // All variants share the existing per-pass time budget and safety checks.
      for (int attempt = 0; attempt < 16 && Wall::now() < deadline; ++attempt) {
        for (size_t variant = 0; variant < nominal.size() && Wall::now() < deadline; ++variant) {
          RCLCPP_INFO(node_->get_logger(),
            "SHELF SEARCH CANDIDATE: pass=%d seed=%d variant=%zu",
            pass + 1, attempt + 1, variant);
          const Eigen::Quaterniond q = nearestCubeRotation(nominal[variant] * relative_angle) *
            relative_angle.inverse();
          Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
          end.linear() = q.toRotationMatrix();
          end.translation() = cell_.center + Eigen::Vector3d(0, 0, 0.0002) - q * relative;
          Eigen::Isometry3d above = end;
          above.translation().z() += 0.0798;
          Eigen::Isometry3d front = above;
          front.translation().y() -= inward_ * 0.12;
          auto state = scene->getCurrentState();
          if (attempt > 0 || pass > 0) {state.setToRandomPositions(group);}
          if (!state.setFromIK(group, end, tcp(), 0.08, valid)) {continue;}
          std::vector<moveit::core::RobotStatePtr> reverse;
          const EigenSTL::vector_Isometry3d waypoints = {above, front};
          const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
            &state, group, reverse, state.getLinkModel(tcp()), waypoints, true,
            moveit::core::MaxEEFStep(0.002), moveit::core::CartesianPrecision{}, valid,
            kinematics::KinematicsQueryOptions());
          if (fraction < 0.999 || reverse.empty()) {continue;}
          for (const auto & offset : midpoint_offsets) {
            if (Wall::now() >= deadline) {break;}
            const Eigen::Vector3d center = cell_.center +
              Eigen::Vector3d(offset.x(), -inward_ * offset.y(), offset.z());
            auto midpoint = front;
            midpoint.translation() = center - q * relative;
            if (planShelfTransfer(*scene, *reverse.back(), midpoint, relative, relative_angle,
                transfer))
            {
              // Same in-shelf posture for every midpoint: if it fails, move on to the next candidate.
              if (!probeShelfInterior(*seating_scene, *reverse.back(), q, relative)) {break;}
              RCLCPP_INFO(node_->get_logger(),
                "SHELF TWO-STAGE PLAN PASSED: pass=%d variant=%zu midpoint_box=(%.3f, %.3f, %.3f)",
                pass + 1, variant, center.x(), center.y(), center.z());
              return q;
            }
          }
        }
      }
    }
    throw std::runtime_error("No collision-checked two-stage shelf placement approach found");
  }

  // Rehearse the in-shelf motions the way placeOnShelf executes them (0.1 mm fine paths,
  // every waypoint >= kPlanJointMargin from the joint limits): insert, descent, seat and the
  // open-gripper retreat (up, then out). A posture that would slide onto a joint limit inside
  // the shelf is rejected before the arm moves.
  bool probeShelfInterior(
    const planning_scene::PlanningScene & scene, const moveit::core::RobotState & front,
    const Eigen::Quaterniond & q, const Eigen::Vector3d & relative)
  {
    const auto tcp_for = [&q, &relative](const Eigen::Vector3d & box_center) {
        const Eigen::Vector3d t = box_center - q * relative;
        Pose p;p.position.x = t.x();p.position.y = t.y();p.position.z = t.z();
        p.orientation.w = q.w();p.orientation.x = q.x();p.orientation.y = q.y();
        p.orientation.z = q.z();
        return p;
      };
    const Eigen::Vector3d rest = cell_.center;
    const std::array<std::pair<const char *, Eigen::Vector3d>, 5> stages = {{
      {"insert", insertCenter()},
      {"descent", rest + Eigen::Vector3d(0, 0, 0.002)},
      {"seat", rest},
      {"retreat-up", rest + Eigen::Vector3d(0, 0, 0.08)},
      {"retreat-out", rest + Eigen::Vector3d(0, -inward_ * 0.12, 0.08)}}};
    const auto * group = front.getJointModelGroup(names_.group);
    auto state = front;
    double margin = jointLimitMargin(state, group);
    for (const auto & [stage, center] : stages) {
      std::vector<moveit::core::RobotStatePtr> path;
      if (!motion_.checkedFinePath(scene, state, tcp_for(center), path, kPlanJointMargin)) {
        RCLCPP_INFO(node_->get_logger(), "SHELF INTERIOR PROBE REJECTED: stage=%s", stage);
        return false;
      }
      for (const auto & waypoint : path) {margin = std::min(margin, jointLimitMargin(*waypoint, group));}
      state = *path.back();
    }
    RCLCPP_INFO(node_->get_logger(), "SHELF INTERIOR PROBE PASSED: min joint margin=%.4f rad",
      margin);
    return true;
  }

  bool planShelfTransfer(
    const planning_scene::PlanningScene & scene, moveit::core::RobotState front,
    const Eigen::Isometry3d & midpoint, const Eigen::Vector3d & relative,
    const Eigen::Quaterniond & relative_angle, ShelfTransfer & transfer)
  {
    const auto * group = front.getJointModelGroup(names_.group);
    std::vector<moveit::core::RobotStatePtr> path;
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &front, group, path, front.getLinkModel(tcp()), midpoint, true,
      moveit::core::MaxEEFStep(0.002), moveit::core::CartesianPrecision{},
      motion_.collisionCheck(scene), kinematics::KinematicsQueryOptions());
    if (fraction < 0.999 || path.size() < 2) {return false;}
    robot_trajectory::RobotTrajectory advance(arm().getRobotModel(), group);
    const auto & names = group->getVariableNames();
    for (size_t i = path.size(); i-- > 0; ) {
      if (i + 1 < path.size() && std::any_of(names.begin(), names.end(),
        [&path, i](const auto & name) {
          return std::abs(path[i]->getVariablePosition(name) -
            path[i + 1]->getVariablePosition(name)) > 0.05;
        }))
      {
        return false;
      }
      advance.addSuffixWayPoint(*path[i], 0.0);
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration timing(0.00001, 0.01, 0.000001);
    if (!timing.computeTimeStamps(advance, fc::speed(0.06), fc::speed(0.06))) {return false;}
    advance.getRobotTrajectoryMsg(transfer.advance);
    if (!checkShelfTransfer(transfer.advance, scene, relative, relative_angle,
        limitBefore(0.09), "advance"))
    {
      return false;
    }
    const auto & start = scene.getCurrentState();
    const auto & finish = *path.back();
    std::vector<double> joints;
    finish.copyJointGroupPositions(group, joints);
    if (!arm().setJointValueTarget(joints)) {return false;}

    bool planned = planDirectShelfTurn(scene, finish, relative, relative_angle, transfer.turn);
    for (int route = 0; !planned && route < 3; ++route) {
      arm().setStartState(start);
      if (arm().plan(transfer.turn) != moveit::core::MoveItErrorCode::SUCCESS) {continue;}
      planned = checkShelfTransfer(transfer.turn.trajectory, scene, relative, relative_angle,
          limitBefore(0.24), "turn");
    }
    arm().setStartStateToCurrentState();
    if (!planned) {return false;}

    transfer.midpoint.position.x = midpoint.translation().x();
    transfer.midpoint.position.y = midpoint.translation().y();
    transfer.midpoint.position.z = midpoint.translation().z();
    const Eigen::Quaterniond angle(midpoint.rotation());
    transfer.midpoint.orientation.w = angle.w();
    transfer.midpoint.orientation.x = angle.x();
    transfer.midpoint.orientation.y = angle.y();
    transfer.midpoint.orientation.z = angle.z();
    return true;
  }

  bool planDirectShelfTurn(
    const planning_scene::PlanningScene & scene, const moveit::core::RobotState & finish,
    const Eigen::Vector3d & relative, const Eigen::Quaterniond & relative_angle, Group::Plan & plan)
  {
    const auto & start = scene.getCurrentState();
    const auto * group = start.getJointModelGroup(names_.group);
    double max_delta = 0.0;
    for (const auto & name : group->getVariableNames()) {
      max_delta = std::max(max_delta,
          std::abs(finish.getVariablePosition(name) - start.getVariablePosition(name)));
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(max_delta / 0.01)));
    robot_trajectory::RobotTrajectory turn(arm().getRobotModel(), group);
    auto sample = start;
    for (int i = 0; i <= steps; ++i) {
      start.interpolate(finish, static_cast<double>(i) / steps, sample);
      sample.update();
      turn.addSuffixWayPoint(sample, 0.0);
    }
    moveit_msgs::msg::RobotTrajectory motion;
    turn.getRobotTrajectoryMsg(motion);
    if (!checkShelfTransfer(motion, scene, relative, relative_angle, limitBefore(0.24),
        "direct-turn"))
    {
      return false;
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration timing(0.00001, 0.01, 0.000001);
    if (!timing.computeTimeStamps(turn, fc::speed(0.06), fc::speed(0.06))) {return false;}
    turn.getRobotTrajectoryMsg(motion);
    if (!checkShelfTransfer(motion, scene, relative, relative_angle, limitBefore(0.24),
        "timed-turn"))
    {
      return false;
    }
    plan.trajectory = std::move(motion);
    RCLCPP_INFO(node_->get_logger(), "SHELF DIRECT TURN SELECTED");
    return true;
  }

  bool onShelf()
  {
    safe();
    const auto board = contacts_.support();
    const auto surface = contacts_.surfaceBox("SHELF");
    const auto now = Wall::now();
    return now - board.wall < 1s && now - surface.wall < 1s &&
           node_->now().nanoseconds() - board.sim < 250000000;
  }
  void waitShelfSupport()
  {
    const auto end = Wall::now() + 15s;int64_t since = -1;
    while (Wall::now() < end) {
      const auto box = box_.fresh();const auto now = node_->now().nanoseconds();
      const bool good = onShelf() && std::abs(box_.bottom(box) - cell_.surface) < 0.001 &&
        box_.tilt(box) < 0.00873;
      if (!good) {since = -1;} else if (since < 0) {
        since = now;
      } else if (now - since >= 1000000000) {
        RCLCPP_INFO(node_->get_logger(), "SHELF SUPPORT VERIFIED: 1 sim s bottom_gap=%.3f mm",
          (box_.bottom(box) - cell_.surface) * 1000);return;
      }
      std::this_thread::sleep_for(100ms);
    }
    throw std::runtime_error("Target shelf support/level not verified; refusing opening");
  }
  void executeShelfTransfer(
    const moveit_msgs::msg::RobotTrajectory & plan, const Eigen::Vector3d & relative,
    const Eigen::Quaterniond & relative_angle)
  {
    const double floor = cell_.surface - 0.0175;  // 35 mm below the planned 0.0175 m floor
    safe();
    auto execution = std::async(std::launch::async, [this, &plan]() {return arm().execute(plan);});
    double min_bottom = std::numeric_limits<double>::infinity();
    double max_drift = 0.0;
    int consecutive_drift = 0;
    try {
      while (execution.wait_for(100ms) == std::future_status::timeout) {
        safe();
        const auto box = box_.fresh();
        const auto hand = arm().getCurrentPose(tcp()).pose;
        const auto hand_inverse = rotation(hand).inverse();
        const double drift = (hand_inverse * (point(box) - point(hand)) - relative).norm();
        const double angle = relative_angle.angularDistance(hand_inverse * rotation(box));
        min_bottom = std::min(min_bottom, box_.bottom(box));
        max_drift = std::max(max_drift, drift);
        require(min_bottom > floor, "Box descended below physical transfer clearance");
        consecutive_drift = (drift > 0.008 || angle > 0.1222) ? consecutive_drift + 1 : 0;
        require(consecutive_drift < 2, "Right grasp moved during shelf transfer");
        require(welded() || contacts_.fingersFresh(1s),
          "Finger contact lost during shelf transfer");
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_->get_logger(), "SHELF TRANSFER STOP REQUESTED: %s", error.what());
      arm().stop();
      execution.wait();
      throw;
    }
    require(execution.get() == moveit::core::MoveItErrorCode::SUCCESS,
      "Shelf-front transfer execution failed");
    safe();
    RCLCPP_INFO(node_->get_logger(),
      "SHELF TRANSFER MONITOR PASSED: min_box_bottom=%.4f m max_sampled_drift=%.3f mm",
      min_bottom, max_drift * 1000);
  }
  void placeOnShelf(const Eigen::Vector3d & reference, const Eigen::Quaterniond & reference_angle)
  {
    auto box = box_.stable();auto hand = arm().getCurrentPose(tcp()).pose;
    syncHeldBox(box, hand);
    Eigen::Vector3d relative = rotation(hand).inverse() * (point(box) - point(hand));
    Eigen::Quaterniond relative_angle = rotation(hand).inverse() * rotation(box);
    arm().setGoalJointTolerance(0.00005);
    arm().setGoalPositionTolerance(0.00015);arm().setGoalOrientationTolerance(0.0005);
    arm().setMaxVelocityScalingFactor(fc::speed(0.06));arm().setMaxAccelerationScalingFactor(fc::speed(0.06));
    arm().setPlanningTime(2.0);
    ShelfTransfer transfer;
    Eigen::Quaterniond aligned = chooseShelfApproach(relative, relative_angle, transfer);
    RCLCPP_INFO(node_->get_logger(), "SHELF TRANSFER TURN START");
    executeShelfTransfer(transfer.turn.trajectory, relative, relative_angle);
    motion_.waitAt(&transfer.midpoint, 0.001, 0.005);
    double middle_drift = 0, middle_angle = 0;
    holdAt("SHELF TRANSFER MIDPOINT", reference, reference_angle, middle_drift, middle_angle,
      1.5);
    RCLCPP_INFO(node_->get_logger(), "SHELF TRANSFER ADVANCE START");
    Pose goal = hand;bool level = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      if (attempt > 0) {
        box = box_.fresh();hand = arm().getCurrentPose(tcp()).pose;
        relative = rotation(hand).inverse() * (point(box) - point(hand));
        relative_angle = rotation(hand).inverse() * rotation(box);
        aligned = nearestCubeRotation(rotation(box)) * relative_angle.inverse();
        syncHeldBox(box, hand);
      }
      goal = hand;goal.orientation.w = aligned.w();goal.orientation.x = aligned.x();
      goal.orientation.y = aligned.y();goal.orientation.z = aligned.z();
      const Eigen::Vector3d target = frontCenter() - aligned * relative;
      goal.position.x = target.x();goal.position.y = target.y();goal.position.z = target.z();
      RCLCPP_INFO(node_->get_logger(), "SHELF ALIGN START: correction %d/3", attempt + 1);
      if (attempt == 0) {
        executeShelfTransfer(transfer.advance, relative, relative_angle);
      } else {
        motion_.linear(goal, 0.01, LinearMode::normal, kExecJointMargin);
      }
      motion_.waitAt(&goal, 0.0003, 0.0015);box = box_.stable();safe();
      if (box_.tilt(box) < 0.00873 && (point(box) - frontCenter()).norm() < 0.001) {
        level = true;break;
      }
    }
    require(level, "Box not aligned at shelf front");
    double drift = 0, angle = 0;
    holdAt("SHELF ALIGNED", reference, reference_angle, drift, angle, 1.0);
    box = box_.fresh();hand = arm().getCurrentPose(tcp()).pose;syncHeldBox(box, hand);
    relative = rotation(hand).inverse() * (point(box) - point(hand));
    Eigen::Vector3d target = insertCenter() - aligned * relative;
    goal.position.x = target.x();goal.position.y = target.y();goal.position.z = target.z();
    RCLCPP_INFO(node_->get_logger(), "SHELF INSERT START: 120 mm into second shelf");
    motion_.linear(goal, 0.03, LinearMode::normal, kExecJointMargin);
    motion_.waitAt(&goal, 0.0003, 0.0015);
    box = box_.fresh();hand = arm().getCurrentPose(tcp()).pose;syncHeldBox(box, hand);
    // Test mode: seating needs box-shelf contacts, which Gazebo hides while the box is welded.
    if (right_weld_) {right_weld_->release();}
    const double descent_z = cell_.center.z() + 0.002;  // 2 mm above the resting centre
    goal.position.x = hand.position.x + cell_.center.x() - box.position.x;
    goal.position.y = hand.position.y + cell_.center.y() - box.position.y;
    goal.position.z = hand.position.z + descent_z - box.position.z;
    RCLCPP_INFO(node_->get_logger(), "SHELF DESCENT START: center z=%.4f", descent_z);
    motion_.linear(goal, 0.02, LinearMode::shelf_descent, kExecJointMargin);
    motion_.waitAt(&goal, 0.0003, 0.0015);
    contacts_.setSeating(true);
    configureContacts(true, false, true);
    const auto before_relax = box_.fresh();gripper_.command(0.0198, true);
    box = box_.stable();hand = arm().getCurrentPose(tcp()).pose;
    RCLCPP_INFO(node_->get_logger(), "SHELF GRIP RELAX: shift=%.3f mm gap=%.3f mm",
      (point(box) - point(before_relax)).norm() * 1000, (box_.bottom(box) - cell_.surface) * 1000);
    require((point(box) - point(before_relax)).norm() < 0.003 &&
      box_.bottom(box) - cell_.surface > -0.0003, "Excessive shelf seating displacement");
    syncHeldBox(box, hand);bool seated = false;
    for (int step = 0; step < 10; ++step) {
      box = box_.fresh();const double gap = box_.bottom(box) - cell_.surface;
      RCLCPP_INFO(node_->get_logger(), "SHELF SEAT step=%d gap=%.3f mm contact=%d", step,
          gap * 1000, onShelf());
      require(gap > -0.0003 && box_.tilt(box) < 0.00873, "Shelf seating penetration/tilt too large");
      // As on the left arm: a corner touch can report contact while the face still
      // hovers. Count as seated only with contact AND a light press.
      if (onShelf() && gap <= -0.000001) {seated = true;break;}
      if (gap <= 0.0) {waitShelfSupport();seated = true;break;}
      goal = arm().getCurrentPose(tcp()).pose;
      goal.orientation.w = aligned.w();goal.orientation.x = aligned.x();
      goal.orientation.y = aligned.y();goal.orientation.z = aligned.z();
      goal.position.z -= std::min(0.0005, gap + 0.0001);
      motion_.linear(goal, 0.01, LinearMode::normal, kExecJointMargin);
      motion_.waitAt(&goal, 0.00015, 0.0015);
    }
    require(seated, "Shelf support not established in bounded steps");waitShelfSupport();
    if (right_weld_) {right_weld_->release();}  // no-op: already released before seating
    const auto release_reference = box_.fresh();
    RCLCPP_INFO(node_->get_logger(), "SHELF OPEN: verified target board, two-step opening");
    gripper_.command(0.021, false);waitShelfSupport();
    double release_slip = (point(box_.fresh()) - point(release_reference)).norm();
    gripper_.command(0.04, false);motion_.detachBox();
    box = box_.stable();
    release_slip = std::max(release_slip, (point(box) - point(release_reference)).norm());
    addBox(box);goal = arm().getCurrentPose(tcp()).pose;goal.position.z += 0.08;
    RCLCPP_INFO(node_->get_logger(), "SHELF RETREAT CLEARANCE: raise 80 mm with open gripper");
    motion_.linear(goal, 0.02, LinearMode::normal, kExecJointMargin);
    motion_.waitAt(&goal, 0.001, 0.005);
    require(onShelf(), "Box lost shelf support during clearance lift");
    goal = arm().getCurrentPose(tcp()).pose;goal.position.y -= inward_ * 0.12;
    RCLCPP_INFO(node_->get_logger(),
        "SHELF RETREAT START: 120 mm at clearance height, no ready return");
    motion_.linear(goal, 0.10, LinearMode::normal, kExecJointMargin);
    motion_.waitAt(&goal);  // gripper open, above box
    const auto xy_error = [this](const Pose & b) {
        return std::hypot(b.position.x - cell_.center.x(), b.position.y - cell_.center.y());
      };
    const auto sim_start = node_->now();const auto wall_start = Wall::now();
    while ((node_->now() - sim_start).seconds() < 3.0) {
      require(Wall::now() - wall_start < 30s, "Shelf post-release clock timed out");
      box = box_.fresh();safe();
      release_slip = std::max(release_slip, (point(box) - point(release_reference)).norm());
      require(onShelf() && release_slip < 0.002 && xy_error(box) < 0.003 &&
        std::abs(box.position.z - cell_.center.z()) < 0.001 && box_.tilt(box) < 0.01745,
        "Shelf box shifted, tilted or lost target support after release");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(),
      "RIGHT SHELF PLACE PASSED: box=(%.5f, %.5f, %.5f) xy_error=%.3f mm release_slip=%.3f mm tilt=%.3f deg",
      box.position.x, box.position.y, box.position.z, xy_error(box) * 1000,
      release_slip * 1000, box_.tilt(box) * 180 / M_PI);
  }

  void holdAt(
    const char * label, const Eigen::Vector3d & reference,
    const Eigen::Quaterniond & reference_angle, double & drift, double & angle,
    double seconds = 3.0)
  {
    const auto sim_start = node_->now();const auto wall_start = Wall::now();
    while ((node_->now() - sim_start).seconds() < seconds) {
      require(Wall::now() - wall_start < 30s, "Right carry hold clock timed out");safe();
      const auto box = box_.fresh();const auto hand = arm().getCurrentPose(tcp()).pose;
      drift = std::max(drift,
          (rotation(hand).inverse() * (point(box) - point(hand)) - reference).norm());
      angle = std::max(angle,
          reference_angle.angularDistance(rotation(hand).inverse() * rotation(box)));
      require(drift < 0.005 && angle < 0.0873,
          std::string(label) + ": right grasp slipped or rotated");
      require(welded() || contacts_.fingersFresh(1s),
        "Missing physical finger contact during carry hold");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(), "%s HOLD PASSED: drift=%.3f mm angle=%.3f deg", label,
      drift * 1000, angle * 180 / M_PI);
  }

  void safe()
  {
    require(!contacts_.carryContact() && !contacts_.unexpected() &&
      !contacts_.robotTouched("TABLE") && !contacts_.robotTouched("SHELF"),
      "Unexpected physical contact");
  }
  bool onTable()
  {
    const auto table = contacts_.surfaceBox("TABLE");
    return Wall::now() - table.wall < 1s &&
           node_->now().nanoseconds() - table.sim < 250000000;
  }
  void waitTableSupport()
  {
    const double rest_z = world_->handoffCell().center.z();
    const auto end = Wall::now() + 10s;int64_t since = -1;
    while (Wall::now() < end) {
      safe();const auto box = box_.fresh();const auto now = node_->now().nanoseconds();
      if (!onTable() || std::abs(box.position.z - rest_z) > 0.001) {since = -1;} else if (since < 0) {
        since = now;
      } else if (now - since > 500000000) {return;}
      std::this_thread::sleep_for(100ms);
    }
    throw std::runtime_error("Table support not verified before right pickup");
  }
  const std::vector<double> ready_joints_{0, -0.785, 0, -2.356, 0, 1.571, 0.785};

  moveit::core::RobotStatePtr parkingStart(Group & group)
  {
    auto state = group.getCurrentState(2.0);
    require(static_cast<bool>(state), "Parking start state unavailable");
    const auto * joints = state->getJointModelGroup(group.getName());
    // Recover only from numeric overshoot at a simulated joint stop.
    constexpr double roundoff = 0.00001;
    for (const auto & name : joints->getVariableNames()) {
      const auto & bounds = state->getRobotModel()->getVariableBounds(name);
      const double position = state->getVariablePosition(name);
      const double bounded = std::clamp(position, bounds.min_position_, bounds.max_position_);
      require(std::abs(position - bounded) <= roundoff, "Parking start exceeds joint bounds");
      if (position != bounded) {
        RCLCPP_INFO(node_->get_logger(), "PARK START ROUNDOFF: %s %.9f -> %.9f",
          name.c_str(), position, bounded);
        state->setVariablePosition(name, bounded);
      }
    }
    state->update();
    return state;
  }

  Group::Plan planPark(Group & group, const moveit::core::RobotState * raised = nullptr)
  {
    group.setMaxVelocityScalingFactor(fc::speed(0.40));
    group.setMaxAccelerationScalingFactor(fc::speed(0.40));
    group.setPlanningTime(10);
    group.setGoalJointTolerance(0.001);
    require(group.setJointValueTarget(ready_joints_), "Invalid parking target");
    Group::Plan plan;
    const int attempts = raised ? 6 : 3;
    for (int i = 0; i < attempts; ++i) {
      if (raised) {
        group.setStartState(*raised);
      } else {
        group.setStartState(*parkingStart(group));
      }
      if (group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {return plan;}
    }
    throw std::runtime_error("Arm parking failed to find a collision-checked route");
  }

  void waitParkingJoints(Group & group, const std::vector<double> & target)
  {
    auto previous = group.getCurrentJointValues();int calm = 0;const auto end = Wall::now() + 20s;
    while (Wall::now() < end) {
      safe();std::this_thread::sleep_for(100ms);  // with 0.0001 rad: same 1 mrad/s criterion
      auto state = group.getCurrentState(2.0);
      require(static_cast<bool>(state), "Parking state stale");
      std::vector<double> current;state->copyJointGroupPositions(group.getName(), current);
      require(current.size() == 7 && previous.size() == 7, "Parking state incomplete");
      double delta = 0, error = 0;
      for (size_t i = 0; i < 7; ++i) {
        delta = std::max(delta, std::abs(current[i] - previous[i]));
        error = std::max(error, std::abs(current[i] - target[i]));
      }
      calm = delta < 0.0001 && error < 0.001 ? calm + 1 : 0;previous = current;
      if (calm >= 3) {return;}
    }
    throw std::runtime_error("Arm failed to settle at parking waypoint");
  }

  void park(Group & group)
  {
    const auto plan = planPark(group);
    safe();
    require(group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS,
      "Arm parking execution failed");
    waitParkingJoints(group, ready_joints_);
  }

  void parkLeftAfterPlacement(Group & left)
  {
    const fc::ArmNames left_names("left");
    safe();
    const auto start = parkingStart(left);
    planning_scene::PlanningScene scene(left.getRobotModel());
    require(scene.setPlanningSceneMsg(motion_.sceneSnapshot()), "Left parking scene import failed");
    const auto * group = start->getJointModelGroup(left_names.group);
    const auto * tcp_link = start->getLinkModel(left_names.tcp);
    Eigen::Isometry3d target = start->getGlobalLinkTransform(tcp_link);
    target.translation().z() += 0.08;
    auto state = *start;
    std::vector<moveit::core::RobotStatePtr> path;
    const auto valid = [&scene, &left_names](moveit::core::RobotState * sample,
      const moveit::core::JointModelGroup * joints, const double * values) {
        sample->setJointGroupPositions(joints, values);
        sample->update();
        return sample->satisfiesBounds(joints) && !scene.isStateColliding(*sample, left_names.group);
      };
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &state, group, path, tcp_link, target, true, moveit::core::MaxEEFStep(0.001),
      moveit::core::CartesianPrecision{}, valid, kinematics::KinematicsQueryOptions());
    require(fraction >= 0.999 && path.size() >= 2, "Incomplete left parking lift");
    robot_trajectory::RobotTrajectory lift(left.getRobotModel(), group);
    const auto & names = group->getVariableNames();
    for (size_t i = 0; i < path.size(); ++i) {
      if (i > 0) {
        for (const auto & name : names) {
          require(std::abs(path[i]->getVariablePosition(name) -
            path[i - 1]->getVariablePosition(name)) < 0.02, "Joint jump in left parking lift");
        }
      }
      lift.addSuffixWayPoint(*path[i], 0);
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration timing(0.00001, 0.01, 0.000001);
    require(timing.computeTimeStamps(lift, fc::speed(0.08), fc::speed(0.08)), "Left parking lift timing failed");
    for (size_t i = 0; i < lift.getWayPointCount(); ++i) {
      require(!scene.isStateColliding(lift.getWayPoint(i), left_names.group),
        "Timed left parking lift is not collision-free");
    }
    const auto & raised = lift.getLastWayPoint();
    const auto & endpoint = raised.getGlobalLinkTransform(tcp_link);
    require((endpoint.translation() - target.translation()).norm() < 0.0005 &&
      Eigen::Quaterniond(endpoint.rotation()).angularDistance(Eigen::Quaterniond(
        target.rotation())) <
      0.005, "Left parking lift misses endpoint");
    const auto parking = planPark(left, &raised);
    std::vector<double> raised_joints;
    raised.copyJointGroupPositions(group, raised_joints);
    moveit_msgs::msg::RobotTrajectory motion;
    lift.getRobotTrajectoryMsg(motion);
    RCLCPP_INFO(node_->get_logger(), "LEFT PARK TWO-STAGE PLAN PASSED: lift=80 mm");
    safe();
    require(left.execute(motion) == moveit::core::MoveItErrorCode::SUCCESS,
      "Left parking lift execution failed");
    waitParkingJoints(left, raised_joints);
    safe();
    require(left.execute(parking) == moveit::core::MoveItErrorCode::SUCCESS,
      "Left park execution after lift failed");
    waitParkingJoints(left, ready_joints_);
    left.setStartStateToCurrentState();
  }
  // Box-vs-link collision allowances for planning: right fingers (pickup and carry),
  // tabletop support, target shelf support. Left hand links always stay checked.
  void configureContacts(
    bool allow_right, bool allow_table_support = true,
    bool allow_shelf_support = false)
  {
    const fc::ArmNames left_names("left");
    motion_.setBoxCollisions({
      {"table_link", allow_table_support},
      {world_->targetShelf().link(), allow_shelf_support},
      {left_names.left_finger, false}, {left_names.right_finger, false}, {left_names.hand, false},
      {names_.left_finger, allow_right}, {names_.right_finger, allow_right},
      {names_.hand, false}});
  }
  double wristMargin(const moveit::core::RobotState & state)
  {
    const std::string wrist = "right_fr3v2_joint7";
    const auto & bounds = state.getRobotModel()->getVariableBounds(wrist);
    const double angle = state.getVariablePosition(wrist);
    return std::min(angle - bounds.min_position_, bounds.max_position_ - angle);
  }
  // Joint-limit margins. Execution requires kExecJointMargin on every waypoint (the pickup
  // descent/lift via LinearMode::pickup, the in-shelf motions explicitly). Planning probes
  // require the larger kPlanJointMargin, so the measured start state (a few mrad away from the
  // planned one) cannot push an accepted candidate below the execution margin.
  static constexpr double kExecJointMargin = 0.05;
  static constexpr double kPlanJointMargin = 0.07;
  Group::Plan planPickupApproach(const Pose & pre, const Pose & pick)
  {
    planning_scene::PlanningScene scene(arm().getRobotModel());
    require(scene.setPlanningSceneMsg(motion_.sceneSnapshot()), "Pickup probe scene import failed");
    auto & acm = scene.getAllowedCollisionMatrixNonConst();
    acm.setEntry("target_box", names_.left_finger, true);
    acm.setEntry("target_box", names_.right_finger, true);
    const auto * group = scene.getRobotModel()->getJointModelGroup(names_.group);
    const auto valid = motion_.collisionCheck(scene, kPlanJointMargin);
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = point(pre);
    target.linear() = rotation(pre).toRotationMatrix();
    for (int attempt = 0; attempt < 12; ++attempt) {
      auto state = scene.getCurrentState();
      if (attempt > 0) {state.setToRandomPositions(group);}
      if (!state.setFromIK(group, target, tcp(), 0.08, valid)) {continue;}
      std::vector<moveit::core::RobotStatePtr> path;
      if (!motion_.checkedFinePath(scene, state, pick, path, kPlanJointMargin)) {continue;}
      std::vector<double> joints;
      state.copyJointGroupPositions(group, joints);
      if (!arm().setJointValueTarget(joints)) {continue;}
      // As on the left arm: replan the same (nearby) candidate before falling back to a
      // random IK seed, which tends to give low joint margins and long reconfigurations.
      Group::Plan plan;bool planned = false;
      for (int replan = 0; replan < 5 && !planned; ++replan) {
        arm().setStartStateToCurrentState();
        planned = arm().plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        if (!planned) {
          RCLCPP_WARN(node_->get_logger(),
            "RIGHT PICK REPLAN %d/5: candidate=%d plan rejected", replan + 1, attempt + 1);
        }
      }
      if (!planned) {continue;}
      double minimum = jointLimitMargin(state, group);
      double wrist = wristMargin(state);
      for (const auto & waypoint : path) {
        minimum = std::min(minimum, jointLimitMargin(*waypoint, group));
        wrist = std::min(wrist, wristMargin(*waypoint));
      }
      RCLCPP_INFO(node_->get_logger(),
        "RIGHT PICK PROBE PASSED: candidate=%d joint margin=%.4f rad joint7 margin=%.4f rad",
        attempt + 1, minimum, wrist);
      return plan;
    }
    throw std::runtime_error("No collision-checked pickup with sufficient joint margin");
  }

  rclcpp::Node::SharedPtr node_;
  fc::ArmNames names_;
  fc::TaskConfig task_;
  fc::BoxTracker box_;
  fc::Gripper gripper_;
  fc::ContactMonitor contacts_;
  fc::ArmMotion motion_;
  std::unique_ptr<fc::WorldModel> world_;
  fc::Cell cell_;
  double inward_ = 1.0;
  std::atomic<bool> monitor_shelf_{false};
  std::unique_ptr<fc::BoxWeld> left_weld_, right_weld_;  // rigid_grasp test mode only
};
}  // namespace

namespace fr3_cooperative_control
{
class RightTransfer::Impl : public RightTablePick
{
public:
  using RightTablePick::RightTablePick;
};

RightTransfer::RightTransfer(rclcpp::Node::SharedPtr node)
: impl_(std::make_unique<Impl>(std::move(node)))
{
}

RightTransfer::~RightTransfer() = default;

void RightTransfer::run(const std::string & stage, bool left_placed)
{
  impl_->run(stage, left_placed);
}
}  // namespace fr3_cooperative_control
