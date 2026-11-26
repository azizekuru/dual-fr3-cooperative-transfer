#include "fr3_cooperative_control/arm_motion.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <thread>

#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace fr3_cooperative_control
{
using namespace std::chrono_literals;

ArmMotion::ArmMotion(rclcpp::Node::SharedPtr node, const ArmNames & names, MotionProfile profile,
  std::function<void()> before_plan, std::function<void()> before_execute)
: node_(std::move(node)), names_(names), profile_(profile),
  before_plan_(std::move(before_plan)), before_execute_(std::move(before_execute))
{
}

void ArmMotion::connect()
{
  group_ = std::make_unique<Group>(node_, names_.group);
  scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
}

double ArmMotion::jointLimitMargin(
  const moveit::core::RobotState & state, const moveit::core::JointModelGroup * group)
{
  double margin = std::numeric_limits<double>::infinity();
  for (const auto & name : group->getVariableNames()) {
    // joint5's planning bound (+/-2.60 rad, fr3_moveit_params.py) is an artificial limit kept
    // 0.276 rad inside its physical stop (+/-2.876 rad); it is enforced by satisfiesBounds and
    // needs no extra margin. The margin protects the physical limits of the other joints.
    if (name.size() >= 7 && name.compare(name.size() - 7, 7, "_joint5") == 0) {continue;}
    const auto & bounds = state.getRobotModel()->getVariableBounds(name);
    const double position = state.getVariablePosition(name);
    margin = std::min({margin, position - bounds.min_position_, bounds.max_position_ - position});
  }
  return margin;
}

moveit_msgs::msg::PlanningScene ArmMotion::sceneSnapshot()
{
  auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  require(client->wait_for_service(3s), "Fine path planning scene unavailable");
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  using Components = moveit_msgs::msg::PlanningSceneComponents;
  request->components.components = Components::SCENE_SETTINGS | Components::ROBOT_STATE |
    Components::ROBOT_STATE_ATTACHED_OBJECTS | Components::WORLD_OBJECT_GEOMETRY |
    Components::OCTOMAP | Components::TRANSFORMS | Components::ALLOWED_COLLISION_MATRIX |
    Components::LINK_PADDING_AND_SCALING;
  auto future = client->async_send_request(request);
  require(future.wait_for(5s) == std::future_status::ready, "Fine planning scene timed out");
  return future.get()->scene;
}

moveit::core::GroupStateValidityCallbackFn ArmMotion::collisionCheck(
  const planning_scene::PlanningScene & scene, double min_joint_margin)
{
  const std::string group = names_.group;
  return [&scene, group, min_joint_margin](moveit::core::RobotState * candidate,
           const moveit::core::JointModelGroup * joints, const double * values) {
           candidate->setJointGroupPositions(joints, values);
           candidate->update();
           return candidate->satisfiesBounds(joints) &&
                  (min_joint_margin < 0 || jointLimitMargin(*candidate, joints) >= min_joint_margin) &&
                  !scene.isStateColliding(*candidate, group);
         };
}

bool ArmMotion::checkedFinePath(
  const planning_scene::PlanningScene & scene, moveit::core::RobotState state,
  const Pose & target, std::vector<moveit::core::RobotStatePtr> & path,
  double min_joint_margin, bool report)
{
  const auto * group = state.getJointModelGroup(names_.group);
  if (min_joint_margin >= 0 && jointLimitMargin(state, group) < min_joint_margin) {return false;}
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = point(target);
  transform.linear() = rotation(target).toRotationMatrix();
  const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
    &state, group, path, state.getLinkModel(names_.tcp), transform, true,
    moveit::core::MaxEEFStep(0.0001), moveit::core::CartesianPrecision{0.00001, 0.0001, 0.00001},
    collisionCheck(scene, min_joint_margin), kinematics::KinematicsQueryOptions());
  if (fraction < 0.999 || path.size() < 2) {
    if (report) {
      RCLCPP_WARN(node_->get_logger(),
        "FINE PATH INCOMPLETE: checked=%.2f%% points=%zu", 100.0 * fraction, path.size());
    }
    return false;
  }
  for (size_t i = 1; i < path.size(); ++i) {
    for (const auto & name : group->getVariableNames()) {
      const double jump = std::abs(path[i]->getVariablePosition(name) -
        path[i - 1]->getVariablePosition(name));
      if (jump >= 0.02) {
        if (report) {
          RCLCPP_WARN(node_->get_logger(),
            "FINE PATH JOINT JUMP: joint=%s point=%zu delta=%.6f rad limit=0.020000 rad",
            name.c_str(), i, jump);
        }
        return false;
      }
    }
  }
  return true;
}

void ArmMotion::finePath(
  const Pose & target, robot_trajectory::RobotTrajectory & trajectory, double min_joint_margin)
{
  planning_scene::PlanningScene scene(group_->getRobotModel());
  require(scene.setPlanningSceneMsg(sceneSnapshot()), "Fine planning scene import failed");
  std::vector<moveit::core::RobotStatePtr> path;
  if (!checkedFinePath(scene, scene.getCurrentState(), target, path, min_joint_margin, true)) {
    throw FinePathRejected("Incomplete collision-checked fine path, joint margin or jump");
  }
  for (const auto & waypoint : path) {trajectory.addSuffixWayPoint(*waypoint, 0.0);}
}

// TOTG occasionally fails numerically ("Negative path velocity") on dense fine
// paths. Retry the same geometric path with slightly different TOTG parameters;
// the endpoint/precision checks after timing still apply unchanged.
bool ArmMotion::timeWithRetry(
  robot_trajectory::RobotTrajectory & trajectory, double scale, bool fine)
{
  const robot_trajectory::RobotTrajectory original(trajectory, true);
  const double tolerance = fine ? 0.00001 : 0.001, dt = fine ? 0.005 : 0.01,
    angle = fine ? 0.000001 : 0.0001;
  const std::array<std::array<double, 3>, 3> variants{{
    {tolerance, dt, angle}, {2 * tolerance, dt, 10 * angle}, {0.5 * tolerance, 0.5 * dt, angle}}};
  for (size_t i = 0; i < variants.size(); ++i) {
    if (i > 0) {
      trajectory = robot_trajectory::RobotTrajectory(original, true);
      RCLCPP_WARN(node_->get_logger(), "TIMING RETRY %zu/%zu: TOTG failed, retrying with "
        "tolerance=%.6f dt=%.4f", i, variants.size() - 1, variants[i][0], variants[i][1]);
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration timing(
      variants[i][0], variants[i][1], variants[i][2]);
    if (timing.computeTimeStamps(trajectory, speed(scale), speed(scale))) {return true;}
  }
  trajectory = robot_trajectory::RobotTrajectory(original, true);
  return false;
}

robot_trajectory::RobotTrajectory ArmMotion::timeShelfDescent(
  const robot_trajectory::RobotTrajectory & raw, double scale)
{
  planning_scene::PlanningScene scene(group_->getRobotModel());
  require(scene.setPlanningSceneMsg(sceneSnapshot()), "Timing fallback scene import failed");
  const auto * group = raw.getGroup();
  const auto & names = group->getVariableNames();
  auto waypoint = raw.getFirstWayPoint();
  const Eigen::Isometry3d start = waypoint.getGlobalLinkTransform(names_.tcp);
  const Eigen::Isometry3d end = raw.getLastWayPoint().getGlobalLinkTransform(names_.tcp);
  const Eigen::Vector3d direction = end.translation() - start.translation();
  require(direction.squaredNorm() > 0, "Timing fallback requires a nonzero descent");
  const Eigen::Quaterniond start_angle(start.rotation()), end_angle(end.rotation());
  for (const auto & name : names) {
    waypoint.setVariableVelocity(name, 0);
    waypoint.setVariableAcceleration(name, 0);
  }
  robot_trajectory::RobotTrajectory timed(group_->getRobotModel(), group);
  timed.addSuffixWayPoint(waypoint, 0);

  // With zero endpoint derivatives, the controller's quintic spline traverses
  // each joint-space segment monotonically. Bound its peak derivatives.
  constexpr double peak_velocity = 1.875;
  constexpr double peak_acceleration = 5.773502691896258;
  constexpr size_t stride = 20;
  const size_t last = raw.getWayPointCount() - 1;
  for (size_t from = 0; from < last; ) {
    const size_t to = std::min(from + stride, last);
    const auto & a = raw.getWayPoint(from);
    const auto & b = raw.getWayPoint(to);
    double duration = 0.02;
    double max_delta = 0;
    for (const auto & name : names) {
      const auto & limits = raw.getRobotModel()->getVariableBounds(name);
      const double velocity =
        std::min(-limits.min_velocity_, limits.max_velocity_) * speed(scale);
      const double acceleration =
        std::min(-limits.min_acceleration_, limits.max_acceleration_) * speed(scale);
      require(limits.velocity_bounded_ && limits.acceleration_bounded_ &&
        velocity > 0 && acceleration > 0, "Missing joint limits for shelf timing fallback");
      const double delta = std::abs(b.getVariablePosition(name) - a.getVariablePosition(name));
      max_delta = std::max(max_delta, delta);
      duration = std::max({duration, peak_velocity * delta / velocity,
            std::sqrt(peak_acceleration * delta / acceleration)});
    }
    const int samples = std::max(1, static_cast<int>(std::ceil(max_delta / 0.001)));
    for (int sample = 0; sample <= samples; ++sample) {
      a.interpolate(b, static_cast<double>(sample) / samples, waypoint);
      waypoint.update();
      const auto & tcp = waypoint.getGlobalLinkTransform(names_.tcp);
      const double ratio = std::clamp(
        (tcp.translation() - start.translation()).dot(direction) / direction.squaredNorm(),
        0.0, 1.0);
      const Eigen::Vector3d expected = start.translation() + ratio * direction;
      require((tcp.translation() - expected).norm() < 0.00025 &&
        Eigen::Quaterniond(tcp.rotation()).angularDistance(start_angle.slerp(ratio, end_angle)) <
        0.0015, "Shelf timing fallback deviates from Cartesian descent");
      require(waypoint.satisfiesBounds(group) && !scene.isStateColliding(waypoint, names_.group),
        "Shelf timing fallback is not collision-free");
    }
    for (const auto & name : names) {
      waypoint.setVariableVelocity(name, 0);
      waypoint.setVariableAcceleration(name, 0);
    }
    timed.addSuffixWayPoint(waypoint, 1.05 * duration);
    from = to;
  }
  RCLCPP_INFO(node_->get_logger(), "SHELF DESCENT FALLBACK TIMED: points=%zu duration=%.3f s",
    timed.getWayPointCount(), timed.getDuration());
  return timed;
}

void ArmMotion::linear(
  const Pose & target, double scale, LinearMode mode, double min_joint_margin)
{
  if (before_plan_) {before_plan_();}
  group_->setStartStateToCurrentState();moveit_msgs::msg::RobotTrajectory msg;
  auto state = group_->getCurrentState(2.0);
  require(static_cast<bool>(state), "No trajectory state");
  robot_trajectory::RobotTrajectory trajectory(group_->getRobotModel(), names_.group);
  const bool pickup = mode == LinearMode::pickup;
  const double margin = std::max(pickup ? 0.05 : -1.0, min_joint_margin);
  const bool fine = margin >= 0 || mode == LinearMode::precise ||
    scale <= profile_.fine_scale_threshold;
  if (fine) {finePath(target, trajectory, margin);} else {
    const double fraction = group_->computeCartesianPath({target}, 0.002, msg, true);
    if (fraction < 0.999) {
      moveit_msgs::msg::RobotTrajectory probe;
      const double unchecked = group_->computeCartesianPath({target}, 0.002, probe, false);
      RCLCPP_ERROR(node_->get_logger(),
        "CARTESIAN DIAG: checked=%.3f unchecked=%.3f (probe not executed)", fraction, unchecked);
      throw std::runtime_error("Incomplete collision-checked Cartesian path");
    }
    trajectory.setRobotTrajectoryMsg(*state, msg);
  }
  if (fine) {
    const auto raw = trajectory.getLastWayPoint().getGlobalLinkTransform(names_.tcp);
    RCLCPP_INFO(node_->get_logger(), "RAW FINE ENDPOINT: error=%.4f mm",
      1000 * (raw.translation() - point(target)).norm());
  }
  std::optional<robot_trajectory::RobotTrajectory> raw;
  if (mode == LinearMode::shelf_descent) {raw.emplace(trajectory, true);}
  if (!timeWithRetry(trajectory, scale, fine)) {
    require(raw.has_value(), "Trajectory timing failed");
    RCLCPP_WARN(node_->get_logger(), "SHELF DESCENT TIMING FALLBACK: validating saved path");
    trajectory = timeShelfDescent(*raw, scale);
  }
  const auto endpoint = trajectory.getLastWayPoint().getGlobalLinkTransform(names_.tcp);
  const double error = (endpoint.translation() - point(target)).norm();
  const double angle = Eigen::Quaterniond(endpoint.rotation()).angularDistance(rotation(target));
  RCLCPP_INFO(node_->get_logger(), "TIMED ENDPOINT: error=%.3f mm angle=%.4f rad",
    error * 1000, angle);
  require(error < (fine ? 0.00005 : profile_.coarse_position_tolerance) &&
    angle < (fine ? profile_.fine_angle_tolerance : profile_.coarse_angle_tolerance),
    "Timed endpoint diverged from target");
  for (const auto & name : state->getJointModelGroup(names_.group)->getVariableNames()) {
    RCLCPP_INFO(node_->get_logger(), "TIMED JOINT %s target=%.6f", name.c_str(),
      trajectory.getLastWayPoint().getVariablePosition(name));
  }
  trajectory.getRobotTrajectoryMsg(msg);
  if (before_execute_) {before_execute_();}
  require(group_->execute(msg) == moveit::core::MoveItErrorCode::SUCCESS,
    "Cartesian execution failed");
}

void ArmMotion::waitAt(
  const Pose * target, double position_tolerance, double angle_tolerance,
  bool allow_pickup_correction)
{
  auto previous = group_->getCurrentJointValues();
  int calm = 0, still = 0;
  const auto end = Wall::now() + 20s;
  while (Wall::now() < end) {
    // 100 ms samples with 0.0001 rad: 1 mrad/s settle criterion.
    std::this_thread::sleep_for(100ms);
    auto state = group_->getCurrentState(2.0);
    require(static_cast<bool>(state), "Fresh arm state missing");
    std::vector<double> current;state->copyJointGroupPositions(names_.group, current);
    require(current.size() == 7 && previous.size() == 7, "Incomplete arm state");
    double delta = 0;
    for (size_t i = 0; i < 7; ++i) {delta = std::max(delta, std::abs(current[i] - previous[i]));}
    bool near = true;
    double position_error = 0.0, angle_error = 0.0;
    if (target) {
      state->update();
      const auto & actual = state->getGlobalLinkTransform(names_.tcp);
      position_error = (actual.translation() - point(*target)).norm();
      angle_error = Eigen::Quaterniond(actual.rotation()).angularDistance(rotation(*target));
      near = position_error < position_tolerance && angle_error < angle_tolerance;
    }
    still = delta < 0.0001 ? still + 1 : 0;
    calm = still > 0 && near ? calm + 1 : 0;
    previous = current;
    if (calm >= 3) {return;}
    if (allow_pickup_correction && target && still >= 3 && !near &&
      position_error < 0.003 && angle_error < 0.015)
    {
      RCLCPP_INFO(node_->get_logger(),
        "PICK APPROACH CORRECTION: position_error=%.3f mm angle=%.4f rad",
        position_error * 1000, angle_error);
      linear(*target, 0.01, LinearMode::pickup);
      waitAt(target, position_tolerance, angle_tolerance);
      RCLCPP_INFO(node_->get_logger(), "PICK APPROACH CORRECTION PASSED");
      return;
    }
  }
  const auto actual = group_->getCurrentPose(names_.tcp).pose;
  if (target) {
    RCLCPP_ERROR(node_->get_logger(), "SETTLE FAILED: position_error=%.3f mm angle=%.4f rad",
      1000 * (point(actual) - point(*target)).norm(),
      rotation(actual).angularDistance(rotation(*target)));
  }
  auto state = group_->getCurrentState(2.0);
  if (state) {
    for (const auto & name : state->getJointModelGroup(names_.group)->getVariableNames()) {
      RCLCPP_ERROR(node_->get_logger(), "SETTLE JOINT %s actual=%.6f", name.c_str(),
        state->getVariablePosition(name));
    }
  }
  throw std::runtime_error("Arm not settled at target; refusing next motion");
}

// The test narrows joint5 planning bounds to +/-2.60 rad (physical stop 2.876 rad). A plan that
// ends on that artificial bound can settle a few microradians outside it, and MoveIt then
// rejects every next plan. Clamp only such tiny excursions (<= 0.005 rad, below the default
// 0.01 rad execution start tolerance); larger violations are left for MoveIt to reject.
void ArmMotion::setClampedStartState()
{
  auto current = group_->getCurrentState(2.0);
  require(static_cast<bool>(current), "No planning start state");
  moveit::core::RobotState start(*current);
  for (const auto * joint : start.getJointModelGroup(names_.group)->getActiveJointModels()) {
    const auto & name = joint->getName();
    const auto & bounds = joint->getVariableBounds(name);
    const double value = start.getVariablePosition(name);
    const double excess = std::max(bounds.min_position_ - value, value - bounds.max_position_);
    if (excess > 0 && excess <= 0.005) {
      start.setVariablePosition(name,
        std::clamp(value, bounds.min_position_, bounds.max_position_));
      RCLCPP_WARN(node_->get_logger(),
        "START STATE CLAMP: %s actual=%.6f excess=%.6f rad (planning bound only)",
        name.c_str(), value, excess);
    }
  }
  start.update();
  group_->setStartState(start);
}

void ArmMotion::executePlan(const PlanOptions & options, const Accept & accept, const Run & run)
{
  Group::Plan plan;bool planned = false;
  for (int attempt = 0; attempt < options.attempts; ++attempt) {
    if (options.clamp_start) {setClampedStartState();} else {group_->setStartStateToCurrentState();}
    if (group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
      (!accept || accept(plan))) {planned = true;break;}
    if (options.log_attempts) {
      RCLCPP_WARN(node_->get_logger(), "PLAN ATTEMPT %d/%d rejected; replanning",
        attempt + 1, options.attempts);
    }
  }
  require(planned, "Planning failed");
  if (run) {run(plan);return;}
  if (before_execute_) {before_execute_();}
  require(group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS, "Execution failed");
}

void ArmMotion::addBox(const Pose & box, double size)
{
  moveit_msgs::msg::CollisionObject object;object.header.frame_id = "world";
  object.id = "target_box";object.operation = object.ADD;
  shape_msgs::msg::SolidPrimitive primitive;primitive.type = primitive.BOX;
  primitive.dimensions = {size, size, size};
  object.primitives.push_back(primitive);object.primitive_poses.push_back(box);
  require(scene_->applyCollisionObject(object), "Planning box synchronization failed");
}

void ArmMotion::syncHeldBox(const Pose & box, const Pose & tcp, double size)
{
  const Eigen::Vector3d relative = rotation(tcp).inverse() * (point(box) - point(tcp));
  const Eigen::Quaterniond q = rotation(tcp).inverse() * rotation(box);
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = names_.tcp;
  attached.touch_links = {names_.left_finger, names_.right_finger, names_.hand};
  attached.object.id = "target_box";attached.object.header.frame_id = names_.tcp;
  attached.object.operation = attached.object.ADD;
  shape_msgs::msg::SolidPrimitive primitive;primitive.type = primitive.BOX;
  primitive.dimensions = {size, size, size};
  Pose p;p.position.x = relative.x();p.position.y = relative.y();p.position.z = relative.z();
  p.orientation.w = q.w();p.orientation.x = q.x();p.orientation.y = q.y();p.orientation.z = q.z();
  attached.object.primitives.push_back(primitive);attached.object.primitive_poses.push_back(p);
  require(scene_->applyAttachedCollisionObject(attached), "Cannot synchronize held planning box");
}

void ArmMotion::attachBox()
{
  group_->attachObject("target_box", names_.tcp,
    {names_.left_finger, names_.right_finger, names_.hand});
  const auto end = Wall::now() + 5s;
  while (scene_->getAttachedObjects({"target_box"}).empty() && Wall::now() < end) {
    std::this_thread::sleep_for(100ms);
  }
  require(!scene_->getAttachedObjects({"target_box"}).empty(), "Planning attachment failed");
}

void ArmMotion::detachBox()
{
  group_->detachObject("target_box");
  const auto end = Wall::now() + 5s;
  while (!scene_->getAttachedObjects({"target_box"}).empty() && Wall::now() < end) {
    std::this_thread::sleep_for(100ms);
  }
  require(scene_->getAttachedObjects({"target_box"}).empty(), "Planning detach failed");
}

void ArmMotion::setBoxCollisions(const std::vector<std::pair<std::string, bool>> & entries)
{
  auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  require(client->wait_for_service(10s), "Planning scene unavailable");
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto future = client->async_send_request(request);
  require(future.wait_for(5s) == std::future_status::ready, "Collision matrix timeout");
  auto acm = future.get()->scene.allowed_collision_matrix;
  const auto ensure = [&acm](const std::string & name) {
      const auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
      if (it != acm.entry_names.end()) {return static_cast<size_t>(it - acm.entry_names.begin());}
      acm.entry_names.push_back(name);
      for (auto & row : acm.entry_values) {row.enabled.push_back(false);}
      moveit_msgs::msg::AllowedCollisionEntry row;
      row.enabled.resize(acm.entry_names.size(), false);
      acm.entry_values.push_back(row);
      return acm.entry_names.size() - 1;
    };
  const size_t box = ensure("target_box");
  for (const auto & [name, allowed] : entries) {
    const size_t index = ensure(name);
    acm.entry_values[box].enabled[index] = allowed;acm.entry_values[index].enabled[box] = allowed;
  }
  moveit_msgs::msg::PlanningScene diff;diff.is_diff = true;diff.allowed_collision_matrix = acm;
  require(scene_->applyPlanningScene(diff), "Box collision allowance update failed");
}

}  // namespace fr3_cooperative_control
