// One arm's MoveIt motion toolbox: planning-scene helpers, collision-checked fine Cartesian
// paths, TOTG timing with retries, settle checks and plan/execute loops.
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fr3_cooperative_control/common.hpp"

namespace fr3_cooperative_control
{

using Group = moveit::planning_interface::MoveGroupInterface;

// Per-arm motion thresholds (kept exactly as each arm was validated with).
struct MotionProfile
{
  double fine_scale_threshold = 0.01;  // linear(): scale <= this uses the fine path
  double coarse_position_tolerance = 0.003;  // timed endpoint error, coarse path
  double coarse_angle_tolerance = 0.025;
  double fine_angle_tolerance = 0.025;
};

enum class LinearMode
{
  normal,         // fine path only for slow scales (profile threshold)
  precise,        // always the fine path
  pickup,         // fine path + joint margin >= 0.05 rad on every waypoint
  shelf_descent,  // like normal; if TOTG fails, validated quintic-segment timing fallback
};

struct PlanOptions
{
  int attempts = 3;
  bool clamp_start = false;     // clamp tiny artificial-bound excursions of the start state
  bool log_attempts = false;
};

class ArmMotion
{
public:
  using Accept = std::function<bool(const Group::Plan &)>;
  using Run = std::function<void(const Group::Plan &)>;

  ArmMotion(rclcpp::Node::SharedPtr node, const ArmNames & names, MotionProfile profile,
    std::function<void()> before_plan = nullptr, std::function<void()> before_execute = nullptr);

  // Create the MoveGroupInterface/PlanningSceneInterface (after the controllers are active).
  void connect();
  Group & group() {return *group_;}
  moveit::planning_interface::PlanningSceneInterface & scene() {return *scene_;}
  const ArmNames & names() const {return names_;}
  const std::string & tcp() const {return names_.tcp;}

  // Planning scene snapshot from move_group, for local planning/collision probes.
  moveit_msgs::msg::PlanningScene sceneSnapshot();
  moveit::core::GroupStateValidityCallbackFn collisionCheck(
    const planning_scene::PlanningScene & scene, double min_joint_margin = -1.0);
  // 0.1 mm steps, every step collision-checked, joint jumps < 0.02 rad, optional joint margin.
  bool checkedFinePath(const planning_scene::PlanningScene & scene, moveit::core::RobotState state,
    const Pose & target, std::vector<moveit::core::RobotStatePtr> & path,
    double min_joint_margin = -1.0, bool report = false);
  bool timeWithRetry(robot_trajectory::RobotTrajectory & trajectory, double scale, bool fine);
  // min_joint_margin >= 0 forces the fine path and requires that margin on every waypoint.
  void linear(const Pose & target, double scale = 0.1, LinearMode mode = LinearMode::normal,
    double min_joint_margin = -1.0);
  // Joints still for 3 x 100 ms and TCP within tolerance; optional small pickup correction.
  void waitAt(const Pose * target, double position_tolerance = 0.003,
    double angle_tolerance = 0.025, bool allow_pickup_correction = false);
  // Clamp start-state excursions <= 0.005 rad beyond the planning bounds (joint5 limits).
  void setClampedStartState();
  void executePlan(const PlanOptions & options, const Accept & accept = nullptr,
    const Run & run = nullptr);

  // Planning-scene box (world object, attached copy, allowed collisions).
  void addBox(const Pose & box, double size);
  void syncHeldBox(const Pose & box, const Pose & tcp, double size);
  void attachBox();
  void detachBox();
  // Set box-vs-link allowed collisions; links not listed keep their current entry.
  void setBoxCollisions(const std::vector<std::pair<std::string, bool>> & entries);

  static double jointLimitMargin(
    const moveit::core::RobotState & state, const moveit::core::JointModelGroup * group);

private:
  void finePath(const Pose & target, robot_trajectory::RobotTrajectory & trajectory,
    double min_joint_margin);
  robot_trajectory::RobotTrajectory timeShelfDescent(
    const robot_trajectory::RobotTrajectory & raw, double scale);

  rclcpp::Node::SharedPtr node_;
  ArmNames names_;
  MotionProfile profile_;
  std::function<void()> before_plan_, before_execute_;
  std::unique_ptr<Group> group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;
};

}  // namespace fr3_cooperative_control
