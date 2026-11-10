// Physical right-arm table pickup; attachment affects planning only.
#include <algorithm>
#include <array>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <Eigen/Geometry>
#include <control_msgs/action/gripper_command.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;
using Pose = geometry_msgs::msg::Pose;
using Group = moveit::planning_interface::MoveGroupInterface;
using Grip = control_msgs::action::GripperCommand;
using Wall = std::chrono::steady_clock;
enum class LinearMode {normal, pickup, shelf_descent};

namespace
{
void require(bool value, const std::string & message)
{
  if (!value) {throw std::runtime_error(message);}
}
Eigen::Vector3d point(const Pose & p) {return {p.position.x, p.position.y, p.position.z};}
Eigen::Quaterniond rotation(const Pose & p)
{
  return {p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z};
}
class RightTablePick
{
public:
  explicit RightTablePick(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    poses_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_box/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (stamp <= stamp_) {return;}
        stamp_ = stamp;box_ = msg->pose;box_.position.z -= 0.05;
        box_received_ = Wall::now();++sequence_;
      });
    joints_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          for (size_t j = 0; j < 2; ++j) {
            if (msg->name[i] == "right_fr3v2_finger_joint" + std::to_string(j + 1)) {
              fingers_[j] = msg->position[i];finger_received_[j] = Wall::now();
            }
          }
        }
      });
    contacts_ = node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/placement/box_contacts", rclcpp::SensorDataQoS(),
      [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & c : msg->contacts) {
          const auto & a = c.collision1.name;const auto & b = c.collision2.name;
          bool finger = false;
          const std::array<std::string, 2> names =
          {"right_fr3v2_leftfinger", "right_fr3v2_rightfinger"};
          for (size_t i = 0; i < 2; ++i) {
            if (a.find(names[i]) != std::string::npos || b.find(names[i]) != std::string::npos) {
              finger = true;grasp_received_[i] = Wall::now();
            }
          }
          const bool table = a.find("table_top_collision") != std::string::npos ||
          b.find("table_top_collision") != std::string::npos;
          const bool board = a.find("shelf_2_level_2_collision") != std::string::npos ||
          b.find("shelf_2_level_2_collision") != std::string::npos;
          bool vertical = false;double depth = 0;
          for (const auto & n : c.normals) {vertical = vertical || std::abs(n.z) > 0.8;}
          for (double d : c.depths) {depth = std::max(depth, d);}
          const bool support = board && vertical && !c.depths.empty() && depth < 0.001;
          if (support) {shelf_box_wall_ = Wall::now();shelf_sim_ = node_->now().nanoseconds();}
          if (carrying_ && !finger && !(seating_ && support)) {
            if (!unsafe_) {
              RCLCPP_ERROR(node_->get_logger(), "UNEXPECTED CARRY CONTACT: %s <-> %s", a.c_str(),
                b.c_str());
            }
            unsafe_ = true;
          }
          if (!finger && !table && (a.find("fr3v2") != std::string::npos ||
          b.find("fr3v2") != std::string::npos))
          {
            unsafe_ = true;
            RCLCPP_ERROR(node_->get_logger(), "UNEXPECTED BOX CONTACT: %s <-> %s", a.c_str(),
              b.c_str());
          }
        }
      });
    table_contacts_ = node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/placement/table_contacts", rclcpp::SensorDataQoS(),
      [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & c : msg->contacts) {
          const auto & a = c.collision1.name;const auto & b = c.collision2.name;
          if (a.find("target_box") != std::string::npos ||
          b.find("target_box") != std::string::npos)
          {
            support_received_ = Wall::now();support_sim_ = node_->now().nanoseconds();
          }
          if (a.find("fr3v2") != std::string::npos || b.find("fr3v2") != std::string::npos) {
            unsafe_ = true;
            RCLCPP_ERROR(node_->get_logger(), "ROBOT TABLE CONTACT: %s <-> %s", a.c_str(),
              b.c_str());
          }
        }
      });
    shelf_contacts_ = node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/placement/shelf2_contacts", rclcpp::SensorDataQoS(),
      [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & c : msg->contacts) {
          const auto & a = c.collision1.name;const auto & b = c.collision2.name;
          if (monitor_shelf_ && (a.find("fr3v2") != std::string::npos ||
          b.find("fr3v2") != std::string::npos))
          {
            if (!unsafe_) {
              RCLCPP_ERROR(node_->get_logger(), "ROBOT SHELF CONTACT: %s <-> %s", a.c_str(),
                b.c_str());
            }
            unsafe_ = true;
          }
          const bool box = a.find("target_box") != std::string::npos ||
          b.find("target_box") != std::string::npos;
          const bool board = a.find("shelf_2_level_2_collision") != std::string::npos ||
          b.find("shelf_2_level_2_collision") != std::string::npos;
          bool vertical = false;double depth = 0;
          for (const auto & n : c.normals) {vertical = vertical || std::abs(n.z) > 0.8;}
          for (double d : c.depths) {depth = std::max(depth, d);}
          if (box && board && vertical && !c.depths.empty() && depth < 0.001) {
            shelf_surface_wall_ = Wall::now();
          }
        }
      });
    grip_ = rclcpp_action::create_client<Grip>(node_, "/right_gripper_controller/gripper_cmd");
  }
  void run()
  {
    std::string stage;node_->get_parameter_or<std::string>("stage", stage, "pick");
    const bool left_placed = node_->get_parameter_or<bool>("left_placed", false);
    require(stage == "pick" || stage == "shelf_hold" || stage == "shelf_place" ||
      stage == "left_park",
        "Invalid right-arm stage");
    {std::lock_guard<std::mutex> lock(mutex_);monitor_shelf_ = stage == "shelf_place";}
    auto controllers = node_->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
    require(controllers->wait_for_service(90s), "Controller manager unavailable");
    bool ready = false;const auto end = Wall::now() + 90s;
    while (Wall::now() < end) {
      auto future = controllers->async_send_request(
        std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
      if (future.wait_for(3s) == std::future_status::ready) {
        const auto response = future.get();int count = 0;
        for (const auto & c : response->controller) {
          if (c.state == "active" && (c.name == "arm_1_controller" ||
            c.name == "arm_2_controller" || c.name == "right_gripper_controller")) {++count;}
        }
        if (count == 3) {ready = true;break;}
      }
      std::this_thread::sleep_for(500ms);
    }
    require(ready && grip_->wait_for_action_server(20s), "Right arm/gripper not ready");
    arm_ = std::make_unique<Group>(node_, "arm_2");
    Group left(node_, "arm_1");
    arm_->setEndEffectorLink(tcp_);arm_->setPoseReferenceFrame("world");
    arm_->setPlanningTime(10);arm_->setNumPlanningAttempts(3);
    arm_->setGoalPositionTolerance(0.0005);arm_->setGoalOrientationTolerance(0.001);
    arm_->setMaxVelocityScalingFactor(0.25);arm_->setMaxAccelerationScalingFactor(0.25);
    require(arm_->startStateMonitor(15) && left.startStateMonitor(15), "Robot state unavailable");
    scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    require(scene_->getAttachedObjects({"target_box"}).empty(),
        "Box still attached; restart simulation");
    const auto table_box = stableBox();
    require((point(table_box) - Eigen::Vector3d(0, 0.05, 0.42)).norm() < 0.006,
      "Box not resting at the table pickup location");
    waitTableSupport();addBox(table_box);configureContacts(false);
    RCLCPP_INFO(node_->get_logger(), "LEFT PARK START: box collisions enabled for left fingers");
    if (left_placed) {
      parkLeftAfterPlacement(left);
    } else {
      park(left);
    }
    waitTableSupport();
    require((point(freshBox()) - point(table_box)).norm() < 0.001,
      "Box moved while parking the left arm");
    RCLCPP_INFO(node_->get_logger(), "LEFT PARK PASSED");
    if (stage == "left_park") {return;}
    gripper(0.04, false);
    park(*arm_);
    const auto initial = stableBox();
    require((point(initial) - point(table_box)).norm() < 0.001, "Box moved while parking arms");
    safe();addBox(initial);
    Pose pick = initial;const double half = std::sqrt(0.5);
    pick.orientation.x = half;pick.orientation.y = half;
    pick.orientation.z = 0;pick.orientation.w = 0;
    Pose pre = pick;pre.position.z += 0.12;
    const auto approach = planPickupApproach(pre, pick);
    require(arm_->execute(approach) == moveit::core::MoveItErrorCode::SUCCESS,
      "Right pickup approach execution failed");
    waitAt(&pre, 0.001, 0.01, true);
    configureContacts(true);
    linear(pick, 0.1, LinearMode::pickup);waitAt(&pick, 0.0005, 0.005);safe();waitTableSupport();
    RCLCPP_INFO(node_->get_logger(), "RIGHT PICK TARGET SETTLED: closing right gripper");
    gripper(0.018, true);
    const auto grasped = stableBox();const auto tcp = arm_->getCurrentPose(tcp_).pose;
    const Eigen::Vector3d reference = rotation(tcp).inverse() * (point(grasped) - point(tcp));
    const Eigen::Quaterniond reference_angle = rotation(tcp).inverse() * rotation(grasped);
    addBox(grasped);
    arm_->attachObject("target_box", tcp_,
      {"right_fr3v2_leftfinger", "right_fr3v2_rightfinger", "right_fr3v2_hand"});
    const auto attach_end = Wall::now() + 5s;
    while (scene_->getAttachedObjects({"target_box"}).empty() && Wall::now() < attach_end) {
      std::this_thread::sleep_for(100ms);
    }
    require(!scene_->getAttachedObjects({"target_box"}).empty(),
        "Right planning attachment failed");
    Pose lifted = pick;lifted.position.z += 0.08;
    linear(lifted, 0.1, LinearMode::pickup);waitAt(&lifted, 0.001, 0.01);safe();
    const auto lifted_box = stableBox();
    const double rise = lifted_box.position.z - initial.position.z;
    RCLCPP_INFO(node_->get_logger(), "RIGHT PHYSICAL LIFT: %.3f mm", rise * 1000);
    require(std::abs(rise - 0.08) < 0.005 &&
      (point(lifted_box).head<2>() - point(initial).head<2>()).norm() < 0.005,
      "Physical box did not follow right-arm lift");
    const auto sim_start = node_->now();const auto wall_start = Wall::now();
    double drift = 0, angle = 0;
    while ((node_->now() - sim_start).seconds() < 3) {
      require(Wall::now() - wall_start < 30s, "Right hold clock timed out");safe();
      const auto box = freshBox();const auto hand = arm_->getCurrentPose(tcp_).pose;
      drift = std::max(drift,
          (rotation(hand).inverse() * (point(box) - point(hand)) - reference).norm());
      angle = std::max(angle,
          reference_angle.angularDistance(rotation(hand).inverse() * rotation(box)));
      require(drift < 0.005 && angle < 0.0873, "Right grasp slipped or rotated");
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto received : grasp_received_) {
          require(Wall::now() - received < 1s, "Missing current physical right-finger contact");
        }
      }
      std::this_thread::sleep_for(100ms);
    }
    require(!onTable(), "Box still has fresh table contact after right lift");
    RCLCPP_INFO(node_->get_logger(),
      "RIGHT PICK TEST PASSED: lift=%.3f mm drift=%.3f mm angle=%.3f deg; right gripper stays closed",
      rise * 1000, drift * 1000, angle * 180 / M_PI);
    if (stage == "pick") {return;}
    moveit::core::RobotState rack_state(arm_->getRobotModel());
    rack_state.setToDefaultValues();
    rack_y_ = rack_state.getGlobalLinkTransform("shelf_2_link").translation().y();
    {std::lock_guard<std::mutex> lock(mutex_);carrying_ = true;}
    configureContacts(true, false);
    if (stage == "shelf_place") {
      raiseForShelfTransfer();
      holdAt("RIGHT CLEARANCE", reference, reference_angle, drift, angle);
      placeOnShelf(reference, reference_angle);
      return;
    }
    Pose high = arm_->getCurrentPose(tcp_).pose;
    high.position.z += 0.10;
    RCLCPP_INFO(node_->get_logger(), "RIGHT CLEARANCE START: 100 mm additional vertical lift");
    linear(high);waitAt(&high, 0.001, 0.01);
    holdAt("RIGHT CLEARANCE", reference, reference_angle, drift, angle);
    Pose shelf = high;
    shelf.orientation.x = -0.5;shelf.orientation.y = -0.5;
    shelf.orientation.z = -0.5;shelf.orientation.w = 0.5;
    const Eigen::Vector3d desired_center(0, rack_y_ - 0.28, 0.5625);
    const Eigen::Vector3d target = desired_center - rotation(shelf) * reference;
    shelf.position.x = target.x();shelf.position.y = target.y();shelf.position.z = target.z();
    arm_->setMaxVelocityScalingFactor(0.10);arm_->setMaxAccelerationScalingFactor(0.10);
    RCLCPP_INFO(node_->get_logger(),
        "RIGHT SHELF TRANSFER START: box target=(0.000, %.3f, 0.5625)", desired_center.y());
    arm_->setPoseTarget(shelf);executePlan();arm_->clearPoseTargets();waitAt(&shelf, 0.0015, 0.005);
    holdAt("RIGHT SHELF FRONT", reference, reference_angle, drift, angle);
    const auto shelf_box = freshBox();
    const double center_error = (point(shelf_box) - desired_center).norm();
    const double face_tilt = std::acos(std::clamp(
      rotation(shelf_box).toRotationMatrix().row(2).cwiseAbs().maxCoeff(), 0.0, 1.0));
    require(center_error < 0.01 && face_tilt < 0.0873, "Box not held level at shelf-front target");
    RCLCPP_INFO(node_->get_logger(),
      "RIGHT SHELF HOLD PASSED: center_error=%.3f mm drift=%.3f mm angle=%.3f deg tilt=%.3f deg; no release",
      center_error * 1000, drift * 1000, angle * 180 / M_PI, face_tilt * 180 / M_PI);

  }

private:
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

  double jointLimitMargin(
    const moveit::core::RobotState & state, const moveit::core::JointModelGroup * group)
  {
    double margin = std::numeric_limits<double>::infinity();
    for (const auto & name : group->getVariableNames()) {
      const auto & bounds = state.getRobotModel()->getVariableBounds(name);
      const double position = state.getVariablePosition(name);
      margin = std::min({margin, position - bounds.min_position_, bounds.max_position_ - position});
    }
    return margin;
  }

  void raiseForShelfTransfer()
  {
    safe();
    planning_scene::PlanningScene scene(arm_->getRobotModel());
    require(scene.setPlanningSceneMsg(planningSceneSnapshot()),
        "Clearance lift scene import failed");
    auto start = scene.getCurrentState();
    start.update();
    const auto * group = start.getJointModelGroup("arm_2");
    const auto & tcp = start.getGlobalLinkTransform(tcp_);
    const Eigen::Quaterniond orientation(tcp.rotation());
    Pose target;
    target.position.x = tcp.translation().x();
    target.position.y = tcp.translation().y();
    target.orientation.w = orientation.w();
    target.orientation.x = orientation.x();
    target.orientation.y = orientation.y();
    target.orientation.z = orientation.z();
    for (const double rise : {0.10, 0.08, 0.06, 0.04}) {
      target.position.z = tcp.translation().z() + rise;
      std::vector<moveit::core::RobotStatePtr> path;
      if (!checkedFinePath(scene, start, target, false, path)) {
        RCLCPP_INFO(node_->get_logger(), "RIGHT CLEARANCE CANDIDATE REJECTED: %.0f mm, path",
          rise * 1000);
        continue;
      }
      double margin = std::numeric_limits<double>::infinity();
      robot_trajectory::RobotTrajectory trajectory(arm_->getRobotModel(), group);
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
      if (!timing.computeTimeStamps(trajectory, 0.10, 0.10)) {continue;}
      bool valid = true;
      for (size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
        const auto & waypoint = trajectory.getWayPoint(i);
        margin = std::min(margin, jointLimitMargin(waypoint, group));
        if (margin < 0.05 || scene.isStateColliding(waypoint, "arm_2")) {
          valid = false;
          break;
        }
      }
      const auto & end = trajectory.getLastWayPoint().getGlobalLinkTransform(tcp_);
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
      require(arm_->execute(motion) == moveit::core::MoveItErrorCode::SUCCESS,
        "Clearance lift execution failed");
      waitAt(&target, 0.001, 0.01);
      auto actual = arm_->getCurrentState(2.0);
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
    const auto * group = state.getJointModelGroup("arm_2");
    double min_margin = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_bottom = std::numeric_limits<double>::infinity();
    for (const auto & waypoint : trajectory.points) {
      state.setVariablePositions(trajectory.joint_names, waypoint.positions);
      state.update();
      min_margin = std::min(min_margin, jointLimitMargin(state, group));
      const auto & tcp = state.getGlobalLinkTransform(tcp_);
      const Eigen::Vector3d center = tcp * relative;
      const Eigen::Matrix3d rotation = tcp.linear() * relative_angle.toRotationMatrix();
      max_y = std::max(max_y, center.y());
      min_bottom = std::min(min_bottom, center.z() - 0.02 * rotation.row(2).cwiseAbs().sum());
      if (!state.satisfiesBounds(group) || scene.isStateColliding(state, "arm_2")) {
        return false;
      }
    }
    const bool accepted = min_margin >= 0.05 && max_y <= y_limit && min_bottom >= 0.48;
    RCLCPP_INFO(node_->get_logger(),
      "SHELF TRANSFER %s: phase=%s joint margin=%.4f rad max_box_y=%.4f m min_box_bottom=%.4f m",
      accepted ? "PLAN" : "REJECTED", phase, min_margin, max_y, min_bottom);
    return accepted;
  }

  Eigen::Quaterniond chooseShelfApproach(
    const Eigen::Vector3d & relative,
    const Eigen::Quaterniond & relative_angle, ShelfTransfer & transfer)
  {
    auto scene = std::make_shared<planning_scene::PlanningScene>(arm_->getRobotModel());
    require(scene->setPlanningSceneMsg(planningSceneSnapshot()), "Shelf probe scene import failed");
    auto seating_scene = scene->diff();
    seating_scene->getAllowedCollisionMatrixNonConst().setEntry("target_box", "shelf_2_link", true);
    const auto * group = scene->getRobotModel()->getJointModelGroup("arm_2");
    const auto valid = [this, &seating_scene](moveit::core::RobotState * state,
      const moveit::core::JointModelGroup * joints, const double * values) {
        state->setJointGroupPositions(joints, values);
        state->update();
        return jointLimitMargin(*state, joints) >= 0.05 &&
               !seating_scene->isStateColliding(*state, "arm_2");
      };
    const double half = std::sqrt(0.5);
    const std::array<Eigen::Quaterniond, 3> nominal = {
      Eigen::Quaterniond(0.5, -0.5, -0.5, -0.5),
      Eigen::Quaterniond(0.5, -0.5, 0.5, 0.5), Eigen::Quaterniond(0, half, half, 0)};
    const auto deadline = Wall::now() + 35s;
    for (size_t variant = 0; variant < nominal.size(); ++variant) {
      const Eigen::Quaterniond q = nearestCubeRotation(nominal[variant] * relative_angle) *
        relative_angle.inverse();
      Eigen::Isometry3d end = Eigen::Isometry3d::Identity();
      end.linear() = q.toRotationMatrix();
      end.translation() = Eigen::Vector3d(0, rack_y_ - 0.16, 0.4827) - q * relative;
      Eigen::Isometry3d above = end;
      above.translation().z() += 0.0798;
      Eigen::Isometry3d front = above;
      front.translation().y() -= 0.12;
      for (int attempt = 0; attempt < 16 && Wall::now() < deadline; ++attempt) {
        auto state = scene->getCurrentState();
        if (attempt > 0) {state.setToRandomPositions(group);}
        if (!state.setFromIK(group, end, tcp_, 0.08, valid)) {continue;}
        std::vector<moveit::core::RobotStatePtr> reverse;
        const EigenSTL::vector_Isometry3d waypoints = {above, front};
        const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
          &state, group, reverse, state.getLinkModel(tcp_), waypoints, true,
          moveit::core::MaxEEFStep(0.002), moveit::core::CartesianPrecision{}, valid,
          kinematics::KinematicsQueryOptions());
        if (fraction < 0.999 || reverse.empty()) {continue;}
        for (double x : {0.0, 0.10, -0.10}) {
          auto midpoint = front;
          midpoint.translation() = Eigen::Vector3d(x, rack_y_ - 0.48, 0.59) - q * relative;
          if (planShelfTransfer(*scene, *reverse.back(), midpoint, relative, relative_angle,
              transfer))
          {
            RCLCPP_INFO(node_->get_logger(),
              "SHELF TWO-STAGE PLAN PASSED: variant=%zu midpoint_box=(%.3f, %.3f, 0.590)",
              variant, x, rack_y_ - 0.48);
            return q;
          }
          if (Wall::now() >= deadline) {break;}
        }
      }
    }
    throw std::runtime_error("No collision-checked two-stage shelf placement approach found");
  }

  bool planShelfTransfer(
    const planning_scene::PlanningScene & scene, moveit::core::RobotState front,
    const Eigen::Isometry3d & midpoint, const Eigen::Vector3d & relative,
    const Eigen::Quaterniond & relative_angle, ShelfTransfer & transfer)
  {
    const auto * group = front.getJointModelGroup("arm_2");
    std::vector<moveit::core::RobotStatePtr> path;
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &front, group, path, front.getLinkModel(tcp_), midpoint, true,
      moveit::core::MaxEEFStep(0.002), moveit::core::CartesianPrecision{},
      collisionCheck(scene, false), kinematics::KinematicsQueryOptions());
    if (fraction < 0.999 || path.size() < 2) {return false;}
    robot_trajectory::RobotTrajectory advance(arm_->getRobotModel(), group);
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
    if (!timing.computeTimeStamps(advance, 0.06, 0.06)) {return false;}
    advance.getRobotTrajectoryMsg(transfer.advance);
    if (!checkShelfTransfer(transfer.advance, scene, relative, relative_angle,
        rack_y_ - 0.25, "advance"))
    {
      return false;
    }
    std::vector<double> joints;
    path.back()->copyJointGroupPositions(group, joints);
    if (!arm_->setJointValueTarget(joints)) {return false;}
    for (int route = 0; route < 3; ++route) {
      arm_->setStartStateToCurrentState();
      if (arm_->plan(transfer.turn) != moveit::core::MoveItErrorCode::SUCCESS) {continue;}
      if (!checkShelfTransfer(transfer.turn.trajectory, scene, relative, relative_angle,
          rack_y_ - 0.40, "turn"))
      {
        continue;
      }
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
    return false;
  }
  bool onShelf()
  {
    safe();std::lock_guard<std::mutex> lock(mutex_);
    return Wall::now() - shelf_box_wall_ < 1s && Wall::now() - shelf_surface_wall_ < 1s &&
           node_->now().nanoseconds() - shelf_sim_ < 250000000;
  }
  void waitShelfSupport()
  {
    const auto end = Wall::now() + 15s;int64_t since = -1;
    while (Wall::now() < end) {
      const auto box = freshBox();const auto now = node_->now().nanoseconds();
      const bool good = onShelf() && std::abs(bottom(box) - 0.4625) < 0.001 && tilt(box) < 0.00873;
      if (!good) {since = -1;} else if (since < 0) {
        since = now;
      } else if (now - since >= 1000000000) {
        RCLCPP_INFO(node_->get_logger(), "SHELF SUPPORT VERIFIED: 1 sim s bottom_gap=%.3f mm",
          (bottom(box) - 0.4625) * 1000);return;
      }
      std::this_thread::sleep_for(100ms);
    }
    throw std::runtime_error("Target shelf support/level not verified; refusing opening");
  }
  void executeShelfTransfer(
    const moveit_msgs::msg::RobotTrajectory & plan, const Eigen::Vector3d & relative,
    const Eigen::Quaterniond & relative_angle)
  {
    safe();
    auto execution = std::async(std::launch::async, [this, &plan]() {return arm_->execute(plan);});
    double min_bottom = std::numeric_limits<double>::infinity();
    double max_drift = 0.0;
    int consecutive_drift = 0;
    try {
      while (execution.wait_for(100ms) == std::future_status::timeout) {
        safe();
        const auto box = freshBox();
        const auto hand = arm_->getCurrentPose(tcp_).pose;
        const auto hand_inverse = rotation(hand).inverse();
        const double drift = (hand_inverse * (point(box) - point(hand)) - relative).norm();
        const double angle = relative_angle.angularDistance(hand_inverse * rotation(box));
        min_bottom = std::min(min_bottom, bottom(box));
        max_drift = std::max(max_drift, drift);
        require(min_bottom > 0.445, "Box descended below physical transfer clearance");
        consecutive_drift = (drift > 0.008 || angle > 0.1222) ? consecutive_drift + 1 : 0;
        require(consecutive_drift < 2, "Right grasp moved during shelf transfer");
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto received : grasp_received_) {
          require(Wall::now() - received < 1s, "Finger contact lost during shelf transfer");
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_->get_logger(), "SHELF TRANSFER STOP REQUESTED: %s", error.what());
      arm_->stop();
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
    auto box = stableBox();auto hand = arm_->getCurrentPose(tcp_).pose;
    syncHeldBox(box, hand);
    Eigen::Vector3d relative = rotation(hand).inverse() * (point(box) - point(hand));
    Eigen::Quaterniond relative_angle = rotation(hand).inverse() * rotation(box);
    arm_->setGoalJointTolerance(0.00005);
    arm_->setGoalPositionTolerance(0.00015);arm_->setGoalOrientationTolerance(0.0005);
    arm_->setMaxVelocityScalingFactor(0.06);arm_->setMaxAccelerationScalingFactor(0.06);
    arm_->setPlanningTime(2.0);
    ShelfTransfer transfer;
    Eigen::Quaterniond aligned = chooseShelfApproach(relative, relative_angle, transfer);
    RCLCPP_INFO(node_->get_logger(), "SHELF TRANSFER TURN START");
    executeShelfTransfer(transfer.turn.trajectory, relative, relative_angle);
    waitAt(&transfer.midpoint, 0.001, 0.005);
    double middle_drift = 0, middle_angle = 0;
    holdAt("SHELF TRANSFER MIDPOINT", reference, reference_angle, middle_drift, middle_angle);
    RCLCPP_INFO(node_->get_logger(), "SHELF TRANSFER ADVANCE START");
    Pose goal = hand;bool level = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      if (attempt > 0) {
        box = freshBox();hand = arm_->getCurrentPose(tcp_).pose;
        relative = rotation(hand).inverse() * (point(box) - point(hand));
        relative_angle = rotation(hand).inverse() * rotation(box);
        aligned = nearestCubeRotation(rotation(box)) * relative_angle.inverse();
        syncHeldBox(box, hand);
      }
      goal = hand;goal.orientation.w = aligned.w();goal.orientation.x = aligned.x();
      goal.orientation.y = aligned.y();goal.orientation.z = aligned.z();
      const Eigen::Vector3d target = Eigen::Vector3d(0, rack_y_ - 0.28,
          0.5625) - aligned * relative;
      goal.position.x = target.x();goal.position.y = target.y();goal.position.z = target.z();
      RCLCPP_INFO(node_->get_logger(), "SHELF ALIGN START: correction %d/3", attempt + 1);
      if (attempt == 0) {
        executeShelfTransfer(transfer.advance, relative, relative_angle);
      } else {
        linear(goal, 0.01);
      }
      waitAt(&goal, 0.0003, 0.0015);box = stableBox();safe();
      if (tilt(box) < 0.00873 && (point(box) - Eigen::Vector3d(0, rack_y_ - 0.28,
          0.5625)).norm() < 0.001)
      {
        level = true;break;
      }
    }
    require(level, "Box not aligned at shelf front");
    double drift = 0, angle = 0;holdAt("SHELF ALIGNED", reference, reference_angle, drift, angle);
    box = freshBox();hand = arm_->getCurrentPose(tcp_).pose;syncHeldBox(box, hand);
    relative = rotation(hand).inverse() * (point(box) - point(hand));
    Eigen::Vector3d target = Eigen::Vector3d(0, rack_y_ - 0.16, 0.5625) - aligned * relative;
    goal.position.x = target.x();goal.position.y = target.y();goal.position.z = target.z();
    RCLCPP_INFO(node_->get_logger(), "SHELF INSERT START: 120 mm into second shelf");
    linear(goal, 0.03);waitAt(&goal, 0.0003, 0.0015);
    box = freshBox();hand = arm_->getCurrentPose(tcp_).pose;syncHeldBox(box, hand);
    goal.position.x = hand.position.x - box.position.x;
    goal.position.y = hand.position.y + rack_y_ - 0.16 - box.position.y;
    goal.position.z = hand.position.z + 0.4845 - box.position.z;
    RCLCPP_INFO(node_->get_logger(), "SHELF DESCENT START: center z=0.4845");
    linear(goal, 0.02, LinearMode::shelf_descent);waitAt(&goal, 0.0003, 0.0015);
    {std::lock_guard<std::mutex> lock(mutex_);seating_ = true;}
    configureContacts(true, false, true);
    const auto before_relax = freshBox();gripper(0.0198, true);
    box = stableBox();hand = arm_->getCurrentPose(tcp_).pose;
    RCLCPP_INFO(node_->get_logger(), "SHELF GRIP RELAX: shift=%.3f mm gap=%.3f mm",
      (point(box) - point(before_relax)).norm() * 1000, (bottom(box) - 0.4625) * 1000);
    require((point(box) - point(before_relax)).norm() < 0.003 &&
      bottom(box) - 0.4625 > -0.0003, "Excessive shelf seating displacement");
    syncHeldBox(box, hand);bool seated = false;
    for (int step = 0; step < 10; ++step) {
      box = freshBox();const double gap = bottom(box) - 0.4625;
      RCLCPP_INFO(node_->get_logger(), "SHELF SEAT step=%d gap=%.3f mm contact=%d", step,
          gap * 1000, onShelf());
      require(gap > -0.0003 && tilt(box) < 0.00873, "Shelf seating penetration/tilt too large");
      if (onShelf() && std::abs(gap) < 0.001) {seated = true;break;}
      if (gap <= 0.0) {waitShelfSupport();seated = true;break;}
      goal = arm_->getCurrentPose(tcp_).pose;
      goal.orientation.w = aligned.w();goal.orientation.x = aligned.x();
      goal.orientation.y = aligned.y();goal.orientation.z = aligned.z();
      goal.position.z -= std::min(0.0005, gap + 0.0001);
      linear(goal, 0.01);waitAt(&goal, 0.00015, 0.0015);
    }
    require(seated, "Shelf support not established in bounded steps");waitShelfSupport();
    const auto release_reference = freshBox();
    RCLCPP_INFO(node_->get_logger(), "SHELF OPEN: verified target board, two-step opening");
    gripper(0.021, false);waitShelfSupport();
    double release_slip = (point(freshBox()) - point(release_reference)).norm();
    gripper(0.04, false);arm_->detachObject("target_box");
    const auto detach_end = Wall::now() + 5s;
    while (!scene_->getAttachedObjects({"target_box"}).empty() && Wall::now() < detach_end) {
      std::this_thread::sleep_for(100ms);
    }
    require(scene_->getAttachedObjects({"target_box"}).empty(), "Shelf planning detach failed");
    box = stableBox();
    release_slip = std::max(release_slip, (point(box) - point(release_reference)).norm());
    addBox(box);goal = arm_->getCurrentPose(tcp_).pose;goal.position.z += 0.08;
    RCLCPP_INFO(node_->get_logger(), "SHELF RETREAT CLEARANCE: raise 80 mm with open gripper");
    linear(goal, 0.02);waitAt(&goal, 0.001, 0.005);
    require(onShelf(), "Box lost shelf support during clearance lift");
    goal = arm_->getCurrentPose(tcp_).pose;goal.position.y -= 0.12;
    RCLCPP_INFO(node_->get_logger(),
        "SHELF RETREAT START: 120 mm at clearance height, no ready return");
    linear(goal, 0.03);waitAt(&goal);
    const auto sim_start = node_->now();const auto wall_start = Wall::now();
    while ((node_->now() - sim_start).seconds() < 3.0) {
      require(Wall::now() - wall_start < 30s, "Shelf post-release clock timed out");
      box = freshBox();safe();
      release_slip = std::max(release_slip, (point(box) - point(release_reference)).norm());
      require(onShelf() && release_slip < 0.002 && std::hypot(box.position.x,
          box.position.y - (rack_y_ - 0.16)) < 0.003 &&
        std::abs(box.position.z - 0.4825) < 0.001 && tilt(box) < 0.01745,
        "Shelf box shifted, tilted or lost target support after release");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(),
      "RIGHT SHELF PLACE PASSED: box=(%.5f, %.5f, %.5f) xy_error=%.3f mm release_slip=%.3f mm tilt=%.3f deg",
      box.position.x, box.position.y, box.position.z,
        std::hypot(box.position.x, box.position.y - (rack_y_ - 0.16)) * 1000,
      release_slip * 1000, tilt(box) * 180 / M_PI);
  }

  void holdAt(
    const char * label, const Eigen::Vector3d & reference,
    const Eigen::Quaterniond & reference_angle, double & drift, double & angle)
  {
    const auto sim_start = node_->now();const auto wall_start = Wall::now();
    while ((node_->now() - sim_start).seconds() < 3.0) {
      require(Wall::now() - wall_start < 30s, "Right carry hold clock timed out");safe();
      const auto box = freshBox();const auto hand = arm_->getCurrentPose(tcp_).pose;
      drift = std::max(drift,
          (rotation(hand).inverse() * (point(box) - point(hand)) - reference).norm());
      angle = std::max(angle,
          reference_angle.angularDistance(rotation(hand).inverse() * rotation(box)));
      require(drift < 0.005 && angle < 0.0873,
          std::string(label) + ": right grasp slipped or rotated");
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto received : grasp_received_) {
          require(Wall::now() - received < 1s, "Missing physical finger contact during carry hold");
        }
      }
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(), "%s HOLD PASSED: drift=%.3f mm angle=%.3f deg", label,
      drift * 1000, angle * 180 / M_PI);
  }

  void syncHeldBox(const Pose & box, const Pose & tcp)
  {
    const Eigen::Vector3d relative = rotation(tcp).inverse() * (point(box) - point(tcp));
    const Eigen::Quaterniond q = rotation(tcp).inverse() * rotation(box);
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = tcp_;attached.touch_links =
    {"right_fr3v2_leftfinger", "right_fr3v2_rightfinger", "right_fr3v2_hand"};
    attached.object.id = "target_box";attached.object.header.frame_id = tcp_;
    attached.object.operation = attached.object.ADD;
    shape_msgs::msg::SolidPrimitive primitive;primitive.type = primitive.BOX;
    primitive.dimensions = {0.04, 0.04, 0.04};
    Pose p;p.position.x = relative.x();p.position.y = relative.y();p.position.z = relative.z();
    p.orientation.w = q.w();p.orientation.x = q.x();p.orientation.y = q.y();p.orientation.z = q.z();
    attached.object.primitives.push_back(primitive);attached.object.primitive_poses.push_back(p);
    require(scene_->applyAttachedCollisionObject(attached), "Cannot synchronize held planning box");
  }
  double tilt(const Pose & box)
  {
    return std::acos(std::clamp(rotation(box).toRotationMatrix().row(2).cwiseAbs().maxCoeff(),
      0.0, 1.0));
  }
  double bottom(const Pose & box)
  {
    return box.position.z - 0.02 * rotation(box).toRotationMatrix().row(2).cwiseAbs().sum();
  }
  void safe()
  {
    std::lock_guard<std::mutex> lock(mutex_);require(!unsafe_, "Unexpected physical contact");
  }
  bool onTable()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return Wall::now() - support_received_ < 1s &&
           node_->now().nanoseconds() - support_sim_ < 250000000;
  }
  void waitTableSupport()
  {
    const auto end = Wall::now() + 10s;int64_t since = -1;
    while (Wall::now() < end) {
      safe();const auto box = freshBox();const auto now = node_->now().nanoseconds();
      if (!onTable() || std::abs(box.position.z - 0.42) > 0.001) {since = -1;} else if (since < 0) {
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
    group.setMaxVelocityScalingFactor(0.25);
    group.setMaxAccelerationScalingFactor(0.25);
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
      safe();std::this_thread::sleep_for(200ms);
      auto state = group.getCurrentState(2.0);
      require(static_cast<bool>(state), "Parking state stale");
      std::vector<double> current;state->copyJointGroupPositions(group.getName(), current);
      require(current.size() == 7 && previous.size() == 7, "Parking state incomplete");
      double delta = 0, error = 0;
      for (size_t i = 0; i < 7; ++i) {
        delta = std::max(delta, std::abs(current[i] - previous[i]));
        error = std::max(error, std::abs(current[i] - target[i]));
      }
      calm = delta < 0.0002 && error < 0.001 ? calm + 1 : 0;previous = current;
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
    safe();
    const auto start = parkingStart(left);
    planning_scene::PlanningScene scene(left.getRobotModel());
    require(scene.setPlanningSceneMsg(planningSceneSnapshot()), "Left parking scene import failed");
    const auto * group = start->getJointModelGroup("arm_1");
    const auto * tcp = start->getLinkModel("left_fr3v2_hand_tcp");
    Eigen::Isometry3d target = start->getGlobalLinkTransform(tcp);
    target.translation().z() += 0.08;
    auto state = *start;
    std::vector<moveit::core::RobotStatePtr> path;
    const auto valid = [&scene](moveit::core::RobotState * sample,
      const moveit::core::JointModelGroup * joints, const double * values) {
        sample->setJointGroupPositions(joints, values);
        sample->update();
        return sample->satisfiesBounds(joints) && !scene.isStateColliding(*sample, "arm_1");
      };
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &state, group, path, tcp, target, true, moveit::core::MaxEEFStep(0.001),
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
    require(timing.computeTimeStamps(lift, 0.08, 0.08), "Left parking lift timing failed");
    for (size_t i = 0; i < lift.getWayPointCount(); ++i) {
      require(!scene.isStateColliding(lift.getWayPoint(i), "arm_1"),
        "Timed left parking lift is not collision-free");
    }
    const auto & raised = lift.getLastWayPoint();
    const auto & endpoint = raised.getGlobalLinkTransform(tcp);
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
  void configureContacts(
    bool allow_right, bool allow_table_support = true,
    bool allow_shelf_support = false)
  {
    auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    require(client->wait_for_service(5s), "Planning scene unavailable");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = client->async_send_request(request);
    require(future.wait_for(5s) == std::future_status::ready, "Collision matrix timeout");
    auto acm = future.get()->scene.allowed_collision_matrix;
    const std::vector<std::string> names = {"target_box", "table_link", "shelf_2_link",
      "left_fr3v2_leftfinger",
      "left_fr3v2_rightfinger", "left_fr3v2_hand", "right_fr3v2_leftfinger",
      "right_fr3v2_rightfinger", "right_fr3v2_hand"};
    for (const auto & name : names) {
      if (std::find(acm.entry_names.begin(), acm.entry_names.end(),
          name) == acm.entry_names.end())
      {
        acm.entry_names.push_back(name);
        for (auto & row : acm.entry_values) {row.enabled.push_back(false);}
        moveit_msgs::msg::AllowedCollisionEntry row;
        row.enabled.resize(acm.entry_names.size(), false);
        acm.entry_values.push_back(row);
      }
    }
    const auto box = std::find(acm.entry_names.begin(), acm.entry_names.end(),
        "target_box") - acm.entry_names.begin();
    for (const auto & name : names) {
      if (name == "target_box") {continue;}
      const auto index = std::find(acm.entry_names.begin(), acm.entry_names.end(),
          name) - acm.entry_names.begin();
      const bool enabled = (allow_table_support && name == "table_link") ||
        (allow_shelf_support && name == "shelf_2_link") || (allow_right &&
        (name == "right_fr3v2_leftfinger" || name == "right_fr3v2_rightfinger"));
      acm.entry_values[box].enabled[index] = enabled;acm.entry_values[index].enabled[box] = enabled;
    }
    moveit_msgs::msg::PlanningScene diff;diff.is_diff = true;diff.allowed_collision_matrix = acm;
    require(scene_->applyPlanningScene(diff), "Right pickup collision setup failed");
  }
  moveit_msgs::msg::PlanningScene planningSceneSnapshot()
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
  double wristMargin(const moveit::core::RobotState & state)
  {
    const std::string wrist = "right_fr3v2_joint7";
    const auto & bounds = state.getRobotModel()->getVariableBounds(wrist);
    const double angle = state.getVariablePosition(wrist);
    return std::min(angle - bounds.min_position_, bounds.max_position_ - angle);
  }
  static constexpr double pickup_joint_margin_ = 0.05;
  moveit::core::GroupStateValidityCallbackFn collisionCheck(
    const planning_scene::PlanningScene & scene, bool pickup)
  {
    return [this, &scene, pickup](moveit::core::RobotState * candidate,
           const moveit::core::JointModelGroup * joints, const double * values) {
             candidate->setJointGroupPositions(joints, values);
             candidate->update();
             return candidate->satisfiesBounds(joints) &&
                    (!pickup || jointLimitMargin(*candidate, joints) >= pickup_joint_margin_) &&
                    !scene.isStateColliding(*candidate, "arm_2");
           };
  }
  bool checkedFinePath(
    const planning_scene::PlanningScene & scene, moveit::core::RobotState state,
    const Pose & target, bool pickup, std::vector<moveit::core::RobotStatePtr> & path)
  {
    const auto * group = state.getJointModelGroup("arm_2");
    if (pickup && jointLimitMargin(state, group) < pickup_joint_margin_) {return false;}
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = point(target);
    transform.linear() = rotation(target).toRotationMatrix();
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &state, group, path, state.getLinkModel(tcp_), transform, true,
      moveit::core::MaxEEFStep(0.0001), moveit::core::CartesianPrecision{0.00001, 0.0001, 0.00001},
      collisionCheck(scene, pickup), kinematics::KinematicsQueryOptions());
    if (fraction < 0.999 || path.size() < 2) {return false;}
    for (size_t i = 1; i < path.size(); ++i) {
      for (const auto & name : group->getVariableNames()) {
        if (std::abs(path[i]->getVariablePosition(name) -
          path[i - 1]->getVariablePosition(name)) >= 0.02)
        {
          return false;
        }
      }
    }
    return true;
  }
  Group::Plan planPickupApproach(const Pose & pre, const Pose & pick)
  {
    planning_scene::PlanningScene scene(arm_->getRobotModel());
    require(scene.setPlanningSceneMsg(planningSceneSnapshot()), "Pickup probe scene import failed");
    auto & acm = scene.getAllowedCollisionMatrixNonConst();
    acm.setEntry("target_box", "right_fr3v2_leftfinger", true);
    acm.setEntry("target_box", "right_fr3v2_rightfinger", true);
    const auto * group = scene.getRobotModel()->getJointModelGroup("arm_2");
    const auto valid = collisionCheck(scene, true);
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = point(pre);
    target.linear() = rotation(pre).toRotationMatrix();
    for (int attempt = 0; attempt < 12; ++attempt) {
      auto state = scene.getCurrentState();
      if (attempt > 0) {state.setToRandomPositions(group);}
      if (!state.setFromIK(group, target, tcp_, 0.08, valid)) {continue;}
      std::vector<moveit::core::RobotStatePtr> path;
      if (!checkedFinePath(scene, state, pick, true, path)) {continue;}
      std::vector<double> joints;
      state.copyJointGroupPositions(group, joints);
      if (!arm_->setJointValueTarget(joints)) {continue;}
      arm_->setStartStateToCurrentState();
      Group::Plan plan;
      if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {continue;}
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
  void finePath(const Pose & target, robot_trajectory::RobotTrajectory & trajectory, bool pickup)
  {
    planning_scene::PlanningScene scene(arm_->getRobotModel());
    require(scene.setPlanningSceneMsg(planningSceneSnapshot()),
        "Fine planning scene import failed");
    std::vector<moveit::core::RobotStatePtr> path;
    require(checkedFinePath(scene, scene.getCurrentState(), target, pickup, path),
      "Incomplete collision-checked fine path, joint margin or jump");
    for (const auto & waypoint : path) {trajectory.addSuffixWayPoint(*waypoint, 0.0);}
  }
  robot_trajectory::RobotTrajectory timeShelfDescent(
    const robot_trajectory::RobotTrajectory & raw, double scale)
  {
    planning_scene::PlanningScene scene(arm_->getRobotModel());
    require(scene.setPlanningSceneMsg(planningSceneSnapshot()),
        "Timing fallback scene import failed");
    const auto * group = raw.getGroup();
    const auto & names = group->getVariableNames();
    auto waypoint = raw.getFirstWayPoint();
    const Eigen::Isometry3d start = waypoint.getGlobalLinkTransform(tcp_);
    const Eigen::Isometry3d end = raw.getLastWayPoint().getGlobalLinkTransform(tcp_);
    const Eigen::Vector3d direction = end.translation() - start.translation();
    require(direction.squaredNorm() > 0, "Timing fallback requires a nonzero descent");
    const Eigen::Quaterniond start_angle(start.rotation()), end_angle(end.rotation());
    for (const auto & name : names) {
      waypoint.setVariableVelocity(name, 0);
      waypoint.setVariableAcceleration(name, 0);
    }
    robot_trajectory::RobotTrajectory timed(arm_->getRobotModel(), group);
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
        const double velocity = std::min(-limits.min_velocity_, limits.max_velocity_) * scale;
        const double acceleration =
          std::min(-limits.min_acceleration_, limits.max_acceleration_) * scale;
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
        const auto & tcp = waypoint.getGlobalLinkTransform(tcp_);
        const double ratio = std::clamp(
          (tcp.translation() - start.translation()).dot(direction) / direction.squaredNorm(),
          0.0, 1.0);
        const Eigen::Vector3d expected = start.translation() + ratio * direction;
        require((tcp.translation() - expected).norm() < 0.00025 &&
          Eigen::Quaterniond(tcp.rotation()).angularDistance(start_angle.slerp(ratio, end_angle)) <
          0.0015, "Shelf timing fallback deviates from Cartesian descent");
        require(waypoint.satisfiesBounds(group) && !scene.isStateColliding(waypoint, "arm_2"),
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
  void linear(const Pose & target, double scale = 0.1, LinearMode mode = LinearMode::normal)
  {
    safe();arm_->setStartStateToCurrentState();moveit_msgs::msg::RobotTrajectory msg;
    auto state = arm_->getCurrentState(2.0);
    require(static_cast<bool>(state), "No trajectory state");
    robot_trajectory::RobotTrajectory trajectory(arm_->getRobotModel(), "arm_2");
    const bool pickup = mode == LinearMode::pickup;
    const bool fine = pickup || scale <= 0.02;
    if (fine) {finePath(target, trajectory, pickup);} else {
      const double fraction = arm_->computeCartesianPath({target}, 0.002, msg, true);
      if (fraction < 0.999) {
        moveit_msgs::msg::RobotTrajectory probe;
        const double unchecked = arm_->computeCartesianPath({target}, 0.002, probe, false);
        RCLCPP_ERROR(node_->get_logger(),
          "CARTESIAN DIAG: checked=%.3f unchecked=%.3f (probe not executed)", fraction, unchecked);
        throw std::runtime_error("Incomplete collision-checked Cartesian path");
      }
      trajectory.setRobotTrajectoryMsg(*state, msg);
    }
    if (fine) {
      const auto raw = trajectory.getLastWayPoint().getGlobalLinkTransform(tcp_);
      RCLCPP_INFO(node_->get_logger(), "RAW FINE ENDPOINT: error=%.4f mm",
        1000 * (raw.translation() - point(target)).norm());
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration timing(
      fine ? 0.00001 : 0.001, fine ? 0.005 : 0.01, fine ? 0.000001 : 0.0001);
    std::optional<robot_trajectory::RobotTrajectory> raw;
    if (mode == LinearMode::shelf_descent) {raw.emplace(trajectory, true);}
    if (!timing.computeTimeStamps(trajectory, scale, scale)) {
      require(raw.has_value(), "Trajectory timing failed");
      RCLCPP_WARN(node_->get_logger(), "SHELF DESCENT TIMING FALLBACK: validating saved path");
      trajectory = timeShelfDescent(*raw, scale);
    }
    const auto endpoint = trajectory.getLastWayPoint().getGlobalLinkTransform(tcp_);
    const double error = (endpoint.translation() - point(target)).norm();
    const double angle = Eigen::Quaterniond(endpoint.rotation()).angularDistance(rotation(target));
    RCLCPP_INFO(node_->get_logger(), "TIMED ENDPOINT: error=%.3f mm angle=%.4f rad",
      error * 1000, angle);
    require(error < (fine ? 0.00005 : 0.0005) && angle < (fine ? 0.0015 : 0.005),
      "Timed endpoint diverged from target");
    for (const auto & name : state->getJointModelGroup("arm_2")->getVariableNames()) {
      RCLCPP_INFO(node_->get_logger(), "TIMED JOINT %s target=%.6f", name.c_str(),
        trajectory.getLastWayPoint().getVariablePosition(name));
    }
    trajectory.getRobotTrajectoryMsg(msg);
    require(arm_->execute(msg) == moveit::core::MoveItErrorCode::SUCCESS,
        "Cartesian execution failed");
  }
  Pose freshBox()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require(sequence_ > 0 && Wall::now() - box_received_ < 2s, "Physical box pose stale/missing");
    return box_;
  }
  Pose stableBox()
  {
    Pose previous;size_t last_sequence = 0;int calm = 0;
    const auto end = Wall::now() + 15s;
    while (Wall::now() < end) {
      std::this_thread::sleep_for(200ms);
      Pose current;size_t sequence;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (sequence_ == 0 || sequence_ == last_sequence) {continue;}
        current = box_;sequence = sequence_;
      }
      calm = last_sequence && (point(current) - point(previous)).norm() < 0.0005 &&
        rotation(current).angularDistance(rotation(previous)) < 0.01 ? calm + 1 : 0;
      previous = current;last_sequence = sequence;
      if (calm >= 3) {return freshBox();}
    }
    throw std::runtime_error("Physical box did not settle");
  }
  void waitAt(
    const Pose * target, double position_tolerance = 0.003,
    double angle_tolerance = 0.025, bool allow_pickup_correction = false)
  {
    auto previous = arm_->getCurrentJointValues();
    int calm = 0;
    int still = 0;
    const auto end = Wall::now() + 20s;
    while (Wall::now() < end) {
      std::this_thread::sleep_for(200ms);
      auto state = arm_->getCurrentState(2.0);
      require(static_cast<bool>(state), "Fresh arm state missing");
      std::vector<double> current;state->copyJointGroupPositions("arm_2", current);
      require(current.size() == 7 && previous.size() == 7, "Incomplete arm state");
      double delta = 0;
      for (size_t i = 0; i < 7; ++i) {delta = std::max(delta, std::abs(current[i] - previous[i]));}
      bool near = true;
      double position_error = 0.0;
      double angle_error = 0.0;
      if (target) {
        state->update();
        const auto & actual = state->getGlobalLinkTransform(tcp_);
        position_error = (actual.translation() - point(*target)).norm();
        angle_error = Eigen::Quaterniond(actual.rotation()).angularDistance(rotation(*target));
        near = position_error < position_tolerance && angle_error < angle_tolerance;
      }
      still = delta < 0.0002 ? still + 1 : 0;
      calm = still > 0 && near ? calm + 1 : 0;
      previous = current;
      if (calm >= 3) {return;}
      if (allow_pickup_correction && target && still >= 3 && !near &&
        position_error < 0.003 && angle_error < 0.015)
      {
        RCLCPP_INFO(node_->get_logger(),
          "RIGHT PICK APPROACH CORRECTION: position_error=%.3f mm angle=%.4f rad",
          position_error * 1000, angle_error);
        linear(*target, 0.01, LinearMode::pickup);
        waitAt(target, position_tolerance, angle_tolerance);
        RCLCPP_INFO(node_->get_logger(), "RIGHT PICK APPROACH CORRECTION PASSED");
        return;
      }
    }
    const auto actual = arm_->getCurrentPose(tcp_).pose;
    if (target) {
      RCLCPP_ERROR(node_->get_logger(), "SETTLE FAILED: position_error=%.3f mm angle=%.4f rad",
        1000 * (point(actual) - point(*target)).norm(),
        rotation(actual).angularDistance(rotation(*target)));
    }
    auto state = arm_->getCurrentState(2.0);
    if (state) {
      for (const auto & name : state->getJointModelGroup("arm_2")->getVariableNames()) {
        RCLCPP_ERROR(node_->get_logger(), "SETTLE JOINT %s actual=%.6f", name.c_str(),
          state->getVariablePosition(name));
      }
    }
    throw std::runtime_error("Arm not settled at target; refusing next motion");
  }
  void executePlan()
  {
    Group::Plan plan;bool planned = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      arm_->setStartStateToCurrentState();
      if (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {planned = true;break;}
    }
    require(planned, "Planning failed");
    require(arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS, "Execution failed");
  }
  void gripper(double position, bool grasp)
  {
    Grip::Goal goal;goal.command.position = position;goal.command.max_effort = 20;
    auto sent = grip_->async_send_goal(goal);
    require(sent.wait_for(5s) == std::future_status::ready, "Gripper goal timed out");
    auto handle = sent.get();require(static_cast<bool>(handle), "Gripper goal rejected");
    auto future = grip_->async_get_result(handle);
    if (future.wait_for(30s) != std::future_status::ready) {
      grip_->async_cancel_goal(handle);throw std::runtime_error("Gripper result timed out");
    }
    auto result = future.get();
    const bool stopped_by_box = grasp && result.result && result.result->stalled &&
      result.code == rclcpp_action::ResultCode::ABORTED;
    require(result.code == rclcpp_action::ResultCode::SUCCEEDED || stopped_by_box,
      "Gripper command failed");
    std::this_thread::sleep_for(500ms);
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < 2; ++i) {
      require(Wall::now() - finger_received_[i] < 1s, "Physical finger state stale");
      require(grasp ? fingers_[i] > 0.0185 && fingers_[i] < 0.022 :
        std::abs(fingers_[i] - position) < 0.001, "Physical finger opening unexpected");
    }
    RCLCPP_INFO(node_->get_logger(), "PHYSICAL FINGERS: %.4f / %.4f m stalled=%d",
      fingers_[0], fingers_[1], result.result ? result.result->stalled : false);
  }
  void addBox(const Pose & box)
  {
    moveit_msgs::msg::CollisionObject object;object.header.frame_id = "world";
    object.id = "target_box";object.operation = object.ADD;
    shape_msgs::msg::SolidPrimitive primitive;primitive.type = primitive.BOX;
    primitive.dimensions = {0.04, 0.04, 0.04};
    object.primitives.push_back(primitive);object.primitive_poses.push_back(box);
    require(scene_->applyCollisionObject(object), "Planning box synchronization failed");
  }
  double rack_y_ = 1.20;
  rclcpp::Node::SharedPtr node_;
  std::mutex mutex_;
  Pose box_;size_t sequence_ = 0;int64_t stamp_ = -1, support_sim_ = 0;
  bool unsafe_ = false, carrying_ = false, monitor_shelf_ = false, seating_ = false;
  Wall::time_point shelf_box_wall_, shelf_surface_wall_;
  int64_t shelf_sim_ = 0;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr shelf_contacts_;
  Wall::time_point box_received_, support_received_;
  std::array<double, 2> fingers_{};
  std::array<Wall::time_point, 2> finger_received_{}, grasp_received_{};
  const std::string tcp_ = "right_fr3v2_hand_tcp";
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poses_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contacts_, table_contacts_;
  rclcpp_action::Client<Grip>::SharedPtr grip_;
  std::unique_ptr<Group> arm_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("right_arm_table_pick",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  std::thread spinner([&executor] {executor.spin();});
  std::unique_ptr<RightTablePick> test;int rc = 0;
  try {
    test = std::make_unique<RightTablePick>(node);test->run();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "RIGHT ARM TEST FAILED: %s; inspect before restarting",
      e.what());rc = 1;
  }
  executor.cancel();spinner.join();test.reset();rclcpp::shutdown();return rc;
}
