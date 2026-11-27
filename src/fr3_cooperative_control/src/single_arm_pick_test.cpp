// Physical pick, transfer and placement. MoveIt attachment does not constrain Gazebo.
#include <algorithm>
#include <array>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
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
#include "fr3_cooperative_control/simulation_hold_clock.hpp"

using namespace std::chrono_literals;
using Pose = geometry_msgs::msg::Pose;
using Group = moveit::planning_interface::MoveGroupInterface;
using Grip = control_msgs::action::GripperCommand;
using Wall = std::chrono::steady_clock;

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
class FinePathRejected : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};
class CompartmentSearchExhausted : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class Pick
{
public:
  explicit Pick(rclcpp::Node::SharedPtr node)
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
            if (msg->name[i] == "left_fr3v2_finger_joint" + std::to_string(j + 1)) {
              fingers_[j] = msg->position[i];finger_received_[j] = Wall::now();
            }
          }
        }
      });
    contacts_ = node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/placement/box_contacts", rclcpp::SensorDataQoS(),
      [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!msg->contacts.empty()) {contacts_seen_ = true;}
        for (const auto & contact : msg->contacts) {
          const auto & a = contact.collision1.name;const auto & b = contact.collision2.name;
          const bool table = a.find("table_top_collision") != std::string::npos ||
          b.find("table_top_collision") != std::string::npos;
          const bool finger = a.find("left_fr3v2_leftfinger") != std::string::npos ||
          a.find("left_fr3v2_rightfinger") != std::string::npos ||
          b.find("left_fr3v2_leftfinger") != std::string::npos ||
          b.find("left_fr3v2_rightfinger") != std::string::npos;
          for (size_t i = 0; i < 2; ++i) {
            const std::string name = i == 0 ? "left_fr3v2_leftfinger" : "left_fr3v2_rightfinger";
            if (a.find(name) != std::string::npos || b.find(name) != std::string::npos) {
              finger_contact_[i] = Wall::now();
            }
          }
          if (!table && !finger && (a.find("fr3v2") != std::string::npos ||
          b.find("fr3v2") != std::string::npos)) {unexpected_box_contact_ = true;}
          // Same rule as the right arm: while carrying, anything except the fingers
          // (and the tabletop once seating has started) is an unexpected contact.
          if (carrying_ && !finger && !(seating_ && table)) {
            if (!carry_contact_) {
              RCLCPP_ERROR(node_->get_logger(), "UNEXPECTED CARRY CONTACT: %s <-> %s",
                a.c_str(), b.c_str());
            }
            carry_contact_ = true;
          }
          bool vertical = false;
          for (const auto & n : contact.normals) {vertical = vertical || std::abs(n.z) > 0.8;}
          double depth = 0;
          for (double d : contact.depths) {depth = std::max(depth, d);}
          if (table && vertical && !contact.depths.empty() && depth < 0.001) {
            if (!support_seen_) {
              RCLCPP_INFO(node_->get_logger(), "TABLE CONTACT: %s <-> %s", a.c_str(), b.c_str());
            }
            support_seen_ = true;support_wall_ = Wall::now();
            support_sim_ = node_->now().nanoseconds();
          }
        }
      });
    table_contacts_ = node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/placement/table_contacts", rclcpp::SensorDataQoS(),
      [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & contact : msg->contacts) {
          const auto & a = contact.collision1.name;const auto & b = contact.collision2.name;
          if (a.find("target_box") != std::string::npos ||
          b.find("target_box") != std::string::npos)
          {
            table_support_wall_ = Wall::now();table_support_seen_ = true;
          }
          if (a.find("fr3v2") != std::string::npos || b.find("fr3v2") != std::string::npos) {
            if (!robot_table_contact_) {
              RCLCPP_ERROR(node_->get_logger(), "ROBOT TABLE CONTACT: %s <-> %s", a.c_str(),
                b.c_str());
            }
            robot_table_contact_ = true;
          }
        }
      });
    grip_ = rclcpp_action::create_client<Grip>(node_, "/left_gripper_controller/gripper_cmd");
  }
  void run()
  {
    std::string stage;node_->get_parameter_or<std::string>("stage", stage, "pick");
    require(stage == "pick" || stage == "transfer" || stage == "place", "Invalid test stage");
    auto controllers = node_->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
    require(controllers->wait_for_service(90s), "Controller manager unavailable");
    const auto deadline = Wall::now() + 90s;
    bool ready = false;
    while (rclcpp::ok() && Wall::now() < deadline) {
      auto future = controllers->async_send_request(
        std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
      if (future.wait_for(3s) == std::future_status::ready) {
        int count = 0;
        const auto response = future.get();
        for (const auto & c : response->controller) {
          if (c.state == "active" &&
            (c.name == "arm_1_controller" || c.name == "left_gripper_controller")) {++count;}
        }
        if (count == 2) {ready = true;break;}
      }
      std::this_thread::sleep_for(500ms);
    }
    require(ready && grip_->wait_for_action_server(30s), "Controllers not ready");
    arm_ = std::make_unique<Group>(node_, "arm_1");
    arm_->setEndEffectorLink(tcp_);arm_->setPoseReferenceFrame("world");
    if (node_->has_parameter("source_xyz") && node_->has_parameter("compartment_planning_pipeline")) {
      const auto pipeline = node_->get_parameter("compartment_planning_pipeline").as_string();
      require(!pipeline.empty(), "Empty compartment planning pipeline");
      arm_->setPlanningPipelineId(pipeline);
      RCLCPP_INFO(node_->get_logger(), "COMPARTMENT PLANNING PIPELINE: %s", pipeline.c_str());
    }
    arm_->setPlanningTime(10);arm_->setNumPlanningAttempts(3);
    arm_->setGoalPositionTolerance(0.002);arm_->setGoalOrientationTolerance(0.01);
    // Boxless motions only (ready and pregrasp); carry motions set their own lower scales.
    arm_->setMaxVelocityScalingFactor(0.40);arm_->setMaxAccelerationScalingFactor(0.40);
    require(arm_->startStateMonitor(15), "Robot state unavailable");
    scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    require(scene_->getAttachedObjects({"target_box"}).empty(), "Restart simulation: box attached");
    Eigen::Vector3d expected_source(0, -1.04, 0.4825);
    double pick_lift = 0.08, pick_approach = 0.12;
    const double withdraw_distance = node_->has_parameter("source_xyz") ?
      node_->get_parameter_or<double>("compartment_withdraw_distance", 0.0) : 0.0;
    require(std::isfinite(withdraw_distance) && withdraw_distance >= 0 && withdraw_distance <= 0.12,
      "Invalid compartment withdrawal distance");
    if (withdraw_distance > 0) {
      require(std::isfinite(node_->get_parameter("compartment_front_y").as_double()),
        "Invalid source shelf front");
    }
    if (node_->has_parameter("source_xyz")) {
      require(stage == "pick" || withdraw_distance > 0,
        "Custom-source table flow requires compartment withdrawal");
      const auto source = node_->get_parameter("source_xyz").as_double_array();
      require(source.size() == 3 && std::all_of(source.begin(), source.end(),
          [](double value) {return std::isfinite(value);}), "Invalid source coordinates");
      expected_source = Eigen::Vector3d(source[0], source[1], source[2]);
      pick_lift = node_->get_parameter("pick_lift").as_double();
      pick_approach = node_->get_parameter("pick_approach").as_double();
      require(std::isfinite(pick_lift) && pick_lift > 0 && pick_lift <= 0.08 &&
        std::isfinite(pick_approach) && pick_approach > 0 && pick_approach <= 0.12,
        "Invalid compartment pick distances");
    }
    Pose initial = stableBox();
    require((point(initial) - expected_source).norm() < 0.005,
      "Box is not resting at expected shelf position");
    if (stage == "place") {
      const auto sensor_deadline = Wall::now() + 10s;
      bool seen = false;
      while (Wall::now() < sensor_deadline) {
        {std::lock_guard<std::mutex> lock(mutex_);seen = contacts_seen_;}
        if (seen) {break;}
        std::this_thread::sleep_for(100ms);
      }
      require(seen, "Box contact sensor unavailable; refusing placement test");
    }
    gripper(0.04, false);
    arm_->setJointValueTarget(std::vector<double>{0, -0.785, 0, -2.356, 0, 1.571, 0.785});
    executePlan();waitAt(nullptr);
    Pose pick = initial;
    pick.orientation.x = 0.5;pick.orientation.y = 0.5;
    pick.orientation.z = -0.5;pick.orientation.w = 0.5;
    addBox(initial);allowFingers(false, !node_->has_parameter("source_xyz"));
    Pose pre = pick;pre.position.y += pick_approach;
    Pose withdraw_goal, withdraw_via;
    if (node_->has_parameter("source_xyz")) {
      const auto approach = planCompartmentPickup(initial, pick, pre, pick_lift, pick_approach,
        withdraw_distance, withdraw_goal, withdraw_via);
      require(arm_->execute(approach) == moveit::core::MoveItErrorCode::SUCCESS,
        "Compartment approach execution failed");
    } else {
      arm_->setPoseTarget(pre);executePlan();
    }
    arm_->clearPoseTargets();waitAt(&pre);
    if (node_->has_parameter("source_xyz")) {checkSourceUnmoved(initial);}
    linear(pick);waitAt(&pick);
    if (node_->has_parameter("source_xyz")) {
      checkSourceUnmoved(initial);allowFingers();
    }
    RCLCPP_INFO(node_->get_logger(), "PICK TARGET SETTLED: closing gripper now");
    gripper(0.018, true);
    const Pose grasped = stableBox();
    const Pose grasp_tcp = arm_->getCurrentPose(tcp_).pose;
    const Eigen::Vector3d reference = rotation(grasp_tcp).inverse() *
      (point(grasped) - point(grasp_tcp));
    const Eigen::Quaterniond reference_angle = rotation(grasp_tcp).inverse() * rotation(grasped);
    addBox(grasped);
    arm_->attachObject("target_box", tcp_,
      {"left_fr3v2_leftfinger", "left_fr3v2_rightfinger", "left_fr3v2_hand"});
    const auto attach_deadline = Wall::now() + 5s;
    while (scene_->getAttachedObjects({"target_box"}).empty() && Wall::now() < attach_deadline) {
      std::this_thread::sleep_for(100ms);
    }
    require(!scene_->getAttachedObjects({"target_box"}).empty(), "Planning attachment failed");
    Pose lifted = pick;lifted.position.z += pick_lift;
    linear(lifted);waitAt(&lifted);
    const auto actual_box = stableBox();
    const double rise = actual_box.position.z - initial.position.z;
    RCLCPP_INFO(node_->get_logger(), "PHYSICAL LIFT: %.2f mm", rise * 1000);
    require(std::abs(rise - pick_lift) < 0.005, "Physical box did not follow requested lift");
    require((point(actual_box).head<2>() - point(initial).head<2>()).norm() < 0.01,
      "Box shifted sideways during lift");
    const auto sim_start = node_->now();const auto wall_start = Wall::now();
    double max_drift = 0, max_angle = 0;
    while ((node_->now() - sim_start).seconds() < 3.0) {
      require(Wall::now() - wall_start < 30s, "Hold simulation clock timed out");
      const Pose box = freshBox();const Pose tcp = arm_->getCurrentPose(tcp_).pose;
      max_drift = std::max(max_drift, (rotation(tcp).inverse() *
          (point(box) - point(tcp)) - reference).norm());
      max_angle = std::max(max_angle, reference_angle.angularDistance(
        rotation(tcp).inverse() * rotation(box)));
      require(max_drift < 0.005 && max_angle < 0.0873, "Box slipped or rotated in gripper");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(),
      "PICK TEST PASSED: lift=%.2f mm hold=3 sim s drift=%.3f mm angle=%.3f deg; gripper stays closed",
      rise * 1000, max_drift * 1000, max_angle * 180 / M_PI);
    Pose clear = lifted;
    if (withdraw_distance > 0) {
      {std::lock_guard<std::mutex> lock(mutex_);carrying_ = true;}
      const Eigen::Matrix3d turn =
        (rotation(withdraw_goal) * rotation(lifted).inverse()).toRotationMatrix();
      RCLCPP_INFO(node_->get_logger(), "COMPARTMENT WITHDRAW START: dy=%.1f mm extra_raise=%.1f mm "
        "yaw_change=%.1f deg", withdraw_distance * 1000,
        (withdraw_goal.position.z - lifted.position.z) * 1000,
        std::atan2(turn(1, 0), turn(0, 0)) * 180 / M_PI);
      compartmentWithdraw(withdraw_via, withdraw_goal, reference, reference_angle);
      waitAt(&withdraw_goal);
      holdTransferred("COMPARTMENT WITHDRAW", reference, reference_angle, max_drift, max_angle);
      const auto box = stableBox();
      const double half_y = 0.02 * rotation(box).toRotationMatrix().row(1).cwiseAbs().sum();
      const double front = node_->get_parameter("compartment_front_y").as_double();
      require(std::isfinite(front), "Invalid source shelf front");
      const double clearance = box.position.y - half_y - front;
      const Eigen::Vector3d expected = point(withdraw_goal) + rotation(withdraw_goal) * reference;
      require((point(box) - expected).norm() < 0.01 && clearance > 0.01,
        "Physical box did not clear the source shelf by 10 mm");
      RCLCPP_INFO(node_->get_logger(), "COMPARTMENT WITHDRAW TEST PASSED: dy=%.2f mm "
        "rear_clearance=%.2f mm drift=%.3f mm angle=%.3f deg; gripper stays closed",
        (box.position.y - actual_box.position.y) * 1000, clearance * 1000,
        max_drift * 1000, max_angle * 180 / M_PI);
      clear = withdraw_goal;
      if (stage == "pick") {return;}
    } else {
      if (stage == "pick") {return;}
      {std::lock_guard<std::mutex> lock(mutex_);carrying_ = true;}
      RCLCPP_INFO(node_->get_logger(), "TRANSFER RETRACT START: 120 mm, gripper stays closed");
      clear = lifted;clear.position.y += 0.12;
      linear(clear);waitAt(&clear);
      holdTransferred("AFTER RETRACT", reference, reference_angle, max_drift, max_angle, 1.0);
    }
    // The shelf and table face opposite sides of the arm. Change hand orientation
    // during the collision-checked transfer; preserve the measured grasp transform.
    Pose table = tableTransferTarget(clear, reference);
    const Eigen::Vector3d desired_center(0.0, 0.05, 0.60);
    arm_->setMaxVelocityScalingFactor(0.10);arm_->setMaxAccelerationScalingFactor(0.10);
    RCLCPP_INFO(node_->get_logger(), "TABLE TRANSFER START: box target=(0.000, 0.050, 0.600)");
    const auto clear_of_table = [this, &reference, &reference_angle](const Group::Plan & plan) {
        return carryClearance(plan, reference, reference_angle, "TABLE TRANSFER");
      };
    const bool coupled = node_->has_parameter("optimize_table_transfer") &&
      node_->get_parameter("optimize_table_transfer").as_bool();
    Group::Plan selected;
    if (node_->has_parameter("source_xyz")) {
      planning_scene::PlanningScene transfer_scene(arm_->getRobotModel());
      require(transfer_scene.setPlanningSceneMsg(planningSceneSnapshot()),
        "Compartment transfer scene unavailable");
      auto start = transfer_scene.getCurrentState();
      auto measured = arm_->getCurrentState(2.0);
      require(static_cast<bool>(measured), "Compartment transfer state unavailable");
      for (const auto & name : start.getJointModelGroup("arm_1")->getVariableNames()) {
        start.setVariablePosition(name, measured->getVariablePosition(name));
      }
      start.update();
      std::vector<Group::Plan> route;
      require(planCompartmentTableRoute(transfer_scene, start, table, reference, reference_angle,
          route, Wall::now() + 20s, "measured"),
        "No checked compartment-to-table route from measured grasp");
      for (size_t i = 0; i < route.size(); ++i) {
        if (i > 0) {
          waitAt(nullptr);
          const auto continuation = prepareCompartmentContinuation(route[i], reference, reference_angle);
          executeMonitored(continuation.trajectory, "TABLE TRANSFER", reference, reference_angle);
        } else {
          executeMonitored(route[i].trajectory, "TABLE TRANSFER", reference, reference_angle);
        }
      }
    } else if (coupled && planCoupledTableTransfer(table, reference, reference_angle, selected)) {
      executeMonitored(selected.trajectory, "TABLE TRANSFER", reference, reference_angle);
    } else {
      arm_->setPoseTarget(table);
      executePlan(clear_of_table, [this, &reference, &reference_angle](const Group::Plan & plan) {
          executeMonitored(plan.trajectory, "TABLE TRANSFER", reference, reference_angle);
        });
    }
    arm_->clearPoseTargets();
    waitAt(&table);
    holdTransferred("ABOVE TABLE", reference, reference_angle, max_drift, max_angle);
    const auto placed = freshBox();
    const double center_error = (point(placed) - desired_center).norm();
    const double normal_z = std::abs((rotation(placed) * Eigen::Vector3d::UnitZ()).z());
    const double tilt = std::acos(std::clamp(normal_z, 0.0, 1.0));
    RCLCPP_INFO(node_->get_logger(), "TABLE BOX: center_error=%.3f mm tilt=%.3f deg",
      center_error * 1000, tilt * 180 / M_PI);
    require(center_error < 0.01 && tilt < 0.0873, "Box not held level at table target");
    RCLCPP_INFO(node_->get_logger(),
      "TRANSFER TEST PASSED: drift=%.3f mm angle=%.3f deg; holding above table, gripper stays closed",
      max_drift * 1000, max_angle * 180 / M_PI);
    if (stage == "place") {place(reference, reference_angle);}
  }

private:
  Pose tableTransferTarget(const Pose & clear, const Eigen::Vector3d & relative)
  {
    Pose table = clear;
    table.orientation.x = -0.5;table.orientation.y = -0.5;
    table.orientation.z = -0.5;table.orientation.w = 0.5;
    const Eigen::Vector3d target = Eigen::Vector3d(0, 0.05, 0.60) - rotation(table) * relative;
    table.position.x = target.x();table.position.y = target.y();table.position.z = target.z();
    return table;
  }

  void checkSourceUnmoved(const Pose & initial)
  {
    const auto actual = stableBox();
    const double shift = (point(actual) - point(initial)).norm();
    const double angle = rotation(actual).angularDistance(rotation(initial));
    RCLCPP_INFO(node_->get_logger(), "SOURCE APPROACH CHECK: shift=%.3f mm angle=%.3f deg",
      shift * 1000, angle * 180 / M_PI);
    require(shift < 0.002 && angle < 0.01745,
      "Source cube moved during open approach; refusing gripper close");
  }

  bool compartmentPath(
    const planning_scene::PlanningScene & scene, moveit::core::RobotState start,
    const Pose & target, std::vector<moveit::core::RobotStatePtr> & path,
    const char * phase)
  {
    const auto * group = start.getJointModelGroup("arm_1");
    Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
    goal.translation() = point(target);goal.linear() = rotation(target).toRotationMatrix();
    std::string blocked = "IK_or_interpolation";
    const auto valid = [&scene, &blocked](moveit::core::RobotState * state,
      const moveit::core::JointModelGroup * joints, const double * values) {
        state->setJointGroupPositions(joints, values);state->update();
        for (const auto & name : joints->getVariableNames()) {
          const auto & bounds = state->getRobotModel()->getVariableBounds(name);
          const double value = state->getVariablePosition(name);
          if (value < bounds.min_position_ + 0.05 || value > bounds.max_position_ - 0.05) {
            blocked = "joint_margin:" + name;
            return false;
          }
        }
        if (!scene.isStateColliding(*state)) {return true;}
        collision_detection::CollisionRequest request;
        request.contacts = true;request.max_contacts = 1;
        collision_detection::CollisionResult result;scene.checkCollision(request, result, *state);
        if (!result.contacts.empty()) {
          const auto & pair = result.contacts.begin()->first;
          blocked = pair.first + ":" + pair.second;
        }
        return false;
      };
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &start, group, path, start.getLinkModel(tcp_), goal, true,
      moveit::core::MaxEEFStep(0.001), moveit::core::CartesianPrecision{}, valid,
      kinematics::KinematicsQueryOptions());
    if (fraction < 0.999 || path.size() < 2) {
      RCLCPP_INFO(node_->get_logger(), "COMPARTMENT PATH REJECTED: phase=%s fraction=%.4f reason=%s",
        phase, fraction, blocked.c_str());return false;
    }
    for (size_t i = 1; i < path.size(); ++i) {
      for (const auto & name : group->getVariableNames()) {
        if (std::abs(path[i]->getVariablePosition(name) -
          path[i - 1]->getVariablePosition(name)) > 0.02) {
          RCLCPP_INFO(node_->get_logger(), "COMPARTMENT PATH REJECTED: phase=%s joint_jump=%s",
            phase, name.c_str());return false;
        }
      }
    }
    return true;
  }

  Group::Plan planCompartmentPickup(
    const Pose & box, Pose & pick, Pose & pre, double lift, double approach_distance,
    double withdraw_distance, Pose & withdraw_goal, Pose & withdraw_via)
  {
    const auto original_joints = arm_->getCurrentJointValues();
    require(original_joints.size() == 7, "Preflight joint state unavailable");
    for (int pass = 0; pass < 2; ++pass) {
      compartment_table_route_.clear();table_goal_seed_.clear();
      arm_->setStartStateToCurrentState();arm_->clearPoseTargets();
      RCLCPP_INFO(node_->get_logger(), "COMPARTMENT PREFLIGHT SEARCH: pass=%d/2", pass + 1);
      try {
        return planCompartmentPickupAttempt(box, pick, pre, lift, approach_distance,
          withdraw_distance, withdraw_goal, withdraw_via);
      } catch (const CompartmentSearchExhausted & error) {
        if (pass == 1) {
          RCLCPP_ERROR(node_->get_logger(), "COMPARTMENT PREFLIGHT SEARCH FAILED: two passes exhausted");
          throw;
        }
        checkSourceUnmoved(box);
        const auto current = arm_->getCurrentJointValues();
        require(current.size() == original_joints.size(), "Retry joint state unavailable");
        double joint_delta = 0;
        for (size_t i = 0; i < current.size(); ++i) {
          joint_delta = std::max(joint_delta, std::abs(current[i] - original_joints[i]));
        }
        require(joint_delta < 0.002, "Arm moved during preflight; refusing planning retry");
        RCLCPP_WARN(node_->get_logger(),
          "COMPARTMENT PREFLIGHT RETRY VERIFIED: source unchanged joint_delta=%.6f rad; "
          "planning only, no pickup started; reason=%s", joint_delta, error.what());
      }
    }
    throw std::runtime_error("Unexpected compartment search exit");
  }

  Group::Plan planCompartmentPickupAttempt(
    const Pose & box, Pose & pick, Pose & pre, double lift, double approach_distance,
    double withdraw_distance, Pose & withdraw_goal, Pose & withdraw_via)
  {
    const auto search_started = Wall::now();
    auto scene = std::make_shared<planning_scene::PlanningScene>(arm_->getRobotModel());
    require(scene->setPlanningSceneMsg(planningSceneSnapshot()), "Compartment scene unavailable");
    // The selected world cube is resting on the source shelf before attachment.
    // Permit that support pair only in this private entry probe, never robot/shelf contact.
    scene->getAllowedCollisionMatrixNonConst().setEntry("target_box", "shelf_1_link", true);
    const auto * group = scene->getRobotModel()->getJointModelGroup("arm_1");
    const auto valid = [&scene](moveit::core::RobotState * state,
      const moveit::core::JointModelGroup * joints, const double * values) {
        state->setJointGroupPositions(joints, values);state->update();
        for (const auto & name : joints->getVariableNames()) {
          const auto & b = state->getRobotModel()->getVariableBounds(name);
          const double v = state->getVariablePosition(name);
          if (v < b.min_position_ + 0.05 || v > b.max_position_ - 0.05) {return false;}
        }
        return !scene->isStateColliding(*state);
      };
    const auto nominal = rotation(pick);
    struct GraspSeed {std::vector<double> joints;int roll;const char * origin;};
    std::vector<GraspSeed> seeds;
    if (node_->has_parameter("compartment_grasp_seed")) {
      const auto probe_seed = node_->get_parameter("compartment_grasp_seed").as_double_array();
      require(probe_seed.size() == 7 && std::all_of(probe_seed.begin(), probe_seed.end(),
          [](double value) {return std::isfinite(value);}), "Invalid compartment diagnostic seed");
      const int probe_roll = node_->get_parameter("compartment_grasp_roll_pi").as_int();
      require(probe_roll == 0 || probe_roll == 1, "Invalid compartment diagnostic roll");
      seeds.push_back({probe_seed, probe_roll, "diagnostic"});
    }
    if (node_->has_parameter("compartment_grasp_seed_bank")) {
      const auto bank = node_->get_parameter("compartment_grasp_seed_bank").as_double_array();
      require(bank.size() % 8 == 0 && bank.size() <= 64 &&
        std::all_of(bank.begin(), bank.end(), [](double v) {return std::isfinite(v);}),
        "Invalid saved grasp seed bank");
      for (size_t i = 0; i < bank.size(); i += 8) {
        require(bank[i] == 0 || bank[i] == 1, "Invalid saved grasp roll");
        seeds.push_back({std::vector<double>(bank.begin() + i + 1, bank.begin() + i + 8),
          static_cast<int>(bank[i]), "saved"});
      }
    }
    const int seed_count = static_cast<int>(seeds.size());
    RCLCPP_INFO(node_->get_logger(), "COMPARTMENT SEARCH START: preferred_seeds=%d fallback_candidates=32",
      seed_count);
    std::string task_stage;node_->get_parameter_or<std::string>("stage", task_stage, "pick");
    const bool check_table = task_stage == "transfer" || task_stage == "place";
    std::array<int, 7> rejected{};
    const auto deadline = Wall::now() + 25s;
    for (int attempt = 0; attempt < seed_count + 32 && Wall::now() < deadline; ++attempt) {
      const bool seeded = attempt < seed_count;
      const auto reject = [&](size_t reason, const char * phase) {
          ++rejected[reason];
          if (seeded) {
            RCLCPP_INFO(node_->get_logger(), "COMPARTMENT PREFERRED REJECTED: seed_index=%d phase=%s",
              attempt + 1, phase);
          }
        };
      const int normal = attempt - seed_count;
      const int roll = seeded ? seeds[attempt].roll : normal % 2;
      const Eigen::Quaterniond q = nominal * Eigen::Quaterniond(Eigen::AngleAxisd(
        roll * M_PI, Eigen::Vector3d::UnitZ()));
      Pose candidate = pick;
      candidate.orientation.w = q.w();candidate.orientation.x = q.x();
      candidate.orientation.y = q.y();candidate.orientation.z = q.z();
      Pose before = candidate;before.position.y += approach_distance;
      // Also seed the grasp first: an arbitrary pre-grasp IK branch can run
      // out of joint margin before reaching a low compartment.
      const bool grasp_first = seeded || (normal >= 2 &&
        (withdraw_distance > 0 || normal % 4 >= 2));
      // A raised entrance avoids extending the elbow at the low grasp height.
      // The complete diagonal entry and subsequent lift remain collision checked.
      if (grasp_first) {
        before.position.y = candidate.position.y + approach_distance * 0.5;
        before.position.z += lift;
      }
      Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
      goal.translation() = point(grasp_first ? candidate : before);
      goal.linear() = q.toRotationMatrix();
      // Seed some withdrawal candidates from outside the shelf, then trace back
      // to the grasp. Forward entry/lift/attached withdrawal are still required.
      const bool exit_seed = withdraw_distance > 0 && !seeded && normal >= 2 && normal % 4 >= 2;
      if (exit_seed) {
        goal.translation() = point(candidate) + Eigen::Vector3d(0, withdraw_distance,
          lift + ((normal / 4) % 2 == 0 ? 0.06 : 0.04));
        const double yaw = ((normal / 4) % 3 - 1) * M_PI / 12;
        goal.linear() = (Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) * q)
          .toRotationMatrix();
      }
      auto state = scene->getCurrentState();
      if (seeded) {state.setJointGroupPositions(group, seeds[attempt].joints);state.update();}
      else if (normal >= 2) {state.setToRandomPositions(group);}
      if (!state.setFromIK(group, goal, tcp_, 0.08, valid)) {reject(0, "IK");continue;}
      if (exit_seed) {
        Pose seed_raised = candidate;seed_raised.position.z = goal.translation().z();
        std::vector<moveit::core::RobotStatePtr> reverse_exit, reverse_lift;
        if (!compartmentPath(*scene, state, seed_raised, reverse_exit, "seed_reverse_exit") ||
          !compartmentPath(*scene, *reverse_exit.back(), candidate, reverse_lift, "seed_reverse_lift"))
        {
          reject(5, "reverse_exit_or_lift");continue;
        }
        state = *reverse_lift.back();
      }
      if (grasp_first) {
        std::vector<moveit::core::RobotStatePtr> reverse_entry;
        if (!compartmentPath(*scene, state, before, reverse_entry, "reverse_entry")) {
          reject(1, "reverse_entry");continue;
        }
        state = *reverse_entry.back();
      }
      std::vector<moveit::core::RobotStatePtr> entry;
      if (!compartmentPath(*scene, state, candidate, entry, "entry")) {reject(1, "entry");continue;}
      auto held_scene = scene->diff();
      held_scene->getAllowedCollisionMatrixNonConst().setEntry("target_box", "shelf_1_link", false);
      for (const auto & finger : {"left_fr3v2_leftfinger", "left_fr3v2_rightfinger"}) {
        held_scene->getAllowedCollisionMatrixNonConst().setEntry("target_box", finger, true);
      }
      auto held = *entry.back();
      held.setVariablePosition("left_fr3v2_finger_joint1", 0.020);
      held.setVariablePosition("left_fr3v2_finger_joint2", 0.020);held.update();
      held_scene->setCurrentState(held);
      moveit_msgs::msg::AttachedCollisionObject attached;
      attached.link_name = tcp_;
      attached.touch_links = {"left_fr3v2_leftfinger", "left_fr3v2_rightfinger", "left_fr3v2_hand"};
      attached.object.id = "target_box";attached.object.header.frame_id = tcp_;
      attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
      shape_msgs::msg::SolidPrimitive cube;cube.type = cube.BOX;cube.dimensions = {0.04, 0.04, 0.04};
      attached.object.primitives.push_back(cube);
      Pose relative;
      const Eigen::Vector3d offset = q.inverse() * (point(box) - point(candidate));
      relative.position.x = offset.x();relative.position.y = offset.y();relative.position.z = offset.z();
      const Eigen::Quaterniond angle = q.inverse() * rotation(box);
      relative.orientation.w = angle.w();relative.orientation.x = angle.x();
      relative.orientation.y = angle.y();relative.orientation.z = angle.z();
      attached.object.primitive_poses.push_back(relative);
      require(held_scene->processAttachedCollisionObjectMsg(attached), "Compartment probe attachment failed");
      Pose raised = candidate;raised.position.z += lift;
      std::vector<moveit::core::RobotStatePtr> lifting;
      if (!compartmentPath(*held_scene, held_scene->getCurrentState(), raised, lifting, "lift")) {
        reject(2, "lift");continue;
      }
      std::vector<moveit::core::RobotStatePtr> withdrawal;
      Pose exit = raised, via = raised;
      if (withdraw_distance > 0) {
        bool found = false;
        for (double extra_raise : {0.0, 0.02, 0.04, 0.06}) {
          via = raised;via.position.z += extra_raise;
          std::vector<moveit::core::RobotStatePtr> up;
          auto outside_start = *lifting.back();
          if (extra_raise > 0) {
            if (!compartmentPath(*held_scene, outside_start, via, up, "withdraw_raise")) {continue;}
            outside_start = *up.back();
          }
          for (double yaw : {0.0, M_PI / 12, -M_PI / 12}) {
            exit = via;exit.position.y += withdraw_distance;
            const Eigen::Quaterniond exit_q =
              Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) * q;
            exit.orientation.w = exit_q.w();exit.orientation.x = exit_q.x();
            exit.orientation.y = exit_q.y();exit.orientation.z = exit_q.z();
            withdrawal.clear();
            if (compartmentPath(*held_scene, outside_start, exit, withdrawal, "withdraw")) {
              withdrawal.insert(withdrawal.begin(), up.begin(), up.end());
              found = true;break;
            }
          }
          if (found) {break;}
        }
        if (!found) {reject(5, "withdraw");continue;}
      }
      if (check_table) {
        const Pose table = tableTransferTarget(exit, offset);
        std::vector<Group::Plan> probe;
        if (!planCompartmentTableRoute(*held_scene, *withdrawal.back(), table, offset, angle,
            probe, deadline, "pregrasp")) {reject(6, "table_route");continue;}
        // Keep the checked geometry; revalidate and retime it with the measured attachment.
        compartment_table_route_ = probe;
        arm_->setMaxVelocityScalingFactor(0.40);arm_->setMaxAccelerationScalingFactor(0.40);
      }
      std::vector<double> joints;state.copyJointGroupPositions(group, joints);
      if (!arm_->setJointValueTarget(joints)) {continue;}
      setPlanningStartState();Group::Plan plan;
      bool planned = false;
      const int route_attempts = withdraw_distance > 0 ? 3 : 1;
      for (int retry = 0; retry < route_attempts && Wall::now() < deadline; ++retry) {
        if (retry > 0) {
          RCLCPP_INFO(node_->get_logger(), "COMPARTMENT APPROACH PLAN RETRY: candidate=%d retry=%d",
            attempt + 1, retry);setPlanningStartState();
        }
        if (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {planned = true;break;}
      }
      if (!planned && withdraw_distance > 0) {
        planned = planCompartmentStagedApproach(*scene, state, before, plan, deadline);
      }
      if (!planned) {reject(3, "approach");continue;}
      bool margin = true;
      for (const auto & waypoint : plan.trajectory.joint_trajectory.points) {
        for (size_t i = 0; i < plan.trajectory.joint_trajectory.joint_names.size(); ++i) {
          const auto & name = plan.trajectory.joint_trajectory.joint_names[i];
          const auto & b = arm_->getRobotModel()->getVariableBounds(name);
          if (waypoint.positions[i] < b.min_position_ + 0.05 ||
            waypoint.positions[i] > b.max_position_ - 0.05) {margin = false;}
        }
      }
      if (!margin) {reject(4, "timed_margin");continue;}
      pick = candidate;pre = before;
      withdraw_goal = exit;
      withdraw_via = via;
      RCLCPP_INFO(node_->get_logger(),
        "COMPARTMENT PICK PREFLIGHT PASSED: candidate=%d roll_pi=%d entry_points=%zu "
        "lift_points=%zu joint_margin=0.05 rad grasp_first=%d seeded=%d withdraw_points=%zu "
        "seed_origin=%s seed_index=%d search_ms=%.1f",
        attempt + 1, roll, entry.size(), lifting.size(), grasp_first, seeded, withdrawal.size(),
        seeded ? seeds[attempt].origin : "normal", seeded ? attempt + 1 : 0,
        std::chrono::duration<double, std::milli>(Wall::now() - search_started).count());
      return plan;
    }
    RCLCPP_WARN(node_->get_logger(),
      "COMPARTMENT PREFLIGHT DIAG: ik=%d entry=%d lift=%d route=%d timed_margin=%d withdraw=%d table_route=%d",
      rejected[0], rejected[1], rejected[2], rejected[3], rejected[4], rejected[5], rejected[6]);
    throw CompartmentSearchExhausted(withdraw_distance > 0 ?
      "No compartment approach with complete entry, lift, withdrawal and joint margin" :
      "No compartment approach with complete entry and lift and joint margin");
  }

  bool planCompartmentStagedApproach(const planning_scene::PlanningScene & scene,
    const moveit::core::RobotState & pre_state, const Pose & pre, Group::Plan & result,
    Wall::time_point deadline)
  {
    auto measured = arm_->getCurrentState(2.0);
    if (!measured) {return false;}
    auto start = scene.getCurrentState();
    const auto * group = start.getJointModelGroup("arm_1");
    for (const auto & name : group->getVariableNames()) {
      start.setVariablePosition(name, measured->getVariablePosition(name));
    }
    start.update();
    const std::array<std::array<double, 2>, 10> stages{{
      {0.02, 0.0}, {0.02, 0.02}, {0.04, 0.02},
      {0.04, 0.04}, {0.08, 0.04}, {0.08, 0.08}, {0.12, 0.08},
      {0.04, 0.0}, {0.08, 0.0}, {0.12, 0.0}}};
    for (const auto & stage : stages) {
      if (Wall::now() >= deadline) {break;}
      Pose outside = pre;outside.position.y += stage[0];outside.position.z += stage[1];
      std::vector<moveit::core::RobotStatePtr> reverse;
      if (!compartmentPath(scene, pre_state, outside, reverse, "approach_stage_reverse")) {continue;}
      std::vector<double> joints;reverse.back()->copyJointGroupPositions(group, joints);
      arm_->setStartState(start);
      if (!arm_->setJointValueTarget(joints)) {continue;}
      Group::Plan prefix;
      if (arm_->plan(prefix) != moveit::core::MoveItErrorCode::SUCCESS ||
        !compartmentJointPlanValid(scene, start, prefix, "STAGED APPROACH PREFIX")) {continue;}
      robot_trajectory::RobotTrajectory timing(arm_->getRobotModel(), "arm_1");
      timing.setRobotTrajectoryMsg(start, prefix.trajectory);
      // Trace the checked Cartesian staging path back to the exact pregrasp branch.
      for (size_t i = reverse.size() - 1; i > 0; --i) {
        timing.addSuffixWayPoint(*reverse[i - 1], 0.01);
      }
      if (!timeWithRetry(timing, 0.40, false)) {continue;}
      Group::Plan candidate = prefix;timing.getRobotTrajectoryMsg(candidate.trajectory);
      if (!compartmentJointPlanValid(scene, start, candidate, "STAGED APPROACH")) {continue;}
      const auto endpoint = timing.getLastWayPoint().getGlobalLinkTransform(tcp_);
      if ((endpoint.translation() - point(pre)).norm() >= 0.00005 ||
        Eigen::Quaterniond(endpoint.linear()).angularDistance(rotation(pre)) >= 0.0015) {continue;}
      result = std::move(candidate);arm_->setStartStateToCurrentState();
      RCLCPP_INFO(node_->get_logger(),
        "COMPARTMENT STAGED APPROACH PASSED: outside=%.0f mm raise=%.0f mm; full path collision checked",
        stage[0] * 1000, stage[1] * 1000);
      return true;
    }
    arm_->setStartStateToCurrentState();
    RCLCPP_INFO(node_->get_logger(), "COMPARTMENT STAGED APPROACH REJECTED");
    return false;
  }

  // Check interpolated joint states in the private scene, including the held object.
  // The remote planner is only a candidate generator; it cannot relax this scene's ACM.
  bool compartmentJointPlanValid(const planning_scene::PlanningScene & scene,
    const moveit::core::RobotState & start, const Group::Plan & plan, const char * label)
  {
    const auto & motion = plan.trajectory.joint_trajectory;
    if (motion.points.empty()) {return false;}
    const auto * group = start.getJointModelGroup("arm_1");
    auto previous = start;
    const auto valid = collisionCheck(scene);
    for (const auto & waypoint : motion.points) {
      auto next = previous;next.setVariablePositions(motion.joint_names, waypoint.positions);
      next.update();
      double largest = 0;
      for (const auto & name : group->getVariableNames()) {
        largest = std::max(largest, std::abs(next.getVariablePosition(name) -
            previous.getVariablePosition(name)));
      }
      const int steps = std::max(1, static_cast<int>(std::ceil(largest / 0.01)));
      for (int i = 0; i <= steps; ++i) {
        auto sample = previous;
        previous.interpolate(next, static_cast<double>(i) / steps, sample);sample.update();
        std::vector<double> values;sample.copyJointGroupPositions(group, values);
        if (!valid(&sample, group, values.data())) {
          RCLCPP_INFO(node_->get_logger(), "COMPARTMENT %s STATE REJECTED: collision=%d",
            label, scene.isStateColliding(sample));
          return false;
        }
      }
      previous = next;
    }
    return true;
  }

  bool compartmentTransferPlanValid(const planning_scene::PlanningScene & scene,
    const moveit::core::RobotState & start, const Group::Plan & plan,
    const Eigen::Vector3d & relative, const Eigen::Quaterniond & angle)
  {
    return compartmentJointPlanValid(scene, start, plan, "TABLE ROUTE") &&
           carryClearance(plan, relative, angle, "COMPARTMENT TABLE ROUTE");
  }

  Group::Plan prepareCompartmentContinuation(const Group::Plan & saved,
    const Eigen::Vector3d & relative, const Eigen::Quaterniond & angle)
  {
    planning_scene::PlanningScene scene(arm_->getRobotModel());
    require(scene.setPlanningSceneMsg(planningSceneSnapshot()), "Continuation scene unavailable");
    auto measured = arm_->getCurrentState(2.0);
    require(static_cast<bool>(measured), "Continuation measured state unavailable");
    auto start = scene.getCurrentState();
    const auto * group = start.getJointModelGroup("arm_1");
    for (const auto & name : group->getVariableNames()) {
      start.setVariablePosition(name, measured->getVariablePosition(name));
    }
    start.update();scene.setCurrentState(start);
    const auto & motion = saved.trajectory.joint_trajectory;
    require(!motion.points.empty(), "Empty continuation trajectory");
    auto expected = start;
    expected.setVariablePositions(motion.joint_names, motion.points.front().positions);expected.update();
    double delta = 0;
    for (const auto & name : group->getVariableNames()) {
      delta = std::max(delta, std::abs(start.getVariablePosition(name) - expected.getVariablePosition(name)));
    }
    require(delta <= 0.03, "Continuation start changed by more than 0.03 rad; holding box");
    robot_trajectory::RobotTrajectory timing(arm_->getRobotModel(), "arm_1");
    timing.addSuffixWayPoint(start, 0.0);
    // Replace the old first point rather than visiting it and returning to the
    // next dense point. The replacement connection is checked below as well.
    for (size_t i = 1; i < motion.points.size(); ++i) {
      const auto & waypoint = motion.points[i];
      auto next = start;next.setVariablePositions(motion.joint_names, waypoint.positions);next.update();
      timing.addSuffixWayPoint(next, 0.01);
    }
    const auto endpoint = timing.getLastWayPoint().getGlobalLinkTransform(tcp_);
    require(timeWithRetry(timing, 0.10, false), "Continuation timing failed");
    const auto & first = timing.getFirstWayPoint();
    for (const auto & name : group->getVariableNames()) {
      require(std::abs(first.getVariablePosition(name) - start.getVariablePosition(name)) < 0.001,
        "Continuation timing changed measured start");
    }
    const auto actual_end = timing.getLastWayPoint().getGlobalLinkTransform(tcp_);
    require((actual_end.translation() - endpoint.translation()).norm() < 0.00005 &&
      Eigen::Quaterniond(actual_end.linear()).angularDistance(Eigen::Quaterniond(endpoint.linear())) < 0.001,
      "Continuation timing changed endpoint");
    Group::Plan result;timing.getRobotTrajectoryMsg(result.trajectory);
    limitCompartmentCarryRotation(start, result);
    require(compartmentTransferPlanValid(scene, start, result, relative, angle),
      "Continuation invalid from measured state; holding box");
    RCLCPP_INFO(node_->get_logger(),
      "COMPARTMENT CONTINUATION SYNC PASSED: start_delta=%.6f rad; measured start rechecked",
      delta);
    return result;
  }

  bool planCompartmentTableRoute(const planning_scene::PlanningScene & scene,
    const moveit::core::RobotState & start, const Pose & target,
    const Eigen::Vector3d & relative, const Eigen::Quaterniond & angle,
    std::vector<Group::Plan> & route, Wall::time_point deadline, const char * phase)
  {
    const auto * group = start.getJointModelGroup("arm_1");
    if (std::string(phase) == "measured" && !compartment_table_route_.empty()) {
      auto current = start;
      std::vector<Group::Plan> checked;
      bool accepted = true;
      for (const auto & saved : compartment_table_route_) {
        const auto & motion = saved.trajectory.joint_trajectory;
        if (motion.points.empty()) {accepted = false;break;}
        auto first = current;
        first.setVariablePositions(motion.joint_names, motion.points.front().positions);
        first.update();
        double delta = 0;
        for (const auto & name : group->getVariableNames()) {
          delta = std::max(delta, std::abs(first.getVariablePosition(name) -
              current.getVariablePosition(name)));
        }
        if (delta > 0.03) {accepted = false;break;}
        robot_trajectory::RobotTrajectory timing(arm_->getRobotModel(), "arm_1");
        timing.addSuffixWayPoint(current, 0.0);
        for (const auto & waypoint : motion.points) {
          auto next = current;
          next.setVariablePositions(motion.joint_names, waypoint.positions);next.update();
          timing.addSuffixWayPoint(next, 0.01);
        }
        if (!timeWithRetry(timing, 0.10, false)) {accepted = false;break;}
        Group::Plan leg;timing.getRobotTrajectoryMsg(leg.trajectory);
        limitCompartmentCarryRotation(current, leg);
        if (!compartmentTransferPlanValid(scene, current, leg, relative, angle)) {
          accepted = false;break;
        }
        current = timing.getLastWayPoint();checked.push_back(leg);
      }
      const auto endpoint = current.getGlobalLinkTransform(tcp_);
      const double error = (endpoint.translation() - point(target)).norm();
      const double turn = Eigen::Quaterniond(endpoint.linear()).angularDistance(rotation(target));
      if (accepted && !checked.empty() && error < 0.003 && turn < 0.025) {
        route = std::move(checked);
        RCLCPP_INFO(node_->get_logger(),
          "COMPARTMENT TABLE ROUTE REUSED: measured checks passed legs=%zu endpoint_error=%.3f mm",
          route.size(), error * 1000);
        return true;
      }
      RCLCPP_INFO(node_->get_logger(),
        "COMPARTMENT TABLE ROUTE REUSE REJECTED: replanning from measured grasp");
    }
    const auto valid = collisionCheck(scene);
    Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
    goal.translation() = point(target);goal.linear() = rotation(target).toRotationMatrix();
    arm_->setPlanningTime(1.5);arm_->setNumPlanningAttempts(1);
    arm_->setMaxVelocityScalingFactor(0.10);arm_->setMaxAccelerationScalingFactor(0.10);
    const auto restore = [this]() {
        arm_->setPlanningTime(10.0);arm_->setNumPlanningAttempts(3);
        arm_->setStartStateToCurrentState();arm_->clearPoseTargets();
      };
    for (double distance : {0.0, 0.08, 0.16}) {
      if (Wall::now() >= deadline) {break;}
      auto outside = start;std::vector<Group::Plan> candidate_route;
      if (distance > 0) {
        Pose bridge;const auto tcp = start.getGlobalLinkTransform(tcp_);
        bridge.position.x = tcp.translation().x();bridge.position.y = tcp.translation().y() + distance;
        bridge.position.z = tcp.translation().z();
        const Eigen::Quaterniond q(tcp.linear());
        bridge.orientation.w = q.w();bridge.orientation.x = q.x();
        bridge.orientation.y = q.y();bridge.orientation.z = q.z();
        std::vector<moveit::core::RobotStatePtr> path;
        if (!compartmentPath(scene, start, bridge, path, "table_bridge")) {continue;}
        robot_trajectory::RobotTrajectory timing(arm_->getRobotModel(), "arm_1");
        for (const auto & waypoint : path) {timing.addSuffixWayPoint(*waypoint, 0.01);}
        if (!timeWithRetry(timing, 0.05, true)) {continue;}
        Group::Plan bridge_plan;timing.getRobotTrajectoryMsg(bridge_plan.trajectory);
        if (!compartmentTransferPlanValid(scene, start, bridge_plan, relative, angle)) {continue;}
        outside = timing.getLastWayPoint();candidate_route.push_back(bridge_plan);
      }
      const bool measured_phase = std::string(phase) == "measured";
      for (int trial = 0; trial < (measured_phase ? 12 : 3) && Wall::now() < deadline; ++trial) {
        auto finish = outside;
        if (measured_phase && trial == 0 && !table_goal_seed_.empty()) {
          finish.setJointGroupPositions(group, table_goal_seed_);finish.update();
        } else if (trial > 0) {finish.setToRandomPositions(group);}
        if (!finish.setFromIK(group, goal, tcp_, 0.08, valid)) {continue;}
        std::vector<double> joints;finish.copyJointGroupPositions(group, joints);
        arm_->setStartState(outside);
        if (!arm_->setJointValueTarget(joints)) {continue;}
        Group::Plan transfer;
        if (arm_->plan(transfer) != moveit::core::MoveItErrorCode::SUCCESS) {continue;}
        limitCompartmentCarryRotation(outside, transfer);
        if (!compartmentTransferPlanValid(scene, outside, transfer, relative, angle)) {continue;}
        candidate_route.push_back(transfer);route = std::move(candidate_route);
        if (!measured_phase) {finish.copyJointGroupPositions(group, table_goal_seed_);}
        restore();
        RCLCPP_INFO(node_->get_logger(),
          "COMPARTMENT TABLE ROUTE PASSED: phase=%s bridge=%.0f mm legs=%zu",
          phase, distance * 1000, route.size());
        return true;
      }
    }
    restore();
    RCLCPP_WARN(node_->get_logger(), "COMPARTMENT TABLE ROUTE REJECTED: phase=%s", phase);
    return false;
  }

  void limitCompartmentCarryRotation(const moveit::core::RobotState & start, Group::Plan & plan)
  {
    const double limit = node_->get_parameter_or<double>("compartment_table_angular_limit", 0.0);
    if (limit == 0) {return;}
    require(std::isfinite(limit) && limit >= 0.05 && limit <= 1.0,
      "Invalid compartment carry angular limit");
    auto & motion = plan.trajectory.joint_trajectory;
    auto state = start;Eigen::Quaterniond previous = Eigen::Quaterniond::Identity();
    double peak = 0, previous_time = 0;
    for (size_t i = 0; i < motion.points.size(); ++i) {
      const auto & p = motion.points[i];
      state.setVariablePositions(motion.joint_names, p.positions);state.update();
      const Eigen::Quaterniond q(state.getGlobalLinkTransform(tcp_).linear());
      const double time = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;
      if (i > 0 && time > previous_time) {
        peak = std::max(peak, previous.angularDistance(q) / (time - previous_time));
      }
      previous = q;previous_time = time;
    }
    const double factor = std::max(1.0, peak / limit);
    // Uniform time scaling keeps every geometric waypoint and only slows motion.
    for (auto & p : motion.points) {
      const int64_t ns = std::llround((p.time_from_start.sec * 1e9 +
          p.time_from_start.nanosec) * factor);
      p.time_from_start.sec = static_cast<int32_t>(ns / 1000000000);
      p.time_from_start.nanosec = static_cast<uint32_t>(ns % 1000000000);
      for (auto & v : p.velocities) {v /= factor;}
      for (auto & a : p.accelerations) {a /= factor * factor;}
    }
    RCLCPP_INFO(node_->get_logger(),
      "CARRY ANGULAR LIMIT: sampled_peak=%.4f rad/s limited_peak=%.4f rad/s time_scale=%.4f",
      peak, peak / factor, factor);
  }

  void compartmentWithdraw(const Pose & via, const Pose & target, const Eigen::Vector3d & reference,
    const Eigen::Quaterniond & reference_angle)
  {
    planning_scene::PlanningScene scene(arm_->getRobotModel());
    require(scene.setPlanningSceneMsg(planningSceneSnapshot()), "Withdrawal scene unavailable");
    auto measured = arm_->getCurrentState(2.0);
    require(static_cast<bool>(measured), "Withdrawal measured state unavailable");
    auto start = scene.getCurrentState();
    const auto * group = start.getJointModelGroup("arm_1");
    for (const auto & name : group->getVariableNames()) {
      start.setVariablePosition(name, measured->getVariablePosition(name));
    }
    start.update();
    std::vector<moveit::core::RobotStatePtr> path, up;
    if ((start.getGlobalLinkTransform(tcp_).translation() - point(via)).norm() > 0.003) {
      require(compartmentPath(scene, start, via, up, "physical_withdraw_raise"),
        "No complete withdrawal raise from measured grasp; holding box");
      start = *up.back();
    }
    require(compartmentPath(scene, start, target, path, "physical_withdraw"),
      "No complete withdrawal from measured grasp; holding box");
    path.insert(path.begin(), up.begin(), up.end());
    robot_trajectory::RobotTrajectory trajectory(arm_->getRobotModel(), "arm_1");
    for (const auto & waypoint : path) {trajectory.addSuffixWayPoint(*waypoint, 0.0);}
    require(timeWithRetry(trajectory, 0.05, true), "Withdrawal timing failed");
    for (size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
      const auto & waypoint = trajectory.getWayPoint(i);
      for (const auto & name : group->getVariableNames()) {
        const auto & bounds = arm_->getRobotModel()->getVariableBounds(name);
        const double value = waypoint.getVariablePosition(name);
        require(value >= bounds.min_position_ + 0.05 && value <= bounds.max_position_ - 0.05,
          "Timed withdrawal lost joint margin");
      }
      require(!scene.isStateColliding(waypoint), "Timed withdrawal collision; holding box");
    }
    const auto & endpoint = trajectory.getLastWayPoint().getGlobalLinkTransform(tcp_);
    require((endpoint.translation() - point(target)).norm() < 0.00005 &&
      Eigen::Quaterniond(endpoint.rotation()).angularDistance(rotation(target)) < 0.025,
      "Timed withdrawal endpoint diverged");
    moveit_msgs::msg::RobotTrajectory msg;trajectory.getRobotTrajectoryMsg(msg);
    executeMonitored(msg, "COMPARTMENT WITHDRAW", reference, reference_angle);
  }

  // Compare complete transfer plans with a collision-checked prospective approach.
  // The second plan is a score/probe only: after moving, placement is recomputed
  // from the physical grasp, so measured slip never reuses a stale approach plan.
  bool planCoupledTableTransfer(
    const Pose & table, const Eigen::Vector3d & relative,
    const Eigen::Quaterniond & relative_angle, Group::Plan & selected)
  {
    planning_scene::PlanningScene scene(arm_->getRobotModel());
    require(scene.setPlanningSceneMsg(planningSceneSnapshot()), "Coupled scene import failed");
    auto measured = arm_->getCurrentState(2.0);
    require(static_cast<bool>(measured), "Coupled measured state unavailable");
    const auto * group = measured->getJointModelGroup("arm_1");
    auto initial = scene.getCurrentState();
    for (const auto & name : group->getVariableNames()) {
      initial.setVariablePosition(name, measured->getVariablePosition(name));
    }
    initial.update();
    const auto valid = collisionCheck(scene);
    Eigen::Isometry3d table_target = Eigen::Isometry3d::Identity();
    table_target.translation() = point(table);
    table_target.linear() = rotation(table).toRotationMatrix();
    const double half = std::sqrt(0.5);
    const Eigen::Quaterniond aligned = Eigen::Quaterniond(half, half, 0, 0) *
      relative_angle.inverse();
    Eigen::Isometry3d pre = Eigen::Isometry3d::Identity();
    pre.linear() = aligned.toRotationMatrix();
    pre.translation() = Eigen::Vector3d(0, 0.05, 0.50) - aligned * relative;
    Pose descent;
    descent.position.x = pre.translation().x();descent.position.y = pre.translation().y();
    descent.position.z = pre.translation().z() - 0.078;
    descent.orientation.w = aligned.w();descent.orientation.x = aligned.x();
    descent.orientation.y = aligned.y();descent.orientation.z = aligned.z();
    const auto duration = [](const Group::Plan & plan) {
        const auto & points = plan.trajectory.joint_trajectory.points;
        if (points.empty()) {return std::numeric_limits<double>::infinity();}
        const auto & t = points.back().time_from_start;
        return static_cast<double>(t.sec) + t.nanosec * 1e-9;
      };
    bool found = false;
    double best = std::numeric_limits<double>::infinity();
    auto deadline = Wall::now() + 12s;
    arm_->setPlanningTime(2.0);
    for (int candidate = 0; candidate < 6 && Wall::now() < deadline; ++candidate) {
      auto above = initial;
      if (candidate > 0) {above.setToRandomPositions(group);}
      if (!above.setFromIK(group, table_target, tcp_, 0.08, valid)) {continue;}
      std::vector<double> joints;
      above.copyJointGroupPositions(group, joints);
      arm_->setMaxVelocityScalingFactor(0.10);arm_->setMaxAccelerationScalingFactor(0.10);
      arm_->setStartState(initial);
      if (!arm_->setJointValueTarget(joints)) {continue;}
      Group::Plan transfer;
      if (arm_->plan(transfer) != moveit::core::MoveItErrorCode::SUCCESS ||
        !carryClearance(transfer, relative, relative_angle, "COUPLED TRANSFER")) {continue;}
      const auto & motion = transfer.trajectory.joint_trajectory;
      if (motion.points.empty()) {continue;}
      auto finish = initial;
      finish.setVariablePositions(motion.joint_names, motion.points.back().positions);
      finish.update();
      auto preplace = finish;
      if (!preplace.setFromIK(group, pre, tcp_, 0.08, valid)) {continue;}
      std::vector<moveit::core::RobotStatePtr> descent_path;
      if (!checkedFinePath(scene, preplace, descent, descent_path)) {continue;}
      preplace.copyJointGroupPositions(group, joints);
      arm_->setStartState(finish);
      arm_->setMaxVelocityScalingFactor(0.05);arm_->setMaxAccelerationScalingFactor(0.05);
      if (!arm_->setJointValueTarget(joints)) {continue;}
      Group::Plan approach;
      if (arm_->plan(approach) != moveit::core::MoveItErrorCode::SUCCESS ||
        !carryClearance(approach, relative, relative_angle, "COUPLED APPROACH")) {continue;}
      const double score = duration(transfer) + duration(approach);
      RCLCPP_INFO(node_->get_logger(),
        "TABLE COUPLED SCORE: candidate=%d transfer=%.3f s approach=%.3f s total=%.3f s",
        candidate + 1, duration(transfer), duration(approach), score);
      if (!found) {deadline = std::min(deadline, Wall::now() + 3s);}
      if (score < best) {best = score;selected = transfer;}
      found = true;
    }
    arm_->setPlanningTime(10.0);
    arm_->setMaxVelocityScalingFactor(0.10);arm_->setMaxAccelerationScalingFactor(0.10);
    arm_->setStartStateToCurrentState();arm_->clearPoseTargets();
    if (found) {
      RCLCPP_INFO(node_->get_logger(), "TABLE COUPLED SELECTED: predicted_total=%.3f s", best);
    } else {
      RCLCPP_WARN(node_->get_logger(), "TABLE COUPLED FALLBACK: using original transfer planner");
    }
    return found;
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
  bool supported()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require(!robot_table_contact_, "Robot contacted tabletop; placement failed");
    require(!unexpected_box_contact_, "Box contacted a robot link outside the fingers");
    return support_seen_ && table_support_seen_ && Wall::now() - table_support_wall_ < 1s &&
           Wall::now() - support_wall_ < 1s &&
           node_->now().nanoseconds() - support_sim_ < 250000000;
  }
  void syncHeldBox(const Pose & box, const Pose & tcp)
  {
    const Eigen::Vector3d relative = rotation(tcp).inverse() * (point(box) - point(tcp));
    const Eigen::Quaterniond q = rotation(tcp).inverse() * rotation(box);
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = tcp_;attached.touch_links =
    {"left_fr3v2_leftfinger", "left_fr3v2_rightfinger", "left_fr3v2_hand"};
    attached.object.id = "target_box";attached.object.header.frame_id = tcp_;
    attached.object.operation = attached.object.ADD;
    shape_msgs::msg::SolidPrimitive primitive;primitive.type = primitive.BOX;
    primitive.dimensions = {0.04, 0.04, 0.04};
    Pose p;p.position.x = relative.x();p.position.y = relative.y();p.position.z = relative.z();
    p.orientation.w = q.w();p.orientation.x = q.x();p.orientation.y = q.y();p.orientation.z = q.z();
    attached.object.primitives.push_back(primitive);attached.object.primitive_poses.push_back(p);
    require(scene_->applyAttachedCollisionObject(attached), "Cannot synchronize held planning box");
  }
  // Diagnostic only: which part of supported() is currently false.
  std::string supportDiagnosis()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto wall = Wall::now();
    auto age = [&](const Wall::time_point & t) {
        return std::chrono::duration<double>(wall - t).count();
      };
    char text[256];
    std::snprintf(text, sizeof(text),
      "box_sensor(seen=%d wall_age=%.2fs sim_age=%.2fs) table_sensor(seen=%d wall_age=%.2fs)",
      support_seen_, support_seen_ ? age(support_wall_) : -1.0,
      support_seen_ ? (node_->now().nanoseconds() - support_sim_) * 1e-9 : -1.0,
      table_support_seen_, table_support_seen_ ? age(table_support_wall_) : -1.0);
    return text;
  }
  void waitSupport(bool reseat = false)
  {
    auto deadline = Wall::now() + 15s;
    int64_t since = -1, hover_since = -1;int reseats = 0;
    int samples = 0, resets = 0, no_contact = 0, bad_gap = 0, bad_tilt = 0;
    double last_gap = 0, last_tilt = 0;
    while (Wall::now() < deadline) {
      const auto box = freshBox();
      const bool contact = supported();
      const bool gap_ok = std::abs(bottom(box) - 0.4) < 0.001;
      const bool tilt_ok = tilt(box) < 0.00873;
      const bool good = contact && gap_ok && tilt_ok;
      ++samples;no_contact += !contact;bad_gap += !gap_ok;bad_tilt += !tilt_ok;
      if (!good && since >= 0) {
        ++resets;
        RCLCPP_WARN(node_->get_logger(),
          "SUPPORT RESET: contact=%d gap=%.3f mm tilt=%.3f deg %s", contact,
          (bottom(box) - 0.4) * 1000, tilt(box) * 180 / M_PI, supportDiagnosis().c_str());
      }
      last_gap = bottom(box) - 0.4;last_tilt = tilt(box);
      const auto now = node_->now().nanoseconds();
      // The held box can rebound and hang a few micrometres above the table after the
      // press; it cannot drop by itself. If it hovers without contact for 1 sim s, make a
      // bounded small re-press (gap + 0.05 mm, at most 0.1 mm, at most twice; gripper closed).
      const bool hovering = !contact && tilt_ok && last_gap > -0.00005 && last_gap < 0.00015;
      hover_since = hovering ? (hover_since < 0 ? now : hover_since) : -1;
      if (reseat && hovering && reseats < 2 && now - hover_since >= 1000000000) {
        const double step_down = std::min(0.0001, last_gap + 0.00005);
        ++reseats;
        RCLCPP_WARN(node_->get_logger(),
          "SUPPORT RESEAT %d/2: no contact for 1 sim s at gap=%.3f mm, descending %.3f mm",
          reseats, last_gap * 1000, step_down * 1000);
        auto goal = arm_->getCurrentPose(tcp_).pose;goal.position.z -= step_down;
        linear(goal, 0.01);waitAt(&goal, 0.00015, 0.0015);
        since = -1;hover_since = -1;deadline = Wall::now() + 15s;
        continue;
      }
      if (!good) {since = -1;} else if (since < 0) {
        since = now;
      } else if (now - since >= 1000000000) {
        RCLCPP_INFO(node_->get_logger(),
            "SUPPORT VERIFIED: 1 sim s table contact, bottom_gap=%.3f mm (samples=%d resets=%d)",
          (bottom(box) - 0.4) * 1000, samples, resets);return;
      }
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_ERROR(node_->get_logger(),
      "SUPPORT DIAG: samples=%d resets=%d no_contact=%d bad_gap=%d bad_tilt=%d "
      "last gap=%.3f mm tilt=%.3f deg %s", samples, resets, no_contact, bad_gap, bad_tilt,
      last_gap * 1000, last_tilt * 180 / M_PI, supportDiagnosis().c_str());
    throw std::runtime_error("Table contact/level support not verified; refusing gripper open");
  }
  Group::Plan planTableApproach(
    const Pose & preplace, const Pose & descent, bool alternate,
    const Eigen::Vector3d & relative, const Eigen::Quaterniond & relative_angle)
  {
    planning_scene::PlanningScene scene(arm_->getRobotModel());
    require(scene.setPlanningSceneMsg(planningSceneSnapshot()), "Table probe scene import failed");
    const auto * group = scene.getRobotModel()->getJointModelGroup("arm_1");
    const auto valid = collisionCheck(scene);
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = point(preplace);
    target.linear() = rotation(preplace).toRotationMatrix();
    // Planning only: if every candidate fails, search once more with fresh random
    // IK seeds before giving up. The arm has not moved, so this is motion-free.
    for (int pass = 0; pass < 2; ++pass) {
      if (pass > 0) {
        RCLCPP_WARN(node_->get_logger(),
          "TABLE APPROACH SEARCH RETRY: no candidate in pass 1, searching again (planning only)");
      }
      for (int attempt = 0; attempt < 12; ++attempt) {
        auto state = scene.getCurrentState();
        if (attempt > 0 || pass > 0) {
          state.setToRandomPositions(group);
        }
        if (!state.setFromIK(group, target, tcp_, 0.08, valid)) {
          continue;
        }
        if (alternate && std::none_of(group->getVariableNames().begin(),
            group->getVariableNames().end(), [&state, &scene](const std::string & joint) {
            return std::abs(state.getVariablePosition(joint) -
                scene.getCurrentState().getVariablePosition(joint)) >= 0.1;
            }))
        {
          continue;
        }
        std::vector<moveit::core::RobotStatePtr> path;
        if (!checkedFinePath(scene, state, descent, path)) {
          continue;
        }
        if (node_->has_parameter("source_xyz") &&
          !tableRetreatProbe(scene, *path.back(), descent, relative_angle)) {continue;}
        std::vector<double> joints;
        state.copyJointGroupPositions(group, joints);
        if (!arm_->setJointValueTarget(joints)) {
          continue;
        }
        // Replan the same (nearby) posture before moving to a random IK seed:
        // a random posture can mean a long reconfiguration that rotates the held box.
        Group::Plan approach;bool planned = false;
        for (int replan = 0; replan < 5 && !planned; ++replan) {
          setPlanningStartState();
          planned = arm_->plan(approach) == moveit::core::MoveItErrorCode::SUCCESS &&
            carryClearance(approach, relative, relative_angle, "TABLE APPROACH");
          if (!planned) {
            RCLCPP_WARN(node_->get_logger(),
              "TABLE APPROACH REPLAN %d/5: candidate=%d plan rejected", replan + 1, attempt + 1);
          }
        }
        if (planned) {
          RCLCPP_INFO(node_->get_logger(),
            "TABLE DESCENT PROBE PASSED: pass=%d candidate=%d points=%zu",
            pass + 1, attempt + 1, path.size());
          return approach;
        }
      }
    }
    throw std::runtime_error("No table approach with a complete collision-checked descent");
  }

  bool tableRetreatProbe(const planning_scene::PlanningScene & scene,
    const moveit::core::RobotState & descent_state, const Pose & descent,
    const Eigen::Quaterniond & relative_angle)
  {
    auto released = std::make_shared<planning_scene::PlanningScene>(scene.getRobotModel());
    moveit_msgs::msg::PlanningScene snapshot;scene.getPlanningSceneMsg(snapshot);
    require(released->setPlanningSceneMsg(snapshot), "Retreat probe scene copy failed");
    released->getAllowedCollisionMatrixNonConst().setEntry("target_box", "table_link", true);
    Pose seated = descent;seated.position.z -= 0.002;
    std::vector<moveit::core::RobotStatePtr> seating;
    if (!checkedFinePath(*released, descent_state, seated, seating)) {return false;}
    auto open = *seating.back();open.clearAttachedBody("target_box");
    open.setVariablePosition("left_fr3v2_finger_joint1", 0.04);
    open.setVariablePosition("left_fr3v2_finger_joint2", 0.04);open.update();
    released->setCurrentState(open);
    moveit_msgs::msg::CollisionObject cube;cube.id = "target_box";
    cube.header.frame_id = "world";cube.operation = cube.ADD;
    shape_msgs::msg::SolidPrimitive shape;shape.type = shape.BOX;shape.dimensions = {0.04, 0.04, 0.04};
    cube.primitives.push_back(shape);
    Pose box;box.position.x = 0;box.position.y = 0.05;box.position.z = 0.42;
    const Eigen::Quaterniond q = rotation(seated) * relative_angle;
    box.orientation.w = q.w();box.orientation.x = q.x();
    box.orientation.y = q.y();box.orientation.z = q.z();cube.primitive_poses.push_back(box);
    if (!released->processCollisionObjectMsg(cube)) {return false;}
    Pose retreat = seated;retreat.position.y -= 0.12;
    robot_trajectory::RobotTrajectory trajectory(arm_->getRobotModel(), "arm_1");
    if (!tableRetreatTrajectory(*released, open, retreat, trajectory, 0.10)) {
      RCLCPP_INFO(node_->get_logger(), "TABLE RETREAT PROBE REJECTED: searching another preplace posture");
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "TABLE RETREAT PROBE PASSED: points=%zu",
      trajectory.getWayPointCount());
    return true;
  }

  // Use one interpolation and timing policy for the preplace probe and the
  // measured, open-gripper retreat. Never execute the earlier probe trajectory.
  bool tableRetreatTrajectory(const planning_scene::PlanningScene & scene,
    const moveit::core::RobotState & start, const Pose & target,
    robot_trajectory::RobotTrajectory & trajectory, double required_margin = 0.05)
  {
    std::vector<moveit::core::RobotStatePtr> path;
    if (!checkedFinePath(scene, start, target, path, true)) {return false;}
    for (const auto & waypoint : path) {trajectory.addSuffixWayPoint(*waypoint, 0.0);}
    if (!timeWithRetry(trajectory, 0.10, true)) {return false;}
    const auto * group = start.getJointModelGroup("arm_1");
    const auto valid = collisionCheck(scene);
    double margin = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
      auto state = trajectory.getWayPoint(i);
      std::vector<double> values;state.copyJointGroupPositions(group, values);
      if (!valid(&state, group, values.data())) {return false;}
      for (const auto & name : group->getVariableNames()) {
        const auto & bounds = state.getRobotModel()->getVariableBounds(name);
        const double value = state.getVariablePosition(name);
        margin = std::min(margin, std::min(value - bounds.min_position_, bounds.max_position_ - value));
      }
    }
    if (margin < required_margin) {
      RCLCPP_INFO(node_->get_logger(),
        "TABLE RETREAT RESERVE REJECTED: margin=%.6f required=%.6f rad", margin, required_margin);
      return false;
    }
    const auto & endpoint = trajectory.getLastWayPoint().getGlobalLinkTransform(tcp_);
    if ((endpoint.translation() - point(target)).norm() >= 0.00005 ||
      Eigen::Quaterniond(endpoint.linear()).angularDistance(rotation(target)) >= 0.025) {return false;}
    RCLCPP_INFO(node_->get_logger(), "TABLE RETREAT TIMED CHECK PASSED: joint_margin=%.6f rad points=%zu",
      margin, trajectory.getWayPointCount());
    return true;
  }

  void tableRetreat(const Pose & target)
  {
    planning_scene::PlanningScene scene(arm_->getRobotModel());
    require(scene.setPlanningSceneMsg(planningSceneSnapshot()), "Retreat scene unavailable");
    auto start = scene.getCurrentState();
    const auto measured = arm_->getCurrentState(2.0);
    require(static_cast<bool>(measured), "Retreat measured state unavailable");
    for (const auto & name : start.getJointModelGroup("arm_1")->getVariableNames()) {
      start.setVariablePosition(name, measured->getVariablePosition(name));
    }
    start.update();
    robot_trajectory::RobotTrajectory trajectory(arm_->getRobotModel(), "arm_1");
    require(tableRetreatTrajectory(scene, start, target, trajectory),
      "No checked fine table retreat from measured release; refusing motion");
    moveit_msgs::msg::RobotTrajectory motion;trajectory.getRobotTrajectoryMsg(motion);
    carrySafe();
    require(arm_->execute(motion) == moveit::core::MoveItErrorCode::SUCCESS,
      "Table retreat execution failed");
  }
  bool tryTableDescent(
    const Eigen::Vector3d & reference, const Eigen::Quaterniond & reference_angle, bool alternate,
    Pose & goal)
  {
    const auto box = stableBox();const auto tcp = arm_->getCurrentPose(tcp_).pose;
    syncHeldBox(box, tcp);
    const Eigen::Vector3d relative = rotation(tcp).inverse() * (point(box) - point(tcp));
    const Eigen::Quaterniond relative_angle = rotation(tcp).inverse() * rotation(box);
    // A cube can rest on any face. Rotate 90 degrees about world X so the
    // fingers approach from above and the wrist stays clear of the tabletop.
    const double half = std::sqrt(0.5);
    const Eigen::Quaterniond desired_box(half, half, 0, 0);
    const Eigen::Quaterniond aligned = desired_box * relative_angle.inverse();
    goal = tcp;
    goal.orientation.w = aligned.w();goal.orientation.x = aligned.x();
    goal.orientation.y = aligned.y();goal.orientation.z = aligned.z();
    const Eigen::Vector3d pre_center(0, 0.05, 0.50);
    const Eigen::Vector3d pre_tcp = pre_center - aligned * relative;
    goal.position.x = pre_tcp.x();goal.position.y = pre_tcp.y();goal.position.z = pre_tcp.z();
    RCLCPP_INFO(node_->get_logger(), "PLACE TOP-DOWN ALIGN AND APPROACH: box center z=0.500");
    arm_->setGoalPositionTolerance(0.00015);arm_->setGoalOrientationTolerance(0.0005);
    arm_->setMaxVelocityScalingFactor(0.05);arm_->setMaxAccelerationScalingFactor(0.05);
    Pose descent = goal;
    descent.position.z += 0.422 - pre_center.z();
    arm_->setGoalJointTolerance(0.00005);
    const auto approach = planTableApproach(goal, descent, alternate, relative, relative_angle);
    executeMonitored(approach.trajectory, "TABLE APPROACH", reference, reference_angle);
    waitAt(&goal, 0.0003, 0.0015);
    {
      // Let the held box stop swinging before judging its tilt (limit unchanged).
      const double settled_tilt = tilt(stableBox());
      RCLCPP_INFO(node_->get_logger(), "PREPLACE TILT: %.3f deg (limit 0.500)",
        settled_tilt * 180 / M_PI);
      char message[96];
      std::snprintf(message, sizeof(message),
        "Box not parallel enough for descent (tilt=%.3f deg)", settled_tilt * 180 / M_PI);
      require(settled_tilt < 0.00873, message);
    }
    double drift = 0, angle = 0;
    holdTransferred("PREPLACE", reference, reference_angle, drift, angle, 1.0);
    auto current_box = freshBox();auto current_tcp = arm_->getCurrentPose(tcp_).pose;
    syncHeldBox(current_box, current_tcp);
    goal.position.x = current_tcp.position.x - current_box.position.x;
    goal.position.y = current_tcp.position.y + 0.05 - current_box.position.y;
    goal.position.z = current_tcp.position.z + 0.422 - current_box.position.z;
    if (node_->has_parameter("source_xyz")) {
      // The measured grasp correction can change the IK branch near a limit.
      // Recheck from the actual approach state before moving down or releasing.
      planning_scene::PlanningScene corrected(arm_->getRobotModel());
      require(corrected.setPlanningSceneMsg(planningSceneSnapshot()),
        "Corrected descent scene unavailable");
      std::vector<moveit::core::RobotStatePtr> descent_path;
      if (!checkedFinePath(corrected, corrected.getCurrentState(), goal, descent_path, true) ||
        !tableRetreatProbe(corrected, *descent_path.back(), goal, relative_angle)) {
        RCLCPP_WARN(node_->get_logger(),
          "TABLE MEASURED RETREAT PROBE REJECTED: retrying another preplace posture before descent");
        return false;
      }
      RCLCPP_INFO(node_->get_logger(), "TABLE MEASURED RETREAT PROBE PASSED");
    }
    {std::lock_guard<std::mutex> lock(mutex_);seating_ = true;}  // tabletop contact allowed now
    RCLCPP_INFO(node_->get_logger(), "PLACE DESCENT: box center z=0.422, then contact steps");
    try {
      linear(goal, 0.02, true);
    } catch (const FinePathRejected & error) {
      RCLCPP_WARN(node_->get_logger(), "TABLE DESCENT PLAN REJECTED: %s", error.what());
      return false;
    }
    waitAt(&goal, 0.0003, 0.0015);
    return true;
  }
  void place(const Eigen::Vector3d & reference, const Eigen::Quaterniond & reference_angle)
  {
    Pose goal;
    if (!tryTableDescent(reference, reference_angle, false, goal)) {
      require(!supported(), "Unexpected table support before descent retry");
      RCLCPP_WARN(node_->get_logger(),
          "TABLE DESCENT RETRY: planning another preplace joint posture");
      require(tryTableDescent(reference, reference_angle, true, goal),
        "Table descent rejected again; gripper stays closed");
      RCLCPP_INFO(node_->get_logger(), "TABLE DESCENT RETRY PASSED");
    }
    const auto aligned = rotation(goal);
    // Remain closed on the 40 mm cube, but remove most of the commanded
    // overclosure before the last 2 mm of seating. Both fingers stay below
    // 20 mm in the command; this does not request a clearance around the cube.
    const auto before_relax = freshBox();
    gripper(0.0198, true);
    auto current_box = stableBox();auto current_tcp = arm_->getCurrentPose(tcp_).pose;
    RCLCPP_INFO(node_->get_logger(),
      "SEAT GRIP RELAX: shift=%.3f mm bottom_gap=%.3f mm box=(%.5f, %.5f, %.5f)",
      (point(current_box) - point(before_relax)).norm() * 1000,
      (bottom(current_box) - 0.4) * 1000, current_box.position.x,
      current_box.position.y, current_box.position.z);
    require((point(current_box) - point(before_relax)).norm() < 0.003 &&
      bottom(current_box) - 0.4 > -0.0003, "Box moved excessively while reducing overclosure");
    syncHeldBox(current_box, current_tcp);
    allowFingers(true);  // Only box-table support; robot-table collisions remain checked.
    bool seated = false;int nudges = 0;
    for (int step = 0; step < 10; ++step) {
      current_box = freshBox();
      if (tilt(current_box) >= 0.00873) {
        // A single sample right after a contact step can catch the box mid-swing.
        // Let it settle, then judge (limit unchanged).
        RCLCPP_WARN(node_->get_logger(), "SEAT TILT TRANSIENT: %.3f deg, waiting for box to settle",
          tilt(current_box) * 180 / M_PI);
        current_box = stableBox();
        char message[96];
        std::snprintf(message, sizeof(message), "Box tilted during seating (tilt=%.3f deg)",
          tilt(current_box) * 180 / M_PI);
        require(tilt(current_box) < 0.00873, message);
      }
      const double gap = bottom(current_box) - 0.4;
      const bool contact = supported();
      RCLCPP_INFO(node_->get_logger(), "SEAT step=%d bottom_gap=%.3f mm contact=%d",
        step, gap * 1000, contact);
      require(gap > -0.0003, "Box penetration too large; refusing further descent");
      // Seated only when in contact AND lightly pressed (passing runs: -0.002..-0.005 mm).
      // A corner touch can report contact while the box still hangs slightly above the
      // table (it then pivots in the fingers and contact disappears), so contact alone at a
      // positive gap is not accepted.
      if (contact && gap <= -0.000001) {seated = true;break;}
      double step_down = std::min(0.0005, gap + 0.0001);
      if (gap <= 0.00015) {
        // The held box cannot fall onto the table by itself. Make a bounded final small
        // descent (gap + 0.1 mm, at most 0.2 mm, at most 3 times) instead of only waiting.
        if (nudges >= 3 || gap < -0.00005) {waitSupport(true);seated = true;break;}
        ++nudges;step_down = std::min(0.0002, gap + 0.0001);
        RCLCPP_INFO(node_->get_logger(),
          "SEAT FINAL NUDGE %d/3: bottom_gap=%.3f mm contact=%d, descending %.3f mm",
          nudges, gap * 1000, contact, step_down * 1000);
      }
      goal = arm_->getCurrentPose(tcp_).pose;
      goal.orientation.w = aligned.w();goal.orientation.x = aligned.x();
      goal.orientation.y = aligned.y();goal.orientation.z = aligned.z();
      goal.position.z -= step_down;
      linear(goal, 0.01);waitAt(&goal, 0.00015, 0.0015);
    }
    require(seated, "Could not establish table support within bounded seating steps");
    waitSupport(true);  // pre-open only: re-press allowed while the gripper is closed
    const auto release_reference = freshBox();
    RCLCPP_INFO(node_->get_logger(), "OPEN ON TABLE: physical support verified, two-step opening");
    gripper(0.021, false);  // Remove squeeze gently before the full opening.
    waitSupport();
    const auto soft_box = freshBox();
    double release_slip = (point(soft_box) - point(release_reference)).norm();
    RCLCPP_INFO(node_->get_logger(), "SOFT OPEN: delta=%.3f mm box=(%.5f, %.5f, %.5f)",
      release_slip * 1000, soft_box.position.x, soft_box.position.y, soft_box.position.z);
    gripper(0.04, false);
    arm_->detachObject("target_box");
    const auto detach_deadline = Wall::now() + 5s;
    while (!scene_->getAttachedObjects({"target_box"}).empty() && Wall::now() < detach_deadline) {
      std::this_thread::sleep_for(100ms);
    }
    require(scene_->getAttachedObjects({"target_box"}).empty(), "Planning detach failed");
    {std::lock_guard<std::mutex> lock(mutex_);carrying_ = false;}
    const auto open_box = stableBox();
    release_slip = std::max(release_slip, (point(open_box) - point(release_reference)).norm());
    RCLCPP_INFO(node_->get_logger(), "BEFORE RETREAT: release_slip=%.3f mm box=(%.5f, %.5f, %.5f)",
      release_slip * 1000, open_box.position.x, open_box.position.y, open_box.position.z);
    addBox(open_box);
    goal = arm_->getCurrentPose(tcp_).pose;goal.position.y -= 0.12;
    RCLCPP_INFO(node_->get_logger(), "PLACE RETREAT: 120 mm horizontally, no ready return");
    if (node_->has_parameter("source_xyz")) {tableRetreat(goal);}
    else {linear(goal, 0.10);}
    waitAt(&goal);  // gripper open, box released
    const auto start = node_->now();const auto wall = Wall::now();
    double max_slip = release_slip;
    while ((node_->now() - start).seconds() < 3.0) {
      require(Wall::now() - wall < 30s, "Post-release simulation clock timed out");
      current_box = freshBox();
      max_slip = std::max(max_slip, (point(current_box) - point(release_reference)).norm());
      RCLCPP_INFO(node_->get_logger(),
        "POST RELEASE: box=(%.5f, %.5f, %.5f) xy=%.3f mm slip=%.3f mm tilt=%.3f deg",
        current_box.position.x, current_box.position.y, current_box.position.z,
        std::hypot(current_box.position.x, current_box.position.y - 0.05) * 1000,
        max_slip * 1000, tilt(current_box) * 180 / M_PI);
      require(supported(), "Box lost table support after release");
      require(max_slip < 0.002 && std::hypot(current_box.position.x,
        current_box.position.y - 0.05) < 0.003 &&
        std::abs(current_box.position.z - 0.42) < 0.001 && tilt(current_box) < 0.01745,
        "Released box moved or tilted outside placement tolerance");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(),
      "PLACE TEST PASSED: box=(%.5f, %.5f, %.5f) xy_error=%.3f mm release_slip=%.3f mm tilt=%.3f deg",
      current_box.position.x, current_box.position.y, current_box.position.z,
      std::hypot(current_box.position.x, current_box.position.y - 0.05) * 1000,
      max_slip * 1000, tilt(current_box) * 180 / M_PI);
  }

  void holdTransferred(
    const char * label, const Eigen::Vector3d & reference,
    const Eigen::Quaterniond & reference_angle, double & max_drift, double & max_angle,
    double seconds = 3.0)
  {
    const auto sim_start = node_->now();const auto wall_start = Wall::now();
    auto next_sample = wall_start;
    const bool progress_guard = node_->has_parameter("source_xyz");
    fr3_cooperative_control::SimulationHoldClock clock_guard;
    auto next_clock_log = wall_start + 5s;
    while (true) {
      const double sim_elapsed = (node_->now() - sim_start).seconds();
      const double wall_elapsed = std::chrono::duration<double>(Wall::now() - wall_start).count();
      if (progress_guard) {
        const auto status = clock_guard.observe(sim_elapsed, wall_elapsed, seconds);
        if (status == fr3_cooperative_control::HoldClockResult::complete) {break;}
        if (status != fr3_cooperative_control::HoldClockResult::waiting) {
          const auto reason = fr3_cooperative_control::SimulationHoldClock::name(status);
          double pose_age;
          {std::lock_guard<std::mutex> lock(mutex_);
            pose_age = std::chrono::duration<double>(Wall::now() - box_received_).count();}
          RCLCPP_ERROR(node_->get_logger(),
            "SIMULATION HOLD CLOCK FAILED: phase=%s reason=%s sim_elapsed=%.4f required=%.3f "
            "wall_elapsed=%.3f idle_wall=%.3f clock_publishers=%zu box_wall_age=%.3f",
            label, reason, sim_elapsed, seconds, wall_elapsed, clock_guard.idleWallSeconds(),
            node_->count_publishers("/clock"), pose_age);
          throw std::runtime_error(std::string(label) + ": simulation hold " + reason);
        }
        if (Wall::now() >= next_clock_log) {
          RCLCPP_INFO(node_->get_logger(),
            "SIMULATION HOLD PROGRESS: phase=%s sim_elapsed=%.4f required=%.3f "
            "wall_elapsed=%.3f idle_wall=%.3f", label, sim_elapsed, seconds,
            wall_elapsed, clock_guard.idleWallSeconds());
          next_clock_log = Wall::now() + 5s;
        }
      } else {
        if (sim_elapsed >= seconds) {break;}
        require(Wall::now() - wall_start < 30s, "Transfer hold simulation clock timed out");
      }
      carrySafe();
      int64_t sample_stamp = 0;Wall::time_point sample_received;
      const auto box = freshBox(&sample_stamp, &sample_received);
      const auto tcp = arm_->getCurrentPose(tcp_).pose;
      const double drift = (rotation(tcp).inverse() * (point(box) - point(tcp)) - reference).norm();
      const double angle = reference_angle.angularDistance(rotation(tcp).inverse() * rotation(box));
      max_drift = std::max(max_drift, drift);max_angle = std::max(max_angle, angle);
      RCLCPP_DEBUG(node_->get_logger(), "%s: drift=%.3f mm angle=%.3f deg", label,
        max_drift * 1000, max_angle * 180 / M_PI);
      if (node_->get_parameter_or<bool>("compartment_grip_diagnostics", false) &&
        Wall::now() >= next_sample) {
        logGripSample(label, drift, angle, sample_stamp, sample_received);
        next_sample = Wall::now() + 500ms;
      }
      if (max_drift >= 0.005 || max_angle >= 0.0873) {
        logGripSample(label, drift, angle, sample_stamp, sample_received);
        RCLCPP_ERROR(node_->get_logger(),
          "HOLD LIMIT EXCEEDED: phase=%s drift=%.3f mm limit=5.000 mm angle=%.3f deg limit=%.3f deg",
          label, max_drift * 1000, max_angle * 180 / M_PI, 0.0873 * 180 / M_PI);
      }
      require(max_drift < 0.005 && max_angle < 0.0873,
        std::string(label) + ": physical box slipped or rotated in gripper");
      std::this_thread::sleep_for(100ms);
    }
    if (progress_guard) {
      RCLCPP_INFO(node_->get_logger(),
        "SIMULATION HOLD COMPLETE: phase=%s sim_elapsed=%.4f required=%.3f wall_elapsed=%.3f",
        label, clock_guard.simElapsedSeconds(), seconds,
        std::chrono::duration<double>(Wall::now() - wall_start).count());
    }
    RCLCPP_INFO(node_->get_logger(), "%s HOLD PASSED: drift=%.3f mm angle=%.3f deg", label,
      max_drift * 1000, max_angle * 180 / M_PI);
  }

  void logGripSample(const char * label, double drift, double angle,
    int64_t sample_stamp, Wall::time_point sample_received)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(node_->get_logger(),
      "GRIP SAMPLE: phase=%s sim=%.3f drift=%.3f mm angle=%.3f deg "
      "box_wall_age=%.4f s box_sim_age=%.4f s fingers=(%.5f,%.5f) m",
      label, node_->now().seconds(), drift * 1000, angle * 180 / M_PI,
      std::chrono::duration<double>(Wall::now() - sample_received).count(),
      (node_->now().nanoseconds() - sample_stamp) * 1e-9, fingers_[0], fingers_[1]);
  }

  Pose freshBox(int64_t * sample_stamp = nullptr, Wall::time_point * sample_received = nullptr)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (sequence_ > 0 && Wall::now() - box_received_ < 2s) {
        if (sample_stamp) {*sample_stamp = stamp_;}
        if (sample_received) {*sample_received = box_received_;}
        return box_;
      }
    }
    // Transient bridge gap: wait up to 3 s more for a NEW pose; never use the stale one.
    RCLCPP_WARN(node_->get_logger(), "BOX POSE STALE: waiting up to 3 s for a fresh Gazebo pose");
    const auto end = Wall::now() + 3s;
    while (Wall::now() < end) {
      std::this_thread::sleep_for(50ms);
      std::lock_guard<std::mutex> lock(mutex_);
      if (sequence_ > 0 && Wall::now() - box_received_ < 200ms) {
        RCLCPP_WARN(node_->get_logger(), "BOX POSE RECOVERED");
        if (sample_stamp) {*sample_stamp = stamp_;}
        if (sample_received) {*sample_received = box_received_;}
        return box_;
      }
    }
    throw std::runtime_error("Physical box pose stale/missing");
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
    double angle_tolerance = 0.025)
  {
    auto previous = arm_->getCurrentJointValues();int calm = 0;
    const auto end = Wall::now() + 20s;
    while (Wall::now() < end) {
      // 100 ms samples with 0.0001 rad: same 1 mrad/s settle criterion as before.
      std::this_thread::sleep_for(100ms);
      auto state = arm_->getCurrentState(2.0);
      require(static_cast<bool>(state), "Fresh arm state missing");
      std::vector<double> current;state->copyJointGroupPositions("arm_1", current);
      require(current.size() == 7 && previous.size() == 7, "Incomplete arm state");
      double delta = 0;
      for (size_t i = 0; i < 7; ++i) {delta = std::max(delta, std::abs(current[i] - previous[i]));}
      bool near = true;
      if (target) {
        state->update();const auto actual = state->getGlobalLinkTransform(tcp_);
        near = (actual.translation() - point(*target)).norm() < position_tolerance &&
          Eigen::Quaterniond(actual.rotation()).angularDistance(rotation(*target)) <
          angle_tolerance;
      }
      calm = delta < 0.0001 && near ? calm + 1 : 0;previous = current;
      if (calm >= 3) {return;}
    }
    const auto actual = arm_->getCurrentPose(tcp_).pose;
    if (target) {
      RCLCPP_ERROR(node_->get_logger(), "SETTLE FAILED: position_error=%.3f mm angle=%.4f rad",
        1000 * (point(actual) - point(*target)).norm(),
        rotation(actual).angularDistance(rotation(*target)));
    }
    auto state = arm_->getCurrentState(2.0);
    if (state) {
      for (const auto & name : state->getJointModelGroup("arm_1")->getVariableNames()) {
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
  void setPlanningStartState()
  {
    auto current = arm_->getCurrentState(2.0);
    require(static_cast<bool>(current), "No planning start state");
    moveit::core::RobotState start(*current);
    for (const auto * joint : start.getJointModelGroup("arm_1")->getActiveJointModels()) {
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
    arm_->setStartState(start);
  }
  // MoveIt only guarantees zero-distance clearance for the held box. Before executing a
  // carry plan, require (a) >= 30 mm above the tabletop while over the margin-expanded
  // table footprint and (b) >= 15 mm from every shelf_1 post and board (box half-diagonal
  // 28.3 mm, box half-height along world z from its current orientation).
  bool carryClearance(
    const Group::Plan & plan, const Eigen::Vector3d & reference,
    const Eigen::Quaterniond & reference_angle, const char * label)
  {
    if (node_->has_parameter("compartment_shelf_boxes")) {
      return compartmentCarryClearance(plan, reference, reference_angle, label);
    }
    moveit::core::RobotState state(arm_->getRobotModel());
    state.setToDefaultValues();
    const auto & trajectory = plan.trajectory.joint_trajectory;
    double min_bottom = std::numeric_limits<double>::infinity();
    double min_post = std::numeric_limits<double>::infinity();
    double min_board = std::numeric_limits<double>::infinity();
    // shelf_1 boards (0.5 x 0.38 x 0.025 m) at these centre heights, shelf at (0, -1.2).
    const std::array<double, 4> boards = {0.08, 0.45, 0.80, 1.15};
    const std::array<Eigen::Vector2d, 4> posts = {
      Eigen::Vector2d(0.25, -1.02), Eigen::Vector2d(0.25, -1.38),
      Eigen::Vector2d(-0.25, -1.02), Eigen::Vector2d(-0.25, -1.38)};
    for (const auto & waypoint : trajectory.points) {
      state.setVariablePositions(trajectory.joint_names, waypoint.positions);
      state.update();
      const auto & tcp = state.getGlobalLinkTransform(tcp_);
      const Eigen::Vector3d center = tcp * reference;
      const Eigen::Matrix3d box = tcp.linear() * reference_angle.toRotationMatrix();
      const double half_height = 0.02 * box.row(2).cwiseAbs().sum();
      if (std::abs(center.x()) <= 0.43 && std::abs(center.y()) <= 0.33) {
        min_bottom = std::min(min_bottom, center.z() - half_height);
      }
      if (std::abs(center.x()) <= 0.25 + 0.0283 + 0.015 &&
        std::abs(center.y() + 1.2) <= 0.19 + 0.0283 + 0.015)
      {
        for (const double board : boards) {
          const double below = (board - 0.0125) - (center.z() + half_height);
          const double above = (center.z() - half_height) - (board + 0.0125);
          min_board = std::min(min_board, std::max(below, above));
        }
      }
      for (const auto & post : posts) {
        const double dx = std::max(std::abs(center.x() - post.x()) - 0.02, 0.0);
        const double dy = std::max(std::abs(center.y() - post.y()) - 0.02, 0.0);
        min_post = std::min(min_post, std::hypot(dx, dy) - 0.0283);
      }
    }
    const bool ok = min_bottom >= 0.43 && min_post >= 0.015 && min_board >= 0.015;
    RCLCPP_INFO(node_->get_logger(),
      "%s CLEARANCE %s: min box bottom over table=%.4f m (limit 0.430) "
      "min shelf_1 post gap=%.1f mm, board gap=%.1f mm (limits 15)", label,
      ok ? "OK" : "REJECTED", min_bottom, min_post * 1000,
      std::isfinite(min_board) ? min_board * 1000 : 9999.0);
    return ok;
  }

  bool compartmentCarryClearance(const Group::Plan & plan, const Eigen::Vector3d & reference,
    const Eigen::Quaterniond & reference_angle, const char * label)
  {
    const auto boxes = node_->get_parameter("compartment_shelf_boxes").as_double_array();
    require(!boxes.empty() && boxes.size() % 6 == 0 &&
      std::all_of(boxes.begin(), boxes.end(), [](double v) {return std::isfinite(v);}),
      "Invalid compartment shelf clearance geometry");
    for (size_t j = 0; j < boxes.size(); j += 6) {
      require(boxes[j + 3] > 0 && boxes[j + 4] > 0 && boxes[j + 5] > 0,
        "Invalid compartment shelf size");
    }
    moveit::core::RobotState state(arm_->getRobotModel());state.setToDefaultValues();
    const auto & trajectory = plan.trajectory.joint_trajectory;
    double min_bottom = std::numeric_limits<double>::infinity();
    double min_shelf = std::numeric_limits<double>::infinity();
    bool margin = true;
    for (const auto & waypoint : trajectory.points) {
      state.setVariablePositions(trajectory.joint_names, waypoint.positions);state.update();
      for (const auto & name : state.getJointModelGroup("arm_1")->getVariableNames()) {
        const auto & b = state.getRobotModel()->getVariableBounds(name);
        const double value = state.getVariablePosition(name);
        margin = margin && value >= b.min_position_ + 0.05 && value <= b.max_position_ - 0.05;
      }
      const auto & tcp = state.getGlobalLinkTransform(tcp_);
      const Eigen::Vector3d center = tcp * reference;
      const Eigen::Vector3d half = 0.02 *
        (tcp.linear() * reference_angle.toRotationMatrix()).cwiseAbs().rowwise().sum();
      if (std::abs(center.x()) <= 0.43 && std::abs(center.y()) <= 0.33) {
        min_bottom = std::min(min_bottom, center.z() - half.z());
      }
      for (size_t j = 0; j < boxes.size(); j += 6) {
        const Eigen::Vector3d origin(boxes[j], boxes[j + 1], boxes[j + 2]);
        const Eigen::Vector3d size(boxes[j + 3], boxes[j + 4], boxes[j + 5]);
        const Eigen::Vector3d gap = ((center - origin).cwiseAbs() - half - 0.5 * size).cwiseMax(0.0);
        min_shelf = std::min(min_shelf, gap.norm());
      }
    }
    const bool ok = !trajectory.points.empty() && margin && min_bottom >= 0.43 && min_shelf >= 0.015;
    RCLCPP_INFO(node_->get_logger(), "%s COMPARTMENT CLEARANCE %s: min_bottom=%.4f m "
      "min_shelf_gap=%.2f mm joint_margin=%d", label, ok ? "OK" : "REJECTED",
      min_bottom, min_shelf * 1000, margin);
    return ok;
  }
  void carrySafe()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require(!carry_contact_, "Unexpected box contact while carrying; motion refused");
    require(!unexpected_box_contact_, "Box contacted a robot link outside the fingers");
  }
  void logCarryRoute(const moveit_msgs::msg::RobotTrajectory & plan, const char * label)
  {
    const auto & trajectory = plan.joint_trajectory;
    if (trajectory.points.empty()) {return;}
    const auto & stamp = trajectory.points.back().time_from_start;
    RCLCPP_INFO(node_->get_logger(),
      "LEFT SELECTED ROUTE: phase=%s planned=%.3f s points=%zu", label,
      static_cast<double>(stamp.sec) + stamp.nanosec * 1e-9, trajectory.points.size());
    for (size_t joint = 0; joint < trajectory.joint_names.size(); ++joint) {
      double travel = 0.0;
      for (size_t i = 1; i < trajectory.points.size(); ++i) {
        travel += std::abs(trajectory.points[i].positions[joint] -
          trajectory.points[i - 1].positions[joint]);
      }
      const double net = std::abs(trajectory.points.back().positions[joint] -
        trajectory.points.front().positions[joint]);
      RCLCPP_INFO(node_->get_logger(),
        "LEFT JOINT TRAVEL: phase=%s joint=%s start=%.4f end=%.4f total=%.4f rad "
        "net=%.4f rad excess=%.4f rad", label, trajectory.joint_names[joint].c_str(),
        trajectory.points.front().positions[joint], trajectory.points.back().positions[joint],
        travel, net, travel - net);
    }
    auto measured = arm_->getCurrentState(2.0);
    if (!measured) {
      RCLCPP_WARN(node_->get_logger(), "LEFT TCP PATH: state unavailable for diagnostics");
      return;
    }
    moveit::core::RobotState state(*measured);
    Eigen::Vector3d first = Eigen::Vector3d::Zero(), previous = first;
    double length = 0.0, angular_length = 0.0, peak_angular = 0.0, peak_linear = 0.0;
    Eigen::Quaterniond previous_q = Eigen::Quaterniond::Identity();
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      state.setVariablePositions(trajectory.joint_names, trajectory.points[i].positions);
      state.update();
      const Eigen::Vector3d position = state.getGlobalLinkTransform(tcp_).translation();
      const Eigen::Quaterniond q(state.getGlobalLinkTransform(tcp_).linear());
      if (i == 0) {first = position;} else {
        const double distance = (position - previous).norm();
        const double turn = previous_q.angularDistance(q);
        const auto & a = trajectory.points[i - 1].time_from_start;
        const auto & b = trajectory.points[i].time_from_start;
        const double dt = (b.sec - a.sec) + (static_cast<double>(b.nanosec) - a.nanosec) * 1e-9;
        length += distance;angular_length += turn;
        if (dt > 1e-9) {
          peak_angular = std::max(peak_angular, turn / dt);
          peak_linear = std::max(peak_linear, distance / dt);
        }
      }
      previous_q = q;
      previous = position;
    }
    RCLCPP_INFO(node_->get_logger(),
      "LEFT TCP PATH: phase=%s length=%.4f m displacement=%.4f m",
      label, length, (previous - first).norm());
    RCLCPP_INFO(node_->get_logger(),
      "LEFT TCP RATES: phase=%s angular_path=%.4f rad peak_angular=%.4f rad/s peak_linear=%.4f m/s",
      label, angular_length, peak_angular, peak_linear);
  }

  // Execute while sampling the held box every 100 ms (mirrors the right arm's
  // SHELF TRANSFER MONITOR). Stops the arm on unexpected contact, lost finger
  // contact, the box dropping near the tabletop, or a gross in-hand slip.
  void executeMonitored(
    const moveit_msgs::msg::RobotTrajectory & trajectory, const char * label,
    const Eigen::Vector3d & reference, const Eigen::Quaterniond & reference_angle)
  {
    carrySafe();
    logCarryRoute(trajectory, label);
    auto execution = std::async(std::launch::async,
        [this, &trajectory]() {return arm_->execute(trajectory);});
    double min_bottom = std::numeric_limits<double>::infinity();
    double max_drift = 0.0, max_angle = 0.0;int gross = 0;
    auto next_sample = Wall::now();
    try {
      while (execution.wait_for(100ms) == std::future_status::timeout) {
        carrySafe();
        int64_t sample_stamp = 0;Wall::time_point sample_received;
        const auto box = freshBox(&sample_stamp, &sample_received);
        const auto tcp = arm_->getCurrentPose(tcp_).pose;
        const auto inverse = rotation(tcp).inverse();
        const double drift = (inverse * (point(box) - point(tcp)) - reference).norm();
        const double angle = reference_angle.angularDistance(inverse * rotation(box));
        max_drift = std::max(max_drift, drift);max_angle = std::max(max_angle, angle);
        if (node_->get_parameter_or<bool>("compartment_grip_diagnostics", false) &&
          Wall::now() >= next_sample) {
          logGripSample(label, drift, angle, sample_stamp, sample_received);
          next_sample = Wall::now() + 500ms;
        }
        if (std::abs(box.position.x) <= 0.43 && std::abs(box.position.y) <= 0.33) {
          min_bottom = std::min(min_bottom, bottom(box));
          require(bottom(box) > 0.41, "Held box came within 10 mm of the tabletop in motion");
        }
        gross = (drift > 0.015 || angle > 0.1745) ? gross + 1 : 0;
        require(gross < 2, std::string(label) + ": box slipped in gripper during motion");
        std::lock_guard<std::mutex> lock(mutex_);
        if (contacts_seen_) {
          for (const auto received : finger_contact_) {
            require(Wall::now() - received < 1s, "Finger contact lost during carry");
          }
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_->get_logger(), "%s STOP REQUESTED: %s", label, error.what());
      arm_->stop();execution.wait();throw;
    }
    require(execution.get() == moveit::core::MoveItErrorCode::SUCCESS,
      std::string(label) + " execution failed");
    carrySafe();
    RCLCPP_INFO(node_->get_logger(),
      "%s MONITOR PASSED: min_box_bottom_over_table=%.4f m max_sampled_drift=%.3f mm "
      "max_sampled_angle=%.3f deg", label, min_bottom, max_drift * 1000, max_angle * 180 / M_PI);
  }
  void executePlan(const std::function<bool(const Group::Plan &)> & accept = nullptr,
    const std::function<void(const Group::Plan &)> & run = nullptr)
  {
    Group::Plan plan;bool planned = false;
    // OMPL paths are sometimes rejected by ValidateSolution after dense time
    // parameterization (arm link touching the table). Each attempt takes ~0.1 s,
    // so allow more fresh attempts; collision validation itself is unchanged.
    for (int attempt = 0; attempt < 10; ++attempt) {
      setPlanningStartState();
      if (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
        (!accept || accept(plan))) {planned = true;break;}
      RCLCPP_WARN(node_->get_logger(), "PLAN ATTEMPT %d/10 rejected; replanning", attempt + 1);
    }
    require(planned, "Planning failed");
    if (run) {run(plan);return;}
    carrySafe();
    require(arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS, "Execution failed");
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
  moveit::core::GroupStateValidityCallbackFn collisionCheck(
    const planning_scene::PlanningScene & scene)
  {
    const bool compartment = node_->has_parameter("source_xyz");
    return [&scene, compartment](moveit::core::RobotState * candidate,
           const moveit::core::JointModelGroup * joints, const double * values) {
             candidate->setJointGroupPositions(joints, values);
             candidate->update();
             if (compartment) {
               for (const auto & name : joints->getVariableNames()) {
                 const auto & b = candidate->getRobotModel()->getVariableBounds(name);
                 const double value = candidate->getVariablePosition(name);
                 if (value < b.min_position_ + 0.05 || value > b.max_position_ - 0.05) {return false;}
               }
               return !scene.isStateColliding(*candidate);
             }
             return candidate->satisfiesBounds(joints) && !scene.isStateColliding(*candidate,
          "arm_1");
           };
  }
  bool checkedFinePath(
    const planning_scene::PlanningScene & scene, moveit::core::RobotState state,
    const Pose & target, std::vector<moveit::core::RobotStatePtr> & path, bool report = false)
  {
    const auto * group = state.getJointModelGroup("arm_1");
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = point(target);
    transform.linear() = rotation(target).toRotationMatrix();
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &state, group, path, state.getLinkModel(tcp_), transform, true,
      moveit::core::MaxEEFStep(0.0001), moveit::core::CartesianPrecision{0.00001, 0.0001, 0.00001},
      collisionCheck(scene), kinematics::KinematicsQueryOptions());
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
  void finePath(const Pose & target, robot_trajectory::RobotTrajectory & trajectory)
  {
    planning_scene::PlanningScene scene(arm_->getRobotModel());
    require(scene.setPlanningSceneMsg(planningSceneSnapshot()),
        "Fine planning scene import failed");
    std::vector<moveit::core::RobotStatePtr> path;
    if (!checkedFinePath(scene, scene.getCurrentState(), target, path, true)) {
      throw FinePathRejected("Incomplete collision-checked fine path or joint jump");
    }
    for (const auto & waypoint : path) {
      trajectory.addSuffixWayPoint(*waypoint, 0.0);
    }
  }

  // TOTG occasionally fails numerically ("Negative path velocity") on dense fine
  // paths. Retry the same geometric path with slightly different TOTG parameters;
  // the endpoint/precision checks after timing still apply unchanged.
  bool timeWithRetry(robot_trajectory::RobotTrajectory & trajectory, double scale, bool fine)
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
      if (timing.computeTimeStamps(trajectory, scale, scale)) {return true;}
    }
    trajectory = robot_trajectory::RobotTrajectory(original, true);
    return false;
  }

  void linear(const Pose & target, double scale = 0.1, bool precise = false)
  {
    arm_->setStartStateToCurrentState();moveit_msgs::msg::RobotTrajectory msg;
    auto state = arm_->getCurrentState(2.0);
    require(static_cast<bool>(state), "No trajectory state");
    robot_trajectory::RobotTrajectory trajectory(arm_->getRobotModel(), "arm_1");
    const bool fine = precise || scale <= 0.01;
    if (fine) {finePath(target, trajectory);} else {
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
    require(timeWithRetry(trajectory, scale, fine), "Trajectory timing failed");
    const auto endpoint = trajectory.getLastWayPoint().getGlobalLinkTransform(tcp_);
    const double error = (endpoint.translation() - point(target)).norm();
    const double angle = Eigen::Quaterniond(endpoint.rotation()).angularDistance(rotation(target));
    RCLCPP_INFO(node_->get_logger(), "TIMED ENDPOINT: error=%.3f mm angle=%.4f rad",
      error * 1000, angle);
    require(error < (fine ? 0.00005 : 0.003) && angle < 0.025,
      "Timed endpoint diverged from target");
    for (const auto & name : state->getJointModelGroup("arm_1")->getVariableNames()) {
      RCLCPP_INFO(node_->get_logger(), "TIMED JOINT %s target=%.6f", name.c_str(),
        trajectory.getLastWayPoint().getVariablePosition(name));
    }
    trajectory.getRobotTrajectoryMsg(msg);
    if (node_->has_parameter("source_xyz")) {
      for (const auto & waypoint : msg.joint_trajectory.points) {
        for (size_t i = 0; i < msg.joint_trajectory.joint_names.size(); ++i) {
          const auto & name = msg.joint_trajectory.joint_names[i];
          const auto & bounds = arm_->getRobotModel()->getVariableBounds(name);
          require(waypoint.positions[i] >= bounds.min_position_ + 0.05 &&
            waypoint.positions[i] <= bounds.max_position_ - 0.05,
            "Compartment Cartesian path lost 0.05 rad joint margin");
        }
      }
    }
    carrySafe();
    require(arm_->execute(msg) == moveit::core::MoveItErrorCode::SUCCESS,
        "Cartesian execution failed");
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
  void allowFingers(bool support = false, bool finger_contact = true)
  {
    auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    require(client->wait_for_service(10s), "Planning scene unavailable");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = client->async_send_request(request);
    require(future.wait_for(5s) == std::future_status::ready, "Collision matrix timeout");
    auto acm = future.get()->scene.allowed_collision_matrix;
    for (const std::string name : {"target_box", "left_fr3v2_leftfinger",
        "left_fr3v2_rightfinger", "table_link"})
    {
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
    const auto box = std::find(acm.entry_names.begin(), acm.entry_names.end(), "target_box") -
      acm.entry_names.begin();
    for (size_t i = 0; i < acm.entry_names.size(); ++i) {
      if (acm.entry_names[i] == "left_fr3v2_leftfinger" ||
        acm.entry_names[i] == "left_fr3v2_rightfinger") {
        acm.entry_values[box].enabled[i] = finger_contact;
        acm.entry_values[i].enabled[box] = finger_contact;
      } else if (support && acm.entry_names[i] == "table_link") {
        acm.entry_values[box].enabled[i] = true;acm.entry_values[i].enabled[box] = true;
      }
    }
    moveit_msgs::msg::PlanningScene diff;diff.is_diff = true;diff.allowed_collision_matrix = acm;
    require(scene_->applyPlanningScene(diff), "Finger collision allowance failed");
  }
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contacts_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr table_contacts_;
  bool robot_table_contact_ = false, table_support_seen_ = false;
  Wall::time_point table_support_wall_;
  bool contacts_seen_ = false, support_seen_ = false, unexpected_box_contact_ = false;
  bool carrying_ = false, seating_ = false, carry_contact_ = false;
  std::array<Wall::time_point, 2> finger_contact_{};
  Wall::time_point support_wall_;
  int64_t support_sim_ = 0;
  rclcpp::Node::SharedPtr node_;
  std::mutex mutex_;
  Pose box_;size_t sequence_ = 0;int64_t stamp_ = -1;
  Wall::time_point box_received_;
  std::array<double, 2> fingers_{};
  std::array<Wall::time_point, 2> finger_received_{};
  const std::string tcp_ = "left_fr3v2_hand_tcp";
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poses_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp_action::Client<Grip>::SharedPtr grip_;
  std::unique_ptr<Group> arm_;
  std::vector<double> table_goal_seed_;
  std::vector<Group::Plan> compartment_table_route_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("single_arm_pick_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  std::thread spinner([&executor] {executor.spin();});
  int rc = 0;
  std::unique_ptr<Pick> test;
  try {
    test = std::make_unique<Pick>(node);
    test->run();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
      "SINGLE ARM TEST FAILED: %s; test stopped, inspect simulation before restarting", e.what());
    rc = 1;
  }
  executor.cancel();spinner.join();test.reset();rclcpp::shutdown();return rc;
}
