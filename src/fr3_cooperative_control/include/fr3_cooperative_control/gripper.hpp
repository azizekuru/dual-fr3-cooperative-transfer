// Franka hand of one arm: GripperCommand action plus measured finger positions.
#pragma once

#include <array>
#include <mutex>

#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "fr3_cooperative_control/common.hpp"

namespace fr3_cooperative_control
{

class Gripper
{
public:
  using Action = control_msgs::action::GripperCommand;
  Gripper(rclcpp::Node::SharedPtr node, const ArmNames & names);

  bool waitForServer(std::chrono::seconds timeout);
  // grasp=true: closing on the box (a box-stalled abort is success; each finger 18.5-22 mm).
  // grasp=false: each finger must reach `position` within 1 mm.
  void command(double position, bool grasp);

private:
  rclcpp::Node::SharedPtr node_;
  ArmNames names_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  std::mutex mutex_;
  std::array<double, 2> fingers_{};
  std::array<Wall::time_point, 2> received_{};
};

}  // namespace fr3_cooperative_control
