#include "fr3_cooperative_control/gripper.hpp"

#include <cmath>
#include <thread>

namespace fr3_cooperative_control
{
using namespace std::chrono_literals;

Gripper::Gripper(rclcpp::Node::SharedPtr node, const ArmNames & names)
: node_(std::move(node)), names_(names)
{
  joints_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        for (size_t j = 0; j < 2; ++j) {
          if (msg->name[i] == names_.finger_joint_prefix + std::to_string(j + 1)) {
            fingers_[j] = msg->position[i];received_[j] = Wall::now();
          }
        }
      }
    });
  client_ = rclcpp_action::create_client<Action>(
    node_, "/" + names_.side + "_gripper_controller/gripper_cmd");
}

bool Gripper::waitForServer(std::chrono::seconds timeout)
{
  return client_->wait_for_action_server(timeout);
}

void Gripper::command(double position, bool grasp)
{
  Action::Goal goal;goal.command.position = position;goal.command.max_effort = 20;
  auto sent = client_->async_send_goal(goal);
  require(sent.wait_for(5s) == std::future_status::ready, "Gripper goal timed out");
  auto handle = sent.get();require(static_cast<bool>(handle), "Gripper goal rejected");
  auto future = client_->async_get_result(handle);
  if (future.wait_for(30s) != std::future_status::ready) {
    client_->async_cancel_goal(handle);throw std::runtime_error("Gripper result timed out");
  }
  auto result = future.get();
  const bool stopped_by_box = grasp && result.result && result.result->stalled &&
    result.code == rclcpp_action::ResultCode::ABORTED;
  require(result.code == rclcpp_action::ResultCode::SUCCEEDED || stopped_by_box,
    "Gripper command failed");
  std::this_thread::sleep_for(500ms);
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < 2; ++i) {
    require(Wall::now() - received_[i] < 1s, "Physical finger state stale");
    require(grasp ? fingers_[i] > 0.0185 && fingers_[i] < 0.022 :
      std::abs(fingers_[i] - position) < 0.001, "Physical finger opening unexpected");
  }
  RCLCPP_INFO(node_->get_logger(), "PHYSICAL FINGERS: %.4f / %.4f m stalled=%d",
    fingers_[0], fingers_[1], result.result ? result.result->stalled : false);
}

}  // namespace fr3_cooperative_control
