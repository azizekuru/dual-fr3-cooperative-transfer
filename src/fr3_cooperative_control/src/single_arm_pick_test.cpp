// Entry point: left-arm task only (parameter `stage`: pick | transfer | place).
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "fr3_cooperative_control/left_transfer.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("single_arm_pick_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  std::thread spinner([&executor] {executor.spin();});
  int rc = 0;
  std::unique_ptr<fr3_cooperative_control::LeftTransfer> task;
  try {
    task = std::make_unique<fr3_cooperative_control::LeftTransfer>(node);
    task->run(node->get_parameter_or<std::string>("stage", "pick"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
      "SINGLE ARM TEST FAILED: %s; test stopped, inspect simulation before restarting", e.what());
    rc = 1;
  }
  executor.cancel();spinner.join();task.reset();rclcpp::shutdown();return rc;
}
