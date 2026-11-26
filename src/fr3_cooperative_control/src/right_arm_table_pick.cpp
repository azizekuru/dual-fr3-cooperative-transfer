// Entry point: right-arm task only (parameters `stage` and `left_placed`).
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "fr3_cooperative_control/right_transfer.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("right_arm_table_pick",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  std::thread spinner([&executor] {executor.spin();});
  std::unique_ptr<fr3_cooperative_control::RightTransfer> task;int rc = 0;
  try {
    task = std::make_unique<fr3_cooperative_control::RightTransfer>(node);
    task->run(node->get_parameter_or<std::string>("stage", "pick"),
      node->get_parameter_or<bool>("left_placed", false));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "RIGHT ARM TEST FAILED: %s; inspect before restarting",
      e.what());rc = 1;
  }
  executor.cancel();spinner.join();task.reset();rclcpp::shutdown();return rc;
}
