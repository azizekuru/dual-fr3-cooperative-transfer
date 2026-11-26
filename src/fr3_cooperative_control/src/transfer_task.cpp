// Single-process coordinator: source shelf cell -> table hand-off -> target shelf cell.
// Runs the left and right arm tasks in order and reports the same markers as the
// two-process launch chain (LEFT PLACE COMPLETE, FULL TRANSFER TEST PASSED, ...).
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "fr3_cooperative_control/left_transfer.hpp"
#include "fr3_cooperative_control/right_transfer.hpp"

namespace fc = fr3_cooperative_control;

namespace
{
// Same markers as table_handoff_test.launch.py (after_right) for the two-process chain.
std::string passMarker(const std::string & right_stage, bool table_only)
{
  if (right_stage == "left_park") {return "LEFT PARK SCENARIO PASSED";}
  if (right_stage == "shelf_place") {
    return table_only ? "RIGHT SHELF PLACE SCENARIO PASSED" : "FULL TRANSFER TEST PASSED";
  }
  if (right_stage == "shelf_hold") {
    return table_only ? "RIGHT SHELF SCENARIO PASSED" : "TABLE TO SHELF TEST PASSED";
  }
  return table_only ? "RIGHT TABLE PICK SCENARIO PASSED" : "TABLE HANDOFF TEST PASSED";
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("transfer_task",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  std::thread spinner([&executor] {executor.spin();});
  const auto right_stage = node->get_parameter_or<std::string>("right_stage", "shelf_place");
  const bool table_only = node->get_parameter_or<bool>("table_only", false);
  // Both tasks stay alive until the executor stops (MoveGroupInterface teardown while spinning
  // is avoided, as in the single-arm entry points).
  std::unique_ptr<fc::LeftTransfer> left;
  std::unique_ptr<fc::RightTransfer> right;
  int rc = 0;
  if (!table_only) {
    try {
      left = std::make_unique<fc::LeftTransfer>(node);
      left->run("place");
      RCLCPP_INFO(node->get_logger(), "LEFT PLACE COMPLETE: checking left-arm parking");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(), "SINGLE ARM TEST FAILED: %s", e.what());
      RCLCPP_ERROR(node->get_logger(),
        "TABLE HANDOFF FAILED: left placement failed; right pickup skipped");
      rc = 1;
    }
  }
  if (rc == 0) {
    try {
      right = std::make_unique<fc::RightTransfer>(node);
      right->run(right_stage, !table_only);
      RCLCPP_INFO(node->get_logger(), "%s", passMarker(right_stage, table_only).c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(), "RIGHT ARM TEST FAILED: %s", e.what());
      RCLCPP_ERROR(node->get_logger(), "TABLE HANDOFF FAILED: right-arm stage failed");
      rc = 1;
    }
  }
  executor.cancel();spinner.join();right.reset();left.reset();rclcpp::shutdown();return rc;
}
