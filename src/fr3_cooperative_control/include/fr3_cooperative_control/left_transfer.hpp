// Left-arm task: pick the box from the source shelf cell and place it on the table hand-off cell.
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace fr3_cooperative_control
{

class LeftTransfer
{
public:
  // Reads the task cells (TaskConfig) from the node parameters. The node must be spinning.
  explicit LeftTransfer(rclcpp::Node::SharedPtr node);
  ~LeftTransfer();
  // stage: "pick" (grasp + lift), "transfer" (carry above the table) or "place" (full task).
  // Throws std::runtime_error with the reason on any failed check.
  void run(const std::string & stage);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace fr3_cooperative_control
