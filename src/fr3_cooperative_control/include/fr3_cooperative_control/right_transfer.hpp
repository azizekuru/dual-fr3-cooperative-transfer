// Right-arm task: park the left arm, pick the box from the table hand-off cell and place it in
// the target shelf cell.
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace fr3_cooperative_control
{

class RightTransfer
{
public:
  // Reads the task cells (TaskConfig) from the node parameters. The node must be spinning.
  explicit RightTransfer(rclcpp::Node::SharedPtr node);
  ~RightTransfer();
  // stage: "pick", "shelf_hold", "shelf_place" (full task) or "left_park".
  // left_placed: the left arm has just placed the box (two-stage left parking).
  // Throws std::runtime_error with the reason on any failed check.
  void run(const std::string & stage, bool left_placed);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace fr3_cooperative_control
