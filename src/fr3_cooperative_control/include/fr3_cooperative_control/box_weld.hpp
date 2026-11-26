// Weld/unweld the box to a hand in Gazebo (test mode).
#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

namespace fr3_cooperative_control
{

// Rigid grasp test mode (rigid_grasp:=true). Gazebo's DetachableJoint system
// (environment_assets.xacro, loaded only with box_weld:=true) welds target_box to the hand
// (link7) on attach() and removes the weld on release(). The system creates the weld as soon as
// the box spawns, so a test first calls release(true).
// While welded, Gazebo treats the box as part of the robot: it reports no contacts between the
// box and the fingers, table or shelves. Tasks therefore weld only for the transfer motions and
// release before any contact-based step (seating), and skip finger-contact checks while welded.
class BoxWeld
{
public:
  BoxWeld(rclcpp::Node::SharedPtr node, const std::string & side)
  : node_(std::move(node)), side_(side)
  {
    const std::string base = "/box_weld/" + side;
    attach_ = node_->create_publisher<std_msgs::msg::Empty>(base + "/attach", 10);
    detach_ = node_->create_publisher<std_msgs::msg::Empty>(base + "/detach", 10);
    state_ = node_->create_subscription<std_msgs::msg::String>(base + "/state", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_text_ = msg->data;++updates_;
      });
  }
  void attach() {set(true);}
  // Removes the weld if this node created it. The spawn weld is not tracked, so the start-up
  // release passes force=true.
  void release(bool force = false)
  {
    if (force || attached()) {set(false);}
  }
  bool attached() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return attached_;
  }

private:
  // Repeat the request until Gazebo reports the new state (covers bridge start-up and a
  // box that has not spawned yet); repeated requests are ignored by Gazebo once applied.
  void set(bool attached)
  {
    using namespace std::chrono_literals;
    const std::string wanted = attached ? "attached" : "detached";
    std::size_t start;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      start = updates_;
    }
    const auto end = std::chrono::steady_clock::now() + 15s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      (attached ? attach_ : detach_)->publish(std_msgs::msg::Empty());
      std::this_thread::sleep_for(200ms);
      std::lock_guard<std::mutex> lock(mutex_);
      if (updates_ != start && state_text_ == wanted) {
        attached_ = attached;
        RCLCPP_INFO(node_->get_logger(), "BOX WELD %s: %s", side_.c_str(), wanted.c_str());
        return;
      }
    }
    throw std::runtime_error("Box weld (" + side_ + ") was not " + wanted);
  }

  rclcpp::Node::SharedPtr node_;
  std::string side_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_, detach_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_;
  mutable std::mutex mutex_;
  std::string state_text_;
  std::size_t updates_ = 0;
  bool attached_ = false;
};

}  // namespace fr3_cooperative_control
