// Physical box pose from Gazebo (/target_box/pose), shifted into the MoveIt world frame.
#pragma once

#include <cstddef>
#include <mutex>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fr3_cooperative_control/common.hpp"

namespace fr3_cooperative_control
{

class BoxTracker
{
public:
  BoxTracker(rclcpp::Node::SharedPtr node, double box_size, double gazebo_z_offset);

  // Latest pose if younger than 2 s; otherwise waits up to 3 s for a new one (never stale).
  Pose fresh();
  // Waits until 3 consecutive new samples move < 0.5 mm and < 0.01 rad.
  Pose stable();
  bool received() const;

  // Tilt of the nearest cube face normal from world z, and lowest point of the cube.
  double tilt(const Pose & box) const;
  double bottom(const Pose & box) const;
  double half() const {return half_;}

private:
  rclcpp::Node::SharedPtr node_;
  double half_, z_offset_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  mutable std::mutex mutex_;
  Pose box_;
  std::size_t sequence_ = 0;
  int64_t stamp_ = -1;
  Wall::time_point received_;
};

}  // namespace fr3_cooperative_control
