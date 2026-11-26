#include "fr3_cooperative_control/box_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

namespace fr3_cooperative_control
{
using namespace std::chrono_literals;

BoxTracker::BoxTracker(rclcpp::Node::SharedPtr node, double box_size, double gazebo_z_offset)
: node_(std::move(node)), half_(0.5 * box_size), z_offset_(gazebo_z_offset)
{
  sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/target_box/pose", rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
      if (stamp <= stamp_) {return;}
      stamp_ = stamp;box_ = msg->pose;box_.position.z -= z_offset_;
      received_ = Wall::now();++sequence_;
    });
}

bool BoxTracker::received() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return sequence_ > 0;
}

Pose BoxTracker::fresh()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sequence_ > 0 && Wall::now() - received_ < 2s) {return box_;}
  }
  // Transient bridge gap: wait up to 3 s more for a NEW pose; never use the stale one.
  RCLCPP_WARN(node_->get_logger(), "BOX POSE STALE: waiting up to 3 s for a fresh Gazebo pose");
  const auto end = Wall::now() + 3s;
  while (Wall::now() < end) {
    std::this_thread::sleep_for(50ms);
    std::lock_guard<std::mutex> lock(mutex_);
    if (sequence_ > 0 && Wall::now() - received_ < 200ms) {
      RCLCPP_WARN(node_->get_logger(), "BOX POSE RECOVERED");
      return box_;
    }
  }
  throw std::runtime_error("Physical box pose stale/missing");
}

Pose BoxTracker::stable()
{
  Pose previous;std::size_t last_sequence = 0;int calm = 0;
  const auto end = Wall::now() + 15s;
  while (Wall::now() < end) {
    std::this_thread::sleep_for(200ms);
    Pose current;std::size_t sequence;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (sequence_ == 0 || sequence_ == last_sequence) {continue;}
      current = box_;sequence = sequence_;
    }
    calm = last_sequence && (point(current) - point(previous)).norm() < 0.0005 &&
      rotation(current).angularDistance(rotation(previous)) < 0.01 ? calm + 1 : 0;
    previous = current;last_sequence = sequence;
    if (calm >= 3) {return fresh();}
  }
  throw std::runtime_error("Physical box did not settle");
}

double BoxTracker::tilt(const Pose & box) const
{
  return std::acos(std::clamp(rotation(box).toRotationMatrix().row(2).cwiseAbs().maxCoeff(),
    0.0, 1.0));
}

double BoxTracker::bottom(const Pose & box) const
{
  return box.position.z - half_ * rotation(box).toRotationMatrix().row(2).cwiseAbs().sum();
}

}  // namespace fr3_cooperative_control
