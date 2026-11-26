// Shared aliases and small helpers for the FR3 cooperative transfer nodes.
#pragma once

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

namespace fr3_cooperative_control
{

using Pose = geometry_msgs::msg::Pose;
using Wall = std::chrono::steady_clock;

inline void require(bool value, const std::string & message)
{
  if (!value) {throw std::runtime_error(message);}
}

inline Eigen::Vector3d point(const Pose & p) {return {p.position.x, p.position.y, p.position.z};}

inline Eigen::Quaterniond rotation(const Pose & p)
{
  return {p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z};
}

// Process-wide test speed factor (launch argument speed_scale, default 1.0). Every MoveIt
// velocity/acceleration scale and every TOTG scale goes through speed(); fine-path selection
// still uses the unscaled value, so path checks are unchanged.
inline double & speedScaleStorage()
{
  static double value = 1.0;
  return value;
}
inline void setSpeedScale(double value) {speedScaleStorage() = value > 0 ? value : 1.0;}
inline double speed(double scale) {return std::min(1.0, scale * speedScaleStorage());}

// A collision-checked fine Cartesian path could not be built (incomplete, joint jump or margin).
class FinePathRejected : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// Arm naming used by the FR3 duo description: side = "left" | "right".
struct ArmNames
{
  explicit ArmNames(const std::string & side_name)
  : side(side_name),
    group(side_name == "left" ? "arm_1" : "arm_2"),
    tcp(side_name + "_fr3v2_hand_tcp"),
    hand(side_name + "_fr3v2_hand"),
    left_finger(side_name + "_fr3v2_leftfinger"),
    right_finger(side_name + "_fr3v2_rightfinger"),
    finger_joint_prefix(side_name + "_fr3v2_finger_joint")
  {
    if (side_name != "left" && side_name != "right") {
      throw std::invalid_argument("Arm side must be 'left' or 'right'");
    }
  }
  std::string side, group, tcp, hand, left_finger, right_finger, finger_joint_prefix;
};

}  // namespace fr3_cooperative_control
