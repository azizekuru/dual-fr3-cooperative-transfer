// Shared aliases and small helpers for the FR3 cooperative transfer nodes.
#pragma once

#include <algorithm>
#include <array>
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

// Nearest of the 24 axis-aligned cube orientations (a cube may rest on any face).
inline Eigen::Quaterniond nearestCubeRotation(const Eigen::Quaterniond & current)
{
  std::array<int, 3> order = {0, 1, 2};double best_error = 10;
  Eigen::Quaterniond best = current;
  do {
    for (int signs = 0; signs < 8; ++signs) {
      Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
      for (int column = 0; column < 3; ++column) {
        matrix(order[column], column) = signs & (1 << column) ? -1.0 : 1.0;
      }
      if (matrix.col(0).cross(matrix.col(1)).dot(matrix.col(2)) < 0.5) {continue;}
      Eigen::Quaterniond q(matrix);const double error = q.angularDistance(current);
      if (error < best_error) {best_error = error;best = q;}
    }
  } while (std::next_permutation(order.begin(), order.end()));
  return best;
}

// Hand (TCP) orientation from its approach axis (hand z) and finger opening axis (hand y).
inline Eigen::Quaterniond handFrame(const Eigen::Vector3d & approach, const Eigen::Vector3d & fingers)
{
  Eigen::Matrix3d r;
  r.col(2) = approach.normalized();
  r.col(1) = fingers.normalized();
  r.col(0) = r.col(1).cross(r.col(2));
  return Eigen::Quaterniond(r);
}

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
