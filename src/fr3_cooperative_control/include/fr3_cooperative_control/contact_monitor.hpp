// Gazebo contact sensors (box, table, target shelf) reduced to the facts the task needs:
// finger contact, box support on a surface, robot-surface contact and unexpected contacts.
#pragma once

#include <array>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>

#include "fr3_cooperative_control/common.hpp"

namespace fr3_cooperative_control
{

struct ContactStamp
{
  bool seen = false;
  Wall::time_point wall{};
  int64_t sim = 0;
};

struct ContactPolicy
{
  // Box-sensor collision (substring) that counts as the box resting on its next support.
  std::string support_collision = "table_top_collision";
  // While seating: true = only supporting contacts are allowed, false = any contact with
  // `support_collision` is allowed.
  bool seat_allows_support_only = false;
  bool log_each_unexpected = false;
  std::string support_label;  // non-empty: log the first supporting contact with this label
};

class ContactMonitor
{
public:
  ContactMonitor(rclcpp::Node::SharedPtr node, const ArmNames & arm, ContactPolicy policy);

  // Contact sensor of a fixed surface (table, shelf). `box_filter`: collision substring the box
  // must touch ("" = any), `box_needs_vertical`: support-like contact only. Robot contacts with
  // the surface are recorded while `watch_robot()` is true.
  void addSurface(const std::string & name, const std::string & topic,
    const std::string & box_filter, bool box_needs_vertical,
    std::function<bool()> watch_robot = [] {return true;});

  void setCarrying(bool value);
  void setSeating(bool value);

  bool anyBoxContactSeen() const;
  bool unexpected() const;         // box touched a robot link other than this arm's fingers
  bool carryContact() const;       // box touched anything but the fingers while carried
  bool robotTouched(const std::string & surface) const;
  bool fingersFresh(std::chrono::milliseconds age) const;
  ContactStamp support() const;    // box resting on `support_collision` (box sensor)
  ContactStamp surfaceBox(const std::string & surface) const;  // surface sensor saw the box

private:
  static bool has(const std::string & a, const std::string & b, const std::string & key)
  {
    return a.find(key) != std::string::npos || b.find(key) != std::string::npos;
  }
  static bool supportLike(const ros_gz_interfaces::msg::Contact & c);
  void onBox(const ros_gz_interfaces::msg::Contacts & msg);

  rclcpp::Node::SharedPtr node_;
  ArmNames arm_;
  ContactPolicy policy_;
  std::vector<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr> subs_;
  mutable std::mutex mutex_;
  bool any_seen_ = false, unexpected_ = false, carry_contact_ = false;
  bool carrying_ = false, seating_ = false;
  std::array<Wall::time_point, 2> fingers_{};
  ContactStamp support_;
  std::map<std::string, ContactStamp> surface_box_;
  std::map<std::string, bool> robot_touched_;
};

}  // namespace fr3_cooperative_control
