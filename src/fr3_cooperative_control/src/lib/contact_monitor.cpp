#include "fr3_cooperative_control/contact_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace fr3_cooperative_control
{

ContactMonitor::ContactMonitor(
  rclcpp::Node::SharedPtr node, const ArmNames & arm, ContactPolicy policy)
: node_(std::move(node)), arm_(arm), policy_(std::move(policy))
{
  subs_.push_back(node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
    "/placement/box_contacts", rclcpp::SensorDataQoS(),
    [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {onBox(*msg);}));
}

bool ContactMonitor::supportLike(const ros_gz_interfaces::msg::Contact & c)
{
  bool vertical = false;
  for (const auto & n : c.normals) {vertical = vertical || std::abs(n.z) > 0.8;}
  double depth = 0;
  for (double d : c.depths) {depth = std::max(depth, d);}
  return vertical && !c.depths.empty() && depth < 0.001;
}

void ContactMonitor::onBox(const ros_gz_interfaces::msg::Contacts & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg.contacts.empty()) {any_seen_ = true;}
  for (const auto & c : msg.contacts) {
    const auto & a = c.collision1.name;const auto & b = c.collision2.name;
    bool finger = false;
    const std::array<std::string, 2> names = {arm_.left_finger, arm_.right_finger};
    for (size_t i = 0; i < 2; ++i) {
      if (has(a, b, names[i])) {finger = true;fingers_[i] = Wall::now();}
    }
    const bool table = has(a, b, "table_top_collision");
    const bool on_support = has(a, b, policy_.support_collision);
    const bool support = on_support && supportLike(c);
    if (support) {
      if (!support_.seen && !policy_.support_label.empty()) {
        RCLCPP_INFO(node_->get_logger(), "%s: %s <-> %s", policy_.support_label.c_str(),
          a.c_str(), b.c_str());
      }
      support_.seen = true;support_.wall = Wall::now();support_.sim = node_->now().nanoseconds();
    }
    // While carrying, anything except the fingers (and the next support once seating has
    // started) is an unexpected contact.
    const bool seat_ok = seating_ && (policy_.seat_allows_support_only ? support : on_support);
    if (carrying_ && !finger && !seat_ok) {
      if (!carry_contact_) {
        RCLCPP_ERROR(node_->get_logger(), "UNEXPECTED CARRY CONTACT: %s <-> %s", a.c_str(),
          b.c_str());
      }
      carry_contact_ = true;
    }
    if (!table && !finger && has(a, b, "fr3v2")) {
      if (policy_.log_each_unexpected) {
        RCLCPP_ERROR(node_->get_logger(), "UNEXPECTED BOX CONTACT: %s <-> %s", a.c_str(),
          b.c_str());
      }
      unexpected_ = true;
    }
  }
}

void ContactMonitor::addSurface(const std::string & name, const std::string & topic,
  const std::string & box_filter, bool box_needs_vertical, std::function<bool()> watch_robot)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    surface_box_[name] = ContactStamp{};robot_touched_[name] = false;
  }
  subs_.push_back(node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
    topic, rclcpp::SensorDataQoS(),
    [this, name, box_filter, box_needs_vertical, watch_robot](
      ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & c : msg->contacts) {
        const auto & a = c.collision1.name;const auto & b = c.collision2.name;
        if (has(a, b, "target_box") && (box_filter.empty() || has(a, b, box_filter)) &&
        (!box_needs_vertical || supportLike(c)))
        {
          auto & stamp = surface_box_[name];
          stamp.seen = true;stamp.wall = Wall::now();stamp.sim = node_->now().nanoseconds();
        }
        if (has(a, b, "fr3v2") && watch_robot()) {
          if (!robot_touched_[name]) {
            RCLCPP_ERROR(node_->get_logger(), "ROBOT %s CONTACT: %s <-> %s", name.c_str(),
              a.c_str(), b.c_str());
          }
          robot_touched_[name] = true;
        }
      }
    }));
}

void ContactMonitor::setCarrying(bool value)
{
  std::lock_guard<std::mutex> lock(mutex_);carrying_ = value;
}

void ContactMonitor::setSeating(bool value)
{
  std::lock_guard<std::mutex> lock(mutex_);seating_ = value;
}

bool ContactMonitor::anyBoxContactSeen() const
{
  std::lock_guard<std::mutex> lock(mutex_);return any_seen_;
}

bool ContactMonitor::unexpected() const
{
  std::lock_guard<std::mutex> lock(mutex_);return unexpected_;
}

bool ContactMonitor::carryContact() const
{
  std::lock_guard<std::mutex> lock(mutex_);return carry_contact_;
}

bool ContactMonitor::robotTouched(const std::string & surface) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = robot_touched_.find(surface);
  return it != robot_touched_.end() && it->second;
}

bool ContactMonitor::fingersFresh(std::chrono::milliseconds age) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = Wall::now();
  return now - fingers_[0] < age && now - fingers_[1] < age;
}

ContactStamp ContactMonitor::support() const
{
  std::lock_guard<std::mutex> lock(mutex_);return support_;
}

ContactStamp ContactMonitor::surfaceBox(const std::string & surface) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = surface_box_.find(surface);
  return it == surface_box_.end() ? ContactStamp{} : it->second;
}

}  // namespace fr3_cooperative_control
