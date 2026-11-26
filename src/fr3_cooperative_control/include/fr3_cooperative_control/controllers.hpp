// Wait until the named ros2_control controllers are active.
#pragma once

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fr3_cooperative_control/common.hpp"

namespace fr3_cooperative_control
{

inline bool waitForControllers(
  const rclcpp::Node::SharedPtr & node, const std::vector<std::string> & names,
  std::chrono::seconds timeout)
{
  using namespace std::chrono_literals;
  auto client = node->create_client<controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers");
  require(client->wait_for_service(timeout), "Controller manager unavailable");
  const auto deadline = Wall::now() + timeout;
  while (rclcpp::ok() && Wall::now() < deadline) {
    auto future = client->async_send_request(
      std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
    if (future.wait_for(3s) == std::future_status::ready) {
      size_t count = 0;
      // Keep the response alive: future.get() hands over the only reference.
      const auto response = future.get();
      for (const auto & c : response->controller) {
        if (c.state == "active" && std::find(names.begin(), names.end(), c.name) != names.end()) {
          ++count;
        }
      }
      if (count == names.size()) {return true;}
    }
    std::this_thread::sleep_for(500ms);
  }
  return false;
}

}  // namespace fr3_cooperative_control
