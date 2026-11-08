// Physical pick, transfer and placement. MoveIt attachment does not constrain Gazebo.
#include <algorithm>
#include <array>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <Eigen/Geometry>
#include <control_msgs/action/gripper_command.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;
using Pose = geometry_msgs::msg::Pose;
using Group = moveit::planning_interface::MoveGroupInterface;
using Grip = control_msgs::action::GripperCommand;
using Wall = std::chrono::steady_clock;

namespace
{
void require(bool value, const std::string & message)
{
  if (!value) {throw std::runtime_error(message);}
}
Eigen::Vector3d point(const Pose & p) {return {p.position.x, p.position.y, p.position.z};}
Eigen::Quaterniond rotation(const Pose & p)
{
  return {p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z};
}
class Pick
{
public:
  explicit Pick(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    poses_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_box/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (stamp <= stamp_) {return;}
        stamp_ = stamp;box_ = msg->pose;box_.position.z -= 0.05;
        box_received_ = Wall::now();++sequence_;
      });
    joints_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          for (size_t j = 0; j < 2; ++j) {
            if (msg->name[i] == "left_fr3v2_finger_joint" + std::to_string(j + 1)) {
              fingers_[j] = msg->position[i];finger_received_[j] = Wall::now();
            }
          }
        }
      });
    contacts_ = node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/placement/box_contacts", rclcpp::SensorDataQoS(),
      [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!msg->contacts.empty()) {contacts_seen_ = true;}
        for (const auto & contact : msg->contacts) {
          const auto & a = contact.collision1.name;const auto & b = contact.collision2.name;
          const bool table = a.find("table_top_collision") != std::string::npos ||
          b.find("table_top_collision") != std::string::npos;
          const bool finger = a.find("left_fr3v2_leftfinger") != std::string::npos ||
          a.find("left_fr3v2_rightfinger") != std::string::npos ||
          b.find("left_fr3v2_leftfinger") != std::string::npos ||
          b.find("left_fr3v2_rightfinger") != std::string::npos;
          if (!table && !finger && (a.find("fr3v2") != std::string::npos ||
          b.find("fr3v2") != std::string::npos)) {unexpected_box_contact_ = true;}
          bool vertical = false;
          for (const auto & n : contact.normals) {vertical = vertical || std::abs(n.z) > 0.8;}
          double depth = 0;
          for (double d : contact.depths) {depth = std::max(depth, d);}
          if (table && vertical && !contact.depths.empty() && depth < 0.001) {
            if (!support_seen_) {
              RCLCPP_INFO(node_->get_logger(), "TABLE CONTACT: %s <-> %s", a.c_str(), b.c_str());
            }
            support_seen_ = true;support_wall_ = Wall::now();
            support_sim_ = node_->now().nanoseconds();
          }
        }
      });
    table_contacts_ = node_->create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/placement/table_contacts", rclcpp::SensorDataQoS(),
      [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & contact : msg->contacts) {
          const auto & a = contact.collision1.name;const auto & b = contact.collision2.name;
          if (a.find("target_box") != std::string::npos ||
          b.find("target_box") != std::string::npos)
          {
            table_support_wall_ = Wall::now();table_support_seen_ = true;
          }
          if (a.find("fr3v2") != std::string::npos || b.find("fr3v2") != std::string::npos) {
            if (!robot_table_contact_) {
              RCLCPP_ERROR(node_->get_logger(), "ROBOT TABLE CONTACT: %s <-> %s", a.c_str(),
                b.c_str());
            }
            robot_table_contact_ = true;
          }
        }
      });
    grip_ = rclcpp_action::create_client<Grip>(node_, "/left_gripper_controller/gripper_cmd");
  }
  void run()
  {
    std::string stage;node_->get_parameter_or<std::string>("stage", stage, "pick");
    require(stage == "pick" || stage == "transfer" || stage == "place", "Invalid test stage");
    auto controllers = node_->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
    require(controllers->wait_for_service(90s), "Controller manager unavailable");
    const auto deadline = Wall::now() + 90s;
    bool ready = false;
    while (rclcpp::ok() && Wall::now() < deadline) {
      auto future = controllers->async_send_request(
        std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
      if (future.wait_for(3s) == std::future_status::ready) {
        int count = 0;
        const auto response = future.get();
        for (const auto & c : response->controller) {
          if (c.state == "active" &&
            (c.name == "arm_1_controller" || c.name == "left_gripper_controller")) {++count;}
        }
        if (count == 2) {ready = true;break;}
      }
      std::this_thread::sleep_for(500ms);
    }
    require(ready && grip_->wait_for_action_server(30s), "Controllers not ready");
    arm_ = std::make_unique<Group>(node_, "arm_1");
    arm_->setEndEffectorLink(tcp_);arm_->setPoseReferenceFrame("world");
    arm_->setPlanningTime(10);arm_->setNumPlanningAttempts(3);
    arm_->setGoalPositionTolerance(0.002);arm_->setGoalOrientationTolerance(0.01);
    arm_->setMaxVelocityScalingFactor(0.25);arm_->setMaxAccelerationScalingFactor(0.25);
    require(arm_->startStateMonitor(15), "Robot state unavailable");
    scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    require(scene_->getAttachedObjects({"target_box"}).empty(), "Restart simulation: box attached");
    Pose initial = stableBox();
    require((point(initial) - Eigen::Vector3d(0, -1.04, 0.4825)).norm() < 0.005,
      "Box is not resting at expected shelf position");
    if (stage == "place") {
      const auto sensor_deadline = Wall::now() + 10s;
      bool seen = false;
      while (Wall::now() < sensor_deadline) {
        {std::lock_guard<std::mutex> lock(mutex_);seen = contacts_seen_;}
        if (seen) {break;}
        std::this_thread::sleep_for(100ms);
      }
      require(seen, "Box contact sensor unavailable; refusing placement test");
    }
    gripper(0.04, false);
    arm_->setJointValueTarget(std::vector<double>{0, -0.785, 0, -2.356, 0, 1.571, 0.785});
    executePlan();waitAt(nullptr);
    Pose pick = initial;
    pick.orientation.x = 0.5;pick.orientation.y = 0.5;
    pick.orientation.z = -0.5;pick.orientation.w = 0.5;
    addBox(initial);allowFingers();
    Pose pre = pick;pre.position.y += 0.12;
    arm_->setPoseTarget(pre);executePlan();arm_->clearPoseTargets();waitAt(&pre);
    linear(pick);waitAt(&pick);
    RCLCPP_INFO(node_->get_logger(), "PICK TARGET SETTLED: closing gripper now");
    gripper(0.018, true);
    const Pose grasped = stableBox();
    const Pose grasp_tcp = arm_->getCurrentPose(tcp_).pose;
    const Eigen::Vector3d reference = rotation(grasp_tcp).inverse() *
      (point(grasped) - point(grasp_tcp));
    const Eigen::Quaterniond reference_angle = rotation(grasp_tcp).inverse() * rotation(grasped);
    addBox(grasped);
    arm_->attachObject("target_box", tcp_,
      {"left_fr3v2_leftfinger", "left_fr3v2_rightfinger", "left_fr3v2_hand"});
    const auto attach_deadline = Wall::now() + 5s;
    while (scene_->getAttachedObjects({"target_box"}).empty() && Wall::now() < attach_deadline) {
      std::this_thread::sleep_for(100ms);
    }
    require(!scene_->getAttachedObjects({"target_box"}).empty(), "Planning attachment failed");
    Pose lifted = pick;lifted.position.z += 0.08;
    linear(lifted);waitAt(&lifted);
    const auto actual_box = stableBox();
    const double rise = actual_box.position.z - initial.position.z;
    RCLCPP_INFO(node_->get_logger(), "PHYSICAL LIFT: %.2f mm", rise * 1000);
    require(std::abs(rise - 0.08) < 0.005, "Physical box did not follow 80 mm lift");
    require((point(actual_box).head<2>() - point(initial).head<2>()).norm() < 0.01,
      "Box shifted sideways during lift");
    const auto sim_start = node_->now();const auto wall_start = Wall::now();
    double max_drift = 0, max_angle = 0;
    while ((node_->now() - sim_start).seconds() < 3.0) {
      require(Wall::now() - wall_start < 30s, "Hold simulation clock timed out");
      const Pose box = freshBox();const Pose tcp = arm_->getCurrentPose(tcp_).pose;
      max_drift = std::max(max_drift, (rotation(tcp).inverse() *
          (point(box) - point(tcp)) - reference).norm());
      max_angle = std::max(max_angle, reference_angle.angularDistance(
        rotation(tcp).inverse() * rotation(box)));
      require(max_drift < 0.005 && max_angle < 0.0873, "Box slipped or rotated in gripper");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(),
      "PICK TEST PASSED: lift=%.2f mm hold=3 sim s drift=%.3f mm angle=%.3f deg; gripper stays closed",
      rise * 1000, max_drift * 1000, max_angle * 180 / M_PI);
    if (stage == "pick") {return;}
    RCLCPP_INFO(node_->get_logger(), "TRANSFER RETRACT START: 120 mm, gripper stays closed");
    Pose clear = lifted;clear.position.y += 0.12;
    linear(clear);waitAt(&clear);
    holdTransferred("AFTER RETRACT", reference, reference_angle, max_drift, max_angle);
    // The shelf and table face opposite sides of the arm. Change hand orientation
    // during the collision-checked transfer; preserve the measured grasp transform.
    Pose table = clear;
    table.orientation.x = -0.5;table.orientation.y = -0.5;
    table.orientation.z = -0.5;table.orientation.w = 0.5;
    const Eigen::Vector3d desired_center(0.0, 0.05, 0.60);
    const Eigen::Vector3d target = desired_center - rotation(table) * reference;
    table.position.x = target.x();table.position.y = target.y();table.position.z = target.z();
    arm_->setMaxVelocityScalingFactor(0.10);arm_->setMaxAccelerationScalingFactor(0.10);
    RCLCPP_INFO(node_->get_logger(), "TABLE TRANSFER START: box target=(0.000, 0.050, 0.600)");
    arm_->setPoseTarget(table);executePlan();arm_->clearPoseTargets();waitAt(&table);
    holdTransferred("ABOVE TABLE", reference, reference_angle, max_drift, max_angle);
    const auto placed = freshBox();
    const double center_error = (point(placed) - desired_center).norm();
    const double normal_z = std::abs((rotation(placed) * Eigen::Vector3d::UnitZ()).z());
    const double tilt = std::acos(std::clamp(normal_z, 0.0, 1.0));
    RCLCPP_INFO(node_->get_logger(), "TABLE BOX: center_error=%.3f mm tilt=%.3f deg",
      center_error * 1000, tilt * 180 / M_PI);
    require(center_error < 0.01 && tilt < 0.0873, "Box not held level at table target");
    RCLCPP_INFO(node_->get_logger(),
      "TRANSFER TEST PASSED: drift=%.3f mm angle=%.3f deg; holding above table, gripper stays closed",
      max_drift * 1000, max_angle * 180 / M_PI);
    if (stage == "place") {place(reference, reference_angle);}
  }

private:
  double tilt(const Pose & box)
  {
    return std::acos(std::clamp(rotation(box).toRotationMatrix().row(2).cwiseAbs().maxCoeff(),
      0.0, 1.0));
  }
  double bottom(const Pose & box)
  {
    return box.position.z - 0.02 * rotation(box).toRotationMatrix().row(2).cwiseAbs().sum();
  }
  bool supported()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require(!robot_table_contact_, "Robot contacted tabletop; placement failed");
    require(!unexpected_box_contact_, "Box contacted a robot link outside the fingers");
    return support_seen_ && table_support_seen_ && Wall::now() - table_support_wall_ < 1s &&
           Wall::now() - support_wall_ < 1s &&
           node_->now().nanoseconds() - support_sim_ < 250000000;
  }
  void syncHeldBox(const Pose & box, const Pose & tcp)
  {
    const Eigen::Vector3d relative = rotation(tcp).inverse() * (point(box) - point(tcp));
    const Eigen::Quaterniond q = rotation(tcp).inverse() * rotation(box);
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = tcp_;attached.touch_links =
    {"left_fr3v2_leftfinger", "left_fr3v2_rightfinger", "left_fr3v2_hand"};
    attached.object.id = "target_box";attached.object.header.frame_id = tcp_;
    attached.object.operation = attached.object.ADD;
    shape_msgs::msg::SolidPrimitive primitive;primitive.type = primitive.BOX;
    primitive.dimensions = {0.04, 0.04, 0.04};
    Pose p;p.position.x = relative.x();p.position.y = relative.y();p.position.z = relative.z();
    p.orientation.w = q.w();p.orientation.x = q.x();p.orientation.y = q.y();p.orientation.z = q.z();
    attached.object.primitives.push_back(primitive);attached.object.primitive_poses.push_back(p);
    require(scene_->applyAttachedCollisionObject(attached), "Cannot synchronize held planning box");
  }
  void waitSupport()
  {
    const auto deadline = Wall::now() + 15s;
    int64_t since = -1;
    while (Wall::now() < deadline) {
      const auto box = freshBox();
      const bool good = supported() && std::abs(bottom(box) - 0.4) < 0.001 &&
        tilt(box) < 0.00873;
      const auto now = node_->now().nanoseconds();
      if (!good) {since = -1;} else if (since < 0) {
        since = now;
      } else if (now - since >= 1000000000) {
        RCLCPP_INFO(node_->get_logger(),
            "SUPPORT VERIFIED: 1 sim s table contact, bottom_gap=%.3f mm",
          (bottom(box) - 0.4) * 1000);return;
      }
      std::this_thread::sleep_for(100ms);
    }
    throw std::runtime_error("Table contact/level support not verified; refusing gripper open");
  }
  void place(const Eigen::Vector3d & reference, const Eigen::Quaterniond & reference_angle)
  {
    const auto box = stableBox();const auto tcp = arm_->getCurrentPose(tcp_).pose;
    syncHeldBox(box, tcp);
    const Eigen::Vector3d relative = rotation(tcp).inverse() * (point(box) - point(tcp));
    const Eigen::Quaterniond relative_angle = rotation(tcp).inverse() * rotation(box);
    // A cube can rest on any face. Rotate 90 degrees about world X so the
    // fingers approach from above and the wrist stays clear of the tabletop.
    const double half = std::sqrt(0.5);
    const Eigen::Quaterniond desired_box(half, half, 0, 0);
    const Eigen::Quaterniond aligned = desired_box * relative_angle.inverse();
    Pose goal = tcp;
    goal.orientation.w = aligned.w();goal.orientation.x = aligned.x();
    goal.orientation.y = aligned.y();goal.orientation.z = aligned.z();
    const Eigen::Vector3d pre_center(0, 0.05, 0.50);
    const Eigen::Vector3d pre_tcp = pre_center - aligned * relative;
    goal.position.x = pre_tcp.x();goal.position.y = pre_tcp.y();goal.position.z = pre_tcp.z();
    RCLCPP_INFO(node_->get_logger(), "PLACE TOP-DOWN ALIGN AND APPROACH: box center z=0.500");
    arm_->setGoalPositionTolerance(0.00015);arm_->setGoalOrientationTolerance(0.0005);
    arm_->setMaxVelocityScalingFactor(0.05);arm_->setMaxAccelerationScalingFactor(0.05);
    arm_->setPoseTarget(goal);executePlan();arm_->clearPoseTargets();
    waitAt(&goal, 0.0003, 0.0015);
    require(tilt(freshBox()) < 0.00873, "Box not parallel enough for descent");
    double drift = 0, angle = 0;
    holdTransferred("PREPLACE", reference, reference_angle, drift, angle);
    auto current_box = freshBox();auto current_tcp = arm_->getCurrentPose(tcp_).pose;
    syncHeldBox(current_box, current_tcp);
    goal.position.x = current_tcp.position.x - current_box.position.x;
    goal.position.y = current_tcp.position.y + 0.05 - current_box.position.y;
    goal.position.z = current_tcp.position.z + 0.422 - current_box.position.z;
    RCLCPP_INFO(node_->get_logger(), "PLACE DESCENT: box center z=0.422, then contact steps");
    linear(goal, 0.02);waitAt(&goal, 0.0003, 0.0015);
    // Remain closed on the 40 mm cube, but remove most of the commanded
    // overclosure before the last 2 mm of seating. Both fingers stay below
    // 20 mm in the command; this does not request a clearance around the cube.
    const auto before_relax = freshBox();
    gripper(0.0198, true);
    current_box = stableBox();current_tcp = arm_->getCurrentPose(tcp_).pose;
    RCLCPP_INFO(node_->get_logger(),
      "SEAT GRIP RELAX: shift=%.3f mm bottom_gap=%.3f mm box=(%.5f, %.5f, %.5f)",
      (point(current_box) - point(before_relax)).norm() * 1000,
      (bottom(current_box) - 0.4) * 1000, current_box.position.x,
      current_box.position.y, current_box.position.z);
    require((point(current_box) - point(before_relax)).norm() < 0.003 &&
      bottom(current_box) - 0.4 > -0.0003, "Box moved excessively while reducing overclosure");
    syncHeldBox(current_box, current_tcp);
    allowFingers(true);  // Only box-table support; robot-table collisions remain checked.
    bool seated = false;
    for (int step = 0; step < 10; ++step) {
      current_box = freshBox();
      require(tilt(current_box) < 0.00873, "Box tilted during seating");
      const double gap = bottom(current_box) - 0.4;
      RCLCPP_INFO(node_->get_logger(), "SEAT step=%d bottom_gap=%.3f mm contact=%d",
        step, gap * 1000, supported());
      require(gap > -0.0003, "Box penetration too large; refusing further descent");
      if (supported() && std::abs(gap) < 0.001) {seated = true;break;}
      if (gap <= 0.00015) {waitSupport();seated = true;break;}
      goal = arm_->getCurrentPose(tcp_).pose;
      goal.orientation.w = aligned.w();goal.orientation.x = aligned.x();
      goal.orientation.y = aligned.y();goal.orientation.z = aligned.z();
      goal.position.z -= std::min(0.0005, gap + 0.0001);
      linear(goal, 0.01);waitAt(&goal, 0.00015, 0.0015);
    }
    require(seated, "Could not establish table support within bounded seating steps");
    waitSupport();
    const auto release_reference = freshBox();
    RCLCPP_INFO(node_->get_logger(), "OPEN ON TABLE: physical support verified, two-step opening");
    gripper(0.021, false);  // Remove squeeze gently before the full opening.
    waitSupport();
    const auto soft_box = freshBox();
    double release_slip = (point(soft_box) - point(release_reference)).norm();
    RCLCPP_INFO(node_->get_logger(), "SOFT OPEN: delta=%.3f mm box=(%.5f, %.5f, %.5f)",
      release_slip * 1000, soft_box.position.x, soft_box.position.y, soft_box.position.z);
    gripper(0.04, false);
    arm_->detachObject("target_box");
    const auto detach_deadline = Wall::now() + 5s;
    while (!scene_->getAttachedObjects({"target_box"}).empty() && Wall::now() < detach_deadline) {
      std::this_thread::sleep_for(100ms);
    }
    require(scene_->getAttachedObjects({"target_box"}).empty(), "Planning detach failed");
    const auto open_box = stableBox();
    release_slip = std::max(release_slip, (point(open_box) - point(release_reference)).norm());
    RCLCPP_INFO(node_->get_logger(), "BEFORE RETREAT: release_slip=%.3f mm box=(%.5f, %.5f, %.5f)",
      release_slip * 1000, open_box.position.x, open_box.position.y, open_box.position.z);
    addBox(open_box);
    goal = arm_->getCurrentPose(tcp_).pose;goal.position.y -= 0.12;
    RCLCPP_INFO(node_->get_logger(), "PLACE RETREAT: 120 mm horizontally, no ready return");
    linear(goal, 0.03);waitAt(&goal);
    const auto start = node_->now();const auto wall = Wall::now();
    double max_slip = release_slip;
    while ((node_->now() - start).seconds() < 3.0) {
      require(Wall::now() - wall < 30s, "Post-release simulation clock timed out");
      current_box = freshBox();
      max_slip = std::max(max_slip, (point(current_box) - point(release_reference)).norm());
      RCLCPP_INFO(node_->get_logger(),
        "POST RELEASE: box=(%.5f, %.5f, %.5f) xy=%.3f mm slip=%.3f mm tilt=%.3f deg",
        current_box.position.x, current_box.position.y, current_box.position.z,
        std::hypot(current_box.position.x, current_box.position.y - 0.05) * 1000,
        max_slip * 1000, tilt(current_box) * 180 / M_PI);
      require(supported(), "Box lost table support after release");
      require(max_slip < 0.002 && std::hypot(current_box.position.x,
        current_box.position.y - 0.05) < 0.003 &&
        std::abs(current_box.position.z - 0.42) < 0.001 && tilt(current_box) < 0.01745,
        "Released box moved or tilted outside placement tolerance");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(),
      "PLACE TEST PASSED: box=(%.5f, %.5f, %.5f) xy_error=%.3f mm release_slip=%.3f mm tilt=%.3f deg",
      current_box.position.x, current_box.position.y, current_box.position.z,
      std::hypot(current_box.position.x, current_box.position.y - 0.05) * 1000,
      max_slip * 1000, tilt(current_box) * 180 / M_PI);
  }

  void holdTransferred(
    const char * label, const Eigen::Vector3d & reference,
    const Eigen::Quaterniond & reference_angle, double & max_drift, double & max_angle)
  {
    const auto sim_start = node_->now();const auto wall_start = Wall::now();
    while ((node_->now() - sim_start).seconds() < 3.0) {
      require(Wall::now() - wall_start < 30s, "Transfer hold simulation clock timed out");
      const auto box = freshBox();const auto tcp = arm_->getCurrentPose(tcp_).pose;
      max_drift = std::max(max_drift, (rotation(tcp).inverse() *
          (point(box) - point(tcp)) - reference).norm());
      max_angle = std::max(max_angle, reference_angle.angularDistance(
        rotation(tcp).inverse() * rotation(box)));
      RCLCPP_DEBUG(node_->get_logger(), "%s: drift=%.3f mm angle=%.3f deg", label,
        max_drift * 1000, max_angle * 180 / M_PI);
      require(max_drift < 0.005 && max_angle < 0.0873,
        std::string(label) + ": physical box slipped or rotated in gripper");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(), "%s HOLD PASSED: drift=%.3f mm angle=%.3f deg", label,
      max_drift * 1000, max_angle * 180 / M_PI);
  }

  Pose freshBox()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require(sequence_ > 0 && Wall::now() - box_received_ < 2s, "Physical box pose stale/missing");
    return box_;
  }
  Pose stableBox()
  {
    Pose previous;size_t last_sequence = 0;int calm = 0;
    const auto end = Wall::now() + 15s;
    while (Wall::now() < end) {
      std::this_thread::sleep_for(200ms);
      Pose current;size_t sequence;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (sequence_ == 0 || sequence_ == last_sequence) {continue;}
        current = box_;sequence = sequence_;
      }
      calm = last_sequence && (point(current) - point(previous)).norm() < 0.0005 &&
        rotation(current).angularDistance(rotation(previous)) < 0.01 ? calm + 1 : 0;
      previous = current;last_sequence = sequence;
      if (calm >= 3) {return freshBox();}
    }
    throw std::runtime_error("Physical box did not settle");
  }
  void waitAt(
    const Pose * target, double position_tolerance = 0.003,
    double angle_tolerance = 0.025)
  {
    auto previous = arm_->getCurrentJointValues();int calm = 0;
    const auto end = Wall::now() + 20s;
    while (Wall::now() < end) {
      std::this_thread::sleep_for(200ms);
      auto state = arm_->getCurrentState(2.0);
      require(static_cast<bool>(state), "Fresh arm state missing");
      std::vector<double> current;state->copyJointGroupPositions("arm_1", current);
      require(current.size() == 7 && previous.size() == 7, "Incomplete arm state");
      double delta = 0;
      for (size_t i = 0; i < 7; ++i) {delta = std::max(delta, std::abs(current[i] - previous[i]));}
      bool near = true;
      if (target) {
        state->update();const auto actual = state->getGlobalLinkTransform(tcp_);
        near = (actual.translation() - point(*target)).norm() < position_tolerance &&
          Eigen::Quaterniond(actual.rotation()).angularDistance(rotation(*target)) <
          angle_tolerance;
      }
      calm = delta < 0.0002 && near ? calm + 1 : 0;previous = current;
      if (calm >= 3) {return;}
    }
    const auto actual = arm_->getCurrentPose(tcp_).pose;
    if (target) {
      RCLCPP_ERROR(node_->get_logger(), "SETTLE FAILED: position_error=%.3f mm angle=%.4f rad",
        1000 * (point(actual) - point(*target)).norm(),
        rotation(actual).angularDistance(rotation(*target)));
    }
    auto state = arm_->getCurrentState(2.0);
    if (state) {
      for (const auto & name : state->getJointModelGroup("arm_1")->getVariableNames()) {
        RCLCPP_ERROR(node_->get_logger(), "SETTLE JOINT %s actual=%.6f", name.c_str(),
          state->getVariablePosition(name));
      }
    }
    throw std::runtime_error("Arm not settled at target; refusing next motion");
  }
  void executePlan()
  {
    Group::Plan plan;bool planned = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      arm_->setStartStateToCurrentState();
      if (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {planned = true;break;}
    }
    require(planned, "Planning failed");
    require(arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS, "Execution failed");
  }
  void finePath(const Pose & target, robot_trajectory::RobotTrajectory & trajectory)
  {
    auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    require(client->wait_for_service(3s), "Fine path planning scene unavailable");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    using Components = moveit_msgs::msg::PlanningSceneComponents;
    request->components.components = Components::SCENE_SETTINGS | Components::ROBOT_STATE |
      Components::ROBOT_STATE_ATTACHED_OBJECTS | Components::WORLD_OBJECT_GEOMETRY |
      Components::OCTOMAP | Components::TRANSFORMS | Components::ALLOWED_COLLISION_MATRIX |
      Components::LINK_PADDING_AND_SCALING;
    auto future = client->async_send_request(request);
    require(future.wait_for(5s) == std::future_status::ready, "Fine planning scene timed out");
    planning_scene::PlanningScene scene(arm_->getRobotModel());
    require(scene.setPlanningSceneMsg(future.get()->scene), "Fine planning scene import failed");
    auto state = scene.getCurrentState();
    const auto * group = state.getJointModelGroup("arm_1");
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = point(target);
    transform.linear() = rotation(target).toRotationMatrix();
    std::vector<moveit::core::RobotStatePtr> path;
    const auto valid = [&scene](moveit::core::RobotState * candidate,
      const moveit::core::JointModelGroup * joints, const double * values) {
        candidate->setJointGroupPositions(joints, values);candidate->update();
        return candidate->satisfiesBounds(joints) && !scene.isStateColliding(*candidate, "arm_1");
      };
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &state, group, path, state.getLinkModel(tcp_), transform, true,
      moveit::core::MaxEEFStep(0.0001), moveit::core::CartesianPrecision{0.00001, 0.0001, 0.00001},
        valid, kinematics::KinematicsQueryOptions());
    require(fraction >= 0.999 && path.size() >= 2, "Incomplete collision-checked fine path");
    for (size_t i = 0; i < path.size(); ++i) {
      if (i > 0) {
        for (const auto & name : group->getVariableNames()) {
          require(std::abs(path[i]->getVariablePosition(name) -
            path[i - 1]->getVariablePosition(name)) < 0.02, "Joint jump in fine path");
        }
      }
      trajectory.addSuffixWayPoint(*path[i], 0.0);
    }
  }

  void linear(const Pose & target, double scale = 0.1)
  {
    arm_->setStartStateToCurrentState();moveit_msgs::msg::RobotTrajectory msg;
    auto state = arm_->getCurrentState(2.0);
    require(static_cast<bool>(state), "No trajectory state");
    robot_trajectory::RobotTrajectory trajectory(arm_->getRobotModel(), "arm_1");
    const bool fine = scale <= 0.01;
    if (fine) {finePath(target, trajectory);} else {
      const double fraction = arm_->computeCartesianPath({target}, 0.002, msg, true);
      if (fraction < 0.999) {
        moveit_msgs::msg::RobotTrajectory probe;
        const double unchecked = arm_->computeCartesianPath({target}, 0.002, probe, false);
        RCLCPP_ERROR(node_->get_logger(),
          "CARTESIAN DIAG: checked=%.3f unchecked=%.3f (probe not executed)", fraction, unchecked);
        throw std::runtime_error("Incomplete collision-checked Cartesian path");
      }
      trajectory.setRobotTrajectoryMsg(*state, msg);
    }
    if (fine) {
      const auto raw = trajectory.getLastWayPoint().getGlobalLinkTransform(tcp_);
      RCLCPP_INFO(node_->get_logger(), "RAW FINE ENDPOINT: error=%.4f mm",
        1000 * (raw.translation() - point(target)).norm());
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration timing(
      fine ? 0.00001 : 0.001, fine ? 0.005 : 0.01, fine ? 0.000001 : 0.0001);
    require(timing.computeTimeStamps(trajectory, scale, scale), "Trajectory timing failed");
    const auto endpoint = trajectory.getLastWayPoint().getGlobalLinkTransform(tcp_);
    const double error = (endpoint.translation() - point(target)).norm();
    const double angle = Eigen::Quaterniond(endpoint.rotation()).angularDistance(rotation(target));
    RCLCPP_INFO(node_->get_logger(), "TIMED ENDPOINT: error=%.3f mm angle=%.4f rad",
      error * 1000, angle);
    require(error < (fine ? 0.00005 : 0.003) && angle < 0.025,
      "Timed endpoint diverged from target");
    for (const auto & name : state->getJointModelGroup("arm_1")->getVariableNames()) {
      RCLCPP_INFO(node_->get_logger(), "TIMED JOINT %s target=%.6f", name.c_str(),
        trajectory.getLastWayPoint().getVariablePosition(name));
    }
    trajectory.getRobotTrajectoryMsg(msg);
    require(arm_->execute(msg) == moveit::core::MoveItErrorCode::SUCCESS,
        "Cartesian execution failed");
  }
  void gripper(double position, bool grasp)
  {
    Grip::Goal goal;goal.command.position = position;goal.command.max_effort = 20;
    auto sent = grip_->async_send_goal(goal);
    require(sent.wait_for(5s) == std::future_status::ready, "Gripper goal timed out");
    auto handle = sent.get();require(static_cast<bool>(handle), "Gripper goal rejected");
    auto future = grip_->async_get_result(handle);
    if (future.wait_for(30s) != std::future_status::ready) {
      grip_->async_cancel_goal(handle);throw std::runtime_error("Gripper result timed out");
    }
    auto result = future.get();
    const bool stopped_by_box = grasp && result.result && result.result->stalled &&
      result.code == rclcpp_action::ResultCode::ABORTED;
    require(result.code == rclcpp_action::ResultCode::SUCCEEDED || stopped_by_box,
      "Gripper command failed");
    std::this_thread::sleep_for(500ms);
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < 2; ++i) {
      require(Wall::now() - finger_received_[i] < 1s, "Physical finger state stale");
      require(grasp ? fingers_[i] > 0.0185 && fingers_[i] < 0.022 :
        std::abs(fingers_[i] - position) < 0.001, "Physical finger opening unexpected");
    }
    RCLCPP_INFO(node_->get_logger(), "PHYSICAL FINGERS: %.4f / %.4f m stalled=%d",
      fingers_[0], fingers_[1], result.result ? result.result->stalled : false);
  }
  void addBox(const Pose & box)
  {
    moveit_msgs::msg::CollisionObject object;object.header.frame_id = "world";
    object.id = "target_box";object.operation = object.ADD;
    shape_msgs::msg::SolidPrimitive primitive;primitive.type = primitive.BOX;
    primitive.dimensions = {0.04, 0.04, 0.04};
    object.primitives.push_back(primitive);object.primitive_poses.push_back(box);
    require(scene_->applyCollisionObject(object), "Planning box synchronization failed");
  }
  void allowFingers(bool support = false)
  {
    auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    require(client->wait_for_service(10s), "Planning scene unavailable");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = client->async_send_request(request);
    require(future.wait_for(5s) == std::future_status::ready, "Collision matrix timeout");
    auto acm = future.get()->scene.allowed_collision_matrix;
    for (const std::string name : {"target_box", "left_fr3v2_leftfinger",
        "left_fr3v2_rightfinger", "table_link"})
    {
      if (std::find(acm.entry_names.begin(), acm.entry_names.end(),
          name) == acm.entry_names.end())
      {
        acm.entry_names.push_back(name);
        for (auto & row : acm.entry_values) {row.enabled.push_back(false);}
        moveit_msgs::msg::AllowedCollisionEntry row;
        row.enabled.resize(acm.entry_names.size(), false);
        acm.entry_values.push_back(row);
      }
    }
    const auto box = std::find(acm.entry_names.begin(), acm.entry_names.end(), "target_box") -
      acm.entry_names.begin();
    for (size_t i = 0; i < acm.entry_names.size(); ++i) {
      if (acm.entry_names[i] == "left_fr3v2_leftfinger" ||
        acm.entry_names[i] == "left_fr3v2_rightfinger" ||
        (support && acm.entry_names[i] == "table_link"))
      {
        acm.entry_values[box].enabled[i] = true;acm.entry_values[i].enabled[box] = true;
      }
    }
    moveit_msgs::msg::PlanningScene diff;diff.is_diff = true;diff.allowed_collision_matrix = acm;
    require(scene_->applyPlanningScene(diff), "Finger collision allowance failed");
  }
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contacts_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr table_contacts_;
  bool robot_table_contact_ = false, table_support_seen_ = false;
  Wall::time_point table_support_wall_;
  bool contacts_seen_ = false, support_seen_ = false, unexpected_box_contact_ = false;
  Wall::time_point support_wall_;
  int64_t support_sim_ = 0;
  rclcpp::Node::SharedPtr node_;
  std::mutex mutex_;
  Pose box_;size_t sequence_ = 0;int64_t stamp_ = -1;
  Wall::time_point box_received_;
  std::array<double, 2> fingers_{};
  std::array<Wall::time_point, 2> finger_received_{};
  const std::string tcp_ = "left_fr3v2_hand_tcp";
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poses_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp_action::Client<Grip>::SharedPtr grip_;
  std::unique_ptr<Group> arm_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("single_arm_pick_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  std::thread spinner([&executor] {executor.spin();});
  int rc = 0;
  std::unique_ptr<Pick> test;
  try {
    test = std::make_unique<Pick>(node);
    test->run();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
      "SINGLE ARM TEST FAILED: %s; test stopped, inspect simulation before restarting", e.what());
    rc = 1;
  }
  executor.cancel();spinner.join();test.reset();rclcpp::shutdown();return rc;
}
