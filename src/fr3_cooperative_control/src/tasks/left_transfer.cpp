// Left arm: physical pick from the source shelf cell, transfer and placement on the table.
// MoveIt attachment does not constrain Gazebo; every critical step is verified physically.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fr3_cooperative_control/left_transfer.hpp"
#include "fr3_cooperative_control/arm_motion.hpp"
#include "fr3_cooperative_control/box_tracker.hpp"
#include "fr3_cooperative_control/box_weld.hpp"
#include "fr3_cooperative_control/common.hpp"
#include "fr3_cooperative_control/contact_monitor.hpp"
#include "fr3_cooperative_control/controllers.hpp"
#include "fr3_cooperative_control/gripper.hpp"
#include "fr3_cooperative_control/world_model.hpp"

using namespace std::chrono_literals;
namespace fc = fr3_cooperative_control;
using fc::Pose;
using fc::Wall;
using fc::point;
using fc::require;
using fc::rotation;
using Group = fc::Group;

namespace
{
fc::ContactPolicy leftPolicy()
{
  fc::ContactPolicy policy;
  policy.support_collision = "table_top_collision";
  policy.seat_allows_support_only = false;
  policy.support_label = "TABLE CONTACT";
  return policy;
}
fc::MotionProfile leftProfile()
{
  fc::MotionProfile profile;
  profile.fine_scale_threshold = 0.01;
  profile.coarse_position_tolerance = 0.003;
  profile.coarse_angle_tolerance = 0.025;
  profile.fine_angle_tolerance = 0.025;
  return profile;
}

class Pick
{
public:
  explicit Pick(rclcpp::Node::SharedPtr node)
  : node_(node), names_("left"), task_(fc::TaskConfig::fromParameters(*node)),
    box_(node, task_.box_size, task_.gazebo_z_offset), gripper_(node, names_),
    contacts_(node, names_, leftPolicy()),
    motion_(node, names_, leftProfile(), nullptr, [this] {carrySafe();})
  {
    fc::setSpeedScale(task_.speed_scale);
    contacts_.addSurface("TABLE", "/placement/table_contacts", "", false);
    if (task_.rigid_grasp) {
      left_weld_ = std::make_unique<fc::BoxWeld>(node_, "left");
      right_weld_ = std::make_unique<fc::BoxWeld>(node_, "right");
    }
  }

  void run(const std::string & stage)
  {
    require(stage == "pick" || stage == "transfer" || stage == "place", "Invalid test stage");
    // Rigid-grasp test mode: Gazebo welds the box to both hands when it spawns.
    if (left_weld_) {left_weld_->release(true);right_weld_->release(true);}
    const bool ready = fc::waitForControllers(node_, {"arm_1_controller", "left_gripper_controller"},
      90s);
    require(ready && gripper_.waitForServer(30s), "Controllers not ready");
    motion_.connect();
    world_ = std::make_unique<fc::WorldModel>(arm().getRobotModel(), task_);
    arm().setEndEffectorLink(tcp());arm().setPoseReferenceFrame("world");
    arm().setPlanningTime(10);arm().setNumPlanningAttempts(3);
    arm().setGoalPositionTolerance(0.002);arm().setGoalOrientationTolerance(0.01);
    // Boxless motions only (ready and pregrasp); carry motions set their own lower scales.
    arm().setMaxVelocityScalingFactor(fc::speed(0.40));arm().setMaxAccelerationScalingFactor(fc::speed(0.40));
    require(arm().startStateMonitor(15), "Robot state unavailable");
    require(motion_.scene().getAttachedObjects({"target_box"}).empty(),
      "Restart simulation: box attached");
    const fc::Cell source = world_->sourceCell();
    Pose initial = box_.stable();
    require((point(initial) - source.center).norm() < 0.005,
      "Box is not resting at expected shelf position");
    if (stage == "place") {
      const auto sensor_deadline = Wall::now() + 10s;
      bool seen = false;
      while (Wall::now() < sensor_deadline) {
        seen = contacts_.anyBoxContactSeen();
        if (seen) {break;}
        std::this_thread::sleep_for(100ms);
      }
      require(seen, "Box contact sensor unavailable; refusing placement test");
    }
    gripper_.command(0.04, false);
    arm().setJointValueTarget(std::vector<double>{0, -0.785, 0, -2.356, 0, 1.571, 0.785});
    executePlan();motion_.waitAt(nullptr);
    addBox(initial);allowFingers();
    // Rehearse the grasp candidates (planning only) and take the first that can approach, lift
    // and retreat with joint margin: straight side grasp, then hand pitched 20/35 deg down.
    const PickChoice choice = choosePickGrasp(initial);
    const Pose pick = choice.pick;
    const Pose pre = choice.pre;
    arm().setJointValueTarget(choice.pre_joints);executePlan();motion_.waitAt(&pre);
    motion_.linear(pick);motion_.waitAt(&pick);
    RCLCPP_INFO(node_->get_logger(), "PICK TARGET SETTLED: closing gripper now");
    gripper_.command(0.018, true);
    if (left_weld_) {left_weld_->attach();}  // test mode: rigid for the transfer motions
    const Pose grasped = box_.stable();
    const Pose grasp_tcp = arm().getCurrentPose(tcp()).pose;
    const Eigen::Vector3d reference = rotation(grasp_tcp).inverse() *
      (point(grasped) - point(grasp_tcp));
    const Eigen::Quaterniond reference_angle = rotation(grasp_tcp).inverse() * rotation(grasped);
    addBox(grasped);
    motion_.attachBox();
    Pose lifted = pick;lifted.position.z += 0.08;
    motion_.linear(lifted);motion_.waitAt(&lifted);
    const auto actual_box = box_.stable();
    const double rise = actual_box.position.z - initial.position.z;
    RCLCPP_INFO(node_->get_logger(), "PHYSICAL LIFT: %.2f mm", rise * 1000);
    require(std::abs(rise - 0.08) < 0.005, "Physical box did not follow 80 mm lift");
    require((point(actual_box).head<2>() - point(initial).head<2>()).norm() < 0.01,
      "Box shifted sideways during lift");
    const auto sim_start = node_->now();const auto wall_start = Wall::now();
    double max_drift = 0, max_angle = 0;
    while ((node_->now() - sim_start).seconds() < 3.0) {
      require(Wall::now() - wall_start < 30s, "Hold simulation clock timed out");
      const Pose box = box_.fresh();const Pose tcp_pose = arm().getCurrentPose(tcp()).pose;
      max_drift = std::max(max_drift, (rotation(tcp_pose).inverse() *
          (point(box) - point(tcp_pose)) - reference).norm());
      max_angle = std::max(max_angle, reference_angle.angularDistance(
        rotation(tcp_pose).inverse() * rotation(box)));
      require(max_drift < 0.005 && max_angle < 0.0873, "Box slipped or rotated in gripper");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(),
      "PICK TEST PASSED: lift=%.2f mm hold=3 sim s drift=%.3f mm angle=%.3f deg; gripper stays closed",
      rise * 1000, max_drift * 1000, max_angle * 180 / M_PI);
    if (stage == "pick") {return;}
    contacts_.setCarrying(true);
    RCLCPP_INFO(node_->get_logger(), "TRANSFER RETRACT START: %.0f mm, gripper stays closed",
      choice.retreat * 1000);
    Pose clear = lifted;
    clear.position.y -= world_->sourceShelf().inward() * choice.retreat;
    motion_.linear(clear);motion_.waitAt(&clear);
    holdTransferred("AFTER RETRACT", reference, reference_angle, max_drift, max_angle, 1.0);
    // The shelf and table face opposite sides of the arm. Change hand orientation
    // during the collision-checked transfer; preserve the measured grasp transform.
    const fc::Cell handoff = world_->handoffCell();
    Pose table = clear;
    table.orientation.x = -0.5;table.orientation.y = -0.5;
    table.orientation.z = -0.5;table.orientation.w = 0.5;
    if (choice.pitch_deg != 0) {
      // Pitched grasp: keep the carried box level above the table (nearest axis-aligned
      // orientation of the box under the nominal transfer hand orientation).
      const Eigen::Quaterniond level =
        fc::nearestCubeRotation(rotation(table) * reference_angle);
      const Eigen::Quaterniond hand = level * reference_angle.inverse();
      table.orientation.w = hand.w();table.orientation.x = hand.x();
      table.orientation.y = hand.y();table.orientation.z = hand.z();
    }
    const Eigen::Vector3d desired_center(
      handoff.center.x(), handoff.center.y(), handoff.surface + 0.20);
    const Eigen::Vector3d target = desired_center - rotation(table) * reference;
    table.position.x = target.x();table.position.y = target.y();table.position.z = target.z();
    arm().setMaxVelocityScalingFactor(fc::speed(0.10));arm().setMaxAccelerationScalingFactor(fc::speed(0.10));
    RCLCPP_INFO(node_->get_logger(), "TABLE TRANSFER START: box target=(%.3f, %.3f, %.3f)",
      desired_center.x(), desired_center.y(), desired_center.z());
    const auto clear_of_table = [this, &reference, &reference_angle](const Group::Plan & plan) {
        return carryClearance(plan, reference, reference_angle, "TABLE TRANSFER");
      };
    arm().setPoseTarget(table);
    executePlan(clear_of_table, [this, &reference, &reference_angle](const Group::Plan & plan) {
        executeMonitored(plan.trajectory, "TABLE TRANSFER", reference, reference_angle);
      });
    arm().clearPoseTargets();
    motion_.waitAt(&table);
    holdTransferred("ABOVE TABLE", reference, reference_angle, max_drift, max_angle);
    const auto placed = box_.fresh();
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
  Group & arm() {return motion_.group();}
  // Test mode: while welded Gazebo reports no finger contacts, so finger checks are skipped.
  bool welded() const {return left_weld_ && left_weld_->attached();}
  const std::string & tcp() const {return names_.tcp;}
  void addBox(const Pose & box) {motion_.addBox(box, task_.box_size);}
  void syncHeldBox(const Pose & box, const Pose & tcp_pose)
  {
    motion_.syncHeldBox(box, tcp_pose, task_.box_size);
  }
  // Fingers may touch the box; with `support`, the box may also touch the tabletop.
  void allowFingers(bool support = false)
  {
    std::vector<std::pair<std::string, bool>> entries = {
      {names_.left_finger, true}, {names_.right_finger, true}};
    if (support) {entries.emplace_back("table_link", true);}
    motion_.setBoxCollisions(entries);
  }
  void executePlan(const fc::ArmMotion::Accept & accept = nullptr,
    const fc::ArmMotion::Run & run = nullptr)
  {
    // OMPL paths are sometimes rejected by ValidateSolution after dense time
    // parameterization (arm link touching the table). Each attempt takes ~0.1 s,
    // so allow more fresh attempts; collision validation itself is unchanged.
    fc::PlanOptions options;
    options.attempts = 10;options.clamp_start = true;options.log_attempts = true;
    motion_.executePlan(options, accept, run);
  }

  bool supported()
  {
    require(!contacts_.robotTouched("TABLE"), "Robot contacted tabletop; placement failed");
    require(!contacts_.unexpected(), "Box contacted a robot link outside the fingers");
    const auto box_support = contacts_.support();
    const auto table_support = contacts_.surfaceBox("TABLE");
    const auto now = Wall::now();
    return box_support.seen && table_support.seen && now - table_support.wall < 1s &&
           now - box_support.wall < 1s &&
           node_->now().nanoseconds() - box_support.sim < 250000000;
  }
  // Diagnostic only: which part of supported() is currently false.
  std::string supportDiagnosis()
  {
    const auto box_support = contacts_.support();
    const auto table_support = contacts_.surfaceBox("TABLE");
    const auto wall = Wall::now();
    auto age = [&](const Wall::time_point & t) {
        return std::chrono::duration<double>(wall - t).count();
      };
    char text[256];
    std::snprintf(text, sizeof(text),
      "box_sensor(seen=%d wall_age=%.2fs sim_age=%.2fs) table_sensor(seen=%d wall_age=%.2fs)",
      box_support.seen, box_support.seen ? age(box_support.wall) : -1.0,
      box_support.seen ? (node_->now().nanoseconds() - box_support.sim) * 1e-9 : -1.0,
      table_support.seen, table_support.seen ? age(table_support.wall) : -1.0);
    return text;
  }
  void waitSupport(bool reseat = false)
  {
    const double surface = world_->handoffCell().surface;
    auto deadline = Wall::now() + 15s;
    int64_t since = -1, hover_since = -1;int reseats = 0;
    int samples = 0, resets = 0, no_contact = 0, bad_gap = 0, bad_tilt = 0;
    double last_gap = 0, last_tilt = 0;
    while (Wall::now() < deadline) {
      const auto box = box_.fresh();
      const bool contact = supported();
      const bool gap_ok = std::abs(box_.bottom(box) - surface) < 0.001;
      const bool tilt_ok = box_.tilt(box) < 0.00873;
      const bool good = contact && gap_ok && tilt_ok;
      ++samples;no_contact += !contact;bad_gap += !gap_ok;bad_tilt += !tilt_ok;
      if (!good && since >= 0) {
        ++resets;
        RCLCPP_WARN(node_->get_logger(),
          "SUPPORT RESET: contact=%d gap=%.3f mm tilt=%.3f deg %s", contact,
          (box_.bottom(box) - surface) * 1000, box_.tilt(box) * 180 / M_PI,
          supportDiagnosis().c_str());
      }
      last_gap = box_.bottom(box) - surface;last_tilt = box_.tilt(box);
      const auto now = node_->now().nanoseconds();
      // The held box can rebound and hang a few micrometres above the table after the
      // press; it cannot drop by itself. If it hovers without contact for 1 sim s, make a
      // bounded small re-press (gap + 0.05 mm, at most 0.1 mm, at most twice; gripper closed).
      const bool hovering = !contact && tilt_ok && last_gap > -0.00005 && last_gap < 0.00015;
      hover_since = hovering ? (hover_since < 0 ? now : hover_since) : -1;
      if (reseat && hovering && reseats < 2 && now - hover_since >= 1000000000) {
        const double step_down = std::min(0.0001, last_gap + 0.00005);
        ++reseats;
        RCLCPP_WARN(node_->get_logger(),
          "SUPPORT RESEAT %d/2: no contact for 1 sim s at gap=%.3f mm, descending %.3f mm",
          reseats, last_gap * 1000, step_down * 1000);
        auto goal = arm().getCurrentPose(tcp()).pose;goal.position.z -= step_down;
        motion_.linear(goal, 0.01);motion_.waitAt(&goal, 0.00015, 0.0015);
        since = -1;hover_since = -1;deadline = Wall::now() + 15s;
        continue;
      }
      if (!good) {since = -1;} else if (since < 0) {
        since = now;
      } else if (now - since >= 1000000000) {
        RCLCPP_INFO(node_->get_logger(),
            "SUPPORT VERIFIED: 1 sim s table contact, bottom_gap=%.3f mm (samples=%d resets=%d)",
          (box_.bottom(box) - surface) * 1000, samples, resets);return;
      }
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_ERROR(node_->get_logger(),
      "SUPPORT DIAG: samples=%d resets=%d no_contact=%d bad_gap=%d bad_tilt=%d "
      "last gap=%.3f mm tilt=%.3f deg %s", samples, resets, no_contact, bad_gap, bad_tilt,
      last_gap * 1000, last_tilt * 180 / M_PI, supportDiagnosis().c_str());
    throw std::runtime_error("Table contact/level support not verified; refusing gripper open");
  }

  Group::Plan planTableApproach(
    const Pose & preplace, const Pose & descent, bool alternate,
    const Eigen::Vector3d & relative, const Eigen::Quaterniond & relative_angle)
  {
    planning_scene::PlanningScene scene(arm().getRobotModel());
    require(scene.setPlanningSceneMsg(motion_.sceneSnapshot()), "Table probe scene import failed");
    const auto * group = scene.getRobotModel()->getJointModelGroup(names_.group);
    const auto valid = motion_.collisionCheck(scene);
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = point(preplace);
    target.linear() = rotation(preplace).toRotationMatrix();
    // Planning only: if every candidate fails, search once more with fresh random
    // IK seeds before giving up. The arm has not moved, so this is motion-free.
    for (int pass = 0; pass < 2; ++pass) {
      if (pass > 0) {
        RCLCPP_WARN(node_->get_logger(),
          "TABLE APPROACH SEARCH RETRY: no candidate in pass 1, searching again (planning only)");
      }
      for (int attempt = 0; attempt < 12; ++attempt) {
        auto state = scene.getCurrentState();
        if (attempt > 0 || pass > 0) {
          state.setToRandomPositions(group);
        }
        if (!state.setFromIK(group, target, tcp(), 0.08, valid)) {
          continue;
        }
        if (alternate && std::none_of(group->getVariableNames().begin(),
            group->getVariableNames().end(), [&state, &scene](const std::string & joint) {
            return std::abs(state.getVariablePosition(joint) -
                scene.getCurrentState().getVariablePosition(joint)) >= 0.1;
            }))
        {
          continue;
        }
        std::vector<moveit::core::RobotStatePtr> path;
        if (!motion_.checkedFinePath(scene, state, descent, path)) {
          continue;
        }
        std::vector<double> joints;
        state.copyJointGroupPositions(group, joints);
        if (!arm().setJointValueTarget(joints)) {
          continue;
        }
        // Replan the same (nearby) posture before moving to a random IK seed:
        // a random posture can mean a long reconfiguration that rotates the held box.
        Group::Plan approach;bool planned = false;
        for (int replan = 0; replan < 5 && !planned; ++replan) {
          motion_.setClampedStartState();
          planned = arm().plan(approach) == moveit::core::MoveItErrorCode::SUCCESS &&
            carryClearance(approach, relative, relative_angle, "TABLE APPROACH", 0.20);
          if (!planned) {
            RCLCPP_WARN(node_->get_logger(),
              "TABLE APPROACH REPLAN %d/5: candidate=%d plan rejected", replan + 1, attempt + 1);
          }
        }
        if (planned) {
          RCLCPP_INFO(node_->get_logger(),
            "TABLE DESCENT PROBE PASSED: pass=%d candidate=%d points=%zu",
            pass + 1, attempt + 1, path.size());
          return approach;
        }
      }
    }
    throw std::runtime_error("No table approach with a complete collision-checked descent");
  }
  bool tryTableDescent(
    const Eigen::Vector3d & reference, const Eigen::Quaterniond & reference_angle, bool alternate,
    Pose & goal)
  {
    const fc::Cell handoff = world_->handoffCell();
    const double rest_z = handoff.center.z() + 0.002;  // 2 mm above the resting height
    const auto box = box_.stable();const auto tcp_pose = arm().getCurrentPose(tcp()).pose;
    syncHeldBox(box, tcp_pose);
    const Eigen::Vector3d relative = rotation(tcp_pose).inverse() * (point(box) - point(tcp_pose));
    const Eigen::Quaterniond relative_angle = rotation(tcp_pose).inverse() * rotation(box);
    // A cube can rest on any face. Rotate 90 degrees about world X so the
    // fingers approach from above and the wrist stays clear of the tabletop.
    const double half = std::sqrt(0.5);
    const Eigen::Quaterniond desired_box(half, half, 0, 0);
    const Eigen::Quaterniond aligned = desired_box * relative_angle.inverse();
    goal = tcp_pose;
    goal.orientation.w = aligned.w();goal.orientation.x = aligned.x();
    goal.orientation.y = aligned.y();goal.orientation.z = aligned.z();
    const Eigen::Vector3d pre_center(handoff.center.x(), handoff.center.y(), handoff.surface + 0.10);
    const Eigen::Vector3d pre_tcp = pre_center - aligned * relative;
    goal.position.x = pre_tcp.x();goal.position.y = pre_tcp.y();goal.position.z = pre_tcp.z();
    RCLCPP_INFO(node_->get_logger(), "PLACE TOP-DOWN ALIGN AND APPROACH: box center z=%.3f",
      pre_center.z());
    arm().setGoalPositionTolerance(0.00015);arm().setGoalOrientationTolerance(0.0005);
    arm().setMaxVelocityScalingFactor(fc::speed(0.05));arm().setMaxAccelerationScalingFactor(fc::speed(0.05));
    Pose descent = goal;
    descent.position.z += rest_z - pre_center.z();
    arm().setGoalJointTolerance(0.00005);
    const auto approach = planTableApproach(goal, descent, alternate, relative, relative_angle);
    executeMonitored(approach.trajectory, "TABLE APPROACH", reference, reference_angle);
    motion_.waitAt(&goal, 0.0003, 0.0015);
    // Test mode: seating needs box-table contacts, which Gazebo hides while the box is welded.
    if (left_weld_) {left_weld_->release();}
    {
      // Let the held box stop swinging before judging its tilt (limit unchanged).
      const double settled_tilt = box_.tilt(box_.stable());
      RCLCPP_INFO(node_->get_logger(), "PREPLACE TILT: %.3f deg (limit 0.500)",
        settled_tilt * 180 / M_PI);
      char message[96];
      std::snprintf(message, sizeof(message),
        "Box not parallel enough for descent (tilt=%.3f deg)", settled_tilt * 180 / M_PI);
      require(settled_tilt < 0.00873, message);
    }
    double drift = 0, angle = 0;
    holdTransferred("PREPLACE", reference, reference_angle, drift, angle, 1.0);
    auto current_box = box_.fresh();auto current_tcp = arm().getCurrentPose(tcp()).pose;
    syncHeldBox(current_box, current_tcp);
    goal.position.x = current_tcp.position.x + handoff.center.x() - current_box.position.x;
    goal.position.y = current_tcp.position.y + handoff.center.y() - current_box.position.y;
    goal.position.z = current_tcp.position.z + rest_z - current_box.position.z;
    contacts_.setSeating(true);  // tabletop contact allowed now
    RCLCPP_INFO(node_->get_logger(), "PLACE DESCENT: box center z=%.3f, then contact steps",
      rest_z);
    try {
      motion_.linear(goal, 0.02, fc::LinearMode::precise);
    } catch (const fc::FinePathRejected & error) {
      RCLCPP_WARN(node_->get_logger(), "TABLE DESCENT PLAN REJECTED: %s", error.what());
      return false;
    }
    motion_.waitAt(&goal, 0.0003, 0.0015);
    return true;
  }
  void place(const Eigen::Vector3d & reference, const Eigen::Quaterniond & reference_angle)
  {
    const fc::Cell handoff = world_->handoffCell();
    const double surface = handoff.surface;
    Pose goal;
    if (!tryTableDescent(reference, reference_angle, false, goal)) {
      require(!supported(), "Unexpected table support before descent retry");
      RCLCPP_WARN(node_->get_logger(),
          "TABLE DESCENT RETRY: planning another preplace joint posture");
      require(tryTableDescent(reference, reference_angle, true, goal),
        "Table descent rejected again; gripper stays closed");
      RCLCPP_INFO(node_->get_logger(), "TABLE DESCENT RETRY PASSED");
    }
    const auto aligned = rotation(goal);
    // Remain closed on the 40 mm cube, but remove most of the commanded
    // overclosure before the last 2 mm of seating. Both fingers stay below
    // 20 mm in the command; this does not request a clearance around the cube.
    const auto before_relax = box_.fresh();
    gripper_.command(0.0198, true);
    auto current_box = box_.stable();auto current_tcp = arm().getCurrentPose(tcp()).pose;
    RCLCPP_INFO(node_->get_logger(),
      "SEAT GRIP RELAX: shift=%.3f mm bottom_gap=%.3f mm box=(%.5f, %.5f, %.5f)",
      (point(current_box) - point(before_relax)).norm() * 1000,
      (box_.bottom(current_box) - surface) * 1000, current_box.position.x,
      current_box.position.y, current_box.position.z);
    require((point(current_box) - point(before_relax)).norm() < 0.003 &&
      box_.bottom(current_box) - surface > -0.0003,
      "Box moved excessively while reducing overclosure");
    syncHeldBox(current_box, current_tcp);
    // Relaxing the grip lets the box slide up to ~1.8 mm on the finger pads (the change of the
    // grip command). The box is still held 2 mm above the table: move the hand horizontally so
    // the box is over the target again before seating.
    for (int attempt = 1; attempt <= 2; ++attempt) {
      const double ex = handoff.center.x() - current_box.position.x;
      const double ey = handoff.center.y() - current_box.position.y;
      if (std::hypot(ex, ey) < 0.0003) {break;}
      RCLCPP_INFO(node_->get_logger(), "SEAT XY CORRECTION %d/2: error=%.3f mm (x=%.3f y=%.3f)",
        attempt, std::hypot(ex, ey) * 1000, ex * 1000, ey * 1000);
      goal = arm().getCurrentPose(tcp()).pose;
      goal.orientation.w = aligned.w();goal.orientation.x = aligned.x();
      goal.orientation.y = aligned.y();goal.orientation.z = aligned.z();
      goal.position.x += ex;goal.position.y += ey;
      motion_.linear(goal, 0.01);motion_.waitAt(&goal, 0.00015, 0.0015);
      current_box = box_.stable();current_tcp = arm().getCurrentPose(tcp()).pose;
      syncHeldBox(current_box, current_tcp);
    }
    allowFingers(true);  // Only box-table support; robot-table collisions remain checked.
    bool seated = false;int nudges = 0;
    for (int step = 0; step < 10; ++step) {
      current_box = box_.fresh();
      if (box_.tilt(current_box) >= 0.00873) {
        // A single sample right after a contact step can catch the box mid-swing.
        // Let it settle, then judge (limit unchanged).
        RCLCPP_WARN(node_->get_logger(), "SEAT TILT TRANSIENT: %.3f deg, waiting for box to settle",
          box_.tilt(current_box) * 180 / M_PI);
        current_box = box_.stable();
        char message[96];
        std::snprintf(message, sizeof(message), "Box tilted during seating (tilt=%.3f deg)",
          box_.tilt(current_box) * 180 / M_PI);
        require(box_.tilt(current_box) < 0.00873, message);
      }
      const double gap = box_.bottom(current_box) - surface;
      const bool contact = supported();
      RCLCPP_INFO(node_->get_logger(), "SEAT step=%d bottom_gap=%.3f mm contact=%d",
        step, gap * 1000, contact);
      require(gap > -0.0003, "Box penetration too large; refusing further descent");
      // Seated only when in contact AND lightly pressed (passing runs: -0.002..-0.005 mm).
      // A corner touch can report contact while the box still hangs slightly above the
      // table (it then pivots in the fingers and contact disappears), so contact alone at a
      // positive gap is not accepted.
      if (contact && gap <= -0.000001) {seated = true;break;}
      double step_down = std::min(0.0005, gap + 0.0001);
      if (gap <= 0.00015) {
        // The held box cannot fall onto the table by itself. Make a bounded final small
        // descent (gap + 0.1 mm, at most 0.2 mm, at most 3 times) instead of only waiting.
        if (nudges >= 3 || gap < -0.00005) {waitSupport(true);seated = true;break;}
        ++nudges;step_down = std::min(0.0002, gap + 0.0001);
        RCLCPP_INFO(node_->get_logger(),
          "SEAT FINAL NUDGE %d/3: bottom_gap=%.3f mm contact=%d, descending %.3f mm",
          nudges, gap * 1000, contact, step_down * 1000);
      }
      goal = arm().getCurrentPose(tcp()).pose;
      goal.orientation.w = aligned.w();goal.orientation.x = aligned.x();
      goal.orientation.y = aligned.y();goal.orientation.z = aligned.z();
      goal.position.z -= step_down;
      motion_.linear(goal, 0.01);motion_.waitAt(&goal, 0.00015, 0.0015);
    }
    require(seated, "Could not establish table support within bounded seating steps");
    waitSupport(true);  // pre-open only: re-press allowed while the gripper is closed
    if (left_weld_) {left_weld_->release();}  // no-op: already released before seating
    const auto release_reference = box_.fresh();
    RCLCPP_INFO(node_->get_logger(), "OPEN ON TABLE: physical support verified, two-step opening");
    gripper_.command(0.021, false);  // Remove squeeze gently before the full opening.
    waitSupport();
    const auto soft_box = box_.fresh();
    double release_slip = (point(soft_box) - point(release_reference)).norm();
    RCLCPP_INFO(node_->get_logger(), "SOFT OPEN: delta=%.3f mm box=(%.5f, %.5f, %.5f)",
      release_slip * 1000, soft_box.position.x, soft_box.position.y, soft_box.position.z);
    gripper_.command(0.04, false);
    motion_.detachBox();
    contacts_.setCarrying(false);
    const auto open_box = box_.stable();
    release_slip = std::max(release_slip, (point(open_box) - point(release_reference)).norm());
    RCLCPP_INFO(node_->get_logger(), "BEFORE RETREAT: release_slip=%.3f mm box=(%.5f, %.5f, %.5f)",
      release_slip * 1000, open_box.position.x, open_box.position.y, open_box.position.z);
    addBox(open_box);
    goal = arm().getCurrentPose(tcp()).pose;goal.position.y -= 0.12;
    RCLCPP_INFO(node_->get_logger(), "PLACE RETREAT: 120 mm horizontally, no ready return");
    motion_.linear(goal, 0.10);motion_.waitAt(&goal);  // gripper open, box released
    const auto start = node_->now();const auto wall = Wall::now();
    double max_slip = release_slip;
    const auto xy_error = [&handoff](const Pose & box) {
        return std::hypot(box.position.x - handoff.center.x(), box.position.y - handoff.center.y());
      };
    while ((node_->now() - start).seconds() < 3.0) {
      require(Wall::now() - wall < 30s, "Post-release simulation clock timed out");
      current_box = box_.fresh();
      max_slip = std::max(max_slip, (point(current_box) - point(release_reference)).norm());
      RCLCPP_INFO(node_->get_logger(),
        "POST RELEASE: box=(%.5f, %.5f, %.5f) xy=%.3f mm slip=%.3f mm tilt=%.3f deg",
        current_box.position.x, current_box.position.y, current_box.position.z,
        xy_error(current_box) * 1000, max_slip * 1000, box_.tilt(current_box) * 180 / M_PI);
      require(supported(), "Box lost table support after release");
      require(max_slip < 0.002 && xy_error(current_box) < 0.003 &&
        std::abs(current_box.position.z - handoff.center.z()) < 0.001 &&
        box_.tilt(current_box) < 0.01745,
        "Released box moved or tilted outside placement tolerance");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(),
      "PLACE TEST PASSED: box=(%.5f, %.5f, %.5f) xy_error=%.3f mm release_slip=%.3f mm tilt=%.3f deg",
      current_box.position.x, current_box.position.y, current_box.position.z,
      xy_error(current_box) * 1000, max_slip * 1000, box_.tilt(current_box) * 180 / M_PI);
  }

  struct PickChoice
  {
    Pose pick, pre;
    std::vector<double> pre_joints;
    double pitch_deg = 0, retreat = 0.12;
  };
  // Grasp candidates for the source shelf cell, rehearsed with the planning margin (0.07 rad):
  // IK at the pre-grasp point `retreat` in front of the box, fine approach to the grasp, then
  // (box attached) lift 8 cm and retreat. Preference: longer retreat, then smaller pitch.
  PickChoice choosePickGrasp(const Pose & box)
  {
    constexpr double kPlanMargin = 0.07, kExecMargin = 0.05;
    // Owned by a shared_ptr: probeExit() derives diff scenes from it.
    const auto scene_ptr = std::make_shared<planning_scene::PlanningScene>(arm().getRobotModel());
    auto & scene = *scene_ptr;
    require(scene.setPlanningSceneMsg(motion_.sceneSnapshot()), "Pick probe scene import failed");
    const auto * group = scene.getRobotModel()->getJointModelGroup(names_.group);
    const auto * tcp_link = scene.getRobotModel()->getLinkModel(tcp());
    const Eigen::Vector3d in = world_->sourceShelf().inward() * Eigen::Vector3d::UnitY();
    const Eigen::Vector3d down = -Eigen::Vector3d::UnitZ();
    const auto valid = motion_.collisionCheck(scene, kPlanMargin);
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> perturb(-0.3, 0.3);
    for (const double retreat : {0.12, 0.08}) {
      for (const double pitch : {0.0, 20.0, 35.0}) {
        const double a = pitch * M_PI / 180.0;
        const Eigen::Quaterniond q = fc::handFrame(
          std::cos(a) * in + std::sin(a) * down, Eigen::Vector3d::UnitX());
        PickChoice c;c.pitch_deg = pitch;c.retreat = retreat;
        c.pick = box;
        c.pick.orientation.w = q.w();c.pick.orientation.x = q.x();
        c.pick.orientation.y = q.y();c.pick.orientation.z = q.z();
        c.pre = c.pick;
        c.pre.position.x -= retreat * in.x();c.pre.position.y -= retreat * in.y();
        const char * stage = "pre-grasp IK";
        Eigen::Isometry3d pre = Eigen::Isometry3d::Identity();
        pre.linear() = q.toRotationMatrix();pre.translation() = point(c.pre);
        for (int attempt = 0; attempt < 12; ++attempt) {
          auto state = scene.getCurrentState();
          // Seeds: the current posture, then postures near it (+-0.3 rad), random ones last;
          // a far random posture means a long swing to the pre-grasp point.
          if (attempt > 0 && attempt <= 6) {
            std::vector<double> near;
            scene.getCurrentState().copyJointGroupPositions(group, near);
            for (auto & value : near) {value += perturb(rng);}
            state.setJointGroupPositions(group, near);
            state.enforceBounds(group);
          } else if (attempt > 6) {
            state.setToRandomPositions(group);
          }
          if (!state.setFromIK(group, pre, tcp(), 0.08, valid)) {continue;}
          stage = "approach";
          std::vector<moveit::core::RobotStatePtr> approach;
          if (!motion_.checkedFinePath(scene, state, c.pick, approach, kPlanMargin)) {continue;}
          stage = "lift/retreat";
          if (!probeExit(scene, *approach.back(), box, in, retreat, kExecMargin, group, tcp_link)) {
            continue;
          }
          state.copyJointGroupPositions(group, c.pre_joints);
          RCLCPP_INFO(node_->get_logger(),
            "PICK GRASP SELECTED: pitch=%.0f deg retreat=%.0f mm (candidate %d)", pitch,
            retreat * 1000, attempt + 1);
          return c;
        }
        RCLCPP_INFO(node_->get_logger(), "PICK GRASP REJECTED: pitch=%.0f deg retreat=%.0f mm, %s",
          pitch, retreat * 1000, stage);
      }
    }
    throw std::runtime_error("No pick grasp with a collision-checked approach, lift and retreat");
  }
  // With the box attached at the grasp, lift 8 cm and retreat `retreat` away from the shelf.
  bool probeExit(const planning_scene::PlanningScene & scene, moveit::core::RobotState state,
    const Pose & box, const Eigen::Vector3d & in, double retreat, double margin,
    const moveit::core::JointModelGroup * group, const moveit::core::LinkModel * tcp_link)
  {
    auto held = scene.diff();
    moveit_msgs::msg::CollisionObject remove;remove.id = "target_box";
    remove.header.frame_id = "world";remove.operation = remove.REMOVE;
    held->processCollisionObjectMsg(remove);
    state.update();
    const Eigen::Isometry3d grasp = state.getGlobalLinkTransform(tcp_link);
    Eigen::Isometry3d box_world = Eigen::Isometry3d::Identity();
    box_world.translation() = point(box);box_world.linear() = rotation(box).toRotationMatrix();
    const Eigen::Isometry3d relative = grasp.inverse() * box_world;
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = tcp();
    attached.touch_links = {names_.left_finger, names_.right_finger, names_.hand};
    attached.object.id = "probe_box";attached.object.header.frame_id = tcp();
    attached.object.operation = attached.object.ADD;
    shape_msgs::msg::SolidPrimitive primitive;primitive.type = primitive.BOX;
    primitive.dimensions = {task_.box_size, task_.box_size, task_.box_size};
    Pose p;p.position.x = relative.translation().x();p.position.y = relative.translation().y();
    p.position.z = relative.translation().z();
    const Eigen::Quaterniond r(relative.rotation());
    p.orientation.w = r.w();p.orientation.x = r.x();p.orientation.y = r.y();p.orientation.z = r.z();
    attached.object.primitives.push_back(primitive);attached.object.primitive_poses.push_back(p);
    held->processAttachedCollisionObjectMsg(attached);
    held->setCurrentState(state);
    auto start = held->getCurrentState();
    Eigen::Isometry3d up = grasp;up.translation().z() += 0.08;
    Eigen::Isometry3d out = up;out.translation() -= retreat * in;
    const EigenSTL::vector_Isometry3d waypoints = {up, out};
    std::vector<moveit::core::RobotStatePtr> path;
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      &start, group, path, tcp_link, waypoints, true, moveit::core::MaxEEFStep(0.002),
      moveit::core::CartesianPrecision{},
      // Full-state check: includes the attached box (its TCP link is not in the arm group).
      [&held, margin](moveit::core::RobotState * s, const moveit::core::JointModelGroup * joints,
      const double * values) {
        s->setJointGroupPositions(joints, values);s->update();
        return s->satisfiesBounds(joints) &&
               fc::ArmMotion::jointLimitMargin(*s, joints) >= margin && !held->isStateColliding(*s);
      },
      kinematics::KinematicsQueryOptions());
    return fraction >= 0.999 && !path.empty();
  }

  void holdTransferred(
    const char * label, const Eigen::Vector3d & reference,
    const Eigen::Quaterniond & reference_angle, double & max_drift, double & max_angle,
    double seconds = 3.0)
  {
    const auto sim_start = node_->now();const auto wall_start = Wall::now();
    while ((node_->now() - sim_start).seconds() < seconds) {
      require(Wall::now() - wall_start < 30s, "Transfer hold simulation clock timed out");
      carrySafe();
      const auto box = box_.fresh();const auto tcp_pose = arm().getCurrentPose(tcp()).pose;
      max_drift = std::max(max_drift, (rotation(tcp_pose).inverse() *
          (point(box) - point(tcp_pose)) - reference).norm());
      max_angle = std::max(max_angle, reference_angle.angularDistance(
        rotation(tcp_pose).inverse() * rotation(box)));
      RCLCPP_DEBUG(node_->get_logger(), "%s: drift=%.3f mm angle=%.3f deg", label,
        max_drift * 1000, max_angle * 180 / M_PI);
      require(max_drift < 0.005 && max_angle < 0.0873,
        std::string(label) + ": physical box slipped or rotated in gripper");
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node_->get_logger(), "%s HOLD PASSED: drift=%.3f mm angle=%.3f deg", label,
      max_drift * 1000, max_angle * 180 / M_PI);
  }

  // MoveIt only guarantees zero-distance clearance for the held box. Before executing a
  // carry plan, require (a) >= 30 mm above the tabletop while over the margin-expanded
  // table footprint and (b) >= 15 mm from every shelf post and board (box half-diagonal,
  // box half-height along world z from its current orientation).
  // min_post_gap: the table approach starts above the table and has no reason to come back
  // near a shelf (a long swing that rotates the held box); it requires 0.20 m.
  bool carryClearance(
    const Group::Plan & plan, const Eigen::Vector3d & reference,
    const Eigen::Quaterniond & reference_angle, const char * label, double min_post_gap = 0.015)
  {
    constexpr double kTableMargin = 0.03, kShelfGap = 0.015;
    const double diagonal = world_->boxHalfDiagonal();
    const double table_floor = world_->table().top() + kTableMargin;
    moveit::core::RobotState state(arm().getRobotModel());
    state.setToDefaultValues();
    const auto & trajectory = plan.trajectory.joint_trajectory;
    double min_bottom = std::numeric_limits<double>::infinity();
    double min_post = std::numeric_limits<double>::infinity();
    double min_board = std::numeric_limits<double>::infinity();
    for (const auto & waypoint : trajectory.points) {
      state.setVariablePositions(trajectory.joint_names, waypoint.positions);
      state.update();
      const auto & tcp_pose = state.getGlobalLinkTransform(tcp());
      const Eigen::Vector3d center = tcp_pose * reference;
      const Eigen::Matrix3d box = tcp_pose.linear() * reference_angle.toRotationMatrix();
      const double half_height = box_.half() * box.row(2).cwiseAbs().sum();
      if (world_->overTable(center, kTableMargin)) {
        min_bottom = std::min(min_bottom, center.z() - half_height);
      }
      for (const auto & shelf : world_->shelves()) {
        const auto & footprint = shelf.boards.front();
        if (std::abs(center.x() - footprint.center.x()) <=
          0.5 * footprint.size.x() + diagonal + kShelfGap &&
          std::abs(center.y() - footprint.center.y()) <=
          0.5 * footprint.size.y() + diagonal + kShelfGap)
        {
          for (const auto & board : shelf.boards) {
            const double below = board.bottom() - (center.z() + half_height);
            const double above = (center.z() - half_height) - board.top();
            min_board = std::min(min_board, std::max(below, above));
          }
        }
        for (const auto & post : shelf.posts) {
          const double dx = std::max(std::abs(center.x() - post.center.x()) - 0.5 * post.size.x(), 0.0);
          const double dy = std::max(std::abs(center.y() - post.center.y()) - 0.5 * post.size.y(), 0.0);
          min_post = std::min(min_post, std::hypot(dx, dy) - diagonal);
        }
      }
    }
    const bool ok = min_bottom >= table_floor && min_post >= std::max(kShelfGap, min_post_gap) &&
      min_board >= kShelfGap;
    RCLCPP_INFO(node_->get_logger(),
      "%s CLEARANCE %s: min box bottom over table=%.4f m (limit %.3f) "
      "min shelf post gap=%.1f mm (limit %.0f), board gap=%.1f mm (limit 15)", label,
      ok ? "OK" : "REJECTED", min_bottom, table_floor, min_post * 1000,
      std::max(kShelfGap, min_post_gap) * 1000, std::isfinite(min_board) ? min_board * 1000 : 9999.0);
    return ok;
  }
  void carrySafe()
  {
    require(!contacts_.carryContact(), "Unexpected box contact while carrying; motion refused");
    require(!contacts_.unexpected(), "Box contacted a robot link outside the fingers");
  }
  // Execute while sampling the held box every 100 ms (mirrors the right arm's
  // SHELF TRANSFER MONITOR). Stops the arm on unexpected contact, lost finger
  // contact, the box dropping near the tabletop, or a gross in-hand slip.
  void executeMonitored(
    const moveit_msgs::msg::RobotTrajectory & trajectory, const char * label,
    const Eigen::Vector3d & reference, const Eigen::Quaterniond & reference_angle)
  {
    const double table_floor = world_->table().top() + 0.01;
    carrySafe();
    auto execution = std::async(std::launch::async,
        [this, &trajectory]() {return arm().execute(trajectory);});
    double min_bottom = std::numeric_limits<double>::infinity();
    double max_drift = 0.0, max_angle = 0.0;int gross = 0;
    try {
      while (execution.wait_for(100ms) == std::future_status::timeout) {
        carrySafe();
        const auto box = box_.fresh();const auto tcp_pose = arm().getCurrentPose(tcp()).pose;
        const auto inverse = rotation(tcp_pose).inverse();
        const double drift = (inverse * (point(box) - point(tcp_pose)) - reference).norm();
        const double angle = reference_angle.angularDistance(inverse * rotation(box));
        max_drift = std::max(max_drift, drift);max_angle = std::max(max_angle, angle);
        if (world_->overTable(point(box), 0.03)) {
          min_bottom = std::min(min_bottom, box_.bottom(box));
          require(box_.bottom(box) > table_floor,
            "Held box came within 10 mm of the tabletop in motion");
        }
        gross = (drift > 0.015 || angle > 0.1745) ? gross + 1 : 0;
        require(gross < 2, std::string(label) + ": box slipped in gripper during motion");
        if (!welded() && contacts_.anyBoxContactSeen()) {
          require(contacts_.fingersFresh(1s), "Finger contact lost during carry");
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_->get_logger(), "%s STOP REQUESTED: %s", label, error.what());
      arm().stop();execution.wait();throw;
    }
    require(execution.get() == moveit::core::MoveItErrorCode::SUCCESS,
      std::string(label) + " execution failed");
    carrySafe();
    RCLCPP_INFO(node_->get_logger(),
      "%s MONITOR PASSED: min_box_bottom_over_table=%.4f m max_sampled_drift=%.3f mm "
      "max_sampled_angle=%.3f deg", label, min_bottom, max_drift * 1000, max_angle * 180 / M_PI);
  }

  rclcpp::Node::SharedPtr node_;
  fc::ArmNames names_;
  fc::TaskConfig task_;
  fc::BoxTracker box_;
  fc::Gripper gripper_;
  fc::ContactMonitor contacts_;
  fc::ArmMotion motion_;
  std::unique_ptr<fc::WorldModel> world_;
  std::unique_ptr<fc::BoxWeld> left_weld_, right_weld_;  // rigid_grasp test mode only
};
}  // namespace

namespace fr3_cooperative_control
{
class LeftTransfer::Impl : public Pick
{
public:
  using Pick::Pick;
};

LeftTransfer::LeftTransfer(rclcpp::Node::SharedPtr node)
: impl_(std::make_unique<Impl>(std::move(node)))
{
}

LeftTransfer::~LeftTransfer() = default;

void LeftTransfer::run(const std::string & stage) {impl_->run(stage);}
}  // namespace fr3_cooperative_control
