#!/usr/bin/env python3
"""Check empty left gripper through MoveIt, including both physical finger states."""
import time

from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class Check(Node):
    def __init__(self):
        super().__init__('gripper_alignment_check')
        self.states = {}
        self.create_subscription(JointState, '/joint_states', self.on_state,
                                 qos_profile_sensor_data)
        self.client = ActionClient(self, MoveGroup, '/move_action')

    def on_state(self, msg):
        for name, value in zip(msg.name, msg.position):
            self.states[name] = (value, time.monotonic())

    def wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('Action response timed out; inspect simulation before retrying')
        return future.result()

    def command(self, position):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = 'gripper_1'
        request.num_planning_attempts = 3
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2
        request.start_state.is_diff = True
        constraint = Constraints()
        constraint.joint_constraints.append(JointConstraint(
            joint_name='left_fr3v2_finger_joint1', position=position,
            tolerance_above=0.0005, tolerance_below=0.0005, weight=1.0))
        request.goal_constraints.append(constraint)
        handle = self.wait(self.client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError('MoveIt rejected gripper goal')
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=45.0)
        if not future.done():
            self.wait(handle.cancel_goal_async(), 5.0)
            raise RuntimeError('Gripper action timed out; cancellation requested')
        result = future.result()
        if (result.status != GoalStatus.STATUS_SUCCEEDED or
                result.result.error_code.val != MoveItErrorCodes.SUCCESS):
            raise RuntimeError('MoveIt gripper execution failed: ' +
                               str(result.result.error_code.val))
        deadline = time.monotonic() + 10.0
        good_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            fingers = [self.states.get('left_fr3v2_finger_joint' + str(i)) for i in (1, 2)]
            valid = all(item is not None and now - item[1] < 1.0 and
                        abs(item[0] - position) < 0.001 for item in fingers)
            good_since = (good_since if good_since is not None else now) if valid else None
            if good_since is not None and now - good_since >= 0.5:
                self.get_logger().info(
                    f'GRIPPER CHECK target={position:.4f} '
                    f'fingers={fingers[0][0]:.4f}/{fingers[1][0]:.4f} m')
                return
        raise RuntimeError('Physical fingers did not both reach target within 1 mm')


def main():
    rclpy.init()
    node = Check()
    try:
        if not node.client.wait_for_server(timeout_sec=60.0):
            raise RuntimeError('MoveIt move_action not ready')
        for position in (0.04, 0.005, 0.04):
            node.command(position)
        node.get_logger().info('GRIPPER ALIGNMENT PASSED (empty gripper only)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
