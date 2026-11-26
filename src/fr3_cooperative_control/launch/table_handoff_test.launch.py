"""Table handoff: validated left placement followed by physical right pickup."""
import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from fr3_moveit_params import moveit_setup  # noqa: E402  (shared helper next to this file)

# Task geometry shared with the C++ nodes (see include/fr3_cooperative_control/world_model.hpp).
# Shelf, board and table positions come from the robot description, never from constants here.
BOX_SIZE = 0.04
GAZEBO_Z_OFFSET = 0.05  # Gazebo world z minus MoveIt world z
SPAWN_DROP = 0.001      # spawn the box 1 mm above its support
CELL_INSET = 0.03       # box centre behind the board's front edge
HANDOFF_XY = (0.0, 0.05)
LEVELS = ['1', '2', '3', '4']


def _xyz(element):
    text = element.get('xyz', '0 0 0') if element is not None else '0 0 0'
    return [float(v) for v in text.split()]


def _link_boxes(robot, link_name):
    """World-aligned (centre, size) of every box collision of a fixed environment link."""
    link = robot.find(f"./link[@name='{link_name}']")
    joint = robot.find(f"./joint/child[@link='{link_name}']/..")
    if link is None or joint is None:
        raise RuntimeError(f'{link_name} not found in robot description')
    base = _xyz(joint.find('origin'))
    boxes = []
    for collision in link.findall('collision'):
        box = collision.find('geometry/box')
        if box is not None:
            offset = _xyz(collision.find('origin'))
            size = [float(v) for v in box.get('size').split()]
            boxes.append(([b + o for b, o in zip(base, offset)], size))
    if not boxes:
        raise RuntimeError(f'No box collision on {link_name}')
    return boxes


def shelf_cell(robot, shelf, level):
    """Resting box centre in a shelf cell (MoveIt world frame); boards ordered bottom-up."""
    boards = sorted((b for b in _link_boxes(robot, f'{shelf}_link')
                     if b[1][2] < 0.05 and b[1][0] > 0.2 and b[1][1] > 0.2),
                    key=lambda b: b[0][2])
    if not 1 <= int(level) <= len(boards):
        raise RuntimeError(f'{shelf} has no board level {level}')
    center, size = boards[int(level) - 1]
    inward = 1.0 if center[1] >= 0 else -1.0
    front = center[1] - inward * size[1] / 2
    return (center[0], front + inward * CELL_INSET, center[2] + size[2] / 2 + BOX_SIZE / 2)


def handoff_cell(robot):
    """Resting box centre at the table hand-off point (MoveIt world frame)."""
    center, size = max(_link_boxes(robot, 'table_link'), key=lambda b: b[1][0] * b[1][1])
    return (HANDOFF_XY[0], HANDOFF_XY[1], center[2] + size[2] / 2 + BOX_SIZE / 2)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('right_stage', default_value='pick',
                              choices=['pick', 'shelf_hold', 'shelf_place', 'left_park']),
        DeclareLaunchArgument('table_only', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('shelf2_y', default_value='1.25',
                              choices=['1.15', '1.20', '1.25']),
        DeclareLaunchArgument('source_level', default_value='2', choices=LEVELS,
                              description='shelf_1 board the box starts on (1 = lowest)'),
        DeclareLaunchArgument('target_level', default_value='2', choices=LEVELS,
                              description='shelf_2 board the box is placed on (1 = lowest)'),
        DeclareLaunchArgument('single_process', default_value='false', choices=['true', 'false'],
                              description='Run both arm tasks in one transfer_task process'),
        DeclareLaunchArgument('speed_scale', default_value='1.0',
                              description='Test mode: multiply every motion speed (capped at 1)'),
        DeclareLaunchArgument('rigid_grasp', default_value='false', choices=['true', 'false'],
                              description='Test mode: weld the box to the hand while grasped'),
        OpaqueFunction(function=setup),
    ])


def setup(context):
    shelf_y = LaunchConfiguration('shelf2_y').perform(context)
    bringup = get_package_share_directory('franka_gazebo_bringup')
    moveit, robot, parameters = moveit_setup(shelf_y)
    table_only = LaunchConfiguration('table_only')
    right_stage = LaunchConfiguration('right_stage')
    left_placed = table_only.perform(context) == 'false'
    if right_stage.perform(context) == 'left_park' and not left_placed:
        raise RuntimeError('left_park requires the left placement stage')
    task = {
        'source_shelf': 'shelf_1',
        'source_level': int(LaunchConfiguration('source_level').perform(context)),
        'target_shelf': 'shelf_2',
        'target_level': int(LaunchConfiguration('target_level').perform(context)),
        'handoff_x': HANDOFF_XY[0], 'handoff_y': HANDOFF_XY[1],
        'cell_inset': CELL_INSET, 'box_size': BOX_SIZE, 'gazebo_z_offset': GAZEBO_Z_OFFSET,
        'speed_scale': float(LaunchConfiguration('speed_scale').perform(context)),
        'rigid_grasp': LaunchConfiguration('rigid_grasp').perform(context) == 'true',
    }
    single_process = LaunchConfiguration('single_process').perform(context) == 'true'
    start = (shelf_cell(robot, task['source_shelf'], task['source_level']) if left_placed
             else handoff_cell(robot))
    spawn = [f'{start[0]:.4f}', f'{start[1]:.4f}',
             f'{start[2] + GAZEBO_Z_OFFSET + SPAWN_DROP:.4f}']
    left_task = Node(package='fr3_cooperative_control', executable='single_arm_pick_test',
                     name='single_arm_pick_test', output='screen',
                     condition=UnlessCondition(table_only),
                     parameters=parameters + [task, {'stage': 'place'}])
    right_task = Node(package='fr3_cooperative_control', executable='right_arm_table_pick',
                      name='right_arm_table_pick', output='screen',
                      parameters=parameters + [task, {'stage': right_stage,
                                                      'left_placed': left_placed}])
    coordinator = Node(package='fr3_cooperative_control', executable='transfer_task',
                       name='transfer_task', output='screen',
                       parameters=parameters + [task, {'right_stage': right_stage,
                                                       'table_only': not left_placed}])

    def after_left(event, context):
        if event.returncode == 0:
            return [LogInfo(msg='LEFT PLACE COMPLETE: checking left-arm parking'),
                    right_task]
        return [LogInfo(msg='TABLE HANDOFF FAILED: left placement failed; right pickup skipped')]

    def start_table_only(context):
        if table_only.perform(context).lower() == 'true':
            return [right_task]
        return []

    def after_right(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='TABLE HANDOFF FAILED: right-arm stage failed')]
        if right_stage.perform(context) == 'left_park':
            return [LogInfo(msg='LEFT PARK SCENARIO PASSED')]
        if right_stage.perform(context) == 'shelf_place':
            marker = ('RIGHT SHELF PLACE SCENARIO PASSED' if
                      table_only.perform(context).lower() == 'true' else
                      'FULL TRANSFER TEST PASSED')
            return [LogInfo(msg=marker)]
        if right_stage.perform(context) == 'shelf_hold':
            marker = ('RIGHT SHELF SCENARIO PASSED' if
                      table_only.perform(context).lower() == 'true' else
                      'TABLE TO SHELF TEST PASSED')
            return [LogInfo(msg=marker)]
        marker = ('RIGHT TABLE PICK SCENARIO PASSED' if
                  table_only.perform(context).lower() == 'true' else 'TABLE HANDOFF TEST PASSED')
        return [LogInfo(msg=marker)]

    # Keep MoveGroup capabilities mapped through executor teardown on Jazzy.
    # https://github.com/moveit/moveit2/issues/3680
    move_group_library = os.path.join(get_package_prefix('moveit_ros_move_group'), 'lib',
                                      'libmoveit_move_group_default_capabilities.so')
    move_group_env = {
        'LD_PRELOAD': [move_group_library, ' ',
                       EnvironmentVariable('LD_PRELOAD', default_value='')]
    } if os.path.isfile(move_group_library) else {}

    weld_bridge = [
        f'/box_weld/{side}/{topic}@{ros}{direction}{gz}'
        for side in ('left', 'right')
        for topic, ros, direction, gz in (
            ('attach', 'std_msgs/msg/Empty', ']', 'gz.msgs.Empty'),
            ('detach', 'std_msgs/msg/Empty', ']', 'gz.msgs.Empty'),
            ('state', 'std_msgs/msg/String', '[', 'gz.msgs.StringMsg'))
    ] if task['rigid_grasp'] else []
    tasks = [coordinator] if single_process else [
        RegisterEventHandler(OnProcessExit(target_action=left_task, on_exit=after_left)),
        RegisterEventHandler(OnProcessExit(target_action=right_task, on_exit=after_right)),
        left_task,
        OpaqueFunction(function=start_table_only),
    ]

    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            bringup, 'launch', 'gazebo_fr3_duo_example.launch.py')),
            launch_arguments={
                'rviz': 'false',
                'world': 'placement.sdf',
                'shelf2_y': shelf_y,
                'box_weld': 'true' if task['rigid_grasp'] else 'false',
                'gz_args': PythonExpression([
                    "'-r -s' if '", LaunchConfiguration('headless'), "' == 'true' else '-r'"])
            }.items()),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             additional_env=move_group_env,
             parameters=[moveit.to_dict(), {
                 'use_sim_time': True,
                 'trajectory_execution.allowed_execution_duration_scaling': 15.0,
                 'trajectory_execution.allowed_goal_duration_margin': 30.0}]),
        Node(package='ros_gz_sim', executable='create', name='spawn_target_box',
             output='screen', arguments=[
                 '-world', 'empty', '-name', 'target_box', '-allow_renaming', 'false',
                 '-file', os.path.join(bringup, 'models', 'target_box', 'model.sdf'),
                 '-x', spawn[0], '-y', spawn[1], '-z', spawn[2]]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=[
                 '/model/target_box/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
                 '/world/empty/model/target_box/link/link/sensor/box_contacts/contact'
                 '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                 '/world/empty/model/fr3_duo_gazebo/link/table_link/sensor/table_contacts/contact'
                 '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                 '/world/empty/model/fr3_duo_gazebo/link/shelf_2_link/sensor/'
                 'shelf_contacts/contact'
                 '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'] + weld_bridge,
             remappings=[('/model/target_box/pose', '/target_box/pose'),
                         ('/world/empty/model/target_box/link/link/sensor/box_contacts/contact',
                          '/placement/box_contacts'),
                         ('/world/empty/model/fr3_duo_gazebo/link/table_link/sensor/'
                          'table_contacts/contact', '/placement/table_contacts'),
                         ('/world/empty/model/fr3_duo_gazebo/link/shelf_2_link/sensor/'
                          'shelf_contacts/contact', '/placement/shelf2_contacts')]),
    ] + tasks
