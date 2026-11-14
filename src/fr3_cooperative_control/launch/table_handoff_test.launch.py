"""Table handoff: validated left placement followed by physical right pickup."""
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('right_stage', default_value='pick',
                              choices=['pick', 'shelf_hold', 'shelf_place', 'left_park']),
        DeclareLaunchArgument('table_only', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('shelf2_y', default_value='1.25',
                              choices=['1.15', '1.20', '1.25']),
        OpaqueFunction(function=setup),
    ])


def setup(context):
    shelf_y = LaunchConfiguration('shelf2_y').perform(context)
    bringup = get_package_share_directory('franka_gazebo_bringup')
    moveit = MoveItConfigsBuilder(
        'fr3_duo_gazebo', package_name='dual_fr3_moveit_config').to_moveit_configs()
    moveit.sensors_3d.clear()
    if shelf_y != '1.20':
        robot = ET.fromstring(moveit.robot_description['robot_description'])
        rack_origin = robot.find("./joint[@name='shelf_2_joint']/origin")
        rack_origin.set('xyz', f'0.0 {shelf_y} 0.0')
        moveit.robot_description['robot_description'] = ET.tostring(robot, encoding='unicode')
    # Keep this test away from joint5's physical stops; share server/client bounds.
    limits = moveit.joint_limits['robot_description_planning']['joint_limits']
    limits['left_fr3v2_joint5'].update(
        has_position_limits=True, min_position=-2.60, max_position=2.60)
    limits['right_fr3v2_joint5'].update(
        has_position_limits=True, min_position=-2.60, max_position=2.60)
    parameters = [moveit.robot_description, moveit.robot_description_semantic,
                  moveit.robot_description_kinematics, moveit.joint_limits,
                  {'use_sim_time': True}]
    table_only = LaunchConfiguration('table_only')
    right_stage = LaunchConfiguration('right_stage')
    left_placed = table_only.perform(context) == 'false'
    if right_stage.perform(context) == 'left_park' and not left_placed:
        raise RuntimeError('left_park requires the left placement stage')
    spawn_y = PythonExpression(["'0.05' if '", table_only, "' == 'true' else '-1.04'"])
    spawn_z = PythonExpression(["'0.471' if '", table_only, "' == 'true' else '0.5335'"])
    left_task = Node(package='fr3_cooperative_control', executable='single_arm_pick_test',
                     name='single_arm_pick_test', output='screen',
                     condition=UnlessCondition(table_only),
                     parameters=parameters + [{'stage': 'place'}])
    right_task = Node(package='fr3_cooperative_control', executable='right_arm_table_pick',
                      name='right_arm_table_pick', output='screen',
                      parameters=parameters + [{'stage': right_stage, 'left_placed': left_placed}])

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

    return [
        RegisterEventHandler(OnProcessExit(target_action=left_task, on_exit=after_left)),
        RegisterEventHandler(OnProcessExit(target_action=right_task, on_exit=after_right)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            bringup, 'launch', 'gazebo_fr3_duo_example.launch.py')),
            launch_arguments={
                'rviz': 'false',
                'world': 'placement.sdf',
                'shelf2_y': shelf_y,
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
                 '-x', '0', '-y', spawn_y, '-z', spawn_z]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=[
                 '/model/target_box/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
                 '/world/empty/model/target_box/link/link/sensor/box_contacts/contact'
                 '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                 '/world/empty/model/fr3_duo_gazebo/link/table_link/sensor/table_contacts/contact'
                 '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                 '/world/empty/model/fr3_duo_gazebo/link/shelf_2_link/sensor/'
                 'shelf_contacts/contact'
                 '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],
             remappings=[('/model/target_box/pose', '/target_box/pose'),
                         ('/world/empty/model/target_box/link/link/sensor/box_contacts/contact',
                          '/placement/box_contacts'),
                         ('/world/empty/model/fr3_duo_gazebo/link/table_link/sensor/'
                          'table_contacts/contact', '/placement/table_contacts'),
                         ('/world/empty/model/fr3_duo_gazebo/link/shelf_2_link/sensor/'
                          'shelf_contacts/contact', '/placement/shelf2_contacts')]),
        left_task,
        OpaqueFunction(function=start_table_only),
    ]
