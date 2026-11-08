"""Physical left-arm pick, transfer and contact-verified placement stages."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    bringup = get_package_share_directory('franka_gazebo_bringup')
    moveit = MoveItConfigsBuilder(
        'fr3_duo_gazebo', package_name='dual_fr3_moveit_config').to_moveit_configs()
    moveit.sensors_3d.clear()
    # Keep this test away from joint5's physical stops; share server/client bounds.
    limits = moveit.joint_limits['robot_description_planning']['joint_limits']
    limits['left_fr3v2_joint5'].update(
        has_position_limits=True, min_position=-2.60, max_position=2.60)
    parameters = [moveit.robot_description, moveit.robot_description_semantic,
                  moveit.robot_description_kinematics, moveit.joint_limits,
                  {'use_sim_time': True}]
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('stage', default_value='pick',
                              choices=['pick', 'transfer', 'place']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            bringup, 'launch', 'gazebo_fr3_duo_example.launch.py')),
            launch_arguments={
                'rviz': 'false',
                'world': PythonExpression(["'placement.sdf' if '",
                                           LaunchConfiguration('stage'),
                                           "' == 'place' else ''"]),
                'gz_args': PythonExpression([
                    "'-r -s' if '", LaunchConfiguration('headless'), "' == 'true' else '-r'"])
            }.items()),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[moveit.to_dict(), {
                 'use_sim_time': True,
                 'trajectory_execution.allowed_execution_duration_scaling': 15.0,
                 'trajectory_execution.allowed_goal_duration_margin': 30.0}]),
        Node(package='ros_gz_sim', executable='create', name='spawn_target_box',
             output='screen', arguments=[
                 '-world', 'empty', '-name', 'target_box', '-allow_renaming', 'false',
                 '-file', os.path.join(bringup, 'models', 'target_box', 'model.sdf'),
                 '-x', '0', '-y', '-1.04', '-z', '0.5335']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=[
                 '/model/target_box/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
                 '/world/empty/model/target_box/link/link/sensor/box_contacts/contact'
                 '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                 '/world/empty/model/fr3_duo_gazebo/link/table_link/sensor/table_contacts/contact'
                 '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],
             remappings=[('/model/target_box/pose', '/target_box/pose'),
                         ('/world/empty/model/target_box/link/link/sensor/box_contacts/contact',
                          '/placement/box_contacts'),
                         ('/world/empty/model/fr3_duo_gazebo/link/table_link/sensor/'
                          'table_contacts/contact', '/placement/table_contacts')]),
        Node(package='fr3_cooperative_control', executable='single_arm_pick_test',
             name='single_arm_pick_test', output='screen',
             parameters=parameters + [{'stage': LaunchConfiguration('stage')}]),
    ])
