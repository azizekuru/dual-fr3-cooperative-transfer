"""Tek komutla uzanma testi: Gazebo + MoveIt move_group + dual_arm_reach_test.

Kullanim:
    ros2 launch fr3_cooperative_control reach_test.launch.py
    ros2 launch fr3_cooperative_control reach_test.launch.py stop_after:=3
    ros2 launch fr3_cooperative_control reach_test.launch.py headless:=true speed:=0.8

Bu dosya mevcut launch dosyalarini degistirmez, onlari icerir:
  - franka_gazebo_bringup/gazebo_fr3_duo_example.launch.py  (robotlar, raflar, masa)
  - dual_fr3_moveit_config/move_group.launch.py             (MoveIt)
Kutu spawn EDILMEZ (Asama 1: sadece uzanma testi).
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'launch', 'gazebo_fr3_duo_example.launch.py')
    move_group_launch = os.path.join(
        get_package_share_directory('dual_fr3_moveit_config'),
        'launch', 'move_group.launch.py')

    args = [
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true: Gazebo GUI acilmaz (daha hizli). GUI icin ayri terminalde: gz sim -g'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='true: Gazebo launch dosyasinin RViz penceresini de ac (yavaslatir)'),
        DeclareLaunchArgument(
            'stop_after', default_value='99',
            description='Bu adimdan sonra dur (1=ready, 2=shelf_1, 3=masa, 4=arm_2 masa, 5=shelf_2)'),
        DeclareLaunchArgument('speed', default_value='0.7',
                              description='Serbest hareketlerin hizi (joint limits olcegi, 0-1)'),
        DeclareLaunchArgument('approach_speed', default_value='0.3',
                              description='Hedefe duz cizgi yaklasma hizi (0-1)'),
        DeclareLaunchArgument('settle_timeout', default_value='2.0',
                              description='Planlamadan once kolun durmasini bekleme suresi, sn (0=bekleme)'),
        DeclareLaunchArgument('via_ready', default_value='false',
                              description='true: shelf_1 sonrasi masaya gecmeden once ready pozuna ugra'),
        DeclareLaunchArgument('move_group_delay', default_value='8.0',
                              description='move_group Gazebo acildiktan kac sn sonra baslasin'),
        DeclareLaunchArgument('coordinator_delay', default_value='30.0',
                              description='Koordinator kac sn sonra baslasin (controller\'lar hazir olmali)'),
    ]

    gz_args = PythonExpression([
        "'-r -s' if '", LaunchConfiguration('headless'), "' == 'true' else '-r'"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'gz_args': gz_args,
        }.items(),
    )

    move_group = TimerAction(
        period=LaunchConfiguration('move_group_delay'),
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(move_group_launch))],
    )

    reach_test = TimerAction(
        period=LaunchConfiguration('coordinator_delay'),
        actions=[Node(
            package='fr3_cooperative_control',
            executable='dual_arm_reach_test',
            name='dual_arm_reach_test',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'stop_after': ParameterValue(LaunchConfiguration('stop_after'), value_type=int),
                'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
                'approach_speed': ParameterValue(LaunchConfiguration('approach_speed'), value_type=float),
                'settle_timeout': ParameterValue(LaunchConfiguration('settle_timeout'), value_type=float),
                'via_ready': ParameterValue(LaunchConfiguration('via_ready'), value_type=bool),
            }],
        )],
    )

    return LaunchDescription(args + [gazebo, move_group, reach_test])