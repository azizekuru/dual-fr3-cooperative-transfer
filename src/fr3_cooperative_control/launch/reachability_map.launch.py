"""Offline reachability map of all shelf/table cells for both arms (no Gazebo, no move_group).

ros2 launch fr3_cooperative_control reachability_map.launch.py output:=/tmp/reachability.csv
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from fr3_moveit_params import moveit_setup  # noqa: E402  (shared helper next to this file)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('shelf2_y', default_value='1.25',
                              choices=['1.15', '1.20', '1.25']),
        DeclareLaunchArgument('output', default_value='reachability.csv',
                              description='CSV file with one row per arm, cell and grasp'),
        DeclareLaunchArgument('seeds', default_value='24',
                              description='IK seeds per arm, cell and grasp'),
        DeclareLaunchArgument('grasp_set', default_value='basic', choices=['basic', 'extended'],
                              description='basic: side/top grasps; extended: pitched and inner'),
        DeclareLaunchArgument('x_offsets', default_value='[-0.15, 0.0, 0.15]',
                              description='Box x offsets from the board centre (m)'),
        DeclareLaunchArgument('insets', default_value='[0.03, 0.10]',
                              description='Box centre depth behind the front edge (m)'),
        DeclareLaunchArgument('include_table', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('report_contacts', default_value='false', choices=['true', 'false'],
                              description='Debug: print the contacts blocking NO_IK cases'),
        DeclareLaunchArgument('check_collisions', default_value='true', choices=['true', 'false'],
                              description='Debug: false ignores collisions (pure reach)'),
        DeclareLaunchArgument('own_shelf_only', default_value='false', choices=['true', 'false'],
                              description='Each arm only evaluates the shelf on its own side'),
        DeclareLaunchArgument('plan_margin', default_value='0.07',
                              description='Joint-limit margin for the holding posture (rad)'),
        DeclareLaunchArgument('exec_margin', default_value='0.05',
                              description='Joint-limit margin along the exit path (rad)'),
        OpaqueFunction(function=setup),
    ])


def setup(context):
    _, _, parameters = moveit_setup(LaunchConfiguration('shelf2_y').perform(context))

    def floats(name):
        text = LaunchConfiguration(name).perform(context).strip().strip('[]')
        return [float(v) for v in text.split(',') if v.strip()]

    def flag(name):
        return LaunchConfiguration(name).perform(context) == 'true'

    return [Node(package='fr3_cooperative_control', executable='reachability_map',
                 name='reachability_map', output='screen',
                 parameters=parameters + [{
                     'use_sim_time': False,
                     'output': LaunchConfiguration('output').perform(context),
                     'seeds': int(LaunchConfiguration('seeds').perform(context)),
                     'plan_margin': float(LaunchConfiguration('plan_margin').perform(context)),
                     'exec_margin': float(LaunchConfiguration('exec_margin').perform(context)),
                     'grasp_set': LaunchConfiguration('grasp_set').perform(context),
                     'x_offsets': floats('x_offsets'), 'insets': floats('insets'),
                     'include_table': flag('include_table'),
                     'check_collisions': flag('check_collisions'),
                     'report_contacts': flag('report_contacts'),
                     'own_shelf_only': flag('own_shelf_only')}])]
