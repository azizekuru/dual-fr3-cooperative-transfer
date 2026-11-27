"""Shelf 1 -> left arm -> table -> right arm -> shelf 2 row 3 column 1."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def setup(context):
    path = Path(get_package_share_directory('fr3_cooperative_control')) / 'launch/compartment_right_place.launch.py'
    spec = importlib.util.spec_from_file_location('compartment_transfer_scene', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.setup(context, handoff=True)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source_row', default_value='3', choices=['1', '2', '3', '4']),
        DeclareLaunchArgument('source_col', default_value='1', choices=['1', '2']),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup)])
