import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('franka_gazebo_bringup')
    
    fr3_duo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'gazebo_fr3_duo_example.launch.py')
        )
    )

    box_sdf = os.path.join(pkg_bringup, 'models', 'target_box', 'model.sdf')

    # Yalnızca hareketli transfer küpünü spawn ediyoruz
    spawn_box = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'target_box', '-file', box_sdf, '-x', '0.0', '-y', '-1.2', '-z', '0.485'],
        output='screen'
    )

    delayed_spawns = TimerAction(
        period=4.0,
        actions=[spawn_box]
    )

    return LaunchDescription([
        fr3_duo_launch,
        delayed_spawns
    ])
