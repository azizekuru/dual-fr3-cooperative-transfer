from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch_ros.actions import SetParameter
from launch import LaunchDescription

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fr3_duo_gazebo", package_name="dual_fr3_moveit_config").to_moveit_configs()
    
    move_group_launch = generate_move_group_launch(moveit_config)
    
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        SetParameter(name='trajectory_execution.allowed_execution_duration_scaling', value=15.0),
        SetParameter(name='trajectory_execution.allowed_goal_duration_margin', value=30.0),
        move_group_launch
    ])