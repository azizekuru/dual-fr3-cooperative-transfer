"""MoveIt configuration shared by the FR3 transfer launch files (not a launch file itself)."""
import xml.etree.ElementTree as ET

from moveit_configs_utils import MoveItConfigsBuilder


def moveit_setup(shelf_y):
    """Return (moveit_config, robot_xml, node_parameters) for the given shelf_2 y position.

    The robot description is patched to the requested shelf_2 position, and joint5 of both arms
    is kept away from its physical stops (shared by move_group and the task nodes).
    """
    moveit = MoveItConfigsBuilder(
        'fr3_duo_gazebo', package_name='dual_fr3_moveit_config').to_moveit_configs()
    moveit.sensors_3d.clear()
    robot = ET.fromstring(moveit.robot_description['robot_description'])
    if shelf_y != '1.20':
        rack_origin = robot.find("./joint[@name='shelf_2_joint']/origin")
        rack_origin.set('xyz', f'0.0 {shelf_y} 0.0')
        moveit.robot_description['robot_description'] = ET.tostring(robot, encoding='unicode')
    limits = moveit.joint_limits['robot_description_planning']['joint_limits']
    for side in ('left', 'right'):
        limits[f'{side}_fr3v2_joint5'].update(
            has_position_limits=True, min_position=-2.60, max_position=2.60)
    parameters = [moveit.robot_description, moveit.robot_description_semantic,
                  moveit.robot_description_kinematics, moveit.joint_limits,
                  {'use_sim_time': True}]
    return moveit, robot, parameters
