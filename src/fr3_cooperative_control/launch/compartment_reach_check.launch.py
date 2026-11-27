"""Offline prototype screening. No Gazebo, controllers, or robot execution."""
import json
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def shelf_parts(cfg):
    width, depth = cfg['width'], cfg['depth']
    side, divider, board = (cfg['side_thickness'], cfg['divider_thickness'],
                            cfg['board_thickness'])
    bottom, roof = cfg['floor_bottoms'][0], cfg['roof_bottom']
    height = roof + board - bottom
    parts = [((width, depth, board), (0, 0, z + board / 2))
             for z in cfg['floor_bottoms'] + [roof]]
    if cfg.get('side_style', 'panels') == 'open_posts':
        parts.extend(((side, side, roof + board), (x, y, (roof + board) / 2))
                     for x in (-(width - side) / 2, (width - side) / 2)
                     for y in (-(depth - side) / 2, (depth - side) / 2))
    elif cfg.get('side_style', 'panels') == 'panels':
        parts.extend([
            ((side, depth, roof + board), (-(width - side) / 2, 0, (roof + board) / 2)),
            ((side, depth, roof + board), ((width - side) / 2, 0, (roof + board) / 2))])
    else:
        raise ValueError('side_style must be open_posts or panels')
    if cfg.get('divider_style', 'panel') == 'open_posts':
        parts.extend(((divider, divider, height), (0, y, bottom + height / 2))
                     for y in (-(depth - divider) / 2, (depth - divider) / 2))
    elif cfg.get('divider_style', 'panel') == 'panel':
        parts.append(((divider, depth, height), (0, 0, bottom + height / 2)))
    else:
        raise ValueError('divider_style must be open_posts or panel')
    return parts


def cell_targets(cfg):
    cell_width = (cfg['width'] - 2 * cfg['side_thickness'] - cfg['divider_thickness']) / 2
    xs = [-(cfg['divider_thickness'] + cell_width) / 2,
          (cfg['divider_thickness'] + cell_width) / 2]
    cells = []
    for shelf in cfg['shelves']:
        y = shelf['y'] + shelf['approach_sign'] * (cfg['depth'] / 2 - cfg['object_front_inset'])
        for row, floor in enumerate(reversed(cfg['floor_bottoms']), 1):
            z = floor + cfg['board_thickness'] + cfg['cube_side'] / 2 + cfg['probe_bottom_gap']
            for col, x in enumerate(xs, 1):
                cells.append({'shelf': shelf['id'], 'row': row, 'col': col, 'xyz': [x, y, z]})
    return cells


def prototype_description(description, cfg):
    """Replace shelf geometry in a private copy of the robot description."""
    root = ET.fromstring(description)
    for shelf in cfg['shelves']:
        name = f"shelf_{shelf['id']}"
        link = root.find(f"./link[@name='{name}_link']")
        joint = root.find(f"./joint[@name='{name}_joint']/origin")
        if link is None or joint is None:
            raise ValueError(f'Missing shelf {name}')
        joint.set('xyz', f"0 {shelf['y']} 0")
        for element in list(link):
            if element.tag in ('visual', 'collision'):
                link.remove(element)
        for i, (size, xyz) in enumerate(shelf_parts(cfg)):
            for kind in ('visual', 'collision'):
                element = ET.SubElement(link, kind, name=f'{name}_prototype_{i}_{kind}')
                ET.SubElement(element, 'origin', xyz=' '.join(map(str, xyz)), rpy='0 0 0')
                ET.SubElement(ET.SubElement(element, 'geometry'), 'box',
                              size=' '.join(map(str, size)))
                if kind == 'visual':
                    material = ET.SubElement(element, 'material', name='prototype_wood')
                    ET.SubElement(material, 'color', rgba='0.75 0.58 0.36 1')
    return ET.tostring(root, encoding='unicode')


def generate_launch_description():
    package = get_package_share_directory('fr3_cooperative_control')
    with open(os.path.join(package, 'config', 'compartment_shelves.json'), encoding='utf-8') as f:
        cfg = json.load(f)
    moveit = MoveItConfigsBuilder(
        'fr3_duo_gazebo', package_name='dual_fr3_moveit_config').to_moveit_configs()
    moveit.robot_description['robot_description'] = prototype_description(
        moveit.robot_description['robot_description'], cfg)
    # Match the position limits used by the physical compartment pick scenario.
    for prefix in ('left', 'right'):
        moveit.joint_limits['robot_description_planning']['joint_limits'][
            prefix + '_fr3v2_joint5'].update(
                has_position_limits=True, min_position=-2.60, max_position=2.60)
    targets = [value for cell in cell_targets(cfg) for value in cell['xyz']]
    return LaunchDescription([
        DeclareLaunchArgument('occupied_cells', default_value='true', choices=['true', 'false']),
        Node(
        package='fr3_cooperative_control', executable='compartment_reach_check',
        name='compartment_reach_check', output='screen',
        parameters=[moveit.robot_description, moveit.robot_description_semantic,
                    moveit.robot_description_kinematics, moveit.joint_limits,
                    {'use_sim_time': False, 'targets': targets,
                     'initially_occupied_shelves': cfg['initially_occupied_shelves'],
                     'occupied_cells': ParameterValue(
                         LaunchConfiguration('occupied_cells'), value_type=bool),
                     'cube_side': cfg['cube_side'], 'lift': cfg['lift'],
                     'withdraw': cfg['withdraw']}])])
