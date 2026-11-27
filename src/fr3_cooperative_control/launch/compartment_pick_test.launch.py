"""Experimental source-cell pick, withdrawal, or existing table transfer/place."""
import importlib.util
import json
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_preview():
    path = Path(__file__).with_name('compartment_preview.launch.py')
    spec = importlib.util.spec_from_file_location('compartment_preview_helpers', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_grasp_seeds(share, cfg, selected, test_stage):
    """Return matching, ordered IK guesses. Pick-only and unmatched layouts keep their search."""
    path = share / 'config' / 'compartment_grasp_seeds.json'
    if test_stage == 'pick' or not path.is_file():
        return []
    bank = json.loads(path.read_text())
    if bank['schema_version'] != 1 or bank['joint_order'] != [
            f'left_fr3v2_joint{i}' for i in range(1, 8)]:
        raise ValueError('Unsupported compartment grasp seed format or joint order')
    expected = list(selected['xyz'])
    expected[2] -= cfg['probe_bottom_gap']
    for cell in bank['cells']:
        if any(cell[key] != selected[key] for key in ('shelf', 'row', 'col')):
            continue
        if test_stage not in cell.get('stages', ('withdraw', 'transfer', 'place')):
            continue
        if (len(cell['source_xyz']) != 3 or
                any(not math.isclose(a, b, rel_tol=0, abs_tol=1e-6)
                    for a, b in zip(cell['source_xyz'], expected)) or
                any(not math.isclose(cell[key], cfg[cfg_key], rel_tol=0, abs_tol=1e-9)
                    for key, cfg_key in (('cube_side', 'cube_side'), ('lift', 'lift'),
                                         ('approach', 'withdraw'), ('withdraw', 'withdraw')))):
            continue
        seeds = cell['seeds']
        if len(seeds) > 8:
            raise ValueError('At most eight saved grasp seeds are supported per source cell')
        values = []
        for seed in seeds:
            if (seed['roll_pi'] not in (0, 1) or len(seed['joints']) != 7 or
                    not all(math.isfinite(v) for v in seed['joints'])):
                raise ValueError('Invalid saved compartment grasp seed')
            values.extend([float(seed['roll_pi']), *map(float, seed['joints'])])
        return values
    return []


def setup(context):
    row = int(LaunchConfiguration('source_row').perform(context))
    col = int(LaunchConfiguration('source_col').perform(context))
    test_stage = LaunchConfiguration('test_stage').perform(context)
    preview = load_preview()
    layout = preview.layout_module()
    share = Path(get_package_share_directory('fr3_cooperative_control'))
    cfg = json.loads((share / 'config' / 'compartment_shelves.json').read_text())
    cells = layout.cell_targets(cfg)
    selected = next(c for c in cells if c['shelf'] == 1 and c['row'] == row and c['col'] == col)
    neighbours = [c for c in cells if c['shelf'] == 1 and c != selected]
    bringup = Path(get_package_share_directory('franka_gazebo_bringup'))
    directory = Path(tempfile.mkdtemp(prefix='fr3_compartment_pick_'))
    physical = xacro.process_file(str(bringup / 'urdf' / 'fr3_duo.gazebo.urdf.xacro'), mappings={
        'robot_types': "['fr3v2', 'fr3v2']", 'hand': 'true', 'ee_id': 'franka_hand',
        'gazebo_effort': 'true', 'shelf2_y': str(cfg['shelves'][1]['y'])}).toxml()
    physical = ET.fromstring(layout.prototype_description(physical, cfg))
    # Old shelf sensor collision names no longer describe this geometry.
    for gazebo in physical.findall('gazebo'):
        if gazebo.get('reference') in ('shelf_1_link', 'shelf_2_link'):
            for sensor in list(gazebo.findall('sensor')):
                gazebo.remove(sensor)
    physical_file = directory / 'compartment_robot.urdf'
    physical_file.write_text(ET.tostring(physical, encoding='unicode'))
    world = ET.parse(bringup / 'worlds' / 'placement.sdf').getroot()
    for cell in neighbours:
        spawned = dict(cell, xyz=[cell['xyz'][0], cell['xyz'][1], cell['xyz'][2] + 0.05])
        preview.add_cube(world.find('world'), cfg, spawned)
    world_file = directory / 'compartment_pick.sdf'
    world_file.write_text(ET.tostring(world, encoding='unicode'))
    moveit = MoveItConfigsBuilder('fr3_duo_gazebo', package_name='dual_fr3_moveit_config').to_moveit_configs()
    moveit.sensors_3d.clear()
    planning = ET.fromstring(layout.prototype_description(moveit.robot_description['robot_description'], cfg))
    for cell in neighbours:
        name = f"neighbour_r{cell['row']}_c{cell['col']}"
        link = ET.SubElement(planning, 'link', name=name)
        collision = ET.SubElement(link, 'collision')
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'box',
                      size=' '.join([str(cfg['cube_side'])] * 3))
        joint = ET.SubElement(planning, 'joint', name=name + '_joint', type='fixed')
        ET.SubElement(joint, 'parent', link='world')
        ET.SubElement(joint, 'child', link=name)
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, cell['xyz'])), rpy='0 0 0')
    moveit.robot_description['robot_description'] = ET.tostring(planning, encoding='unicode')
    srdf = ET.fromstring(moveit.robot_description_semantic['robot_description_semantic'])
    for pair in list(srdf.findall('disable_collisions')):
        if any(pair.get(link) in ('shelf_1_link', 'shelf_2_link', 'table_link')
               for link in ('link1', 'link2')):
            srdf.remove(pair)
    moveit.robot_description_semantic['robot_description_semantic'] = ET.tostring(srdf, encoding='unicode')
    limits = moveit.joint_limits['robot_description_planning']['joint_limits']
    for prefix in ('left', 'right'):
        limits[prefix + '_fr3v2_joint5'].update(
            has_position_limits=True, min_position=-2.60, max_position=2.60)
    parameters = [moveit.robot_description, moveit.robot_description_semantic,
                  moveit.robot_description_kinematics, moveit.joint_limits, {'use_sim_time': True}]
    # Reuse the existing measured table flow after the compartment withdrawal.
    expected = list(selected['xyz'])
    expected[2] -= cfg['probe_bottom_gap']
    shelf_boxes = []
    for shelf in cfg['shelves']:
        for size, xyz in layout.shelf_parts(cfg):
            shelf_boxes.extend([float(xyz[0]), float(xyz[1] + shelf['y']), float(xyz[2]),
                                *map(float, size)])
    task_stage = test_stage if test_stage in ('transfer', 'place') else 'pick'
    saved_seeds = load_grasp_seeds(share, cfg, selected, test_stage)
    seed_parameters = {'compartment_grasp_seed_bank': saved_seeds} if saved_seeds else {}
    carry_parameters = ({'compartment_table_angular_limit': 0.45}
                        if (row, col) == (4, 2) and test_stage in ('transfer', 'place') else {})
    task = Node(package='fr3_cooperative_control', executable='single_arm_pick_test',
                name='single_arm_pick_test', output='screen', parameters=parameters + [{
                    'stage': task_stage, 'source_xyz': expected,
                    **seed_parameters,
                    **carry_parameters,
                    'pick_lift': cfg['lift'], 'pick_approach': cfg['withdraw'],
                    'compartment_withdraw_distance': cfg['withdraw'] if test_stage != 'pick' else 0.0,
                    'compartment_shelf_boxes': shelf_boxes,
                    'compartment_front_y': cfg['shelves'][0]['y'] + cfg['depth'] / 2}])
    library = Path(get_package_prefix('moveit_ros_move_group')) / 'lib' / 'libmoveit_move_group_default_capabilities.so'
    env = {'LD_PRELOAD': [str(library), ' ', EnvironmentVariable('LD_PRELOAD', default_value='')]} if library.is_file() else {}
    def finished(event, context):
        label = 'COMPARTMENT ' + test_stage.upper()
        return [LogInfo(msg=label + (' PASSED' if event.returncode == 0 else
                        ' FAILED: inspect the simulation before restarting'))]
    return [
        LogInfo(msg=f'Experimental {test_stage}: shelf 1 row {row} col {col}; target shelf empty.'),
        LogInfo(msg=f'Saved grasp IK guesses: {len(saved_seeds) // 8}; all paths are rechecked.'),
        LogInfo(msg=f'Generated scene: {directory}'),
        RegisterEventHandler(OnProcessExit(target_action=task, on_exit=finished)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            bringup / 'launch' / 'gazebo_fr3_duo_example.launch.py')), launch_arguments={
                'rviz': 'false', 'world': str(world_file),
                'robot_description_file': str(physical_file),
                'shelf2_y': str(cfg['shelves'][1]['y']),
                'gz_args': '-r -s' if LaunchConfiguration('headless').perform(context) == 'true' else '-r'
            }.items()),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             additional_env=env, parameters=[moveit.to_dict(), {
                 'use_sim_time': True, 'trajectory_execution.allowed_execution_duration_scaling': 15.0,
                 'trajectory_execution.allowed_goal_duration_margin': 30.0}]),
        Node(package='ros_gz_sim', executable='create', name='spawn_target_box', output='screen',
             arguments=['-world', 'empty', '-name', 'target_box', '-allow_renaming', 'false',
                        '-file', str(bringup / 'models' / 'target_box' / 'model.sdf'),
                        '-x', str(selected['xyz'][0]), '-y', str(selected['xyz'][1]),
                        '-z', str(selected['xyz'][2] + 0.05)]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/model/target_box/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
                        '/world/empty/model/target_box/link/link/sensor/box_contacts/contact'
                        '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                        '/world/empty/model/fr3_duo_gazebo/link/table_link/sensor/table_contacts/contact'
                        '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],
             remappings=[('/model/target_box/pose', '/target_box/pose'),
                         ('/world/empty/model/target_box/link/link/sensor/box_contacts/contact',
                          '/placement/box_contacts'),
                         ('/world/empty/model/fr3_duo_gazebo/link/table_link/sensor/table_contacts/contact',
                          '/placement/table_contacts')]),
        task]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source_row', default_value='3', choices=['1', '2', '3', '4']),
        DeclareLaunchArgument('source_col', default_value='1', choices=['1', '2']),
        DeclareLaunchArgument('test_stage', default_value='pick',
                              choices=['pick', 'withdraw', 'transfer', 'place']),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup)])
