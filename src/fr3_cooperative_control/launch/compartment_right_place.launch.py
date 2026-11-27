"""Shared compartment scene: isolated right placement or sequential handoff."""
import importlib.util
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context, handoff=False):
    """Build one scene; start the right task after left success in handoff mode."""
    share = Path(get_package_share_directory('fr3_cooperative_control'))
    spec = importlib.util.spec_from_file_location('compartment_geometry', share / 'launch/compartment_pick_test.launch.py')
    helpers = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helpers)
    preview = helpers.load_preview()
    layout = preview.layout_module()
    cfg = json.loads((share / 'config/compartment_shelves.json').read_text())
    if cfg['cube_side'] != 0.04:
        raise ValueError('This first placement test requires a 40 mm cube')
    cells = layout.cell_targets(cfg)
    destination = next(c for c in cells if (c['shelf'], c['row'], c['col']) == (2, 3, 1))
    source = [c for c in cells if c['shelf'] == 1]
    selected = None
    spawn = [0.0, 0.05, 0.471]
    if handoff:
        row = int(LaunchConfiguration('source_row').perform(context))
        col = int(LaunchConfiguration('source_col').perform(context))
        selected = next(c for c in source if (c['row'], c['col']) == (row, col))
        spawn = [*selected['xyz'][:2], selected['xyz'][2] + 0.05]
    neighbours = [c for c in source if c != selected]
    target = list(destination['xyz'])
    target[2] -= cfg['probe_bottom_gap']
    floor_index = len(cfg['floor_bottoms']) - destination['row']
    board_token = f'shelf_2_prototype_{floor_index}_collision'
    bringup = Path(get_package_share_directory('franka_gazebo_bringup'))
    directory = Path(tempfile.mkdtemp(prefix='fr3_compartment_right_'))
    physical = ET.fromstring(layout.prototype_description(xacro.process_file(str(
        bringup / 'urdf/fr3_duo.gazebo.urdf.xacro'), mappings={
            'robot_types': "['fr3v2', 'fr3v2']", 'hand': 'true', 'ee_id': 'franka_hand',
            'gazebo_effort': 'true', 'shelf2_y': str(cfg['shelves'][1]['y'])}).toxml(), cfg))
    for gazebo in physical.findall('gazebo'):
        if gazebo.get('reference') in ('shelf_1_link', 'shelf_2_link'):
            for sensor in list(gazebo.findall('sensor')):
                gazebo.remove(sensor)
    gazebo = ET.SubElement(physical, 'gazebo', reference='shelf_2_link')
    sensor = ET.SubElement(gazebo, 'sensor', name='compartment_shelf_contacts', type='contact')
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '50'
    ET.SubElement(sensor, 'topic').text = '/compartment/shelf2_contacts'
    contact = ET.SubElement(sensor, 'contact')
    ET.SubElement(contact, 'topic').text = '/compartment/shelf2_contacts'
    for collision in physical.find("./link[@name='shelf_2_link']").findall('collision'):
        ET.SubElement(contact, 'collision').text = collision.get('name')
    physical_file = directory / 'robot.urdf'
    physical_file.write_text(ET.tostring(physical, encoding='unicode'))
    # URDF fixed-joint lumping renames collision entities. Sensors must refer
    # to the converted names, not the original URDF labels.
    converted = ET.fromstring(subprocess.run(
        ['gz', 'sdf', '-p', str(physical_file)], check=True, capture_output=True, text=True).stdout)
    converted_rack = converted.find("./model/link[@name='shelf_2_link']")
    collision_names = [c.get('name') for c in converted_rack.findall('collision')
                       if 'shelf_2_prototype_' in c.get('name', '')] if converted_rack is not None else []
    if len(collision_names) != len(contact.findall('collision')):
        raise ValueError('Converted shelf sensor geometry does not match the rack')
    for old in list(contact.findall('collision')):
        contact.remove(old)
    for name in collision_names:
        ET.SubElement(contact, 'collision').text = name
    physical_file.write_text(ET.tostring(physical, encoding='unicode'))
    world = ET.parse(bringup / 'worlds/placement.sdf').getroot()
    for cell in neighbours:
        preview.add_cube(world.find('world'), cfg,
                         dict(cell, xyz=[*cell['xyz'][:2], cell['xyz'][2] + 0.05]))
    world_file = directory / 'world.sdf'
    world_file.write_text(ET.tostring(world, encoding='unicode'))
    moveit = MoveItConfigsBuilder('fr3_duo_gazebo', package_name='dual_fr3_moveit_config').to_moveit_configs()
    if handoff:
        # Preserve the default pipeline for the right arm and legacy commands.
        precise = copy.deepcopy(moveit.planning_pipelines['ompl'])
        precise.setdefault('arm_1', {})['longest_valid_segment_fraction'] = 0.001
        precise.setdefault('totg', {})['path_tolerance'] = 0.001
        moveit.planning_pipelines['ompl_compartment_left'] = precise
        moveit.planning_pipelines['planning_pipelines'].append('ompl_compartment_left')
    moveit.sensors_3d.clear()
    planning = ET.fromstring(layout.prototype_description(moveit.robot_description['robot_description'], cfg))
    rack = planning.find("./link[@name='shelf_2_link']")
    board = rack.find(f"./collision[@name='{board_token}']")
    if board is None:
        raise ValueError('Destination board collision missing')
    rack.remove(board)
    support = ET.SubElement(planning, 'link', name='destination_support_link')
    support.append(board)
    joint = ET.SubElement(planning, 'joint', name='destination_support_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='shelf_2_link')
    ET.SubElement(joint, 'child', link='destination_support_link')
    for cell in neighbours:
        name = f"neighbour_r{cell['row']}_c{cell['col']}"
        link = ET.SubElement(planning, 'link', name=name)
        ET.SubElement(ET.SubElement(ET.SubElement(link, 'collision'), 'geometry'), 'box', size='0.04 0.04 0.04')
        joint = ET.SubElement(planning, 'joint', name=name + '_joint', type='fixed')
        ET.SubElement(joint, 'parent', link='world')
        ET.SubElement(joint, 'child', link=name)
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, cell['xyz'])), rpy='0 0 0')
    moveit.robot_description['robot_description'] = ET.tostring(planning, encoding='unicode')
    srdf = ET.fromstring(moveit.robot_description_semantic['robot_description_semantic'])
    for pair in list(srdf.findall('disable_collisions')):
        if any(pair.get(link) in ('shelf_1_link', 'shelf_2_link', 'table_link') for link in ('link1', 'link2')):
            srdf.remove(pair)
    ET.SubElement(srdf, 'disable_collisions', link1='shelf_2_link',
                  link2='destination_support_link', reason='Adjacent static shelf components')
    moveit.robot_description_semantic['robot_description_semantic'] = ET.tostring(srdf, encoding='unicode')
    for prefix in ('left', 'right'):
        moveit.joint_limits['robot_description_planning']['joint_limits'][prefix + '_fr3v2_joint5'].update(
            has_position_limits=True, min_position=-2.60, max_position=2.60)
    params = [moveit.robot_description, moveit.robot_description_semantic,
              moveit.robot_description_kinematics, moveit.joint_limits, {'use_sim_time': True}]
    stage = 'shelf_place' if handoff else LaunchConfiguration('test_stage').perform(context)
    task = Node(package='fr3_cooperative_control', executable='right_arm_table_pick',
                name='right_arm_table_pick', output='screen', parameters=params + [{
                    'stage': stage, 'left_placed': handoff, 'destination_xyz': target,
                    'destination_board_token': board_token,
                    'destination_support_link': 'destination_support_link'}])

    state = {'left_passed': False, 'right_started': False}
    left_task = None
    if handoff:
        expected = list(selected['xyz'])
        expected[2] -= cfg['probe_bottom_gap']
        seeds = helpers.load_grasp_seeds(share, cfg, selected, 'place')
        extra = {'compartment_grasp_seed_bank': seeds} if seeds else {}
        if (row, col) == (4, 2):
            extra['compartment_table_angular_limit'] = 0.45
        shelf_boxes = []
        for shelf in cfg['shelves']:
            for size, xyz in layout.shelf_parts(cfg):
                shelf_boxes.extend([float(xyz[0]), float(xyz[1] + shelf['y']), float(xyz[2]),
                                    *map(float, size)])
        left_task = Node(package='fr3_cooperative_control', executable='single_arm_pick_test',
                         name='single_arm_pick_test', output='screen', parameters=params + [{
                             'stage': 'place', 'source_xyz': expected, **extra,
                             'compartment_planning_pipeline': 'ompl_compartment_left',
                             'pick_lift': cfg['lift'], 'pick_approach': cfg['withdraw'],
                             'compartment_withdraw_distance': cfg['withdraw'],
                             'compartment_shelf_boxes': shelf_boxes,
                             'compartment_front_y': cfg['shelves'][0]['y'] + cfg['depth'] / 2}])

    def after_left(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='COMPARTMENT FULL TRANSFER FAILED: left placement failed; right pickup skipped')]
        state['left_passed'] = True
        state['right_started'] = True
        return [LogInfo(msg='COMPARTMENT LEFT PLACE PASSED: starting right-arm parking and pickup'), task]

    def finished(event, context):
        actions = [LogInfo(msg='COMPARTMENT RIGHT ' + ('PLACE' if stage == 'shelf_place' else 'PICK') +
                           (' PASSED' if event.returncode == 0 else ' FAILED'))]
        if handoff:
            passed = event.returncode == 0 and state['left_passed'] and state['right_started']
            actions.append(LogInfo(msg='COMPARTMENT FULL TRANSFER ' +
                                   ('PASSED' if passed else 'FAILED: right stage did not complete')))
        return actions

    library = Path(get_package_prefix('moveit_ros_move_group')) / 'lib/libmoveit_move_group_default_capabilities.so'
    env = {'LD_PRELOAD': [str(library), ' ', EnvironmentVariable('LD_PRELOAD', default_value='')]} if library.is_file() else {}
    return [
        LogInfo(msg=(f'Compartment transfer: shelf 1 row {row} col {col} to shelf 2 row 3 col 1; '
                     if handoff else 'Isolated right test: table to shelf 2 row 3 col 1; ') +
                    f'target={target}; generated={directory}'),
        RegisterEventHandler(OnProcessExit(target_action=task, on_exit=finished)),
        *([RegisterEventHandler(OnProcessExit(target_action=left_task, on_exit=after_left))] if handoff else []),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup / 'launch/gazebo_fr3_duo_example.launch.py')),
                                 launch_arguments={'rviz': 'false', 'world': str(world_file),
                                                   'robot_description_file': str(physical_file),
                                                   'shelf2_y': str(cfg['shelves'][1]['y']),
                                                   'gz_args': '-r -s' if LaunchConfiguration('headless').perform(context) == 'true' else '-r'}.items()),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen', additional_env=env,
             parameters=[moveit.to_dict(), {'use_sim_time': True,
                         'trajectory_execution.allowed_execution_duration_scaling': 15.0,
                         'trajectory_execution.allowed_goal_duration_margin': 30.0}]),
        Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
            '-world', 'empty', '-name', 'target_box', '-allow_renaming', 'false',
            '-file', str(bringup / 'models/target_box/model.sdf'),
            '-x', str(spawn[0]), '-y', str(spawn[1]), '-z', str(spawn[2])]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen', arguments=[
            '/model/target_box/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '/world/empty/model/target_box/link/link/sensor/box_contacts/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/empty/model/fr3_duo_gazebo/link/table_link/sensor/table_contacts/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/compartment/shelf2_contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'], remappings=[
                ('/model/target_box/pose', '/target_box/pose'),
                ('/world/empty/model/target_box/link/link/sensor/box_contacts/contact', '/placement/box_contacts'),
                ('/world/empty/model/fr3_duo_gazebo/link/table_link/sensor/table_contacts/contact', '/placement/table_contacts'),
                ('/compartment/shelf2_contacts', '/placement/shelf2_contacts')]),
        left_task if handoff else task]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('test_stage', default_value='shelf_place', choices=['pick', 'shelf_place']),
        OpaqueFunction(function=setup)])
