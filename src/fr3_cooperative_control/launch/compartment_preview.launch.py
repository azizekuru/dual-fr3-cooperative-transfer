"""Separate Gazebo layout preview: frozen robots, physical cubes, no controllers."""
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def layout_module():
    path = Path(__file__).with_name('compartment_reach_check.launch.py')
    spec = importlib.util.spec_from_file_location('compartment_layout', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matrix_rpy(r):
    p = math.asin(max(-1.0, min(1.0, -r[2][0])))
    if abs(math.cos(p)) > 1e-8:
        return [math.atan2(r[2][1], r[2][2]), p, math.atan2(r[1][0], r[0][0])]
    return [0, p, math.atan2(-r[0][1], r[1][1])]


def freeze_description(description):
    """Bake ready joint values into fixed origins; strip all control/sensor plugins."""
    root = ET.fromstring(description)
    ready = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    positions = {f'{prefix}_fr3v2_joint{i + 1}': value
                 for prefix in ('left', 'right') for i, value in enumerate(ready)}
    positions.update({f'{prefix}_fr3v2_finger_joint{i}': 0.04
                      for prefix in ('left', 'right') for i in (1, 2)})
    for element in list(root):
        if element.tag in ('gazebo', 'ros2_control', 'transmission'):
            root.remove(element)
    for joint in root.findall('joint'):
        kind = joint.get('type')
        if kind == 'fixed':
            continue
        if kind not in ('revolute', 'continuous', 'prismatic'):
            raise ValueError(f'Unsupported preview joint: {joint.get("name")} ({kind})')
        if joint.get('name') not in positions:
            raise ValueError(f'Missing preview position for {joint.get("name")}')
        value = positions[joint.get('name')]
        origin = joint.find('origin')
        if origin is None:
            origin = ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
        xyz = list(map(float, origin.get('xyz', '0 0 0').split()))
        rotation = rpy_matrix(list(map(float, origin.get('rpy', '0 0 0').split())))
        axis_element = joint.find('axis')
        axis = list(map(float, axis_element.get('xyz').split())) if axis_element is not None else [1, 0, 0]
        norm = math.sqrt(sum(v * v for v in axis))
        x, y, z = [v / norm for v in axis]
        if kind == 'prismatic':
            xyz = [xyz[i] + value * sum(rotation[i][k] * (x, y, z)[k] for k in range(3))
                   for i in range(3)]
        else:
            c, s, t = math.cos(value), math.sin(value), 1 - math.cos(value)
            turn = [[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
                    [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
                    [t*x*z-s*y, t*y*z+s*x, t*z*z+c]]
            rotation = matmul(rotation, turn)
        origin.set('xyz', ' '.join(map(str, xyz)))
        origin.set('rpy', ' '.join(map(str, matrix_rpy(rotation))))
        joint.set('type', 'fixed')
        for child in list(joint):
            if child.tag in ('axis', 'limit', 'dynamics', 'mimic', 'safety_controller'):
                joint.remove(child)
    gazebo = ET.SubElement(root, 'gazebo')
    ET.SubElement(gazebo, 'static').text = 'true'
    return ET.tostring(root, encoding='unicode')


def add_cube(world, cfg, cell):
    name = f"shelf_{cell['shelf']}_r{cell['row']}_c{cell['col']}_cube"
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'pose').text = ' '.join(map(str, cell['xyz'])) + ' 0 0 0'
    link = ET.SubElement(model, 'link', name='link')
    side, mass = cfg['cube_side'], 0.05
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = str(mass)
    inertia = ET.SubElement(inertial, 'inertia')
    for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'):
        ET.SubElement(inertia, key).text = str(mass * side * side / 6 if key in ('ixx', 'iyy', 'izz') else 0)
    colors = ['0.18 0.48 0.85 1', '0.90 0.45 0.12 1', '0.22 0.65 0.35 1', '0.66 0.30 0.72 1']
    for kind in ('collision', 'visual'):
        element = ET.SubElement(link, kind, name=kind)
        ET.SubElement(ET.SubElement(ET.SubElement(element, 'geometry'), 'box'), 'size').text = f'{side} {side} {side}'
        if kind == 'visual':
            material = ET.SubElement(element, 'material')
            for field in ('ambient', 'diffuse'):
                ET.SubElement(material, field).text = colors[cell['row'] - 1]
        else:
            ode = ET.SubElement(ET.SubElement(ET.SubElement(element, 'surface'), 'friction'), 'ode')
            ET.SubElement(ode, 'mu').text = '1.0'
            ET.SubElement(ode, 'mu2').text = '1.0'
    return dict(cell, model=name)


def build_preview(directory):
    directory = Path(directory)
    layout = layout_module()
    package = get_package_share_directory('fr3_cooperative_control')
    cfg = json.loads(Path(package, 'config', 'compartment_shelves.json').read_text())
    moveit = MoveItConfigsBuilder('fr3_duo_gazebo', package_name='dual_fr3_moveit_config').to_moveit_configs()
    prototype = layout.prototype_description(moveit.robot_description['robot_description'], cfg)
    urdf = directory / 'static_layout.urdf'
    urdf.write_text(freeze_description(prototype))
    conversion = subprocess.run(['gz', 'sdf', '-p', str(urdf)], text=True,
                                capture_output=True, check=True)
    converted = ET.fromstring(conversion.stdout)
    model = converted.find('model')
    if model is None:
        raise ValueError('Gazebo did not convert the preview model')
    model.set('name', 'static_dual_fr3_compartment_layout')
    static = model.find('static')
    if static is None:
        static = ET.SubElement(model, 'static')
    static.text = 'true'
    # No controls or sensors from the old rack may survive this conversion.
    if model.findall('.//plugin') or model.findall('.//sensor'):
        raise ValueError('Unexpected plugin/sensor in frozen preview')
    bringup = Path(get_package_share_directory('franka_gazebo_bringup'))
    root = ET.parse(bringup / 'worlds' / 'placement.sdf').getroot()
    world = root.find('world')
    world.set('name', 'compartment_preview')
    world.append(model)
    cells = []
    for cell in layout.cell_targets(cfg):
        occupied = cell['shelf'] in cfg['initially_occupied_shelves']
        cells.append(dict(add_cube(world, cfg, cell), occupied=True) if occupied else
                     dict(cell, model=None, occupied=False))
    path = directory / 'compartment_preview.sdf'
    ET.indent(root)
    path.write_text(ET.tostring(root, encoding='unicode'))
    (directory / 'cells.json').write_text(json.dumps(cells, indent=2))
    return path


def setup(context):
    # Keep the generated files available after closing for inspection.
    directory = tempfile.mkdtemp(prefix='fr3_compartment_preview_')
    world = build_preview(directory)
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    iterations = int(LaunchConfiguration('iterations').perform(context))
    if iterations < 0:
        raise ValueError('iterations must be non-negative')
    command = ['gz', 'sim', '-r', '-v', '3']
    if headless:
        command.append('-s')
    if iterations:
        command.extend(['--iterations', str(iterations)])
    command.append(str(world))
    resources = str(Path(get_package_share_directory('franka_description')).parent)
    resources += os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    return [LogInfo(msg=f'PREVIEW WORLD: {world}'),
            LogInfo(msg='Preview only: shelf 1 has eight cubes; shelf 2 is empty. Frozen arms; no tasks.'),
            LogInfo(msg='Rows top-to-bottom 1-4; columns world -X/+X. Entity names: shelf_N_rR_cC_cube.'),
            ExecuteProcess(cmd=command, output='screen', additional_env={
                'GZ_SIM_RESOURCE_PATH': resources,
                'GZ_PARTITION': f'fr3_compartment_preview_{os.getpid()}'})]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('iterations', default_value='0'),
        OpaqueFunction(function=setup)])
