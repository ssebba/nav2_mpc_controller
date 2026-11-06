"""Nav2 + simulated TurtleBot3 Waffle with a selectable local controller.

The parameters start from the official nav2_bringup/params/nav2_params.yaml, then
config/nav2_overrides.yaml is applied and the FollowPath section is replaced by
config/controllers/<controller>.yaml. Planner, costmaps, behavior tree and velocity
smoother are therefore identical for every controller, which keeps comparisons fair.

Examples:
  ros2 launch mpc_bringup navigation.launch.py                       # Gazebo + RViz + MPC
  ros2 launch mpc_bringup navigation.launch.py controller:=mppi
  ros2 launch mpc_bringup navigation.launch.py simulator:=loopback   # no Gazebo, fast
"""

import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml

CONTROLLERS = ['mpc_rti', 'mpc_ltv', 'baseline', 'dwb', 'rpp', 'mppi']


def _deep_merge(base: dict, overlay: dict) -> dict:
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def _load(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def build_params_file(controller: str, extra_params: str, pose: dict, speed_factor: float,
                      profile: bool = False) -> str:
    """Write the merged Nav2 parameter file and return its path."""
    bringup_share = get_package_share_directory('mpc_bringup')
    params = _load(os.path.join(
        get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml'))
    _deep_merge(params, _load(os.path.join(bringup_share, 'config', 'nav2_overrides.yaml')))

    overlay = _load(os.path.join(bringup_share, 'config', 'controllers', f'{controller}.yaml'))
    follow_path = overlay['controller_server']['ros__parameters'].pop('FollowPath')
    # Replace (not merge) the controller section: leftovers of the default MPPI config
    # must not leak into another plugin's namespace.
    if profile:
        # Same plugin, wrapped by the benchmark decorator that publishes its compute time.
        follow_path['inner_plugin'] = follow_path['plugin']
        follow_path['plugin'] = 'mpc_benchmark::ProfilingController'
    params['controller_server']['ros__parameters']['FollowPath'] = follow_path
    _deep_merge(params, overlay)

    if extra_params:
        _deep_merge(params, _load(extra_params))

    params['amcl']['ros__parameters']['initial_pose'] = {
        'x': pose['x'], 'y': pose['y'], 'z': 0.0, 'yaw': pose['yaw']}
    params.setdefault('loopback_simulator', {}).setdefault('ros__parameters', {})[
        'speed_factor'] = speed_factor

    fd, path = tempfile.mkstemp(prefix=f'nav2_{controller}_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(params, f, sort_keys=False)
    return path


def _initial_pose_msg(pose: dict) -> str:
    qz, qw = math.sin(pose['yaw'] / 2.0), math.cos(pose['yaw'] / 2.0)
    return ('{header: {frame_id: map}, pose: {pose: {position: {x: %f, y: %f}, '
            'orientation: {z: %f, w: %f}}}}' % (pose['x'], pose['y'], qz, qw))


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    controller = arg('controller')
    if controller not in CONTROLLERS:
        raise RuntimeError(f"Unknown controller '{controller}', choose one of {CONTROLLERS}")
    simulator = arg('simulator')
    pose = {'x': float(arg('x_pose')), 'y': float(arg('y_pose')), 'yaw': float(arg('yaw'))}
    params_file = build_params_file(
        controller, arg('extra_params'), pose, float(arg('speed_factor')),
        arg('profile').lower() == 'true')

    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    sim_share = get_package_share_directory('nav2_minimal_tb3_sim')
    bringup_share = get_package_share_directory('mpc_bringup')
    worlds = {
        'sandbox': (os.path.join(sim_share, 'worlds', 'tb3_sandbox.sdf.xacro'),
                    os.path.join(get_package_share_directory('nav2_bringup'), 'maps',
                                 'tb3_sandbox.yaml')),
        'empty': (os.path.join(sim_share, 'worlds', 'tb3_empty_world.sdf.xacro'),
                  os.path.join(bringup_share, 'maps', 'empty.yaml')),
        # Same (empty) map as 'empty': the boxes are unknown to the planner.
        'empty_obstacles': (os.path.join(bringup_share, 'worlds', 'empty_obstacles.sdf.xacro'),
                            os.path.join(bringup_share, 'maps', 'empty.yaml')),
    }
    world_file, map_file = worlds[arg('world')]
    common = {
        'params_file': params_file,
        'map': map_file,
        'use_rviz': arg('use_rviz'),
        'rviz_config_file': os.path.join(bringup_share, 'rviz', arg('rviz_config')),
        'autostart': 'True',
    }

    if simulator == 'gazebo':
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'tb3_simulation_launch.py')),
            launch_arguments={**common, 'world': world_file, 'headless': arg('headless'),
                              'x_pose': str(pose['x']), 'y_pose': str(pose['y']),
                              'yaw': str(pose['yaw'])}.items())]
    if simulator == 'loopback':
        # The loopback simulator waits for an initial pose before publishing TF.
        publish_initial_pose = TimerAction(period=3.0, actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '-w', '1', '/initialpose',
                 'geometry_msgs/msg/PoseWithCovarianceStamped', _initial_pose_msg(pose)],
            output='log')])
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'tb3_loopback_simulation_launch.py')),
            launch_arguments=common.items()), publish_initial_pose]
    raise RuntimeError(f"Unknown simulator '{simulator}', choose 'gazebo' or 'loopback'")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='mpc_rti',
                              description=f'Local controller: {CONTROLLERS}'),
        DeclareLaunchArgument('simulator', default_value='gazebo',
                              description="'gazebo' or 'loopback' (Nav2 kinematic simulator)"),
        DeclareLaunchArgument('world', default_value='sandbox',
                              description="'sandbox', 'empty' or 'empty_obstacles' (boxes not "
                                          "in the map)"),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('rviz_config', default_value='mpc.rviz',
                              description="RViz layout in mpc_bringup/rviz ('mpc_record.rviz' for videos)"),
        DeclareLaunchArgument('headless', default_value='True',
                              description='Run Gazebo without its GUI'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('speed_factor', default_value='1.0',
                              description='Clock speed of the loopback simulator'),
        DeclareLaunchArgument('extra_params', default_value='',
                              description='Optional YAML merged last'),
        DeclareLaunchArgument('profile', default_value='False',
                              description='Wrap the controller with mpc_benchmark::ProfilingController'),
        OpaqueFunction(function=launch_setup),
    ])
