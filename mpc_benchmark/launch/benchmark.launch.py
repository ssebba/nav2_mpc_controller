"""One benchmark run: simulation + Nav2 (profiled controller) + scenario runner + recorder.

The launch shuts down when the scenario runner exits, so runs can be chained by a script:
  ros2 launch mpc_benchmark benchmark.launch.py controller:=mppi scenario:=s2_waypoints
See scripts/run_benchmark.sh for the full campaign.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
import yaml


def _flatten(params: dict, prefix: str = '') -> dict:
    """{'path': {'shape': 'x'}} -> {'path.shape': 'x'} (ROS parameter names)."""
    out = {}
    for key, value in params.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            out.update(_flatten(value, f'{name}.'))
        else:
            out[name] = value
    return out


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('mpc_benchmark')
    with open(os.path.join(share, 'config', 'scenarios.yaml'), 'r', encoding='utf-8') as f:
        scenarios = yaml.safe_load(f)
    name = arg('scenario')
    if name not in scenarios:
        raise RuntimeError(f"Unknown scenario '{name}', choose one of {list(scenarios)}")
    scenario = scenarios[name]
    controller = arg('controller')
    run_id = arg('run_id') or f'{name}__{controller}'
    output_dir = arg('output_dir')
    x, y, yaw = scenario['initial_pose']
    extra_params = ''
    if scenario.get('nav2_overrides'):
        fd, extra_params = tempfile.mkstemp(prefix=f'nav2_overrides_{name}_', suffix='.yaml')
        with os.fdopen(fd, 'w') as f:
            yaml.safe_dump(scenario['nav2_overrides'], f)

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mpc_bringup'), 'launch', 'navigation.launch.py')),
        launch_arguments={
            'controller': controller,
            'simulator': arg('simulator'),
            'world': scenario['world'],
            'x_pose': str(x), 'y_pose': str(y), 'yaw': str(yaw),
            'use_rviz': arg('use_rviz'),
            'rviz_config': arg('rviz_config'),
            'headless': arg('headless'),
            'profile': 'True',
            'extra_params': extra_params,
        }.items())

    common = {'use_sim_time': True, 'run_id': run_id, 'output_dir': output_dir}
    recorder_params = _flatten(scenario.get('recorder', {}))
    extra_nodes = []
    if scenario.get('ground_truth') and arg('simulator') == 'gazebo':
        # Robot pose from the simulator instead of wheel odometry, for the trajectory plots.
        extra_nodes.append(Node(
            package='mpc_benchmark', executable='ground_truth_relay_node',
            name='ground_truth_relay', output='log',
            remappings=[('~/pose', '/sim/robot_pose')]))
        recorder_params['ground_truth_topic'] = '/sim/robot_pose'
    # Configured and activated by the runner, at the start of the run.
    recorder = LifecycleNode(
        package='mpc_benchmark', executable='metrics_recorder', name='metrics_recorder',
        namespace='', output='screen', parameters=[{**common, **recorder_params}])
    runner = LifecycleNode(
        package='mpc_benchmark', executable='scenario_runner', name='scenario_runner',
        namespace='', output='screen', autostart=True,
        parameters=[{**common, 'scenario': name, 'controller': controller,
                     **_flatten(scenario['runner'])}])
    shutdown_when_done = RegisterEventHandler(OnProcessExit(
        target_action=runner,
        on_exit=[EmitEvent(event=Shutdown(reason='scenario finished'))]))
    return [navigation, *extra_nodes, recorder, runner, shutdown_when_done]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='mpc_rti'),
        DeclareLaunchArgument('scenario', default_value='s2_waypoints'),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('output_dir', default_value='/tmp/mpc_benchmark'),
        DeclareLaunchArgument('simulator', default_value='gazebo'),
        DeclareLaunchArgument('headless', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('rviz_config', default_value='mpc.rviz',
                              description='RViz configuration in mpc_bringup/rviz'),
        OpaqueFunction(function=launch_setup),
    ])
