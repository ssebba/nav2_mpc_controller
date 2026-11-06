"""One-command demo: Gazebo (with GUI) + Nav2 + RViz, driven by the MPC controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    navigation = os.path.join(
        get_package_share_directory('mpc_bringup'), 'launch', 'navigation.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='mpc_rti'),
        DeclareLaunchArgument('headless', default_value='False'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation),
            launch_arguments={
                'controller': LaunchConfiguration('controller'),
                'headless': LaunchConfiguration('headless'),
                'simulator': 'gazebo',
                'use_rviz': 'True',
            }.items()),
    ])
