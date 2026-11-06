"""Integration test: full Nav2 stack + MPC plugin on the Nav2 loopback simulator.

No Gazebo and no GPU are needed, so it runs in CI. The robot starts at (-2.0, -0.5) in the
TurtleBot3 sandbox map and must reach a goal on the other side of the pillar grid.
The controller under test is selected with the `controller` launch argument
(see CMakeLists.txt, one test per controller).

Exit codes are not checked after shutdown: the Nav2 component container often needs
SIGKILL to terminate (upstream behavior); a crash of the plugin, which runs inside the
controller server, would make the navigation test itself fail.
"""

import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_testing
import launch_testing.actions
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
import pytest
import rclpy

V_MAX = 0.26
W_MAX = 1.8
TIMEOUT_S = 180.0


@pytest.mark.launch_test
def generate_test_description():
    navigation = os.path.join(
        get_package_share_directory('mpc_bringup'), 'launch', 'navigation.launch.py')
    return launch.LaunchDescription([
        DeclareLaunchArgument('controller', default_value='mpc_rti'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation),
            launch_arguments={
                'controller': LaunchConfiguration('controller'),
                'simulator': 'loopback',
                'use_rviz': 'False',
                'speed_factor': '2.0',
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class TestNavigation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Do not parse sys.argv: it holds the launch arguments (controller:=...).
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_reaches_goal_within_limits(self):
        navigator = BasicNavigator('integration_test_navigator')
        commands = []
        predictions = []
        navigator.create_subscription(
            TwistStamped, 'cmd_vel', lambda msg: commands.append(msg.twist), 10)
        navigator.create_subscription(
            Path, 'FollowPath/predicted_path', lambda msg: predictions.append(msg), 10)

        navigator.setInitialPose(make_pose(navigator, -2.0, -0.5, 0.0))
        navigator.waitUntilNav2Active(localizer='controller_server')

        # Through the pillar grid, between the pillars at (1.1, 0.0) and (1.1, 1.1).
        navigator.goToPose(make_pose(navigator, 2.0, 0.5, 0.0))
        start = time.monotonic()
        while not navigator.isTaskComplete():
            if time.monotonic() - start > TIMEOUT_S:
                navigator.cancelTask()
                self.fail(f'goal not reached within {TIMEOUT_S} s')

        self.assertEqual(navigator.getResult(), TaskResult.SUCCEEDED)
        self.assertGreater(len(commands), 20, 'no velocity commands received')
        self.assertLessEqual(max(c.linear.x for c in commands), V_MAX + 1e-3)
        self.assertLessEqual(max(abs(c.angular.z) for c in commands), W_MAX + 1e-3)
        self.assertGreater(len(predictions), 0, 'the MPC did not publish its prediction')
        navigator.destroy_node()
