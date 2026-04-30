#  Copyright (c) 2026 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the 'License');
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an 'AS IS' BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

"""Unified Gazebo launch test for all franka_gazebo_bringup launch files.

This single test file is parametrized over (launch_file, launch_arguments)
combinations, replacing the previous per-launch-file test scripts.
"""

import unittest

from launch import (
    actions,
    launch_description_sources,
    LaunchDescription,
    substitutions,
)
import launch_ros.substitutions
import launch_testing
import launch_testing.actions
import rclpy
import subprocess
import time

TEST_DURATION = 5.0  # sec


def ensure_gz_sim_not_running():
    """Kill any remaining Gazebo and related ROS processes between tests.

    On CI, gz sim and the controller_manager may not shut down in time
    between parametrized tests, causing the next test to fail because
    controllers are still active. We forcefully kill them here.
    See https://github.com/ros2/launch/issues/545 for details.
    """
    # First try SIGINT for graceful shutdown
    subprocess.run(['pkill', '-2', '-f', '^gz sim'], check=False)
    time.sleep(2)
    # Then SIGKILL to ensure they are gone
    subprocess.run(['pkill', '-9', '-f', '^gz sim'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'ruby.*gz'], check=False)
    # Also kill any lingering controller_manager nodes
    subprocess.run(['pkill', '-9', '-f', 'controller_manager'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'robot_state_publisher'], check=False)
    time.sleep(2)  # Allow OS to release resources (ports, shared memory)

# Each entry: (launch_file, {launch_arguments})
params = [
    # --- Franka arm example controllers ---
    (
        'gazebo_franka_arm_example_controller.launch.py',
        {
            'robot_type': 'fr3',
            'controller': 'gravity_compensation_example_controller',
            'gz_args': 'empty.sdf -r -s --headless-rendering',
            'rviz': 'false',
        },
    ),
    (
        'gazebo_franka_arm_example_controller.launch.py',
        {
            'robot_type': 'fr3',
            'controller': 'joint_impedance_example_controller',
            'gz_args': 'empty.sdf -r -s --headless-rendering',
            'rviz': 'false',
        },
    ),
    (
        'gazebo_franka_arm_example_controller.launch.py',
        {
            'robot_type': 'fr3',
            'controller': 'joint_position_example_controller',
            'gz_args': 'empty.sdf -r -s --headless-rendering',
            'rviz': 'false',
        },
    ),
    (
        'gazebo_franka_arm_example_controller.launch.py',
        {
            'robot_type': 'fr3',
            'controller': 'joint_velocity_example_controller',
            'gz_args': 'empty.sdf -r -s --headless-rendering',
            'rviz': 'false',
        },
    ),
    # --- TMR example controller ---
    (
        'gazebo_tmr_example_controller.launch.py',
        {
            'gz_args': 'empty.sdf -r -s --headless-rendering',
            'rviz': 'false',
        },
    ),
    # --- Mobile FR3 duo example ---
    (
        'gazebo_mobile_fr3_duo_example.launch.py',
        {
            'gz_args': '-r -s --headless-rendering',
            'rviz': 'false',
        },
    ),
]


@launch_testing.parametrize('launch_file, launch_args', params)
def generate_test_description(launch_file, launch_args):
    """Generate the test launch description for a given launch file."""
    ensure_gz_sim_not_running()

    launch_description = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare(
                        'franka_gazebo_bringup'
                    ),
                    'launch',
                    launch_file,
                ]
            )
        ),
        launch_arguments=launch_args.items(),
    )

    test_description = (
        LaunchDescription(
            [
                launch_description,
                actions.TimerAction(
                    period=TEST_DURATION, actions=[
                        launch_testing.actions.ReadyToTest()]
                ),
            ],
        ),
        {'launch_description': launch_description},
    )
    return test_description


class TestGazeboLaunch(unittest.TestCase):
    """Verify that Gazebo launch files start without errors."""

    @classmethod
    def setUpClass(cls):
        """Initialize the ROS context."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown the ROS context."""
        rclpy.shutdown()
        ensure_gz_sim_not_running()

    def test_has_no_error(self, proc_output):
        """Check if any error messages have been logged."""
        has_no_error = not proc_output.waitFor(
            'ERROR', timeout=TEST_DURATION, stream='stderr'
        )

        assert has_no_error, 'Found [ERROR] log messages in launch output'
