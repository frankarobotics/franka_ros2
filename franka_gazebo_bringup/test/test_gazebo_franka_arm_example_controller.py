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

TEST_DURATION = 2.0  # sec
ROBOT_TYPES = [
    "fr3", 
    # "fr3v2", 
    # "fr3v2_1"
]
CONTROLLERS = [
    "gravity_compensation_example_controller",
    "joint_impedance_example_controller",
    "joint_position_example_controller",
    "joint_velocity_example_controller",
]

params = [(robot_type, controller) for robot_type in ROBOT_TYPES for controller in CONTROLLERS]


@launch_testing.parametrize('robot_type, controller', params)
def generate_test_description(robot_type, controller):
    """Generate the test launch descriptions."""

    launch_description = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare(
                        'franka_gazebo_bringup'
                    ),
                    'launch',
                    'gazebo_franka_arm_example_controller.launch.py',
                ]
            )
        ),

        # let's use gazebo server mode and headless rendering for fast setup/teardown
        launch_arguments={
            'robot_type': robot_type,
            'controller': controller,
            'gz_args': 'empty.sdf -r -s --headless-rendering',
            'rviz': 'false'
        }.items(),
    )

    test_description = (
        LaunchDescription(
            [
                launch_description,
                actions.TimerAction(
                    period=TEST_DURATION, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ],
        ),
        {'launch_description': launch_description},
    )
    return test_description


class TestExampleController(unittest.TestCase):
    """Class for testing an Example Controller."""

    @classmethod
    def setUpClass(cls):
        """Initialize the ROS context."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown the ROS context."""
        rclpy.shutdown()

    def test_has_no_error(self, proc_output):
        """Check if any error messages have been logged."""
        has_no_error = not proc_output.waitFor(
            'ERROR', timeout=TEST_DURATION, stream='stderr'
        )

        assert has_no_error, 'Found [ERROR] log messages in launch output'
