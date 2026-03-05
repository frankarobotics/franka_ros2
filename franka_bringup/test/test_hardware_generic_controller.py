# Copyright 2026 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

from franka_bringup.testing.controller_test_utils import (
    MOVE_TO_START_CONTROLLER,
    run_controller_smoke_test,
    run_move_to_start_and_switch_to_target_controller,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import rclpy


test_parameters = [
    {
        'name': 'joint_position_example_controller',
        'description': 'Joint position control test',
        'needs_move_to_start': True,
    },
    {
        'name': 'cartesian_velocity_example_controller',
        'description': 'Cartesian velocity control test',
        'needs_move_to_start': True,
    },
    {
        'name': 'cartesian_orientation_example_controller',
        'description': 'Cartesian orientation control test',
        'needs_move_to_start': True,
    },
]


@launch_testing.parametrize('test_parameter', test_parameters)
def generate_test_description(test_parameter):
    """Generate the test launch description."""
    controller_name = test_parameter['name']
    needs_move_to_start = test_parameter['needs_move_to_start']

    robot_ip_parameter_name = 'robot_ip'
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)

    # Launch the franka robot with real hardware
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_bringup'),
                        'launch',
                        'franka.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'robot_type': 'fr3',
            robot_ip_parameter_name: robot_ip,
            'load_gripper': 'true',
        }.items(),
    )

    # Spawn the controller
    # If NEEDS_MOVE_TO_START is true, spawn in inactive mode so we can run move_to_start first
    # The test code will then activate this controller after move_to_start completes
    spawner_args = [controller_name, '--controller-manager-timeout', '30']
    if (
        needs_move_to_start
        and controller_name != 'move_to_start_example_controller'
    ):
        spawner_args.append('--inactive')

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=spawner_args,
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('franka_bringup'),
                    'config',
                    'controllers.yaml',
                ]
            )
        ],
        output='screen',
    )

    return (
        LaunchDescription(
            [
                DeclareLaunchArgument(
                    robot_ip_parameter_name,
                    description='Hostname or IP address of the robot (required).',
                ),
                franka_launch,
                controller_spawner,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            'controller_name': controller_name,
            'needs_move_to_start': needs_move_to_start,
        },
    )


class TestGenericController(unittest.TestCase):
    """Generic test class for any controller."""

    @classmethod
    def setUpClass(cls):
        """Initialize the ROS context for the test node."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown the ROS context."""
        rclpy.shutdown()

    def setUp(self):
        """Create a ROS node for tests."""
        self.link_node = rclpy.create_node('generic_controller_test_link')

    def tearDown(self):
        """Destroy the test node."""
        self.link_node.destroy_node()

    def test_controller(self, controller_name, needs_move_to_start):
        """
        Test that the controller runs successfully.

        The controller runs for a fixed duration (10 seconds). If it runs
        without errors and joint states are being published, the test passes.
        The controller is then stopped externally.

        If NEEDS_MOVE_TO_START environment variable is set to 'true', the
        move_to_start_example_controller will be run first to ensure the robot
        starts from a known position.
        """
        # Run move_to_start first if needed (and we're not already testing move_to_start)
        if needs_move_to_start and controller_name != MOVE_TO_START_CONTROLLER:
            self.link_node.get_logger().info(
                f'Running {MOVE_TO_START_CONTROLLER} to move robot to start position '
                f'before testing {controller_name}...'
            )
            move_to_start_success = (
                run_move_to_start_and_switch_to_target_controller(
                    self.link_node,
                    target_controller=controller_name,
                    wait_duration_sec=30.0,
                )
            )
            self.assertTrue(
                move_to_start_success,
                f'Failed to run {MOVE_TO_START_CONTROLLER} before testing {controller_name}',
            )
            self.link_node.get_logger().info(
                f'{MOVE_TO_START_CONTROLLER} completed. {controller_name} is now active.'
            )

        # Test duration - let controller run for this long
        # Use longer duration for move_to_start as it may take longer to reach target
        if 'move_to_start' in controller_name:
            test_duration_sec = 15.0
        else:
            test_duration_sec = 10.0

        self.link_node.get_logger().info(
            f'Testing controller: {controller_name} (will run for {test_duration_sec}s)'
        )

        run_controller_smoke_test(
            self,
            controller_name,
            test_duration_sec=test_duration_sec,
        )
