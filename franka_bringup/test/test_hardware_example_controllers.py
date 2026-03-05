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


def put_parameter_in_between_parameters(parameter, parameters):
    """Put the parameter in between the list to ensure it runs first."""
    result = []
    for element in parameters:
        result.extend([parameter, element])
    return result


initialize_step = {
    'controller_name': 'move_to_start_example_controller',
    'config_file_name': 'test_0.config.yaml',
}

parameters = [
    {
        'controller_name': 'cartesian_elbow_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'cartesian_orientation_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'cartesian_orientation_example_controller',
        'config_file_name': 'test_1.config.yaml',
    },
    {
        'controller_name': 'cartesian_pose_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'cartesian_pose_example_controller',
        'config_file_name': 'test_1.config.yaml',
    },
    {
        'controller_name': 'cartesian_velocity_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'elbow_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'gravity_compensation_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'gravity_compensation_example_controller',
        'config_file_name': 'test_1.config.yaml',
    },
    {
        'controller_name': 'gripper_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'joint_impedance_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'joint_impedance_with_ik_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'joint_impedance_with_ik_example_controller',
        'config_file_name': 'test_1.config.yaml',
    },
    {
        'controller_name': 'joint_position_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'joint_velocity_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'model_example_controller',
        'config_file_name': 'test_0.config.yaml',
    },
    {
        'controller_name': 'model_example_controller',
        'config_file_name': 'test_1.config.yaml',
    },
]

full_test_parameters = put_parameter_in_between_parameters(
    initialize_step, parameters
)


@launch_testing.parametrize('test_parameter', full_test_parameters)
def generate_test_description(test_parameter):
    """Generate the test launch descriptions."""
    controller_name = test_parameter['controller_name']
    config_file_name = test_parameter['config_file_name']

    robot_ip_parameter_name = 'robot_ip'
    robot_ip = substitutions.LaunchConfiguration(robot_ip_parameter_name)

    config_file = substitutions.PathJoinSubstitution(
        [
            launch_ros.substitutions.FindPackageShare(
                'franka_bringup'
            ),
            'test',
            'config',
            config_file_name,
        ]
    )

    example_launch_description = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare(
                        'franka_bringup'
                    ),
                    'launch',
                    'example.launch.py',
                ]
            )
        ),
        launch_arguments={
            'robot_config_file': config_file,
            'controller_names': controller_name,
            'robot_ips': robot_ip,
        }.items(),
    )

    test_description = (
        LaunchDescription(
            [
                actions.DeclareLaunchArgument(
                    robot_ip_parameter_name,
                    description='Hostname or IP address of the robot (required).',
                ),
                example_launch_description,
                actions.TimerAction(
                    period=3.0, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ],
        ),
        {'example_launch_description': example_launch_description},
    )
    print('Generated test description')
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
            'ERROR', timeout=5, stream='stderr'
        )

        assert has_no_error, 'Found [ERROR] log messages in launch output'
