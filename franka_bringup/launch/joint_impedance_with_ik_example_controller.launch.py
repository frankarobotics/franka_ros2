#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import franka_bringup.launch_utils as launch_utils
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

load_yaml = launch_utils.load_yaml


def generate_robot_nodes(context):
    additional_nodes = []
    # Get the arguments from the launch configuration
    robot_config_file = LaunchConfiguration('robot_config_file').perform(context)

    # Include the existing example.launch.py file
    additional_nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('franka_bringup'), 'launch', 'example.launch.py'
                ])
            ),
            launch_arguments={
                'robot_config_file': robot_config_file,
                'controller_names': 'joint_impedance_with_ik_example_controller',
            }.items(),
        )
    )

    # Load the robot configuration file
    configs = load_yaml(robot_config_file)

    for _, config in configs.items():
        robot_ip = config['robot_ip']
        namespace = config['namespace']
        load_gripper = config['load_gripper']
        use_fake_hardware = config['use_fake_hardware']
        fake_sensor_commands = config['fake_sensor_commands']
        use_rviz = config['use_rviz']
        arm_prefix = config['arm_prefix']
        robot_type = config['robot_type']

        # Define the additional nodes
        additional_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare('franka_fr3_moveit_config'),
                                'launch',
                                'move_group.launch.py',
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    'robot_ip': str(robot_ip),
                    'namespace': str(namespace),
                    'load_gripper': str(load_gripper),
                    'use_fake_hardware': str(use_fake_hardware),
                    'fake_sensor_commands': str(fake_sensor_commands),
                    'use_rviz': str(use_rviz),
                    'arm_prefix': str(arm_prefix),
                    'robot_type': str(robot_type),
                }.items(),
            ),
        )
    return additional_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_bringup'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        OpaqueFunction(function=generate_robot_nodes),
    ])
