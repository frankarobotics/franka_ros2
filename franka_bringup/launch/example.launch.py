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

############################################################################
# Parameters:
# controller_names: Comma-separated list of controller names to spawn (required, no default)
# robot_config_file: Configuration file name or path. If just a filename is
#                   provided (e.g., 'franka.config.yaml'), it will be
#                   looked up in franka_bringup/config/ directory.
#                   (default: franka.config.yaml)
#
# The example.launch.py launch file provides a flexible and unified interface
# for launching Franka Robotics example controllers for single robot setups
# via the 'controller_names' parameter, such as 'elbow_example_controller'.
# For dual-arm (duo) setups, use fr3_duo.launch.py directly.
# Example:
# ros2 launch franka_bringup example.launch.py controller_names:=elbow_example_controller
#
# This script "includes" franka.launch.py to declare core component nodes,
# including: robot_state_publisher, ros2_control_node, joint_state_publisher,
# joint_state_broadcaster, franka_robot_state_broadcaster, and optionally
# franka_gripper and rviz, with support for namespaced and non-namespaced
# environments as defined in franka.config.yaml. RViz is launched if
# 'use_rviz' is set to true in the configuration file.
#
# The default robot_config_file is franka.config.yaml in the
# franka_bringup/config directory. See that file for its own documentation.
#
# This approach improves upon the earlier individual launch scripts, which
# varied in structure and lacked namespace support, offering a more consistent
# and maintainable solution. While some may favor the older scripts for their
# specific use cases, example.launch.py enhances scalability and ease of use
# for a wide range of Franka Robotics applications.
#
# Ensure the specified controller_names match controllers defined in
# controllers.yaml to avoid runtime errors.
############################################################################

import os
import sys

from ament_index_python.packages import get_package_share_directory

import franka_bringup.launch_utils as launch_utils

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# constant for the controller name parameter
CONTROLLER_EXAMPLE = 'controller'


load_yaml = launch_utils.load_yaml
get_parameter_for_config = launch_utils.get_parameter_for_config
package_share = get_package_share_directory('franka_bringup')

# Iterates over the uncommented lines in file specified by the robot_config_file parameter.
# 'Includes' franka.launch.py for each active (uncommented) Robot.
# That file is well documented.
# The function also checks if the 'use_rviz' parameter is set to true in the YAML file.
# If so, it includes a node for RViz to visualize the robot's state.
# The function returns a list of nodes to be launched.


def generate_robot_nodes(context):
    config_file = LaunchConfiguration('robot_config_file').perform(context)

    # If config_file is just a filename (no path separators), look in franka_bringup/config/
    if not os.path.isabs(config_file) and os.path.sep not in config_file:
        config_file = os.path.join(package_share, 'config', config_file)

    controller_names = LaunchConfiguration('controller_names').perform(context)
    robot_ips = LaunchConfiguration('robot_ips').perform(context)
    configs = load_yaml(config_file)
    nodes = []

    for index, (_, config) in enumerate(configs.items()):
        namespace = config.get('namespace', '')

        if robot_ips:
            robot_ip = get_parameter_for_config(
                robot_ips, num_configs=len(configs), config_index=index
            )
        else:
            robot_ip = str(config['robot_ip'])

        # Single robot configuration: use franka.launch.py
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare('franka_bringup'),
                            'launch',
                            'franka.launch.py',
                        ]
                    )
                ),
                launch_arguments={
                    'robot_type': str(config['robot_type']),
                    'arm_prefix': str(config['arm_prefix']),
                    'namespace': str(namespace),
                    'robot_ip': robot_ip,
                    'load_gripper': str(config['load_gripper']),
                    'use_fake_hardware': str(config['use_fake_hardware']),
                    'fake_sensor_commands': str(
                        config['fake_sensor_commands']
                    ),
                    'joint_state_rate': str(config['joint_state_rate']),
                }.items(),
            )
        )

        # Determine which controller to use for this config
        controller_name = get_parameter_for_config(
            controller_names, num_configs=len(configs), config_index=index
        )
        if not controller_name:
            print(
                'Error: No controller names provided. Please provide at least one controller name.'
            )
            sys.exit(1)

        if CONTROLLER_EXAMPLE in controller_name:
            # Spawn the example as ros2_control controller
            nodes.append(
                Node(
                    package='controller_manager',
                    executable='spawner',
                    namespace=namespace,
                    arguments=[
                        controller_name,
                        '--controller-manager-timeout',
                        '30',
                    ],
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
            )
        else:
            # Spawn the example as node
            nodes.append(
                Node(
                    package='franka_example_controllers',
                    executable=controller_name,
                    namespace=namespace,
                    output='screen',
                )
            )

    if any(
        str(config.get('use_rviz', 'false')).lower() == 'true'
        for config in configs.values()
    ):
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '--display-config',
                    PathJoinSubstitution(
                        [
                            FindPackageShare('franka_description'),
                            'rviz',
                            'visualize_franka.rviz',
                        ]
                    ),
                ],
                output='screen',
            )
        )
    return nodes


# The generate_launch_description function is the entry point (like 'main')
# It is called by the ROS 2 launch system when the launch file is executed.
# via: ros2 launch franka_bringup example.launch.py ARGS...
# This function must return a LaunchDescription object containing nodes to be launched.
# it calls the generate_robot_nodes function to get the list of nodes to be launched.


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_config_file',
                default_value='franka.config.yaml',
                description='Config file name (looked up in franka_bringup/config/) or full path',
            ),
            DeclareLaunchArgument(
                'controller_names',
                description='Comma-separated list of controller names to spawn (required)',
            ),
            DeclareLaunchArgument(
                'robot_ips',
                default_value='',
                description='Comma-separated list of IP adresses (optional).'
                ' If provided, these will override the robot_ip values in the config file.',
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
