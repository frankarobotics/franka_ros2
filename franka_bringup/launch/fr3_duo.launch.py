#  Copyright (c) 2026 Franka Robotics GmbH
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

############################################################################
# Parameters:
# robot_config_file: Configuration file name or path. If just a filename is
#                   provided (e.g., 'fr3_duo.config.yaml'), it will be
#                   looked up in franka_bringup/config/ directory.
#                   If provided, robot_ips, robot_types, and arm_prefixes
#                   will be read from this file. (default: 'fr3_duo.config.yaml')
# controller_name: Controller name to spawn (required). Only one controller is
#                 supported for duo setups.
# robot_types: Types of the robot arms as a string list (e.g., "['fr3','fr3']")
#             Required if robot_config_file is not provided.
# arm_prefixes: Prefixes for the arms as a string list (e.g., "['right','left']")
#              Required if robot_config_file is not provided.
# robot_ips: IP addresses of the robots as a string list (e.g., "['172.16.0.5','172.16.0.6']")
#           Required if robot_config_file is not provided.
# load_gripper: Use Franka Gripper as end-effector (default: 'false')
# use_fake_hardware: Use fake hardware (default: 'false')
# fake_sensor_commands: Fake sensor commands (default: 'false')
# is_async: Use async hardware interface (default: 'true')
# joint_state_rate: Rate for joint state publishing in Hz (default: '30')
# namespace: Namespace for the robot (default: '')
# use_rviz: Launch RViz for the robot (default: 'true')
# check_selfcollision: Launch self_collision_node for the robot (default: 'false')
# thread_priority: Thread priority for the hardware interface (default: '50')
#
# The fr3_duo.launch.py launch file provides a robust interface for launching
# a Franka Robotics dual-arm setup. It generates the robot description from the
# duo URDF xacro file and launches the necessary nodes for controlling both arms.
#
# Usage examples:
# 1. Launch with config file and controller:
#    ros2 launch franka_bringup fr3_duo.launch.py \
#      controller_name:=fr3_duo_joint_impedance_example_controller
#
# 2. Launch with parameters:
#    ros2 launch franka_bringup fr3_duo.launch.py \
#      robot_types:="['fr3','fr3']" \
#      robot_ips:="['172.16.0.5','172.16.0.6']" \
#      arm_prefixes:="['right','left']"
#
# NOTE: The franka_robot_state_broadcaster is NOT launched for duo setups as it
# is not supported for multi-arm configurations.
############################################################################

import os
import sys

from ament_index_python.packages import get_package_share_directory

import franka_bringup.launch_utils as launch_utils

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

package_share = get_package_share_directory('franka_bringup')

# constant for the controller name parameter
CONTROLLER_EXAMPLE = 'controller'

load_yaml = launch_utils.load_yaml
parse_string_list = launch_utils.parse_string_list
validate_duo_arrays_length = launch_utils.validate_duo_arrays_length
validate_arm_prefixes_unique = launch_utils.validate_arm_prefixes_unique
is_duo_config = launch_utils.is_duo_config


def generate_robot_nodes(context):
    robot_config_file = LaunchConfiguration(
        'robot_config_file').perform(context)

    # If config_file is just a filename (no path separators), look in
    # franka_bringup/config/
    if not os.path.isabs(
            robot_config_file) and os.path.sep not in robot_config_file:
        robot_config_file = os.path.join(
            package_share, 'config', robot_config_file)

    # Load configuration from file
    configs = load_yaml(robot_config_file)
    # Get the first config (assuming single duo config in file)
    config = next(iter(configs.values()))

    # Validate it's a duo config
    if not is_duo_config(config):
        print(
            f'Error: Configuration file {
                robot_config_file} does not contain a duo configuration.\n'
            f'Expected keys: robot_types, robot_ips, arm_prefixes\n'
            f'For single robot configurations, use example.launch.py instead.'
        )
        sys.exit(1)

    # Extract parameters from config
    robot_ips_str = str(config['robot_ips'])
    robot_types_str = str(config['robot_types'])
    arm_prefixes_str = str(config['arm_prefixes'])
    use_fake_hardware_str = str(config.get('use_fake_hardware', 'false'))
    fake_sensor_commands_str = str(config.get('fake_sensor_commands', 'false'))
    load_gripper_str = str(config.get('load_gripper', 'false'))
    namespace = str(config.get('namespace', ''))
    joint_state_rate = int(config.get('joint_state_rate', 30))
    use_rviz = str(config.get('use_rviz', 'true')).lower() == 'true'
    check_selfcollision = str(config.get('check_selfcollision', 'false')).lower() == 'true'
    thread_priority_str = str(config.get('thread_priority', 50))

    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context)
    # Parse string list representations into actual Python lists for
    # ros2_control_node
    robot_types_list = parse_string_list(robot_types_str)
    robot_ips_list = parse_string_list(robot_ips_str)
    arm_prefixes_list = parse_string_list(arm_prefixes_str)

    # Validate duo configuration
    validate_duo_arrays_length(
        robot_types_list,
        robot_ips_list,
        arm_prefixes_list)
    validate_arm_prefixes_unique(arm_prefixes_list)

    # Build URDF path
    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare('franka_description'),
            'robots',
            'fr3_duo',
            'fr3_duo.urdf.xacro',
        ]
    ).perform(context)

    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            'ros2_control': 'true',
            'robot_types': robot_types_str,
            'robot_ips': robot_ips_str,
            'hand': load_gripper_str,
            'use_fake_hardware': use_fake_hardware_str,
            'fake_sensor_commands': fake_sensor_commands_str,
            'is_async': 'true',
            'thread_priority': thread_priority_str,
            'arm_prefixes': arm_prefixes_str,
        },
    ).toprettyxml(indent='  ')

    # Build SRDF path
    srdf_path = PathJoinSubstitution(
        [
            FindPackageShare('franka_description'),
            'robots',
            'fr3_duo',
            'fr3_duo.srdf.xacro',
        ]
    ).perform(context)

    robot_description_semantic = xacro.process_file(
        srdf_path,
        mappings={
            'robot_types': robot_types_str,
            'arm_prefixes': arm_prefixes_str,
            'hand': load_gripper_str,
        }
    ).toprettyxml(indent='  ')

    joint_state_publisher_sources = [
        'franka/joint_states',
        'franka_gripper/joint_states',
    ]

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[
                controllers_yaml,
                {'robot_description': robot_description},
                {'robot_types': robot_types_list},
                {'arm_prefixes': arm_prefixes_list},
            ],
            remappings=[('joint_states', 'franka/joint_states')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[
                {
                    'source_list': joint_state_publisher_sources,
                    'rate': joint_state_rate,
                }
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        # NOTE: franka_robot_state_broadcaster is NOT launched for duo setups
        # as it is not supported for multi-arm configurations.
        # TODO: Gripper feature is not implemented yet.
    ]

    # Spawn controller
    controller_name = LaunchConfiguration('controller_name').perform(context)
    if not controller_name:
        print(
            'Error: No controller name provided. Please provide a controller name.'
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
                    '30'],
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
    if use_rviz:
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
    if check_selfcollision:
        nodes.append(
            Node(
                package='franka_selfcollision',
                executable='self_collision_node',
                name='self_collision_node',
                namespace=namespace,
                parameters=[
                    {
                        'robot_description_semantic': robot_description_semantic,
                    }
                ],
            )
        )
    return nodes


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('franka_bringup'),
                 'config', 'fr3_duo.config.yaml']
            ),
            description='Config file name (looked up in franka_bringup/config/) or full path. '
            'If provided, robot_ips, robot_types, and arm_prefixes will be read from this file.',
        ),
        DeclareLaunchArgument(
            'controllers_yaml',
            default_value=PathJoinSubstitution(
                [FindPackageShare('franka_bringup'),
                 'config', 'controllers.yaml']
            ),
            description='Override the default controllers.yaml file',
        ),
        DeclareLaunchArgument(
            'controller_name',
            description='Controller name to spawn (required). '
                        'Only one controller is supported for duo setups.',
        ),
    ]

    return LaunchDescription(
        launch_args + [OpaqueFunction(function=generate_robot_nodes)]
    )
