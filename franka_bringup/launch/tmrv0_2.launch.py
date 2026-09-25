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
#                   provided (e.g., 'tmr.config.yaml'), it will be
#                   looked up in franka_bringup/config/ directory.
#                   (default: 'tmr.config.yaml')
# controller_name: Controller name to spawn (required).
# use_fake_hardware: Use fake hardware (default: 'false')
# fake_sensor_commands: Fake sensor commands (default: 'false')
# joint_state_rate: Rate for joint state publishing in Hz (default: '30')
# namespace: Namespace for the robot (default: '')
# use_rviz: Launch RViz for the robot (default: 'true')
#
# The tmrv0_2.launch.py launch file provides a self-contained interface for
# launching a Franka Robotics TMR v0.2 mobile base. It generates the robot
# description from the tmrv0_2.urdf.xacro file and launches the necessary nodes
# for controlling the mobile base.
#
# Usage examples:
# 1. Launch with config file and controller:
#    ros2 launch franka_bringup tmrv0_2.launch.py \
#      controller_name:=swerve_drive_controller
#
# 2. Launch with default config for mobile_teleop:
#    ros2 launch franka_bringup tmrv0_2.launch.py \
#      robot_config_file:=tmr.config.yaml \
#      controller_name:=swerve_drive_controller
#
# NOTE: The franka_robot_state_broadcaster is NOT launched for TMR robots
# as it is not supported.
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

load_yaml = launch_utils.load_yaml


def generate_robot_nodes(context):
    robot_config_file = LaunchConfiguration('robot_config_file').perform(context)

    # If config_file is just a filename (no path separators), look in
    # franka_bringup/config/
    if not os.path.isabs(robot_config_file) and os.path.sep not in robot_config_file:
        robot_config_file = os.path.join(package_share, 'config', robot_config_file)

    # Load configuration from file
    configs = load_yaml(robot_config_file)
    # Get the first config (assuming single TMR config in file)
    config = next(iter(configs.values()))

    # Extract parameters from config
    robot_type = str(config.get('robot_type', 'tmrv0_2'))
    arm_prefix = str(config.get('arm_prefix', ''))
    robot_ip = str(config.get('robot_ip', '172.16.16.10'))
    use_fake_hardware_str = str(config.get('use_fake_hardware', 'false'))
    fake_sensor_commands_str = str(config.get('fake_sensor_commands', 'false'))
    namespace = str(config.get('namespace', ''))
    joint_state_rate = int(config.get('joint_state_rate', 30))
    use_rviz = str(config.get('use_rviz', 'true')).lower() == 'true'

    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context)

    # Build URDF path
    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare('franka_bringup'),
            'urdf',
            'tmrv0_2.urdf.xacro',
        ]
    ).perform(context)

    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            'robot_type': robot_type,
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware_str,
            'fake_sensor_commands': fake_sensor_commands_str,
        },
    ).toprettyxml(indent='  ')

    joint_state_publisher_sources = [
        'franka/joint_states',
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
                {'robot_type': robot_type},
                {'arm_prefix': arm_prefix},
            ],
            remappings=[('joint_states', joint_state_publisher_sources[0])],
            output='screen',
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
                    'use_robot_description': False,
                }
            ],
            output='screen',
        ),
        # Disabled for the SGS demo, see franka.launch.py for the reasoning.
        # Re-enable it if you need /joint_states, TF, RViz or MoveIt for the base.
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     namespace=namespace,
        #     arguments=['joint_state_broadcaster'],
        #     output='screen',
        # ),
    ]

    # Spawn controller
    controller_name = LaunchConfiguration('controller_name').perform(context)
    if not controller_name:
        print('Error: No controller name provided. Please provide a controller name.')
        sys.exit(1)

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
                '--service-call-timeout',
                '60',
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

    return nodes


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'robot_config_file',
            default_value='tmr.config.yaml',
            description='Config file name (looked up in franka_bringup/config/) or full path.',
        ),
        DeclareLaunchArgument(
            'controllers_yaml',
            default_value=PathJoinSubstitution(
                [FindPackageShare('franka_bringup'), 'config', 'controllers.yaml']
            ),
            description='Override the default controllers.yaml file',
        ),
        DeclareLaunchArgument(
            'controller_name',
            description='Controller name to spawn (required).',
        ),
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=generate_robot_nodes)])
