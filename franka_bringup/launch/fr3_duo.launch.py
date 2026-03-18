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

parse_string_list = launch_utils.parse_string_list

validate_duo_arrays_length = launch_utils.validate_duo_arrays_length

validate_arm_prefixes_unique = launch_utils.validate_arm_prefixes_unique

is_duo_config = launch_utils.is_duo_config
 
 
def generate_robot_nodes(context):

    robot_config_file = LaunchConfiguration(

        'robot_config_file').perform(context)
 
    if not os.path.isabs(

            robot_config_file) and os.path.sep not in robot_config_file:

        robot_config_file = os.path.join(

            package_share, 'config', robot_config_file)
 
    configs = load_yaml(robot_config_file)

    config = next(iter(configs.values()))
 
    if not is_duo_config(config):

        print(

            f'Error: Configuration file {robot_config_file} does not contain a duo configuration.\n'

            f'Expected keys: robot_types, robot_ips, arm_prefixes\n'

            f'For single robot configurations, use example.launch.py instead.'

        )

        sys.exit(1)
 
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
 
    robot_types_list = parse_string_list(robot_types_str)

    robot_ips_list = parse_string_list(robot_ips_str)

    arm_prefixes_list = parse_string_list(arm_prefixes_str)
 
    validate_duo_arrays_length(robot_types_list, robot_ips_list, arm_prefixes_list)

    validate_arm_prefixes_unique(arm_prefixes_list)
 
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

            package='joint_state_publisher_gui',

            executable='joint_state_publisher_gui',

            name='joint_state_publisher_gui',

            namespace=namespace,

            output='screen',

        ),

    ]
 
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

            description='Config file name (looked up in franka_bringup/config/) or full path.',

        ),

    ]
 
    return LaunchDescription(

        launch_args + [OpaqueFunction(function=generate_robot_nodes)]

    )
 