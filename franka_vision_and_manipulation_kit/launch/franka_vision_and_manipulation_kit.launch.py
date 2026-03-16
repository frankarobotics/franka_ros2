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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description() -> LaunchDescription:

    launch_args = [
        DeclareLaunchArgument(
            'start_robotiq_grippers',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to start Robotiq gripper drivers',
        ),
        DeclareLaunchArgument(
            'start_realsense_cameras',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to start RealSense camera drivers',
        ),
        DeclareLaunchArgument(
            'start_zed_camera',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to start ZED camera drivers',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to start RViz visualization',
        ),
        DeclareLaunchArgument(
            'use_mobile_platform',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to use FR3 duo or its mobile variant',
        ),
        DeclareLaunchArgument(
            'config_file_path',
            default_value=(
                'package://franka_vision_and_manipulation_kit/config/'
                'default_config.yaml',
            ),
            description=('Configuration file to use. Either a file path or a ROS package resource '
                         '(e.g. package://my_package/config.yaml)'),
        ),
        DeclareLaunchArgument(
            'xacro_file_path',
            default_value=(
                'package://franka_vision_and_manipulation_kit/robots/'
                'vision_and_manipulation_kit.urdf.xacro',
            ),
            description=('Robot xacro file to use. Either a file path or a ROS package resource '
                         '(e.g. package://my_package/robot.xacro)'),
        ),
    ]

    return LaunchDescription(
        [
            *launch_args,
            OpaqueFunction(function=launch_vision_and_manipulation_kit),
        ]
    )


def launch_vision_and_manipulation_kit(context: LaunchContext) -> list:

    pkg = get_package_share_directory('franka_vision_and_manipulation_kit')

    xacro_mappings = {
        'mobile_platform': context.perform_substitution(LaunchConfiguration('use_mobile_platform'))
    }

    robot_description_string = xacro.process_file(
        resolve_path(context.perform_substitution(LaunchConfiguration('xacro_file_path'))),
        mappings=xacro_mappings,
    ).toprettyxml(indent='  ')

    launch_args = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_string}],
        ),
    ]

    config_file_path = resolve_path(
        context.perform_substitution(LaunchConfiguration('config_file_path'))
    )

    if context.perform_substitution(LaunchConfiguration('start_robotiq_grippers')) == 'true':
        launch_args.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'subsystems', 'robotiq.launch.py')
                ),
                launch_arguments={'config_file_path': config_file_path}.items(),
            )
        )

    if context.perform_substitution(LaunchConfiguration('start_rviz')) == 'true':
        launch_args.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'subsystems', 'rviz.launch.py')
                )
            )
        )

    if context.perform_substitution(LaunchConfiguration('start_realsense_cameras')) == 'true':
        launch_args.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'subsystems', 'realsense_camera.launch.py')
                ),
                launch_arguments={'config_file_path': config_file_path}.items(),
            )
        )

    if context.perform_substitution(LaunchConfiguration('start_zed_camera')) == 'true':
        launch_args.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'subsystems', 'zed_camera.launch.py')
                ),
                launch_arguments={'config_file_path': config_file_path}.items(),
            )
        )

    return launch_args


def resolve_path(path: str) -> str:

    if path.startswith('package://'):
        resolved_path = path.replace('package://', '')
        package_name = resolved_path.split('/')[0]
        path_inside_package = resolved_path.split('/')[1:]

        resolved_path = os.path.join(
            get_package_share_directory(package_name), *path_inside_package
        )

        return resolved_path

    return path
