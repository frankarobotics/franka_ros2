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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def generate_launch_description() -> LaunchDescription:

    launch_args = [
        DeclareLaunchArgument(
            'config_file_path',
        ),
    ]

    return LaunchDescription(
        [
            *launch_args,
            OpaqueFunction(function=launch_robotiq),
        ]
    )


def launch_robotiq(context):

    with open(context.perform_substitution(LaunchConfiguration('config_file_path'))) as f:
        config = yaml.safe_load(f)

    left_gripper_config = config['left_arm']['gripper']
    right_gripper_config = config['right_arm']['gripper']

    def get_robotiq_urdf(name):
        return os.path.join(
            get_package_share_directory('robotiq_description'),
            'urdf',
            f'robotiq_{name}_gripper.urdf.xacro',
        )

    controller_config_file_path = os.path.join(
        get_package_share_directory('franka_vision_and_manipulation_kit'),
        'config',
        'robotiq_controllers.yaml',
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('franka_vision_and_manipulation_kit'),
                    'launch',
                    'subsystems',
                    'robotiq_controller.launch.py',
                )
            ),
            launch_arguments={
                'model': get_robotiq_urdf(left_gripper_config.get('type', '2f_85')),
                'com_port': left_gripper_config.get('com_port', '/dev/ttyUSB0'),
                'use_fake_hardware': left_gripper_config.get('use_fake_hardware', 'false'),
                'launch_rviz': 'false',
                'namespace': 'left/gripper',
                'controller_file': controller_config_file_path,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('franka_vision_and_manipulation_kit'),
                    'launch',
                    'subsystems',
                    'robotiq_controller.launch.py',
                )
            ),
            launch_arguments={
                'model': get_robotiq_urdf(right_gripper_config.get('type', '2f_85')),
                'com_port': right_gripper_config.get('com_port', '/dev/ttyUSB1'),
                'use_fake_hardware': right_gripper_config.get('use_fake_hardware', 'false'),
                'launch_rviz': 'false',
                'namespace': 'right/gripper',
                'controller_file': controller_config_file_path,
            }.items(),
        ),
    ]
