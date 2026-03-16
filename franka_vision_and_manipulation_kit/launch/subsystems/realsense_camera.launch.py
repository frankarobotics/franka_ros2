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
            OpaqueFunction(function=launch_realsense_cameras),
        ]
    )


def launch_realsense_cameras(context):

    with open(context.perform_substitution(LaunchConfiguration('config_file_path'))) as f:
        config = yaml.safe_load(f)

    left_wrist_camera_config = {
        key: str(value) for (key, value) in config['left_arm']['camera'].items()
    }
    right_wrist_camera_config = {
        key: str(value) for (key, value) in config['right_arm']['camera'].items()
    }

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py',
                )
            ),
            launch_arguments=left_wrist_camera_config.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py',
                )
            ),
            launch_arguments=right_wrist_camera_config.items(),
        ),
    ]
