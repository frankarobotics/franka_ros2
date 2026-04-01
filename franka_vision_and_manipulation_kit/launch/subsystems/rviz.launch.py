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
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    pkg = get_package_share_directory('franka_vision_and_manipulation_kit')

    return LaunchDescription(
        [
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=[
                    '--display-config',
                    os.path.join(pkg, 'rviz', 'vision_and_manipulation_kit.rviz'),
                ],
            ),
        ]
    )
