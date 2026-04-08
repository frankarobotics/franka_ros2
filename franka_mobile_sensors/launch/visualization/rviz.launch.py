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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def spawn_robot_and_rviz(context: LaunchContext, *args, **kwargs):
    """Spawn robot_state_publisher, joint_state_publisher_gui, and RViz."""
    robot_xacro = context.perform_substitution(
        LaunchConfiguration('robot_xacro'))
    xacro_file = os.path.join(get_package_share_directory(
        'franka_mobile_sensors'), 'robots', robot_xacro)
    robot_description = xacro.process_file(xacro_file).toprettyxml(indent='  ')
    rviz_config = os.path.join(get_package_share_directory(
        'franka_mobile_sensors'), 'rviz', 'tmr_sensors.rviz')
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', rviz_config],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for RViz."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_xacro',
                default_value='tmrv0_2_with_sensors.urdf.xacro',
                description='XACRO file for robot with sensors',
            ),
            OpaqueFunction(function=spawn_robot_and_rviz),
        ]
    )


if __name__ == '__main__':
    generate_launch_description()
