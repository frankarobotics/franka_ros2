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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for franka_mobile_sensors."""
    pkg_dir = get_package_share_directory('franka_mobile_sensors')

    global_args = [
        DeclareLaunchArgument(
            'start_cameras',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to start RealSense camera drivers',
        ),
        DeclareLaunchArgument(
            'start_lidars',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to start SICK safety scanner drivers',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to start RViz visualization',
        ),
        DeclareLaunchArgument(
            'publish_sensor_tf',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to publish the sensor frames to TF when RViz is off '
                        '(with RViz, its robot_state_publisher publishes them)',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='default_sensor_suite',
            description='Configuration file to use (without .yaml extension)',
        ),
        DeclareLaunchArgument(
            'robot_xacro',
            default_value='tmrv0_2_with_sensors.urdf.xacro',
            description='XACRO file for robot with sensors',
        ),
    ]

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_dir, '/launch/cameras/realsense_cameras.launch.py']
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_cameras')),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_dir, '/launch/lidars/safety_scanners.launch.py']
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_lidars')),
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([pkg_dir, 'robots', LaunchConfiguration('robot_xacro')]),
        ]),
        value_type=str,
    )
    # The namespace keeps this URDF off /robot_description and the node off
    # /joint_states. The sensor tree starts at base_link, leaving base ->
    # base_link to the swerve odometry.
    sensor_tf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='sensors',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('publish_sensor_tf'), "' == 'true' and '",
            LaunchConfiguration('start_rviz'), "' == 'false'",
        ])),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_dir, '/launch/visualization/rviz.launch.py']
        ),
        launch_arguments={
            'robot_xacro': LaunchConfiguration('robot_xacro'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return LaunchDescription(
        [
            *global_args,
            camera_launch,
            lidar_launch,
            sensor_tf_publisher,
            rviz_launch,
        ]
    )


if __name__ == '__main__':
    generate_launch_description()
