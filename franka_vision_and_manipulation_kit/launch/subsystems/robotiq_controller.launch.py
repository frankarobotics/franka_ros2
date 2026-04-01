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

import launch
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
import launch_ros


def generate_launch_description():
    description_pkg_share = launch_ros.substitutions.FindPackageShare(
        package='robotiq_description'
    ).find('robotiq_description')

    default_model_path = os.path.join(
        description_pkg_share, 'urdf', 'robotiq_2f_85_gripper.urdf.xacro'
    )
    default_rviz_config_path = os.path.join(description_pkg_share, 'rviz', 'view_urdf.rviz')

    default_controllers_path = PathJoinSubstitution(
        [description_pkg_share, 'config', 'robotiq_controllers.yaml']
    )

    args = []
    args.append(
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to gripper URDF file',
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file',
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name='launch_rviz', default_value='false', description='Launch RViz?'
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name='com_port',
            default_value='/dev/ttyUSB0',
            description='Port for communicating with Robotiq hardware',
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name='namespace',
            default_value='',
            description='namespace of the nodes',
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name='controller_file',
            default_value=default_controllers_path,
            description='controller config file',
        )
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            LaunchConfiguration('model'),
            ' ',
            'use_fake_hardware:=false',
            ' ',
            'com_port:=',
            LaunchConfiguration('com_port'),
        ]
    )

    robot_description_param = {
        'robot_description': launch_ros.parameter_descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }

    update_rate_config_file = PathJoinSubstitution(
        [
            description_pkg_share,
            'config',
            'robotiq_update_rate.yaml',
        ]
    )

    control_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description_param,
            update_rate_config_file,
            LaunchConfiguration('controller_file'),
        ],
        namespace=LaunchConfiguration('namespace'),
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param],
        namespace=LaunchConfiguration('namespace'),
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        namespace=LaunchConfiguration('namespace'),
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
        ],
        namespace=LaunchConfiguration('namespace'),
    )

    robotiq_gripper_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_gripper_controller'],
        namespace=LaunchConfiguration('namespace'),
    )

    robotiq_activation_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_activation_controller'],
        namespace=LaunchConfiguration('namespace'),
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        robotiq_gripper_controller_spawner,
        robotiq_activation_controller_spawner,
        rviz_node,
    ]

    return launch.LaunchDescription(args + nodes)
