# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(context: LaunchContext, robot_type, load_gripper, franka_hand):
    robot_type_str = context.perform_substitution(robot_type)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        robot_type_str,
        f'{robot_type_str}.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'robot_type': robot_type_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    return [robot_state_publisher]


def generate_launch_description():
    # Configure ROS nodes for launch
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    robot_type_name = 'robot_type'
    namespace_name = 'namespace'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    robot_type = LaunchConfiguration(robot_type_name)
    namespace = LaunchConfiguration(namespace_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value='false',
        description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name,
        default_value='franka_hand',
        description='Default value: franka_hand')
    robot_type_launch_argument = DeclareLaunchArgument(
        robot_type_name,
        default_value='fr3',
        description='Available values: fr3, fp3 and fer')
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the root namespace.')

    # Get robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[robot_type, load_gripper, franka_hand])

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        arguments=['--display-config', rviz_file, '-f', 'base_link'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        robot_type_launch_argument,
        namespace_launch_argument,
        gazebo_empty_world,
        robot_state_publisher,
        rviz,
        spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[
                {'source_list': ['joint_states'],
                 'rate': 30}],
        ),
    ])
