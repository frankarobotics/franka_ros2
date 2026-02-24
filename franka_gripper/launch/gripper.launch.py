# Copyright (c) 2023 Franka Robotics GmbH
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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_robot_nodes(context):
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    # Declare the launch argument names
    default_joint_name_postfix = '_finger_joint'
    joint_names_1 = robot_type + default_joint_name_postfix + '1'
    joint_names_2 = robot_type + default_joint_name_postfix + '2'
    joint_names = [joint_names_1, joint_names_2]

    nodes = []

    # Load the gripper configuration file
    gripper_config = os.path.join(
        get_package_share_directory('franka_gripper'), 'config', 'franka_gripper_node.yaml'
    )
    nodes.append(
        Node(
                package='franka_gripper',
                executable='franka_gripper_node',
                name=['franka_gripper'],
                namespace=namespace,
                parameters=[{'robot_ip': robot_ip, 'joint_names': joint_names}, gripper_config],
                condition=UnlessCondition(use_fake_hardware),
        )
    )
    nodes.append(
        Node(
            package='franka_gripper',
            executable='fake_gripper_state_publisher.py',
            name=['franka_gripper'],
            namespace=namespace,
            parameters=[{'robot_ip': robot_ip, 'joint_names': joint_names}, gripper_config],
            condition=IfCondition(use_fake_hardware),
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_ip',
                description='Hostname or IP address of the robot.'
            ),
            DeclareLaunchArgument(
                'use_fake_hardware',
                default_value='false',
                description=(
                    'Publish fake gripper joint states without connecting to a real gripper'
                ),
            ),
            DeclareLaunchArgument(
                'robot_type',
                default_value='fr3',
                description=(
                    'Name of the arm in the URDF file. This is used to generate the joint '
                    'names of the gripper.'
                ),
            ),
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description=(
                    'Namespace for the gripper nodes. If not set, the nodes will not be '
                    'namespaced.'
                ),
            ),
            OpaqueFunction(function=generate_robot_nodes)
        ]
    )
