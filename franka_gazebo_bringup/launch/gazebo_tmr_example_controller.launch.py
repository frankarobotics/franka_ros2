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
import xml.dom.minidom

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

cmd_vel_node = """
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import TwistStamped

class StampedCmdVelPub(Node):
    def __init__(self):
        super().__init__('stamped_cmd_vel_pub')
        self.pub = self.create_publisher(TwistStamped, '/swerve_drive_controller/cmd_vel', 10)
        self.create_timer(0.01, self.cb)  # 100 Hz

    def cb(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.1
        msg.twist.angular.z = 0.1
        self.pub.publish(msg)

rclpy.init()
rclpy.spin(StampedCmdVelPub())
"""


def get_robot_description(context: LaunchContext):

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        'tmrv0_2',
        'tmrv0_2.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'ros2_control': 'true',
            'gazebo': 'true',
        }
    )

    if not isinstance(robot_description_config, xml.dom.minidom.Document):
        raise RuntimeError(
            f'The given xacro file {franka_xacro_file} is not a valid xml format.')

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
    namespace_name = 'namespace'
    rviz_name = 'rviz'
    gz_args_name = 'gz_args'

    namespace = LaunchConfiguration(namespace_name)
    rviz = LaunchConfiguration(rviz_name)
    gz_args = LaunchConfiguration(gz_args_name)

    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the root namespace.')
    gz_args_launch_argument = DeclareLaunchArgument(
        gz_args_name,
        default_value='empty.sdf -r',
        description='Extra args to be forwared to gazebo')
    rviz_launch_argument = DeclareLaunchArgument(
        rviz_name,
        default_value='true',
        description='true/false for visualizing the robot in rviz')

    # Get robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[])

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description'))
    gazebo_empty_world = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     namespace=namespace,
                     arguments=['--display-config',
                                rviz_file, '-f', 'world'],
                     condition=IfCondition(rviz))

    # Start of the control chain, a simple circular reference
    circle_reference_node = ExecuteProcess(
        cmd=['python3', '-c', cmd_vel_node], output='screen')

    # Let's use the cartesian velocity example controller
    mobile_cartesian_velocity_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'swerve_ik_controller',
            'swerve_drive_controller',
            '--controller-manager-timeout', '30',
            '--activate-as-group'
        ],
        parameters=[PathJoinSubstitution([
            FindPackageShare('franka_gazebo_bringup'),
            'config',
            'franka_gazebo_controllers.yaml'
        ])],
        output='screen',
    )

    return LaunchDescription([
        namespace_launch_argument,
        gz_args_launch_argument,
        rviz_launch_argument,
        gazebo_empty_world,
        robot_state_publisher,
        rviz_node,
        spawn,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[mobile_cartesian_velocity_controller_node],
            )
        ),
        circle_reference_node,
    ])
