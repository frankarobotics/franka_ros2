# Copyright (c) 2026 Franka Robotics GmbH
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

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_robot_description(context: LaunchContext, load_gripper, franka_hand, with_sensors):
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)
    with_sensors_val = context.perform_substitution(with_sensors).lower()

    if with_sensors_val == 'true':
        franka_xacro_file = os.path.join(
            get_package_share_directory('franka_gazebo_bringup'),
            'urdf',
            'mobile_fr3_duo_v0_2_with_sensors.gazebo.urdf.xacro'
        )
    else:
        franka_xacro_file = os.path.join(
            get_package_share_directory('franka_gazebo_bringup'),
            'urdf',
            'mobile_fr3_duo_v0_2.gazebo.urdf.xacro'
        )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'robot_types': "['tmrv0_2', 'fr3v2', 'fr3v2']",
            'hand': load_gripper_str,
            'ee_id': franka_hand_str,
            'ros2_control': 'true',
            'gazebo_effort': 'true'
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


def set_gz_sim_resource_path(context, with_sensors):
    with_sensors_val = context.perform_substitution(with_sensors).lower()
    if with_sensors_val == 'true':
        sensors_share = os.path.dirname(get_package_share_directory('franka_mobile_sensors'))
        description_share = os.path.dirname(get_package_share_directory('franka_description'))
        olv_module_descriptions_share = os.path.dirname(
            get_package_share_directory('olv_module_descriptions'))
        os.environ['GZ_SIM_RESOURCE_PATH'] = f"{sensors_share}:{
            description_share}:{olv_module_descriptions_share}"
    else:
        description_share = os.path.dirname(get_package_share_directory('franka_description'))
        os.environ['GZ_SIM_RESOURCE_PATH'] = description_share
    return []


def get_gz_world(context: LaunchContext, with_sensors, world, gz_args):
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    with_sensors_val = context.perform_substitution(with_sensors).lower()
    world_val = context.perform_substitution(world).strip()
    gz_args_val = context.perform_substitution(gz_args)

    if world_val:
        world_path = os.path.join(
            get_package_share_directory('franka_gazebo_bringup'),
            'worlds', world_val)
    elif with_sensors_val == 'true':
        world_path = os.path.join(
            get_package_share_directory('franka_gazebo_bringup'),
            'worlds', 'mobile_fr3_duo_sensors.sdf')
    else:
        world_path = 'empty.sdf'

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world_path} {gz_args_val}'}.items(),
    )]


def get_bridge(context, with_sensors):
    with_sensors_val = context.perform_substitution(with_sensors).lower()

    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    remappings = []

    if with_sensors_val == 'true':
        bridge_args.extend([
            # Cameras
            '/camera_front/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_front/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_rear/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_rear/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_left/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_right/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # LiDARs
            '/lidar_front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar_rear/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ])
        remappings.extend([
            ('/camera_front/image_raw', '/camera_front/color/image_raw'),
            ('/camera_front/image_raw/camera_info', '/camera_front/color/camera_info'),
            ('/camera_rear/image_raw', '/camera_rear/color/image_raw'),
            ('/camera_rear/image_raw/camera_info', '/camera_rear/color/camera_info'),
            ('/camera_left/image_raw', '/camera_left/color/image_raw'),
            ('/camera_left/image_raw/camera_info', '/camera_left/color/camera_info'),
            ('/camera_right/image_raw', '/camera_right/color/image_raw'),
            ('/camera_right/image_raw/camera_info', '/camera_right/color/camera_info'),
        ])

    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=remappings,
        output='screen'
    )]


def generate_launch_description():
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    namespace_name = 'namespace'
    with_sensors_name = 'with_sensors'
    world_name = 'world'
    rviz_name = 'rviz'
    gz_args_name = 'gz_args'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    namespace = LaunchConfiguration(namespace_name)
    with_sensors = LaunchConfiguration(with_sensors_name)
    world = LaunchConfiguration(world_name)
    rviz = LaunchConfiguration(rviz_name)
    gz_args = LaunchConfiguration(gz_args_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value='true',
        description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name,
        default_value='franka_hand',
        description='Default value: franka_hand')
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the root namespace.')
    with_sensors_launch_argument = DeclareLaunchArgument(
        with_sensors_name,
        default_value='false',
        description='If true, use sensor-enhanced description package (franka_mobile_sensors)')
    world_launch_argument = DeclareLaunchArgument(
        world_name,
        default_value='',
        description='SDF world filename inside franka_gazebo_bringup/worlds/ to load. '
                    'Overrides the default world selection. '
                    'Example: sensor_demo_world.sdf')
    gz_args_launch_argument = DeclareLaunchArgument(
        gz_args_name,
        default_value='-r',
        description='Extra args to be forwared to gazebo')
    rviz_launch_argument = DeclareLaunchArgument(
        rviz_name,
        default_value='true',
        description='true/false for visualizing the robot in rviz')

    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[load_gripper, franka_hand, with_sensors])

    set_gz_sim_resource_path_action = OpaqueFunction(
        function=set_gz_sim_resource_path, args=[with_sensors])
    gazebo_world = OpaqueFunction(function=get_gz_world, args=[with_sensors, world, gz_args])
    bridge = OpaqueFunction(function=get_bridge, args=[with_sensors])

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description',
                   '-x', '0', '-y', '0', '-z', '0.05'],
        output='screen',
    )

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz_node = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                namespace=namespace,
                arguments=['--display-config', rviz_file, '-f', 'base_link'],
                condition=IfCondition(rviz))

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '120',
                   '--service-call-timeout', '60'],
        output='screen',
    )

    mobile_fr3_duo_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mobile_fr3_duo_joint_impedance_example_controller',
                   '--controller-manager-timeout', '120',
                   '--service-call-timeout', '60'],
        parameters=[PathJoinSubstitution([
            FindPackageShare('franka_gazebo_bringup'),
            'config',
            'franka_gazebo_controllers.yaml'
        ])],
        output='screen',
    )

    # chain the ik controller to simulate master controller's ik
    swerve_ik_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_ik_controller"],
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        namespace_launch_argument,
        with_sensors_launch_argument,
        world_launch_argument,
        gz_args_launch_argument,
        rviz_launch_argument,
        set_gz_sim_resource_path_action,
        gazebo_world,
        robot_state_publisher,
        rviz_node,
        spawn,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[swerve_ik_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=swerve_ik_controller,
                on_exit=[mobile_fr3_duo_controller],
            )
        ),
    ])
