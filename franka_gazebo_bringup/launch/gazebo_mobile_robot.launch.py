# Copyright (c) 2026 Franka Robotics GmbH
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
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


NAMESPACE = 'mobile_base'
CONTROLLER = 'mobile_cartesian_velocity_with_ik_example_controller'


def set_gz_sim_resource_path(context, with_sensors):
    with_sensors_val = context.perform_substitution(with_sensors).lower()
    description_share = os.path.dirname(get_package_share_directory('franka_description'))
    if with_sensors_val == 'true':
        sensors_share = os.path.dirname(get_package_share_directory('franka_mobile_sensors'))
        oliv_module_descriptions_share = os.path.dirname(get_package_share_directory('olv_module_descriptions'))
        gz_sim_resource_path = f"{sensors_share}:{description_share}:{oliv_module_descriptions_share}"
    else:
        gz_sim_resource_path = description_share
    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_sim_resource_path)]


def launch_all(context: LaunchContext, with_sensors, world):
    """Launch Gazebo, spawn TMR, bridge sensor data, load controllers."""
    with_sensors_val = context.perform_substitution(with_sensors).lower()
    world_val = context.perform_substitution(world).strip()
    gazebo_bringup_share = get_package_share_directory('franka_gazebo_bringup')

    if world_val:
        gz_world = os.path.join(gazebo_bringup_share, 'worlds', world_val)
    else:
        gz_world = os.path.join(
            gazebo_bringup_share, 'worlds', 'mobile_fr3_duo_sensors.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {gz_world}'}.items(),
    )

    if with_sensors_val == 'true':
        xacro_file = os.path.join(
            gazebo_bringup_share, 'urdf',
            'tmrv0_2_with_sensors.gazebo.urdf.xacro')
    else:
        xacro_file = os.path.join(
            gazebo_bringup_share, 'urdf',
            'tmrv0_2.gazebo.urdf.xacro')

    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={
            'robot_type': 'tmrv0_2',
            'gazebo': 'true',
            'robot_namespace': NAMESPACE,
        }
    ).toxml()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=NAMESPACE,
        parameters=[{'robot_description': robot_description_xml}],
        remappings=[('/joint_states', f'/{NAMESPACE}/joint_states')],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-topic', f'/{NAMESPACE}/robot_description',
            '-name', NAMESPACE,
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )

    load_jsb = Node(
        package='controller_manager', executable='spawner',
        namespace=NAMESPACE,
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', f'/{NAMESPACE}/controller_manager',
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '60',
        ],
        output='screen',
    )

    load_ctrl = Node(
        package='controller_manager', executable='spawner',
        namespace=NAMESPACE,
        arguments=[
            CONTROLLER,
            '--controller-manager', f'/{NAMESPACE}/controller_manager',
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '60',
        ],
        output='screen',
    )

    if with_sensors_val == 'true':
        bridge_args = [
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera_front/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_front/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_rear/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_rear/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_left/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera_right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_right/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/lidar_front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar_rear/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ]
        remappings = [
            ('/camera_front/image_raw',             f'/{NAMESPACE}/camera_front/color/image_raw'),
            ('/camera_front/image_raw/camera_info', f'/{NAMESPACE}/camera_front/color/camera_info'),
            ('/camera_rear/image_raw',              f'/{NAMESPACE}/camera_rear/color/image_raw'),
            ('/camera_rear/image_raw/camera_info',  f'/{NAMESPACE}/camera_rear/color/camera_info'),
            ('/camera_left/image_raw',              f'/{NAMESPACE}/camera_left/color/image_raw'),
            ('/camera_left/image_raw/camera_info',  f'/{NAMESPACE}/camera_left/color/camera_info'),
            ('/camera_right/image_raw',             f'/{NAMESPACE}/camera_right/color/image_raw'),
            ('/camera_right/image_raw/camera_info', f'/{NAMESPACE}/camera_right/color/camera_info'),
            ('/lidar_front/scan',                   f'/{NAMESPACE}/lidar_front/scan'),
            ('/lidar_rear/scan',                    f'/{NAMESPACE}/lidar_rear/scan'),
            ('/imu/data',                           f'/{NAMESPACE}/imu/data'),
        ]
    else:
        bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
        remappings = []

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=remappings,
        output='screen',
    )

    return [
        gazebo,
        bridge,
        rsp_node,
        spawn,
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=spawn, on_exit=[load_jsb])),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=load_jsb, on_exit=[load_ctrl])),
    ]


def generate_launch_description():
    with_sensors = LaunchConfiguration('with_sensors')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '--display-config',
            PathJoinSubstitution([
                FindPackageShare('franka_description'),
                'rviz', 'visualize_franka.rviz',
            ]),
        ],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'with_sensors', default_value='false',
            description='If true, use sensor-enhanced TMR description'),
        DeclareLaunchArgument(
            'world', default_value='',
            description='SDF world filename inside franka_gazebo_bringup/worlds/'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='If true, launch RViz2'),
        OpaqueFunction(function=set_gz_sim_resource_path, args=[with_sensors]),
        OpaqueFunction(function=launch_all, args=[with_sensors, world]),
        rviz_node,
    ])
