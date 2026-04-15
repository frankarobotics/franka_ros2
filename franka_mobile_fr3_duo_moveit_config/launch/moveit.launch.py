import os

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    Shutdown,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from franka_mobile_fr3_duo_moveit_config.description import get_robot_descriptions


PACKAGE_NAME = 'franka_mobile_fr3_duo_moveit_config'


def get_ros2_control_node(namespace, robot_description):
    ros2_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare(PACKAGE_NAME),
            'config',
            'mobile_fr3_duo_controllers.yaml',
        ]
    )

    return Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[
            {'robot_description': robot_description},
            {'robot_types': ['tmrv0_2', 'fr3v2', 'fr3v2']},
            {'robot_prefixes': ['', 'left', 'right']},
            ros2_controllers_path,
        ],
        remappings=[('joint_states', 'franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )


def activate_controller(controller_name, wait_time):
    """Return an ExecuteProcess that retries activating a controller until it succeeds."""
    set_active = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name='ros2'),
                ' control set_controller_state ',
                controller_name,
                ' active',
            ]
        ],
        shell=True,
    )

    return TimerAction(period=wait_time, actions=[set_active])


def generate_nodes(context):
    """Generate infrastructure nodes (robot state, controllers, Gazebo)."""
    simulate_in_gazebo = LaunchConfiguration('simulate_in_gazebo').perform(context)
    simulate_in_gazebo_bool = simulate_in_gazebo == 'true' or simulate_in_gazebo == 'True'

    robot_name = 'mobile_fr3_duo_v0_2'
    namespace = LaunchConfiguration('namespace', default='')

    robot_description, _ = get_robot_descriptions(robot_name, simulate_in_gazebo)

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='both',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[
                {
                    'source_list': ['franka/joint_states'],
                    'rate': 30,
                }
            ],
        ),
    ]

    if simulate_in_gazebo_bool:
        controller_names = [
            'joint_state_broadcaster',
            'swerve_ik_controller',
            'swerve_drive_controller',
            'full_body_controller',
        ]
    else:
        nodes += [get_ros2_control_node(namespace, robot_description)]
        controller_names = [
            'joint_state_broadcaster',
            'swerve_drive_controller',
            'full_body_controller',
        ]

    nodes += [
        activate_controller(name, wait_time)
        for (name, wait_time) in zip(controller_names, [3.0, 5.0, 10.0, 12.0])
    ]

    return nodes


def generate_launch_description():
    """Top-level launch: infrastructure nodes + included MoveIt nodes."""
    moveit_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(PACKAGE_NAME),
                'launch',
                'moveit_nodes.launch.py',
            )
        ),
        launch_arguments={
            'simulate_in_gazebo': LaunchConfiguration('simulate_in_gazebo'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'simulate_in_gazebo',
                default_value='false',
                description='Simulates the robot in Gazebo if true. ',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='Visualize the robot in rviz. ',
            ),
            OpaqueFunction(function=generate_nodes),
            moveit_nodes_launch,
        ],
    )
