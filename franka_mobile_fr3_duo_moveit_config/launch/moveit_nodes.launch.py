from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

from franka_mobile_fr3_duo_moveit_config.description import get_robot_descriptions
from franka_mobile_fr3_duo_moveit_config.parameters import (
    get_combined_parameters,
    get_rviz_parameters,
)


PACKAGE_NAME = 'franka_mobile_fr3_duo_moveit_config'


def get_controller_nodes(simulate_in_gazebo, namespace):
    """Return the list of controller spawner nodes."""
    if simulate_in_gazebo:
        cartesian_velocity_interface_prefix = "swerve_ik_controller/"
    else:
        cartesian_velocity_interface_prefix = "swerve_drive_controller/"

    full_body_controller_node_parameters = [
        PathJoinSubstitution(
            [
                FindPackageShare(PACKAGE_NAME),
                "config",
                "mobile_fr3_duo_controllers.yaml",
            ]
        ),
        {
            "full_body_controller": {
                "ros__parameters": {
                    "cartesian_velocity_interface_prefix": cartesian_velocity_interface_prefix,
                }
            }
        },
    ]

    if simulate_in_gazebo:
        spawn = Node(
            package="ros_gz_sim",
            executable="create",
            namespace=namespace,
            arguments=[
                "-topic", "/robot_description",
                "-x", "0", "-y", "0", "-z", "0.05",
            ],
            output="screen",
        )

        mobile_fr3_duo_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "swerve_ik_controller",
                "full_body_controller",
                "--inactive",
                "--controller-manager-timeout", "120",
                "--service-call-timeout", "60",
                "--controller-ros-args",
                "--remap joint_states:=/franka/joint_states",
            ],
            parameters=full_body_controller_node_parameters,
            output="screen",
        )

        return [
            spawn,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[mobile_fr3_duo_controller],
                )
            ),
        ]

    full_body_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            'swerve_drive_controller',
            "full_body_controller",
            "--inactive",
            "--controller-manager-timeout", "120",
            "--service-call-timeout", "60",
            "--controller-ros-args",
            "--remap joint_states:=/franka/joint_states",
        ],
        parameters=full_body_controller_node_parameters,
        output="screen",
    )

    return [full_body_controller_node]

def generate_moveit_nodes(context):
    """Generate MoveIt-specific nodes: move_group and optionally rviz."""
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    simulate_in_gazebo = LaunchConfiguration("simulate_in_gazebo").perform(context)
    simulate_in_gazebo_bool = simulate_in_gazebo.lower() == "true"
    rviz = LaunchConfiguration("rviz").perform(context)
    namespace = LaunchConfiguration("namespace", default="")

    robot_name = "mobile_fr3_duo_v0_2"

    robot_description, robot_description_semantic = get_robot_descriptions(
        robot_name, use_fake_hardware, simulate_in_gazebo
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output='screen',
        parameters=get_combined_parameters(
            robot_description,
            robot_description_semantic,
            simulate_in_gazebo_bool,
        ),
    )

    nodes = [move_group_node] + get_controller_nodes(simulate_in_gazebo_bool, namespace)

    if rviz.lower() == "true":
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            namespace=namespace,
            output="screen",
            respawn=False,
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare(PACKAGE_NAME),
                        "config",
                        "moveit.rviz",
                    ]
                ),
            ],
            parameters=get_rviz_parameters(simulate_in_gazebo_bool),
        )
        nodes.append(rviz_node)

    return nodes

def generate_launch_description():
    """Entry point for this MoveIt nodes launch file."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "simulate_in_gazebo",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
            ),
            OpaqueFunction(function=generate_moveit_nodes),
        ]
    )
