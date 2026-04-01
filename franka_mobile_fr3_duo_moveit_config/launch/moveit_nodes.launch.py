import os
import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory
import yaml
from franka_mobile_fr3_duo_moveit_config.description import get_robot_descriptions


PACKAGE_NAME = "franka_mobile_fr3_duo_moveit_config"

def load_config_yaml(file_name):
    """Load a YAML config file from this package's config directory."""
    config_yaml_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "config",
        file_name,
    )
    with open(config_yaml_path, "r") as file:
        return yaml.load(file, Loader=yaml.FullLoader)


def load_moveit_defaults():
    """Load moveit_defaults.json from this package's config directory."""
    moveit_defaults_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        "config",
        "moveit_defaults.json",
    )
    with open(moveit_defaults_path, "r") as file:
        return json.load(file)


def build_move_group_params(
    robot_description,
    robot_description_semantic,
    kinematics,
    joint_limits,
    trajectory_execution,
    moveit_defaults,
    simulate_in_gazebo,
):
    """Build the parameter list for the move_group node."""
    move_group_configuration = {
        "robot_description": robot_description,
        "publish_robot_description": True,
        "robot_description_kinematics": kinematics,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_planning": joint_limits,
    }

    extra_params = {
        "allow_trajectory_execution": True,
        "capabilities": "",
        "disable_capabilities": "",
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "use_sim_time": simulate_in_gazebo,
    }

    move_group_configuration.update(moveit_defaults)
    move_group_configuration.update(trajectory_execution)
    move_group_configuration.update(extra_params)

    return [move_group_configuration, extra_params]

def get_controller_nodes(simulate_in_gazebo, namespace):
    """Return the list of controller spawner nodes."""
    if simulate_in_gazebo:
        cartesian_velocity_interface_prefix = "swerve_ik_controller/"
    else:
        cartesian_velocity_interface_prefix = ""

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

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--inactive",
            "--controller-manager-timeout", "30",
            "--controller-ros-args",
            "--remap joint_states:=/franka/joint_states",
        ],
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(PACKAGE_NAME),
                    "config",
                    "mobile_fr3_duo_controllers.yaml",
                ]
            )
        ],
        output="screen",
    )

    full_body_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "full_body_controller",
            "--inactive",
            "--controller-manager-timeout", "120",
            "--service-call-timeout", "60",
        ],
        parameters=full_body_controller_node_parameters,
        output="screen",
    )

    return [joint_state_broadcaster_node, full_body_controller_node]

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

    kinematics = load_config_yaml("kinematics.yaml")
    joint_limits = load_config_yaml("joint_limits.yaml")
    trajectory_execution = load_config_yaml("moveit_controllers.yaml")
    moveit_defaults = load_moveit_defaults()

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=build_move_group_params(
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
            trajectory_execution,
            moveit_defaults,
            simulate_in_gazebo_bool,
        ),
    )

    nodes = [move_group_node] + get_controller_nodes(simulate_in_gazebo_bool, namespace)

    if rviz.lower() == "true":
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
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
            parameters=[
                {"planning_pipelines": moveit_defaults["planning_pipelines"]},
                {"default_planning_pipeline": moveit_defaults["default_planning_pipeline"]},
                {"robot_description_kinematics": kinematics},
                {"robot_description_planning": joint_limits},
                {"use_sim_time": simulate_in_gazebo_bool},
            ],
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
