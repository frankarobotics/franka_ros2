import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    Shutdown,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from franka_mobile_fr3_duo_moveit_config.description import get_robot_descriptions


PACKAGE_NAME = "franka_mobile_fr3_duo_moveit_config"


def set_gz_sim_resource_path(context):
    description_share = os.path.dirname(
        get_package_share_directory("franka_description")
    )
    os.environ["GZ_SIM_RESOURCE_PATH"] = description_share
    return []


def get_gz_world(context):
    """Return the Gazebo world include action."""
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": "empty.sdf -r"}.items(),
        )
    ]


def gazebo_nodes():
    """Return the list of actions needed to start Gazebo simulation."""
    return [
        OpaqueFunction(function=set_gz_sim_resource_path),
        OpaqueFunction(function=get_gz_world),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
    ]

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

def get_franka_robot_state_broadcaster(use_fake_hardware, namespace):
    return Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['franka_robot_state_broadcaster'],
            condition=UnlessCondition(use_fake_hardware),
            output='screen',
        )

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
            "--controller-manager-timeout", "120",
            "--service-call-timeout", "60",
        ],
        parameters=full_body_controller_node_parameters,
        output="screen",
    )

    return [joint_state_broadcaster_node, full_body_controller_node]


def generate_nodes(context):
    """Generate infrastructure nodes (robot state, controllers, Gazebo)."""
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    simulate_in_gazebo = LaunchConfiguration("simulate_in_gazebo").perform(context)
    simulate_in_gazebo_bool = simulate_in_gazebo == "true" or simulate_in_gazebo == "True"

    robot_name = "mobile_fr3_duo_v0_2"
    namespace = LaunchConfiguration("namespace", default="")

    robot_description, _ = get_robot_descriptions(
        robot_name, use_fake_hardware, simulate_in_gazebo
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            output="both",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            namespace=namespace,
            parameters=[
                {
                    "source_list": ["franka/joint_states"],
                    "rate": 30,
                }
            ],
        ),
    ] + get_controller_nodes(simulate_in_gazebo_bool, namespace)

    if simulate_in_gazebo_bool:
        nodes += gazebo_nodes()
    else:
        nodes += [get_franka_robot_state_broadcaster(use_fake_hardware, namespace), get_ros2_control_node(namespace, robot_description)]

    return nodes


def generate_launch_description():
    """Top-level launch: infrastructure nodes + included MoveIt nodes."""
    moveit_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(PACKAGE_NAME),
                "launch",
                "moveit_nodes.launch.py",
            )
        ),
        launch_arguments={
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "simulate_in_gazebo": LaunchConfiguration("simulate_in_gazebo"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    return LaunchDescription(
        [
                DeclareLaunchArgument(
                    "use_fake_hardware",
                    default_value='false',
                    description='Fakes the hardware if true. If false, the real hardware is expected to be connected.',
                ),
                DeclareLaunchArgument(
                    "simulate_in_gazebo",
                    default_value='false',
                    description='Simulates the robot in Gazebo if true. ',
                ),
                DeclareLaunchArgument(
                    "rviz",
                    default_value='true',
                    description='Visualize the robot in rviz. ',
                ),
            OpaqueFunction(function=generate_nodes),
            moveit_nodes_launch
        ],
    )
