import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, Shutdown, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro
from ament_index_python.packages import get_package_share_directory
import yaml
import json

def set_gz_sim_resource_path(context):
    description_share = os.path.dirname(get_package_share_directory('franka_description'))
    os.environ['GZ_SIM_RESOURCE_PATH'] = description_share
    return []

def get_gz_world(context):
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_args = '-r'

    world_path = 'empty.sdf'

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world_path} {gz_args}'}.items(),
    )]

def get_bridge():

    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    remappings = []

    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=remappings,
        output='screen'
    )

def gazebo_nodes():
    set_gz_sim_resource_path_action = OpaqueFunction(
        function=set_gz_sim_resource_path)
    gazebo_world = OpaqueFunction(function=get_gz_world)
    bridge = get_bridge()

    return [
        set_gz_sim_resource_path_action,
        gazebo_world,
        bridge,
    ]


def get_robot_descriptions(robot_name, package_name, use_fake_hardware, simulate_in_gazebo):

    print(f"{simulate_in_gazebo=}")
    print(f"{use_fake_hardware=}")
    if simulate_in_gazebo == 'true':
        robot_description_file_path = os.path.join(
            get_package_share_directory('franka_gazebo_bringup'),
            'urdf',
            f'{robot_name}.gazebo.urdf.xacro'
        )
    else:
        robot_description_file_path = os.path.join(
            get_package_share_directory("franka_description"),
            "robots",
            f"{robot_name}",
            f"{robot_name}.urdf.xacro",
        )

    robot_description_semantic_file_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        f"{robot_name}.srdf",
    )
    robot_description = xacro.process_file(
        robot_description_file_path,
        mappings={
            "robot_ips": "['172.16.16.10', '172.16.16.12', '172.16.16.11']",
            "robot_prefixes": "['', 'left', 'right']",
            "robot_types": "['tmrv0_2', 'fr3v2', 'fr3v2']",
            "hand": "false",
            "load_gripper": "false",
            "ee_id": "None",
            "use_fake_hardware": use_fake_hardware,
            "ros2_control": "true",
            'gazebo_effort': simulate_in_gazebo,
            "fake_sensor_commands": "false",
            'is_async': 'true',
            'thread_priority': '97',
        },
    ).toxml()

    robot_description_semantic = xacro.process_file(
        robot_description_semantic_file_path,
        mappings={},
    ).toxml()

    return (robot_description, robot_description_semantic)


def get_joint_state_publisher(namespace):

    return Node(
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
    )


def get_ros2_control_node(namespace, robot_description, package_name):
    ros2_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
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


def get_robot_state_publisher(namespace, robot_description):
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[{"robot_description": robot_description}],
    )


def get_config_yaml(package_name, file_name):
    config_yaml_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        file_name,
    )

    with open(config_yaml_path, "r") as file:
        return yaml.load(file, Loader=yaml.FullLoader)


def get_move_group_params(
    robot_description,
    robot_description_semantic,
    kinematics,
    joint_limits,
    trajectory_execution,
    moveit_defaults,
):

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
    }

    move_group_configuration.update(moveit_defaults)
    move_group_configuration.update(trajectory_execution)
    move_group_configuration.update(extra_params)

    return [move_group_configuration, extra_params]


def get_move_group_node(
    namespace,
    robot_description,
    robot_description_semantic,
    kinematics,
    joint_limits,
    trajectory_execution,
    moveit_defaults,
):
    return Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=get_move_group_params(
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
            trajectory_execution,
            moveit_defaults,
        ),
    )


def get_controller_nodes(package_name, simulate_in_gazebo, namespace):
    if simulate_in_gazebo == 'true':
        cartesian_velocity_interface_prefix = 'swerve_ik_controller/'
    else:
        cartesian_velocity_interface_prefix = ''

    full_body_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["full_body_controller", '--controller-manager-timeout', '120',
                   '--service-call-timeout', '60'],
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
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
        ],
        output="screen",
    )

    joint_state_broadcaster_node = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", '--controller-manager-timeout', '30'],
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare(package_name),
                            'config',
                            'mobile_fr3_duo_controllers.yaml',
                        ]
                    )
                ],
                output="screen",
            )

    if simulate_in_gazebo == 'true':
        # chain the ik controller to simulate TMR ik
        swerve_ik_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["swerve_ik_controller"],
        )
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=['-topic', '/robot_description',
                    '-x', '0', '-y', '0', '-z', '0.05'],
            output='screen',
        )

        controller_nodes = [spawn,RegisterEventHandler(event_handler=OnProcessExit(target_action=spawn,on_exit=[joint_state_broadcaster_node],)),
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=spawn,
                        on_exit=[swerve_ik_controller],
                    )
                ),
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=swerve_ik_controller,
                        on_exit=[full_body_controller_node],
                    )
                ),
        ]
    else:
        controller_nodes = [joint_state_broadcaster_node, full_body_controller_node]

    return controller_nodes


def generate_nodes(context):
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    simulate_in_gazebo = LaunchConfiguration("simulate_in_gazebo").perform(context)

    robot_type = "mobile_fr3_duo"
    hardware_version = "v0_2"

    robot_name = f"{robot_type}_{hardware_version}"
    package_name = "franka_mobile_fr3_duo_moveit_config"

    namespace = LaunchConfiguration('namespace', default='')

    robot_description, robot_description_semantic = get_robot_descriptions(
        robot_name, package_name, use_fake_hardware, simulate_in_gazebo
    )

    joint_state_publisher = get_joint_state_publisher(namespace)
    robot_state_publisher = get_robot_state_publisher(
        namespace, robot_description
    )

    ros2_control_node = get_ros2_control_node(
        namespace, robot_description, package_name
    )

    kinematics = get_config_yaml(package_name, "kinematics.yaml")

    joint_limits = get_config_yaml(package_name, "joint_limits.yaml")

    trajectory_execution = get_config_yaml(
        package_name, "moveit_controllers.yaml"
    )

    moveit_defaults_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "moveit_defaults.json",
    )

    with open(moveit_defaults_path, "r") as file:
        moveit_defaults = json.load(file)

    rviz_parameters = [
        {"planning_pipelines": moveit_defaults["planning_pipelines"]},
        {
            "default_planning_pipeline": moveit_defaults[
                "default_planning_pipeline"
            ]
        },
        {"robot_description_kinematics": kinematics},
        {"robot_description_planning": joint_limits},
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        output='screen',
        respawn=False,
        arguments=[
            '-d',
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    'config',
                    'moveit.rviz',
                ]
            ),
        ],
        parameters=rviz_parameters,
    )

    move_group_node = get_move_group_node(
        namespace,
        robot_description,
        robot_description_semantic,
        kinematics,
        joint_limits,
        trajectory_execution,
        moveit_defaults,
    )

    controller_nodes = get_controller_nodes(package_name, simulate_in_gazebo, namespace)

    nodes = [
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_publisher,
            rviz_node,
        ] + controller_nodes
    
    if simulate_in_gazebo == 'true':
        nodes += gazebo_nodes()

    return nodes

def generate_launch_description():
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
            OpaqueFunction(function=generate_nodes)
        ]
    )
