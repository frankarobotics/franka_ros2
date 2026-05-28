"""
sim_launch.py — Phase 4 "Gazebo Physics" version
══════════════════════════════════════════════════
Architecture changes from the fake-hardware version:
  • gazebo:=true is passed to the XACRO, so franka_description injects the
    ign_ros2_control/IgnitionSystem hardware plugin + the Gazebo controller
    manager plugin automatically. No standalone ros2_control_node needed.
  • use_sim_time: True everywhere — Gazebo is now the physics clock.
  • Robot spawned with -J joint arguments so it starts in the HOME position
    (matching the joint values in the state machine) instead of falling.
  • YAML files loaded via load_yaml() (dict injection) to bypass Humble's
    strict ros__parameters parser bug.
  • The ign_ros2_control plugin inside Gazebo reads fr3_ros_controllers.yaml
    and spawns the controllers itself; we only need to wait for them.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


# ── HOME joint values (must match pick_and_place.py READY_POSITIONS) ─────────
# [j1, j2, j3, j4, j5, j6, j7]
HOME = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]


def load_yaml(package_name: str, relative_path: str) -> dict:
    """
    Load a MoveIt-schema YAML file as a plain Python dict.
    These files lack the ros__parameters: root key so they cannot be passed
    via --params-file without crashing Humble's rcl parser.
    """
    abs_path = os.path.join(
        get_package_share_directory(package_name), relative_path
    )
    try:
        with open(abs_path, 'r') as f:
            return yaml.safe_load(f) or {}
    except OSError:
        return {}


def generate_launch_description():
    pkg_ros_gz_sim  = FindPackageShare('ros_gz_sim')
    pkg_custom_sim  = FindPackageShare('fr3_delivery_sim')
    pkg_franka_desc = FindPackageShare('franka_description')

    moveit_config_dir = get_package_share_directory('franka_fr3_moveit_config')

    # ── Mesh resource paths ───────────────────────────────────────────────────
    franka_share    = get_package_share_directory('franka_description')
    realsense_share = get_package_share_directory('realsense2_description')
    model_paths = (
        os.path.dirname(franka_share) + ':' + os.path.dirname(realsense_share)
    )
    set_ign_env = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', model_paths)
    set_gz_env  = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',     model_paths)

    # ── Robot description ─────────────────────────────────────────────────────
    # gazebo:=true  → franka_description emits ign_ros2_control plugin + hardware block
    # ros2_control:=true → emits the <ros2_control> tag (required by the plugin)
    # use_fake_hardware:=false → real Gazebo physics simulation
    xacro_file = PathJoinSubstitution(
        [pkg_custom_sim, 'urdf', 'fr3_vision_env.urdf.xacro']
    )
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file,
        ' hand:=true',
        ' ros2_control:=true',
        ' use_fake_hardware:=false',
        ' fake_sensor_commands:=false',
        ' gazebo:=true',
        ' gazebo_effort:=false',
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ── Semantic description (SRDF) ───────────────────────────────────────────
    srdf_file = PathJoinSubstitution(
        [pkg_franka_desc, 'robots', 'fr3', 'fr3.srdf.xacro']
    )
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ', srdf_file, ' hand:=true',
            ]),
            value_type=str,
        )
    }

    # ── MoveIt config dicts (bypass rcl strict YAML parser) ───────────────────
    kinematics_raw = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    ompl_raw       = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_raw}

    # ── Gazebo Fortress ───────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # ── Robot state publisher (use_sim_time!) ─────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # ── Spawn robot into Gazebo with -J to set HOME joint positions ────────────
    # The -J arguments prevent gravity collapse on spawn.
    spawn_args = [
        '-topic', 'robot_description',
        '-name',  'fr3_vision_system',
        '-z',     '0.0',
    ]
    for name, val in zip(JOINT_NAMES, HOME):
        spawn_args += ['-J', name, str(val)]

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=spawn_args,
        output='screen',
    )

    # ── Controller spawners ───────────────────────────────────────────────────
    # Gazebo's ign_ros2_control plugin starts the controller manager.
    # We only need to tell it which controllers to activate.
    # Use TimerAction delays to give Gazebo time to fully load the plugin.
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            )
        ],
    )

    arm_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'fr3_arm_controller',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            )
        ],
    )

    # ── Topic bridge: Ignition ↔ ROS 2 ────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/world/empty/create@ros_gz_interfaces/srv/SpawnEntity',
            # Bridge clock so ROS nodes get Gazebo sim time
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen',
    )

    # ── MoveIt 2 move_group (use_sim_time!) ───────────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_raw,
            {'use_sim_time': True},
            {
                'move_group/planning_plugin':
                    'ompl_interface/OMPLPlanner',
                'move_group/request_adapters':
                    'default_planner_request_adapters/AddTimeOptimalParameterization '
                    'default_planner_request_adapters/FixWorkspaceBounds '
                    'default_planner_request_adapters/FixStartStateBounds '
                    'default_planner_request_adapters/FixStartStateCollision '
                    'default_planner_request_adapters/FixStartStatePathConstraints',
                'publish_planning_scene':     True,
                'publish_geometry_updates':   True,
                'publish_state_updates':      True,
                'publish_transforms_updates': True,
            },
        ],
    )

    # ── RViz 2 (use_sim_time!) ────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    # ── Phase 2: Object spawner (6 s delay — more time needed with physics) ───
    object_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='fr3_delivery_sim',
                executable='object_spawner.py',
                name='object_spawner',
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        set_ign_env,
        set_gz_env,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz,
        object_spawner,
    ])