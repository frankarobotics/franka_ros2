"""
sim_launch.py  —  Phase 4 (flat controller params fix)
=======================================================
Root cause of CONTROL_FAILED / "0 controllers":
  When a Python dict with nested values {'fr3_arm_controller': {'action_ns': ...}}
  is serialized to a ROS 2 temp params file by the launch system, the inner dict
  does NOT reliably become sub-namespace parameters in Humble.
  moveit_simple_controller_manager then finds controller_names=['fr3_arm_controller']
  but cannot read fr3_arm_controller.action_ns / type / joints → registers 0 controllers.

Fix: use FLAT DOT-NOTATION keys in a single inline dict — these are written as
  literal parameter names (e.g. 'fr3_arm_controller.action_ns') and are always
  read correctly regardless of YAML nesting behaviour.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

HOME = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]


def load_yaml(package_name: str, relative_path: str) -> dict:
    abs_path = os.path.join(
        get_package_share_directory(package_name), relative_path)
    try:
        with open(abs_path, 'r') as f:
            return yaml.safe_load(f) or {}
    except OSError:
        return {}


def generate_launch_description():
    pkg_ros_gz_sim  = FindPackageShare('ros_gz_sim')
    pkg_custom      = FindPackageShare('fr3_delivery_sim')
    pkg_franka_desc = FindPackageShare('franka_description')

    pkg_franka_gazebo_dir = get_package_share_directory('franka_gazebo_bringup')
    pkg_delivery_dir      = get_package_share_directory('fr3_delivery_sim')

    franka_share_dir = get_package_share_directory('franka_description')
    set_gz_env  = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(franka_share_dir))
    set_ign_env = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', os.path.dirname(franka_share_dir))

    # ── Full URDF with camera ─────────────────────────────────────────────────
    xacro_file = PathJoinSubstitution(
        [pkg_custom, 'urdf', 'fr3_vision_env.urdf.xacro'])
    full_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file,
        ' hand:=true', ' ros2_control:=true', ' gazebo:=true',
        ' use_fake_hardware:=false', ' fake_sensor_commands:=false',
    ])
    full_robot_description = {
        'robot_description': ParameterValue(full_description_content, value_type=str)
    }

    # ── Kinematic-only URDF for move_group ───────────────────────────────────
    franka_xacro = PathJoinSubstitution(
        [pkg_franka_desc, 'robots', 'fr3', 'fr3.urdf.xacro'])
    moveit_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', franka_xacro,
        ' ros2_control:=false', ' hand:=true', ' robot_type:=fr3',
        ' robot_ip:=dont-care', ' use_fake_hardware:=false',
        ' fake_sensor_commands:=false',
    ])
    moveit_robot_description = {
        'robot_description': ParameterValue(moveit_description_content, value_type=str)
    }

    # ── SRDF ─────────────────────────────────────────────────────────────────
    srdf_xacro = PathJoinSubstitution(
        [pkg_franka_desc, 'robots', 'fr3', 'fr3.srdf.xacro'])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command([PathJoinSubstitution([FindExecutable(name='xacro')]),
                     ' ', srdf_xacro, ' hand:=true']),
            value_type=str,
        )
    }

    # ── MoveIt YAML configs ───────────────────────────────────────────────────
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    ompl_yaml       = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')

    # ── MoveIt controller manager — exact pattern from franka moveit.launch.py ──
    # The official franka moveit.launch.py uses:
    #   moveit_simple_controllers_yaml = load_yaml('...', 'config/fr3_controllers.yaml')
    #   moveit_controllers = {
    #       'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
    #       'moveit_controller_manager': 'moveit_simple.../MoveItSimpleControllerManager',
    #   }
    # The controller config MUST be nested under 'moveit_simple_controller_manager'.
    # Flat/dot-notation keys do NOT work — the plugin reads from its sub-node namespace.
    #
    # We use only fr3_arm_controller (no gripper — not running in Gazebo).
    moveit_controllers = {
        'moveit_simple_controller_manager': {
            'controller_names': ['fr3_arm_controller'],
            'fr3_arm_controller': {
                'action_ns': 'follow_joint_trajectory',
                'type': 'FollowJointTrajectory',
                'default': True,
                'joints': [
                    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
                    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
                ],
            },
        },
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # ── Controller YAML file paths ────────────────────────────────────────────
    franka_gazebo_ctrl_yaml = os.path.join(
        pkg_franka_gazebo_dir, 'config', 'franka_gazebo_controllers.yaml')
    our_position_ctrl_yaml  = os.path.join(
        pkg_delivery_dir, 'config', 'fr3_gazebo_position_controller.yaml')
    gripper_ctrl_yaml = os.path.join(
        pkg_delivery_dir, 'config', 'fr3_gripper_controller.yaml')

    # ── Gazebo ────────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # ── Robot state publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[full_robot_description, {'use_sim_time': True}],
    )

    # ── Spawn robot at HOME ───────────────────────────────────────────────────
    spawn_args = [
        '-topic', 'robot_description', '-name', 'fr3_vision_system', '-z', '0.0']
    for joint, val in zip(JOINT_NAMES, HOME):
        spawn_args += ['-J', joint, str(val)]
    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=spawn_args, output='screen')

    # ── Topic + service bridge ────────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/world/empty/create@ros_gz_interfaces/srv/SpawnEntity',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen',
    )

    # ── Spawner 1 (5 s): joint_state_broadcaster ─────────────────────────────
    jsb_spawner = TimerAction(period=5.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--param-file', franka_gazebo_ctrl_yaml],
        output='screen',
    )])

    # ── Spawner 2 (12 s): fr3_arm_controller (position interface) ────────────
    arm_controller_spawner = TimerAction(period=12.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['fr3_arm_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-type',
                   'joint_trajectory_controller/JointTrajectoryController',
                   '--param-file', our_position_ctrl_yaml],
        output='screen',
    )])
    # After arm_controller_spawner, add:
    gripper_spawner = TimerAction(period=13.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['fr3_gripper',
                    '--controller-manager', '/controller_manager',
                    '--controller-type',
                    'position_controllers/GripperActionController',
                    '--param-file', gripper_ctrl_yaml],
        output='screen',
)])
    # ── MoveIt 2 move_group (delayed 18 s) ──────────────────────────────────
    # CRITICAL: move_group must start AFTER fr3_arm_controller is active.
    # moveit_simple_controller_manager creates a FollowJointTrajectoryControllerHandle
    # at plugin init time. If the action server doesn't exist at that moment,
    # the handle marks itself disconnected and NEVER retries — causing "0 controllers"
    # on every subsequent Execute call even after the controller is active.
    # fr3_arm_controller spawns at 12 s, so 18 s gives a 6 s safety margin.
    # OMPL config — must be nested under 'move_group' key, exactly as in the
    # official franka moveit.launch.py. Flat keys cause CHOMP to load instead.
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_yaml)

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,   # nested under 'move_group' key
            trajectory_execution,
            moveit_controllers,              # nested under 'moveit_simple_controller_manager'
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        parameters=[
            moveit_robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': True},
        ],
    )

    # ── Static TF: world → base ───────────────────────────────────────────────
    # The fr3 SRDF virtual_joint uses 'base' as its parent_frame (the fixed
    # world frame for MoveIt planning).  'base' is never published as a TF
    # frame by robot_state_publisher — only 'world' and 'fr3_link0' are.
    # Without this publisher:
    #   • move_group logs "base does not exist" on every planning scene update
    #   • MoveIt's interactive goal markers in RViz can't anchor → invisible
    #   • Collision objects can't be transformed into the planning frame
    # Identity transform makes 'base' == 'world' for all practical purposes.
    static_tf_world_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base'],
        output='screen',
    )

    # ── Object spawner (15 s) ─────────────────────────────────────────────────
    object_spawner = TimerAction(period=15.0, actions=[Node(
        package='fr3_delivery_sim',
        executable='object_spawner.py',
        name='object_spawner',
        output='screen',
    )])

    # Delay move_group and rviz until after fr3_arm_controller is active
    move_group = TimerAction(period=18.0, actions=[move_group_node])
    rviz_delayed = TimerAction(period=20.0, actions=[rviz])

    return LaunchDescription([
        set_ign_env, set_gz_env,
        gazebo, robot_state_publisher, spawn_entity, bridge,
        static_tf_world_base,
        jsb_spawner, arm_controller_spawner,gripper_spawner,
        move_group, rviz_delayed, object_spawner,
    ])