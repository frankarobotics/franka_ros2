#  Copyright (c) 2026 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

############################################################################
# SGS demo: both FR3 arms, the mobile base and the spine moving continuously.
#
# Launch arguments:
#   arms_config_file: Arm configuration (default: sgs_demo_arms.config.yaml)
#   base_config_file: Mobile base configuration (default: sgs_demo_base.config.yaml)
#   spine_ip: Address of the spine HTTP API (default: 172.16.16.10, the mobile base)
#   spine_namespace: Namespace for the spine server and its motion (default: spine)
#
# Each robot runs in its own namespace and therefore its own controller_manager, in its
# own process: /left_arm, /right_arm, /base and /spine. That is deliberate. A single
# controller commanding arms and base together puts all of them behind one 1 kHz loop, so
# whatever stalls that loop stalls every robot at once. Split like this, the robots only
# share the network.
#
# For the same reason no broadcasters are started (see franka.launch.py and
# tmrv0_2.launch.py, where the spawners are commented out). There is consequently no
# /joint_states, no TF, no RViz and no MoveIt, and nothing publishes FrankaRobotState.
#
# The arms start in joint_impedance_example_controller, which sweeps joints 4 and 5 around
# the pose captured when that controller activates. An initialization phase therefore runs
# first: move_to_start_example_controller takes every arm to the FR3 home joint configuration
# (0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4) and only then is the sweep activated. The base and the
# spine stay still until that hand-off, then the two motion nodes at the bottom of this file
# start.
#
# Usage:
#   ros2 launch franka_bringup sgs_demo.launch.py
#   ros2 launch franka_bringup sgs_demo.launch.py spine_ip:=172.16.16.10
#
# WARNING: nothing checks for self collision between the two arms. They are commanded
# independently and neither knows where the other is.
############################################################################

import os

from ament_index_python.packages import get_package_share_directory
import franka_bringup.launch_utils as launch_utils
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Both are spawned inactive, in one spawner per arm. HOME_CONTROLLER runs first.
ARM_CONTROLLER = 'joint_impedance_example_controller'
HOME_CONTROLLER = 'move_to_start_example_controller'

load_yaml = launch_utils.load_yaml


def resolve_config_path(config_file):
    """Return an absolute path for a config given as a bare name or a full path."""
    if os.path.isabs(config_file) or os.path.sep in config_file:
        return config_file
    return os.path.join(get_package_share_directory('franka_bringup'), 'config', config_file)


def generate_demo_nodes(context):
    arms_config_file = resolve_config_path(
        LaunchConfiguration('arms_config_file').perform(context)
    )
    base_config_file = LaunchConfiguration('base_config_file').perform(context)
    spine_namespace = LaunchConfiguration('spine_namespace').perform(context)

    # Read the namespace back out of the configs rather than repeating them here, so the
    # motion nodes cannot drift into a namespace the matching controller is not in.
    arms_config = load_yaml(resolve_config_path(arms_config_file))
    arm_namespaces = [str(config.get('namespace', '')) for config in arms_config.values()]
    base_config = next(iter(load_yaml(resolve_config_path(base_config_file)).values()))
    base_namespace = str(base_config.get('namespace', ''))

    bringup_launch = PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch'])
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('franka_bringup'), 'config', 'controllers.yaml']
    )

    # One spawner per arm loads both controllers, one after the other. Two spawners on
    # the same controller manager drop each other's service replies, and Humble's
    # spawner then retries load_controller and exits because the controller is already
    # loaded. home_arms starts only after every arm spawner has exited cleanly.
    arm_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=[
                ARM_CONTROLLER,
                HOME_CONTROLLER,
                '--inactive',
                '--controller-manager-timeout',
                '30',
                '--service-call-timeout',
                '60',
            ],
            parameters=[controllers_yaml],
            output='screen',
        )
        for namespace in arm_namespaces
    ]
    home_arms = Node(
        package='franka_bringup',
        executable='home_arms.py',
        name='arm_homing',
        output='screen',
        parameters=[
            {
                'arm_namespaces': arm_namespaces,
                'home_controller': HOME_CONTROLLER,
                'demo_controller': ARM_CONTROLLER,
            }
        ],
    )
    base_oscillation = Node(
        package='franka_mobile',
        executable='base_oscillation_node.py',
        name='base_oscillation',
        namespace=base_namespace,
        output='screen',
    )
    spine_oscillation = Node(
        package='franka_spine_examples',
        executable='spine_oscillation_example.py',
        name='spine_oscillation',
        namespace=spine_namespace,
        output='screen',
    )

    def start_demo_motion(event, context):
        if event.returncode != 0:
            return []
        return [base_oscillation, spine_oscillation]

    pending_spawners = set(arm_namespaces)

    def start_homing_after_spawners(namespace):
        def handler(event, context):
            if event.returncode != 0:
                return []
            pending_spawners.discard(namespace)
            if pending_spawners:
                return []
            return [home_arms]

        return handler

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_launch, 'example.launch.py'])
            ),
            launch_arguments={
                'robot_config_file': arms_config_file,
                'spawn_controllers': 'false',
            }.items(),
        ),
        *arm_spawners,
        *[
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawner,
                    on_exit=start_homing_after_spawners(namespace),
                )
            )
            for namespace, spawner in zip(arm_namespaces, arm_spawners)
        ],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_launch, 'tmrv0_2.launch.py'])
            ),
            launch_arguments={
                'robot_config_file': base_config_file,
                'controller_name': 'swerve_drive_controller',
            }.items(),
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=home_arms, on_exit=start_demo_motion)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('franka_spine_server'), 'launch', 'spine.launch.py']
                )
            ),
            launch_arguments={
                'spine_ip': LaunchConfiguration('spine_ip'),
                'namespace': spine_namespace,
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'arms_config_file',
                default_value='sgs_demo_arms.config.yaml',
                description='Arm configuration file name or full path.',
            ),
            DeclareLaunchArgument(
                'base_config_file',
                default_value='sgs_demo_base.config.yaml',
                description='Mobile base configuration file name or full path.',
            ),
            DeclareLaunchArgument(
                'spine_ip',
                default_value='172.16.16.10',
                description='Address of the spine HTTP API, served by the mobile base.',
            ),
            DeclareLaunchArgument(
                'spine_namespace',
                default_value='spine',
                description='Namespace for the spine server and the spine motion node.',
            ),
            OpaqueFunction(function=generate_demo_nodes),
        ]
    )
