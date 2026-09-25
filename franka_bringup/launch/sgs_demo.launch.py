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
# The arms need no demo-specific controller: joint_impedance_example_controller already
# sweeps joints 4 and 5, which is the arm motion this demo shows. The base and the spine
# are driven by the two small nodes started at the bottom of this file.
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# The arm controller spawned in every arm namespace. example.launch.py indexes its
# controller_names list per robot, so it needs one entry per arm rather than one entry
# broadcast to both.
ARM_CONTROLLER = 'joint_impedance_example_controller'
NUMBER_OF_ARMS = 2

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

    # Read the namespace back out of the base config rather than repeating it here, so the
    # motion node cannot drift into a namespace the swerve drive controller is not in.
    base_config = next(iter(load_yaml(resolve_config_path(base_config_file)).values()))
    base_namespace = str(base_config.get('namespace', ''))

    bringup_launch = PathJoinSubstitution([FindPackageShare('franka_bringup'), 'launch'])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_launch, 'example.launch.py'])
            ),
            launch_arguments={
                'robot_config_file': arms_config_file,
                'controller_names': ','.join([ARM_CONTROLLER] * NUMBER_OF_ARMS),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_launch, 'tmrv0_2.launch.py'])
            ),
            launch_arguments={
                'robot_config_file': base_config_file,
                'controller_name': 'swerve_drive_controller',
            }.items(),
        ),
        Node(
            package='franka_mobile',
            executable='base_oscillation_node.py',
            name='base_oscillation',
            namespace=base_namespace,
            output='screen',
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
        # Shares the spine's namespace so its relative service and action names resolve
        # onto the server started just above.
        Node(
            package='franka_spine_examples',
            executable='spine_oscillation_example.py',
            name='spine_oscillation',
            namespace=spine_namespace,
            output='screen',
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
