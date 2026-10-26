#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(package_name, file_path, prefix=None, robot_type='fr3'):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            yaml_string = file.read()
            # Joint and group names carry '<arm_prefix>_<robot_type>', e.g.
            # 'left_fr3v2_joint1' or 'fr3v2_arm', while the shipped yamls only
            # contain the bare 'fr3' names.
            if prefix or robot_type != 'fr3':
                arm_identifier = (prefix + '_' if prefix else '') + robot_type
                yaml_string = yaml_string.replace('fr3', arm_identifier)
            return yaml.safe_load(yaml_string)
    except EnvironmentError:  # parent of IOError, OSError *and* Windows Error where available
        return None


def generate_robot_nodes(context, *args, **kwargs):
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    namespace_parameter_name = 'namespace'
    arm_prefix_parameter_name = 'arm_prefix'
    robot_type_parameter_name = 'robot_type'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    namespace = LaunchConfiguration(namespace_parameter_name)
    arm_prefix = LaunchConfiguration(arm_prefix_parameter_name)
    robot_type = LaunchConfiguration(robot_type_parameter_name)

    prefix_str = arm_prefix.perform(context)
    robot_type_str = robot_type.perform(context)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', robot_type_str, robot_type_str + '.urdf.xacro'
    )

    robot_description_command = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            franka_xacro_file,
            ' ros2_control:=false',
            ' hand:=', load_gripper,
            ' robot_type:=', robot_type,
            ' arm_prefix:=', arm_prefix,
            ' robot_ip:=', robot_ip,
            ' use_fake_hardware:=', use_fake_hardware,
            ' fake_sensor_commands:=', fake_sensor_commands,
        ]
    )

    robot_description = {'robot_description': ParameterValue(
        robot_description_command, value_type=str)}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', robot_type_str, robot_type_str + '.srdf.xacro'
    )

    robot_description_semantic_command = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            franka_semantic_xacro_file,
            ' hand:=', load_gripper,
            ' robot_type:=', robot_type,
            ' arm_prefix:=', arm_prefix]
    )

    # Use ParameterValue here as well if needed
    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_command, value_type=str)}

    kinematics_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/kinematics.yaml', prefix_str, robot_type_str)

    kinematics_config = {
        'robot_description_kinematics': kinematics_yaml
    }

    joint_limits_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_joint_limits.yaml', prefix_str, robot_type_str
    )

    joint_limits_config = {
        'robot_description_planning': joint_limits_yaml
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
                                ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath'
                                  ],
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml', prefix_str, robot_type_str
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    return [run_move_group_node]


def generate_launch_description():
    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )
    robot_type_arg = DeclareLaunchArgument(
        'robot_type', default_value='fr3', description='ID of the type of arm used'
    )

    return LaunchDescription([
        db_arg,
        robot_type_arg,
        OpaqueFunction(function=generate_robot_nodes)
    ])
