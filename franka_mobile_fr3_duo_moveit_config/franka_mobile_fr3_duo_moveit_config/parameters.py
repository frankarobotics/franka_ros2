import json
import os
import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'franka_mobile_fr3_duo_moveit_config'


def load_config_yaml(file_name):
    """Load a YAML config file from this package's config directory."""
    config_yaml_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'config',
        file_name,
    )
    with open(config_yaml_path, 'r') as file:
        return yaml.load(file, Loader=yaml.FullLoader)


def load_moveit_defaults():
    """Load moveit_defaults.json from this package's config directory."""
    moveit_defaults_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'config',
        'moveit_defaults.json',
    )
    with open(moveit_defaults_path, 'r') as file:
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
        'robot_description': robot_description,
        'publish_robot_description': True,
        'robot_description_kinematics': kinematics,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_planning': joint_limits,
    }

    extra_params = {
        'allow_trajectory_execution': True,
        'capabilities': '',
        'disable_capabilities': '',
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
        'use_sim_time': simulate_in_gazebo,
    }

    move_group_configuration.update(moveit_defaults)
    move_group_configuration.update(trajectory_execution)
    move_group_configuration.update(extra_params)

    return [move_group_configuration, extra_params]


def get_parameters():
    kinematics = load_config_yaml('kinematics.yaml')
    joint_limits = load_config_yaml('joint_limits.yaml')
    trajectory_execution = load_config_yaml('moveit_controllers.yaml')
    moveit_defaults = load_moveit_defaults()
    return [kinematics, joint_limits, trajectory_execution, moveit_defaults]


def get_combined_parameters(robot_description, robot_description_semantic, simulate_in_gazebo):
    [kinematics, joint_limits, trajectory_execution, moveit_defaults] = get_parameters()
    return build_move_group_params(
        robot_description,
        robot_description_semantic,
        kinematics,
        joint_limits,
        trajectory_execution,
        moveit_defaults,
        simulate_in_gazebo,
    )


def get_rviz_parameters(simulate_in_gazebo):
    [kinematics, joint_limits, _, moveit_defaults] = get_parameters()
    return [
        {'planning_pipelines': moveit_defaults['planning_pipelines']},
        {'default_planning_pipeline': moveit_defaults['default_planning_pipeline']},
        {'robot_description_kinematics': kinematics},
        {'robot_description_planning': joint_limits},
        {'use_sim_time': simulate_in_gazebo},
    ]
