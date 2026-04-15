import os
from ament_index_python import get_package_share_directory
import xacro


def get_robot_descriptions(robot_name, simulate_in_gazebo):
    """Return (robot_description, robot_description_semantic) XML strings."""
    robot_description_file_path = os.path.join(
        get_package_share_directory('franka_mobile_fr3_duo_moveit_config'),
        'urdf',
        f'{robot_name}.moveit.urdf.xacro',
    )

    robot_description_semantic_file_path = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        robot_name,
        f'{robot_name}.srdf.xacro',
    )

    robot_description = xacro.process_file(
        robot_description_file_path,
        mappings={
            'gazebo': simulate_in_gazebo,
            'gazebo_effort': simulate_in_gazebo,
        },
    ).toxml()

    robot_description_semantic = xacro.process_file(
        robot_description_semantic_file_path,
        mappings={},
    ).toxml()

    return robot_description, robot_description_semantic
