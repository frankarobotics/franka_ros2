import os
from ament_index_python import get_package_share_directory
import xacro

def get_robot_descriptions(robot_name, use_fake_hardware, simulate_in_gazebo):
    """Return (robot_description, robot_description_semantic) XML strings."""
    if simulate_in_gazebo == "true":
        robot_description_file_path = os.path.join(
            get_package_share_directory("franka_gazebo_bringup"),
            "urdf",
            f"{robot_name}.gazebo.urdf.xacro",
        )
    else:
        robot_description_file_path = os.path.join(
            get_package_share_directory("franka_description"),
            "robots",
            robot_name,
            f"{robot_name}.urdf.xacro",
        )

    robot_description_semantic_file_path = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        robot_name,
        f"{robot_name}.srdf.xacro",
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
            "gazebo_effort": simulate_in_gazebo,
            "fake_sensor_commands": "false",
            "is_async": "true",
            "thread_priority": "97",
        },
    ).toxml()

    robot_description_semantic = xacro.process_file(
        robot_description_semantic_file_path,
        mappings={},
    ).toxml()

    return robot_description, robot_description_semantic