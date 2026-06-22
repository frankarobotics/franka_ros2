#  Copyright (c) 2026 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the 'License');
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an 'AS IS' BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import unittest

from launch import (
    actions,
    launch_description_sources,
    LaunchDescription,
    substitutions,
)
import launch_ros.substitutions
import launch_testing
import launch_testing.actions
import rclpy
import subprocess

TEST_DURATION = 5.0  # sec

# Known-benign ERROR-level messages that do NOT indicate a real failure.
# Each entry is a substring; any ERROR line matching an entry is excluded from
# the assertion.  Keep this list tight — every entry must have a justification.
KNOWN_BENIGN_ERRORS = [
    # gz_ros2_control plugin races with robot_state_publisher during startup.
    # The service appears shortly after and the system functions normally.
    'robot_state_publisher service not available',
]

def ensure_gz_sim_not_running():
    # Kill any remaining Gazebo/Ignition processes between test runs.
    # See https://github.com/ros2/launch/issues/545 for details.
    # On Humble (Ignition Fortress) the process is 'ign gazebo', not 'gz sim'.
    patterns = [
        '^gz sim',        # Gazebo Garden+ (gz-sim)
        'ign gazebo',     # Ignition Fortress (Humble)
        'ignition',       # Ignition sub-processes
        'ruby.*ign',      # Ruby launcher for Ignition
        'gzserver',       # Classic Gazebo server (fallback)
    ]
    for pattern in patterns:
        subprocess.run(['pkill', '-9', '-f', pattern], check=False)

def generate_test_description():
    """Generate the test launch descriptions."""

    launch_description = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            substitutions.PathJoinSubstitution(
                [
                    launch_ros.substitutions.FindPackageShare(
                        'franka_gazebo_bringup'
                    ),
                    'launch',
                    'gazebo_tmr_example_controller.launch.py',
                ]
            )
        ),

        # let's use gazebo server mode and headless rendering for fast setup/teardown
        launch_arguments={
            'gz_args': 'empty.sdf -r -s --headless-rendering',
            'rviz': 'false'
        }.items(),
    )

    test_description = (
        LaunchDescription(
            [
                launch_description,
                actions.TimerAction(
                    period=TEST_DURATION, actions=[launch_testing.actions.ReadyToTest()]
                ),
            ],
        ),
        {'launch_description': launch_description},
    )
    return test_description


class TestExampleController(unittest.TestCase):
    """Class for testing an Example Controller."""

    @classmethod
    def setUpClass(cls):
        """Initialize the ROS context."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown the ROS context."""
        rclpy.shutdown()
        ensure_gz_sim_not_running()

    def test_has_no_error(self, proc_output):
        """Check that no unexpected ERROR messages appear in launch output.

        Lines matching KNOWN_BENIGN_ERRORS are excluded — these are transient
        startup races that resolve on their own and do not affect functionality.
        """
        error_lines = []
        for event in proc_output:
            if not event.from_stderr:
                continue
            text = event.text.decode('utf-8', errors='replace')
            for line in text.splitlines():
                if 'ERROR' not in line:
                    continue
                if any(pattern in line for pattern in KNOWN_BENIGN_ERRORS):
                    continue
                error_lines.append(line)

        assert not error_lines, (
            'Found unexpected [ERROR] log messages in launch output:\n'
            + '\n'.join(error_lines)
        )
