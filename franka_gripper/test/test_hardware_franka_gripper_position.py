#!/usr/bin/env python3
#  Copyright (c) 2024 Franka Robotics GmbH
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

import time
import unittest

from franka_msgs.action import Move
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
import rclpy
from rclpy.action import ActionClient
import sensor_msgs.msg


def generate_test_description():
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    namespace_parameter_name = 'namespace'
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)

    franka_gripper_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_gripper'),
                        'launch',
                        'gripper.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            robot_ip_parameter_name: robot_ip,
            namespace_parameter_name: 'test_namespace',
            use_fake_hardware_parameter_name: 'false',
        }.items(),
    )

    return (
        LaunchDescription(
            [
                DeclareLaunchArgument(
                    robot_ip_parameter_name,
                    description='Hostname or IP address of the robot (required).',
                ),
                franka_gripper_description,
                # Start test right away, no need to wait for anything
                ReadyToTest(),
            ],
        ),
        {
            'franka_gripper': franka_gripper_description,
        },
    )


# Test node
class TestStartJointPositions(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.link_node = rclpy.create_node('gripper_test_link')  # type: ignore

    def tearDown(self):
        self.link_node.destroy_node()

    def test_gripper_position(self):
        # max gripper width 0.08m
        self.gripper_positions_goal = [0.02, 0.05, 0.08]
        self.gripper_position = None
        self.gripper_speed = 1.0

        # decimal places in assertion
        ACCURACY = 2

        def _service_callback(msg):
            if len(msg.position) >= 2:
                gripper_left_position = msg.position[0]
                gripper_right_position = msg.position[1]
                self.gripper_position = (
                    gripper_left_position + gripper_right_position
                )

        sub = self.link_node.create_subscription(
            sensor_msgs.msg.JointState,
            '/test_namespace/franka_gripper/joint_states',
            _service_callback,
            10,
        )

        action_client = ActionClient(
            self.link_node,
            action_name='/test_namespace/franka_gripper/move',
            action_type=Move,
        )

        while not action_client.wait_for_server(timeout_sec=1.0):
            self.link_node.get_logger().info(
                'Action server not available, waiting...'
            )

        goal_msg = Move.Goal()
        goal_msg.speed = self.gripper_speed

        for position in self.gripper_positions_goal:
            print('\n')
            self.link_node.get_logger().info(
                f'VALIDATING POSITION {position}:'
            )
            goal_msg.width = position
            future = action_client.send_goal_async(goal_msg)

            """
            status 1 - accepted
            status 2 - executing
            status 3 - canceling
            status 4 - success
            status 5 - aborted
            status 6 - canceled
            """
            # Wait for goal to be accepted
            goal_handle = None
            while goal_handle is None:
                if future.done():
                    goal_handle = future.result()
                rclpy.spin_once(self.link_node, timeout_sec=0.1)

            # Wait for action to complete (status 4 = success)
            result_future = goal_handle.get_result_async()
            action_status = 0
            while action_status != 4:
                if result_future.done():
                    result = result_future.result()
                    action_status = result.status
                    if action_status == 5:  # aborted
                        self.fail(f'Action aborted for position {position}')
                    elif action_status == 6:  # canceled
                        self.fail(f'Action canceled for position {position}')
                rclpy.spin_once(self.link_node, timeout_sec=0.1)

            # Give some time for the position to update via subscription
            time.sleep(0.5)
            # Spin a few times to ensure we get the latest position
            for _ in range(10):
                rclpy.spin_once(self.link_node, timeout_sec=0.1)

            if self.gripper_position is None:
                self.fail('Gripper position was not received')
            self.assertAlmostEqual(position, self.gripper_position, ACCURACY)  # type: ignore
            time.sleep(1)

        self.link_node.destroy_subscription(sub)
        action_client.destroy()
