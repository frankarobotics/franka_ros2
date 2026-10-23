#  Copyright (c) 2026 Franka Robotics GmbH
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

"""
MoveIt planning tests over robot_type (fr3, fr3v2).

End-to-end proof that move_group.launch.py honors the robot_type launch argument:
the MoveIt model, the planning group and the kinematics config must resolve to the
requested type, so that a joint goal for the matching <type>_arm group plans. The
fr3 case additionally guards the default path the rewrite touched. Both move_groups
launch namespaced in one description; each case talks to its own action server.
"""

import time
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

import launch_testing.actions

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    PlanningOptions,
    RobotState,
)

import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState


# Comfortably inside both arms' joint limits, distinct from the goal below.
START_POSITIONS = [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5]
TARGET_POSITIONS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


class MoveItPlanningTestCase:
    """
    Plans a joint goal for one arm; subclass sets the arm constants.

    Plain mixin on purpose: inheriting unittest.TestCase here would let test
    discovery run the base with empty arm constants. Concrete cases below mix
    this in with unittest.TestCase.
    """

    ROBOT_TYPE = None
    GROUP_NAME = None
    JOINT_NAMES = []
    MOVE_GROUP_ACTION_NAMES = ()

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f'moveit_planning_{self.ROBOT_TYPE}_test')
        self.move_group_clients = {
            action_name: ActionClient(self.node, MoveGroup, action_name)
            for action_name in self.MOVE_GROUP_ACTION_NAMES
        }
        self.move_group_client = None

    def tearDown(self):
        if self.move_group_client is not None:
            self.move_group_client.destroy()
        for action_name, client in self.move_group_clients.items():
            if client is not self.move_group_client:
                client.destroy()
        self.node.destroy_node()

    def test_move_group_plans_joint_goal(self):
        """Verify MoveIt can plan to the joint target."""
        self.move_group_client = self._wait_for_move_group_action()

        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.GROUP_NAME
        motion_plan_request.num_planning_attempts = 5
        motion_plan_request.allowed_planning_time = 5.0
        motion_plan_request.start_state = self._build_start_state()
        motion_plan_request.goal_constraints = [self._build_joint_goal()]

        self.assertTrue(
            self.move_group_client is not None,
            'MoveGroup action server did not become available within 30s',
        )

        request = MoveGroup.Goal()
        request.request = motion_plan_request
        request.planning_options = PlanningOptions()
        request.planning_options.plan_only = True

        goal_future = self.move_group_client.send_goal_async(request)
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=10.0)
        self.assertTrue(goal_future.done(), 'MoveGroup goal request timed out')

        goal_handle = goal_future.result()
        self.assertIsNotNone(goal_handle, 'MoveGroup did not return a goal handle')
        self.assertTrue(goal_handle.accepted, 'MoveGroup goal was not accepted')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=30.0)
        self.assertTrue(result_future.done(), 'MoveGroup result did not arrive in time')
        self.assertIsNotNone(result_future.result(), 'MoveGroup did not return a result')

        result = result_future.result().result
        self.assertEqual(
            result.error_code.val,
            MoveItErrorCodes.SUCCESS,
            f'Planning failed with MoveIt error code {result.error_code.val}',
        )
        self.assertGreater(
            len(result.planned_trajectory.joint_trajectory.points),
            0,
            'MoveIt returned an empty planned trajectory',
        )

    def _build_start_state(self):
        """Create an explicit start state (no live joint states in this launch)."""
        start_state = RobotState()
        start_state.is_diff = False
        start_state.joint_state = JointState()
        start_state.joint_state.name = self.JOINT_NAMES
        start_state.joint_state.position = START_POSITIONS
        return start_state

    def _build_joint_goal(self):
        """Create a joint-space goal within the arm limits."""
        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [
            JointConstraint(
                joint_name=joint_name,
                position=position,
                tolerance_above=1e-3,
                tolerance_below=1e-3,
                weight=1.0,
            )
            for joint_name, position in zip(self.JOINT_NAMES, TARGET_POSITIONS)
        ]
        return goal_constraints

    def _wait_for_move_group_action(self):
        """Wait for a compatible MoveGroup action server to become available."""
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            for action_name in self.MOVE_GROUP_ACTION_NAMES:
                client = self.move_group_clients[action_name]
                if client.wait_for_server(timeout_sec=0.5):
                    return client

        return None


class TestMoveItPlanningFr3(MoveItPlanningTestCase, unittest.TestCase):
    """Verify MoveIt plans a joint goal for the default FR3 arm."""

    ROBOT_TYPE = 'fr3'
    GROUP_NAME = 'fr3_arm'
    JOINT_NAMES = [f'fr3_joint{joint_index}' for joint_index in range(1, 8)]
    MOVE_GROUP_ACTION_NAMES = ('/move_group', '/move_action')


class TestMoveItPlanningFr3v2(MoveItPlanningTestCase, unittest.TestCase):
    """Verify MoveIt plans a joint goal for the FR3v2 arm."""

    ROBOT_TYPE = 'fr3v2'
    GROUP_NAME = 'fr3v2_arm'
    JOINT_NAMES = [f'fr3v2_joint{joint_index}' for joint_index in range(1, 8)]
    MOVE_GROUP_ACTION_NAMES = ('/fr3v2/move_group', '/fr3v2/move_action')


def move_group_launch(robot_type, namespace):
    """Include move_group.launch.py for one arm in its own namespace."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('franka_fr3_moveit_config'),
                        'launch',
                        'move_group.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'robot_ip': 'dont-care',
            'robot_type': robot_type,
            'arm_prefix': '',
            'namespace': namespace,
            'load_gripper': 'false',
            'use_fake_hardware': 'true',
            'fake_sensor_commands': 'false',
        }.items(),
    )


def generate_test_description():
    return (
        LaunchDescription(
            [
                move_group_launch('fr3', ''),
                move_group_launch('fr3v2', 'fr3v2'),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )
