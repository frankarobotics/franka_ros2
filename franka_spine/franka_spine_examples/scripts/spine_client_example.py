#!/usr/bin/env python3
# Copyright (c) 2026 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Example client that exercises the Franka Spine action/service server."""

from franka_spine_msgs.action import MoveAbsolute
from franka_spine_msgs.srv import (
    GetParameters,
    GetPosition,
    GetSpineState,
    SwitchOff,
    SwitchOn,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class SpineClient(Node):
    """Minimal client for the Franka Spine server."""

    def __init__(self):
        super().__init__('spine_client')
        self.state_client = self.create_client(GetSpineState, '/franka_spine_node/get_state')
        self.position_client = self.create_client(GetPosition, '/franka_spine_node/get_position')
        self.switch_on_client = self.create_client(SwitchOn, '/franka_spine_node/switch_on')
        self.switch_off_client = self.create_client(SwitchOff, '/franka_spine_node/switch_off')
        self.get_parameters_client = self.create_client(
            GetParameters, '/franka_spine_node/get_parameters_spine'
        )
        self.move_client = ActionClient(self, MoveAbsolute, '/franka_spine_node/move_absolute')
        self.get_logger().info('Waiting for spine services and action server...')
        try:
            if not self.state_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning('GetState service not available after timeout')
            if not self.position_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning('GetPosition service not available after timeout')
            if not self.switch_on_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning('SwitchOn service not available after timeout')
            if not self.switch_off_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning('SwitchOff service not available after timeout')
            if not self.get_parameters_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning('GetParameters service not available after timeout')
            if not self.move_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warning('Move action server not available after timeout')
        except Exception as e:
            self.get_logger().warning(f'Exception while waiting for services: {e}')

    def get_state(self):
        """Query the current spine state."""
        future = self.state_client.call_async(GetSpineState.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'State: {result.state}, success: {result.success}')
        return result

    def get_position(self):
        """Query the current position in metres."""
        future = self.position_client.call_async(GetPosition.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'Position: {result.position:.4f} m, success: {result.success}')
        return result

    def switch_on(self):
        """Switch the spine on."""
        future = self.switch_on_client.call_async(SwitchOn.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'Switch on: {result.message}')
        return result

    def switch_off(self):
        """Switch the spine off."""
        future = self.switch_off_client.call_async(SwitchOff.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'Switch off: {result.message}')
        return result

    def get_parameters(self):
        """Retrieve spine parameters (user limits)."""
        future = self.get_parameters_client.call_async(GetParameters.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result.success:
            limits = result.parameters.user_limits
            self.get_logger().info(
                f'User limits: [{limits.lower_limit:.4f}, {limits.upper_limit:.4f}] m'
            )
        return result

    def move_to(
        self,
        position,
        velocity=0.1,
        acceleration=0.2,
        deceleration=0.2,
    ):
        """Send a MoveAbsolute goal and wait for the result."""
        self.move_client.wait_for_server()
        goal = MoveAbsolute.Goal()
        goal.position = position
        goal.velocity = velocity
        goal.acceleration = acceleration
        goal.deceleration = deceleration

        future = self.move_client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result

    def _feedback_callback(self, feedback_msg):
        """Log position feedback during motion."""
        self.get_logger().info(f'Position: {feedback_msg.feedback.current_position:.4f} m')


def main():
    rclpy.init()
    client = SpineClient()

    print('--- Franka Spine Client Example ---')

    # Switch on
    client.switch_on()

    # Query state and position
    client.get_state()
    client.get_position()
    client.get_parameters()

    result = client.move_to(
        position=0.4,
        velocity=0.1,
        acceleration=0.2,
        deceleration=0.2,
    )
    if result and result.success:
        client.get_logger().info('Motion 1 completed successfully')

    result = client.move_to(
        position=0.05,
        velocity=0.1,
        acceleration=0.2,
        deceleration=0.2,
    )
    if result and result.success:
        client.get_logger().info('Motion 2 completed successfully')

    # Switch off
    client.switch_off()

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
