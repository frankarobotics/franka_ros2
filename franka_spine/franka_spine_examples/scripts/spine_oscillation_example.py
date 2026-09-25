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

"""Move the Franka Spine up and down continuously for the SGS demo."""

from franka_spine_msgs.action import MoveAbsolute
from franka_spine_msgs.srv import SwitchOff, SwitchOn
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class SpineOscillation(Node):
    """
    Cycles the spine between a lower and an upper height for as long as it runs.

    The spine is not a ros2_control joint on real hardware, so it is driven through the
    MoveAbsolute action of franka_spine_server rather than by a controller. Service and
    action names are relative, so the node picks up the server in its own namespace.
    """

    def __init__(self):
        super().__init__('spine_oscillation')

        self.declare_parameter('spine_node_name', 'franka_spine_node')
        self.declare_parameter('lower_position', 0.05)
        self.declare_parameter('upper_position', 0.4)
        self.declare_parameter('velocity', 0.1)
        self.declare_parameter('acceleration', 0.1)
        self.declare_parameter('deceleration', 0.1)

        spine_node = self.get_parameter('spine_node_name').value
        self.switch_on_client = self.create_client(SwitchOn, f'{spine_node}/switch_on')
        self.switch_off_client = self.create_client(SwitchOff, f'{spine_node}/switch_off')
        self.move_client = ActionClient(self, MoveAbsolute, f'{spine_node}/move_absolute')

    def wait_for_server(self, timeout_sec=30.0):
        """Wait for the spine services and action server to show up."""
        if not self.switch_on_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('SwitchOn service not available after timeout')
            return False
        if not self.move_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('Move action server not available after timeout')
            return False
        return True

    def switch_on(self):
        """Switch the spine on so that it accepts motion goals."""
        future = self.switch_on_client.call_async(SwitchOn.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error('Failed to switch the spine on')
            return False
        self.get_logger().info(f'Switch on: {result.message}')
        return True

    def switch_off(self):
        """Switch the spine off."""
        if not self.switch_off_client.service_is_ready():
            return
        future = self.switch_off_client.call_async(SwitchOff.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is not None:
            self.get_logger().info(f'Switch off: {result.message}')

    def run(self):
        """Alternate between the upper and the lower position until the node is stopped."""
        if not self.wait_for_server() or not self.switch_on():
            return

        targets = (
            self.get_parameter('upper_position').value,
            self.get_parameter('lower_position').value,
        )
        index = 0
        while rclpy.ok():
            if not self.move_to(targets[index]):
                # A rejected or aborted goal is not fatal: report it and try the next leg.
                self.get_logger().warning(f'Motion to {targets[index]:.4f} m did not complete')
            index = 1 - index

    def move_to(self, position):
        """Send a MoveAbsolute goal and block until the spine reports it is done."""
        goal = MoveAbsolute.Goal()
        goal.position = position
        goal.velocity = self.get_parameter('velocity').value
        goal.acceleration = self.get_parameter('acceleration').value
        goal.deceleration = self.get_parameter('deceleration').value

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        return result is not None and result.result.success

    def shutdown(self):
        """Stop the spine."""
        # On SIGTERM the context is already torn down here, so there is no way left to reach
        # the spine. Ctrl-C arrives as a KeyboardInterrupt and does get to switch it off.
        if rclpy.ok():
            try:
                self.switch_off()
            except Exception as error:  # noqa: BLE001
                self.get_logger().warning(f'Failed to switch the spine off: {error}')


def main():
    """Run the continuous spine motion until interrupted."""
    rclpy.init()
    node = SpineOscillation()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception:  # noqa: BLE001
        # A SIGTERM from the launch file invalidates the context, which makes whichever
        # service or action call is in flight fail. Anything else is a real error.
        if rclpy.ok():
            raise
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
