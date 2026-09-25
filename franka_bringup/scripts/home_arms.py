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

"""Move every configured arm to the Franka home joint pose, then hand it to the demo controller.

Both controllers are spawned inactive. This node activates move_to_start_example_controller
on every arm at once, waits until each one reports process_finished, and only then switches
every arm to the demo controller. The demo controller records its oscillation center in
on_activate, so it must not start until the arm is already at home.
"""

import sys
import time

from franka_bringup.testing.controller_service_client import ControllerServiceClient
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node

HOME_CONTROLLER = 'move_to_start_example_controller'
DEMO_CONTROLLER = 'joint_impedance_example_controller'
# move_to_start's default goal: the FR3 home pose.
# [0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4]


class ArmHoming(Node):
    """Drive a set of namespaced arms through move-to-start and into the demo controller."""

    def __init__(self):
        super().__init__('arm_homing')
        self.declare_parameter('arm_namespaces', ['left_arm', 'right_arm'])
        self.declare_parameter('home_controller', HOME_CONTROLLER)
        self.declare_parameter('demo_controller', DEMO_CONTROLLER)
        self.declare_parameter('ready_timeout', 90.0)
        self.declare_parameter('motion_timeout', 60.0)

    def run(self):
        """Home every arm. Returns a process exit code."""
        namespaces = list(self.get_parameter('arm_namespaces').value)
        home_controller = self.get_parameter('home_controller').value
        demo_controller = self.get_parameter('demo_controller').value
        ready_timeout = self.get_parameter('ready_timeout').value
        motion_timeout = self.get_parameter('motion_timeout').value

        if not namespaces:
            self.get_logger().error('arm_namespaces is empty')
            return 1

        clients = {
            namespace: ControllerServiceClient(self, f'/{namespace}/controller_manager')
            for namespace in namespaces
        }
        try:
            for namespace, client in clients.items():
                self.get_logger().info(f'Waiting for {namespace} controller manager')
                if not client.wait_for_services(timeout_sec=ready_timeout):
                    return 1
                if not client.wait_for_controller_state(
                    home_controller, ['inactive'], timeout_sec=ready_timeout
                ):
                    return 1
                if not client.wait_for_controller_state(
                    demo_controller, ['inactive'], timeout_sec=ready_timeout
                ):
                    return 1

            for namespace, client in clients.items():
                self.get_logger().info(f'Moving {namespace} to the home joint configuration')
                if not client.switch_controllers(activate=[home_controller]):
                    return 1

            if not self._wait_until_home(namespaces, home_controller, motion_timeout):
                return 1

            for namespace, client in clients.items():
                self.get_logger().info(f'Starting the demo controller on {namespace}')
                if not client.switch_controllers(
                    activate=[demo_controller],
                    deactivate=[home_controller],
                    strict=False,
                ):
                    return 1
                client.unload_controller(home_controller)

            self.get_logger().info('Both arms are at the home joint configuration')
            return 0
        finally:
            for client in clients.values():
                client.destroy()

    def _wait_until_home(self, namespaces, home_controller, timeout_sec):
        """Block until every arm's move-to-start controller sets process_finished."""
        # Humble rclpy has no rclpy.parameter_client, so call get_parameters directly.
        parameter_clients = {}
        for namespace in namespaces:
            service = f'/{namespace}/{home_controller}/get_parameters'
            parameter_clients[namespace] = self.create_client(GetParameters, service)
        pending = set(namespaces)
        deadline = time.monotonic() + timeout_sec
        self.get_logger().info(
            f'Waiting up to {timeout_sec:.0f}s for {sorted(pending)} to reach home'
        )

        while pending and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            for namespace in list(pending):
                client = parameter_clients[namespace]
                if not client.service_is_ready():
                    continue
                request = GetParameters.Request()
                request.names = ['process_finished']
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                if not future.done() or future.result() is None:
                    continue
                values = future.result().values
                finished = (
                    values
                    and values[0].type == ParameterType.PARAMETER_BOOL
                    and values[0].bool_value
                )
                if finished:
                    pending.discard(namespace)
                    self.get_logger().info(f'{namespace} is at the home joint configuration')
            time.sleep(0.2)

        if pending:
            self.get_logger().error(
                f'Timed out waiting for {sorted(pending)} to reach the home joint configuration'
            )
            return False
        return True


def main():
    """Home the configured arms and exit."""
    rclpy.init()
    node = ArmHoming()
    exit_code = 1
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
