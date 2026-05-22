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

"""ROS 2 action/service server for the Franka Spine."""

from franka_spine_msgs.action import MoveAbsolute
from franka_spine_msgs.msg import SpineParameters, UserLimits
from franka_spine_msgs.srv import (
    FaultReset,
    GetParameters,
    GetPosition,
    GetSpineState,
    Halt,
    SwitchOff,
    SwitchOn,
)
from franka_spine_server.spine_controller import SpineController
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class SpineActionServer(Node):
    """ROS 2 node that wires services and actions to the SpineController."""

    def __init__(self):
        super().__init__('franka_spine_node')

        self.declare_parameter('spine_ip', '')
        self.declare_parameter('http_timeout', 5.0)
        self.declare_parameter('feedback_rate', 10.0)

        spine_ip = self.get_parameter('spine_ip').get_parameter_value().string_value
        if not spine_ip:
            self.get_logger().fatal("Parameter 'spine_ip' not set")
            raise ValueError("Parameter 'spine_ip' not set")

        http_timeout = self.get_parameter('http_timeout').get_parameter_value().double_value
        feedback_rate = self.get_parameter('feedback_rate').get_parameter_value().double_value

        self.controller = SpineController(
            spine_ip,
            timeout=http_timeout,
            feedback_rate=feedback_rate,
            logger=self.get_logger(),
        )
        self.get_logger().info(f'Spine controller initialized for {spine_ip}')

        reentrant_callback_group = ReentrantCallbackGroup()
        self.get_state_service = self.create_service(
            GetSpineState,
            '~/get_state',
            self._get_state_callback,
            callback_group=reentrant_callback_group,
        )
        self.get_position_service = self.create_service(
            GetPosition,
            '~/get_position',
            self._get_position_callback,
            callback_group=reentrant_callback_group,
        )
        self.fault_reset_service = self.create_service(
            FaultReset,
            '~/fault_reset',
            self._fault_reset_callback,
            callback_group=reentrant_callback_group,
        )
        self.switch_on_service = self.create_service(
            SwitchOn,
            '~/switch_on',
            self._switch_on_callback,
            callback_group=reentrant_callback_group,
        )
        self.switch_off_service = self.create_service(
            SwitchOff,
            '~/switch_off',
            self._switch_off_callback,
            callback_group=reentrant_callback_group,
        )
        self.halt_service = self.create_service(
            Halt,
            '~/halt',
            self._halt_callback,
            callback_group=reentrant_callback_group,
        )
        self.get_parameters_spine_service = self.create_service(
            GetParameters,
            '~/get_parameters_spine',
            self._get_parameters_callback,
            callback_group=reentrant_callback_group,
        )

        self.move_absolute_server = ActionServer(
            self,
            MoveAbsolute,
            '~/move_absolute',
            execute_callback=self._execute_move_absolute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=reentrant_callback_group,
        )

        self.get_logger().info('Spine action server ready')

    def _goal_callback(self, goal_request):
        """Accept all goals if no motion is in progress."""
        if self.controller.motion_in_progress:
            self.get_logger().warn('Rejecting goal: another motion is in progress')
            return GoalResponse.REJECT
        self.get_logger().info('Goal accepted')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Accept all cancellation requests (will trigger halt)."""
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def _execute_move_absolute(self, goal_handle):
        """Execute MoveAbsolute action."""
        goal = goal_handle.request
        self.get_logger().info(
            f'MoveAbsolute: position={goal.position} m, '
            f'velocity={goal.velocity} m/s, '
            f'acceleration={goal.acceleration} m/s\u00b2, '
            f'deceleration={goal.deceleration} m/s\u00b2'
        )

        feedback = MoveAbsolute.Feedback()

        def on_feedback(current_position):
            feedback.current_position = current_position
            goal_handle.publish_feedback(feedback)

        motion_result = self.controller.move_absolute(
            goal.position,
            goal.velocity,
            goal.acceleration,
            goal.deceleration,
            is_cancelled=lambda: goal_handle.is_cancel_requested,
            on_feedback=on_feedback,
            is_active=rclpy.ok,
        )

        result = MoveAbsolute.Result()
        result.success = motion_result.success
        result.stop_by = motion_result.stop_by
        result.error = motion_result.error

        if motion_result.cancelled:
            goal_handle.canceled()
        elif motion_result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _get_state_callback(self, request, response):
        """Handle GetSpineState service."""
        state = self.controller.get_state()
        response.success = state.success
        response.state = state.state
        response.error_code = state.error_code
        response.error_description = state.error_description
        return response

    def _get_position_callback(self, request, response):
        """Handle GetPosition service."""
        position = self.controller.get_position()
        response.success = position.success
        response.position = position.position
        return response

    def _fault_reset_callback(self, request, response):
        """Handle FaultReset service."""
        cmd = self.controller.fault_reset()
        response.success = cmd.success
        response.state = cmd.state
        response.message = cmd.message
        return response

    def _switch_on_callback(self, request, response):
        """Handle SwitchOn service."""
        cmd = self.controller.switch_on()
        response.success = cmd.success
        response.state = cmd.state
        response.message = cmd.message
        return response

    def _switch_off_callback(self, request, response):
        """Handle SwitchOff service."""
        cmd = self.controller.switch_off()
        response.success = cmd.success
        response.state = cmd.state
        response.message = cmd.message
        return response

    def _halt_callback(self, request, response):
        """Handle Halt service."""
        cmd = self.controller.halt()
        response.success = cmd.success
        response.state = cmd.state
        response.message = cmd.message
        return response

    def _get_parameters_callback(self, request, response):
        """Handle GetParameters service."""
        params = self.controller.get_parameters()
        response.success = params.success
        response.message = params.message
        if params.success and params.user_limits:
            response.parameters = SpineParameters()
            response.parameters.user_limits = UserLimits(
                lower_limit=params.user_limits.lower_limit,
                upper_limit=params.user_limits.upper_limit,
            )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SpineActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('Spine action server shut down gracefully')


if __name__ == '__main__':
    main()
