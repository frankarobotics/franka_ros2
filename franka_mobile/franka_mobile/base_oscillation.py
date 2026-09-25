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

"""Drive the mobile base in a continuous crab-steer figure for the SGS demo."""

import math

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node

DEFAULT_PUBLISH_RATE = 100.0
DEFAULT_MAX_VELOCITY = 0.1
DEFAULT_DRIVE_PERIOD = 8.0
DEFAULT_STEER_AMPLITUDE = math.pi / 4.0
DEFAULT_STEER_PERIOD = 8.0


class BaseOscillation(Node):
    """
    Publishes a continuously varying twist to the swerve drive controller.

    The speed follows a raised cosine over ``drive_period`` and flips sign on every period,
    so the base drives out and back rather than away. The heading of that twist is swept
    sinusoidally, and because the swerve modules steer to the heading of the commanded
    twist, sweeping it makes both wheels steer continuously while the base keeps
    translating. Together they reproduce the base motion of the mobile FR3 duo example,
    which computed the same profile inside its control loop.
    """

    def __init__(self):
        super().__init__('base_oscillation')

        self.declare_parameter('cmd_vel_topic', 'swerve_drive_controller/cmd_vel')
        self.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE)
        self.declare_parameter('max_velocity', DEFAULT_MAX_VELOCITY)
        self.declare_parameter('drive_period', DEFAULT_DRIVE_PERIOD)
        self.declare_parameter('steer_amplitude', DEFAULT_STEER_AMPLITUDE)
        self.declare_parameter('steer_period', DEFAULT_STEER_PERIOD)

        self._max_velocity = self.get_parameter('max_velocity').value
        self._drive_period = self.get_parameter('drive_period').value
        self._steer_amplitude = self.get_parameter('steer_amplitude').value
        self._steer_period = self.get_parameter('steer_period').value

        if self._drive_period <= 0.0 or self._steer_period <= 0.0:
            raise ValueError('drive_period and steer_period must be positive')

        publish_rate = self.get_parameter('publish_rate').value
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')

        topic = self.get_parameter('cmd_vel_topic').value
        # The controller drops to zero when a command is older than its cmd_vel_timeout, so
        # every message carries the time it was produced and the timer has to keep up.
        self._publisher = self.create_publisher(TwistStamped, topic, 10)
        self._elapsed_time = 0.0
        self._period = 1.0 / publish_rate
        self._timer = self.create_timer(self._period, self._publish_command)

        self.get_logger().info(
            f'Driving the base on {topic} at {publish_rate:.1f} Hz '
            f'({self._max_velocity:.3f} m/s over {self._drive_period:.1f} s, steering '
            f'{math.degrees(self._steer_amplitude):.1f} deg over {self._steer_period:.1f} s)'
        )

    def _publish_command(self):
        """Advance the profile by one tick and publish the twist it produces."""
        # The controller subscribes when it activates, so this waits for it rather than
        # guessing a startup delay. Holding the clock here also means the profile always
        # starts from standstill instead of jumping into the middle of a stroke.
        if self._publisher.get_subscription_count() == 0:
            return

        self._elapsed_time += self._period

        velocity = self._direction() * self._max_velocity / 2.0 * (
            1.0 - math.cos(2.0 * math.pi / self._drive_period * self._elapsed_time)
        )
        heading = self._steer_amplitude * math.sin(
            2.0 * math.pi / self._steer_period * self._elapsed_time
        )

        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = math.cos(heading) * velocity
        message.twist.linear.y = math.sin(heading) * velocity
        self._publisher.publish(message)

    def _direction(self):
        """Return +1 on even drive periods and -1 on odd ones, so the base returns."""
        return 1.0 if int(self._elapsed_time // self._drive_period) % 2 == 0 else -1.0

    def stop(self):
        """Command zero velocity so the base does not coast on its last twist."""
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(message)


def main(args=None):
    """Run the base oscillation until interrupted."""
    rclpy.init(args=args)
    node = BaseOscillation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # On SIGTERM the context is already gone, so this only reaches the base on Ctrl-C.
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
