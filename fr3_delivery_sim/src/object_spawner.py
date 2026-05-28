#!/usr/bin/env python3
"""
Phase 2/3 — Random block spawner for fr3_delivery_sim.

Spawns a single red cube at a random (x, y) position on the floor
within the camera's FOV and the robot's reachable workspace.
"""

import random
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.msg import EntityFactory
from geometry_msgs.msg import Pose

# ── Workspace bounds ──────────────────────────────────────────────────────────
X_MIN, X_MAX =  0.30,  0.70   # metres forward from robot base
Y_MIN, Y_MAX = -0.25,  0.25   # metres lateral
CUBE_HALF_HEIGHT = 0.025      # cube is 0.05 m, so z = 0.025 sits on the floor

# ── SDF template for the red cube ─────────────────────────────────────────────
def make_cube_sdf(name: str) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static> <link name="link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>4.2e-6</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>4.2e-6</iyy><iyz>0</iyz>
          <izz>4.2e-6</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>0.05 0.05 0.05</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.05 0.05 0.05</size></box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>0.9 0.1 0.1 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

class ObjectSpawner(Node):
    SPAWN_SERVICE = '/world/empty/create'

    def __init__(self):
        super().__init__('object_spawner')
        self.client = self.create_client(SpawnEntity, self.SPAWN_SERVICE)

        self.get_logger().info('Waiting for Gazebo spawn service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'  {self.SPAWN_SERVICE} not ready yet, retrying...')

        self.get_logger().info('Spawn service found — spawning red cube.')
        self._spawn_cube()

    def _spawn_cube(self):
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        z = CUBE_HALF_HEIGHT

        self.get_logger().info(f'Target position: x={x:.3f}  y={y:.3f}  z={z}')

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        request = SpawnEntity.Request()
        
        # Modern Gazebo Fortress EntityFactory structure
        factory = EntityFactory()
        factory.name = 'red_cube'
        factory.sdf = make_cube_sdf('red_cube')
        factory.pose = pose
        factory.allow_renaming = True
        
        request.entity_factory = factory

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error('Service call returned no result (timeout?).')
        elif result.success:
            self.get_logger().info(f'Red cube spawned at ({x:.3f}, {y:.3f}, {z}) ✓')
        else:
            self.get_logger().error(f'Spawn failed: {result.status_message}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSpawner()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()