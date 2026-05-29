#!/usr/bin/env python3
"""
pick_and_place.py  —  Phase 4 FINAL (lambda fix + depth fallback)
==================================================================
Key fixes vs previous version:
  1. CRITICAL: broken lambda for /detected_block/pixel subscriber replaced
     with a proper callback method.  Lock.__enter__() returns True (truthy),
     so "True or setattr(...)" short-circuited — self._pixel was never set.
  2. rclpy.spin_until_future_complete() replaced with future.done() polling
     so the background _run thread doesn't fight the MultiThreadedExecutor.
  3. Depth-based 3D reconstruction falls back to ray–floor intersection if
     the depth image patch is invalid (NaN / zero), which can happen in Gazebo
     when the block is at a grazing angle or the depth sensor misses small faces.
  4. Added per-step debug logs so it's obvious which condition is blocking.
"""

import math
import time
from threading import Thread, Lock

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, PoseStamped, Pose
from sensor_msgs.msg import Image, CameraInfo
from controller_manager_msgs.srv import ListControllers

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers transform types)

from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive

# ── Robot config ──────────────────────────────────────────────────────────────
ARM_JOINTS = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]
# Must match HOME list in sim.launch.py
READY_POSITIONS = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
# Joint-space drop configuration
DROP_JOINTS     = [0.5, -0.5, 0.0, -2.0, 0.0, 1.8, 1.1]

APPROACH_HEIGHT = 0.15   # m above block top surface before descending
PICK_HEIGHT     = 0.01   # m above block top surface at grasp

# Camera fixed transform in fr3_link0 frame (from URDF camera_joint)
# origin xyz="0.5 0.0 1.0" rpy="0 pi/2 0"
CAM_POS = np.array([0.5, 0.0, 1.0])   # camera origin in fr3_link0
# Ry(pi/2): maps camera +Z → fr3 +X,  camera +Y → fr3 +Y,  camera +X → fr3 -Z
CAM_R = np.array([
    [0.0,  0.0,  1.0],   # fr3_X = cam_Z
    [0.0,  1.0,  0.0],   # fr3_Y = cam_Y
    [-1.0, 0.0,  0.0],   # fr3_Z = -cam_X
])
FLOOR_Z = 0.025   # m — top of the 5 cm cube sitting on the floor


class PickAndPlace(Node):

    def __init__(self):
        super().__init__('pick_and_place')
        self.get_logger().info('pick_and_place node starting...')

        self._cb_group = ReentrantCallbackGroup()

        # ── TF2 ──────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── MoveGroup action client ───────────────────────────────────────────
        self._move_client = ActionClient(
            self, MoveGroupAction, '/move_group',
            callback_group=self._cb_group)

        # ── Camera state ──────────────────────────────────────────────────────
        self._cam_lock   = Lock()
        self._fx = self._fy = self._cx = self._cy = None
        self._depth: np.ndarray | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self._cam_info_cb, qos, callback_group=self._cb_group)
        self.create_subscription(
            Image, '/camera/depth_image',
            self._depth_cb, qos, callback_group=self._cb_group)

        # ── Vision pixel ──────────────────────────────────────────────────────
        self._pixel_lock = Lock()
        self._pixel: Point | None = None
        # FIX: proper method, not a broken lambda.
        # The old lambda used Lock.__enter__() which returns True (truthy),
        # causing "True or setattr(...)" to short-circuit — setattr never ran.
        self.create_subscription(
            Point, '/detected_block/pixel',
            self._pixel_cb, 10, callback_group=self._cb_group)

        # ── Controller check ──────────────────────────────────────────────────
        self._ctrl_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers',
            callback_group=self._cb_group)

        Thread(target=self._run, daemon=True).start()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pixel_cb(self, msg: Point):
        """Receive pixel centroid from vision_detector."""
        with self._pixel_lock:
            self._pixel = msg

    def _cam_info_cb(self, msg: CameraInfo):
        with self._cam_lock:
            if self._fx is None:
                self._fx = msg.k[0]
                self._fy = msg.k[4]
                self._cx = msg.k[2]
                self._cy = msg.k[5]
                self.get_logger().info(
                    f'Camera intrinsics received: fx={self._fx:.1f} cx={self._cx:.1f}')

    def _depth_cb(self, msg: Image):
        try:
            if msg.encoding == '32FC1':
                data = np.frombuffer(
                    msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            elif msg.encoding == '16UC1':
                data = (np.frombuffer(
                    msg.data, dtype=np.uint16).reshape(
                        msg.height, msg.width).astype(np.float32) / 1000.0)
            else:
                self.get_logger().warn(
                    f'Unknown depth encoding: {msg.encoding}', throttle_duration_sec=5.0)
                return
            with self._cam_lock:
                self._depth = data.copy()
        except Exception as e:
            self.get_logger().error(f'Depth decode error: {e}')

    # ── Controller readiness ──────────────────────────────────────────────────

    def _wait_for_arm_controller(self, timeout=60.0) -> bool:
        self.get_logger().info('Waiting for fr3_arm_controller to become active...')
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self._ctrl_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info('  controller_manager not ready yet...')
                continue
            future = self._ctrl_client.call_async(ListControllers.Request())
            # FIX: poll future.done() instead of rclpy.spin_until_future_complete().
            # spin_until_future_complete() in a background thread fights the
            # MultiThreadedExecutor and can deadlock.
            poll_deadline = time.time() + 5.0
            while not future.done() and time.time() < poll_deadline:
                time.sleep(0.05)
            if not future.done():
                time.sleep(1.0)
                continue
            active = {c.name for c in future.result().controller
                      if c.state == 'active'}
            if 'fr3_arm_controller' in active:
                self.get_logger().info('fr3_arm_controller is active ✓')
                return True
            self.get_logger().info(
                f'  waiting... active controllers: {active}')
            time.sleep(2.0)
        self.get_logger().error('Timed out waiting for fr3_arm_controller')
        return False

    # ── 3-D coordinate reconstruction ────────────────────────────────────────

    def _depth_to_3d(self, u: float, v: float) -> np.ndarray | None:
        """
        Primary method: deproject pixel + depth patch → 3-D point in fr3_link0.
        Returns None if depth values are all invalid.
        """
        with self._cam_lock:
            if self._fx is None or self._depth is None:
                return None
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
            depth = self._depth.copy()

        iu, iv = int(round(u)), int(round(v))
        h, w = depth.shape
        if not (0 <= iu < w and 0 <= iv < h):
            return None

        r = 5  # sample a 11×11 patch
        patch = depth[max(0, iv-r):min(h, iv+r+1),
                      max(0, iu-r):min(w, iu+r+1)]
        valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 5.0)]
        if valid.size < 4:
            self.get_logger().debug('Depth patch has too few valid pixels')
            return None

        d = float(np.median(valid))
        # Camera frame: +Z forward, +X right, +Y down
        x_cam = (u - cx) * d / fx
        y_cam = (v - cy) * d / fy
        z_cam = d
        pt_cam = np.array([x_cam, y_cam, z_cam])

        # Rotate to fr3_link0 frame using camera rotation matrix
        return CAM_POS + CAM_R @ pt_cam

    def _ray_floor_3d(self, u: float, v: float) -> np.ndarray | None:
        """
        Fallback method: ray–floor intersection.
        Assumes the block is on the floor at z = FLOOR_Z (fr3_link0 frame).
        Does not require a depth image.
        """
        with self._cam_lock:
            if self._fx is None:
                return None
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy

        # Ray direction in camera frame (unnormalized)
        d_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        # Ray direction in fr3_link0 frame
        d_base = CAM_R @ d_cam

        # Intersect with plane z = FLOOR_Z
        if abs(d_base[2]) < 1e-6:
            return None   # Ray is parallel to floor
        t = (FLOOR_Z - CAM_POS[2]) / d_base[2]
        if t <= 0:
            return None   # Intersection behind camera
        pt = CAM_POS + t * d_base
        return pt

    def _get_block_position(self) -> np.ndarray | None:
        with self._pixel_lock:
            px = self._pixel
        if px is None:
            self.get_logger().debug('No pixel received yet')
            return None

        # Try depth-based reconstruction first
        pt = self._depth_to_3d(px.x, px.y)
        if pt is None:
            # Fall back to ray–floor intersection
            self.get_logger().debug('Depth unavailable — using ray-floor fallback')
            pt = self._ray_floor_3d(px.x, px.y)

        if pt is None:
            self.get_logger().debug('Camera intrinsics not received yet')
            return None

        # Sanity-check: block must be inside plausible workspace
        if not (0.05 < pt[0] < 1.5 and -0.8 < pt[1] < 0.8 and -0.1 < pt[2] < 0.6):
            self.get_logger().warn(
                f'Block position {pt} outside workspace bounds — skipping')
            return None

        return pt

    # ── MoveGroup helpers ─────────────────────────────────────────────────────

    def _wait_for_future(self, future, timeout: float) -> bool:
        """Poll a future until done or timeout.  Safe from any thread."""
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        return future.done()

    def _send_joint_goal(self, positions: list[float],
                         tol: float = 0.01,
                         vel: float = 0.3) -> bool:
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not found')
            return False

        req = MotionPlanRequest()
        req.group_name = 'fr3_arm'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = 0.2

        c = Constraints()
        for name, pos in zip(ARM_JOINTS, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        goal = MoveGroupAction.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        f = self._move_client.send_goal_async(goal)
        if not self._wait_for_future(f, 15.0) or not f.result().accepted:
            self.get_logger().error('Joint goal rejected')
            return False
        rf = f.result().get_result_async()
        if not self._wait_for_future(rf, 30.0):
            self.get_logger().error('Joint goal timed out')
            return False
        ok = rf.result().result.error_code.val == 1
        if not ok:
            self.get_logger().error(
                f'Joint goal failed: code {rf.result().result.error_code.val}')
        return ok

    def _send_pose_goal(self, x: float, y: float, z: float,
                        vel: float = 0.2) -> bool:
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            return False

        req = MotionPlanRequest()
        req.group_name = 'fr3_arm'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 15.0
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = 0.15
        req.workspace_parameters.header.frame_id = 'fr3_link0'
        for attr, val in (('min_corner', -1.5), ('max_corner', 1.5)):
            corner = getattr(req.workspace_parameters, attr)
            corner.x = corner.y = corner.z = val

        c = Constraints()
        # Position constraint — 2 cm sphere
        pc = PositionConstraint()
        pc.header.frame_id = 'fr3_link0'
        pc.link_name = 'fr3_hand_tcp'
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.02]
        bv.primitives.append(sp)
        p = Pose()
        p.position.x = x; p.position.y = y; p.position.z = z
        p.orientation.w = 1.0
        bv.primitive_poses.append(p)
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)
        # Orientation constraint — EEF pointing down
        oc = OrientationConstraint()
        oc.header.frame_id = 'fr3_link0'
        oc.link_name = 'fr3_hand_tcp'
        oc.orientation.x = 1.0; oc.orientation.w = 0.0
        oc.absolute_x_axis_tolerance = 0.4
        oc.absolute_y_axis_tolerance = 0.4
        oc.absolute_z_axis_tolerance = 0.4
        oc.weight = 0.5
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)

        goal = MoveGroupAction.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        f = self._move_client.send_goal_async(goal)
        if not self._wait_for_future(f, 20.0) or not f.result().accepted:
            self.get_logger().error('Pose goal rejected')
            return False
        rf = f.result().get_result_async()
        if not self._wait_for_future(rf, 40.0):
            self.get_logger().error('Pose goal timed out')
            return False
        ok = rf.result().result.error_code.val == 1
        if not ok:
            self.get_logger().error(
                f'Pose goal failed: code {rf.result().result.error_code.val}')
        return ok

    # ── Main sequence ─────────────────────────────────────────────────────────

    def _run(self):
        if not self._wait_for_arm_controller():
            return

        # Wait for camera intrinsics (needed for both reconstruction paths)
        self.get_logger().info('Waiting for camera intrinsics...')
        deadline = time.time() + 20.0
        while time.time() < deadline:
            with self._cam_lock:
                has_intrinsics = self._fx is not None
            if has_intrinsics:
                break
            time.sleep(0.5)
        if not has_intrinsics:
            self.get_logger().warn(
                'Camera intrinsics not received — will retry in vision loop')

        # Wait for a valid block position
        self.get_logger().info('Waiting for vision coordinates...')
        block_pos = None
        deadline = time.time() + 60.0
        while time.time() < deadline:
            with self._pixel_lock:
                px_set = self._pixel is not None
            with self._cam_lock:
                intrinsics_set = self._fx is not None
                depth_set = self._depth is not None
            if not px_set:
                self.get_logger().info(
                    'Waiting: no pixel from vision_detector yet '
                    '(is vision_detector.py running?)',
                    throttle_duration_sec=5.0)
            elif not intrinsics_set:
                self.get_logger().info(
                    'Waiting: camera intrinsics not received yet',
                    throttle_duration_sec=5.0)
            else:
                if not depth_set:
                    self.get_logger().debug(
                        'No depth image yet — will use floor intersection fallback')
                block_pos = self._get_block_position()
                if block_pos is not None:
                    break
            time.sleep(0.5)

        if block_pos is None:
            self.get_logger().error(
                'Could not determine block position within timeout — aborting.\n'
                'Check: is vision_detector.py running? Is the block spawned?')
            return

        bx, by, bz = block_pos
        self.get_logger().info(
            f'Target locked: x={bx:.3f}  y={by:.3f}  z={bz:.3f}')

        # ── Execute sequence ──────────────────────────────────────────────────
        self.get_logger().info('─── Pick-and-place sequence started ───')

        def step(name: str, ok: bool) -> bool:
            self.get_logger().info(f'  {name}: {"✓" if ok else "✗ FAILED"}')
            return ok

        if not step('1. HOME', self._send_joint_goal(READY_POSITIONS, vel=0.4)):
            return
        if not step('2. APPROACH',
                    self._send_pose_goal(bx, by, bz + APPROACH_HEIGHT, vel=0.25)):
            return
        if not step('3. DESCEND',
                    self._send_pose_goal(bx, by, bz + PICK_HEIGHT, vel=0.1)):
            return
        self.get_logger().info('  4. GRASP (simulated — 1 s)')
        time.sleep(1.0)
        if not step('5. RETRACT',
                    self._send_pose_goal(bx, by, bz + APPROACH_HEIGHT, vel=0.2)):
            return
        if not step('6. DELIVER',
                    self._send_joint_goal(DROP_JOINTS, vel=0.35)):
            return
        self.get_logger().info('  7. RELEASE (simulated — 1 s)')
        time.sleep(1.0)
        if not step('8. HOME', self._send_joint_goal(READY_POSITIONS, vel=0.4)):
            return

        self.get_logger().info('─── Sequence complete ✓ ───')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()