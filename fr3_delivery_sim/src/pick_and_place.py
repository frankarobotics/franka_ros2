#!/usr/bin/env python3
"""
pick_and_place.py  —  Phase 4 FINAL (direct JTC execution)
===========================================================
The moveit_controller_manager plugin (whether moveit_simple_controller_manager
or Ros2ControlManager) consistently returns "0 controllers" in this Humble +
Gazebo setup. Rather than fight the plugin configuration, we bypass it entirely:

  1. Send goal to /move_action with plan_only=True  → get a planned trajectory
  2. Send that trajectory directly to
     /fr3_arm_controller/follow_joint_trajectory    → execute it

This completely bypasses MoveIt's controller manager and uses the JTC action
server directly — the one we know is active and working.
"""

import time
from threading import Thread, Lock

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import Image, CameraInfo
from controller_manager_msgs.srv import ListControllers

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory

# ── Robot config ──────────────────────────────────────────────────────────────
ARM_JOINTS = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]
READY_POSITIONS = [0.0,  -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
SAFE_POSITIONS  = [0.05, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
DROP_JOINTS     = [0.5,  -0.5,      0.0, -2.0,      0.0, 1.8,      1.1     ]

APPROACH_HEIGHT = 0.15
PICK_HEIGHT     = 0.01

# Camera pose in fr3_link0 frame (URDF: xyz="0.5 0 1.0" rpy="0 pi/2 0")
CAM_POS = np.array([0.5, 0.0, 1.0])
CAM_R   = np.array([[ 0.,  0.,  1.],
                     [ 0.,  1.,  0.],
                     [-1.,  0.,  0.]])
FLOOR_Z = 0.025


class PickAndPlace(Node):

    def __init__(self):
        super().__init__('pick_and_place')
        self.get_logger().info('pick_and_place node starting...')
        self._cb = ReentrantCallbackGroup()

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # MoveGroup — used for PLANNING ONLY
        self._move_client = ActionClient(
            self, MoveGroupAction, '/move_action', callback_group=self._cb)

        # JTC — used for EXECUTION directly, bypassing moveit controller manager
        self._jtc_client = ActionClient(
            self, FollowJointTrajectory,
            '/fr3_arm_controller/follow_joint_trajectory',
            callback_group=self._cb)

        self._cam_lock = Lock()
        self._fx = self._fy = self._cx = self._cy = None
        self._depth: np.ndarray | None = None

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self._cam_info_cb, qos, callback_group=self._cb)
        self.create_subscription(Image, '/camera/depth_image',
                                 self._depth_cb,    qos, callback_group=self._cb)

        self._pixel_lock = Lock()
        self._pixel: Point | None = None
        self.create_subscription(Point, '/detected_block/pixel',
                                 self._pixel_cb, 10, callback_group=self._cb)

        self._ctrl_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers',
            callback_group=self._cb)

        Thread(target=self._run, daemon=True).start()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pixel_cb(self, msg: Point):
        with self._pixel_lock:
            self._pixel = msg

    def _cam_info_cb(self, msg: CameraInfo):
        with self._cam_lock:
            if self._fx is None:
                self._fx, self._fy = msg.k[0], msg.k[4]
                self._cx, self._cy = msg.k[2], msg.k[5]
                self.get_logger().info(
                    f'Camera intrinsics: fx={self._fx:.1f} cx={self._cx:.1f}')

    def _depth_cb(self, msg: Image):
        try:
            if msg.encoding == '32FC1':
                data = np.frombuffer(msg.data, dtype=np.float32
                                     ).reshape(msg.height, msg.width)
            elif msg.encoding == '16UC1':
                data = (np.frombuffer(msg.data, dtype=np.uint16
                                      ).reshape(msg.height, msg.width
                                                ).astype(np.float32) / 1000.0)
            else:
                return
            with self._cam_lock:
                self._depth = data.copy()
        except Exception as e:
            self.get_logger().error(f'Depth decode: {e}')

    # ── Gates ─────────────────────────────────────────────────────────────────

    def _wait_for_arm_controller(self, timeout=90.0) -> bool:
        self.get_logger().info('Waiting for fr3_arm_controller...')
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self._ctrl_client.wait_for_service(timeout_sec=2.0):
                continue
            fut = self._ctrl_client.call_async(ListControllers.Request())
            t = time.time() + 5.0
            while not fut.done() and time.time() < t:
                time.sleep(0.05)
            if not fut.done():
                time.sleep(1.0); continue
            active = {c.name for c in fut.result().controller
                      if c.state == 'active'}
            if 'fr3_arm_controller' in active:
                self.get_logger().info('fr3_arm_controller is active ✓')
                return True
            self.get_logger().info(
                f'  active: {active}', throttle_duration_sec=5.0)
            time.sleep(2.0)
        return False

    def _wait_for_move_group(self, timeout=120.0) -> bool:
        self.get_logger().info('Waiting for MoveGroup action server...')
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._move_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info('MoveGroup ready ✓')
                return True
            elapsed = timeout - (deadline - time.time())
            self.get_logger().info(
                f'  waiting... ({elapsed:.0f} s)', throttle_duration_sec=10.0)
        self.get_logger().error('MoveGroup never became available')
        return False

    def _wait_for_jtc(self, timeout=30.0) -> bool:
        self.get_logger().info('Waiting for JTC action server...')
        if self._jtc_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().info(
                '/fr3_arm_controller/follow_joint_trajectory ready ✓')
            return True
        self.get_logger().error('JTC action server not available')
        return False

    # ── 3-D reconstruction ────────────────────────────────────────────────────

    def _depth_to_3d(self, u, v):
        with self._cam_lock:
            if self._fx is None or self._depth is None:
                return None
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
            depth = self._depth.copy()
        iu, iv = int(round(u)), int(round(v))
        h, w = depth.shape
        if not (0 <= iu < w and 0 <= iv < h):
            return None
        r = 5
        patch = depth[max(0,iv-r):min(h,iv+r+1), max(0,iu-r):min(w,iu+r+1)]
        valid = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 5.0)]
        if valid.size < 4:
            return None
        d = float(np.median(valid))
        pt_cam = np.array([(u-cx)*d/fx, (v-cy)*d/fy, d])
        return CAM_POS + CAM_R @ pt_cam

    def _ray_floor_3d(self, u, v):
        with self._cam_lock:
            if self._fx is None:
                return None
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
        d_cam  = np.array([(u-cx)/fx, (v-cy)/fy, 1.0])
        d_base = CAM_R @ d_cam
        if abs(d_base[2]) < 1e-6:
            return None
        t = (FLOOR_Z - CAM_POS[2]) / d_base[2]
        if t <= 0:
            return None
        return CAM_POS + t * d_base

    def _get_block_position(self):
        with self._pixel_lock:
            px = self._pixel
        if px is None:
            return None
        pt = self._depth_to_3d(px.x, px.y)
        if pt is None:
            pt = self._ray_floor_3d(px.x, px.y)
        if pt is None:
            return None
        if not (0.05 < pt[0] < 1.5 and -0.8 < pt[1] < 0.8 and -0.1 < pt[2] < 0.6):
            self.get_logger().warn(f'Block at {pt} outside workspace')
            return None
        return pt

    # ── Future poll ───────────────────────────────────────────────────────────

    def _poll(self, future, timeout: float) -> bool:
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        return future.done()

    # ── Core: plan via MoveGroup, execute directly via JTC ────────────────────

    def _plan_and_execute(self, req: MotionPlanRequest,
                          exec_timeout: float = 60.0,
                          retries: int = 3) -> bool:
        """
        Plan using MoveGroup (plan_only=True), then send the resulting
        trajectory directly to the JTC action server.
        Bypasses moveit_controller_manager entirely.
        """
        for attempt in range(1, retries + 1):
            if attempt > 1:
                self.get_logger().info(f'  retry {attempt}/{retries}...')
                time.sleep(1.0)

            # ── Step 1: Plan ──────────────────────────────────────────────────
            goal = MoveGroupAction.Goal()
            goal.request = req
            goal.planning_options.plan_only = True   # plan only, no MoveIt exec

            f = self._move_client.send_goal_async(goal)
            if not self._poll(f, 20.0):
                self.get_logger().error('Plan request timed out')
                continue
            if not f.result().accepted:
                self.get_logger().error('Plan request rejected')
                continue

            rf = f.result().get_result_async()
            if not self._poll(rf, 30.0):
                self.get_logger().error('Planning timed out')
                continue

            plan_result = rf.result().result
            if plan_result.error_code.val != 1:
                self.get_logger().error(
                    f'Planning failed: code {plan_result.error_code.val}')
                continue

            trajectory = plan_result.planned_trajectory.joint_trajectory
            if not trajectory.points:
                self.get_logger().error('Planned trajectory is empty')
                continue

            self.get_logger().debug(
                f'Plan succeeded ({len(trajectory.points)} waypoints)')

            # ── Step 2: Execute via JTC directly ──────────────────────────────
            jtc_goal = FollowJointTrajectory.Goal()
            jtc_goal.trajectory = trajectory

            ef = self._jtc_client.send_goal_async(jtc_goal)
            if not self._poll(ef, 10.0):
                self.get_logger().error('JTC goal send timed out')
                continue
            if not ef.result().accepted:
                self.get_logger().error('JTC goal rejected by controller')
                continue

            erf = ef.result().get_result_async()
            # Wait for execution to finish (trajectory duration + buffer)
            if not self._poll(erf, exec_timeout):
                self.get_logger().error('JTC execution timed out')
                continue

            err = erf.result().result.error_code
            if err == FollowJointTrajectory.Result.SUCCESSFUL:
                return True
            self.get_logger().error(f'JTC execution failed: error_code={err}')

        return False

    # ── Motion goal helpers ───────────────────────────────────────────────────

    def _joint_req(self, positions, tol=0.01, vel=0.3) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = 'fr3_arm'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = 0.2
        c = Constraints()
        for name, pos in zip(ARM_JOINTS, positions):
            jc = JointConstraint()
            jc.joint_name = name; jc.position = pos
            jc.tolerance_above = jc.tolerance_below = tol; jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        return req

    def _pose_req(self, x, y, z, vel=0.2) -> MotionPlanRequest:
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
        pc = PositionConstraint()
        pc.header.frame_id = 'fr3_link0'; pc.link_name = 'fr3_hand_tcp'
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE; sp.dimensions = [0.02]
        bv.primitives.append(sp)
        p = Pose()
        p.position.x = x; p.position.y = y; p.position.z = z
        p.orientation.w = 1.0; bv.primitive_poses.append(p)
        pc.constraint_region = bv; pc.weight = 1.0
        c.position_constraints.append(pc)
        oc = OrientationConstraint()
        oc.header.frame_id = 'fr3_link0'; oc.link_name = 'fr3_hand_tcp'
        oc.orientation.x = 1.0; oc.orientation.w = 0.0
        oc.absolute_x_axis_tolerance = 0.4
        oc.absolute_y_axis_tolerance = 0.4
        oc.absolute_z_axis_tolerance = 0.4
        oc.weight = 0.5; c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)
        return req

    # ── Main sequence ─────────────────────────────────────────────────────────

    def _run(self):
        if not self._wait_for_arm_controller(): return
        if not self._wait_for_move_group():     return
        if not self._wait_for_jtc():            return

        self.get_logger().info('Waiting for camera intrinsics...')
        deadline = time.time() + 20.0
        while time.time() < deadline:
            with self._cam_lock:
                if self._fx is not None: break
            time.sleep(0.3)

        self.get_logger().info(
            'Waiting for vision coordinates (is vision_detector.py running?)...')
        block_pos = None
        deadline = time.time() + 60.0
        while time.time() < deadline:
            block_pos = self._get_block_position()
            if block_pos is not None: break
            time.sleep(0.5)

        if block_pos is None:
            self.get_logger().error('No block position — aborting'); return

        bx, by, bz = block_pos
        self.get_logger().info(f'Target: x={bx:.3f}  y={by:.3f}  z={bz:.3f}')
        self.get_logger().info('─── Sequence started ───')

        def step(name, req, exec_t=60.0, vel_override=None):
            if vel_override is not None:
                req.max_velocity_scaling_factor = vel_override
            ok = self._plan_and_execute(req, exec_timeout=exec_t)
            self.get_logger().info(f'  {name}: {"✓" if ok else "✗ FAILED"}')
            return ok

        if not step('1. SAFE HOME',  self._joint_req(SAFE_POSITIONS, vel=0.3)):  return
        if not step('2. APPROACH',   self._pose_req(bx, by, bz+APPROACH_HEIGHT, vel=0.25)): return
        if not step('3. DESCEND',    self._pose_req(bx, by, bz+PICK_HEIGHT,     vel=0.1)):  return
        self.get_logger().info('  4. GRASP (1 s)'); time.sleep(1.0)
        if not step('5. RETRACT',    self._pose_req(bx, by, bz+APPROACH_HEIGHT, vel=0.2)):  return
        if not step('6. DELIVER',    self._joint_req(DROP_JOINTS,      vel=0.35)): return
        self.get_logger().info('  7. RELEASE (1 s)'); time.sleep(1.0)
        if not step('8. HOME',       self._joint_req(READY_POSITIONS,  vel=0.4)):  return

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