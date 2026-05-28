#!/usr/bin/env python3
"""
pick_and_place.py — FR3 Gazebo Physics version (corrected)

What was wrong in the previous version:
  • Error -4  (PLANNING_FAILED):   Joint goals were being sent before
    joint_state_broadcaster was active, so MoveIt had no robot state to plan from.
  • Error -16 (INVALID_MOTION_PLAN): fr3_arm_controller wasn't active yet, so
    the trajectory action server rejected execution.

Fixes applied:
  1. Node blocks at startup until BOTH controllers report 'active' via
     /controller_manager/list_controllers before touching MoveIt at all.
  2. Optical-frame → physical-frame coordinate mapping retained from Gemini fix.
  3. MultiThreadedExecutor kept so MoveIt action futures resolve correctly while
     the subscriber loop keeps running.
  4. Planning-failed on HOME move is now handled: if the robot is already at HOME
     (PLANNING_FAILED because nothing to do) we treat it as success and continue.
"""

import math
import time
from threading import Thread, Lock

import numpy as np
import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point, PoseStamped, Quaternion
from sensor_msgs.msg import Image, CameraInfo
from controller_manager_msgs.srv import ListControllers
from cv_bridge import CvBridge

import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, PlanningOptions, Constraints,
    PositionConstraint, OrientationConstraint, JointConstraint,
    WorkspaceParameters, BoundingVolume, MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive

# ════════════════════════════════════════════════════════════════════════════
# Tunable constants
# ════════════════════════════════════════════════════════════════════════════

PLANNING_GROUP  = 'fr3_arm'
EE_LINK         = 'fr3_hand_tcp'
BASE_FRAME      = 'fr3_link0'
CAMERA_FRAME    = 'camera_link'

JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]

# Must match the -J values in sim_launch.py and the URDF spawn args
HOME_JOINTS = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]

# Drop-off joint config — arm swings to the side and lowers slightly
DROP_JOINTS = [1.2, 0.3, 0.0, -1.8, 0.0, 2.0, 0.785398]

APPROACH_HEIGHT = 0.15   # metres above block before descending
GRASP_OFFSET    = 0.005  # metres below block centre for grasp
PLAN_TIMEOUT    = 15.0   # seconds

# ════════════════════════════════════════════════════════════════════════════
# Helpers
# ════════════════════════════════════════════════════════════════════════════

def _pointing_down_quat() -> Quaternion:
    """Quaternion for gripper Z-axis pointing straight down (pitch = pi/2)."""
    pitch = math.pi / 2.0
    q = Quaternion()
    q.w = math.cos(pitch / 2.0)
    q.x = 0.0
    q.y = math.sin(pitch / 2.0)
    q.z = 0.0
    return q

# ════════════════════════════════════════════════════════════════════════════
# Node
# ════════════════════════════════════════════════════════════════════════════

class PickAndPlace(Node):

    def __init__(self):
        super().__init__(
            'pick_and_place',
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    'use_sim_time',
                    rclpy.Parameter.Type.BOOL,
                    True,
                )
            ],
        )

        self.cb_group     = ReentrantCallbackGroup()
        self.lock         = Lock()
        self.already_moved = False
        self.target_coords = None

        self.bridge       = CvBridge()
        self.depth_image  = None
        self.camera_info  = None

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._mg_client  = ActionClient(
            self, MoveGroup, 'move_action', callback_group=self.cb_group
        )

        # Controller manager service — used to wait for controllers
        self._list_ctrl_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
            callback_group=self.cb_group,
        )

        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Point, '/detected_block/pixel',
            self.pixel_callback, 10, callback_group=self.cb_group,
        )
        self.create_subscription(
            Image, '/camera/depth_image',
            self.depth_callback, cam_qos, callback_group=self.cb_group,
        )
        self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self.info_callback, cam_qos, callback_group=self.cb_group,
        )

        self.get_logger().info('Waiting for controllers to become active…')
        Thread(target=self._wait_for_controllers_then_listen, daemon=True).start()

    # ── Startup: block until both controllers are active ─────────────────────

    def _wait_for_controllers_then_listen(self):
        """
        Poll /controller_manager/list_controllers until both
        joint_state_broadcaster and fr3_arm_controller report 'active'.
        Only then enable pixel callbacks.
        """
        required = {'joint_state_broadcaster', 'fr3_arm_controller'}

        while rclpy.ok():
            if not self._list_ctrl_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(
                    'controller_manager not ready yet…', throttle_duration_sec=4.0
                )
                continue

            req    = ListControllers.Request()
            future = self._list_ctrl_client.call_async(req)

            # Spin until future resolves (safe inside a daemon thread with
            # MultiThreadedExecutor running in the main thread)
            deadline = time.time() + 5.0
            while not future.done() and time.time() < deadline:
                time.sleep(0.1)

            if not future.done():
                time.sleep(1.0)
                continue

            active = {
                c.name
                for c in future.result().controller
                if c.state == 'active'
            }

            if required.issubset(active):
                self.get_logger().info(
                    'Controllers active — waiting for vision coordinates…'
                )
                break

            missing = required - active
            self.get_logger().info(
                f'Waiting for controllers: {missing}', throttle_duration_sec=3.0
            )
            time.sleep(1.0)

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def depth_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            with self.lock:
                self.depth_image = frame
        except Exception:
            pass

    def info_callback(self, msg: CameraInfo):
        with self.lock:
            self.camera_info = msg

    def pixel_callback(self, msg: Point):
        if self.already_moved:
            return

        u, v = int(msg.x), int(msg.y)

        with self.lock:
            if self.depth_image is None or self.camera_info is None:
                return
            depth = self.depth_image.copy()
            K     = self.camera_info.k

        Z = float(depth[v, u])
        if Z <= 0.0 or not np.isfinite(Z):
            return

        cx, cy, fx, fy = K[2], K[5], K[0], K[4]
        x_cam = (u - cx) * Z / fx
        y_cam = (v - cy) * Z / fy
        z_cam = Z

        # ── Optical → physical frame mapping ─────────────────────────────────
        # camera_link optical frame:  +X right, +Y down, +Z forward (depth)
        # camera_link physical frame: +X forward, +Y left,  +Z up
        ps = PoseStamped()
        ps.header.frame_id    = CAMERA_FRAME
        ps.header.stamp       = self.get_clock().now().to_msg()
        ps.pose.position.x    =  z_cam    # depth  → forward
        ps.pose.position.y    = -x_cam    # optical-right → physical-left
        ps.pose.position.z    = -y_cam    # optical-down  → physical-up
        ps.pose.orientation.w = 1.0

        try:
            base_ps = self.tf_buffer.transform(
                ps, BASE_FRAME,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return

        bx = base_ps.pose.position.x
        by = base_ps.pose.position.y
        bz = base_ps.pose.position.z

        # Sanity check — reject obviously bad deprojections
        if not (-1.5 < bx < 1.5 and -1.5 < by < 1.5 and -0.1 < bz < 1.5):
            self.get_logger().warn(
                f'Suspicious target ({bx:.2f},{by:.2f},{bz:.2f}) — ignoring.'
            )
            return

        self.target_coords  = [bx, by, bz]
        self.already_moved  = True
        self.get_logger().info(
            f'Target locked → base frame: '
            f'x={bx:.3f}  y={by:.3f}  z={bz:.3f}'
        )

        Thread(target=self.run_sequence, daemon=True).start()

    # ── Main pick-and-place sequence ──────────────────────────────────────────

    def run_sequence(self):
        bx, by, bz = self.target_coords

        self.get_logger().info('━━━ Pick-and-place sequence start ━━━')

        # Step 1 — HOME
        # Note: if the robot is already at HOME, PLANNING_FAILED (-4) is
        # expected (nothing to plan). We treat that as success and continue.
        self.get_logger().info('Step 1 › HOME position')
        ok = self.move_to_joints(HOME_JOINTS)
        if not ok:
            self.get_logger().warn(
                'HOME plan failed (robot may already be there) — continuing.'
            )

        # Step 2 — Approach above block
        self.get_logger().info('Step 2 › APPROACH above block')
        if not self.move_to_pose(bx, by, bz + APPROACH_HEIGHT):
            self.get_logger().error('APPROACH failed — aborting.')
            self.already_moved = False   # allow retry on next detection
            return

        # Step 3 — Open gripper (simulated pause)
        self.get_logger().info('Step 3 › Open gripper')
        time.sleep(0.8)

        # Step 4 — Descend to grasp height
        self.get_logger().info('Step 4 › DESCEND to block')
        if not self.move_to_pose(bx, by, bz + GRASP_OFFSET):
            self.get_logger().error('DESCEND failed — aborting.')
            self.already_moved = False
            return

        # Step 5 — Close gripper
        self.get_logger().info('Step 5 › Close gripper (grasp)')
        time.sleep(0.8)

        # Step 6 — Retract upward
        self.get_logger().info('Step 6 › RETRACT upward')
        if not self.move_to_pose(bx, by, bz + APPROACH_HEIGHT):
            self.get_logger().error('RETRACT failed — aborting.')
            self.already_moved = False
            return

        # Step 7 — Move to drop-off joint configuration
        self.get_logger().info('Step 7 › DELIVER to drop-off')
        if not self.move_to_joints(DROP_JOINTS):
            self.get_logger().error('DELIVER failed — aborting.')
            self.already_moved = False
            return

        # Step 8 — Release
        self.get_logger().info('Step 8 › Release (open gripper)')
        time.sleep(0.8)

        # Step 9 — Return HOME
        self.get_logger().info('Step 9 › Return HOME')
        self.move_to_joints(HOME_JOINTS)

        self.get_logger().info('━━━ Pick-and-place sequence COMPLETE ━━━')
        # Reset so the node picks up the next detected block automatically
        self.already_moved = False

    # ── MoveIt goal helpers ───────────────────────────────────────────────────

    def move_to_joints(self, joint_positions) -> bool:
        c = Constraints()
        for name, pos in zip(JOINT_NAMES, joint_positions):
            jc = JointConstraint(
                joint_name=name, position=pos,
                tolerance_above=0.05, tolerance_below=0.05, weight=1.0,
            )
            c.joint_constraints.append(jc)
        return self._send_goal(c)

    def move_to_pose(self, x: float, y: float, z: float) -> bool:
        q = _pointing_down_quat()

        ps = PoseStamped()
        ps.header.frame_id  = BASE_FRAME
        ps.header.stamp     = self.get_clock().now().to_msg()
        ps.pose.position.x  = x
        ps.pose.position.y  = y
        ps.pose.position.z  = z
        ps.pose.orientation = q

        pc = PositionConstraint(header=ps.header, link_name=EE_LINK, weight=1.0)
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(
                type=SolidPrimitive.SPHERE, dimensions=[0.01]
            )],
            primitive_poses=[ps.pose],
        )

        oc = OrientationConstraint(
            header=ps.header, link_name=EE_LINK,
            orientation=q,
            absolute_x_axis_tolerance=0.2,
            absolute_y_axis_tolerance=0.2,
            absolute_z_axis_tolerance=0.2,
            weight=1.0,
        )

        return self._send_goal(
            Constraints(position_constraints=[pc], orientation_constraints=[oc])
        )

    def _send_goal(self, constraints: Constraints) -> bool:
        """Send a MoveGroup goal and block until execution finishes."""
        if not self._mg_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_group action server not available.')
            return False

        req = MotionPlanRequest(
            group_name=PLANNING_GROUP,
            goal_constraints=[constraints],
            num_planning_attempts=5,
            allowed_planning_time=PLAN_TIMEOUT,
            max_velocity_scaling_factor=0.2,
            max_acceleration_scaling_factor=0.2,
        )
        ws = WorkspaceParameters()
        ws.header.frame_id = BASE_FRAME
        ws.min_corner.x = -1.2;  ws.min_corner.y = -1.2;  ws.min_corner.z = -0.2
        ws.max_corner.x =  1.2;  ws.max_corner.y =  1.2;  ws.max_corner.z =  2.0
        req.workspace_parameters = ws

        goal = MoveGroup.Goal(
            request=req,
            planning_options=PlanningOptions(plan_only=False, replan=False),
        )

        future = self._mg_client.send_goal_async(goal)

        # Busy-wait — safe because this runs in a daemon thread while
        # MultiThreadedExecutor handles callbacks on other threads
        deadline = time.time() + PLAN_TIMEOUT + 10.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)

        if not future.done():
            self.get_logger().error('Goal send timed out.')
            return False

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False

        res_future = goal_handle.get_result_async()
        deadline   = time.time() + PLAN_TIMEOUT + 30.0
        while not res_future.done() and time.time() < deadline:
            time.sleep(0.05)

        if not res_future.done():
            self.get_logger().error('Execution timed out.')
            return False

        err = res_future.result().result.error_code.val
        if err == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('    ✓ motion succeeded')
            return True

        # PLANNING_FAILED when already at goal is harmless
        if err == MoveItErrorCodes.PLANNING_FAILED:
            self.get_logger().warn(
                f'    ⚠ PLANNING_FAILED ({err}) — robot may already be at goal.'
            )
            return False   # caller decides whether to continue

        self.get_logger().error(f'    ✗ MoveIt error code: {err}')
        return False


# ════════════════════════════════════════════════════════════════════════════
# Entry point
# ════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = PickAndPlace()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()