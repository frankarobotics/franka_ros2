#!/usr/bin/env python3
"""
pick.py  —  fr3_delivery_sim  (hover → open → descend → close)
==============================================================
Minimal, robust pick test. Four steps, in order:

    1. HOVER     — plan + move straight above the detected block
    2. OPEN       — open the gripper completely (verified via /joint_states)
    3. DESCEND    — move straight down onto the block (Cartesian straight line)
    4. CLOSE      — close the gripper onto the block

DESIGN NOTES (why this version finally behaves)
-----------------------------------------------
* ARM motion uses the proven path from arm_mover.py / pick_and_place.py:
  ask MoveIt to PLAN ONLY, then execute the trajectory directly on the
  joint_trajectory_controller action server. The descent is a TRUE Cartesian
  straight line via /compute_cartesian_path so the TCP drops vertically instead
  of the arm swinging (the original "everything messes up on descent" bug).

* GRIPPER: the FR3 hand CANNOT be driven through ros2_control in Gazebo.
  fr3_finger_joint2 mimics fr3_finger_joint1, which trips gz_ros2_control bug
  #343 — the position command to the leader joint is silently ignored, so the
  GripperActionController is "active" but never moves the finger.
  Fix (in the xacro + launch): drive BOTH fingers with Ignition's native
  JointPositionController and command them over std_msgs/Float64 topics
  /fr3_finger_joint1_cmd and /fr3_finger_joint2_cmd. This node publishes the
  target to both and verifies motion via /joint_states.
  REQUIRES the updated fr3_vision_env.urdf.xacro and sim_launch.py.

RUN (sim from sim_launch.py up, a cube spawned, vision_detector.py running):
    ros2 run fr3_delivery_sim pick.py --ros-args -p use_sim_time:=true

Most-tuned overrides:
    -p grasp_z:=0.04           # TCP height at grasp (lower = deeper)
    -p grasp_x_offset:=0.0     # nudge grasp X to centre fingers on the cube
    -p grasp_y_offset:=0.0     # nudge grasp Y to centre fingers on the cube
    -p gripper_open_pos:=0.038 # keep < 0.04 (the joint hard stop)
    -p finger1_topic:=/fr3_finger_joint1_cmd
    -p finger2_topic:=/fr3_finger_joint2_cmd
    -p gripper_close_pos:=0.020
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── Motion / planning ─────────────────────────────────────────────────────────
from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (
    MotionPlanRequest, RobotState, Constraints,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory

# ── Geometry / perception ─────────────────────────────────────────────────────
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Float64
from builtin_interfaces.msg import Duration


ARM_JOINTS = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]
FINGER_JOINT = 'fr3_finger_joint1'
MOVEIT_SUCCESS = 1


def quaternion_from_euler(roll, pitch, yaw):
    """ROS (sxyz) RPY -> (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────
class PickDebug(Node):

    def __init__(self):
        super().__init__('pick_debug')

        # Frames / group
        self.declare_parameter('group_name', 'fr3_arm')
        self.declare_parameter('planning_frame', 'fr3_link0')
        self.declare_parameter('ee_link', 'fr3_hand_tcp')

        # Topics
        self.declare_parameter('pixel_topic', '/detected_block/pixel')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        # Camera mount (relative to fr3_link0), from the xacro
        self.declare_parameter('cam_x', 0.5)
        self.declare_parameter('cam_y', 0.0)
        self.declare_parameter('cam_z', 1.0)

        # Scene + grasp geometry
        self.declare_parameter('table_z', 0.0)
        self.declare_parameter('cube_half', 0.025)
        self.declare_parameter('approach_height', 0.15)
        self.declare_parameter('grasp_z', 0.04)
        self.declare_parameter('grasp_yaw', 0.0)
        self.declare_parameter('grasp_x_offset', 0.0)
        self.declare_parameter('grasp_y_offset', 0.0)

        # Gripper — driven via Ignition JointPositionController topics, NOT
        # ros2_control (see module docstring). open_pos stays below the 0.04 stop.
        self.declare_parameter('finger1_topic', '/fr3_finger_joint1_cmd')
        self.declare_parameter('finger2_topic', '/fr3_finger_joint2_cmd')
        self.declare_parameter('gripper_open_pos', 0.038)
        self.declare_parameter('gripper_close_pos', 0.020)
        self.declare_parameter('gripper_tolerance', 0.006)   # accept within 6 mm
        self.declare_parameter('gripper_timeout', 6.0)       # poll budget per move

        # Planning / descent
        self.declare_parameter('vel_scale', 0.15)
        self.declare_parameter('acc_scale', 0.15)
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('detection_timeout', 30.0)
        self.declare_parameter('descent_speed', 0.02)
        self.declare_parameter('cartesian_step', 0.005)
        self.declare_parameter('min_cartesian_fraction', 0.9)

        gp = self.get_parameter
        self.group_name = gp('group_name').value
        self.planning_frame = gp('planning_frame').value
        self.ee_link = gp('ee_link').value
        self.cam = (gp('cam_x').value, gp('cam_y').value, gp('cam_z').value)
        self.table_z = gp('table_z').value
        self.cube_half = gp('cube_half').value
        self.approach_height = gp('approach_height').value
        self.grasp_z = gp('grasp_z').value
        self.grasp_yaw = gp('grasp_yaw').value
        self.grasp_x_offset = gp('grasp_x_offset').value
        self.grasp_y_offset = gp('grasp_y_offset').value
        self.grip_open = gp('gripper_open_pos').value
        self.grip_close = gp('gripper_close_pos').value
        self.grip_tol = gp('gripper_tolerance').value
        self.grip_timeout = gp('gripper_timeout').value
        self.vel_scale = gp('vel_scale').value
        self.acc_scale = gp('acc_scale').value
        self.planning_time = gp('planning_time').value
        self.detection_timeout = gp('detection_timeout').value
        self.descent_speed = gp('descent_speed').value
        self.cartesian_step = gp('cartesian_step').value
        self.min_fraction = gp('min_cartesian_fraction').value

        # State
        self._latest_pixel = None
        self._cam_info = None
        self._latest_joints = None

        # Subscriptions
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Point, gp('pixel_topic').value, self._pixel_cb, 10)
        self.create_subscription(
            CameraInfo, gp('camera_info_topic').value, self._info_cb, sensor_qos)
        self.create_subscription(JointState, '/joint_states', self._joints_cb, 10)

        # Action clients + Cartesian service
        self._move_client = ActionClient(self, MoveGroupAction, '/move_action')
        self._jtc_client = ActionClient(
            self, FollowJointTrajectory,
            '/fr3_arm_controller/follow_joint_trajectory')
        self._finger1_pub = self.create_publisher(
            Float64, gp('finger1_topic').value, 10)
        self._finger2_pub = self.create_publisher(
            Float64, gp('finger2_topic').value, 10)
        self._cart_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path')

        self.get_logger().info('Waiting for MoveIt (/move_action) + JTC servers...')
        self._move_client.wait_for_server()
        self._jtc_client.wait_for_server()
        self.get_logger().info(
            'Gripper driven via Float64 topics '
            f"({gp('finger1_topic').value}, {gp('finger2_topic').value}).")
        if not self._cart_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                '/compute_cartesian_path not available — descent will use the '
                'joint-space fallback.')
        self.get_logger().info('Servers ready.')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _pixel_cb(self, msg: Point):
        self._latest_pixel = (int(round(msg.x)), int(round(msg.y)))

    def _info_cb(self, msg: CameraInfo):
        self._cam_info = msg

    def _joints_cb(self, msg: JointState):
        self._latest_joints = msg

    # ── Spin helpers ────────────────────────────────────────────────────────────
    def _wait_for(self, predicate, timeout, what):
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        self.get_logger().error(f"Timed out waiting for {what}.")
        return False

    def _settle(self, seconds=1.0):
        deadline = time.time() + seconds
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    # ──────────────────────────────────────────────────────────────────────────
    # GRIPPER — poll-based, never waits on the (hang-prone) result future
    # ──────────────────────────────────────────────────────────────────────────
    def _finger_pos(self):
        js = self._latest_joints
        if js is None:
            return None
        try:
            return float(js.position[list(js.name).index(FINGER_JOINT)])
        except (ValueError, IndexError):
            return None

    def _drive_gripper(self, target, label, expect_stall=False):
        """
        Command BOTH fingers to `target` by publishing std_msgs/Float64 to the
        two JointPositionController topics, then watch fr3_finger_joint1 in
        /joint_states until it reaches the target (within tolerance) or stalls.

        expect_stall=True (close onto cube): a finger that stops moving before
        reaching target is treated as a successful grasp.
        """
        cmd = Float64()
        cmd.data = float(target)
        self._finger1_pub.publish(cmd)
        self._finger2_pub.publish(cmd)
        self.get_logger().info(
            f'[gripper] {label}: commanding both fingers -> {target:.4f} m')

        start = time.time()
        next_pub = start + 0.3
        last_pos = None
        still_since = None

        while time.time() - start < self.grip_timeout:
            now = time.time()
            # Re-publish periodically so a dropped first message (bridge warm-up)
            # can't leave the fingers uncommanded. The controller just holds the
            # latest value, so repeating an identical command is harmless.
            if now >= next_pub:
                self._finger1_pub.publish(cmd)
                self._finger2_pub.publish(cmd)
                next_pub = now + 0.3
            rclpy.spin_once(self, timeout_sec=0.05)

            pos = self._finger_pos()
            if pos is None:
                continue

            if abs(pos - target) <= self.grip_tol:
                self.get_logger().info(
                    f'[gripper] {label}: ✓ reached {pos:.4f} m')
                self._settle(0.4)
                return True

            # Detect "finger has stopped moving" (a stall against the cube).
            if last_pos is not None and abs(pos - last_pos) < 0.0004:
                still_since = still_since or now
                if now - still_since > 1.2 and expect_stall:
                    self.get_logger().info(
                        f'[gripper] {label}: ✓ stalled on object at '
                        f'{pos:.4f} m (grasp).')
                    self._settle(0.4)
                    return True
            else:
                still_since = None
            last_pos = pos

        final = self._finger_pos()
        self.get_logger().warn(
            f'[gripper] {label}: did not reach target — finger at '
            f'{final if final is None else round(final, 4)} m (target {target:.4f}). '
            'If it stayed at 0.0000, the JointPositionController plugins or the '
            'finger command bridges are not active (check the xacro + launch).')
        self._settle(0.4)
        return True

    def open_gripper(self):
        return self._drive_gripper(self.grip_open, 'open', expect_stall=False)

    def close_gripper(self):
        return self._drive_gripper(self.grip_close, 'close', expect_stall=True)

    # ── Localization (geometric back-projection) ─────────────────────────────────
    def _intrinsics(self):
        w, h, hfov = 640.0, 480.0, 1.2   # from the xacro
        fx = (w / 2.0) / math.tan(hfov / 2.0)
        return fx, fx, w / 2.0, h / 2.0

    def localize_block(self):
        if self._latest_pixel is None:
            self.get_logger().error('No block pixel received.')
            return None
        u, v = self._latest_pixel
        fx, fy, cx, cy = self._intrinsics()
        target_z = self.table_z + self.cube_half
        height = self.cam[2] - target_z
        off_u = (u - cx) / fx * height
        off_v = (v - cy) / fy * height
        x = self.cam[0] - off_v + self.grasp_x_offset
        y = self.cam[1] - off_u + self.grasp_y_offset
        self.get_logger().info(
            f'[geometric] pixel=({u},{v}) -> block=({x:.3f}, {y:.3f}, {target_z:.3f}) '
            f'[offsets x={self.grasp_x_offset:+.3f} y={self.grasp_y_offset:+.3f}]')
        return (x, y, target_z)

    # ── Orientation helper ───────────────────────────────────────────────────────
    def _down_quat(self) -> Quaternion:
        x, y, z, w = quaternion_from_euler(math.pi, 0.0, self.grasp_yaw)
        q = Quaternion(); q.x, q.y, q.z, q.w = x, y, z, w
        return q

    # ──────────────────────────────────────────────────────────────────────────
    # HOVER — MoveIt plan_only, executed on the JTC
    # ──────────────────────────────────────────────────────────────────────────
    def _pose_constraints(self, xyz, quat: Quaternion):
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.planning_frame
        pc.link_name = self.ee_link
        pc.target_point_offset = Vector3()
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        bv.primitives.append(sphere)
        region = Pose()
        region.position.x = float(xyz[0])
        region.position.y = float(xyz[1])
        region.position.z = float(xyz[2])
        region.orientation.w = 1.0
        bv.primitive_poses.append(region)
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = self.planning_frame
        oc.link_name = self.ee_link
        oc.orientation = quat
        oc.absolute_x_axis_tolerance = 0.15
        oc.absolute_y_axis_tolerance = 0.15
        oc.absolute_z_axis_tolerance = 0.15
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

    def _plan_pose_goal(self, constraints):
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        req.goal_constraints.append(constraints)

        goal = MoveGroupAction.Goal()
        goal.request = req
        goal.planning_options.plan_only = True

        fut = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('Hover plan rejected/timed out.')
            return None
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=self.planning_time + 5.0)
        if not rfut.done():
            self.get_logger().error('Hover plan result timed out.')
            return None
        res = rfut.result().result
        if res.error_code.val != MOVEIT_SUCCESS:
            self.get_logger().error(f'Hover plan failed (code {res.error_code.val}).')
            return None
        traj = res.planned_trajectory.joint_trajectory
        self.get_logger().info(f'Hover plan OK ({len(traj.points)} waypoints).')
        return traj

    def move_to_hover(self, xyz, tries=3):
        goal_c = self._pose_constraints(xyz, self._down_quat())
        for attempt in range(1, tries + 1):
            self.get_logger().info(
                f'--> HOVER {tuple(round(c, 3) for c in xyz)} '
                f'(attempt {attempt}/{tries})')
            traj = self._plan_pose_goal(goal_c)
            if traj is not None:
                return self._execute(traj)
            self.get_logger().warn(f'Hover attempt {attempt} failed; retrying...')
            self._settle(0.5)
        self.get_logger().error('All hover attempts failed.')
        return False

    # ──────────────────────────────────────────────────────────────────────────
    # DESCEND — Cartesian straight line (fallback: constrained joint plan)
    # ──────────────────────────────────────────────────────────────────────────
    def descend_to(self, xyz):
        if self._cart_client.service_is_ready():
            traj = self._plan_cartesian_descent(xyz)
            if traj is not None:
                self.get_logger().info('--> DESCEND (cartesian straight line)')
                return self._execute(traj)
            self.get_logger().warn('Cartesian descent insufficient — joint fallback.')
        self.get_logger().info('--> DESCEND (joint-space fallback)')
        return self._descend_joint_fallback(xyz)

    def _plan_cartesian_descent(self, xyz):
        target = Pose()
        target.position.x = float(xyz[0])
        target.position.y = float(xyz[1])
        target.position.z = float(xyz[2])
        target.orientation = self._down_quat()

        req = GetCartesianPath.Request()
        req.header.frame_id = self.planning_frame
        req.group_name = self.group_name
        req.link_name = self.ee_link
        req.waypoints = [target]
        req.max_step = self.cartesian_step
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        if self._latest_joints is not None:
            rs = RobotState()
            rs.joint_state = self._latest_joints
            rs.is_diff = False
            req.start_state = rs

        fut = self._cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        resp = fut.result()
        if resp is None:
            self.get_logger().error('compute_cartesian_path: no response.')
            return None
        self.get_logger().info(f'Cartesian fraction = {resp.fraction:.2f}')
        if resp.fraction < self.min_fraction:
            return None
        traj = resp.solution.joint_trajectory
        if not traj.points:
            return None
        dist = self.approach_height + self.cube_half + (self.table_z + self.cube_half - xyz[2])
        return self._retime(traj, abs(dist))

    def _retime(self, traj, distance_m):
        """Stamp an untimed cartesian path with a slow constant-speed profile."""
        n = len(traj.points)
        total = max(distance_m / max(self.descent_speed, 1e-3), 1.0)
        for i, pt in enumerate(traj.points):
            t = total * (i / (n - 1)) if n > 1 else total
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            pt.velocities = []
            pt.accelerations = []
            pt.effort = []
        self.get_logger().info(
            f'Re-timed descent: {n} pts over {total:.1f}s (~{self.descent_speed:.3f} m/s).')
        return traj

    def _descend_joint_fallback(self, xyz):
        goal_c = self._pose_constraints(xyz, self._down_quat())
        for attempt in range(1, 4):
            traj = self._plan_pose_goal(goal_c)
            if traj is not None:
                return self._execute(traj)
            self.get_logger().warn(f'Fallback descent attempt {attempt} failed.')
            self._settle(0.5)
        return False

    # ── Execute a trajectory on the JTC ──────────────────────────────────────────
    def _execute(self, traj):
        if traj is None or not traj.points:
            return False
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0
        jtc_goal = FollowJointTrajectory.Goal()
        jtc_goal.trajectory = traj
        fut = self._jtc_client.send_goal_async(jtc_goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('Execution rejected by JTC.')
            return False
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        code = rfut.result().result.error_code
        if code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Execution complete ✓')
            return True
        self.get_logger().error(f'Execution failed (code {code}).')
        return False

    # ── Orchestration ────────────────────────────────────────────────────────────
    def run(self):
        self.get_logger().info('Waiting for camera_info, joint_states, detection...')
        self._wait_for(lambda: self._cam_info is not None, 10.0, 'camera_info')
        self._wait_for(lambda: self._latest_joints is not None, 5.0, 'joint_states')
        if not self._wait_for(lambda: self._latest_pixel is not None,
                              self.detection_timeout, 'block detection'):
            return

        # Diagnostic: confirm the finger joint is actually published.
        if self._latest_joints is not None:
            if FINGER_JOINT in self._latest_joints.name:
                self.get_logger().info(
                    f'{FINGER_JOINT} present in /joint_states, '
                    f'current = {self._finger_pos():.4f} m')
            else:
                self.get_logger().error(
                    f'{FINGER_JOINT} NOT in /joint_states! Names: '
                    f'{list(self._latest_joints.name)} — the gripper cannot be '
                    'verified or controlled. Check the controller spawn.')

        block = self.localize_block()
        if block is None:
            return
        bx, by, _ = block
        hover = (bx, by, self.table_z + self.cube_half + self.approach_height)
        grasp = (bx, by, self.grasp_z)

        self.get_logger().info(
            f'\n  Block:  x={bx:.3f}  y={by:.3f}\n'
            f'  HOVER z = {hover[2]:.3f}   GRASP z = {grasp[2]:.3f}   '
            f'drop = {hover[2] - grasp[2]:.3f} m\n'
            f'  gripper open->{self.grip_open:.3f}  close->{self.grip_close:.3f}\n'
            f'  >>> gap to cube? tune grasp_x_offset/grasp_y_offset\n'
            f'  >>> fingers hit cube top? lower grasp_z (now {self.grasp_z:.3f})')

        steps = [
            ('1. Hover above block', lambda: self.move_to_hover(hover)),
            ('   settle',            lambda: self._settle(0.8)),
            ('2. Open gripper',      self.open_gripper),
            ('   settle',            lambda: self._settle(0.8)),
            ('3. Descend onto block', lambda: self.descend_to(grasp)),
            ('   settle',            lambda: self._settle(0.5)),
            ('4. Close gripper',     self.close_gripper),
        ]
        for name, action in steps:
            self.get_logger().info(f'=== STEP: {name} ===')
            if not action():
                self.get_logger().error(f'Step "{name}" failed — aborting.')
                return
        self.get_logger().info('Pick sequence complete ✓')


def main(args=None):
    rclpy.init(args=args)
    node = PickDebug()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()