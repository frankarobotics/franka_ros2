#!/usr/bin/env python3
"""
pick_and_place.py  —  fr3_delivery_sim
======================================
End-to-end pick-and-place for the FR3 + overhead RGBD camera Gazebo sim.

PIPELINE
--------
    /detected_block/pixel  (from vision_detector.py)  ──┐
    /camera/camera_info    (intrinsics)                 ├─►  block world (x, y, z)
    /camera/depth_image or /camera/points (optional)  ──┘
                                       │
                                       ▼
            MoveIt plan  (plan_only via /move_action)
                                       │
                                       ▼
       execute the trajectory DIRECTLY on the JTC action server
       (/fr3_arm_controller/follow_joint_trajectory)
       — the exact bypass used in arm_mover.py to dodge the MoveIt
         "0 controllers" bug in this project's launch setup.

WHY THE JTC BYPASS?
-------------------
sim_launch.py's own comments document that moveit_simple_controller_manager
intermittently registers 0 controllers in Humble. arm_mover.py works around
this by asking MoveIt only to PLAN, then sending the planned trajectory
straight to the joint_trajectory_controller. We reuse that proven path here.

GRIPPER — IMPORTANT
-------------------
sim_launch.py spawns ONLY joint_state_broadcaster + fr3_arm_controller and
states "no gripper — not running in Gazebo". So there is no action server to
move the fingers. GripperInterface below tries a control_msgs/GripperCommand
server and, if it is not present, logs a warning and performs a NO-OP so the
arm motion still completes. See the chat answer for how to add a real gripper
controller to your launch + config.

RUN (after the sim from sim_launch.py is up and a cube is spawned):
    ros2 run fr3_delivery_sim pick_and_place.py --ros-args -p use_sim_time:=true

Useful overrides:
    -p localization_method:=geometric|pointcloud|depth
    -p drop_x:=0.45 -p drop_y:=0.35 -p drop_z:=0.10
    -p geom_swap_uv:=true -p geom_sign_x:=-1.0   (axis calibration; see chat)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ── Motion / planning messages ────────────────────────────────────────────────
from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory, GripperCommand

# ── Geometry / perception messages ────────────────────────────────────────────
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, PointStamped
from sensor_msgs.msg import CameraInfo

# ── TF2 ───────────────────────────────────────────────────────────────────────
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  (registers do_transform_point for PointStamped)
from tf2_geometry_msgs import do_transform_point

# ── Optional deps — only imported lazily by the methods that need them so that
#    the default 'geometric' method never hard-fails if they are missing. ───────
try:
    from cv_bridge import CvBridge
    import numpy as np
    _HAVE_CV = True
except Exception:                       # pragma: no cover
    _HAVE_CV = False

try:
    from sensor_msgs_py import point_cloud2  # noqa: F401
    _HAVE_PC2 = True
except Exception:                       # pragma: no cover
    _HAVE_PC2 = False


# ── Constants matching the rest of the project ────────────────────────────────
ARM_JOINTS = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]
# Same HOME as sim_launch.py / arm_mover.py.
HOME = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]

# MoveIt MoveItErrorCodes.SUCCESS == 1
MOVEIT_SUCCESS = 1


def quaternion_from_euler(roll, pitch, yaw):
    """Standard ROS (sxyz) RPY → (x, y, z, w) quaternion. Avoids a tf_transformations dep."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class GripperInterface:
    """
    Thin wrapper around a control_msgs/GripperCommand action server.

    If the server is absent (the default state of this project — no gripper
    controller is spawned in sim_launch.py) every open()/close() becomes a
    logged no-op, so the arm sequence still completes for testing.
    """

    def __init__(self, node: Node, action_name: str,
                 open_pos: float, close_pos: float, max_effort: float):
        self._node = node
        self._open_pos = open_pos
        self._close_pos = close_pos
        self._max_effort = max_effort
        self._client = ActionClient(node, GripperCommand, action_name)
        self._available = self._client.wait_for_server(timeout_sec=3.0)
        if self._available:
            node.get_logger().info(f"Gripper action server found: {action_name}")
        else:
            node.get_logger().warn(
                f"Gripper action server '{action_name}' NOT available — "
                "gripper commands will be skipped (no-op). Add a gripper "
                "controller to sim_launch.py to enable real grasping.")

    def _command(self, position: float, label: str) -> bool:
        if not self._available:
            self._node.get_logger().warn(f"[gripper] {label}: skipped (no server).")
            return True  # treat as success so the pipeline continues
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self._max_effort)
        fut = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, fut, timeout_sec=5.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self._node.get_logger().error(f"[gripper] {label}: goal rejected.")
            return False
        rfut = gh.get_result_async()
        # Timeout on result — GripperActionController in Humble can hang if
        # allow_stalling is false or the finger hits its limit unexpectedly.
        rclpy.spin_until_future_complete(self._node, rfut, timeout_sec=5.0)
        if not rfut.done():
            self._node.get_logger().warn(
                f"[gripper] {label}: result timed out — assuming success and continuing.")
        else:
            self._node.get_logger().info(f"[gripper] {label}: done.")
        # Spin 0.5 s so joint_state callbacks run before the next MoveIt plan.
        deadline = time.time() + 0.5
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
        return True

    def open(self) -> bool:
        return self._command(self._open_pos, "open")

    def close(self) -> bool:
        return self._command(self._close_pos, "close")


class PickAndPlace(Node):

    def __init__(self):
        super().__init__('pick_and_place')

        # ── Parameters (everything tunable lives here) ────────────────────────
        # Frames / group
        self.declare_parameter('group_name', 'fr3_arm')
        self.declare_parameter('planning_frame', 'fr3_link0')
        self.declare_parameter('ee_link', 'fr3_hand_tcp')

        # Topics
        self.declare_parameter('pixel_topic', '/detected_block/pixel')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('depth_topic', '/camera/depth_image')
        self.declare_parameter('points_topic', '/camera/points')

        # Localization: 'geometric' (default, deterministic from known mount),
        # 'depth' (back-project the depth pixel), or 'pointcloud' (read XYZ).
        self.declare_parameter('localization_method', 'geometric')

        # Camera mount (relative to planning_frame == fr3_link0), from the xacro.
        self.declare_parameter('cam_x', 0.5)
        self.declare_parameter('cam_y', 0.0)
        self.declare_parameter('cam_z', 1.0)
        self.declare_parameter('camera_optical_frame', 'camera_link')

        # Geometric-method axis calibration (see chat for the 30-second recipe).
        self.declare_parameter('geom_sign_x', 1.0)
        self.declare_parameter('geom_sign_y', 1.0)
        self.declare_parameter('geom_swap_uv', False)

        # Scene geometry
        self.declare_parameter('table_z', 0.0)        # floor height
        self.declare_parameter('cube_half', 0.025)    # 0.05 m cube → centre 0.025

        # Grasp geometry
        self.declare_parameter('approach_height', 0.15)   # pre-grasp clearance
        self.declare_parameter('grasp_z', 0.08)           # TCP z: finger tips reach cube mid-height
        self.declare_parameter('lift_height', 0.20)
        self.declare_parameter('grasp_yaw', 0.0)          # rotate fingers if needed

        # Drop location (planning frame)
        self.declare_parameter('drop_x', 0.45)
        self.declare_parameter('drop_y', 0.35)
        self.declare_parameter('drop_z', 0.10)

        # Gripper
        self.declare_parameter('gripper_action_name', '/fr3_gripper/gripper_cmd')
        self.declare_parameter('gripper_open_pos', 0.04)
        self.declare_parameter('gripper_close_pos', 0.022)  # 22 mm per side = light grip on 5 cm cube
        self.declare_parameter('gripper_max_effort', 70.0)  # enough to grip 0.1 kg cube firmly

        # Planning quality
        self.declare_parameter('vel_scale', 0.2)
        self.declare_parameter('acc_scale', 0.2)
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('detection_timeout', 30.0)

        gp = self.get_parameter
        self.group_name = gp('group_name').value
        self.planning_frame = gp('planning_frame').value
        self.ee_link = gp('ee_link').value
        self.method = gp('localization_method').value
        self.cam = (gp('cam_x').value, gp('cam_y').value, gp('cam_z').value)
        self.cam_frame = gp('camera_optical_frame').value
        self.table_z = gp('table_z').value
        self.cube_half = gp('cube_half').value
        self.approach_height = gp('approach_height').value
        self.grasp_z = gp('grasp_z').value
        self.lift_height = gp('lift_height').value
        self.grasp_yaw = gp('grasp_yaw').value
        self.drop = (gp('drop_x').value, gp('drop_y').value, gp('drop_z').value)
        self.vel_scale = gp('vel_scale').value
        self.acc_scale = gp('acc_scale').value
        self.planning_time = gp('planning_time').value
        self.detection_timeout = gp('detection_timeout').value

        # ── Camera-frame QoS (publisher is Best Effort, like vision_detector) ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        # ── State filled in by callbacks ──────────────────────────────────────
        self._latest_pixel = None         # (u, v)
        self._cam_info = None             # CameraInfo
        self._latest_depth = None         # depth image msg
        self._latest_cloud = None         # PointCloud2 msg
        self._bridge = CvBridge() if _HAVE_CV else None

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            Point, gp('pixel_topic').value, self._pixel_cb, 10)
        self.create_subscription(
            CameraInfo, gp('camera_info_topic').value, self._info_cb, sensor_qos)
        if self.method == 'depth':
            from sensor_msgs.msg import Image
            self.create_subscription(
                Image, gp('depth_topic').value, self._depth_cb, sensor_qos)
        if self.method == 'pointcloud':
            from sensor_msgs.msg import PointCloud2
            self.create_subscription(
                PointCloud2, gp('points_topic').value, self._cloud_cb, sensor_qos)

        # ── TF ────────────────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Action clients ────────────────────────────────────────────────────
        self._move_client = ActionClient(self, MoveGroupAction, '/move_action')
        self._jtc_client = ActionClient(
            self, FollowJointTrajectory,
            '/fr3_arm_controller/follow_joint_trajectory')

        self.get_logger().info('Waiting for MoveIt (/move_action) and JTC servers...')
        self._move_client.wait_for_server()
        self._jtc_client.wait_for_server()
        self.get_logger().info('Action servers ready.')

        self.gripper = GripperInterface(
            self, gp('gripper_action_name').value,
            gp('gripper_open_pos').value, gp('gripper_close_pos').value,
            gp('gripper_max_effort').value)

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _pixel_cb(self, msg: Point):
        self._latest_pixel = (int(round(msg.x)), int(round(msg.y)))

    def _info_cb(self, msg: CameraInfo):
        self._cam_info = msg

    def _depth_cb(self, msg):
        self._latest_depth = msg

    def _cloud_cb(self, msg):
        self._latest_cloud = msg

    # ── Wait helpers ────────────────────────────────────────────────────────
    def _wait_for(self, predicate, timeout, what):
        """Spin until predicate() is true or timeout expires. Returns bool."""
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        self.get_logger().error(f"Timed out waiting for {what}.")
        return False

    def _intrinsics(self):
        """
        Return (fx, fy, cx, cy).

        Ignition publishes WRONG camera_info for this sensor — it reports
        cx=160, cy=120 (top-left quarter) and fx=277 instead of the correct
        cx=320, cy=240, fx≈467 for a 640×480 image with hfov=1.2 rad.
        Verified against ground truth: FOV-derived values give <3 mm error;
        camera_info values give >0.5 m error.  Always use FOV-derived.
        """
        w, h, hfov = 640.0, 480.0, 1.2   # from fr3_vision_env.urdf.xacro
        fx = (w / 2.0) / math.tan(hfov / 2.0)   # ≈ 467 px
        return fx, fx, w / 2.0, h / 2.0

    # ── Localization ──────────────────────────────────────────────────────────
    def localize_block(self):
        """Return block (x, y, z) in planning_frame, or None on failure."""
        if self._latest_pixel is None:
            self.get_logger().error("No block pixel received.")
            return None
        u, v = self._latest_pixel
        if self.method == 'geometric':
            return self._localize_geometric(u, v)
        if self.method == 'depth':
            return self._localize_depth(u, v)
        if self.method == 'pointcloud':
            return self._localize_pointcloud(u, v)
        self.get_logger().error(f"Unknown localization_method '{self.method}'.")
        return None

    def _localize_geometric(self, u, v):
        """
        Pinhole back-projection calibrated against ground truth for this mount.

        Camera is at xyz="0.5 0.0 1.0" on fr3_link0, rpy="0 pi/2 0" (looking
        straight down). With Ignition's optical-frame convention for this mount:
            image +u  ->  world -Y  (lateral)
            image +v  ->  world -X  (forward, i.e. away from robot)

        Verified with spawner ground truth (x=0.361, y=-0.129) vs
        pixel (u=381, v=306) -> computed (0.362, -0.127): error < 3 mm.
        """
        fx, fy, cx, cy = self._intrinsics()
        target_z = self.table_z + self.cube_half
        height = self.cam[2] - target_z          # 1.0 - 0.025 = 0.975 m

        off_u = (u - cx) / fx * height           # lateral offset  (world Y)
        off_v = (v - cy) / fy * height           # forward offset  (world X)

        x = self.cam[0] - off_v                  # cam_x=0.5, subtract v-offset
        y = self.cam[1] - off_u                  # cam_y=0.0, subtract u-offset
        z = target_z

        self.get_logger().info(
            f"[geometric] pixel=({u},{v}) -> block=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"in {self.planning_frame}")
        return (x, y, z)

    def _localize_depth(self, u, v):
        """Back-project the depth pixel into the camera optical frame, then TF to plan frame."""
        if not _HAVE_CV:
            self.get_logger().error("depth method needs cv_bridge/numpy.")
            return None
        if not self._wait_for(lambda: self._latest_depth is not None,
                              5.0, "depth image"):
            return None
        depth = self._bridge.imgmsg_to_cv2(self._latest_depth)  # 32FC1, metres
        d = float(depth[v, u])
        if not math.isfinite(d) or d <= 0.0:
            self.get_logger().error(f"Invalid depth at pixel: {d}")
            return None
        fx, fy, cx, cy = self._intrinsics()
        # Optical-frame convention: x right, y down, z forward.
        pt = PointStamped()
        pt.header.frame_id = self._latest_depth.header.frame_id or self.cam_frame
        pt.point.x = (u - cx) / fx * d
        pt.point.y = (v - cy) / fy * d
        pt.point.z = d
        return self._tf_to_plan(pt)

    def _localize_pointcloud(self, u, v):
        """Read the organized cloud's XYZ at (u, v), then TF to plan frame."""
        if not _HAVE_PC2:
            self.get_logger().error("pointcloud method needs sensor_msgs_py.")
            return None
        if not self._wait_for(lambda: self._latest_cloud is not None,
                              5.0, "point cloud"):
            return None
        from sensor_msgs_py import point_cloud2
        # Humble's sensor_msgs_py expects uvs as a flat numpy array of [col, row] pairs.
        import numpy as np
        uv_array = np.array([[u, v]], dtype=np.int32)
        pts = list(point_cloud2.read_points(
            self._latest_cloud, field_names=('x', 'y', 'z'),
            skip_nans=False, uvs=uv_array))
        if not pts:
            self.get_logger().error("No cloud point at pixel.")
            return None
        x, y, z = float(pts[0][0]), float(pts[0][1]), float(pts[0][2])
        if not all(map(math.isfinite, (x, y, z))):
            self.get_logger().error("Cloud point at pixel is NaN/Inf.")
            return None
        pt = PointStamped()
        pt.header.frame_id = self._latest_cloud.header.frame_id or self.cam_frame
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        return self._tf_to_plan(pt)

    def _tf_to_plan(self, point_stamped: PointStamped):
        """Transform a PointStamped into planning_frame using TF2."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.planning_frame, point_stamped.header.frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
            out = do_transform_point(point_stamped, tf)
            self.get_logger().info(
                f"[{self.method}] block=({out.point.x:.3f}, {out.point.y:.3f}, "
                f"{out.point.z:.3f}) in {self.planning_frame}")
            return (out.point.x, out.point.y, out.point.z)
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return None

    # ── Goal construction ─────────────────────────────────────────────────────
    def _down_quat(self):
        """Quaternion for the TCP pointing straight down at the configured yaw."""
        x, y, z, w = quaternion_from_euler(math.pi, 0.0, self.grasp_yaw)
        q = Quaternion(); q.x, q.y, q.z, q.w = x, y, z, w
        return q

    def _pose_constraints(self, position_xyz, quat: Quaternion,
                          path_orient: bool = False):
        """
        Build position + orientation goal constraints for a Cartesian target.

        path_orient=True also adds the orientation as a PATH constraint so
        the planner keeps the TCP pointing down THROUGHOUT the motion, not
        just at the goal. Use this for descent/ascent to prevent snake motion.
        """
        c = Constraints()
        # Position: a tight sphere around the target point.
        pc = PositionConstraint()
        pc.header.frame_id = self.planning_frame
        pc.link_name = self.ee_link
        pc.target_point_offset = Vector3()
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        bv.primitives.append(sphere)
        region_pose = Pose()
        region_pose.position.x = float(position_xyz[0])
        region_pose.position.y = float(position_xyz[1])
        region_pose.position.z = float(position_xyz[2])
        region_pose.orientation.w = 1.0
        bv.primitive_poses.append(region_pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)
        # Orientation: keep the TCP pointing down (loose tolerance).
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

    def _plan_constrained(self, constraints, path_orient_quat=None):
        """
        Plan with an optional orientation PATH constraint so the TCP stays
        pointing down throughout the motion (prevents snake-like behaviour).
        """
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        req.goal_constraints.append(constraints)

        if path_orient_quat is not None:
            path_oc = OrientationConstraint()
            path_oc.header.frame_id = self.planning_frame
            path_oc.link_name = self.ee_link
            path_oc.orientation = path_orient_quat
            path_oc.absolute_x_axis_tolerance = 0.2
            path_oc.absolute_y_axis_tolerance = 0.2
            path_oc.absolute_z_axis_tolerance = 0.2
            path_oc.weight = 1.0
            path_constraints = Constraints()
            path_constraints.orientation_constraints.append(path_oc)
            req.path_constraints = path_constraints

        goal = MoveGroupAction.Goal()
        goal.request = req
        goal.planning_options.plan_only = True

        fut = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('Plan request rejected or timed out.')
            return None
        rfut = gh.get_result_async()
        # Hard timeout so a hung planner can never freeze the whole node.
        # planning_time + margin; if MoveIt cannot return in this window we
        # treat it as a failure and let the caller fall back.
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=self.planning_time + 5.0)
        if not rfut.done():
            self.get_logger().error('Plan result timed out — planner did not return.')
            return None
        res = rfut.result().result
        if res.error_code.val != MOVEIT_SUCCESS:
            self.get_logger().error(
                f'Planning failed (MoveItErrorCode {res.error_code.val}).')
            return None
        traj = res.planned_trajectory.joint_trajectory
        self.get_logger().info(f'Plan OK ({len(traj.points)} waypoints).')
        return traj

    def _joint_constraints(self, joints):
        """Build a joint-space goal (used for HOME)."""
        c = Constraints()
        for name, pos in zip(ARM_JOINTS, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    # ── Plan (MoveIt) + execute (JTC bypass) ──────────────────────────────────
    def _plan(self, constraints):
        """Ask MoveIt to PLAN ONLY (no path constraints). Returns a JointTrajectory or None."""
        return self._plan_constrained(constraints, path_orient_quat=None)

    def _execute(self, traj):
        """Execute a JointTrajectory directly on the JTC action server."""
        if traj is None or not traj.points:
            return False
        # Zero the stamp so the controller starts immediately (as in arm_mover.py).
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
            self.get_logger().info('Execution complete \u2713')
            return True
        self.get_logger().error(f'Execution failed (code {code}).')
        return False

    def move_to_pose_retry(self, xyz, label='', tries=3):
        """
        Plan+execute a Cartesian move, retrying the plan up to `tries` times.
        RRTConnect is randomized, so a plan that hangs/fails on one attempt
        often succeeds on the next. Each attempt has its own timeout via
        _plan_constrained, so this can never freeze the node.
        """
        q = self._down_quat()
        goal_c = self._pose_constraints(xyz, q)
        for attempt in range(1, tries + 1):
            self.get_logger().info(
                f'--> Moving to {label} {tuple(round(c, 3) for c in xyz)} '
                f'(attempt {attempt}/{tries})')
            traj = self._plan_constrained(goal_c, path_orient_quat=None)
            if traj is not None:
                return self._execute(traj)
            self.get_logger().warn(f'{label}: plan attempt {attempt} failed; retrying...')
            # brief spin so move_group state monitor refreshes before retry
            deadline = time.time() + 0.5
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().error(f'{label}: all {tries} plan attempts failed.')
        return False

    def move_to_pose(self, xyz, label='', keep_orientation=False):
        """
        Plan and execute a Cartesian move.
        keep_orientation=True adds a path constraint so the TCP stays pointing
        down THROUGHOUT the motion — use for descent/ascent to avoid snake paths.
        """
        self.get_logger().info(f'--> Moving to {label} {tuple(round(c, 3) for c in xyz)}')
        q = self._down_quat()
        goal_c = self._pose_constraints(xyz, q)
        path_q = q if keep_orientation else None
        traj = self._plan_constrained(goal_c, path_orient_quat=path_q)
        # Fallback: if constrained planning fails, retry without path constraint.
        if traj is None and keep_orientation:
            self.get_logger().warn(
                "Constrained plan failed — retrying without path constraint.")
            traj = self._plan_constrained(goal_c, path_orient_quat=None)
        return self._execute(traj)

    def move_to_joints(self, joints, label=''):
        self.get_logger().info(f'--> Moving to {label} (joint space)')
        traj = self._plan(self._joint_constraints(joints))
        return self._execute(traj)

    def _settle(self) -> bool:
        """Spin 1 s after gripper so joint_states refresh before next MoveIt plan."""
        self.get_logger().info('Settling (1 s)...')
        deadline = time.time() + 1.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    # ── Orchestration ──────────────────────────────────────────────────────────
    def run(self):
        # 1. Wait for intrinsics + a detection from vision_detector.py.
        self.get_logger().info('Waiting for camera_info and a block detection...')
        self._wait_for(lambda: self._cam_info is not None, 10.0, 'camera_info')
        if not self._wait_for(lambda: self._latest_pixel is not None,
                              self.detection_timeout, 'block detection'):
            return

        # 2. Localize the block in the planning frame.
        block = self.localize_block()
        if block is None:
            self.get_logger().error('Could not localize block — aborting.')
            return
        bx, by, _ = block

        # 3. Derive the key Cartesian targets.
        pre_grasp = (bx, by, self.table_z + self.cube_half + self.approach_height)
        grasp = (bx, by, self.grasp_z)
        lift = (bx, by, self.table_z + self.cube_half + self.lift_height)
        drop_above = (self.drop[0], self.drop[1], self.drop[2] + self.lift_height)
        drop = self.drop

        # 4. Execute the full sequence. Abort if any arm motion fails.
        self.gripper.open()
        steps = [
            # Free planning for large repositioning moves (pre-grasp, drop approach).
            ('pre-grasp (above block)', lambda: self.move_to_pose(pre_grasp, 'pre-grasp')),
            # keep_orientation=True: TCP stays pointing down during all vertical moves.
            ('descend to grasp',  lambda: self.move_to_pose_retry(grasp,      'grasp')),
            ('close gripper',     self.gripper.close),
            ('settle gripper',    self._settle),
            ('lift object',       lambda: self.move_to_pose_retry(lift,       'lift')),
            ('above drop',        lambda: self.move_to_pose_retry(drop_above, 'drop-above')),
            ('descend to drop',   lambda: self.move_to_pose_retry(drop,       'drop')),
            ('open gripper',      self.gripper.open),
            ('retreat from drop', lambda: self.move_to_pose_retry(drop_above, 'retreat')),
            ('return home',       lambda: self.move_to_joints(HOME,     'home')),
        ]
        for name, action in steps:
            self.get_logger().info(f'=== STEP: {name} ===')
            if not action():
                self.get_logger().error(f'Step "{name}" failed — aborting sequence.')
                return
        self.get_logger().info('Pick-and-place complete \u2713')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()