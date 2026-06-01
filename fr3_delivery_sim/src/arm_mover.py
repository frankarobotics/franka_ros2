#!/usr/bin/env python3
"""
Simple node to move the FR3 arm from its current pose to a target preset or joint array.
It uses the exact same JTC execution bypass method proven to work in pick_and_place.py
to avoid MoveIt 2 Humble "0 controllers" bugs.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory

# Your FR3 joints
ARM_JOINTS = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]

# Named presets (in radians)
PRESETS = {
    'home':     [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398],
    'extended': [0.0, 0.0, 0.0, -0.1, 0.0, 0.1, 0.0],
    'side':     [1.57, -0.5, 0.0, -2.0, 0.0, 1.57, 0.78],
    'low':      [0.0, 0.5, 0.0, -1.0, 0.0, 1.57, 0.0]
}

class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')

        self.declare_parameter('goal_preset', 'home')

        # Provide a dummy float array default. 
        # This perfectly forces ROS 2 type inference to DOUBLE_ARRAY.
        self.declare_parameter('joint_goal', [999.0])

        # Action clients (Planning -> Execution)
        self._move_client = ActionClient(self, MoveGroupAction, '/move_action')
        self._jtc_client = ActionClient(self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory')

        self.get_logger().info('Waiting for MoveIt and JTC action servers...')
        self._move_client.wait_for_server()
        self._jtc_client.wait_for_server()
        self.get_logger().info('Action servers ready.')

    def run(self):
        preset_name = self.get_parameter('goal_preset').value
        joint_goal_list = self.get_parameter('joint_goal').value

        target_joints = None

        # Check if the user passed a custom array (overriding our [999.0] default)
        if len(joint_goal_list) != 1 or joint_goal_list[0] != 999.0:
            if len(joint_goal_list) != 7:
                self.get_logger().error(f"joint_goal must contain exactly 7 floats. Received {len(joint_goal_list)}.")
                return
            target_joints = joint_goal_list
        # Otherwise, fall back to the preset logic
        else:
            if preset_name in PRESETS:
                target_joints = PRESETS[preset_name]
            else:
                self.get_logger().error(f"Unknown preset: '{preset_name}'. Available: {list(PRESETS.keys())}")
                return

        self.get_logger().info(f"Planning motion to: {target_joints}")
        self.plan_and_execute(target_joints)

    def plan_and_execute(self, target_joints):
        # 1. Setup Planning Request
        req = MotionPlanRequest()
        req.group_name = 'fr3_arm'
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        c = Constraints()
        for name, pos in zip(ARM_JOINTS, target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        goal = MoveGroupAction.Goal()
        goal.request = req
        goal.planning_options.plan_only = True

        # 2. Plan trajectory
        f = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)

        if not f.result().accepted:
            self.get_logger().error('Plan rejected by MoveIt.')
            return

        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self, rf)

        plan_result = rf.result().result
        if plan_result.error_code.val != 1:
            self.get_logger().error(f'Planning failed with error code: {plan_result.error_code.val}')
            return

        traj = plan_result.planned_trajectory.joint_trajectory
        self.get_logger().info(f"Plan successful ({len(traj.points)} waypoints). Executing directly via JTC...")

        # 3. Execute directly via JTC (bypassing moveit controller manager)
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0
        jtc_goal = FollowJointTrajectory.Goal()
        jtc_goal.trajectory = traj

        ef = self._jtc_client.send_goal_async(jtc_goal)
        rclpy.spin_until_future_complete(self, ef)

        if not ef.result().accepted:
            self.get_logger().error('Execution rejected by JTC.')
            return

        erf = ef.result().get_result_async()
        rclpy.spin_until_future_complete(self, erf)
        
        result_code = erf.result().result.error_code
        if result_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Execution complete \u2713")
        else:
            self.get_logger().error(f"Execution failed with code: {result_code}")

def main(args=None):
    rclpy.init(args=args)
    node = ArmMover()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()