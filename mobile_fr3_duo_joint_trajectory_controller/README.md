# Mobile FR3 Duo Joint Trajectory Controller

Adaptation of the [`JointTrajectoryController`](https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller) to account for a mix of Cartesian velocity control for the mobile base and Joint Commands (Position/Effort) for the arms.

This controller should run together with the `franka_mobile_fr3_duo_moveit_config`:

```sh
ros2 launch franka_mobile_fr3_duo_moveit_config moveit.launch.py
```