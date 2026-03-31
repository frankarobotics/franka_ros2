mobile_fr3_duo_trajectory_controller
====================================

Package Overview
----------------

This package contains a controller needed for executing trajectories on the mobile fr3 duo, both in simulation and on real hardware.

Controllers
-----------

The package provides:

* ``mobile_fr3_duo_trajectory_controller``: a controller derived from the [`JointTrajectoryController`](https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller). The key differences are:
    - tailored to control the full `mobile_fr3_duo_v02` via the joint torque command interface for the arms, and cartesian velocity commands for the `tmrv0_2`
    - intended to be used together with Moveit 2. For more information and an example launch file, see the `franka_mobile_fr3_duo_moveit_config` package
    - provides a `FollowJointTrajectory` action server, but for the `tmrv0_2`, it converts the velocities of the joints corresponding to the virtual planar joint used for planning to cartesian velocity commands for the mobile base.
    - the arms follow a simple joint impedance control law with damping on the filtered joint velocities, which can be parametrized through the `k_gains` and `p_gains` from the `.yaml` configuration file passed to the `controller_manager`