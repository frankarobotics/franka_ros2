franka_mobile
=============

Package Overview
----------------

This package contains the controllers needed for franka tmr swerve drive robot, both in simulation and on real hardware.

Controllers
--------------------

The package provides:

* ``franka_mobile/swerve_drive_controller``: this is the main drive controller for the tmr. Its main features are:
    - subscribes to `geometry_msgs::msg::TwistStamped` messages on ``~/cmd_vel`` 
    - publishes to `/tf` the odometry transformation between `world` and `base_link`. Frame names are configurable
    - publishes to `~/odom` the `nav_msgs::msg::Odometry` containing the twist coming from the robot's master controller or simulation.
    - writes to a loaned `franka_semantic_components::CartesianVelocity` command interface to send the desired twist to the robot (either real or simulated).
    - Both state and reference interfaces are configurable with prefixes for chaining purposes.
* ``franka_mobile/swerve_ik_controller``: this controller is needed for simulation purposes, e.g. with gazebo. Its main features are:
    - Reads/writes from/to `joint/position` and `joint/velocity` state/command interface exposes by `gz_ros2_control` plugins.
    - Performs the forward/inverse kinematics computation needed to replicated the master controller's behaviour.
    - Exports a `franka_semantic_components::CartesianVelocityInterface` as reference interface.
    - Exports a `franka_semantic_components::CartesianPoseInterface` as state interface.
    - Both state and reference interfaces are configurable with prefixes for chaining purposes.
    

Both these controllers are derived from ``controller_interface::ChainableControllerInterface`` and can be composed with other controllers
that adhere to the same state/reference interfaces exposed. 
Please look the the ros2 control documentation for more informations about chaining controllers.

Configuration
-------------

Please look into the `config/swerve_drive_controller_parameters.yaml` for available overrides in your yaml files.
