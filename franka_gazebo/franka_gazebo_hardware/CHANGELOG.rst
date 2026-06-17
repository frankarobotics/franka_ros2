^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package franka_gazebo_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

UNRELEASED
----------

* feat: new package holding the Gazebo (gz_ros2_control) system plugin
  ``franka_gazebo_hardware/GazeboGravityCompensationSystem``. The plugin computes
  pinocchio-based gravity torque and injects it on the effort-controlled Franka
  arm joints so the zero-torque example controllers behave as on the real robot
  instead of collapsing under their own weight. Split out of ``franka_gazebo_bringup``.
