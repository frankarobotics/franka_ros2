^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package franka_gazebo_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

UNRELEASED
----------

* feat: added a model-based gravity-compensation system plugin
  (``franka_gazebo_hardware/GazeboGravityCompensationSystem``) for gz_ros2_control that
  injects pinocchio-computed gravity torque on the effort-controlled arm joints. Gravity is
  enabled globally in the Gazebo world, so the zero-torque example controllers behave as on
  the real robot instead of collapsing. Split out of the former ``franka_gazebo`` package.
