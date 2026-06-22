^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package franka_gazebo_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

UNRELEASED
----------

* chore: split ``franka_gazebo`` into a ``franka_gazebo`` metapackage grouping
  ``franka_gazebo_bringup`` (launch/world/urdf/config assets, this package) and
  ``franka_gazebo_hardware`` (the gz system plugin). The launch command
  ``ros2 launch franka_gazebo_bringup <file>`` is unchanged.
* feat: added a model-based gravity-compensation system plugin
  (``franka_gazebo_hardware::GazeboGravityCompensationSystem``) for gz_ros2_control that
  injects pinocchio-computed gravity torque on the effort-controlled arm joints. Gravity is
  now enabled globally in the Gazebo world (the per-link ``<gravity>false</gravity>``
  overrides were removed), so the zero-torque example controllers behave as on the real
  robot instead of collapsing. The mobile platform spine is held at its initial height by a
  ``spine_joint_trajectory_controller`` (JointTrajectoryController on a position command
  interface).

1.0.0 (2025-01-22)
------------------

