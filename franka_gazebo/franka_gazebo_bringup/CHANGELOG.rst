^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package franka_gazebo_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

UNRELEASED
----------

* chore: split ``franka_gazebo`` into a metapackage plus ``franka_gazebo_bringup``
  (launch files, worlds, robot descriptions, controller configs) and
  ``franka_gazebo_hardware`` (the gz_ros2_control gravity-compensation system plugin).
  The public ``ros2 launch franka_gazebo_bringup ...`` command is unchanged.

1.0.0 (2025-01-22)
------------------

