^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package franka_selfcollision
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-03-18)
------------------
* Extended self-collision checking to the Mobile FR3 Duo robot.
* Unified ``SelfCollisionChecker`` with optional ``link_filter`` parameter — replaces separate
  mobile checker class.
* Added ``mobile_self_collision_node`` executable with ``idx_q``-based joint indexing and
  neutral-configuration initialization for non-tracked joints.
* Added mobile self-collision test fixture with ``GTEST_SKIP`` guard until test assets are present.

1.0.0 (2025-01-22)
------------------