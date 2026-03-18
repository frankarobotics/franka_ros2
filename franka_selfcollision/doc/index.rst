franka_selfcollision
====================

This package provides self-collision checking for the FR3 Duo and Mobile FR3 Duo robots. It exposes
a shared ``SelfCollisionChecker`` library and two executable nodes — one for each robot variant.

.. important::

    Minimum necessary `franka_description` version is 2.3.2.
    You can clone franka_description package from https://github.com/frankarobotics/franka_description.

Architecture
------------

The package is structured around a single unified ``SelfCollisionChecker`` class that handles both
robot variants:

* ``self_collision_checker.hpp/.cpp`` — unified collision checker with an optional ``link_filter``
  parameter. When provided, only collision pairs where both geometry names satisfy the predicate are
  kept. This is used by the mobile node to restrict checking to arm and spine links only.
* ``self_collision_node.hpp`` — shared node header used by both executables.
* ``self_collision_node.cpp`` — FR3 Duo executable. Uses sequential joint indexing. No link filter.
* ``mobile_self_collision_node.cpp`` — Mobile FR3 Duo executable. Uses ``idx_q``-based joint
  indexing and initializes the full configuration vector from the neutral pose so that non-tracked
  joints (base, casters, TMR drivetrain) remain safely at neutral throughout.

Functionality
-------------

Each node continuously monitors the robot's joint states and checks for self-collisions between
the relevant links. Upon detecting a collision or a violation of the security margin it:

1. **Publishes status** — sends a boolean to the collision topic.
2. **Logs warning** — prints the specific colliding link pairs to the console if enabled
   (throttled to 500ms to prevent spam).

The published topics are:

* FR3 Duo: ``/fr3_duo_self_collision_node/collision_detected``
* Mobile FR3 Duo: ``/mobile_fr3_duo_self_collision_node/collision_detected``

Configuration
-------------

Parameters are defined in ``config/self_collision_node.yaml``:

* ``security_margin``: Safety buffer around the robot links in meters (default: ``0.045``).
* ``print_collisions``: If ``true``, logs the names of the colliding links to the console.
* ``robot_description_semantic``: SRDF XML used to exclude allowable collision pairs.

Usage
-----

Both nodes are automatically started when the robot is launched with ``check_selfcollision`` set
to ``true``:

.. code-block:: shell

    # FR3 Duo
    ros2 launch franka_bringup fr3_duo.launch.py \
        check_selfcollision:=true

    # Mobile FR3 Duo
    ros2 launch franka_bringup mobile_fr3_duo.launch.py \
        check_selfcollision:=true