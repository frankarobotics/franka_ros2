franka_selfcollision
====================

This package contains the library and the service for the FR3_duo self-collision check.

.. important::

    Minimum necessary `franka_description` version is 2.3.2.
    You can clone franka_description package from https://github.com/frankarobotics/franka_description.

Functionality
-------------

This monitoring node is spawned by ``fr3_duo.launch.py`` in ``franka_bringup`` if the ``check_selfcollision`` argument is enabled.

The node continuously monitors the robot's joint states to check for self-collisions between the FR3_duo links.
It performs two main actions upon detecting a collision (or violation of the security margin):

1. **Publishes Status:** Sends a boolean to the topic ``/fr3_duo_self_collision_node/collision_detected``.
2. **Logs Warning:** Prints the specific colliding link pairs to the console if enabled (throttled to 1Hz to prevent spam).

Configuration
-------------

Parameters are defined in ``config/self_collision_node.yaml``:

* ``security_margin``: Safety buffer around the robot links in meters (default: ``0.045``).
* ``print_collisions``: If ``true``, logs the names of the colliding links to the console.

Usage
-----

The self-collision node is automatically started when you launch the robot with ``check_selfcollision`` set to true. 












FATMA
----
self_collision_checker.hpp/.cpp     ← unified, with optional link_filter param
collision_monitor_node.hpp          ← one shared node header
self_collision_node.cpp             ← fr3 duo main(), sequential joint indexing
mobile_self_collision_node.cpp      ← mobile main(), idx_q indexing + neutral init