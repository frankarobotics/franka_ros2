franka_selfcollision
====================

This package contains the library and the node for self-collision checking of the FR3 Duo and
Mobile FR3 Duo robots.

.. important::

    Minimum necessary `franka_description` version is 2.3.2.
    You can clone franka_description package from https://github.com/frankarobotics/franka_description.

Functionality
-------------

This monitoring node is spawned by ``fr3_duo.launch.py`` or ``mobile_fr3_duo.launch.py`` in ``franka_bringup`` if the ``check_selfcollision`` argument is enabled.

The node continuously monitors the robot's joint states to check for self-collisions between the robot links. It handles both ``fr3_duo`` and ``mobile_fr3_duo`` configurations.
It performs two main actions upon detecting a collision (or violation of the security margin):

1. **Publishes Status:** Sends a boolean to the topic ``~/<node_name>/collision_detected``
   (where ``<node_name>`` is set via the ``name`` parameter in the launch file, default: ``self_collision_node``).
   The topic uses **best effort** QoS (``rclcpp::SensorDataQoS``). Subscribers must use a
   compatible QoS profile (e.g. ``rclcpp::SensorDataQoS()`` in C++ or
   ``qos_profile_sensor_data`` in Python) to receive messages.
2. **Logs Warning:** Prints the specific colliding link pairs to the console if enabled (throttled to 1Hz to prevent spam).

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