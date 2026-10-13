franka_mobile_sensors
=====================

Package Overview
----------------

This package contains launch files and runtime configuration for the sensor suite on Franka
Robotics mobile research robots. It integrates:

* **RealSense Cameras** - Intel RealSense depth cameras (D455)
* **SICK Safety Scanners** - SICK nanoScan3 safety lidars
* **Olive Robotics IMU** - Olive Robotics olixSense X1 IMU

The package handles sensor configuration, driver startup, and RViz visualization for the
complete suite.

Usage
-----

Start the complete sensor suite with the default configuration:

.. code-block:: shell

    ros2 launch franka_mobile_sensors franka_mobile_sensors.launch.py \
        config_file:=default_sensor_suite \
        robot_xacro:=tmrv0_2_with_sensors.urdf.xacro

Launch Arguments
^^^^^^^^^^^^^^^^

* ``start_cameras``

  * Type: ``bool``
  * Default: ``true``
  * Description: Whether to start the RealSense camera drivers.

* ``start_lidars``

  * Type: ``bool``
  * Default: ``true``
  * Description: Whether to start the SICK safety scanner drivers.

* ``start_rviz``

  * Type: ``bool``
  * Default: ``true``
  * Description: Whether to start RViz visualization.

* ``publish_sensor_tf``

  * Type: ``bool``
  * Default: ``true``
  * Description: Whether to publish the sensor frames (e.g. ``lidar_front``,
    ``lidar_rear``) to TF when ``start_rviz`` is ``false``; with RViz, its
    ``robot_state_publisher`` publishes them. This ``robot_state_publisher``
    runs as ``/sensors/robot_state_publisher`` with its tree rooted at
    ``base_link``. Next to ``tmrv0_2.launch.py``, set ``start_rviz:=false``:
    the RViz path starts its own ``robot_state_publisher`` and
    ``joint_state_publisher_gui`` and is meant for standalone visualization.

* ``config_file``

  * Type: ``string``
  * Default: ``default_sensor_suite``
  * Description: Configuration file to use, without the ``.yaml`` extension.

* ``robot_xacro``

  * Type: ``string``
  * Default: ``tmrv0_2_with_sensors.urdf.xacro``
  * Description: XACRO file for the robot model used in RViz and for the sensor frames.
    A file used for the sensor frames must be rooted at ``base_link``, so
    ``base`` -> ``base_link`` stays with the swerve odometry.

Configuration
-------------

Sensor Suite Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The main sensor suite is configured in:

* ``config/default_sensor_suite.yaml`` - Defines which sensors are used

This file specifies:

* List of cameras with their device profiles
* List of lidars with their device profiles
* Network configurations
* Sensor placement and mounting parameters

Device-Specific Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Individual device parameters are configured in device profile files:

**Cameras:**

* ``config/cameras/franka_mobile_d455.yaml`` - RealSense D455 parameters

**Lidars:**

* ``config/lidars/sick_nanoscan3.yaml`` - SICK nanoScan3 parameters

Custom Configurations
^^^^^^^^^^^^^^^^^^^^^

To create a custom configuration:

1. Copy ``config/default_sensor_suite.yaml`` to ``config/my_custom_suite.yaml``
2. Modify camera/lidar lists and reference existing or new device profiles
3. Launch the suite with your new configuration:

   .. code-block:: shell

      ros2 launch franka_mobile_sensors franka_mobile_sensors.launch.py \
          config_file:=my_custom_suite

Examples
--------

Start only the cameras:

.. code-block:: shell

    ros2 launch franka_mobile_sensors franka_mobile_sensors.launch.py \
        start_cameras:=true \
        start_lidars:=false \
        start_rviz:=false

Use a custom sensor configuration:

.. code-block:: shell

    ros2 launch franka_mobile_sensors franka_mobile_sensors.launch.py \
        config_file:=my_custom_suite \
        robot_xacro:=tmrv0_2_with_sensors.urdf.xacro

Gazebo Simulation
-----------------

The mobile sensor suite can be simulated in Gazebo without physical hardware. Use the
``with_sensors:=true`` argument on the Mobile FR3 Duo Gazebo launch file:

.. code-block:: shell

    ros2 launch franka_gazebo_bringup gazebo_mobile_fr3_duo_example.launch.py with_sensors:=true

This simulates all 7 mobile platform sensors (4× D455 cameras, 2× LiDARs, 1× IMU) with
proper kinematic attachment and topic bridging. See the
:doc:`franka_gazebo_bringup documentation <../../../franka_gazebo/franka_gazebo_bringup/doc/index>`
for full details on available topics and launch arguments.
