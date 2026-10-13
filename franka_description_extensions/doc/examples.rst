.. _franka_description_extension_examples:

Worked examples and entry points
================================

The source package names are unchanged by the
``franka_description_extensions/`` grouping folder. Use
``franka_mobile_sensors`` and
``franka_vision_and_manipulation_kit`` in ROS commands.

Standalone description entry points
-----------------------------------

The mobile sensor package installs these description files:

* ``franka_mobile_sensors/robots/tmrv0_2_with_sensors.urdf.xacro``
  exposes ``robot_type`` (default ``tmrv0_2``), ``reduced_version``
  (default ``false``), and ``robot_namespace`` (default empty).
* ``franka_mobile_sensors/robots/mobile_fr3_duo_v0_2_with_sensors.urdf.xacro``
  exposes the mobile duo description arguments, including
  ``robot_types`` (default
  ``['tmrv0_2','fr3v2','fr3v2']``), ``hand``, ``ee_id``,
  ``special_connection``, ``no_prefix``, ``with_sc``,
  ``include_self_collision_geometry``, ``description_pkg``,
  ``connected_to``, ``reduced_version``, and ``use_arms``. The transform and
  collision arguments are ``xyz_ee``, ``rpy_ee``, ``tcp_xyz``, ``tcp_rpy``,
  and ``safety_distance``.

The vision kit installs:

``franka_vision_and_manipulation_kit/robots/vision_and_manipulation_kit.urdf.xacro``

Its xacro arguments include ``prefix``, ``mobile_platform``,
``mobile_platform_type`` (default ``tmrv0_2``), ``left_arm_type`` and
``right_arm_type`` (default ``fr3v2_1``), ``no_prefix``,
``special_connection``, ``xyz_ee``, ``rpy_ee``, ``tcp_xyz``, ``tcp_rpy``,
``safety_distance``, ``with_sc``, ``include_self_collision_geometry``,
``ros2_control``, the three ``*_ip`` arguments, ``use_fake_hardware``,
``fake_sensor_commands``, and ``description_pkg``. These are xacro
arguments; the ROS launch file exposes only the launch arguments described
below and maps ``use_mobile_platform`` to ``mobile_platform``.

Mobile sensor suite
-------------------

Start the physical mobile sensor drivers and RViz with the default
configuration:

.. code-block:: shell

   ros2 launch franka_mobile_sensors franka_mobile_sensors.launch.py

The launch file declares:

* ``start_cameras`` (default ``true``);
* ``start_lidars`` (default ``true``);
* ``start_rviz`` (default ``true``);
* ``publish_sensor_tf`` (default ``true``, sensor frames in TF when RViz is off);
* ``config_file`` (default ``default_sensor_suite``, without ``.yaml``); and
* ``robot_xacro`` (default ``tmrv0_2_with_sensors.urdf.xacro``).

For example, start only the camera drivers and omit RViz:

.. code-block:: shell

   ros2 launch franka_mobile_sensors franka_mobile_sensors.launch.py \
       start_lidars:=false start_rviz:=false

The default configuration contains four D455 cameras, two nanoScan3
LiDARs, and the OLV-IMU01 IMU. Camera and LiDAR nodes load their device
profiles from the ``config/cameras`` and ``config/lidars`` directories.
``config_file`` is resolved by ``franka_mobile_sensors`` and may omit the
``.yaml`` suffix.

For the package-specific launch arguments and configuration layout, see
:doc:`Mobile sensor package documentation <../franka_mobile_sensors/doc/index>`.

Vision and Manipulation Kit
---------------------------

Start the kit's robot-state publisher and enabled drivers with the default
configuration:

.. code-block:: shell

   ros2 launch franka_vision_and_manipulation_kit \
       franka_vision_and_manipulation_kit.launch.py

The launch file declares:

* ``start_robotiq_grippers`` (default ``true``);
* ``start_realsense_cameras`` (default ``true``);
* ``start_zed_camera`` (default ``false``);
* ``start_rviz`` (default ``true``);
* ``use_mobile_platform`` (default ``false``);
* ``config_file_path`` (default
  ``package://franka_vision_and_manipulation_kit/config/default_config.yaml``);
  and
* ``xacro_file_path`` (default
  ``package://franka_vision_and_manipulation_kit/robots/vision_and_manipulation_kit.urdf.xacro``).

Use the mobile description and start the ZED driver without starting RViz:

.. code-block:: shell

   ros2 launch franka_vision_and_manipulation_kit \
       franka_vision_and_manipulation_kit.launch.py \
       use_mobile_platform:=true start_zed_camera:=true start_rviz:=false

The default YAML has ``head``, ``left_arm``, and ``right_arm`` sections.
Each arm section contains a Robotiq ``gripper`` configuration and a
RealSense ``camera`` configuration. The ``config_file_path`` and
``xacro_file_path`` arguments accept a filesystem path or a
``package://`` resource.

For the complete prerequisite and configuration details, see
:doc:`Vision and Manipulation Kit package documentation
<../franka_vision_and_manipulation_kit/doc/index>`.

Gazebo composition
------------------

Gazebo uses Layer-3 entry points from ``franka_gazebo_bringup``. Do not pass
``with_sensors`` or ``gazebo_effort`` to either standalone Layer-2 xacro.
Use the Gazebo launch files instead:

.. code-block:: shell

   ros2 launch franka_gazebo_bringup gazebo_fr3_duo_example.launch.py \
       with_sensors:=true

This composes the two-arm vision kit. For the mobile base plus both sensor
suites, use:

.. code-block:: shell

   ros2 launch franka_gazebo_bringup gazebo_mobile_fr3_duo_example.launch.py \
       with_sensors:=true

The mobile Gazebo entry point composes the seven mobile-platform sensors with
the two wrist cameras and one head camera from the vision kit. It also
selects the Gazebo worlds, hardware plugins, transmissions, and controller
configuration owned by ``franka_gazebo_bringup``. See
:doc:`Gazebo bringup documentation
<../../franka_gazebo/franka_gazebo_bringup/doc/index>` for its launch arguments.

