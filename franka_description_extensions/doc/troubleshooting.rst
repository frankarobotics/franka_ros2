.. _franka_description_extension_troubleshooting:

Troubleshooting
===============

Unknown ``gazebo`` or ``gazebo_effort`` argument
------------------------------------------------

The standalone extension descriptions are Layer-2 geometry compositions. They
do not declare or forward a ``gazebo`` argument, and they do not contain
Gazebo tags. Use the matching
``franka_gazebo_bringup`` launch file for simulation:

.. code-block:: shell

   ros2 launch franka_gazebo_bringup gazebo_mobile_fr3_duo_example.launch.py \
       with_sensors:=true

If an expansion still reports the removed argument after updating the source,
the command may be resolving an older installed copy of an extension package.
Rebuild the affected packages with ``--symlink-install`` and source the
resulting workspace before expanding again.

An accessory parent frame is missing
------------------------------------

Check the expanded parent rather than the symbolic input:

* a single arm with ``arm_prefix:=left`` and
  ``robot_type:=fr3v2_1`` uses ``left_fr3v2_1_link8``;
* stationary vision-kit arms with ``prefix:=kit_`` use
  ``kit_left_fr3v2_1_link8`` and ``kit_right_fr3v2_1_link8``; and
* mobile vision-kit arms use ``left_fr3v2_1_link8`` and
  ``right_fr3v2_1_link8`` because the current mobile macro fixes the arm
  prefixes to ``left`` and ``right``.

The mobile sensor suite uses unprefixed TMR frames such as
``front_mounting_point`` and ``lidar_rear_mounting_point``. Those frames are
provided by the TMR/mobile base descriptions and are not created by the
sensor package alone. See :doc:`Mounting-point catalog
<mounting_points>`.

The vision kit creates duplicate head links
--------------------------------------------

``fr3_duo`` already calls the ``head`` macro and creates
``head_link``, ``head_camera_mounting_point``, and their joints. The vision
kit only attaches its ZED camera to the existing
``head_camera_mounting_point``. Remove any extra ``xacro:head`` call from a
wrapper.

The supplied vision accessories do not match the arm links
----------------------------------------------------------

Leave ``no_prefix`` at its default ``false`` for the supplied vision kit.
Its component macro constructs the arm parents from the current prefixed
robot type. Setting ``no_prefix:=true`` changes the base arm to unprefixed
``link8`` while the accessory macro still asks for names such as
``left_fr3v2_1_link8``.

For a custom accessory, calculate the expanded prefix once and use the same
value for the base arm's ``arm_prefix`` and the accessory's fixed-joint
parent. Do not prepend an additional prefix to a value already expanded by a
duo macro.

``special_connection`` did not move my external gripper
--------------------------------------------------------

``special_connection`` is consumed by the upstream built-in end-effector
path. It changes the parent selected by ``franka_robot`` when the built-in
end-effector is enabled; it does not change the ``parent`` passed to an
external gripper macro. Disable the built-in end effector with
``hand="false"`` and ``ee_id="none"``, then attach the external macro
directly to the expanded ``link8`` frame. See
:doc:`Attaching a third-party gripper <custom_grippers>`.

Mesh or inertial data cannot be found
-------------------------------------

Keep the arm description on ``franka_description``. The current
``description_pkg`` parameter is forwarded through the outer and built-in
end-effector calls, but the base macros still use ``franka_description`` for
the arm's visual resources, several YAML files, collision meshes, and include
paths. It is not a complete replacement-package mechanism.

Third-party gripper visual and collision resources should use the third-party
package's own ``package://`` URIs, and its macro should emit the inertial
values and TCP frame. Do not rely on an arbitrary third-party ``ee_id`` being
registered by ``franka_description``.

The description expands but no controller starts
------------------------------------------------

URDF expansion proves the geometry and fixed joints only. Add the
``<ros2_control>`` component, hardware plugin, controller configuration, and
driver startup in a separate runtime overlay. For the supplied vision kit,
``ros2_control`` controls the optional Robotiq block; it is not a replacement
for the arm hardware composition.

Use ``franka_bringup``/``franka_hardware`` for real or mock hardware and
``franka_gazebo_bringup`` for Gazebo. A standalone Layer-2 xacro does not
start a controller.

Launch configuration is ignored
--------------------------------

For ``franka_mobile_sensors``:

* ``config_file`` is a file name resolved inside the package's ``config``
  directory and may omit ``.yaml``;
* ``robot_xacro`` is the file name of the model used by RViz and for the
  sensor frames; and
* ``start_cameras``, ``start_lidars``, ``start_rviz``, and
  ``publish_sensor_tf`` control runtime nodes, not the links emitted by the
  xacro.

For ``franka_vision_and_manipulation_kit``:

* ``config_file_path`` and ``xacro_file_path`` can be filesystem paths or
  ``package://`` resources;
* ``use_mobile_platform`` is mapped to the xacro's
  ``mobile_platform`` argument; and
* ``start_robotiq_grippers``, ``start_realsense_cameras``,
  ``start_zed_camera``, and ``start_rviz`` control driver or visualization
  processes after the robot description is generated.

The grouping folder is not a ROS package
-----------------------------------------

Use the child package names in commands and resource lookups:
``franka_mobile_sensors`` and
``franka_vision_and_manipulation_kit``. The source path
``franka_description_extensions/`` organizes those packages but is not a
replacement package name and is not used in ``$(find ...)`` or
``package://`` URIs.

