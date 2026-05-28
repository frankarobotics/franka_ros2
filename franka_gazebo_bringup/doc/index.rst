franka_gazebo
=============

.. important::

    Minimum necessary `franka_description` version is 0.3.0.
    You can clone franka_description package from https://github.com/frankarobotics/franka_description.

A project integrating Franka ROS 2 with the Gazebo simulator.

Launch RVIZ + Gazebo
--------------------

Launch an example which spawns RVIZ and Gazebo showing the robot:


.. code-block:: shell

    ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py

If you want to display another robot, you can define the robot_type:

.. code-block:: shell

    ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py robot_type:=fp3

If you want to start the simulation including the franka_hand:

.. code-block:: shell

    ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py load_gripper:=true franka_hand:='franka_hand'

Joint Velocity Control Example with Gazebo
-------------------------------------------

Before starting, be sure to build `franka_example_controllers` and `franka_description` packages.
`franka_description` must have the minimum version of 0.3.0.


.. code-block:: shell

    colcon build --packages-select franka_example_controllers


Now you can launch the velocity example with Gazebo simulator.

.. code-block:: shell

    ros2 launch franka_gazebo_bringup gazebo_franka_arm_example_controller.launch.py load_gripper:=true franka_hand:='franka_hand' controller:='joint_velocity_example_controller'


Keep in mind that the gripper joint has a bug with the joint velocity controller.
If you are interested in controlling the gripper please use joint position interface.


Joint Position Control Example with Gazebo
-------------------------------------------

To run the joint position control example you need to have the required software listed in the joint velocity control section.

Then you can run with the following command.

.. code-block:: shell

    ros2 launch franka_gazebo_bringup gazebo_franka_arm_example_controller.launch.py load_gripper:=true franka_hand:='franka_hand' controller:='joint_position_example_controller'


Joint Impedance Control Example with Gazebo
--------------------------------------------

Source your workspace.

.. code-block:: shell

    source install/setup.sh

Then you can run the impedance control example.

.. code-block:: shell

    ros2 launch franka_gazebo_bringup gazebo_franka_arm_example_controller.launch.py load_gripper:=true franka_hand:='franka_hand' controller:='joint_impedance_example_controller'

FR3 Duo Example with Gazebo
---------------------------

Before starting, be sure to build ``franka_example_controllers``, ``franka_gazebo_bringup``,
``gz_ros2_control`` and ``franka_description`` packages.

.. code-block:: shell

    colcon build --packages-select franka_example_controllers franka_gazebo_bringup franka_description gz_ros2_control
    source install/setup.bash

Now you can launch the FR3 duo example with Gazebo:

.. code-block:: shell

    ros2 launch franka_gazebo_bringup gazebo_fr3_duo_example.launch.py

To launch with the complete sensor suite including the Vision and Manipulation Kit sensors,
also build ``franka_vision_and_manipulation_kit``:

.. code-block:: shell

    colcon build --packages-select franka_vision_and_manipulation_kit
    source install/setup.bash
    ros2 launch franka_gazebo_bringup gazebo_fr3_duo_example.launch.py with_sensors:=true

.. note::

   The sensor suite integrates:

   - **franka_vision_and_manipulation_kit** provides 3 sensors (2 wrist D405 cameras, 1 ZED Mini head camera)

   All sensors are properly attached to the robot kinematic tree, ensuring proper simulation and sensor data streaming.

   **Important**: When using ``with_sensors:=true``, the Vision and Manipulation Kit includes Robotiq grippers.

**Sensor Configuration with** ``with_sensors:=true``:

This command enables:

**Vision and Manipulation Kit Sensors** (from ``franka_vision_and_manipulation_kit``):
  - 2x RealSense D405 cameras (left and right wrist cameras)
  - 1x ZED Mini camera (head camera)

**Topics available:**

Wrist cameras (Vision and Manipulation Kit):
  - ``/left_wrist_camera/image_raw``, ``/right_wrist_camera/image_raw``

Head camera (ZED Mini):
  - ``/head_camera/image_raw``, ``/head_camera/image_raw/camera_info``

**Arguments:**

- ``with_sensors``: If set to ``true``, uses the complete sensor-enhanced description from the Vision and Manipulation Kit sensors (``franka_vision_and_manipulation_kit``)
  with Gazebo sensor plugins. Defaults to ``false``.
- ``world``: SDF world filename inside ``franka_gazebo_bringup/worlds/`` to load.
  Overrides the default world selection.

This will spawn two FR3 arms with gripper and wrist cameras, and start the joint impedance controller
for both arms. RViz will also launch for visualization.

Mobile FR3 Duo Example with Gazebo
-----------------------------------

Before starting, be sure to build ``franka_example_controllers``, ``franka_gazebo_bringup``,
``gz_ros2_control`` and ``franka_description`` packages.

.. code-block:: shell

    colcon build --packages-select franka_example_controllers franka_gazebo_bringup franka_description gz_ros2_control
    source install/setup.bash

Now you can launch the mobile FR3 duo example with Gazebo:

.. code-block:: shell

    ros2 launch franka_gazebo_bringup gazebo_mobile_fr3_duo_example.launch.py

.. note::

   In simulation, a stub publisher sends ``false`` on ``/collision_detected`` with **best effort**
   QoS at 10 Hz, replacing the real self-collision node. This prevents the impedance controller
   from timing out on the collision topic.

To launch with the complete sensor suite including both the mobile platform sensors and the Vision and Manipulation Kit sensors,
also build ``franka_mobile_sensors`` and ``franka_vision_and_manipulation_kit``:

.. code-block:: shell

    colcon build --packages-select franka_mobile_sensors franka_vision_and_manipulation_kit
    source install/setup.bash
    ros2 launch franka_gazebo_bringup gazebo_mobile_fr3_duo_example.launch.py with_sensors:=true

.. note::

   The sensor suite integrates 10 sensors total from two packages:

   - **franka_mobile_sensors** provides 7 sensors (4 RGB cameras, 2 LiDARs, 1 IMU)
   - **franka_vision_and_manipulation_kit** provides 3 sensors (2 wrist D405 cameras, 1 ZED Mini head camera)

   All sensors are properly attached to the robot kinematic tree, ensuring proper simulation and sensor data streaming.

   **Important**: When using ``with_sensors:=true``, the Vision and Manipulation Kit includes Robotiq grippers.

**Sensor Configuration with** ``with_sensors:=true``:

This command enables BOTH sensor suites:

**Mobile Platform Sensors** (from ``franka_mobile_sensors``):
  - 4x RealSense D455 cameras (front, rear, left, right)
  - 2x SICK nanoScan3 LiDARs (front, rear)
  - 1x OLV-IMU01 IMU

**Vision and Manipulation Kit Sensors** (from ``franka_vision_and_manipulation_kit``):
  - 2x RealSense D405 cameras (left and right wrist cameras)
  - 1x ZED Mini camera (head camera)

**Topics available:**

Mobile platform cameras:
  - ``/camera_front/color/image_raw``, ``/camera_rear/color/image_raw``, etc.

Mobile platform LiDARs:
  - ``/lidar_front/scan``, ``/lidar_rear/scan``

Mobile platform IMU:
  - ``/imu/data``

Wrist cameras (Vision and Manipulation Kit):
  - ``/left_wrist_camera/image_raw``, ``/right_wrist_camera/image_raw``

Head camera (ZED Mini):
  - ``/head_camera/image_raw``, ``/head_camera/image_raw/camera_info``

**Arguments:**

- ``with_sensors``: If set to ``true``, uses the complete sensor-enhanced description with both mobile platform sensors
  (``franka_mobile_sensors``) and Vision and Manipulation Kit sensors (``franka_vision_and_manipulation_kit``)
  with Gazebo sensor plugins. Defaults to ``false``.
- ``world``: SDF world filename inside ``franka_gazebo_bringup/worlds/`` to load.
  Overrides the default world selection.

This will spawn the mobile base and two FR3 arms with gripper and wrist cameras, and start the joint impedance controller
for both arms and cartesian velocity control for the mobile base. RViz will also launch
for visualization. Select ``base_link`` to see the robot there.


Troubleshooting
---------------

If you experience that Gazebo can't find your model files, try to include the workspace. E.g.


.. code-block:: shell

    export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/workspaces/src/
