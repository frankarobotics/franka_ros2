franka_example_controllers
==========================

This package contains a few controllers that can be seen as example of how to write controllers in ROS 2. Currently,
a controller only has access to measured joint positions and joint velocities. Based on this information the controller
can send torque commands. It is currently not possible to use other interfaces like the joint position interface.

Example Controllers
-------------------

This repo comes with a few example controllers located in the ``franka_example_controllers`` package.

The following launch files are executed with the gripper by default. If you
do not have the gripper attached you can disable the gripper in the launch file with ``load_gripper:=false``.

Move-to-start
^^^^^^^^^^^^^

This controller moves the robot to its home configuration.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=move_to_start_example_controller

.. _gravity_example:

Gravity Compensation
^^^^^^^^^^^^^^^^^^^^

This is the simplest controller that we have and is a good starting point to write your own.
It sends zero as torque command to all joints, which means that the robot only compensates its own weight.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=gravity_compensation_example_controller

Gripper Example
^^^^^^^^^^^^^^^

Demonstrates the Franka *Action Interface* for controlling the Franka Hand (aka: Gripper). The controller submits *Goals* to repeatedly close, then reopen, the gripper given a hard-coded target grasp size with epsilon. It evaluates whether the grasp is successful or
failed based on the object's size and the defined tolerances.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=gripper_example_controller


Joint Impedance Example
^^^^^^^^^^^^^^^^^^^^^^^

The example moves joints 4 and 5 in a periodic movement that is very compliant. You can try to move the
joints while it is running.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=joint_impedance_example_controller

Joint Impedance FR3 Duo Example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This example is designed for the FR3 Duo (dual-arm) setup. It moves joints 4 and 5 on both arms
in a periodic, compliant movement. Only the torque (effort) command interface is supported.

.. code-block:: shell

    ros2 launch franka_bringup fr3_duo.launch.py \
        controller_name:=fr3_duo_joint_impedance_example_controller

.. note::

    Use ``controller_name`` (singular) for ``fr3_duo.launch.py``, not ``controller_names`` (plural).

Joint Impedance Mobile FR3 Duo Example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This example is designed for the Mobile FR3 Duo setup (TMRv0.2 mobile base with dual FR3 arms).
It combines:

* **Dual FR3 arms**: Joint impedance control moving joints 4 and 5 in periodic, compliant movement
* **Mobile base**: Cartesian velocity control for periodic forward/backward motion

The controller integrates both arm and mobile base control in a single unified controller.

.. code-block:: shell

    ros2 launch franka_bringup mobile_fr3_duo.launch.py \
        controller_name:=mobile_fr3_duo_joint_impedance_example_controller

**System Architecture:**

The Mobile FR3 Duo system consists of three hardware components:

* TMRv0.2 mobile base (controlled via GPIO cartesian velocity interfaces: ``vx``, ``vy``, ``vz``, ``wx``, ``wy``, ``wz``)
* Left FR3 arm (torque/effort interface)
* Right FR3 arm (torque/effort interface)

.. note::

    Use ``controller_name`` (singular) for ``mobile_fr3_duo.launch.py``, not ``controller_names`` (plural).

Joint Impedance With IK Example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The example uses the LMA-Orocos solver from MoveIt service to compute the joint positions for the desired pose.
The desired pose is to move the end-effector periodically in x and z directions. You can change the kinematic solver
in the franka_fr3_moveit_config package, kinematics.yaml file.

.. code-block:: shell

    ros2 launch franka_bringup joint_impedance_with_ik_example_controller.launch.py


Model Example Controller
^^^^^^^^^^^^^^^^^^^^^^^^
This is a read-only controller which prints the coriolis force vector, gravity force vector, pose matrix of Joint4,
Joint4 body jacobian and end-effector jacobian with respect to the base frame.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=model_example_controller

Joint Position Example
^^^^^^^^^^^^^^^^^^^^^^
This example sends periodic position commands to the robot.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=joint_position_example_controller

Joint Velocity Example
^^^^^^^^^^^^^^^^^^^^^^
This example sends periodic velocity commands to the 4th and 5th joint of the robot.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=joint_velocity_example_controller

Cartesian Pose Example
^^^^^^^^^^^^^^^^^^^^^^
This example uses the CartesianPose interface to send periodic pose commands to the robot.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=cartesian_pose_example_controller

Cartesian Orientation Example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This example uses CartesianOrientation interface to send periodic orientation commands around X axis of the end effector of the robot.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=cartesian_orientation_example_controller

Cartesian Pose Elbow Example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This example sends periodic elbow commands while keeping the end effector pose constant.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=cartesian_elbow_example_controller

Cartesian Velocity Example
^^^^^^^^^^^^^^^^^^^^^^^^^^
This example uses the CartesianVelocity interface to send periodic velocity commands to the robot.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=cartesian_velocity_example_controller

Cartesian Elbow Example
^^^^^^^^^^^^^^^^^^^^^^^
This example uses the CartesianElbow interface to send periodic elbow commands to the robot while keeping the end effector velocity constant.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=elbow_example_controller

Self Collision FR3 Duo Example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This example is designed for the FR3 Duo (dual-arm) setup. First they move to a start configuration, then they move toward a collision configuration, 
and if a collision is detected they immediately retreat back to the start position.

.. code-block:: shell

    ros2 launch franka_bringup fr3_duo.launch.py \
        controller_name:=fr3_duo_self_collision_example_controller

.. important::

    You must set ``check_selfcollision: true`` in ``fr3_duo.config.yaml`` before running this command.

Mobile Cartesian Velocity Example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This controller subscribes to a topic that publishes cartesian velocities and forwards them to the 
robot after going through a rate limiter.

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=mobile_cartesian_velocity_example_controller


Writing Custom Controllers
---------------------------

Compared to ``franka_ros`` we currently offer a reduced set of controller interfaces:

* Joint positions
* Joint velocities
* Measured torques
* Franka robot state
* Franka robot model

.. important::
    Franka robot state is published through :doc:`franka_robot_state_broadcaster <../../franka_robot_state_broadcaster/doc/index>`
    package to the topic named  `/franka_robot_state_broadcaster/robot_state`

.. important::
    Both Franka robot state and Franka robot model are advised to use through :doc:`franka_semantic_components <../../franka_semantic_components/doc/index>` class.
    They are stored in the state_interface as double pointers and casted back to their original objects inside the franka_semantic_component class.

    Example of using franka_model can be found in the franka_example_controllers package:
    `model_example_controller <https://github.com/frankarobotics/franka_ros2/blob/jazzy/franka_example_controllers/src/model_example_controller.cpp>`_.


You can base your own controller on one of the example controllers in this package. To compute kinematic
and dynamic quantities of the robot you can use the joint states and the URDF of the robot in libraries like
`KDL <https://www.orocos.org/kdl/user-manual>`_ (of which there is also a ROS 2 package available).