franka_hardware
===============

.. important::
    Breaking changes as of 0.1.14 release: ``franka_hardware`` robot_state and robot_model will be prefixed by the ``robot_type``.

        - ``panda/robot_model  -> ${robot_type}/robot_model``
        - ``panda/robot_state  -> ${robot_type}/robot_state``

    There is no change with the state and command interfaces naming. They are prefixed with the joint names in the URDF.

Package Overview
----------------

This package contains the ``franka_hardware`` plugin needed for `ros2_control <https://control.ros.org/jazzy/index.html>`_.
The plugin is loaded from the URDF of the robot and passed to the controller manager via the robot description.

Hardware Interfaces
--------------------

The hardware plugin provides for each joint:

* a ``position state interface`` that contains the measured joint position.
* a ``velocity state interface`` that contains the measured joint velocity.
* an ``effort state interface`` that contains the measured link-side joint torques.
* an ``initial_position state interface`` that contains the initial joint position of the robot.
* an ``effort command interface`` that contains the desired joint torques without gravity.
* a  ``position command interface`` that contains the desired joint position.
* a  ``velocity command interface`` that contains the desired joint velocity.

Additional State Interfaces
---------------------------

In addition to joint interfaces, the hardware plugin provides:

* a ``franka_robot_state`` that contains the robot state information, `franka_robot_state <https://github.com/frankarobotics/franka_ros2/blob/jazzy/franka_msgs/msg/FrankaRobotState.msg>`_.
* a ``franka_robot_model_interface`` that contains the pointer to the model object.
* a ``ForceTorqueSensor`` (``<arm_prefix><robot_type>_tcp``) that exposes the estimated
  external wrench in the stiffness frame (``K_F_ext_hat_K`` from libfranka) as six state
  interfaces: ``force.x``, ``force.y``, ``force.z``, ``torque.x``, ``torque.y``, ``torque.z``.

.. important::
    ``franka_robot_state`` and ``franka_robot_model_interface`` state interfaces should not be used directly from hardware state interface.
    Rather, they should be utilized by the :doc:`franka_semantic_components <../../franka_semantic_components/doc/index>` interface.

    The ``ForceTorqueSensor`` interfaces follow the standard ``ros2_control`` sensor convention
    and can be consumed directly via the ``semantic_components::ForceTorqueSensor`` component
    in any controller (e.g. the admittance controller) without requiring a topic bridge.
    See the gravity compensation example controller for usage.

ros2_control Macro Library
--------------------------

This package owns the ``ros2_control`` xacro macro library used to declare hardware interfaces
for all Franka robot configurations. The macros live in ``franka_hardware/ros2_control/``:

* ``franka_ros2_control_macros.xacro`` — shared building blocks (``configure_arm_joints``,
  ``configure_finger_joint``, ``configure_steering_joint``, ``configure_driving_joint``,
  ``configure_mobile_drive_joints``, ``configure_passive_mobile_base_joints``,
  ``general_purpose_io``, ``cartesian_velocity_io``, ``cartesian_pose_loop``,
  ``configure_arm_interfaces``, etc.)
* ``franka_arm.ros2_control.xacro`` — single-arm configuration
* ``tmrv0_2.ros2_control.xacro`` — standalone TMR base

These are composed with ``franka_description`` robot models via thin wrappers in
``franka_bringup/urdf/`` to produce complete robot descriptions with hardware interfaces.

Configuration
-------------

The IP of the robot is read over a parameter from the URDF.

Usage with Controllers
----------------------

Controllers can access these interfaces through the standard ros2_control framework. For examples of how to use these interfaces in practice, see the :doc:`franka_example_controllers <../../franka_example_controllers/doc/index>` package.