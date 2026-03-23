franka_bringup
==============

Installation
------------

Please refer to the `README.md <https://github.com/frankarobotics/franka_ros2/blob/jazzy/README.md>`_

Package Overview
----------------

This package contains the launch files for the examples as well as the basic ``franka.launch.py`` launch file, that
can be used to start the robot without any controllers.

When you start the robot with::

    ros2 launch franka_bringup franka.launch.py robot_type:=fr3 robot_ip:=<fci-ip>

There is no controller running apart from the ``joint_state_broadcaster``. However, a connection with the robot is still
established and the current robot pose is visualized in RViz. In this mode the robot can be guided when the user stop
button is pressed. However, once a controller that uses the ``effort_command_interface`` is started, the robot will be
using the torque interface from libfranka. For example it is possible to launch the
``gravity_compensation_example_controller`` by running::

    ros2 control load_controller --set-state active  gravity_compensation_example_controller

This is the equivalent of running the ``gravity_compensation_example_controller`` example mentioned in
:doc:`Gravity Compensation <../../franka_example_controllers/doc/index>`.

When the controller is stopped with::

    ros2 control set_controller_state gravity_compensation_example_controller inactive

the robot will stop the torque control and will only send its current state over the FCI.

You can now choose to start the same controller again with::

    ros2 control set_controller_state gravity_compensation_example_controller active

or load and start a different one::

    ros2 control load_controller --set-state active joint_impedance_example_controller


Namespace enabled launch files
------------------------------

To demonstrate how to launch the robot within a specified namespace, an example launch file is provided at
``franka_bringup/launch/example.launch.py``.

By default, ``example.launch.py`` reads robot configuration from ``franka_bringup/config/franka.config.yaml``.
You can provide a different YAML file by specifying its path in the command line.

The ``franka.config.yaml`` file specifies critical parameters, including:

* The path to the robot's URDF file.
* The namespace for the robot instance.
* Additional robot-specific configuration details.

``example.launch.py`` includes ``franka.launch.py``, which defines the core nodes for robot operation.
``franka.launch.py``, in turn, relies on ``controllers.yaml`` to configure the ``ros2_control`` framework.
This setup ensures that controllers are loaded in a namespace-agnostic manner, supporting consistent behavior across multiple namespaces.

The ``controllers.yaml`` file is designed to accommodate multiple namespaces, provided they share the same node configuration parameters.

Each configuration and launch file (``franka.config.yaml``, ``example.launch.py``, ``franka.launch.py``, and ``controllers.yaml``)
contains detailed inline documentation. For more information about namespaces in ROS 2, refer to the
`ROS 2 documentation <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#namespaces>`_.

To execute any of the example controllers defined in ``controllers.yaml``, use the ``example.launch.py`` launch file and specify
the controller name as a command-line argument.

First, modify ``franka.config.yaml`` as needed for your setup.

Then, to run the ``move_to_start_example_controller``, use the following command:

.. code-block:: shell

    ros2 launch franka_bringup example.launch.py controller_names:=move_to_start_example_controller

Non-realtime robot parameter setting
------------------------------------

Non-realtime robot parameter setting can be done via ROS 2 services. They are advertised after the robot hardware is initialized.

Service names are given below::

 * /service_server/set_cartesian_stiffness
 * /service_server/set_force_torque_collision_behavior
 * /service_server/set_full_collision_behavior
 * /service_server/set_joint_stiffness
 * /service_server/set_load
 * /service_server/set_parameters
 * /service_server/set_parameters_atomically
 * /service_server/set_stiffness_frame
 * /service_server/set_tcp_frame

Service message descriptions are given below.

 * ``franka_msgs::srv::SetJointStiffness`` specifies joint stiffness for the internal controller
   (damping is automatically derived from the stiffness).
 * ``franka_msgs::srv::SetCartesianStiffness`` specifies Cartesian stiffness for the internal
   controller (damping is automatically derived from the stiffness).
 * ``franka_msgs::srv::SetTCPFrame`` specifies the transformation from <robot_type>_EE (end effector) to
   <robot_type>_NE (nominal end effector) frame. The transformation from flange to end effector frame
   is split into two transformations: <robot_type>_EE to <robot_type>_NE frame and <robot_type>_NE to
   <robot_type>_link8 frame. The transformation from <robot_type>_NE to <robot_type>_link8 frame can only be
   set through the administrator's interface.
 * ``franka_msgs::srv::SetStiffnessFrame`` specifies the transformation from <robot_type>_K to <robot_type>_EE frame.
 * ``franka_msgs::srv::SetForceTorqueCollisionBehavior`` sets thresholds for external Cartesian
   wrenches to configure the collision reflex.
 * ``franka_msgs::srv::SetFullCollisionBehavior`` sets thresholds for external forces on Cartesian
   and joint level to configure the collision reflex.
 * ``franka_msgs::srv::SetLoad`` sets an external load to compensate (e.g. of a grasped object).

Launch franka_bringup/franka.launch.py file to initialize robot hardware::

    ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>

Here is a minimal example:

.. code-block:: shell

    ros2 service call /service_server/set_joint_stiffness \
      franka_msgs/srv/SetJointStiffness \
      "{joint_stiffness: [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]}"

.. important::

    Non-realtime parameter setting can only be done when the robot hardware is in `idle` mode.
    If a controller is active and claims command interface this will put the robot in the `move` mode.
    In `move` mode non-realtime param setting is not possible.

.. important::

    The <robot_type>_EE frame denotes the part of the
    configurable end effector frame which can be adjusted during run time through `franka_ros`. The
    <robot_type>_K frame marks the center of the internal
    Cartesian impedance. It also serves as a reference frame for external wrenches. *Neither the
    <robot_type>_EE nor the <robot_type>_K are contained in the URDF as they can be changed at run time*.
    By default, <robot_type> is set to "panda".

    .. figure:: ../../docs/assets/frames.svg
        :align: center
        :figclass: align-center

        Overview of the end-effector frames.

Non-realtime ROS 2 actions
--------------------------

Non-realtime ROS 2 actions can be done via the `ActionServer`. Following actions are available:

* ``/action_server/error_recovery`` - Recovers automatically from a robot error.

The used messages are:

* ``franka_msgs::action::ErrorRecovery`` - no parameters.

Example usage:::

    ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery {}

Known Issues
------------

* When using the ``fake_hardware`` with MoveIt, it takes some time until the default position is applied.
