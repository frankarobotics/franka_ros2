franka_fr3_moveit_config
========================

This package contains the configuration for MoveIt2.

Move Group
----------

There is one move group called ``(robot_type)_arm``, where ``(robot_type)`` is the arm type specified by the user.
The TCP placement varies depending on the end-effector configuration. If the Franka Hand is set, the TCP is located
in the center between the fingertips and the TCP-frame-axes are in parallel with the gripper frame.
Otherwise, the TCP is located at the robot flange.

.. figure:: ../../docs/assets/move_groups.png
    :align: center
    :figclass: align-center

    Visualization of the different TCP placements.

Usage
-----

To see if everything works, you can try to run the MoveIt example on the robot::

    ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=<fci-ip>

Then activate the ``MotionPlanning`` display in RViz.

If you do not have a robot you can still test your setup by running on a dummy hardware::

    ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

Wait until you can see the green ``You can start planning now!`` message from MoveIt inside the
terminal. Then turn off the ``PlanningScene`` and turn it on again. After that turn on the ``MotionPlanning``.

You can use the arguments ``load_gripper`` to activate or deactivate the end-effector and ``ee_id`` to set
which end-effector you want to use. By default, the Franka Hand is activated.

Configuration Files
-------------------

This package includes:

* Motion planning configuration for the FR3 robot
* Joint limits and safety settings
* Planning groups and link configurations
* Kinematics solver configuration (kinematics.yaml)

For the Joint Impedance With IK Example controller, you can change the kinematic solver
in this package's kinematics.yaml file.
