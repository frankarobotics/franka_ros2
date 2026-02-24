franka_gripper
==============

This package contains the ``franka_gripper_node`` for interfacing with the ``Franka Hand``.

Actions and Services
--------------------

The ``franka_gripper_node`` provides the following actions:

* ``homing`` - homes the gripper and updates the maximum width given the mounted fingers.
* ``move`` - moves to a target width with the defined speed.
* ``grasp`` - tries to grasp at the desired width with the desired force while closing with the given speed. The operation is successful if the
  distance ``d`` between the gripper fingers is ``width - epsilon.inner < d < width + epsilon.outer``
* ``gripper_action`` - a special grasping action for MoveIt.

Also, there is a ``stop`` service that aborts gripper actions and stops grasping.

Usage
-----

Use the following launch file to start the gripper::

    ros2 launch franka_gripper gripper.launch.py robot_ip:=<fci-ip>


In a different tab you can now perform the homing and send a grasp command.::


    ros2 action send_goal /franka_gripper/homing franka_msgs/action/Homing {}
    ros2 action send_goal -f /franka_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 50}"

The inner and outer epsilon are 0.005 meter per default. You can also explicitly set the epsilon::

    ros2 action send_goal -f /franka_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 50, epsilon: {inner: 0.01, outer: 0.01}}"

To stop the grasping, you can use ``stop`` service.::

    ros2 service call /franka_gripper/stop std_srvs/srv/Trigger {}

Changes from franka_ros
-----------------------

* All topics and actions were previously prefixed with ``franka_gripper``. This prefix was renamed to ``panda_gripper``
  to enable, in the future, a workflow where all prefixes are based on the ``robot_type``
  to effortlessly enable multi arm setups.

* The ``stop`` action is now a service action as it is not preemptable.

* All actions (apart from the ``gripper_action``) have the current gripper width as feedback.