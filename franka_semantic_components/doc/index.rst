franka_semantic_components
==========================

This package contains franka_robot_model, franka_robot_state and cartesian command classes.
These classes are used to convert franka_robot_model object and franka_robot_state objects,
which are stored in the hardware_state_interface as a double pointer.

For further reference on how to use these classes:
`Franka Robot State Broadcaster <https://github.com/frankarobotics/franka_ros2/tree/jazzy/franka_robot_state_broadcaster>`_
and
`Franka Example Controllers(model_example_controller)
<https://github.com/frankarobotics/franka_ros2/blob/jazzy/franka_example_controllers/src/fr3/model_example_controller.cpp>`_

Cartesian Pose Interface
-------------------------

This interface is used to send Cartesian pose commands to the robot by using the loaned command interfaces.
FrankaSemanticComponentInterface class is handling the loaned command interfaces and state interfaces.
While starting the cartesian pose interface, the user needs to pass a boolean flag to the constructor
to indicate whether the interface is for the elbow or not. If the robot is using an arm_prefix, 
the prefix should also be passed to the constructor.

.. code-block:: cpp

   auto is_elbow_active = false;
   // Standard initialization
    CartesianPoseInterface cartesian_pose_interface(is_elbow_active);

    // Initialization with an arm prefix
    std::string arm_prefix = "arm_1";
    CartesianPoseInterface prefixed_interface(arm_prefix, is_elbow_active);

This interface allows users to read the current pose command interface values set by the franka hardware interface.

.. code-block:: cpp

   std::array<double, 16> pose;
   pose = cartesian_pose_interface.getInitialPoseMatrix();

One could also read quaternion and translation values in Eigen format.

.. code-block:: cpp

    Eigen::Quaterniond quaternion;
    Eigen::Vector3d translation;
    std::tie(quaternion, translation) = cartesian_pose_interface.getInitialOrientationAndTranslation();

After setting up the cartesian interface, you need to ``assign_loaned_command_interfaces`` and ``assign_loaned_state_interfaces`` in your controller.
This needs to be done in the on_activate() function of the controller. Examples can be found in the
`assign loaned comamand interface example
<https://github.com/frankarobotics/franka_ros2/blob/jazzy/franka_example_controllers/src/fr3/cartesian_pose_example_controller.cpp>`_

.. code-block:: cpp

    cartesian_pose_interface.assign_loaned_command_interfaces(command_interfaces_);
    cartesian_pose_interface.assign_loaned_state_interfaces(state_interfaces_);

In the update function of the controller you can send pose commands to the robot.

.. code-block:: cpp

    std::array<double, 16> pose;
    pose = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.5, 0, 0.5, 1};
    cartesian_pose_interface.setCommanded(pose);

Or you can send quaternion, translation values in Eigen format.

.. code-block:: cpp

    Eigen::Quaterniond quaternion(1, 0, 0, 0);
    Eigen::Vector3d translation(0.5, 0, 0.5);
    cartesian_pose_interface.setCommand(quaternion, translation);

Cartesian Velocity Interface
----------------------------

This interface is used to send Cartesian velocity commands to the robot by using the loaned command interfaces.
FrankaSemanticComponentInterface class is handling the loaned command interfaces and state interfaces.

.. code-block:: cpp

    auto is_elbow_active = false;
    // Standard initialization
    CartesianVelocityInterface cartesian_pose_interface(is_elbow_active);

    // Initialization with an arm prefix
    std::string arm_prefix = "arm_1";
    CartesianVelocityInterface prefixed_interface(arm_prefix, is_elbow_active);

To send the velocity command to the robot, you need to assign_loaned_command_interface in your custom controller.

.. code-block:: cpp

    cartesian_velocity_interface.assign_loaned_command_interface(command_interfaces_);

In the update function of the controller you can send cartesian velocity command to the robot.

.. code-block:: cpp

    std::array<double, 6> cartesian_velocity;
    cartesian_velocity = {0, 0, 0, 0, 0, 0.1};
    cartesian_velocity_interface.setCommand(cartesian_velocity);

Robot State and Model Access
-----------------------------

The semantic components provide safe access to the robot state and model objects that are stored as pointers in the hardware interface. This approach ensures proper type casting and memory management when working with these complex objects in controllers.