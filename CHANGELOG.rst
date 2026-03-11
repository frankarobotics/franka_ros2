Changelog for package franka_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

UNRELEASED
----------

* feat: Added the spline action server and examples for the Franka Spine
* chore: refactored gazebo launch files, added integration tests
* chore: refactored mFR3duo example controller to be gazebo independent by chaining `swerve_ik_controller`

v3.2.2 (2026-03-03)
-------------------

* feat: update to franka_description 2.6.0

v3.2.1 (2026-03-02)
----------
Requires libfranka >= 0.20.4 and franka_description >= 2.4.0 requires ROS 2 Jazzy

* feat: Added a selfcollision package and example controller for the fr3_duo
* feat: Updated `franka_description` dependency to 2.4.0
* feat: integration_launch_testing: added smoke tests for the example controllers
* feat: integration_launch_testing: test example controllers using example.launch.py
* fix: gripper_example_controller also works without namespace
* fix: corrected logs in franka_hardware
* fix: gravity_compensation_example_controller, move_to_start_example_controller, joint_impedance_example_controller work with parametrized robot_type
* fix: jazzy compatibility matrix
* fix: joint_impedance_with_ik_example_controller checked for 'robot_id' instead of 'robot_type' argument
* fix: added missing dependency to rclcpp_action in franka_hardware package.xml
* fix: franka_hardware test fixed
* fix: franka_hardware test fixed
* fix: rclpy.parameter_client.AsyncParameterClient replaced with custom version because the package is missing in humble
* feat: update to libfranka 0.20.4
* chore: cleanup franka_bringup launch utils import
* feat: add franka_mobile_sensors as optional package for TMR robots
* feat: mobile fr3 duo example added
* refactor: franka example controllers divided into robot types
* feat: mobile fr3 duo example for gazebo
* feat: add gazebo sensors xacro in franka_gazebo
* fix: added arm_prefix functionality to all franka_example_controllers and franka_robot_state_broadcaster
* chore: vendored controller_manager 4.39.2 to avoid version mismatch with gz_ros_sim
* chore: removed integration_launch_testing package, moved integration tests in their subpackage

v3.2.0 (2026-01-15)
-------------------
Requires libfranka >= 0.19.0 and franka_description >= 2.3.0 requires ROS 2 Jazzy

* Add: Added a joint-based point-to-point motion action with usage example
* BREAKING CHANGE: arm_id replaced by robot_type and controller_name by controller_names
* Remove: `olvx_description_module` dependency removed
* Feat: TMRv0.2 teleoperation example controller added
* Feat: arm_id replaced by robot_type
* chore: bump required ros2_control version from franka_description to 1.0.0 from franka_hardware_interface
* feat: pass is_async argument to franka_arm.ros2_control.xacro
* feat: support dual arm fr3 setup only with torque command interface using is_async from ros2_control
* chore: removed the custom controller manager from franka_ros2 and using the ros2_control controller manager instead
* chore: added custom hardware interface and realtime tools forked from the ros2_control repository with the passing the manage overrun feature

v3.1.1 (2025-11-10)
-------------------
Requires libfranka >= 0.18.0 and franka_description >= 2.0.0 requires ROS 2 Jazzy

* Fix: controller manager overrun feature disabled

v3.1.0 (2025-10-24)
-------------------
Requires libfranka >= 0.18.0 and franka_description >= 2.0.0 requires ROS 2 Jazzy

* Updated dependencies: libfranka to 0.18.0
* BREAKING CHANGE: only one move group called `(arm_id)_arm` is available. If Franka Hand is set, the TCP is placed as in the former `(arm_id)_manipulator`. Otherwise, its location corresponds to the one from the former `(arm_id)_arm`.
* Refactor: ee_id and load_gripper arguments added in moveit launch file

v3.0.0 (2025-09-18)
-------------------

Requires libfranka >= 0.15.0 and franka_description >= 2.0.0 requires ROS 2 Jazzy

* Add support for ROS 2 Jazzy
* Add docs under each package
* Refactor: Optimized the franka_robot_state_broadcaster to not block the RT loop of ros2_control
* Added fixed forked of the controller manager to avoid the [issue #2529](https://github.com/ros-controls/ros2_control/issues/2529) in ros2_control

v2.0.2 (2025-07-09)
-------------------
Requires libfranka >= 0.15.0 and franka_description >= 1.0.0 requires ROS 2 Humble and Jazzy

* refactor: srdf files come from franka description
* Fix: FrankaHardwareInterface: Fix eager claiming bug when multiple hardware components are present
* Fix: joint_state_publisher uses correct topics to avoid rviz glitches

v2.0.1 (2025-06-26)
-------------------
Requires libfranka >= 0.15.0 and franka_description >= 1.0.0 requires ROS 2 Humble

* Fix: joint_impedance_with_ik_example_controller uses correct time from robot

v2.0.0 (2025-06-10)
-------------------
Requires libfranka >= 0.15.0 and franka_description >= 0.5.0 requires ROS 2 Humble

* BREAKING CHANGE: `franka.launch.py` is adapted to use namespaces
* BREAKING CHANGE: the controller examples were removed to use a single launch script named `example.launch.py`, which can launch multiple robots and takes the arguments from a config file named `franka.config.yaml`
* Fix: franka gripper works with namespaces
* Add: `example.launch.py` - a single launch script to launch any number of namespaces
* Feat: `franka.launch.py` can launch different robots in specific namespaces
* Add: `franka.config.yaml` to configure the input arguments for multiple robots
* Add: `controllers.yaml` controller file for namespace-agnostic launch of existing controllers


v1.0.2 (2025-05-30)
-------------------

Requires libfranka >= 0.15.0 and franka_description >= 0.5.0 requires ROS 2 Humble

* Fix: gripper example controller does not start any hardware interface


v1.0.1 (2025-05-26)
-------------------

Requires libfranka >= 0.15.0 and franka_description >= 0.5.0 requires ROS 2 Humble

* Fix: FrankaRobotStateBroadcaster Lock issue - add configurable timeout (see controllers.yaml)
* Add: vcstool import for compatible libfranka and franka_description
* Fix: Franka robot state broadcaster GitHub Issue #94 and #105
* Test: Re-enable a test and provide Mock functions
* Style: Adjust clang-tidy config due to changes in generate_parameter_library()
* Chore: Eliminate annoying CMake configure time messages
* Feat: Added prefix to single robot control
* Doc: Added a link to the Gazebo README.md for better visibility
* Breaking feat: Automatically spawn command interfaces depending on the configured ones coming from the URDF


v1.0.0 (2025-01-22)
-------------------

Requires libfranka >= 0.15.0 and franka_description >= 0.3.0 requires ROS 2 Humble

* feat: franka_example_controllers - Add a Franka Hand controller example (gripper_example_controller)
* fix: reduced acceleration discontinuities by adding new robot_time state to franka_hardware that allows to update controllers with same time that robot uses
* refactor: Improved Docker image for development with VSCode
* BREAKING_CHANGE: initial_joint_position state removed from franka_hardware. rename/replace functions in franka_semantic_components as follows:

  ::

        -  initial_cartesian_pose, initial_elbow_state
        +  cartesian_pose_state,   elbow_state.
        - getInitialElbowConfiguration, getInitialOrientationAndTranslation, getInitialPoseMatrix
        + getCurrentElbowConfiguration, getCurrentOrientationAndTranslation, getCurrentPoseMatrix


0.1.15 (2024-06-21)
----------------------

Requires libfranka >= 0.13.2 and franka_description >= 0.3.0 requires ROS 2 Humble

* feat:  franka_gazebo_bringup: Released and supports joint position, velocity and effort commands
* feat:  franka_ign_ros2_control: ROS 2 hardware interface for gazebo controller. Modified to add gravity torques for Franka robots.
* fix: the joint-impedance-with-IK example to work without a gripper

0.1.14 (2024-05-13)
----------------------

Requires libfranka >= 0.13.2, and franka_description >= 0.2.0 requires ROS 2 Humble

* BREAKING CHANGE: franka_description package
* BREAKING CHANGE: using the franka_description standalone package https://github.com/frankarobotics/franka_description
* build:  install pinocchio dependency from ros-humble-pinocchio apt package
* feat: Added error recovery action to ROS 2 node
* fix: hard-coded panda robot references
* fix: franka_hardware prefixes the robot_state and robot model state interfaces with the read robot name from the urdf.

0.1.13 (2024-01-18)
----------------------

Requires libfranka >= 0.13.2, requires ROS 2 Humble

* BREAKING CHANGE: update libfranka dependency in devcontainer to 0.13.3(requires system image 5.5.0)
* fix: devcontainer typo

0.1.12 (2024-01-12)
----------------------

Requires libfranka >= 0.13.2, requires ROS 2 Humble

* feat: franka_semantic_component: Read robot state from urdf robot description.
* feat: franka_state_broadcaster: Publish visualizable topics seperately.

0.1.11 (2023-12-20)
----------------------

Requires libfranka >= 0.13.2, requires ROS 2 Humble

* feat: franka_example_controllers: Add a joint impedance example using OrocosKDL(LMA-ik) through MoveIt service.
* feat: franka_hardware: Register initial joint positions and cartesian pose state interface without having running command interfaces.

0.1.10 (2023-12-04)
----------------------

Requires libfranka >= 0.13.0, required ROS 2 Humble

* feat: Adapted the franka robot state broadcaster to use ROS 2 message types
* feat: Adapted the Cartesian velocity command interface to use Eigen types

0.1.9 (2023-12-04)
------------------

Requires libfranka >= 0.13.0, required ROS 2 Humble

* feat: franka_hardware: add state interfaces for initial position, cartesian pose and elbow.
* feat: franka_hardware: support cartesian pose interface.
* feat: franka_semantic_component: support cartesian pose interface.
* feat: franka_example_controllers: add cartesian pose example controller
* feat: franka_example_controllers: add cartesian elbow controller
* feat: franka_example_controllers: add cartesian orientation controller

0.1.8 (2023-11-16)
------------------

Requires libfranka >= 0.13.0, required ROS 2 Humble

* test: franka_hardware: add unit tests for robot class.
* fix:  joint_trajectory_controller: hotfix add joint patched old JTC back.

0.1.7 (2023-11-10)
------------------

Requires libfranka >= 0.12.1, required ROS 2 Humble

* feat: franka_hardware: joint position command interface supported
* feat: franka_hardware: controller initializer automatically acknowledges error, if arm is in reflex mode
* feat: franka_example_controllers: joint position example controller provided
* fix:  franka_example_controllers: fix second start bug with the example controllers

0.1.6 (2023-11-03)
------------------

Requires libfranka >= 0.12.1, required ROS 2 Humble

* feat: franka_hardware: support for cartesian velocity command interface
* feat: franka_semantic_component: implemented cartesian velocity interface
* feat: franka_example_controllers: implement cartesian velocity example controller
* feat: franka_example_controllers: implement elbow example controller

0.1.5 (2023-10-13)
------------------

Requires libfranka >= 0.12.1, required ROS 2 Humble

* feat: franka_hardware: support joint velocity command interface
* feat: franka_example_controllers: implement joint velocity example controller
* feat: franka_description: add velocity command interface to the control tag

0.1.4 (2023-09-26)
------------------

Requires libfranka >= 0.12.1, required ROS 2 Humble

* feat: franka_hardware: adapt to libfranka active control 0.12.1

0.1.3 (2023-08-24)
------------------

Requires libfranka >= 0.11.0, required ROS 2 Humble

* fix: franka_hardware: hotfix start controller when user claims the command interface

0.1.2 (2023-08-21)
------------------

Requires libfranka >= 0.11.0, required ROS 2 Humble

* feat: franka_hardware: implement non-realtime parameter services

0.1.1 (2023-08-21)
------------------

Requires libfranka >= 0.11.0, required ROS 2 Humble

* feat: franka_hardware: uses updated libfranka version providing the possibility to have the control loop on the ROS side

0.1.0 (2023-07-28)
------------------

Requires libfranka >= 0.10.0, required ROS 2 Humble

* feat: franka_bringup: franka_robot_state broadcaster added to franka.launch.py.
* feat: franka_example_controllers: model printing read only controller implemented
* feat: franka_robot_model: semantic component to access robot model parameters.
* feat: franka_msgs: franka robot state msg added
* feat: franka_robot_state: broadcaster publishes robot state.
* feat: joint_effort_trajectory_controller package that contains a version of the\
        joint_trajectory_controller that can use the torque interface. \
        [See this PR](https://github.com/ros-controls/ros2_controllers/pull/225)
* feat: franka_bringup package that contains various launch files to start controller examples or Moveit2.
* feat: franka_moveit_config package that contains a minimal moveit config to control the robot.
* feat: franka_example_controllers package that contains some example controllers to use.
* feat: franka_hardware package that contains a plugin to access the robot.
* feat: franka_msgs package that contains common message, service and action type definitions.
* feat: franka_description package that contains all meshes and xacro files.
* feat: franka_gripper package that offers action and service interfaces to use the Franka Hand gripper.
* fix:  franka_hardware Fix the mismatched joint state interface type logger error message.
* test: CI tests in Jenkins.
