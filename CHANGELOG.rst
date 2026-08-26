Changelog for package franka_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

UNRELEASED
----------
Requires libfranka >= 0.20.4 and franka_description >= 2.9.0 requires ROS 2 Jazzy

* fix: ``franka_semantic_components::FrankaRobotState`` now sizes and labels the joint arrays of
  ``FrankaRobotState`` from the arm the component is bound to, instead of from every revolute
  joint in the global ``robot_description``. With more than seven revolute joints in the
  description the arrays came out longer than seven, and the trailing entries were filled by
  reading past the end of libfranka's seven-element arrays, so they carried a different quantity
  than their joint name promised and each broadcaster labelled its own arm's data with another
  arm's joint names.

v3.5.2 (2026-08-17)
-------------------
Requires libfranka >= 0.20.4 and franka_description >= 2.9.0 requires ROS 2 Jazzy

* fix: updated franka_description to 2.9.0

v3.5.1 (2026-08-14)
-------------------
Requires libfranka >= 0.20.4 and franka_description >= 2.9.0 requires ROS 2 Jazzy

* fix: accelerometers data is float in libfranka 0.20.5

v3.5.0 (2026-08-14)
-------------------
Requires libfranka >= 0.20.4 and franka_description >= 2.9.0 requires ROS 2 Jazzy

* feat: add TMR battery ROS 2 support in ``franka_mobile`` (``sensor_msgs/BatteryState``
  topic at 1 Hz and ``std_srvs/Trigger`` services for wireless charging), with an
  internal ``franka_desk_api`` HTTPS helper used by the spine and battery clients.
* fix: franka_hardware tests now run in isolated ROS domains to avoid possible collisions.
* fix: fixed franka_hardware stop motion test to check for stopping_joint_positions matching the last 
        commanded target instead of zero positions. 
* fix: tune mobile teleop velocity/acceleration limits to stay within the RCU
  2-norm bounds. Raise swerve translational limits to 0.35 m/s / 0.4 m/s^2 and
  rotational to 0.5 rad/s / 0.3 rad/s^2 in ``controllers.yaml``, rescale the xbox
  normal/turbo axes in ``xbox.config.yaml`` to match ``max_velocity``, and reduce
  the joystick deadzone (0.1) with a higher autorepeat rate (50 Hz) in
  ``mobile_teleop.launch.py`` for smoother teleop.
* fix: the description extension entry points and the mobile fr3 duo moveit config no longer
  hardcode the franka hand's tcp offset, so a cobot pump mounted on them is placed at its own
  ``0 0 0.105`` instead of the hand's ``0 0 0.1034``. Requires franka_description with the resolved
  ``tcp_xyz`` default. MIGRATION: pass ``tcp_xyz`` explicitly to keep the previous value; the franka
  hand is unaffected.
* fix: **BREAKING CHANGE** thread-safe access to robot state interface (changed from RealtimeBuffer to RealtimeThreadSafeBox).
  This fixes possible race conditions that could happen when running the broadcaster and the 
  cartesian impedance example controller (e.g. via the `franka_semantic_components::FrankaRobotModel`) with async=true. 
  Users of the `franka_semantic_components::FrankaRobotState` should be not impacted, only direct state interface users claiming the hardware interface.
* fix: default thread priority for franka_hardware interface threads is now 98 (was 50) to 
  ensure RT stability.
* fix: recover from a ``franka::ControlException`` without restarting
  ``ros2_control_node``. ``franka_robot_state_broadcaster`` and
  ``joint_state_broadcaster`` stay lifecycle-active. One update cycle may
  publish the pre-fault / frozen sample after ``read()`` latches, then the
  controller-manager update thread blocks for the approximately two-second
  braking stop and topic publication pauses; after the block, inactive-state
  reads resume and publish live reflex state. After clearing the robot error,
  only the command controller needs reactivation.
* docu: add an umbrella guide for the relocated description extensions,
  including composition layers, mounting points, prefix rules, and external
  gripper attachment.
* docs: comprehensive documentation audit and fixes across 12 packages — corrected
  inaccurate launch arguments, removed references to non-existent files, fixed API method
  names, updated parameter names to match code, added undocumented launch args, improved
  grammar and user experience. Fixed tuple default bug in
  franka_vision_and_manipulation_kit launch file.
* fix: align MoveIt gripper controller name with actual gripper node name.
  ``fr3_controllers.yaml`` and ``moveit.launch.py`` referenced ``fr3_gripper``
  but the node is launched as ``franka_gripper``, breaking gripper action commands
  and joint state aggregation in MoveIt. (GitHub PR #206)
* test: comprehensive no-hardware test suite

  - Add controller load test covering all 16 example controllers (one GTest case each)
  - Add launch file parsing validation for 19 first-party launch files
  - Add fake-hardware integration tests (controller spawn, activate, verify)
  - Add fake-hardware test config files
    (``test_fake_hardware_{fr3,fr3_duo,mobile_fr3_duo,tmr}.config.yaml``)
  - Register ``test_gripper_topic_consistency.py`` in franka_fr3_moveit_config

* ci: improve Jenkins pipeline test coverage

  - Fix package regex to include ``mobile_fr3_duo_trajectory_controller`` and
    ``franka_bringup`` structural tests (previously silently dropped)
  - Isolate Gazebo tests into separate optional stage
  - Add ``executeGazeboTests`` parameter (default: false)

* fix: add missing dependencies in package.xml for isolated builds
  (``rclcpp_components`` in franka_hardware; ``rclcpp_lifecycle``, ``urdf``, ``eigen``,
  ``controller_interface`` in franka_semantic_components). Fixes rosdep-based
  fresh-environment builds. (community contribution: GitHub PRs #96, #169)
* refactor: decoupled simulation backend selection from the franka_hardware ros2_control
  macros and removed all Gazebo-specific content from franka_description. The hardware
  ``<plugin>`` block (real / mock / gazebo) and the gz_ros2_control ``<gazebo>`` system
  element are now injected by the owning package (franka_bringup for real/mock, franka_gazebo
  for gazebo) instead of being selected by ``xacro:if`` inside franka_hardware. Mode and
  interface selection moved up to the entry-point URDFs as implementation-agnostic capability
  flags (effort command, finger joint, passive base, self-collision geometry), replacing the
  former ``gazebo`` / ``gazebo_effort`` semantic flags. franka_description no longer contains
  any Gazebo tags, macros, or arguments — the Gazebo SDF/transmission machinery, the world
  anchor, and self-collision suppression now live in franka_gazebo, making franka_description
  simulator-agnostic. All robot × mode URDF expansions remain byte-identical to before the
  refactor.

v3.4.1 (2026-07-07)
-------------------
Requires libfranka >= 0.20.4 and franka_description >= 2.8.0 requires ROS 2 Jazzy

* feat: add tmr launch file in franka bringup and use it for the mobile teleop launch file

v3.4.0 (2026-06-23)
-------------------
Requires libfranka >= 0.20.4 and franka_description >= 2.8.0 requires ROS 2 Jazzy

* feat: expose ``K_F_ext_hat_K`` as ``ForceTorqueSensor`` state interfaces (force.x/y/z,
  torque.x/y/z) on the ``<arm_prefix><robot_type>_tcp`` sensor, enabling direct wrench
  consumption at control frequency without a topic bridge.
* fix(franka_mobile): enable jerk limiting on the swerve_drive_controller velocity
  limiter (linear x/y, angular z) so the commanded base velocity ramps with a bounded
  acceleration onset, preventing the firmware ``cartesian_motion_generator_velocity_discontinuity``
  reflex from aborting base motion when switching between whole-body and base-only control.
  The rate-limiter timestep is fixed at the nominal 1 kHz control period.
* BREAKING CHANGE: collision_detected topic now uses best_effort QoS (SensorDataQoS);
  thread-safe atomics for collision state in example controllers.
  Subscribers using the default ``reliable`` QoS will no longer receive messages.
  To migrate, set the subscriber QoS to ``best_effort`` (``SensorDataQoS``):

  .. code-block:: cpp

     // Before (default reliable QoS — no longer receives messages):
     auto sub = node->create_subscription<std_msgs::msg::Bool>(
         "collision_detected", 1, callback);

     // After (best_effort QoS):
     auto sub = node->create_subscription<std_msgs::msg::Bool>(
         "collision_detected", rclcpp::SensorDataQoS(), callback);
* refactor: Removed the robot_description from launch files - controller_manager gets it natively via topic
* docu: Added documentation for error recovery after an FCI error.
* fix: Pinocchios collision detection component dropped the 'fcl/hpp' namespace and only supports now the coal namespace
* feat: added a model-based gravity-compensation system plugin
  (``franka_gazebo_hardware::GazeboGravityCompensationSystem``) for gz_ros2_control that
  injects pinocchio-computed gravity torque on the effort-controlled arm joints, so the
  zero-torque example controllers behave in Gazebo as on the real robot (where the master
  controller performs gravity compensation) instead of collapsing under gravity.
* chore: restructured the Gazebo support into two packages under the ``franka_gazebo/``
  grouping folder: ``franka_gazebo_bringup`` (launch/world/urdf/config assets) and
  ``franka_gazebo_hardware`` (the gz system plugin). The public launch command
  ``ros2 launch franka_gazebo_bringup <file>`` is unchanged; the plugin is now referenced as
  ``franka_gazebo_hardware/GazeboGravityCompensationSystem``.
* chore: removed the per-link ``<gravity>false</gravity>`` xacro overrides; gravity is now
  enabled globally in the Gazebo world. This is engine-independent (the previous approach
  relied on a gz-fortress gravity-disable behavior that is not forwarded by some physics
  engines). Simulation-only behavior change; no impact on real-robot users.
* feat: hold the mobile platform (``mobile_fr3_duo_v0_2``) prismatic
  ``franka_spine_vertical_joint`` against gravity with a ``spine_joint_trajectory_controller``
  (JointTrajectoryController on a position command interface) that holds the column at its
  initial height.

v3.3.0 (2026-05-04)
-------------------
Requires libfranka >= 0.20.4 and franka_description >= 2.7.0 requires ROS 2 Jazzy

* feat: Added the spline action server and examples for the Franka Spine
* chore: refactored gazebo launch files, added integration tests
* chore: refactored mFR3duo example controller to be gazebo independent by chaining `swerve_ik_controller`
* chore: removed vendored controller_manager
* feat: Added `franka_mobile` package with `swerve_drive_controller` (tf and odom support) and `swerve_ik_controller` for gazebo sim.
* docu: Maintenance work on documentation
* feat: add franka_vision_and_manipulation_kit package with urdf descriptions and launch files for the Franka Vision and Manipulation Kit
* refactor: replace blocking mutex in franka_robot_state_broadcaster with lock-free AsyncBuffer
* BREAKING CHANGE: franka_robot_state_broadcaster convenience topics are published with best_effort QoS
* chore: Added a CI job to try backporting jazzy to humble
* BREAKING CHANGE: franka_robot_state_broadcaster convenience topics are published with best_effort QoS.
  Topics affected: ``current_pose``, ``stiffness_frame_wrench``, and all other convenience topics
  from the broadcaster. Subscribers using the default ``reliable`` QoS will no longer receive
  messages. To migrate, set the subscriber QoS to ``best_effort``:

  **C++ (rclcpp)**

  .. code-block:: cpp

     // Before (default reliable QoS — no longer receives messages):
     auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
         "current_pose", 10, callback);

     // After (best_effort QoS):
     rclcpp::QoS qos(10);
     qos.best_effort();
     auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
         "current_pose", qos, callback);

  **Python (rclpy)**

  .. code-block:: python

     # Before (default reliable QoS — no longer receives messages):
     self.create_subscription(PoseStamped, 'current_pose', callback, 10)

     # After (best_effort QoS):
     from rclpy.qos import QoSProfile, ReliabilityPolicy
     qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
     self.create_subscription(PoseStamped, 'current_pose', callback, qos)

* feat: franka_vision_and_manipulation_kit package used in Gazebo to display the sensors of the kit
* fix: clang-tidy test duration reduced by applying a filter
* feat: added a gazebo example for fr3 duo using the franka vision and manipulation kit
* fix: teleop node is publishing correctly if there is no namespace
* feat: Added moveit support for mobile fr3 duo
* feat: Updated franka_selfcollision package to support mobile_fr3_duo in addition to fr3_duo
* fix: resolve segfault caused by ABI mismatch between hardware_interface and controller_manager **C++ (rclcpp)**
* chore: Unify ros2_control (Jazzy) sources to avoid ABI mismatches (add ros2_control to dependency.repos so hardware_interface/controller_manager build from the same ABI;
remove the standalone hardware_interface fork; keep manage_overruns for async hardware components via a small patch auto-applied in franka_entrypoint.sh).
**BREAKING**: existing workspaces must be cleaned before rebuilding: ``rm -rf build/ install/ && colcon build``
* fix: Removed the `franka_robot_state_broadcaster` while using the TMR
* fix: With async control, it could happen that the robot_state is teared while publishing. This fix provides a
  lock-free intermediate buffer to allow mutual exclusive robot_state buffer data access.
* fix: improved integration test reliability
* chore: Removed the explicit self collision example and added it by default to the fr3 duo and mobile fr3 duo examples
* fix: make FrankaHardwareInterface error recoverable
* refactor: FrankaHardwareInterface to use enums for the control mode
* fix: ActionServers crashing when exception is not caught
* fix: re-enable franka_gazebo_bringup launch tests with robust gz sim cleanup between tests
* feat: replace std::mutex with prio_inherit_mutex in franka_hardware for RT safety
* feat: add Cartesian impedance example controller for the fr3

v3.2.2 (2026-03-03)
-------------------
Requires libfranka >= 0.20.4 and franka_description >= 2.6.0 requires ROS 2 Jazzy

* feat: update to franka_description 2.6.0

v3.2.1 (2026-03-02)
-------------------
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
-------------------

Requires libfranka >= 0.13.2 and franka_description >= 0.3.0 requires ROS 2 Humble

* feat:  franka_gazebo_bringup: Released and supports joint position, velocity and effort commands
* feat:  franka_ign_ros2_control: ROS 2 hardware interface for gazebo controller. Modified to add gravity torques for Franka robots.
* fix: the joint-impedance-with-IK example to work without a gripper

0.1.14 (2024-05-13)
-------------------

Requires libfranka >= 0.13.2, and franka_description >= 0.2.0 requires ROS 2 Humble

* BREAKING CHANGE: franka_description package
* BREAKING CHANGE: using the franka_description standalone package https://github.com/frankarobotics/franka_description
* build:  install pinocchio dependency from ros-humble-pinocchio apt package
* feat: Added error recovery action to ROS 2 node
* fix: hard-coded panda robot references
* fix: franka_hardware prefixes the robot_state and robot model state interfaces with the read robot name from the urdf.

0.1.13 (2024-01-18)
-------------------

Requires libfranka >= 0.13.2, requires ROS 2 Humble

* BREAKING CHANGE: update libfranka dependency in devcontainer to 0.13.3(requires system image 5.5.0)
* fix: devcontainer typo

0.1.12 (2024-01-12)
-------------------

Requires libfranka >= 0.13.2, requires ROS 2 Humble

* feat: franka_semantic_component: Read robot state from urdf robot description.
* feat: franka_state_broadcaster: Publish visualizable topics seperately.

0.1.11 (2023-12-20)
-------------------

Requires libfranka >= 0.13.2, requires ROS 2 Humble

* feat: franka_example_controllers: Add a joint impedance example using OrocosKDL(LMA-ik) through MoveIt service.
* feat: franka_hardware: Register initial joint positions and cartesian pose state interface without having running command interfaces.

0.1.10 (2023-12-04)
-------------------

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
