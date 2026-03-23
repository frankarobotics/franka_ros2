cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)

project(franka_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ErrorRecovery.action"
  "action/Grasp.action"
  "action/Homing.action"
  "action/Move.action"
  "action/PTPMotion.action"

  "msg/CollisionIndicators.msg"
  "msg/Elbow.msg"
  "msg/Errors.msg"
  "msg/FrankaRobotState.msg"
  "msg/GraspEpsilon.msg"
  "msg/TargetStatus.msg"

  "srv/SetCartesianStiffness.srv"
  "srv/SetTCPFrame.srv"
  "srv/SetForceTorqueCollisionBehavior.srv"
  "srv/SetFullCollisionBehavior.srv"
  "srv/SetJointStiffness.srv"
  "srv/SetStiffnessFrame.srv"
  "srv/SetLoad.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
