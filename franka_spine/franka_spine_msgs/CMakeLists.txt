cmake_minimum_required(VERSION 3.5)

option(CHECK_TIDY "Enable clang-tidy checks" OFF)

project(franka_spine_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveAbsolute.action"

  "msg/UserLimits.msg"
  "msg/SpineParameters.msg"

  "srv/GetSpineState.srv"
  "srv/GetPosition.srv"
  "srv/FaultReset.srv"
  "srv/SwitchOn.srv"
  "srv/SwitchOff.srv"
  "srv/Halt.srv"
  "srv/GetParameters.srv"

  DEPENDENCIES std_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
