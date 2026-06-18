cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)

project(franka_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  ament_lint_auto_find_test_dependencies()

  if(NOT ROBOT_IP)
    set(ROBOT_IP "172.16.0.1")
    message(STATUS "ROBOT_IP not defined, defaulting to ${ROBOT_IP}...")
  endif()

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ARGS "robot_ip:=${ROBOT_IP}" TIMEOUT 500)
  endfunction()

  add_ros_isolated_launch_test(test/test_hardware_example_controllers.py)
  add_ros_isolated_launch_test(test/test_hardware_generic_controller.py)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_franka_arm_interfaces test/test_franka_arm_interfaces.py)
  ament_add_pytest_test(test_xacro_path_resolution test/test_xacro_path_resolution.py)
  ament_add_pytest_test(test_gazebo_wrapper_expansion test/test_gazebo_wrapper_expansion.py)

  install(
    DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
