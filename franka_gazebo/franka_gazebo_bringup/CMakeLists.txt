cmake_minimum_required(VERSION 3.5)
project(franka_gazebo_bringup)

find_package(ament_cmake REQUIRED)

# Install project launch files and assets
install(
  DIRECTORY
    launch config worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_gazebo_franka_arm_example_controller.py)
  add_ros_isolated_launch_test(test/test_gazebo_tmr_example_controller.py)
endif()

ament_package()
