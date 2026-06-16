cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning 
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)


project(franka_gazebo_bringup)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(
        franka_gz_hardware_plugins
        SHARED
        src/gazebo_gravity_compensation_system.cpp
)

target_include_directories(franka_gz_hardware_plugins
    PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(franka_gz_hardware_plugins
        gz_ros2_control
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

target_link_libraries(franka_gz_hardware_plugins
        gz-sim::gz-sim
        pinocchio::pinocchio
)

pluginlib_export_plugin_description_file(
        gz_ros2_control franka_gz_hardware_plugins.xml)

install(
        TARGETS
        franka_gz_hardware_plugins
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

# Install project launch files
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
  # TODO add these back when gz sim issue is properly solved on ci
  add_ros_isolated_launch_test(test/test_gazebo_franka_arm_example_controller.py)
  add_ros_isolated_launch_test(test/test_gazebo_tmr_example_controller.py)
  # add_ros_isolated_launch_test(test/test_gazebo_mobile_fr3_duo_example_controller.py)
endif()

ament_package()
