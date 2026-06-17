cmake_minimum_required(VERSION 3.5)

option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)

project(franka_gazebo_hardware)

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
find_package(ignition-gazebo6 REQUIRED)
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
        src/gravity_compensation_model.cpp
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
        ignition-gazebo6::core
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(gravity_compensation_model_test test/gravity_compensation_model_test.cpp)
  target_include_directories(gravity_compensation_model_test PRIVATE
          include
          ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(gravity_compensation_model_test
          franka_gz_hardware_plugins
          pinocchio::pinocchio
  )
endif()

ament_package()
