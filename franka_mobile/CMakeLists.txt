cmake_minimum_required(VERSION 3.8)
project(franka_mobile)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(pluginlib REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(urdf REQUIRED)

pluginlib_export_plugin_description_file(
    controller_interface controllers.xml
)

generate_parameter_library(swerve_drive_controller_parameters
  config/swerve_drive_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
    src/swerve_kinematics.cpp
    src/urdf_utils.cpp
    src/odometry.cpp
    src/swerve_ik_controller.cpp
    src/swerve_drive_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
      ${CMAKE_CURRENT_BINARY_DIR}/include
  PUBLIC
      include
      ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
 swerve_drive_controller_parameters
)

ament_target_dependencies(
        ${PROJECT_NAME}
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        tf2_msgs
        nav_msgs
        geometry_msgs
        diff_drive_controller
        generate_parameter_library
        franka_semantic_components
        realtime_tools
        kdl_parser
        urdf
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # TODO add this back when gz sim issue is properly solved on ci
  # ament_lint_auto_find_test_dependencies()

  function (add_franka_mobile_test test_name source_file)
    ament_add_gtest(${test_name} ${source_file})
    target_include_directories(${test_name} PRIVATE include)
    target_link_libraries(${test_name} ${PROJECT_NAME})
    ament_target_dependencies(${test_name} rclcpp)
  endfunction()

  add_franka_mobile_test(test_odometry test/test_odometry.cpp)
  add_franka_mobile_test(test_swerve_kinematics test/test_swerve_kinematics.cpp)

endif()

ament_export_libraries(franka_mobile)

ament_package()