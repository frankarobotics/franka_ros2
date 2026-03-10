cmake_minimum_required(VERSION 3.22)
project(mobile_fr3_duo_joint_trajectory_controller)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEFAULT_BUILD_TYPE "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Build type" FORCE)
endif()

find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ament_cmake REQUIRED)

generate_parameter_library(mobile_fr3_duo_joint_trajectory_controller_parameters
  src/mobile_fr3_duo_joint_trajectory_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/mobile_fr3_duo_joint_trajectory_controller.cpp
  src/trajectory.cpp
)

target_include_directories(
        mobile_fr3_duo_joint_trajectory_controller
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)

# TODO (wink_ma): this is ugly
target_link_libraries(${PROJECT_NAME} PUBLIC
                      mobile_fr3_duo_joint_trajectory_controller_parameters)


ament_target_dependencies(
        ${PROJECT_NAME}
        PUBLIC
        rclcpp
        rclcpp_action
        rclcpp_lifecycle
        realtime_tools
        control_msgs
        controller_interface
        trajectory_msgs
)

pluginlib_export_plugin_description_file(controller_interface mobile_fr3_duo_joint_trajectory_controller.xml)


install(
        TARGETS
        ${PROJECT_NAME}
        mobile_fr3_duo_joint_trajectory_controller_parameters
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        control_msgs
        controller_interface
        rclcpp
        rclcpp_lifecycle
        realtime_tools
        trajectory_msgs
)

ament_package()