cmake_minimum_required(VERSION 3.22)
project(mobile_fr3_duo_trajectory_controller)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
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
find_package(franka_semantic_components REQUIRED)

find_package(ament_cmake REQUIRED)

generate_parameter_library(${PROJECT_NAME}_parameters
  config/${PROJECT_NAME}_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
  src/trajectory.cpp
  src/mapping.cpp
)

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
                      ${PROJECT_NAME}_parameters)


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
        franka_semantic_components
)

pluginlib_export_plugin_description_file(controller_interface ${PROJECT_NAME}.xml)

install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)


if(BUILD_TESTING)
find_package(ament_cmake_clang_format REQUIRED)
find_package(ament_cmake_copyright REQUIRED)
find_package(ament_cmake_cppcheck REQUIRED)
find_package(ament_cmake_flake8 REQUIRED)
find_package(ament_cmake_lint_cmake REQUIRED)
find_package(ament_cmake_pep257 REQUIRED)
find_package(ament_cmake_xmllint REQUIRED)
find_package(ros2_control_test_assets REQUIRED)

set(CPP_DIRECTORIES src include)
        ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
        ament_copyright(launch ${CPP_DIRECTORIES} package.xml)
        ament_cppcheck(${CPP_DIRECTORIES})
        ament_lint_cmake(CMakeLists.txt)
        ament_flake8()
        ament_pep257()
        ament_xmllint()

        find_package(ament_cmake_gmock REQUIRED)
        ament_add_gmock(${PROJECT_NAME}_test
                test/mobile_fr3_duo_trajectory_controller_test.cpp
        )

        target_compile_definitions(${PROJECT_NAME}_test PRIVATE
                "TEST_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test\"")

        target_link_libraries(${PROJECT_NAME}_test
                ${PROJECT_NAME}
        )

        target_include_directories(${PROJECT_NAME}_test PRIVATE
                include
        )
endif()

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