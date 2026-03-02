cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)

project(franka_gripper)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(Franka 0.19.0 REQUIRED)

add_library(gripper_server SHARED
        src/gripper_action_server.cpp)
target_link_libraries(gripper_server Franka::Franka)
target_include_directories(gripper_server PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(gripper_server
        franka_msgs
        rclcpp
        rclcpp_action
        rclcpp_components
        Franka
        sensor_msgs
        std_srvs
        control_msgs)
rclcpp_components_register_node(gripper_server PLUGIN "franka_gripper::GripperActionServer" EXECUTABLE franka_gripper_node)
install(TARGETS
        gripper_server
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY
        launch config
        DESTINATION share/${PROJECT_NAME}/)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
        scripts/fake_gripper_state_publisher.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)

    find_package(ament_cmake_ros REQUIRED)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(launch ${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()

    if(NOT ROBOT_IP)
        set(ROBOT_IP "172.16.0.1")
        message(STATUS "ROBOT_IP not defined, defaulting to ${ROBOT_IP}...")
    endif()

    function(add_ros_isolated_launch_test path)
      set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
      add_launch_test("${path}" RUNNER "${RUNNER}" ARGS "robot_ip:=${ROBOT_IP}" TIMEOUT 500)
    endfunction()

    add_ros_isolated_launch_test(test/test_hardware_franka_gripper_position.py)

endif()

ament_package()
