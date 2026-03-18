cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)

project(franka_selfcollision)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(srdfdom REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/self_collision_checker.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    pinocchio::pinocchio
    rclcpp::rclcpp
)

add_executable(self_collision_node
        src/self_collision_node.cpp)

target_include_directories(self_collision_node PRIVATE
  include
)

target_link_libraries(self_collision_node
                    ${PROJECT_NAME}
                    pinocchio::pinocchio)

ament_target_dependencies(self_collision_node
    ament_index_cpp
    franka_msgs
    rclcpp
    sensor_msgs
    std_msgs
)


# mobile collision node
add_executable(mobile_self_collision_node
        src/mobile_self_collision_node.cpp)

target_include_directories(mobile_self_collision_node PRIVATE
  include
)

target_link_libraries(mobile_self_collision_node
                    ${PROJECT_NAME}
                    pinocchio::pinocchio)

ament_target_dependencies(mobile_self_collision_node
    ament_index_cpp
    rclcpp
    sensor_msgs
    std_msgs
)


install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(TARGETS
        self_collision_node
        mobile_self_collision_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}/)

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
        test/test_self_collision_checker.cpp
    )

    target_compile_definitions(${PROJECT_NAME}_test PRIVATE
        "TEST_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test\"")

    target_link_libraries(franka_selfcollision_test
        ${PROJECT_NAME}
        pinocchio::pinocchio
    )

    target_include_directories(franka_selfcollision_test PRIVATE
        include
        ${pinocchio_INCLUDE_DIRS}
    )
endif()


ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)

ament_package()
