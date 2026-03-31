cmake_minimum_required(VERSION 3.5)
project(franka_spine_examples)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Enable clang-tidy checks" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(franka_spine_msgs REQUIRED)

add_executable(spine_client_example_cpp src/spine_client_example.cpp)
target_include_directories(spine_client_example_cpp PRIVATE include)
ament_target_dependencies(spine_client_example_cpp rclcpp rclcpp_action franka_spine_msgs)

install(TARGETS spine_client_example_cpp
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(PROGRAMS
  scripts/spine_client_example.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()

    if(CHECK_TIDY)
        find_package(ament_cmake_clang_tidy REQUIRED)
        # Get scripts directory from franka_ros2 package
        get_filename_component(SCRIPTS_DIR "${CMAKE_SOURCE_DIR}/../../franka_ros2/scripts" ABSOLUTE)
        set(CLANG_TIDY_TEST "${SCRIPTS_DIR}/.run_clang_tidy_filtered.sh")
        get_filename_component(SRC_DIR "${CMAKE_SOURCE_DIR}/../../.." ABSOLUTE)
        set(CLANG_TIDY_CONFIG "${SRC_DIR}/.clang-tidy")
        # Use wrapper script to filter compile_commands.json before clang-tidy
        add_test(NAME clang_tidy
            COMMAND bash "${CLANG_TIDY_TEST}" "${CMAKE_BINARY_DIR}" --jobs 6 --config "${CLANG_TIDY_CONFIG}"
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        )
    endif()
endif()

ament_package()
