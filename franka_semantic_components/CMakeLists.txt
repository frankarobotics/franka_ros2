cmake_minimum_required(VERSION 3.5)
project(franka_semantic_components)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    Franka
    franka_hardware
    franka_msgs
    geometry_msgs
    sensor_msgs
    hardware_interface
    controller_interface
    rclcpp_lifecycle
    urdf)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Franka 0.13.0 REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(
  franka_semantic_components SHARED
  src/franka_robot_state.cpp
  src/franka_robot_model.cpp
  src/franka_cartesian_velocity_interface.cpp
  src/franka_cartesian_pose_interface.cpp
  src/franka_semantic_component_interface.cpp
  src/translation_utils.cpp)

target_include_directories(
  franka_semantic_components PRIVATE include ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(franka_semantic_components Franka::Franka Eigen3::Eigen)

ament_target_dependencies(franka_semantic_components Franka sensor_msgs
                          geometry_msgs ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS franka_semantic_components DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(franka_msgs REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_interface REQUIRED)

  set(CPP_DIRECTORIES src include)
  ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
  ament_copyright(${CPP_DIRECTORIES} package.xml)
  ament_cppcheck(${CPP_DIRECTORIES})
  ament_lint_cmake(CMakeLists.txt)
  ament_flake8()
  ament_pep257()
  ament_xmllint()
  if(CHECK_TIDY)
    find_package(ament_cmake_clang_tidy REQUIRED)
    set(ament_cmake_clang_tidy_CONFIG_FILE ../.clang-tidy)
    ament_clang_tidy(${CMAKE_BINARY_DIR} --jobs 6)
  endif()

  ament_add_gmock(test_franka_robot_state test/franka_robot_state_test.cpp)
  target_include_directories(test_franka_robot_state PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_franka_robot_state
    franka_semantic_components
  )
  ament_target_dependencies(test_franka_robot_state
    controller_interface
    hardware_interface
    franka_msgs
    sensor_msgs
    geometry_msgs
    Franka
  )
  ament_add_gmock(test_franka_robot_model test/franka_robot_model_test.cpp)
  target_include_directories(test_franka_robot_model PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

  target_link_libraries(test_franka_robot_model
    franka_semantic_components
  )
  ament_target_dependencies(test_franka_robot_model
    controller_interface
    hardware_interface
    franka_msgs
    sensor_msgs
    geometry_msgs
    Franka
  )

  ament_add_gmock(
    franka_semantic_component_interface_test
    test/franka_semantic_interface_test.cpp
  )
  target_include_directories(franka_semantic_component_interface_test PRIVATE  ${CMAKE_CURRENT_SOURCE_DIR}/include)

  target_link_libraries(franka_semantic_component_interface_test
    franka_semantic_components
  )

  ament_target_dependencies(
    franka_semantic_component_interface_test
    hardware_interface
    sensor_msgs
    geometry_msgs
  )

  ament_add_gmock(
    franka_cartesian_velocity_interface_test
    test/franka_cartesian_velocity_interface_test.cpp
  )

  target_include_directories(franka_cartesian_velocity_interface_test PRIVATE  ${CMAKE_CURRENT_SOURCE_DIR}/include)

  target_link_libraries(franka_cartesian_velocity_interface_test
    franka_semantic_components
  )

  ament_target_dependencies(
    franka_cartesian_velocity_interface_test
    hardware_interface
  )

  ament_add_gmock(
    franka_cartesian_pose_interface_test
    test/franka_cartesian_pose_interface_test.cpp
  )

  target_include_directories(franka_cartesian_pose_interface_test PRIVATE  ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS})

  target_link_libraries(franka_cartesian_pose_interface_test
    franka_semantic_components
  )

  ament_target_dependencies(
    franka_cartesian_pose_interface_test
    hardware_interface
    sensor_msgs
    geometry_msgs
  )

  install(FILES test/robot_description_test.txt
          DESTINATION share/${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(franka_semantic_components)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
