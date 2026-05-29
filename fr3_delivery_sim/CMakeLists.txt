cmake_minimum_required(VERSION 3.8)
project(fr3_delivery_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Python executables
install(PROGRAMS
  src/object_spawner.py
  src/vision_detector.py
  src/pick_and_place.py
  DESTINATION lib/${PROJECT_NAME}
)

# All share-directory content
install(DIRECTORY
  urdf
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()