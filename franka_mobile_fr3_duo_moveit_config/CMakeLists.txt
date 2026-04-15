cmake_minimum_required(VERSION 3.22)
project(franka_mobile_fr3_duo_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})


ament_package()

install(DIRECTORY launch config urdf DESTINATION share/${PROJECT_NAME})
