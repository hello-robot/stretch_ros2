cmake_minimum_required(VERSION 3.10.2)
project(stretch_moveit2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz urdf DESTINATION share/${PROJECT_NAME})

ament_package()
