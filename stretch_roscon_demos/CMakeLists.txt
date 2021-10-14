cmake_minimum_required(VERSION 3.10.2)
project(stretch_roscon_demos)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(rviz_visual_tools REQUIRED)

add_executable(move_group_interface_demo
    src/move_group_interface_demo.cpp
)

target_include_directories(move_group_interface_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(move_group_interface_demo
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  rviz_visual_tools
)

# Install executable
install(TARGETS move_group_interface_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Install packages
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
