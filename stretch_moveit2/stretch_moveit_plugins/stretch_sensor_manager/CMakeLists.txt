cmake_minimum_required(VERSION 3.10.2)
project(stretch_sensor_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosparam_shortcuts REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/stretch_sensor_manager.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rosparam_shortcuts
  moveit_ros_planning_interface
  tf2_eigen
)
target_include_directories(${PROJECT_NAME}
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE $<INSTALL_INTERFACE:include>)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(moveit_core stretch_sensor_manager_plugin_description.xml)
ament_package()
