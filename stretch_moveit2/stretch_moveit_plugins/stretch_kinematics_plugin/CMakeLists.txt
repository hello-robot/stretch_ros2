cmake_minimum_required(VERSION 3.10.2)
project(stretch_kinematics_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(random_numbers REQUIRED)
find_package(class_loader REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/stretch_kinematics_plugin.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  random_numbers
  pluginlib
  moveit_core
  moveit_msgs
  EIGEN3
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

pluginlib_export_plugin_description_file(moveit_core stretch_kinematics_plugin_description.xml)
ament_package()
