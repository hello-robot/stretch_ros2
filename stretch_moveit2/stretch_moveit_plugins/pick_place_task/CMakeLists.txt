cmake_minimum_required(VERSION 3.5)
project(pick_place_task)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-W -Wall -Wextra
    -Wwrite-strings -Wunreachable-code -Wpointer-arith
    -Winit-self -Wredundant-decls
    -Wno-unused-parameter -Wno-unused-function)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosparam_shortcuts REQUIRED)

# System dependencies are found with CMake's conventions
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
  src/pick_place_task.cpp)
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  geometry_msgs
  rosparam_shortcuts
)
# To disable warning moveit/task_constructor/task.h
ament_target_dependencies(${PROJECT_NAME} SYSTEM
  moveit_task_constructor_core)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)

add_executable(${PROJECT_NAME}_demo src/pick_place_demo.cpp
)
target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_demo
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  rclcpp
)
# To disable warning moveit/task_constructor/task.h
ament_target_dependencies(${PROJECT_NAME}_demo SYSTEM
  moveit_task_constructor_core)

#############
## Install ##
#############

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_demo
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  Eigen3
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  rclcpp
  rosparam_shortcuts
)
ament_package()
