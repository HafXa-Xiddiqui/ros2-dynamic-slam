cmake_minimum_required(VERSION 3.5)
project(my_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(moving_box_cmd_vel_plugin SHARED
  src/moving_box_cmd_vel_plugin.cpp)

ament_target_dependencies(moving_box_cmd_vel_plugin
  rclcpp
  geometry_msgs
  nav_msgs
)

target_link_libraries(moving_box_cmd_vel_plugin
  ${GAZEBO_LIBRARIES}
)

install(TARGETS moving_box_cmd_vel_plugin
  LIBRARY DESTINATION lib)

install(
  PROGRAMS scripts/moving_box_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
