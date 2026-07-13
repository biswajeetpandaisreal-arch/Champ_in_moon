cmake_minimum_required(VERSION 3.8)
project(champ_nav2_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav2_core
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_library(champ_nav2_controller SHARED
  src/champ_nav2_controller.cpp
)

target_include_directories(champ_nav2_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(champ_nav2_controller ${dependencies})

pluginlib_export_plugin_description_file(nav2_core champ_nav2_controller_plugin.xml)

install(TARGETS champ_nav2_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(champ_nav2_controller)
ament_export_dependencies(${dependencies})

ament_package()
