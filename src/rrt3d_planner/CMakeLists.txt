cmake_minimum_required(VERSION 3.0.2)
project(rrt3d_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  pluginlib
  costmap_2d
  geometry_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  visualization_msgs
  octomap_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrt3d_planner
  CATKIN_DEPENDS roscpp nav_core pluginlib costmap_2d geometry_msgs nav_msgs tf tf2 tf2_ros tf2_geometry_msgs sensor_msgs visualization_msgs octomap_msgs pcl_ros pcl_conversions
)

add_compile_options(-std=c++17)

find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrt3d_planner
  CATKIN_DEPENDS roscpp nav_core pluginlib costmap_2d geometry_msgs nav_msgs octomap_msgs tf2_ros tf2_geometry_msgs visualization_msgs
  DEPENDS octomap
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(rrt3d_planner
  src/informed_rrt3d.cpp
)

target_link_libraries(rrt3d_planner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

install(TARGETS rrt3d_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES informed_rrt3d_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
