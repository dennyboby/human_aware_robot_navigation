cmake_minimum_required(VERSION 2.8.3)
project(timed_path_follower)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  tf2_ros
)

generate_dynamic_reconfigure_options(
   cfg/PathFollower.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
    src/path_follower.cpp
)

add_dependencies(
  ${PROJECT_NAME} 
    ${PROJECT_NAME}_gencfg
)

target_link_libraries(
  ${PROJECT_NAME} 
    ${catkin_LIBRARIES}
)
