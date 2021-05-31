cmake_minimum_required(VERSION 2.8.3)
project(mpc_waypoint_tracking_controller)

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
   cfg/MPCWayPointTrackerROS.cfg
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS dynamic_reconfigure
)

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/${PROJECT_NAME}_ros.cpp
)

add_dependencies(
  ${PROJECT_NAME} 
    ${PROJECT_NAME}_gencfg
)

target_link_libraries(
  ${PROJECT_NAME} 
    ${catkin_LIBRARIES}
    ipopt
)
