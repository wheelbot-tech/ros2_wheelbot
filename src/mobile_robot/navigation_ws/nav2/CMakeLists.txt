cmake_minimum_required(VERSION 3.5)
project(nav2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(ament_cmake REQUIRED)
find_package(navigation2 REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

install(DIRECTORY launch
  config
  rviz
  maps
  behavior_tree
  DESTINATION share/${PROJECT_NAME})

ament_package()
