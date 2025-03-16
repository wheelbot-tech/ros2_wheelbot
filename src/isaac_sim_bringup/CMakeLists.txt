cmake_minimum_required(VERSION 3.8)
project(isaac_sim_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
  DIRECTORY
    config
    launch
    tools
    isaac_worlds
    isaac_description
  DESTINATION
    share/${PROJECT_NAME}/
)
add_executable(test_wheel_command src/test_wheel_command.cpp)
ament_target_dependencies(test_wheel_command rclcpp sensor_msgs)

install(TARGETS test_wheel_command
  DESTINATION lib/${PROJECT_NAME})

ament_package()
