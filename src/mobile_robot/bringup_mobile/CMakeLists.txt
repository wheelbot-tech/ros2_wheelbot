cmake_minimum_required(VERSION 3.5)
project(bringup_mobile)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(twist_mux REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(topic_based_ros2_control REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(steering_controllers_library REQUIRED)
find_package(swerve_drive_controller REQUIRED)
find_package(angles REQUIRED)

install(
  DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

add_executable(jointstate_to_twist_2 src/jointstate_to_twist_2.cpp)
ament_target_dependencies(jointstate_to_twist_2 rclcpp geometry_msgs sensor_msgs angles)

add_executable(jointstate_sim_aggregator_2 src/jointstate_sim_aggregator_2.cpp)
ament_target_dependencies(jointstate_sim_aggregator_2 rclcpp sensor_msgs)

add_executable(jointstate_to_twist_4 src/jointstate_to_twist_4.cpp)
ament_target_dependencies(jointstate_to_twist_4 rclcpp geometry_msgs sensor_msgs)

add_executable(jointstate_sim_aggregator_4 src/jointstate_sim_aggregator_4.cpp)
ament_target_dependencies(jointstate_sim_aggregator_4 rclcpp sensor_msgs)



add_executable(jointstate_to_twist_2_ackermann src/jointstate_to_twist_2_ackermann.cpp)
ament_target_dependencies(jointstate_to_twist_2_ackermann rclcpp geometry_msgs sensor_msgs)

add_executable(jointstate_aggregator_2 src/jointstate_aggregator_2.cpp)
ament_target_dependencies(jointstate_aggregator_2 rclcpp sensor_msgs)

add_executable(jointstate_aggregator_2_swerve src/jointstate_aggregator_2_swerve.cpp)
ament_target_dependencies(jointstate_aggregator_2_swerve rclcpp sensor_msgs)

add_executable(jointstate_aggregator_2_ackermann src/jointstate_aggregator_2_ackermann.cpp)
ament_target_dependencies(jointstate_aggregator_2_ackermann rclcpp sensor_msgs)

add_executable(jointstate_aggregator_4 src/jointstate_aggregator_4.cpp)
ament_target_dependencies(jointstate_aggregator_4 rclcpp sensor_msgs)


install(
  TARGETS
    jointstate_to_twist_2
    jointstate_sim_aggregator_2
    jointstate_to_twist_4
    jointstate_sim_aggregator_4
    jointstate_aggregator_2
    jointstate_aggregator_2_swerve
    jointstate_to_twist_2_ackermann
    jointstate_aggregator_2_ackermann
    jointstate_aggregator_4

  DESTINATION 
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
