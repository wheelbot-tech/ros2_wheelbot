cmake_minimum_required(VERSION 3.5)
project(bringup)

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
find_package(topic_based_ros2_control REQUIRED)
find_package(ackermann_steering_controller REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bringup_mobile REQUIRED)
find_package(description_mobile REQUIRED)
find_package(mm_moveit_config REQUIRED)
find_package(description REQUIRED)
find_package(swerve_controller REQUIRED)
find_package(joint_trajectory_controller REQUIRED)

install(
  DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
