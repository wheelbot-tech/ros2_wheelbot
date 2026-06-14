cmake_minimum_required(VERSION 3.8)
project(bringup_mobile)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(wheelbot_serial_protocol
  src/serial_protocol.cpp
)
set_target_properties(wheelbot_serial_protocol PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(wheelbot_serial_protocol PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(wheelbot_serial_hardware SHARED
  src/serial_hardware.cpp
)
target_link_libraries(wheelbot_serial_hardware wheelbot_serial_protocol)
target_include_directories(wheelbot_serial_hardware PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(wheelbot_serial_hardware hardware_interface pluginlib rclcpp sensor_msgs)
pluginlib_export_plugin_description_file(hardware_interface wheelbot_serial_hardware.xml)

install(
  DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/record_rotation_test.py
    scripts/rotation_test_server.py
    scripts/start_rotation_test.py
    scripts/rectangle_diagonals_path.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS wheelbot_serial_protocol wheelbot_serial_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp sensor_msgs)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_serial_protocol test/test_serial_protocol.cpp)
  if(TARGET test_serial_protocol)
    target_link_libraries(test_serial_protocol wheelbot_serial_protocol)
    target_include_directories(test_serial_protocol PRIVATE include)
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
