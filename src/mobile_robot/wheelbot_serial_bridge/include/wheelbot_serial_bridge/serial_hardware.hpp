#pragma once

#include "wheelbot_serial_bridge/serial_protocol.hpp"

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace wheelbot_serial_bridge
{

class WheelbotSerialHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WheelbotSerialHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct ModuleMapping
  {
    std::string module;
    std::size_t steering_joint_index{0};
    std::size_t wheel_joint_index{0};
    double mounting_yaw_rad{0.0};
  };

  bool open_serial();
  void close_serial();
  bool configure_port(int fd) const;
  void read_serial_lines();
  bool write_line(const std::string & line);
  void apply_state(const ModuleState & state);
  void publish_imu_sample(const ImuSample & sample);
  void setup_imu_publishers();
  void send_zero_commands();
  void build_module_mappings();
  double get_state(std::size_t joint_index, const std::string & interface_name) const;
  double * command_ptr(std::size_t joint_index, const std::string & interface_name);
  double normalize_angle(double angle_rad) const;
  double shortest_angular_distance(double from, double to) const;
  double steering_scale(double error_rad) const;
  int baudrate_to_constant(int baudrate) const;

  std::string serial_port_{"/dev/ttyACM0"};
  int baudrate_{115200};
  int command_timeout_ms_{500};
  double wheel_radius_{0.0825};
  double wheel_drive_len_{0.23};
  double steering_gain_{0.1};
  bool use_common_speed_scale_{true};
  bool zero_steering_when_stopped_{true};
  std::vector<std::string> active_modules_{"RL", "RR", "FL", "FR"};

  int serial_fd_{-1};
  std::string rx_buffer_;
  rclcpp::Time last_open_attempt_;
  rclcpp::Time last_command_time_;

  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> position_commands_;
  std::vector<double> velocity_commands_;
  std::vector<ModuleMapping> module_mappings_;
  std::map<std::string, std::size_t> module_to_mapping_;
  rclcpp::Node::SharedPtr imu_node_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imu_publishers_;
};

}  // namespace wheelbot_serial_bridge
