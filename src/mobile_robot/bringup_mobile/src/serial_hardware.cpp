#include "bringup_mobile/serial_hardware.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace bringup_mobile
{
namespace
{

constexpr std::size_t kNotFound = std::numeric_limits<std::size_t>::max();
constexpr double kPi = 3.14159265358979323846;

std::vector<std::string> split_csv_or_spaces(const std::string & value)
{
  std::string normalized = value;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');

  std::istringstream stream(normalized);
  std::vector<std::string> result;
  std::string item;
  while (stream >> item) {
    item = normalize_module(item);
    if (is_known_module(item)) {
      result.push_back(item);
    }
  }
  return result;
}

std::string get_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name, const std::string & default_value)
{
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? default_value : it->second;
}

std::size_t find_joint(const hardware_interface::HardwareInfo & info, const std::string & name)
{
  for (std::size_t i = 0; i < info.joints.size(); ++i) {
    if (info.joints[i].name == name) {
      return i;
    }
  }
  return kNotFound;
}

}  // namespace

hardware_interface::CallbackReturn WheelbotSerialHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = get_parameter(info_, "serial_port", "/dev/ttyACM0");
  imu_topic_ = get_parameter(info_, "imu_topic", "/imu/data");
  imu_frame_id_ = get_parameter(info_, "imu_frame_id", "imu_link");
  baudrate_ = std::stoi(get_parameter(info_, "baudrate", "115200"));
  command_timeout_ms_ = std::stoi(get_parameter(info_, "command_timeout_ms", "500"));
  wheel_radius_ = std::stod(get_parameter(info_, "wheel_radius", "0.0825"));
  wheel_drive_len_ = std::stod(get_parameter(info_, "wheel_drive_len", "0.23"));
  steering_gain_ = std::stod(get_parameter(info_, "steering_gain", "0.1"));
  steering_alignment_tolerance_ =
    std::stod(get_parameter(info_, "steering_alignment_tolerance", "0.08"));
  steering_min_speed_scale_ =
    std::clamp(std::stod(get_parameter(info_, "steering_min_speed_scale", "0.15")), 0.0, 1.0);
  steering_zero_speed_error_ =
    std::stod(get_parameter(info_, "steering_zero_speed_error", "1.2"));
  smooth_arc_steering_ = get_parameter(info_, "steering_drive_mode", "strict_gate") == "smooth_arc";
  use_common_speed_scale_ = get_parameter(info_, "use_common_speed_scale", "true") != "false";
  zero_steering_when_stopped_ =
    get_parameter(info_, "zero_steering_when_stopped", "true") != "false";

  const auto modules = split_csv_or_spaces(get_parameter(info_, "active_modules", "RL,RR,FL,FR"));
  if (!modules.empty()) {
    active_modules_ = modules;
  }

  position_states_.assign(info_.joints.size(), 0.0);
  velocity_states_.assign(info_.joints.size(), 0.0);
  position_commands_.assign(info_.joints.size(), 0.0);
  velocity_commands_.assign(info_.joints.size(), 0.0);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    for (const auto & interface : joint.state_interfaces) {
      if (interface.name != hardware_interface::HW_IF_POSITION &&
        interface.name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_ERROR(rclcpp::get_logger("WheelbotSerialHardware"), "Unsupported state interface %s/%s",
          joint.name.c_str(), interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (!interface.initial_value.empty()) {
        const double initial_value = std::stod(interface.initial_value);
        if (interface.name == hardware_interface::HW_IF_POSITION) {
          position_states_[i] = initial_value;
          position_commands_[i] = initial_value;
        } else {
          velocity_states_[i] = initial_value;
          velocity_commands_[i] = initial_value;
        }
      }
    }
    for (const auto & interface : joint.command_interfaces) {
      if (interface.name != hardware_interface::HW_IF_POSITION &&
        interface.name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_ERROR(rclcpp::get_logger("WheelbotSerialHardware"), "Unsupported command interface %s/%s",
          joint.name.c_str(), interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  build_module_mappings();
  if (module_mappings_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("WheelbotSerialHardware"),
      "No active module joints found in robot description");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheelbotSerialHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  setup_imu_publishers();
  open_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheelbotSerialHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  send_zero_commands();
  close_serial();
  imu_publisher_.reset();
  imu_node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheelbotSerialHardware::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  send_zero_commands();
  close_serial();
  imu_publisher_.reset();
  imu_node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WheelbotSerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & interface : info_.joints[i].state_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(info_.joints[i].name, interface.name, &position_states_[i]);
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        interfaces.emplace_back(info_.joints[i].name, interface.name, &velocity_states_[i]);
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> WheelbotSerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto & interface : info_.joints[i].command_interfaces) {
      double * value = command_ptr(i, interface.name);
      if (value == nullptr) {
        throw std::runtime_error("Unsupported command interface: " + info_.joints[i].name + "/" + interface.name);
      }
      interfaces.emplace_back(info_.joints[i].name, interface.name, value);
    }
  }
  return interfaces;
}

hardware_interface::return_type WheelbotSerialHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (last_open_attempt_.get_clock_type() != time.get_clock_type()) {
    last_open_attempt_ = time;
  }

  if (serial_fd_ < 0 && (time - last_open_attempt_).seconds() > 1.0) {
    last_open_attempt_ = time;
    open_serial();
  }

  if (serial_fd_ >= 0) {
    read_serial_lines();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WheelbotSerialHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (last_open_attempt_.get_clock_type() != time.get_clock_type()) {
    last_open_attempt_ = time;
  }
  if (last_command_time_.get_clock_type() != time.get_clock_type()) {
    last_command_time_ = time;
  }

  if (serial_fd_ < 0) {
    if ((time - last_open_attempt_).seconds() > 1.0) {
      last_open_attempt_ = time;
      open_serial();
    }
    return hardware_interface::return_type::OK;
  }

  std::map<std::string, double> scale_by_module;
  double common_scale = 1.0;
  bool robot_is_stopped = true;
  for (const auto & mapping : module_mappings_) {
    if (std::fabs(velocity_commands_[mapping.wheel_joint_index]) > 1.0e-4) {
      robot_is_stopped = false;
      break;
    }
  }

  for (const auto & mapping : module_mappings_) {
    const double current = position_states_[mapping.steering_joint_index];
    const double target = (zero_steering_when_stopped_ && robot_is_stopped)
      ? 0.0
      : position_commands_[mapping.steering_joint_index];
    const double scale = steering_scale(shortest_angular_distance(current, target));
    scale_by_module[mapping.module] = scale;
    common_scale = std::min(common_scale, scale);
  }

  bool ok = true;
  for (const auto & mapping : module_mappings_) {
    const double current = position_states_[mapping.steering_joint_index];
    const double target = (zero_steering_when_stopped_ && robot_is_stopped)
      ? 0.0
      : position_commands_[mapping.steering_joint_index];
    const double error = shortest_angular_distance(current, target);
    const double scale = use_common_speed_scale_ ? common_scale : scale_by_module[mapping.module];
    const double linear_m_s = velocity_commands_[mapping.wheel_joint_index] * wheel_radius_ * scale;
    const double angular_rad_s = steering_gain_ * error / wheel_drive_len_;
    const double right_rad_s =
      (linear_m_s + angular_rad_s * wheel_drive_len_ / 2.0) / wheel_radius_;
    const double left_rad_s =
      (linear_m_s - angular_rad_s * wheel_drive_len_ / 2.0) / wheel_radius_;

    ok = write_line(format_velocity_command(mapping.module, right_rad_s, left_rad_s)) && ok;
  }

  if (ok) {
    last_command_time_ = time;
    return hardware_interface::return_type::OK;
  }

  close_serial();
  return hardware_interface::return_type::ERROR;
}

bool WheelbotSerialHardware::open_serial()
{
  close_serial();

  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_WARN(rclcpp::get_logger("WheelbotSerialHardware"), "Cannot open %s: %s",
      serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  if (!configure_port(serial_fd_)) {
    close_serial();
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("WheelbotSerialHardware"), "Opened WheelBot serial port %s at %d baud",
    serial_port_.c_str(), baudrate_);
  return true;
}

void WheelbotSerialHardware::close_serial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool WheelbotSerialHardware::configure_port(int fd) const
{
  termios tty {};
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("WheelbotSerialHardware"), "tcgetattr failed: %s", std::strerror(errno));
    return false;
  }

  cfmakeraw(&tty);
  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  const int baud = baudrate_to_constant(baudrate_);
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_WARN(rclcpp::get_logger("WheelbotSerialHardware"), "tcsetattr failed: %s", std::strerror(errno));
    return false;
  }

  tcflush(fd, TCIOFLUSH);
  return true;
}

void WheelbotSerialHardware::read_serial_lines()
{
  std::array<char, 256> buffer {};
  while (true) {
    const ssize_t count = ::read(serial_fd_, buffer.data(), buffer.size());
    if (count > 0) {
      rx_buffer_.append(buffer.data(), static_cast<std::size_t>(count));
    } else if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("WheelbotSerialHardware"), "Serial read failed: %s", std::strerror(errno));
      close_serial();
      return;
    }
  }

  std::size_t newline = rx_buffer_.find_first_of("\r\n");
  while (newline != std::string::npos) {
    const auto line = rx_buffer_.substr(0, newline);
    rx_buffer_.erase(0, newline + 1);
    if (const auto state = parse_state_line(line)) {
      apply_state(*state);
    } else if (const auto imu_sample = parse_imu_line(line)) {
      publish_imu_sample(*imu_sample);
    }
    newline = rx_buffer_.find_first_of("\r\n");
  }

  if (rx_buffer_.size() > 512) {
    rx_buffer_.erase(0, rx_buffer_.size() - 512);
  }
}

bool WheelbotSerialHardware::write_line(const std::string & line)
{
  const char * data = line.data();
  std::size_t remaining = line.size();
  while (remaining > 0) {
    const ssize_t count = ::write(serial_fd_, data, remaining);
    if (count > 0) {
      data += count;
      remaining -= static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return true;
    }
    RCLCPP_WARN(rclcpp::get_logger("WheelbotSerialHardware"), "Serial write failed: %s", std::strerror(errno));
    return false;
  }
  return true;
}

void WheelbotSerialHardware::apply_state(const ModuleState & state)
{
  const auto it = module_to_mapping_.find(state.module);
  if (it == module_to_mapping_.end()) {
    return;
  }

  const auto & mapping = module_mappings_[it->second];
  position_states_[mapping.steering_joint_index] =
    normalize_angle(state.steering_rad + mapping.mounting_yaw_rad);
  velocity_states_[mapping.steering_joint_index] = 0.0;
  if (mapping.module_steering_joint_index != kNotFound) {
    position_states_[mapping.module_steering_joint_index] =
      normalize_angle(state.steering_rad + mapping.mounting_yaw_rad);
    velocity_states_[mapping.module_steering_joint_index] = 0.0;
  }
  position_states_[mapping.wheel_joint_index] = 0.5 * (state.pos_right_rad + state.pos_left_rad);
  velocity_states_[mapping.wheel_joint_index] = 0.5 * (state.vel_right_rad_s + state.vel_left_rad_s);
  if (mapping.right_wheel_joint_index != kNotFound) {
    position_states_[mapping.right_wheel_joint_index] = state.pos_right_rad;
    velocity_states_[mapping.right_wheel_joint_index] = state.vel_right_rad_s;
  }
  if (mapping.left_wheel_joint_index != kNotFound) {
    position_states_[mapping.left_wheel_joint_index] = state.pos_left_rad;
    velocity_states_[mapping.left_wheel_joint_index] = state.vel_left_rad_s;
  }
}

void WheelbotSerialHardware::publish_imu_sample(const ImuSample & sample)
{
  if (sample.module != "MASTER" && sample.module != "BASE" && sample.module != "CHASSIS") {
    return;
  }
  if (!imu_publisher_) {
    return;
  }

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = imu_node_ ? imu_node_->now() : rclcpp::Clock().now();
  msg.header.frame_id = imu_frame_id_;
  msg.orientation_covariance[0] = -1.0;
  msg.angular_velocity.x = sample.gyro_x_rad_s;
  msg.angular_velocity.y = sample.gyro_y_rad_s;
  msg.angular_velocity.z = sample.gyro_z_rad_s;
  msg.linear_acceleration.x = sample.accel_x_m_s2;
  msg.linear_acceleration.y = sample.accel_y_m_s2;
  msg.linear_acceleration.z = sample.accel_z_m_s2;

  imu_publisher_->publish(msg);
}

void WheelbotSerialHardware::setup_imu_publishers()
{
  if (!imu_node_) {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    imu_node_ = std::make_shared<rclcpp::Node>("wheelbot_serial_imu_bridge", options);
  }

  imu_publisher_ = imu_node_->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO(rclcpp::get_logger("WheelbotSerialHardware"),
    "Publishing chassis IMU from ESP-NOW master on %s with frame_id %s",
    imu_topic_.c_str(), imu_frame_id_.c_str());
}

void WheelbotSerialHardware::send_zero_commands()
{
  if (serial_fd_ < 0) {
    return;
  }
  for (const auto & module : active_modules_) {
    write_line(format_velocity_command(module, 0.0, 0.0));
  }
}

void WheelbotSerialHardware::build_module_mappings()
{
  struct ModuleJointNames
  {
    std::string steering;
    std::string virtual_wheel;
    std::string module_steering;
    std::string right_wheel;
    std::string left_wheel;
  };

  const std::map<std::string, ModuleJointNames> joints_by_module = {
    {"FR", {"virtual_front_right_steering_joint", "virtual_front_right_wheel_joint",
      "FR_steering_joint", "FR_drive_right_joint", "FR_drive_left_joint"}},
    {"FL", {"virtual_front_left_steering_joint", "virtual_front_left_wheel_joint",
      "FL_steering_joint", "FL_drive_right_joint", "FL_drive_left_joint"}},
    {"RR", {"virtual_rear_right_steering_joint", "virtual_rear_right_wheel_joint",
      "RR_steering_joint", "RR_drive_right_joint", "RR_drive_left_joint"}},
    {"RL", {"virtual_rear_left_steering_joint", "virtual_rear_left_wheel_joint",
      "RL_steering_joint", "RL_drive_right_joint", "RL_drive_left_joint"}},
  };

  module_mappings_.clear();
  module_to_mapping_.clear();
  for (const auto & module : active_modules_) {
    const auto config_it = joints_by_module.find(module);
    if (config_it == joints_by_module.end()) {
      continue;
    }
    const auto steering_index = find_joint(info_, config_it->second.steering);
    const auto wheel_index = find_joint(info_, config_it->second.virtual_wheel);
    if (steering_index == kNotFound || wheel_index == kNotFound) {
      RCLCPP_WARN(rclcpp::get_logger("WheelbotSerialHardware"),
        "Skipping module %s because virtual joints are missing", module.c_str());
      continue;
    }
    const auto module_steering_index = find_joint(info_, config_it->second.module_steering);
    const auto right_wheel_index = find_joint(info_, config_it->second.right_wheel);
    const auto left_wheel_index = find_joint(info_, config_it->second.left_wheel);
    module_to_mapping_[module] = module_mappings_.size();
    module_mappings_.push_back(
      {module, steering_index, wheel_index, module_steering_index, right_wheel_index,
        left_wheel_index, module == "RL" ? kPi : 0.0});
  }
}

double WheelbotSerialHardware::get_state(std::size_t joint_index, const std::string & interface_name) const
{
  if (interface_name == hardware_interface::HW_IF_POSITION) {
    return position_states_[joint_index];
  }
  if (interface_name == hardware_interface::HW_IF_VELOCITY) {
    return velocity_states_[joint_index];
  }
  return 0.0;
}

double * WheelbotSerialHardware::command_ptr(std::size_t joint_index, const std::string & interface_name)
{
  if (interface_name == hardware_interface::HW_IF_POSITION) {
    return &position_commands_[joint_index];
  }
  if (interface_name == hardware_interface::HW_IF_VELOCITY) {
    return &velocity_commands_[joint_index];
  }
  return nullptr;
}

double WheelbotSerialHardware::normalize_angle(double angle_rad) const
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

double WheelbotSerialHardware::shortest_angular_distance(double from, double to) const
{
  return normalize_angle(to - from);
}

double WheelbotSerialHardware::steering_scale(double error_rad) const
{
  const double abs_error = std::fabs(error_rad);
  if (abs_error <= steering_alignment_tolerance_) {
    return 1.0;
  }
  if (!smooth_arc_steering_) {
    return 0.0;
  }

  const double zero_speed_error =
    std::max(steering_zero_speed_error_, steering_alignment_tolerance_ + 1.0e-6);
  if (abs_error >= zero_speed_error) {
    return steering_min_speed_scale_;
  }

  const double ratio =
    (abs_error - steering_alignment_tolerance_) / (zero_speed_error - steering_alignment_tolerance_);
  return steering_min_speed_scale_ + (1.0 - steering_min_speed_scale_) * (1.0 - ratio);
}

int WheelbotSerialHardware::baudrate_to_constant(int baudrate) const
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      RCLCPP_WARN(rclcpp::get_logger("WheelbotSerialHardware"),
        "Unsupported baudrate %d, using 115200", baudrate);
      return B115200;
  }
}

}  // namespace bringup_mobile

PLUGINLIB_EXPORT_CLASS(bringup_mobile::WheelbotSerialHardware, hardware_interface::SystemInterface)
