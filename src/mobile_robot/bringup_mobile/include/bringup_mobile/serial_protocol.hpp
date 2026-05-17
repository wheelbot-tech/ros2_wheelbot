#pragma once

#include <optional>
#include <cstdint>
#include <string>
#include <vector>

namespace bringup_mobile
{

struct ModuleState
{
  std::string module;
  double pos_right_rad{0.0};
  double pos_left_rad{0.0};
  double vel_right_rad_s{0.0};
  double vel_left_rad_s{0.0};
  double steering_rad{0.0};
};

struct ImuSample
{
  std::string module;
  double accel_x_m_s2{0.0};
  double accel_y_m_s2{0.0};
  double accel_z_m_s2{0.0};
  double gyro_x_rad_s{0.0};
  double gyro_y_rad_s{0.0};
  double gyro_z_rad_s{0.0};
  double vel_right_rad_s{0.0};
  double vel_left_rad_s{0.0};
  double steering_rad{0.0};
  uint32_t timestamp_ms{0};
  uint32_t seq{0};
};

bool is_known_module(const std::string & module);
std::string normalize_module(std::string module);
std::optional<ModuleState> parse_state_line(const std::string & line);
std::optional<ImuSample> parse_imu_line(const std::string & line);
std::string format_velocity_command(const std::string & module, double right_rad_s, double left_rad_s);
std::string format_estop_command(const std::string & module);
std::vector<std::string> default_modules();

}  // namespace bringup_mobile
