#pragma once

#include <cstdint>
#include <optional>
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
  static constexpr uint32_t kImuValid = 0x01;
  static constexpr uint32_t kGyroCalibrated = 0x02;
  static constexpr uint32_t kAttitudeValid = 0x04;
  static constexpr uint32_t kAccelCorrectionActive = 0x08;
  static constexpr uint32_t kSampleGap = 0x10;
  static constexpr uint32_t kImuReadError = 0x20;

  std::string module;
  double orientation_x{0.0};
  double orientation_y{0.0};
  double orientation_z{0.0};
  double orientation_w{1.0};
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
  uint64_t timestamp_us{0};
  uint32_t seq{0};
  uint32_t status{0};
  bool has_orientation{false};
};

bool is_known_module(const std::string & module);
std::string normalize_module(std::string module);
std::optional<std::vector<std::string>> parse_active_modules(const std::string & value);
bool is_jetson_shutdown_request(const std::string & line);
std::optional<ModuleState> parse_state_line(const std::string & line);
std::optional<ImuSample> parse_imu_line(const std::string & line);
std::optional<ImuSample> parse_imuq_line(const std::string & line);
std::string format_velocity_command(const std::string & module, double right_rad_s, double left_rad_s);
std::string format_estop_command(const std::string & module);
std::vector<std::string> default_modules();

}  // namespace bringup_mobile
