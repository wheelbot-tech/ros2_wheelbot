#include "bringup_mobile/serial_protocol.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace bringup_mobile
{
namespace
{

std::string trim(const std::string & text)
{
  const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  });
  const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }).base();

  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

}  // namespace

std::vector<std::string> default_modules()
{
  return {"RL", "RR", "FL", "FR"};
}

std::string normalize_module(std::string module)
{
  if (!module.empty() && module.back() == '_') {
    module.pop_back();
  }

  std::transform(module.begin(), module.end(), module.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return module;
}

bool is_known_module(const std::string & module)
{
  const auto normalized = normalize_module(module);
  const auto modules = default_modules();
  return std::find(modules.begin(), modules.end(), normalized) != modules.end();
}

bool is_jetson_shutdown_request(const std::string & line)
{
  return line == "JETSON_SHUTDOWN";
}

bool is_chassis_imu_source(const std::string & source)
{
  const auto normalized = normalize_module(source);
  return normalized == "MASTER" || normalized == "BASE" || normalized == "CHASSIS";
}

std::optional<ModuleState> parse_state_line(const std::string & line)
{
  const auto cleaned = trim(line);
  if (cleaned.empty() || cleaned.front() == '#') {
    return std::nullopt;
  }

  std::istringstream stream(cleaned);
  std::string tag;
  std::string module;
  ModuleState state;

  stream >> tag >> module >> state.pos_right_rad >> state.pos_left_rad >> state.vel_right_rad_s >>
    state.vel_left_rad_s >> state.steering_rad;

  if (!stream || tag != "STATE") {
    return std::nullopt;
  }

  module = normalize_module(module);
  if (!is_known_module(module)) {
    return std::nullopt;
  }

  state.module = module;
  return state;
}

std::optional<ImuSample> parse_imu_line(const std::string & line)
{
  const auto cleaned = trim(line);
  if (cleaned.empty() || cleaned.front() == '#') {
    return std::nullopt;
  }

  std::istringstream stream(cleaned);
  std::string tag;
  std::string module;
  ImuSample sample;

  stream >> tag >> module >> sample.accel_x_m_s2 >> sample.accel_y_m_s2 >>
    sample.accel_z_m_s2 >> sample.gyro_x_rad_s >> sample.gyro_y_rad_s >>
    sample.gyro_z_rad_s >> sample.vel_right_rad_s >> sample.vel_left_rad_s >>
    sample.steering_rad >> sample.timestamp_ms >> sample.seq;

  if (!stream || tag != "IMU") {
    return std::nullopt;
  }

  module = normalize_module(module);
  if (!is_chassis_imu_source(module)) {
    return std::nullopt;
  }

  sample.module = module;
  return sample;
}

std::optional<ImuSample> parse_imuq_line(const std::string & line)
{
  const auto cleaned = trim(line);
  if (cleaned.empty() || cleaned.front() == '#') {
    return std::nullopt;
  }

  std::istringstream stream(cleaned);
  std::string tag;
  std::string module;
  std::string status;
  ImuSample sample;

  stream >> tag >> module >>
    sample.orientation_x >> sample.orientation_y >> sample.orientation_z >> sample.orientation_w >>
    sample.gyro_x_rad_s >> sample.gyro_y_rad_s >> sample.gyro_z_rad_s >>
    sample.accel_x_m_s2 >> sample.accel_y_m_s2 >> sample.accel_z_m_s2 >>
    sample.timestamp_us >> sample.seq >> status;

  if (!stream || tag != "IMUQ") {
    return std::nullopt;
  }

  module = normalize_module(module);
  if (!is_chassis_imu_source(module)) {
    return std::nullopt;
  }

  try {
    std::size_t parsed_chars = 0;
    sample.status = static_cast<uint32_t>(std::stoul(status, &parsed_chars, 0));
    if (parsed_chars != status.size()) {
      return std::nullopt;
    }
  } catch (const std::exception &) {
    return std::nullopt;
  }

  const double quaternion_norm = std::sqrt(
    sample.orientation_x * sample.orientation_x +
    sample.orientation_y * sample.orientation_y +
    sample.orientation_z * sample.orientation_z +
    sample.orientation_w * sample.orientation_w);
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-6 ||
    !std::isfinite(sample.gyro_x_rad_s) ||
    !std::isfinite(sample.gyro_y_rad_s) ||
    !std::isfinite(sample.gyro_z_rad_s) ||
    !std::isfinite(sample.accel_x_m_s2) ||
    !std::isfinite(sample.accel_y_m_s2) ||
    !std::isfinite(sample.accel_z_m_s2))
  {
    return std::nullopt;
  }

  sample.orientation_x /= quaternion_norm;
  sample.orientation_y /= quaternion_norm;
  sample.orientation_z /= quaternion_norm;
  sample.orientation_w /= quaternion_norm;
  sample.module = module;
  sample.has_orientation = true;
  return sample;
}

std::string format_velocity_command(const std::string & module, double right_rad_s, double left_rad_s)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6);
  stream << "VEL " << normalize_module(module) << " " << right_rad_s << " " << left_rad_s << "\n";
  return stream.str();
}

std::string format_estop_command(const std::string & module)
{
  return "ESTOP " + normalize_module(module) + "\n";
}

}  // namespace bringup_mobile
