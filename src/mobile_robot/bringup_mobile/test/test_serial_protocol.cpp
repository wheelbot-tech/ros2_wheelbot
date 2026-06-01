#include "bringup_mobile/serial_protocol.hpp"

#include <gtest/gtest.h>

namespace bringup_mobile
{

TEST(SerialProtocol, ParsesMasterStateLine)
{
  const auto state = parse_state_line("STATE RL 1.000000 2.000000 3.000000 4.000000 0.500000");

  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->module, "RL");
  EXPECT_DOUBLE_EQ(state->pos_right_rad, 1.0);
  EXPECT_DOUBLE_EQ(state->pos_left_rad, 2.0);
  EXPECT_DOUBLE_EQ(state->vel_right_rad_s, 3.0);
  EXPECT_DOUBLE_EQ(state->vel_left_rad_s, 4.0);
  EXPECT_DOUBLE_EQ(state->steering_rad, 0.5);
}

TEST(SerialProtocol, IgnoresNonStateLines)
{
  EXPECT_FALSE(parse_state_line("OK VEL RL 1.000 2.000").has_value());
  EXPECT_FALSE(parse_state_line("ERR usage: VEL RL 0.0 0.0").has_value());
  EXPECT_FALSE(parse_state_line("# comment").has_value());
}

TEST(SerialProtocol, ParsesMasterImuLine)
{
  const auto sample =
    parse_imu_line("IMU MASTER 0.123986 -0.531202 9.819922 -0.003196 0.000000 0.001065 "
                   "0.000000 0.000000 -2.419917 151860 42");

  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->module, "MASTER");
  EXPECT_DOUBLE_EQ(sample->accel_x_m_s2, 0.123986);
  EXPECT_DOUBLE_EQ(sample->accel_y_m_s2, -0.531202);
  EXPECT_DOUBLE_EQ(sample->accel_z_m_s2, 9.819922);
  EXPECT_DOUBLE_EQ(sample->gyro_x_rad_s, -0.003196);
  EXPECT_DOUBLE_EQ(sample->gyro_y_rad_s, 0.0);
  EXPECT_DOUBLE_EQ(sample->gyro_z_rad_s, 0.001065);
  EXPECT_DOUBLE_EQ(sample->vel_right_rad_s, 0.0);
  EXPECT_DOUBLE_EQ(sample->vel_left_rad_s, 0.0);
  EXPECT_DOUBLE_EQ(sample->steering_rad, -2.419917);
  EXPECT_EQ(sample->timestamp_ms, 151860u);
  EXPECT_EQ(sample->seq, 42u);
}

TEST(SerialProtocol, IgnoresNonImuLines)
{
  EXPECT_FALSE(parse_imu_line("STATE FR 1.0 2.0 3.0 4.0 5.0").has_value());
  EXPECT_FALSE(parse_imu_line("IMU FR 0 0 0 0 0 0 0 0 0 0 0").has_value());
  EXPECT_FALSE(parse_imu_line("IMU XX 0 0 0 0 0 0 0 0 0 0 0").has_value());
  EXPECT_FALSE(parse_imu_line("IMU MASTER 0 0 0").has_value());
}

TEST(SerialProtocol, FormatsVelocityCommandInRightLeftOrder)
{
  EXPECT_EQ(format_velocity_command("rl_", 1.25, -2.5), "VEL RL 1.250000 -2.500000\n");
}

TEST(SerialProtocol, FormatsEstopCommand)
{
  EXPECT_EQ(format_estop_command("all"), "ESTOP ALL\n");
}

}  // namespace bringup_mobile
