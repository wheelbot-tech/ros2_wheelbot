#include <memory>
#include <cmath>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"

// Simple Madgwick filter for orientation estimation
class MadgwickFilter {
 public:
  MadgwickFilter(float sampleRate = 100.0f, float beta = 0.1f)
      : sampleRate_(sampleRate), beta_(beta),
        q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f) {}

  void update(float gx, float gy, float gz, float ax, float ay, float az) {
    if (az == 0.0f && ay == 0.0f && ax == 0.0f) return;

    float norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;

    ax /= norm;
    ay /= norm;
    az /= norm;

    float q0q0 = q0_ * q0_;
    float q0q1 = q0_ * q1_;
    float q0q2 = q0_ * q2_;
    float q0q3 = q0_ * q3_;
    float q1q1 = q1_ * q1_;
    float q1q2 = q1_ * q2_;
    float q1q3 = q1_ * q3_;
    float q2q2 = q2_ * q2_;
    float q2q3 = q2_ * q3_;
    float q3q3 = q3_ * q3_;

    float S0 = 2.0f * (-q2_ * (2.0f * q1q3 - 2.0f * q0q2 - ax) + q1_ * (2.0f * q2q3 + 2.0f * q0q1 - ay) +
                       (-4.0f * q1q1 - 4.0f * q2q2) * az + q0_ * (2.0f * q1_ * ay + 2.0f * q2_ * ax)) +
               ax * (q0q0 - q1q1 - q2q2 + q3q3);
    float S1 = 2.0f * (q3_ * (2.0f * q1q2 - 2.0f * q0q3 + ax) + q2_ * (2.0f * q2q3 + 2.0f * q0q1 - ay) +
                       (4.0f * q1q1) * az - q0_ * (2.0f * q2_ * ay - 2.0f * q3_ * ax)) +
               ay * (q0q0 - q1q1 + q2q2 - q3q3);
    float S2 = 2.0f * (q1_ * (2.0f * q1q2 - 2.0f * q0q3 - ax) - q0_ * (2.0f * q1_ * az + 2.0f * q3_ * ay) +
                       (4.0f * q2q2) * az - q3_ * (2.0f * q2q3 - 2.0f * q0q1 + ay)) +
               az * (q0q0 + q1q1 - q2q2 - q3q3);
    float S3 = 2.0f * (q0_ * (2.0f * q1_ * ay + 2.0f * q2_ * ax) + q1_ * (2.0f * q3_ * ay - 2.0f * q2_ * az) +
                       q2_ * (2.0f * q3_ * ax - 2.0f * q1_ * az));

    norm = std::sqrt(S0 * S0 + S1 * S1 + S2 * S2 + S3 * S3);
    if (norm != 0.0f) {
      S0 /= norm;
      S1 /= norm;
      S2 /= norm;
      S3 /= norm;
    }

    q0_ = q0_ - beta_ * S0 * (1.0f / sampleRate_);
    q1_ = q1_ - beta_ * S1 * (1.0f / sampleRate_);
    q2_ = q2_ - beta_ * S2 * (1.0f / sampleRate_);
    q3_ = q3_ - beta_ * S3 * (1.0f / sampleRate_);

    // Integrate gyro
    q0_ = q0_ + (-q1_ * gx - q2_ * gy - q3_ * gz) * (0.5f / sampleRate_);
    q1_ = q1_ + (q0_ * gx + q2_ * gz - q3_ * gy) * (0.5f / sampleRate_);
    q2_ = q2_ + (q0_ * gy - q1_ * gz + q3_ * gx) * (0.5f / sampleRate_);
    q3_ = q3_ + (q0_ * gz + q1_ * gy - q2_ * gx) * (0.5f / sampleRate_);

    norm = std::sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    if (norm != 0.0f) {
      q0_ /= norm;
      q1_ /= norm;
      q2_ /= norm;
      q3_ /= norm;
    }
  }

  void getQuaternion(float& w, float& x, float& y, float& z) const {
    w = q0_;
    x = q1_;
    y = q2_;
    z = q3_;
  }

 private:
  float sampleRate_, beta_;
  float q0_, q1_, q2_, q3_;
};

class ImuMotionVisualizer : public rclcpp::Node {
 public:
  ImuMotionVisualizer() : Node("imu_motion_visualizer"), filter_(100.0f, 0.05f) {
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", rclcpp::SensorDataQoS(),
        std::bind(&ImuMotionVisualizer::imu_callback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("imu_markers", 10);
  }

 private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Update filter with gyro and accel data
    float gx = msg->angular_velocity.x;
    float gy = msg->angular_velocity.y;
    float gz = msg->angular_velocity.z;

    float ax = msg->linear_acceleration.x;
    float ay = msg->linear_acceleration.y;
    float az = msg->linear_acceleration.z;

    filter_.update(gx, gy, gz, ax, ay, az);

    // Create marker array
    auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();

    // Acceleration vector marker (red)
    auto accel_marker = create_arrow_marker(
        0, msg->header.frame_id, msg->header.stamp, "Acceleration",
        ax * 0.05f, ay * 0.05f, az * 0.05f, 1.0f, 0.0f, 0.0f);  // Red
    marker_array->markers.push_back(accel_marker);

    // Angular velocity marker (green)
    auto gyro_marker = create_arrow_marker(
        1, msg->header.frame_id, msg->header.stamp, "Angular Velocity",
        gx * 2.0f, gy * 2.0f, gz * 2.0f, 0.0f, 1.0f, 0.0f);  // Green (scaled)
    marker_array->markers.push_back(gyro_marker);

    // Total linear acceleration magnitude (blue sphere)
    float accel_mag = std::sqrt(ax * ax + ay * ay + az * az);
    auto accel_magnitude_marker = std::make_shared<visualization_msgs::msg::Marker>();
    accel_magnitude_marker->header.frame_id = msg->header.frame_id;
    accel_magnitude_marker->header.stamp = msg->header.stamp;
    accel_magnitude_marker->ns = "acceleration_magnitude";
    accel_magnitude_marker->id = 2;
    accel_magnitude_marker->type = visualization_msgs::msg::Marker::SPHERE;
    accel_magnitude_marker->action = visualization_msgs::msg::Marker::ADD;
    accel_magnitude_marker->pose.position.x = 0;
    accel_magnitude_marker->pose.position.y = 0;
    accel_magnitude_marker->pose.position.z = 0;
    accel_magnitude_marker->scale.x = 0.05f + accel_mag / 60.0f;
    accel_magnitude_marker->scale.y = 0.05f + accel_mag / 60.0f;
    accel_magnitude_marker->scale.z = 0.05f + accel_mag / 60.0f;
    accel_magnitude_marker->color.r = 0.0f;
    accel_magnitude_marker->color.g = 0.0f;
    accel_magnitude_marker->color.b = 1.0f;
    accel_magnitude_marker->color.a = 0.6f;
    accel_magnitude_marker->lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_array->markers.push_back(*accel_magnitude_marker);

    // Orientation frame (from Madgwick filter)
    float qw, qx, qy, qz;
    filter_.getQuaternion(qw, qx, qy, qz);

    // Frame axes (XYZ)
    auto frame_x = create_arrow_marker(
        3, msg->header.frame_id, msg->header.stamp, "Frame X",
        0.2f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, qx, qy, qz, qw);
    frame_x.pose.position.z = 0.1f;
    marker_array->markers.push_back(frame_x);

    auto frame_y = create_arrow_marker(
        4, msg->header.frame_id, msg->header.stamp, "Frame Y",
        0.0f, 0.2f, 0.0f, 0.0f, 1.0f, 0.0f, qx, qy, qz, qw);
    frame_y.pose.position.z = 0.1f;
    marker_array->markers.push_back(frame_y);

    auto frame_z = create_arrow_marker(
        5, msg->header.frame_id, msg->header.stamp, "Frame Z",
        0.0f, 0.0f, 0.2f, 0.0f, 0.0f, 1.0f, qx, qy, qz, qw);
    frame_z.pose.position.z = 0.1f;
    marker_array->markers.push_back(frame_z);

    marker_pub_->publish(*marker_array);
  }

  visualization_msgs::msg::Marker create_arrow_marker(
      int id, const std::string& frame_id, const builtin_interfaces::msg::Time& stamp,
      const std::string& ns, float x, float y, float z,
      float r, float g, float b,
      float qx = 0, float qy = 0, float qz = 0, float qw = 1) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = qx;
    marker.pose.orientation.y = qy;
    marker.pose.orientation.z = qz;
    marker.pose.orientation.w = qw;

    marker.scale.x = 0.025f;  // Shaft diameter
    marker.scale.y = 0.06f;   // Head diameter
    marker.scale.z = 0.08f;   // Head length

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.8f;

    marker.lifetime = rclcpp::Duration::from_seconds(0.1);

    // Set end point for arrow
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    marker.points.push_back(marker.pose.position);
    marker.points.push_back(p);

    return marker;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  MadgwickFilter filter_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuMotionVisualizer>());
  rclcpp::shutdown();
  return 0;
}
