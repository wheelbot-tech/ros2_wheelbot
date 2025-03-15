#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "sensor_msgs/msg/joint_state.hpp" 

class AckermannToJointStateConverter : public rclcpp::Node
{
public:
    AckermannToJointStateConverter() : Node("ackermann_to_joint_state_converter")
    {
        this->declare_parameter<double>("wheel_tread", 0.227); // Default  wheel_separation
        
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        //auto best_effort_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/drive_joint_states", default_qos);

        ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/tricycle_controller/cmd_ackermann", default_qos, std::bind(&AckermannToJointStateConverter::ackermannCallback, this, std::placeholders::_1));
    }

private:
    void ackermannCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr ackermann_msg)
    {
        double wheel_tread;
        this->get_parameter("wheel_tread", wheel_tread);

        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->get_clock()->now();

        // Assuming 'left_wheel' and 'right_wheel' are the names of the joints
        joint_state_msg.name.push_back("left_wheel");
        joint_state_msg.name.push_back("right_wheel");

        // Calculate velocities for left and right wheels
        double left_wheel_velocity = ackermann_msg->speed - (ackermann_msg->steering_angle * wheel_tread) / 2;
        double right_wheel_velocity = ackermann_msg->speed + (ackermann_msg->steering_angle * wheel_tread) / 2;

        // Populate the velocity field in the JointState message
        joint_state_msg.velocity.push_back(left_wheel_velocity);
        joint_state_msg.velocity.push_back(right_wheel_velocity);

        joint_state_pub_->publish(joint_state_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannToJointStateConverter>());
    rclcpp::shutdown();
    return 0;
}
/*
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class AckermannToJointStateConverter : public rclcpp::Node
{
public:
    AckermannToJointStateConverter() : Node("ackermann_to_joint_state_converter")
    {
        this->declare_parameter<double>("wheel_tread", 0.5); // Default wheel tread
        this->declare_parameter<double>("wheel_radius", 0.15); // Default wheel radius

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/ackermann_cmd", 10, std::bind(&AckermannToJointStateConverter::ackermannCallback, this, std::placeholders::_1));
    }

private:
    void ackermannCallback(const ackermann_msgs::msg::AckermannDrive::SharedPtr ackermann_msg)
    {
        double wheel_tread, wheel_radius;
        this->get_parameter("wheel_tread", wheel_tread);
        this->get_parameter("wheel_radius", wheel_radius);

        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->get_clock()->now();

        // Add traction and steering joints
        joint_state_msg.name.push_back("traction_joint"); // For linear velocity
        joint_state_msg.name.push_back("steering_joint"); // For steering angle

        // Convert speed to traction_joint velocity (assuming direct relationship)
        double traction_velocity = ackermann_msg->speed / wheel_radius; // Convert linear speed to angular velocity for the wheel
        joint_state_msg.velocity.push_back(traction_velocity); // Assuming the traction joint represents the drive wheel(s)

        // Convert steering angle to steering_joint position
        joint_state_msg.position.push_back(ackermann_msg->steering_angle); // Direct use of steering angle

        // Calculate velocities for left and right wheels based on steering
        double left_wheel_velocity = traction_velocity - (ackermann_msg->steering_angle * wheel_tread) / (2 * wheel_radius);
        double right_wheel_velocity = traction_velocity + (ackermann_msg->steering_angle * wheel_tread) / (2 * wheel_radius);

        // Assuming left_wheel and right_wheel are the names of the wheel joints
        joint_state_msg.name.push_back("left_wheel");
        joint_state_msg.name.push_back("right_wheel");

        // Add velocities for left and right wheels
        joint_state_msg.velocity.push_back(left_wheel_velocity);
        joint_state_msg.velocity.push_back(right_wheel_velocity);

        joint_state_pub_->publish(joint_state_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannToJointStateConverter>());
    rclcpp::shutdown();
    return 0;
}
*/