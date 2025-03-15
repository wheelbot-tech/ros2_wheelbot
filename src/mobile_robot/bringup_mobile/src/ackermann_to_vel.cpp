#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>



class AckermannToVel : public rclcpp::Node
{
public:
    AckermannToVel(): Node("ackermann_to_vel")
    {
        // Declare the wheel_separation parameter
        this->declare_parameter<double>("wheel_separation", 0.23); 

    
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        //auto best_effort_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/drive/cmd_vel", default_qos);

        subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/tricycle_controller/cmd_ackermann", default_qos , std::bind(&AckermannToVel::ackermann_callback, this, std::placeholders::_1));
    }

private:

    geometry_msgs::msg::Twist convert_ackermann_to_twist(const ackermann_msgs::msg::AckermannDrive& ackermann_msg)
    {
        geometry_msgs::msg::Twist twist_msg;
        double wheel_separation ;
        this->get_parameter("wheel_separation", wheel_separation);
        twist_msg.linear.x = ackermann_msg.speed;
        twist_msg.angular.z = ackermann_msg.speed * tan(ackermann_msg.steering_angle) / wheel_separation;
        return twist_msg;
    }

    void ackermann_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr ackermann_msg)
    {
        auto twist_msg = convert_ackermann_to_twist(*ackermann_msg);
        cmd_vel_pub_->publish(twist_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AckermannToVel>());
    rclcpp::shutdown();
    return 0;
}