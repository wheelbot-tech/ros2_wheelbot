#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;


using std::placeholders::_1;

class TestTopic : public rclcpp::Node
{
  public:  
  TestTopic() : Node("test_topic")
    {
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
      auto best_effort_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/esp32_joint_commands", default_qos, std::bind(&TestTopic::topic_callback, this, _1));
      
      joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/esp32_joint_states", best_effort_qos);
      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", default_qos);

      timer_ = this->create_wall_timer(500ms, std::bind(&TestTopic::publish_messages, this));
    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Name=%s, Vel=%f", msg->name[0].c_str(), msg->velocity[0]);
    }

    void publish_messages()
    {
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->now();
        message.name = {"left_wheel_joint", "right_wheel_joint"};
        message.position = {0.0, 0.0};
        message.velocity = {0.0, 0.0};
        message.effort = {0.0, 0.0};
        joint_states_pub_->publish(message);

        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = 0.5; 
        cmd_vel_msg.angular.z = 0.1; 
        cmd_vel_pub_->publish(cmd_vel_msg);

        RCLCPP_INFO(this->get_logger(), "Publishing measges");
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestTopic>());
  rclcpp::shutdown();
  return 0;
}




