// bicycle configuration
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// *******************  for 1 diff BOT ****************************

class JointStateToTwist : public rclcpp::Node
{
public:
    JointStateToTwist(const std::string& prefix, double wheel_radius, const std::string& drive_pos, const std::string& steering_pos, double wheel_drive_len): 
        Node("jointstate_to_twist"), prefix_(prefix), wheel_radius_(wheel_radius), drive_pos_(drive_pos), steering_pos_(steering_pos), wheel_drive_len_(wheel_drive_len)
    {   
        //printf("Prefix: %s, Steering Pos: %s\n", prefix_.c_str(), steering_pos_.c_str());


        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        //auto best_effort_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        full_joint_name_ = prefix_ + steering_pos_ + "steering_wheel_joint";
        //printf("Full Joint Name after assignment: %s\n", full_joint_name_.c_str());
        full_topic_name_ = "/"+ drive_pos_ + "drive_controller/cmd_vel";

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(full_topic_name_, default_qos);

        subscription_2_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/amr_joint_states", default_qos, std::bind(&JointStateToTwist::joint_state_callback_2, this, std::placeholders::_1));
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/amr_joint_commands", default_qos, std::bind(&JointStateToTwist::joint_state_callback, this, std::placeholders::_1));
        
    }

private:
    void joint_state_callback_2(const sensor_msgs::msg::JointState::SharedPtr msg_state) {
                  encoder_position_ = msg_state->position[0];
    }
        
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg_cmd) {

        geometry_msgs::msg::TwistStamped twist_msg;

        double target_position = msg_cmd->position[0] ;                  // rad
        double error_position = encoder_position_ - target_position;     
        // Normalize error_position to be within [-π, π]
        error_position = fmod(error_position, 2 * M_PI); // First, ensure it's within [0, 2π) or (-2π, 0]
        if (error_position < -M_PI) {
            error_position += 2 * M_PI;                  // If error is in (-2π, -π], map it to (0, π)
        } else if (error_position > M_PI) {
            error_position -= 2 * M_PI;                  // If error is in (π, 2π), map it to (-π, 0)
        }
        if (fabs(error_position) > M_PI_2) {
            // For large errors, provide a maximum bounded angular velocity
            error_position = (error_position > 0) ? M_PI_2 : -M_PI_2;
        } 
        // Reduce wheel speed until the target angle has been reached
        double k = 2.0;    // 1 , 2
        double beta = 3.0; // 2 , 3
        double alpha_delta = fabs(error_position);
        double scale;
        if (alpha_delta < 0.2 ) {  
             scale = 1;  
        }
        else if (alpha_delta > 1.0 )   {
            scale = 0.01;  
        }
        else {
            //scale = cos(alpha_delta);   
            scale = 1 - std::pow(k * alpha_delta, beta);
            // Ensure the scale factor never goes below 0
            scale = std::max(scale, 0.0);
        }

        double linear_velocity = msg_cmd->velocity[0] * wheel_radius_ ;  // m/s
        double angular_velocity = 0.4 ; 
        twist_msg.twist.linear.x = linear_velocity * scale;            
        // twist_msg.twist.angular.z = fabs(linear_velocity) * error_position / wheel_drive_len_;
        twist_msg.twist.angular.z = angular_velocity * error_position / wheel_drive_len_;

        twist_msg.header.stamp = this->get_clock()->now();

        cmd_vel_pub_->publish(twist_msg);
        
    }

    std::string prefix_;
    double wheel_radius_;
    std::string drive_pos_;
    std::string steering_pos_;
    double wheel_drive_len_;

    double encoder_position_;
    std::string full_joint_name_;
    std::string full_topic_name_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_2_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    if (argc < 6) {
        std::cerr << "Usage: jointstate_to_twist <prefix> <wheel_radius> <drive_pos> <steering_pos> <wheel_drive_len>" << std::endl;
        return 1;
    }
    auto node = std::make_shared<JointStateToTwist>(argv[1], std::stod(argv[2]), argv[3], argv[4], std::stod(argv[5]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}