// ackermann configuration
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


// *******************  for 2 diff BOTs ****************************


class JointStateToTwist : public rclcpp::Node
{
public:
JointStateToTwist(const std::string& prefix, double wheel_radius, double wheel_drive_len, const std::string& drive_1_pos, const std::string& drive_2_pos ): 
    Node("jointstate_to_twist"), prefix_(prefix), wheel_radius_(wheel_radius), wheel_drive_len_(wheel_drive_len), drive_1_pos_(drive_1_pos), drive_2_pos_(drive_2_pos)

    {   
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        //auto best_effort_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        full_joint_name_1_steering_ = prefix_ + "virtual_front_right_steering_joint";
        full_joint_name_1_wheel_ = prefix_ + "virtual_rear_right_wheel_joint";
        //printf("Full Joint Name after assignment: %s\n", full_joint_name_.c_str());
        full_topic_name_1_ = "/" + drive_1_pos_ + "drive_controller/cmd_vel";
        cmd_vel_pub_1_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(full_topic_name_1_, default_qos);

        full_joint_name_2_steering_ = prefix_ + "virtual_front_left_steering_joint";
        full_joint_name_2_wheel_ = prefix_ + "virtual_rear_left_wheel_joint";
        full_topic_name_2_ = "/" + drive_2_pos_ + "drive_controller/cmd_vel";
        cmd_vel_pub_2_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(full_topic_name_2_, default_qos);
        
        subscription_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/amr_joint_states", default_qos, std::bind(&JointStateToTwist::joint_state_callback_states, this, std::placeholders::_1));
        
        subscription_cmds_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/amr_joint_commands", default_qos, std::bind(&JointStateToTwist::joint_state_callback_cmds, this, std::placeholders::_1));
        
    }

private:
    void joint_state_callback_states(const sensor_msgs::msg::JointState::SharedPtr msg_state) {

        for (size_t i = 0; i < msg_state->name.size(); ++i) {
            if (msg_state->name[i] == full_joint_name_1_steering_ ) {
                encoder_position[0] = msg_state->position[i];
            } else if (msg_state->name[i] == full_joint_name_2_steering_) {
                encoder_position[1] = msg_state->position[i];
            }
        }

    }
        
    void joint_state_callback_cmds(const sensor_msgs::msg::JointState::SharedPtr msg_cmd) {
        double linear_velocity[2] = {0.0, 0.0};
        double error_position[2] = {0.0 , 0.0};
        double target_position[2] = {0.0 , 0.0};
        double scale[2] = {0.0 , 0.0};
        geometry_msgs::msg::TwistStamped twist_msg_1;
        geometry_msgs::msg::TwistStamped twist_msg_2;

        for (size_t i = 0; i < msg_cmd->name.size(); ++i)  {
            if (msg_cmd->name[i] == full_joint_name_1_steering_) {
                target_position[0] = msg_cmd->position[i];
                error_position[0] = encoder_position[0] - target_position[0];
            } else if (msg_cmd->name[i] == full_joint_name_2_steering_) {
                target_position[1] = msg_cmd->position[i];
                error_position[1] = encoder_position[1] - target_position[1];
            }
        }

        for (int i = 0; i < 2; ++i) {         
            //RCLCPP_INFO(this->get_logger(), "Error-position(%d):  %f", i, error_position[i]);   
            // Normalize error_position to be within [-π, π]
            error_position[i] = fmod(error_position[i], 2 * M_PI);  // First, ensure it's within [0, 2π) or (-2π, 0]
            if (error_position[i] < -M_PI) {
                error_position[i] += 2 * M_PI;                      // If error is in (-2π, -π], map it to (0, π)
            } else if (error_position[i] > M_PI) {
                error_position[i] -= 2 * M_PI;                      // If error is in (π, 2π), map it to (-π, 0)
            }
            if (fabs(error_position[i]) > M_PI_2) {
                // For large errors, provide a maximum bounded angular velocity
                error_position[i] = (error_position[i] > 0) ? M_PI_2 : -M_PI_2;  
            }
            // Reduce wheel speed until the target angle has been reached
            double k = 2.0;  
            double beta = 3.0; 
            double alpha_delta = fabs(error_position[i]);
            if (alpha_delta < 0.2 ) {  
                scale[i] = 1;  
            }
            else if (alpha_delta > 1.0 ) {
                scale[i] = 0.01;  
            }
            else {
                //scale = cos(alpha_delta); 
                scale[i] = 1 - std::pow(k * alpha_delta, beta);
                // Ensure the scale factor never goes below 0
                scale[i] = std::max(scale[i], 0.0);
            }
        }
        
        double scale_min = std::min(scale[0],scale[1]);
   
        double angular_velocity = 0.4 ;  

        for (size_t i = 0; i < msg_cmd->name.size(); ++i) {
            if (msg_cmd->name[i] == full_joint_name_1_wheel_ ) {
                    linear_velocity[i] = msg_cmd->velocity[i] * wheel_radius_ * scale_min;
                    //if ( msg_cmd->velocity[i] != 0.0 ) {
                    //    RCLCPP_INFO(this->get_logger(), " cmd_vel %f Linear velocity(%d):  %f", msg_cmd->velocity[i], i,  linear_velocity[i]);
                    //}
                    twist_msg_1.twist.linear.x = linear_velocity[i] ;        
                    twist_msg_1.twist.angular.z = angular_velocity * error_position[0] / wheel_drive_len_ ;
                    twist_msg_1.header.stamp = this->get_clock()->now();
                    cmd_vel_pub_1_->publish(twist_msg_1);
            } else if (msg_cmd->name[i] == full_joint_name_2_wheel_) {
                    linear_velocity[i] = msg_cmd->velocity[i] * wheel_radius_ * scale_min;
                    //if ( msg_cmd->velocity[i] != 0.0 ) {
                    //    RCLCPP_INFO(this->get_logger(), " cmd_vel %f Linear velocity(%d):  %f", msg_cmd->velocity[i], i,  linear_velocity[i]);
                    //}
                    twist_msg_2.twist.linear.x = linear_velocity[i] ;            
                    twist_msg_2.twist.angular.z = angular_velocity * error_position[1] / wheel_drive_len_ ;
                    twist_msg_2.header.stamp = this->get_clock()->now();
                    cmd_vel_pub_2_->publish(twist_msg_2);
            }
        }
    }

    std::string prefix_;
    double wheel_radius_;
    double wheel_drive_len_;
    std::string drive_1_pos_;
    std::string drive_2_pos_;
    double encoder_position[2];
    std::string full_joint_name_1_steering_;
    std::string full_joint_name_1_wheel_;
    std::string full_topic_name_1_;
    std::string full_joint_name_2_steering_;
    std::string full_joint_name_2_wheel_;
    std::string full_topic_name_2_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_states_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_cmds_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_1_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_2_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    if (argc < 6) {
        std::cerr << "Usage: jointstate_to_twist <prefix> <wheel_radius> <wheel_drive_len> <drive_1_pos> <drive_2_pos> " << std::endl;
        return 1;
    }
    auto node = std::make_shared<JointStateToTwist>(argv[1], std::stod(argv[2]), std::stod(argv[3]), argv[4], argv[5] );
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}