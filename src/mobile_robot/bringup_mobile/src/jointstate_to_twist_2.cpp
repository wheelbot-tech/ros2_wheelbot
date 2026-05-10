// ackermann configuration
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm> 
#include <array>
#include <angles/angles.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


// *******************  for 2 diff BOTs ****************************
class JointStateToTwist : public rclcpp::Node
{
public:
JointStateToTwist(const std::string& prefix, const std::string& use_stamped,
                    double wheel_radius, double wheel_drive_len, 
                    const std::string& drive_1_pos, const std::string& drive_2_pos, 
                    const std::string& drive_3_pos, const std::string& drive_4_pos): 
    Node("jointstate_to_twist_2"), 
    prefix_(prefix), use_stamped_(use_stamped),
    wheel_radius_(wheel_radius), wheel_drive_len_(wheel_drive_len), 
    drive_1_pos_(drive_1_pos), drive_2_pos_(drive_2_pos), 
    drive_3_pos_(drive_3_pos), drive_4_pos_(drive_4_pos),
    encoder_position{0.0, 0.0, 0.0, 0.0},  // Initialize encoder positions to 0
    states_received_(false)

    {           
        // Obține namespace-ul din parametri
        std::string namespace_prefix = this->get_namespace();
        if (namespace_prefix == "/" ) {
            namespace_prefix = ""; }
        else    {
            namespace_prefix = namespace_prefix  + "/";
        }
        std::string amr_state_topic = namespace_prefix + "amr_joint_states";
        std::string amr_cmds_topic = namespace_prefix + "amr_joint_commands";
 
        auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        auto best_effort_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        
        full_joint_name_0_ = prefix_ + "virtual_front_right_steering_joint";
        full_topic_name_0_ = namespace_prefix + drive_1_pos_ + "drive_controller/cmd_vel";
        //printf("Full Joint Name after assignment: %s\n", full_joint_name_.c_str());
        //printf("Full Topic Name after assignment: %s\n", full_topic_name_0_.c_str());
        full_joint_name_1_ = prefix_ + "virtual_front_left_steering_joint";
        full_topic_name_1_ = namespace_prefix +  drive_2_pos_ + "drive_controller/cmd_vel";
        full_joint_name_2_ = prefix_ + "virtual_rear_right_steering_joint";
        full_topic_name_2_ = namespace_prefix + drive_3_pos_ + "drive_controller/cmd_vel";
        full_joint_name_3_ = prefix_ + "virtual_rear_left_steering_joint";
        full_topic_name_3_ = namespace_prefix  + drive_4_pos_ + "drive_controller/cmd_vel";


        cmd_vel_pub_0_stamped_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(full_topic_name_0_, best_effort_qos);
        cmd_vel_pub_3_stamped_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(full_topic_name_3_, best_effort_qos);

        
        subscription_amr_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            amr_state_topic, reliable_qos, std::bind(&JointStateToTwist::amr_state_callback, this, std::placeholders::_1));
        
        subscription_amr_cmds_ = this->create_subscription<sensor_msgs::msg::JointState>(
            amr_cmds_topic, reliable_qos, std::bind(&JointStateToTwist::amr_cmds_callback, this, std::placeholders::_1));
        
    }

private:
   void amr_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg_state) {
        // Find joint indices by name and extract steering positions
        bool found_any = false;
        for (size_t i = 0; i < msg_state->name.size(); ++i) {
            if (msg_state->name[i] == full_joint_name_0_ && i < msg_state->position.size()) {
                encoder_position[0] = msg_state->position[i]; // front_right steering
                found_any = true;
            } else if (msg_state->name[i] == full_joint_name_1_ && i < msg_state->position.size()) {
                encoder_position[1] = msg_state->position[i]; // front_left steering
                found_any = true;
            } else if (msg_state->name[i] == full_joint_name_2_ && i < msg_state->position.size()) {
                encoder_position[2] = msg_state->position[i];  // rear_right steering
                found_any = true;
            } else if (msg_state->name[i] == full_joint_name_3_ && i < msg_state->position.size()) {
                encoder_position[3] = msg_state->position[i]; // rear_left steering
                found_any = true;
            } 
        }
        if (!found_any && !states_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "No matching joints found! Looking for: %s, %s, %s, %s",
                full_joint_name_0_.c_str(), full_joint_name_1_.c_str(),
                full_joint_name_2_.c_str(), full_joint_name_3_.c_str());
        }
        if (found_any) {
            states_received_ = true;
        }
    }
        
    void amr_cmds_callback(const sensor_msgs::msg::JointState::SharedPtr msg_cmd) {
        // Don't process commands until we have received state data
        if (!states_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for state data before processing commands");
            return;
        }
        
        double linear_velocity[4] = {0.0, 0.0, 0.0, 0.0};
        double error_position[4] = {0.0, 0.0, 0.0, 0.0};
        double target_position[4] = {0.0, 0.0, 0.0, 0.0};
        double scale[4] = {0.0, 0.0, 0.0, 0.0};

        // Message structure is: steering0, wheel0, steering1, wheel1, steering2, wheel2, steering3, wheel3
        // So steering positions are at even indices (0,2,4,6) and wheel velocities at odd indices (1,3,5,7)
        // Ensure we have enough data (8 elements for 4 steering + 4 wheels)
        if (msg_cmd->position.size() < 8 || msg_cmd->velocity.size() < 8) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Insufficient data: position.size()=%zu, velocity.size()=%zu (need 8 each)",
                msg_cmd->position.size(), msg_cmd->velocity.size());
            return;
        }

        // Extract steering target positions from indices (0, 6)
        // and wheel velocities from indices (1, 7) - only FR and RL modules
        int module_indices[2] = {0, 3};  // FR = 0, RL = 3
        int steering_indices[2] = {0, 6};
        int wheel_indices[2] = {1, 7};
        
        for (int j = 0; j < 2; ++j)  {
            int i = module_indices[j];
            int steering_idx = steering_indices[j];
            
            target_position[i] = msg_cmd->position[steering_idx];
            
            // Compute the signed shortest angular error from current encoder to target.
            error_position[i] = angles::shortest_angular_distance(encoder_position[i], target_position[i]);
            if (fabs(error_position[i]) > M_PI_2) {
                // For large errors, provide a maximum bounded angular velocity
                error_position[i] = (error_position[i] > 0) ? M_PI_2 : -M_PI_2;
            }
            // Reduce wheel speed until the target angle has been reached
            double k = 2.0;    // 1 , 2
            double beta = 3.0; // 2 , 3
            double alpha_delta = fabs(error_position[i]);
            if (alpha_delta < 0.2) {
                scale[i] = 1.0;
            } else if (alpha_delta > 1.0) {
                scale[i] = 0.01;
            } else {
                //scale = cos(alpha_delta);
                scale[i] = 1 - std::pow(k * alpha_delta, beta);
                // Ensure the scale factor never goes below 0
                scale[i] = std::max(scale[i], 0.0);
            }
        }
        
        // Apply scaling per-module, not the minimum across both modules.
        // Each wheel should slow down based on its own steering error.
        for (int j = 0; j < 2; ++j) 
            {
                int i = module_indices[j];  // 0, 3
                int wheel_idx = wheel_indices[j];  // 1, 7
                linear_velocity[i] = msg_cmd->velocity[wheel_idx] * wheel_radius_ * scale[i];  // m/s
            }

        double angular_velocity = 0.4 ;  

        // Initialize all fields to 0 to avoid NaN in unset fields
        geometry_msgs::msg::TwistStamped twist_msg_stamped_0;
        geometry_msgs::msg::TwistStamped twist_msg_stamped_3;
        
        // Explicitly zero all twist fields
        twist_msg_stamped_0.twist.linear.x = 0.0;
        twist_msg_stamped_0.twist.linear.y = 0.0;
        twist_msg_stamped_0.twist.linear.z = 0.0;
        twist_msg_stamped_0.twist.angular.x = 0.0;
        twist_msg_stamped_0.twist.angular.y = 0.0;
        twist_msg_stamped_0.twist.angular.z = 0.0;
        
        twist_msg_stamped_3.twist.linear.x = 0.0;
        twist_msg_stamped_3.twist.linear.y = 0.0;
        twist_msg_stamped_3.twist.linear.z = 0.0;
        twist_msg_stamped_3.twist.angular.x = 0.0;
        twist_msg_stamped_3.twist.angular.y = 0.0;
        twist_msg_stamped_3.twist.angular.z = 0.0;

        // Now set the actual values
        twist_msg_stamped_0.twist.linear.x = linear_velocity[0];
        twist_msg_stamped_0.twist.angular.z = angular_velocity * error_position[0] / wheel_drive_len_ ;

        twist_msg_stamped_3.twist.linear.x = linear_velocity[3];
        twist_msg_stamped_3.twist.angular.z = angular_velocity * error_position[3] / wheel_drive_len_ ;

        twist_msg_stamped_0.header.stamp = this->get_clock()->now();
        twist_msg_stamped_3.header.stamp = this->get_clock()->now();    

        cmd_vel_pub_0_stamped_->publish(twist_msg_stamped_0);
        cmd_vel_pub_3_stamped_->publish(twist_msg_stamped_3);

       // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
       //     "cmd_vel: FR(X=%.3f, Z=%.3f Error=%.3f, )",
       //    twist_msg_stamped_0.twist.linear.x, twist_msg_stamped_0.twist.angular.z, error_position[0]);
       // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
       //     "cmd_vel: RL(X=%.3f, Z=%.3f Error=%.3f, )",
       //     twist_msg_stamped_3.twist.linear.x, twist_msg_stamped_3.twist.angular.z, error_position[3]);
        
    }
    std::string namespace_prefix;
    std::string amr_state_topic;
    std::string amr_cmds_topic;
    std::string prefix_; 
    std::string use_stamped_;
    double wheel_radius_;
    double wheel_drive_len_;
    std::string drive_1_pos_;
    std::string drive_2_pos_;
    std::string drive_3_pos_;
    std::string drive_4_pos_;
    double encoder_position[4];
    bool states_received_;
    std::string full_joint_name_0_;
    std::string full_topic_name_0_;
    std::string full_joint_name_1_;
    std::string full_topic_name_1_;
    std::string full_joint_name_2_;
    std::string full_topic_name_2_;
    std::string full_joint_name_3_;
    std::string full_topic_name_3_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_amr_states_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_amr_cmds_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_0_stamped_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_3_stamped_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    if (argc < 9) {
        std::cerr << "Usage: jointstate_to_twist <prefix> <use_stamped> <wheel_radius> <wheel_drive_len> <drive_1_pos> <drive_2_pos> <drive_3_pos> <drive_4_pos>" << std::endl;
        return 1;
    }
    auto node = std::make_shared<JointStateToTwist>(argv[1], argv[2], std::stod(argv[3]), std::stod(argv[4]), argv[5], argv[6], argv[7], argv[8]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
