// ackermann configuration
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm> 
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


// *******************  for 2 diff BOTs ****************************
class JointStateToTwist : public rclcpp::Node
{
public:
JointStateToTwist(const std::string& prefix, double wheel_radius, double wheel_drive_len, 
                    const std::string& drive_1_pos, const std::string& drive_2_pos, 
                    const std::string& drive_3_pos, const std::string& drive_4_pos ): 
    Node("jointstate_to_twist"), 
    prefix_(prefix), 
    wheel_radius_(wheel_radius), wheel_drive_len_(wheel_drive_len), 
    drive_1_pos_(drive_1_pos), drive_2_pos_(drive_2_pos), 
    drive_3_pos_(drive_3_pos), drive_4_pos_(drive_4_pos)

    {           
        // Obține namespace-ul din parametri
        std::string namespace_prefix = this->get_namespace();
        std::string amr_state_topic = namespace_prefix + "/amr_joint_states";
        std::string amr_cmds_topic = namespace_prefix + "/amr_joint_commands";

        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        //auto best_effort_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        full_joint_name_0_ = prefix_ + "virtual_front_right_steering_joint";
        full_topic_name_0_ = namespace_prefix + "/" + drive_1_pos_ + "drive_controller/cmd_vel";
        //printf("Full Joint Name after assignment: %s\n", full_joint_name_.c_str());
        //printf("Full Topic Name after assignment: %s\n", full_topic_name_0_.c_str());
        cmd_vel_pub_0_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(full_topic_name_0_, default_qos);

        full_joint_name_1_ = prefix_ + "virtual_front_left_steering_joint";
        full_topic_name_1_ = namespace_prefix + "/" + drive_2_pos_ + "drive_controller/cmd_vel";
        //cmd_vel_pub_1_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(full_topic_name_1_, default_qos);

        full_joint_name_2_ = prefix_ + "virtual_rear_right_steering_joint";
        full_topic_name_2_ = namespace_prefix + "/" + drive_3_pos_ + "drive_controller/cmd_vel";
        //cmd_vel_pub_2_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(full_topic_name_2_, default_qos);

        full_joint_name_3_ = prefix_ + "virtual_rear_left_steering_joint";
        full_topic_name_3_ = namespace_prefix + "/" + drive_4_pos_ + "drive_controller/cmd_vel";
        cmd_vel_pub_3_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(full_topic_name_3_, default_qos);
        
        subscription_amr_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            amr_state_topic, default_qos, std::bind(&JointStateToTwist::amr_state_callback, this, std::placeholders::_1));
        
        subscription_amr_cmds_ = this->create_subscription<sensor_msgs::msg::JointState>(
            amr_cmds_topic, default_qos, std::bind(&JointStateToTwist::amr_cmds_callback, this, std::placeholders::_1));
        
    }

private:
    void amr_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg_state) {

        for (size_t i = 0; i < msg_state->name.size(); ++i) {
            if (msg_state->name[i] == full_joint_name_0_ ) {
                encoder_position[0] = msg_state->position[0]; // front_right
            } else if (msg_state->name[i] == full_joint_name_1_) {
                encoder_position[1] = msg_state->position[2]; // front_left
            } else if (msg_state->name[i] == full_joint_name_2_) {
                encoder_position[2] = msg_state->position[4];  // rear_right
            } else if (msg_state->name[i] == full_joint_name_3_) {
                encoder_position[3] = msg_state->position[6]; // rear_left
                //RCLCPP_INFO(this->get_logger(), "joint name_3: %s", full_joint_name_3_.c_str());
            } 
        }

    }
        
    void amr_cmds_callback(const sensor_msgs::msg::JointState::SharedPtr msg_cmd) {
        double linear_velocity[4] = {0.0, 0.0, 0.0, 0.0};
        double error_position[4] = {0.0, 0.0, 0.0, 0.0};
        double target_position[4] = {0.0, 0.0, 0.0, 0.0};
        double scale[4] = {0.0, 0.0, 0.0, 0.0};
        geometry_msgs::msg::TwistStamped twist_msg_0;
        geometry_msgs::msg::TwistStamped twist_msg_1;
        geometry_msgs::msg::TwistStamped twist_msg_2;
        geometry_msgs::msg::TwistStamped twist_msg_3;

        for (int i = 0; i < 4; ++i)  {
            target_position[i] = msg_cmd->position[i]; 
            error_position[i] = encoder_position[i] - target_position[i];  
            //RCLCPP_INFO(this->get_logger(), "Error-position(%d):  %f", i, error_position[i]);   
            // Normalize error_position to be within [-π, π]
            error_position[i] = fmod(error_position[i], 2 * M_PI); // First, ensure it's within [0, 2π) or (-2π, 0]
            if (error_position[i] < -M_PI) {
                error_position[i] += 2 * M_PI;                  // If error is in (-2π, -π], map it to (0, π)
            } else if (error_position[i] > M_PI) {
                error_position[i] -= 2 * M_PI;                  // If error is in (π, 2π), map it to (-π, 0)
            }
            if (fabs(error_position[i]) > M_PI_2) {
                // For large errors, provide a maximum bounded angular velocity
                error_position[i] = (error_position[i] > 0) ? M_PI_2 : -M_PI_2;
            } 
            // Reduce wheel speed until the target angle has been reached
            double k = 2.0;    // 1 , 2
            double beta = 3.0; // 2 , 3
            double alpha_delta = fabs(error_position[i]);
            if (alpha_delta < 0.2 ) {  
                scale[i] = 1;  
            }
            else if (alpha_delta > 1.0 )   {
                scale[i] = 0.01;  
            }
            else {
                //scale = cos(alpha_delta);   
                scale[i] = 1 - std::pow(k * alpha_delta, beta);
                // Ensure the scale factor never goes below 0
                scale[i] = std::max(scale[i], 0.0);
            }
        }
        
        //double scale_min = *std::min_element(std::begin(scale), std::end(scale));
        double scale_min = std::min(scale[0], scale[3]);
        for (int i = 0; i < 4; ++i) 
            {
                linear_velocity[i] = msg_cmd->velocity[i] * wheel_radius_ * scale_min;  // m/s
            }
        double angular_velocity = 0.8 ;  

        twist_msg_0.twist.linear.x = linear_velocity[0];        
        twist_msg_0.twist.angular.z = angular_velocity * error_position[0] / wheel_drive_len_ ;
        twist_msg_0.header.stamp = this->get_clock()->now();

        //twist_msg_1.twist.linear.x = linear_velocity[1] ;            
        //twist_msg_1.twist.angular.z = angular_velocity * error_position[1] / wheel_drive_len_ ;
        //twist_msg_1.header.stamp = this->get_clock()->now();

        //twist_msg_2.twist.linear.x = linear_velocity[2];        
        //twist_msg_2.twist.angular.z = angular_velocity * error_position[2] / wheel_drive_len_ ;
        //twist_msg_2.header.stamp = this->get_clock()->now();
        
        twist_msg_3.twist.linear.x = linear_velocity[3] ;            
        twist_msg_3.twist.angular.z = angular_velocity * error_position[3] / wheel_drive_len_ ;
        twist_msg_3.header.stamp = this->get_clock()->now();
        
        cmd_vel_pub_0_->publish(twist_msg_0);
        //cmd_vel_pub_1_->publish(twist_msg_1);
        //cmd_vel_pub_2_->publish(twist_msg_2);
        cmd_vel_pub_3_->publish(twist_msg_3);

        //RCLCPP_INFO(this->get_logger(), "Linear Velocity[1]: %f", linear_velocity[1]);
    }
    std::string namespace_prefix;
    std::string amr_state_topic;
    std::string amr_cmds_topic;
    std::string prefix_; 
    double wheel_radius_;
    double wheel_drive_len_;
    std::string drive_1_pos_;
    std::string drive_2_pos_;
    std::string drive_3_pos_;
    std::string drive_4_pos_;
    double encoder_position[4];
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
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_0_;
    //rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_1_;
    //rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_2_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_3_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    if (argc < 8) {
        std::cerr << "Usage: jointstate_to_twist <prefix> <wheel_radius> <wheel_drive_len> <drive_1_pos> <drive_2_pos> <drive_3_pos> <drive_4_pos>" << std::endl;
        return 1;
    }
    auto node = std::make_shared<JointStateToTwist>(argv[1], std::stod(argv[2]), std::stod(argv[3]), argv[4], argv[5], argv[6], argv[7] );
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}