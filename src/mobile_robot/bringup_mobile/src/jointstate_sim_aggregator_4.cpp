#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class JointStateConverterNode : public rclcpp::Node
{
public:
    JointStateConverterNode()
    : Node("joint_state_converter")

    {
        std::string namespace_prefix = this->get_namespace();
        if (namespace_prefix == "/" ) {
            namespace_prefix = "" ;}
        else    {
            namespace_prefix = namespace_prefix  + "/";
        }
        
        fl_drive_topic = namespace_prefix + "FL_drive_joint_states";
        fr_drive_topic = namespace_prefix + "FR_drive_joint_states";
        rl_drive_topic = namespace_prefix + "RL_drive_joint_states";
        rr_drive_topic = namespace_prefix + "RR_drive_joint_states";
        amr_state_topic = namespace_prefix + "amr_joint_states";
        isaac_state_topic = namespace_prefix + "isaac_amr_joint_states";

        // Subscriem la topicul "isaac_joint_states"
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        isaac_state_topic, 10, std::bind(&JointStateConverterNode::joint_state_callback, this, std::placeholders::_1));

        // Publicăm în topicul "amr_joint_states"
        publisher_amr_ = this->create_publisher<sensor_msgs::msg::JointState>(amr_state_topic, 10);
        
        // Publicăm în topicurile de drive
        publisher_FL_drive_ = this->create_publisher<sensor_msgs::msg::JointState>(fl_drive_topic, 10);
        publisher_FR_drive_ = this->create_publisher<sensor_msgs::msg::JointState>(fr_drive_topic, 10);
        publisher_RL_drive_ = this->create_publisher<sensor_msgs::msg::JointState>(rl_drive_topic, 10);
        publisher_RR_drive_ = this->create_publisher<sensor_msgs::msg::JointState>(rr_drive_topic, 10);

    }

private:
   void publish_drive_joint_state(const std::string& topic, double pos_left, double pos_right, double vel_left, double vel_right)
    
    {
        auto drive_msg = sensor_msgs::msg::JointState();

        if (topic == fl_drive_topic) 
            drive_msg.name = {"FL_drive_left_joint", "FL_drive_right_joint"};
        else if (topic == fr_drive_topic) 
            drive_msg.name = {"FR_drive_left_joint", "FR_drive_right_joint"};
        else if (topic == rl_drive_topic) 
            drive_msg.name = {"RL_drive_left_joint", "RL_drive_right_joint"};
        else if (topic == rr_drive_topic) 
            drive_msg.name = {"RR_drive_left_joint", "RR_drive_right_joint"};
        else 
            RCLCPP_INFO(this->get_logger(), "Error-drive topic: %s", topic.c_str());

        drive_msg.position = { pos_left, pos_right };
        drive_msg.velocity = { vel_left, vel_right };
        drive_msg.effort = { 0.0, 0.0 }; // efortul rămâne la 0.0

        if (topic == fl_drive_topic) 
            publisher_FL_drive_->publish(drive_msg);
        else if (topic == fr_drive_topic) 
            publisher_FR_drive_->publish(drive_msg);
        else if (topic == rl_drive_topic) 
            publisher_RL_drive_->publish(drive_msg);
        else if (topic == rr_drive_topic) 
            publisher_RR_drive_->publish(drive_msg);
        else 
            RCLCPP_INFO(this->get_logger(), "Error-drive topic: %s", topic.c_str());
    }
    
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto new_msg_amr = sensor_msgs::msg::JointState();
        
        new_msg_amr.name = {
            "virtual_front_right_steering_joint",
            "virtual_front_right_wheel_joint",
            "virtual_front_left_steering_joint",
            "virtual_front_left_wheel_joint",
            "virtual_rear_right_steering_joint",
            "virtual_rear_right_wheel_joint",
            "virtual_rear_left_steering_joint",
            "virtual_rear_left_wheel_joint",
        };

        new_msg_amr.position = {
            msg->position[1],  // FR_steering_joint (virtual_front_right_steering_joint)
            (msg->position[6] + msg->position[7]) / 2.0,  // FR_drive_left_joint + FR_drive_right_joint (virtual_front_right_wheel_joint)
            msg->position[0],  // FL_steering_joint (virtual_front_left_steering_joint)
            (msg->position[4] + msg->position[5]) / 2.0,  // FL_drive_left_joint + FL_drive_right_joint (virtual_front_left_wheel_joint)
            msg->position[3],  // RR_steering_joint (virtual_rear_right_steering_joint)
            (msg->position[10] + msg->position[11]) / 2.0,  // RR_drive_left_joint + RR_drive_right_joint (virtual_rear_right_wheel_joint)
            msg->position[2],  // RL_steering_joint (virtual_rear_left_steering_joint)
            (msg->position[8] + msg->position[9]) / 2.0   // RL_drive_left_joint + RL_drive_right_joint (virtual_rear_left_wheel_joint)
        };


        new_msg_amr.velocity = {
            msg->velocity[1],  // FR_steering_joint (virtual_front_right_steering_joint)
            (msg->velocity[6] + msg->velocity[7]) / 2.0,  // FR_drive_left_joint + FR_drive_right_joint (virtual_front_right_wheel_joint)
            msg->velocity[0],  // FL_steering_joint (virtual_front_left_steering_joint)
            (msg->velocity[4] + msg->velocity[5]) / 2.0,  // FL_drive_left_joint + FL_drive_right_joint (virtual_front_left_wheel_joint)
            msg->velocity[3],  // RR_steering_joint (virtual_rear_right_steering_joint)
            (msg->velocity[10] + msg->velocity[11]) / 2.0,  // RR_drive_left_joint + RR_drive_right_joint (virtual_rear_right_wheel_joint)
            msg->velocity[2],  // RL_steering_joint (virtual_rear_left_steering_joint)
            (msg->velocity[8] + msg->velocity[9]) / 2.0   // RL_drive_left_joint + RL_drive_right_joint (virtual_rear_left_wheel_joint)
        };

        new_msg_amr.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            
        publisher_amr_->publish(new_msg_amr);

        // Creăm mesajele pentru topicurile de drive
        publish_drive_joint_state(fl_drive_topic, msg->position[4], msg->position[5], msg->velocity[4], msg->velocity[5]);
        publish_drive_joint_state(fr_drive_topic, msg->position[6], msg->position[7], msg->velocity[6], msg->velocity[7]);
        publish_drive_joint_state(rl_drive_topic, msg->position[8], msg->position[9], msg->velocity[8], msg->velocity[9]);
        publish_drive_joint_state(rr_drive_topic, msg->position[10], msg->position[11], msg->velocity[10], msg->velocity[11]);
    }
    
 
    std::string namespace_prefix;
    std::string fl_drive_topic;
    std::string fr_drive_topic;
    std::string rl_drive_topic;
    std::string rr_drive_topic;
    std::string amr_state_topic;
    std::string isaac_state_topic;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_amr_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_FL_drive_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_FR_drive_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_RL_drive_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_RR_drive_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateConverterNode>());
    rclcpp::shutdown();
    return 0;
}
