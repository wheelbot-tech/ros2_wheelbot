#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

double wrap_angle(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle - M_PI;
}

// Function to convert the encoder output
double convert_encoder_output(double theta) {
    return wrap_angle(theta + M_PI);
}



class JointStateAggregator : public rclcpp::Node
{
public:
    JointStateAggregator()
        : Node("joint_state_aggregator")
    {
        front_right_joints_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "FR_amr_joint_states", 10, std::bind(&JointStateAggregator::front_right_JointsCallback, this, std::placeholders::_1));

        front_left_joints_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "FL_amr_joint_states", 10, std::bind(&JointStateAggregator::front_left_JointsCallback, this, std::placeholders::_1));

        rear_right_joints_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "RR_amr_joint_states", 10, std::bind(&JointStateAggregator::rear_right_JointsCallback, this, std::placeholders::_1));

        rear_left_joints_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "RL_amr_joint_states", 10, std::bind(&JointStateAggregator::rear_left_JointsCallback, this, std::placeholders::_1));

        amr_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("amr_joint_states", 10);
    }

private:
    void front_right_JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        front_right_joint_state_ = *msg;
        publishJointState();
    }

    void front_left_JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        front_left_joint_state_ = *msg;
        publishJointState();
    }
    void rear_right_JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        rear_right_joint_state_ = *msg;
        publishJointState();
    }

    void rear_left_JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        rear_left_joint_state_ = *msg;
        for (auto& position : rear_left_joint_state_.position) {
        position = convert_encoder_output(position);
        }
        publishJointState();
    }
    void publishJointState()
    {
        auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
        joint_state_msg->header.stamp = this->now();

        joint_state_msg->name.insert(joint_state_msg->name.end(), front_right_joint_state_.name.begin(), front_right_joint_state_.name.end());
        joint_state_msg->name.insert(joint_state_msg->name.end(), front_left_joint_state_.name.begin(), front_left_joint_state_.name.end());
        joint_state_msg->name.insert(joint_state_msg->name.end(), rear_right_joint_state_.name.begin(), rear_right_joint_state_.name.end());
        joint_state_msg->name.insert(joint_state_msg->name.end(), rear_left_joint_state_.name.begin(), rear_left_joint_state_.name.end());
        
        joint_state_msg->position.insert(joint_state_msg->position.end(), front_right_joint_state_.position.begin(), front_right_joint_state_.position.end());
        joint_state_msg->position.insert(joint_state_msg->position.end(), front_left_joint_state_.position.begin(), front_left_joint_state_.position.end());
        joint_state_msg->position.insert(joint_state_msg->position.end(), rear_right_joint_state_.position.begin(), rear_right_joint_state_.position.end());
        joint_state_msg->position.insert(joint_state_msg->position.end(), rear_left_joint_state_.position.begin(), rear_left_joint_state_.position.end());

        joint_state_msg->velocity.insert(joint_state_msg->velocity.end(), front_right_joint_state_.velocity.begin(), front_right_joint_state_.velocity.end());
        joint_state_msg->velocity.insert(joint_state_msg->velocity.end(), front_left_joint_state_.velocity.begin(), front_left_joint_state_.velocity.end());
        joint_state_msg->velocity.insert(joint_state_msg->velocity.end(), rear_right_joint_state_.velocity.begin(), rear_right_joint_state_.velocity.end());
        joint_state_msg->velocity.insert(joint_state_msg->velocity.end(), rear_left_joint_state_.velocity.begin(), rear_left_joint_state_.velocity.end());

        joint_state_msg->effort.insert(joint_state_msg->effort.end(), front_right_joint_state_.effort.begin(), front_right_joint_state_.effort.end());
        joint_state_msg->effort.insert(joint_state_msg->effort.end(), front_left_joint_state_.effort.begin(), front_left_joint_state_.effort.end());
        joint_state_msg->effort.insert(joint_state_msg->effort.end(), rear_right_joint_state_.effort.begin(), rear_right_joint_state_.effort.end());
        joint_state_msg->effort.insert(joint_state_msg->effort.end(), rear_left_joint_state_.effort.begin(), rear_left_joint_state_.effort.end());

        amr_joint_state_pub_->publish(*joint_state_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr front_right_joints_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr front_left_joints_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr rear_right_joints_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr rear_left_joints_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr amr_joint_state_pub_;

    sensor_msgs::msg::JointState front_right_joint_state_;
    sensor_msgs::msg::JointState front_left_joint_state_;
    sensor_msgs::msg::JointState rear_right_joint_state_;
    sensor_msgs::msg::JointState rear_left_joint_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateAggregator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
