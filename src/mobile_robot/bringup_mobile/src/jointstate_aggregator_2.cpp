// filepath: /home/george/ROS2/ros2_wheelbot/src/mobile_robot/bringup_mobile/src/jointstate_aggregator_2.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

// Wrap angle to [-π, π]
double wrap_angle(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle - M_PI;
}

// Function to convert the encoder output (add π offset and wrap)
double convert_encoder_output(double theta) {
    return wrap_angle(theta + M_PI);
}

class JointStateAggregator : public rclcpp::Node
{
public:
    JointStateAggregator()
        : Node("joint_state_aggregator"),
          fr_received_(false),
          rl_received_(false)
    {
        // Handle namespace
        std::string namespace_prefix = this->get_namespace();
        if (namespace_prefix == "/") {
            namespace_prefix = "";
        } else {
            namespace_prefix = namespace_prefix + "/";
        }

        std::string fr_topic = namespace_prefix + "FR_amr_joint_states";
        std::string rl_topic = namespace_prefix + "RL_amr_joint_states";
        amr_state_topic_ = namespace_prefix + "amr_joint_states";

        auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

        // Subscriptions for actual encoder data (FR and RL)

        front_right_joints_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            fr_topic, reliable_qos, std::bind(&JointStateAggregator::front_right_JointsCallback, this, std::placeholders::_1));

        rear_left_joints_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            rl_topic, reliable_qos, std::bind(&JointStateAggregator::rear_left_JointsCallback, this, std::placeholders::_1));

        amr_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(amr_state_topic_, reliable_qos);

        RCLCPP_INFO(this->get_logger(), "JointStateAggregator initialized");
        RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s, %s", fr_topic.c_str(), rl_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publishing to: %s", amr_state_topic_.c_str());
    }

private:
    void front_right_JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        front_right_joint_state_ = *msg;
        fr_received_ = true;
        publishJointState();
    }

    void rear_left_JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        rear_left_joint_state_ = *msg;
        for (auto& position : rear_left_joint_state_.position) {
            position = convert_encoder_output(position);
        }
        rl_received_ = true;
        publishJointState();
    }

    // Helper to get steering position from joint state
    // ESP32 publishes: [0] = steering_joint, [1] = wheel_joint
    double getSteeringPosition(const sensor_msgs::msg::JointState& state) {
        return state.position.size() > 0 ? state.position[0] : 0.0;
    }

    double getSteeringVelocity(const sensor_msgs::msg::JointState& state) {
        return state.velocity.size() > 0 ? state.velocity[0] : 0.0;
    }

    // Helper to get wheel position
    // ESP32 publishes only 2 joints, so wheel is at index 1
    double getWheelPosition(const sensor_msgs::msg::JointState& state) {
        return state.position.size() > 1 ? state.position[1] : 0.0;
    }

    double getWheelVelocity(const sensor_msgs::msg::JointState& state) {
        return state.velocity.size() > 1 ? state.velocity[1] : 0.0;
    }

    void publishJointState()
    {
        // Publish when at least one source is available.
        // Missing side will be filled with safe defaults (0.0) via getters.
        if (!fr_received_ && !rl_received_) {
            return;
        }

        if (!fr_received_ || !rl_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Publishing partial amr_joint_states (FR received=%s, RL received=%s)",
                fr_received_ ? "true" : "false",
                rl_received_ ? "true" : "false");
        }

        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();

        // Virtual joints names for swerve controller compatibility
        // Order: FR_steer, FR_wheel, FL_steer, FL_wheel, RR_steer, RR_wheel, RL_steer, RL_wheel
        joint_state_msg.name = {
            "virtual_front_right_steering_joint",
            "virtual_front_right_wheel_joint",
            "virtual_front_left_steering_joint",
            "virtual_front_left_wheel_joint",
            "virtual_rear_right_steering_joint",
            "virtual_rear_right_wheel_joint",
            "virtual_rear_left_steering_joint",
            "virtual_rear_left_wheel_joint"
        };

        // FR from FR_amr_joint_states
        // FL copied from RL_amr_joint_states
        // RR copied from FR_amr_joint_states
        // RL from RL_amr_joint_states

        // Positions:
        joint_state_msg.position = {
            getSteeringPosition(front_right_joint_state_),  // virtual_front_right_steering_joint (from FR)
            getWheelPosition(front_right_joint_state_),     // virtual_front_right_wheel_joint (from FR)
            getSteeringPosition(front_right_joint_state_),  // virtual_front_left_steering_joint (mirrored from FR)
            getWheelPosition(front_right_joint_state_),     // virtual_front_left_wheel_joint (mirrored from FR)
            getSteeringPosition(rear_left_joint_state_),    // virtual_rear_right_steering_joint (mirrored from RL)
            getWheelPosition(rear_left_joint_state_),       // virtual_rear_right_wheel_joint (mirrored from RL)
            getSteeringPosition(rear_left_joint_state_),    // virtual_rear_left_steering_joint (from RL)
            getWheelPosition(rear_left_joint_state_)        // virtual_rear_left_wheel_joint (from RL)
        };

        // Velocities: 8 elements matching name order
        joint_state_msg.velocity = {
            getSteeringVelocity(front_right_joint_state_),  // virtual_front_right_steering_joint vel
            getWheelVelocity(front_right_joint_state_),     // virtual_front_right_wheel_joint (from FR)
            getSteeringVelocity(front_right_joint_state_),  // virtual_front_left_steering_joint vel
            getWheelVelocity(front_right_joint_state_),     // virtual_front_left_wheel_joint (from FR)
            getSteeringVelocity(rear_left_joint_state_),    // virtual_rear_right_steering_joint vel
            getWheelVelocity(rear_left_joint_state_),       // virtual_rear_right_wheel_joint (from RL)
            getSteeringVelocity(rear_left_joint_state_),    // virtual_rear_left_steering_joint vel
            getWheelVelocity(rear_left_joint_state_)        // virtual_rear_left_wheel_joint (from RL)
        };

        // Efforts (set to 0.0)
        //joint_state_msg.effort = {};

        amr_joint_state_pub_->publish(joint_state_msg);
    }

    std::string amr_state_topic_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr front_right_joints_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr rear_left_joints_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr amr_joint_state_pub_;

    sensor_msgs::msg::JointState front_right_joint_state_;
    sensor_msgs::msg::JointState rear_left_joint_state_;
    bool fr_received_;
    bool rl_received_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateAggregator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
