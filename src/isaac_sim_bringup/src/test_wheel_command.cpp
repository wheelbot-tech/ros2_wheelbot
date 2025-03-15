#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class WheelCommandPublisher : public rclcpp::Node
{
public:
    WheelCommandPublisher()
        : Node("wheel_command_publisher"), left_velocity_(2.5), right_velocity_(2.5)
    {
        // Definirea articulațiilor și topicurilor specifice
        joint_groups_["/FL_drive_joint_commands"] = {"FL_drive_left_joint", "FL_drive_right_joint"};
        joint_groups_["/FR_drive_joint_commands"] = {"FR_drive_left_joint", "FR_drive_right_joint"};
        joint_groups_["/RL_drive_joint_commands"] = {"RL_drive_left_joint", "RL_drive_right_joint"};
        joint_groups_["/RR_drive_joint_commands"] = {"RR_drive_left_joint", "RR_drive_right_joint"};

        // Crearea publisherilor pentru fiecare topic
        for (const auto &group : joint_groups_)
        {
            publishers_[group.first] = this->create_publisher<sensor_msgs::msg::JointState>(group.first, 10);
        }

        // Timer pentru publicarea periodică
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WheelCommandPublisher::publish_commands, this));
    }

private:
void publish_commands()
{
    for (const auto &group : joint_groups_)
    {
        const std::string &topic = group.first;
        const std::vector<std::string> &joints = group.second;

        // Crearea mesajului JointState pentru articulațiile specifice
        auto message = sensor_msgs::msg::JointState();
        message.name = joints;
        message.velocity.resize(joints.size());

        // Parcurgem fiecare articulație și setăm viteza corespunzătoare
        for (size_t i = 0; i < joints.size(); ++i)
        {
            if (joints[i].find("_drive_left_joint") != std::string::npos)
            {
                message.velocity[i] = left_velocity_;
            }
            else if (joints[i].find("_drive_right_joint") != std::string::npos)
            {
                message.velocity[i] = right_velocity_;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Joint %s does not match left or right velocity pattern.", joints[i].c_str());
                message.velocity[i] = 0.0; // Setare implicită pentru articulații neașteptate
            }
        }

        // Publicarea mesajului pe topicul specific
        publishers_[topic]->publish(message);
    }
}

    double left_velocity_;
    double right_velocity_;
    std::map<std::string, std::vector<std::string>> joint_groups_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> publishers_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelCommandPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
