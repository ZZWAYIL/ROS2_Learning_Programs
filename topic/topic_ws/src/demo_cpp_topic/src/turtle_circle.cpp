#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <memory>
#include <functional>
#include <chrono>

class TurtleCircleNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::optional<turtlesim::msg::Pose> current_pose_;

public:
    TurtleCircleNode(const std::string &node_name)
        : Node(node_name)
    {
        // Publisher to send velocity commands to the turtle
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        // Subscriber to receive the turtle's pose
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&TurtleCircleNode::pose_callback, this, std::placeholders::_1));

        // Timer to publish velocity commands at a fixed rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TurtleCircleNode::publish_velocity, this));
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
    }

    void publish_velocity()
    {
        if (!current_pose_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for pose data...");
            return;
        }

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 1.0;  // Constant forward speed
        twist_msg.angular.z = 1.0; // Constant angular speed for circular motion

        velocity_publisher_->publish(twist_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleCircleNode>("turtle_circle_node");
    node->publish_velocity();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}