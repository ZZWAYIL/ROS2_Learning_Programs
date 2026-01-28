#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <memory>
#include <functional>
#include <chrono>



class TurtleTrackNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    std::optional<turtlesim::msg::Pose> current_pose_;
    std::optional<turtlesim::msg::Pose> target_pose_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    // rclcpp::TimerBase::SharedPtr timer_;

public:
    TurtleTrackNode(const std::string &node_name, const turtlesim::msg::Pose &target_pose)
        : Node(node_name)
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&TurtleTrackNode::Pose_Sub, this, std::placeholders::_1));
        target_pose_ = target_pose;

        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TurtleTrackNode::Cmd_Pub, this));
    }

    void Pose_Sub(const turtlesim::msg::Pose::SharedPtr pose)
    {
        current_pose_ = *pose;
    }

    void Cmd_Pub(){
        if (!current_pose_){
            RCLCPP_WARN(this->get_logger(), "Waiting for pose data...");
            return;
        }
        
        auto twist_msg = geometry_msgs::msg::Twist();
        auto dx = target_pose_->x - current_pose_->x;
        auto dy = target_pose_->y - current_pose_->y;
        auto distance = std::sqrt(dx * dx + dy * dy);
        auto angle_to_target = std::atan2(dy, dx);
        auto angle_diff = angle_to_target - current_pose_->theta;
        if(angle_diff > M_PI){
            angle_diff -= 2 * M_PI;
        } else if (angle_diff < -M_PI){
            angle_diff += 2 * M_PI;
        }

        // // 分开控制角度和距离
        // if(distance > 0.1){
        //     if(std::abs(angle_diff) > 0.1){
        //         twist_msg.linear.x = 0.0;
        //         twist_msg.angular.z = angle_diff;
        //     } 
        //     else {
        //         twist_msg.linear.x = distance;
        //         twist_msg.angular.z = 0.0;
        //     }
        // } else {
        //     twist_msg.linear.x = 0.0;
        //     twist_msg.angular.z = 0.0;
        // }

        // 同时控制角度和距离
        if (distance > 0.1){
            twist_msg.linear.x = 0.5 * distance;
            twist_msg.angular.z = angle_diff;
        } else {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
        }

        RCLCPP_INFO(this->get_logger(), "Current Pose: x=%.2f, y=%.2f, theta=%.2f", current_pose_->x, current_pose_->y, current_pose_->theta);
        pub_->publish(twist_msg);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    turtlesim::msg::Pose target_pose;
    target_pose.x = 2.0;
    target_pose.y = 2.0;
    auto node = std::make_shared<TurtleTrackNode>("turtle_track_node", target_pose);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}