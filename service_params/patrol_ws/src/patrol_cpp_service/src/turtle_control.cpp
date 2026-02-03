#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <memory>
#include <functional>
#include <chrono>
#include "user_interfaces/srv/face_detector.hpp"
#include "user_interfaces/srv/patrol.hpp"

using Patrol = user_interfaces::srv::Patrol;

class TurtleTrackNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    std::optional<turtlesim::msg::Pose> current_pose_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    // 必须是成员变量，生命周期与节点一致
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr handler_;
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<Patrol>::SharedPtr patrol_service_;
    double target_x_ = 1.0;
    double target_y_ = 2.0;

public:
    TurtleTrackNode(const std::string &node_name)
        : Node(node_name)
    {
        this->declare_parameter("target_x_", 1.0);
        this->declare_parameter("target_y_", 2.0);
        this->get_parameter("target_x_", target_x_);
        this->get_parameter("target_y_", target_y_);

        // 让外界的参数修改生效
        this->handler_ = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters)-> auto
        {
            auto result = rcl_interfaces::msg::SetParametersResult();
            result.successful = true;
            for (const auto &param : parameters)
            {
                if (param.get_name() == "target_x_")
                {
                    target_x_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated target_x_ to %.2f", target_x_);
                }
                else if (param.get_name() == "target_y_")
                {
                    target_y_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated target_y_ to %.2f", target_y_);
                }
            }
            return result;
        });

        patrol_service_ = this->create_service<Patrol>(
            "/turtle_patrol",
            [this](const std::shared_ptr<Patrol::Request> request, std::shared_ptr<Patrol::Response> response) -> void
            {
                // 边界判断
                if (request->target_x < 0.0 || request->target_x > 11.0 ||
                    request->target_y < 0.0 || request->target_y > 11.0)
                {
                    RCLCPP_WARN(this->get_logger(), "Target position out of bounds (0.0~11.0).");
                    response->result = Patrol::Response::FAIL;
                    return;
                }
                else
                {
                    this->target_x_ = request->target_x;
                    this->target_y_ = request->target_y;
                    response->result = Patrol::Response::SUCCESS;
                }
            });
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&TurtleTrackNode::Pose_Sub, this, std::placeholders::_1));
        
        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&TurtleTrackNode::Cmd_Pub, this));
    }

    void Pose_Sub(const turtlesim::msg::Pose::SharedPtr pose)
    {
        current_pose_ = *pose;
    }

    void Cmd_Pub()
    {
        if (!current_pose_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for pose data...");
            return;
        }

        auto twist_msg = geometry_msgs::msg::Twist();
        auto dx = target_x_ - current_pose_->x;
        auto dy = target_y_ - current_pose_->y;
        auto distance = std::sqrt(dx * dx + dy * dy);
        auto angle_to_target = std::atan2(dy, dx);
        auto angle_diff = angle_to_target - current_pose_->theta;
        if (angle_diff > M_PI)
        {
            angle_diff -= 2 * M_PI;
        }
        else if (angle_diff < -M_PI)
        {
            angle_diff += 2 * M_PI;
        }

        // 分开控制角度和距离
        if(distance > 0.1){
            if(std::abs(angle_diff) > 0.1){
                twist_msg.linear.x = 0.2*distance;
                twist_msg.angular.z = 1.8 * angle_diff;
            }
            else {
                twist_msg.linear.x = 0.9*distance;
                twist_msg.angular.z = angle_diff;
            }
        } else {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
        }

        // // 同时控制角度和距离
        // if (distance > 0.1)
        // {
        //     twist_msg.linear.x = 0.8 * distance;
        //     twist_msg.angular.z = angle_diff;
        // }
        // else
        // {
        //     twist_msg.linear.x = 0.0;
        //     twist_msg.angular.z = 0.0;
        // }

        RCLCPP_INFO(this->get_logger(), "Current Pose: x=%.2f, y=%.2f, theta=%.2f", current_pose_->x, current_pose_->y, current_pose_->theta);
        pub_->publish(twist_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // turtlesim::msg::Pose target_pose;
    // target_pose.x = 2.0;
    // target_pose.y = 2.0;
    auto node = std::make_shared<TurtleTrackNode>("turtle_track_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}