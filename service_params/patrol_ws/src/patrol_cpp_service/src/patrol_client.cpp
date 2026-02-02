#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <chrono>
#include "user_interfaces/srv/face_detector.hpp"
#include "user_interfaces/srv/patrol.hpp"
#include <ctime>

using Patrol = user_interfaces::srv::Patrol;

class PatrolNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;


public:
    PatrolNode(const std::string &node_name) : Node(node_name){
            srand((unsigned)time(NULL));
            patrol_client_ = this->create_client<Patrol>("turtle_patrol");
            timer_ = this->create_wall_timer(std::chrono::seconds(5),
                std::bind(&PatrolNode::send_patrol_request, this));
        }
    
        void send_patrol_request(){
            while(!this->patrol_client_->wait_for_service(std::chrono::seconds(1))){
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Waiting for the patrol service to be available...");
            }

            auto request = std::make_shared<Patrol::Request>();
            request->target_x = rand()%99 / 100.0 * 13.0; 
            request->target_y = rand()%99 / 100.0 * 13.0;
            RCLCPP_INFO(this->get_logger(), "Requesting patrol to (%.2f, %.2f)", request->target_x, request->target_y);

            // 发送请求并等待响应
            patrol_client_->async_send_request(request,[&](rclcpp::Client<Patrol>::SharedFuture result_future)->void{
                auto result = result_future.get();
                if(result->result == Patrol::Response::SUCCESS)
                    RCLCPP_INFO(this->get_logger(), "Patrol succeeded.");
                else
                    RCLCPP_WARN(this->get_logger(), "Patrol failed.");
            });
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolNode>("turtle_patrol_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}