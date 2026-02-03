#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <chrono>
#include "user_interfaces/srv/face_detector.hpp"
#include "user_interfaces/srv/patrol.hpp"
#include <ctime>


// 添加消息接口
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

using SetP = rcl_interfaces::srv::SetParameters;
using Patrol = user_interfaces::srv::Patrol;



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
            patrol_client_ = this->create_client<Patrol>("/turtle_patrol");
            timer_ = this->create_wall_timer(std::chrono::seconds(8),
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
            request->target_x = rand()%99 / 100.0 * 11.0; 
            request->target_y = rand()%99 / 100.0 * 11.0;
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

        /* 
        
        创建客户端发送请求，返回结果
        
        */
        SetP::Response::SharedPtr call_set_paramters(const std::vector<rcl_interfaces::msg::Parameter> &params){
            auto client_ = this->create_client<SetP>("/turtle_track_node/set_parameters");

            // 等待服务可用
            while(!client_->wait_for_service(std::chrono::seconds(1))){
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return nullptr;
                }
                RCLCPP_INFO(this->get_logger(), "Waiting for the patrol service to be available...");
            }
            // 创建请求
            auto request = std::make_shared<SetP::Request>();
            request->parameters = params;

            // 发送请求并等待响应
            auto future = client_->async_send_request(request);
            rclcpp::spin_until_future_complete(this->get_node_base_interface(),future);
            auto response = future.get();
            return response;
        }

        /* 更新参数 */
        void update_param(double target_x_,double target_y_){
            // 统一打包
            std::vector<rcl_interfaces::msg::Parameter> params_to_send;

            auto param_x_ = rcl_interfaces::msg::Parameter();
            auto param_y_ = rcl_interfaces::msg::Parameter();
            param_x_.name = "target_x_";
            param_x_.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
            param_x_.value.double_value = target_x_;
            params_to_send.push_back(param_x_);

            param_y_.name = "target_y_";
            param_y_.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
            param_y_.value.double_value = target_y_;
            params_to_send.push_back(param_y_);

            
            // 调用服务更新参数
            auto response_ = call_set_paramters(params_to_send);

            if(response_){
                for (size_t i = 0; i < response_->results.size(); ++i) {
                    if (response_->results[i].successful) {
                        RCLCPP_INFO(this->get_logger(), "Param [%s] set success", params_to_send[i].name.c_str());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Param [%s] failed: %s", 
                                    params_to_send[i].name.c_str(), response_->results[i].reason.c_str());
                    }
                }
            }
            
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolNode>("turtle_patrol_node");
    node->update_param(4.0,2.0);
    rclcpp::sleep_for(std::chrono::seconds(8));

    node->update_param(1.0,4.0);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}