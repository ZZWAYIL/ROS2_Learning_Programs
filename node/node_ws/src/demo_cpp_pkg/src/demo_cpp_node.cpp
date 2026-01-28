#include "iostream"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ROS2_Cpp_Node");
    
    RCLCPP_INFO(node->get_logger(), "Hello, ROS2 from C++17!");
    rclcpp::spin(node);
    rclcpp::shutdown();


    return 0;
}