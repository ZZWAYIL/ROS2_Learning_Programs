#include "rclcpp/rclcpp.hpp"

class PersonNode : public rclcpp::Node{
private:
    std::string name;
    int age;

public:
    PersonNode(const std::string &node_name,const std::string &name,const int &age): rclcpp::Node(node_name){
        this->name = name;
        this->age = age;
    };

    void eat(const std::string &food){
        RCLCPP_INFO(this->get_logger(), "%s is eating %s.", this->name.c_str(), food.c_str());
    };
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PersonNode>("ROS2_Cpp_Node", "ZZW", 20);
    
    node->eat("apple");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}