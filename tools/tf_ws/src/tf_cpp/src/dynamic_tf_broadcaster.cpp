#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"  // 用于四元数计算
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // 用于转换消息类型
#include "tf2_ros/transform_broadcaster.hpp"
#include <chrono>

class TFBroadcaster : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
    rclcpp::TimerBase::SharedPtr time_;
public:
    TFBroadcaster():Node("tf_broadcaster"){
        dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        this->time_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&TFBroadcaster::publish_tf,this));
    }

    void publish_tf(){
        // 定义一个静态变换
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_link";

        // 设置平移部分
        transform_stamped.transform.translation.x = 2.0;
        transform_stamped.transform.translation.y = 3.0;
        transform_stamped.transform.translation.z = 0.0;

        // 设置旋转部分（使用四元数表示）
        tf2::Quaternion quat;
        quat.setRPY(0, 0, 30*M_PI/180.0); // Roll, Pitch, Yaw
        transform_stamped.transform.rotation.x = quat.x();
        transform_stamped.transform.rotation.y = quat.y();
        transform_stamped.transform.rotation.z = quat.z();
        transform_stamped.transform.rotation.w = quat.w();

        // 发布变换
        this->dynamic_broadcaster_->sendTransform(transform_stamped);

        RCLCPP_INFO(this->get_logger(), "Transform from 'map' to 'base_link' published.");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



