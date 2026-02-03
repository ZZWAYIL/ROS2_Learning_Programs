#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"  // 用于四元数计算
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // 用于转换消息类型
#include "tf2_ros/static_transform_broadcaster.hpp"

class StaticTFBroadcaster : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
public:
    StaticTFBroadcaster():Node("static_tf_broadcaster"){
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->publish_tf();
    }

    void publish_tf(){
        // 定义一个静态变换
        geometry_msgs::msg::TransformStamped static_transform_stamped;

        static_transform_stamped.header.stamp = this->get_clock()->now();
        static_transform_stamped.header.frame_id = "map";
        static_transform_stamped.child_frame_id = "target_point";

        // 设置平移部分
        static_transform_stamped.transform.translation.x = 5.0;
        static_transform_stamped.transform.translation.y = 3.0;
        static_transform_stamped.transform.translation.z = 0.0;

        // 设置旋转部分（使用四元数表示）
        tf2::Quaternion quat;
        quat.setRPY(0, 0, 60*M_PI/180.0); // Roll, Pitch, Yaw
        static_transform_stamped.transform.rotation.x = quat.x();
        static_transform_stamped.transform.rotation.y = quat.y();
        static_transform_stamped.transform.rotation.z = quat.z();
        static_transform_stamped.transform.rotation.w = quat.w();

        // 发布静态变换
        static_broadcaster_->sendTransform(static_transform_stamped);

        RCLCPP_INFO(this->get_logger(), "Static Transform from 'map' to 'target_point' published.");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



