#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"  // 用于四元数计算
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // 用于转换消息类型
#include "tf2_ros/transform_listener.hpp"
#include <chrono>
#include "tf2/utils.hpp"
#include "tf2_ros/buffer.hpp"

class TFListener : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr time_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
public:
    TFListener():Node("tf_listener"){
        this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*(this->tf_buffer_),this);
        this->time_ = this->create_wall_timer(std::chrono::seconds(2),std::bind(&TFListener::get_tf,this));
    }

    void get_tf(){
        try
        {
            const auto tf = this->tf_buffer_->lookupTransform("base_link","target_point",this->get_clock()->now(),
            rclcpp::Duration::from_seconds(2.0f));
            auto translation = tf.transform.translation;
            auto rotation = tf.transform.rotation;
            double y,p,r;
            tf2::getEulerYPR(rotation,y,p,r);
            RCLCPP_INFO(this->get_logger(),"Transform from 'base_link' to 'target_point':");
            RCLCPP_INFO(this->get_logger(),"Translation: x=%.2f, y=%.2f, z=%.2f",translation.x,translation.y,translation.z);
            RCLCPP_INFO(this->get_logger(),"Rotation (RPY): roll=%.2f, pitch=%.2f, yaw=%.2f",r,p,y);
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(),"Could not get transform from 'base_link' to 'target_point'");
            RCLCPP_WARN(this->get_logger(),"reason : %s",e.what());
        }
        
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



