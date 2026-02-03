import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster # 导入静态变换广播器
from geometry_msgs.msg import TransformStamped # 导入TransformStamped消息类型
from tf_transformations import quaternion_from_euler # 欧拉角转四元数
import math

class StaticTFBroadcasterNode(Node):
    def __init__(self):
        super().__init__("static_tf_broadcaster")
        self.tfbrosdcaster = StaticTransformBroadcaster(self)
        # 发布静态TF点
        self.publish_static_tf()

    
    def publish_static_tf(self):
        """
        手眼转换
        发布静态TF点
        从 base_link 到 camera_link
        """
        tf = TransformStamped()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "camera_link"
        tf.header.stamp = self.get_clock().now().to_msg()

        tf.transform.translation.x = 0.5
        tf.transform.translation.y = 0.3
        tf.transform.translation.z = 0.6

        q = quaternion_from_euler(math.radians(180),0,0)
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]

        self.tfbrosdcaster.sendTransform(tf)
        self.get_logger().info("发布静态TF点: 从 base_link 到 camera_link，{}".format(tf))

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFBroadcasterNode()
    rclpy.spin(node) 
    rclpy.shutdown()