import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener,Buffer # 导入变换监听
from geometry_msgs.msg import TransformStamped # 导入TransformStamped消息类型
from tf_transformations import euler_from_quaternion # 欧拉角转四元数
import math

class TFListenerNode(Node):
    def __init__(self):
        super().__init__("tf_listener")
        self.buffer_ = Buffer()
        self.tf_listener = TransformListener(self.buffer_,self)
        # 发布动态TF点
        self.timer_ = self.create_timer(1,self.listener_tf)
    
    def listener_tf(self):
        """
        定时监听TF点
        
        """
        try:
            result = self.buffer_.lookup_transform("base_link","bottle_link",
                                                   rclpy.time.Time(seconds=0),timeout=rclpy.duration.Duration(seconds=1.0))
            tf = result.transform
            self.get_logger().info(f'平移量: x={tf.translation.x}, y={tf.translation.y}, z={tf.translation.z}')
            q = euler_from_quaternion([
                tf.rotation.x,
                tf.rotation.y,
                tf.rotation.z,
                tf.rotation.w
            ])
            self.get_logger().info(f'欧拉角: roll={math.degrees(q[0])}, pitch={math.degrees(q[1])}, yaw={math.degrees(q[2])}') 
            
        except Exception as e:
            self.get_logger().error("监听TF点失败: {}".format(e))
            return


def main(args=None):
    rclpy.init(args=args)
    node = TFListenerNode()
    rclpy.spin(node) 
    rclpy.shutdown()