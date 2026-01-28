import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue

class NovelPublisher(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info("Novel Publisher has been started.")
        # 创建队列存储小说内容，记得放前面，防止后面调用（time_callback）时报错
        self.queue_ = Queue()  
        # 设置发布者，按时间发布小说内容
        self.publisher_ = self.create_publisher(String,"novel",20)
        self.create_timer(3,self.time_callback)
    
    # 发布小说内容
    def time_callback(self):
        # self.publisher_.publish()
        if not self.queue_.empty():
            line = self.queue_.get()
            msg = String()
            msg.data = line
            self.publisher_.publish(msg)   
            self.get_logger().info(f"Published line: {line}") 
    
    def download_novel(self, url):
        response = requests.get(url)
        response.encoding = 'utf-8'
        if response.status_code == 200:
            novel_text = response.text
            self.get_logger().info("Novel downloaded successfully.")
            for line in novel_text.splitlines():
                self.queue_.put(line)
        else:
            self.get_logger().error(f"Failed to download novel. Status code: {response.status_code}")
            return None


def main():
    rclpy.init()
    novel_pub = NovelPublisher("novel_pub_node")
    novel_pub.download_novel("http://0.0.0.0:8000/novels/novel1.txt")
    rclpy.spin(novel_pub)
    rclpy.shutdown()