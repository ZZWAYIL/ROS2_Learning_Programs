import espeakng
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue
import threading
import time


class NovelSubscriber(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info("Novel Subscriber has been started.")
        self.queue_ = Queue()
        self.novel_sub_ = self.create_subscription(String,"novel",self.novel_callback,20)
        self.speak_thread_ = threading.Thread(target=self.speak_novel)
        self.speak_thread_.start()

    # msg来自发布的小说内容，ros2自动调用publisher发布的内容
    def novel_callback(self,msg):
        self.queue_.put(msg.data)

    def speak_novel(self):
        speaker = espeakng.Speaker()
        speaker.voice = 'zh'
        while rclpy.ok():
            if not self.queue_.empty():
                line = self.queue_.get()
                self.get_logger().info(f"Speaking line: {line}")
                speaker.say(line)  
                speaker.wait()  # 等待当前朗读完成再进行下一句朗读 
            else:
                time.sleep(1)  # 避免空转过快占用CPU


def main():
    rclpy.init()
    novel_sub = NovelSubscriber("novel_sub_node")
    rclpy.spin(novel_sub)
    rclpy.shutdown()