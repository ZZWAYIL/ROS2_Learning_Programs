import rclpy
from rclpy.node import Node
from face_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory # 获得包的share路径
from cv_bridge import CvBridge
import time

class FaceDetectNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info('Face Detect Service Node has been started.')
        # 创建服务
        self.service_ = self.create_service(FaceDetector,'face_detect',self.face_detect_callback)
        # 创建CvBridge对象,用于ROS图像消息和OpenCV图像之间的转换
        self.bridge = CvBridge()
        # 加载资源文件路径
        self.resource_path = get_package_share_directory('demo_python_service') + '/resource/default.jpg'
        self.declare_parameter('model','hog')  # 声明参数
        self.declare_parameter("number_of_times_to_upsample",1) # 声明参数
        self.model = self.get_parameter('model').get_parameter_value().string_value
        self.upsample_times = self.get_parameter('number_of_times_to_upsample').get_parameter_value().integer_value
    
    def face_detect_callback(self,request,response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            # 如果没有传入图像，则使用资源文件中的图像
            cv_image = cv2.imread(self.resource_path)
            self.get_logger().info('No image provided in request, using default image.')
        start_time = time.time()
        self.get_logger().info('Starting face detection...')
        # 检测人脸位置
        face_locations = face_recognition.face_locations(cv_image,number_of_times_to_upsample=self.upsample_times,model=self.model)
        response.count = len(face_locations)
        response.use_time = time.time() - start_time
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response
        

def main():
    rclpy.init()
    node = FaceDetectNode('face_detect_service_node')
    rclpy.spin(node)
    rclpy.shutdown()