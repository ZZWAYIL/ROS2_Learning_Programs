import rclpy
from rclpy.node import Node
from face_interfaces.srv import FaceDetector
import cv2
from ament_index_python.packages import get_package_share_directory # 获得包的share路径
from cv_bridge import CvBridge
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType


class FaceDetectClientNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info('Face Detect Client Node has been started.')
        # 创建服务客户端
        self.client_ = self.create_client(FaceDetector,'face_detect')
        # 创建CvBridge对象,用于ROS图像消息和OpenCV图像之间的转换
        self.bridge = CvBridge()
        # 加载请求图像资源文件路径
        self.resource_path = get_package_share_directory('demo_python_service') + '/resource/persons.jpg'
        # 读取图像
        self.image = cv2.imread(self.resource_path)

        self.declare_parameter('model','hog')  # 声明参数
        self.declare_parameter("number_of_times_to_upsample",1) # 声明参数
        default_model = self.get_parameter('model').get_parameter_value().string_value
        default_upsample = self.get_parameter('number_of_times_to_upsample').get_parameter_value().integer_value
        
    # 创建服务客户端请求函数
    def call_set_parameters(self,parameters):
        """
        调用服务客户端请求函数
        """
        # 创建服务客户端
        updata_params_client = self.create_client(SetParameters,'/face_detect_service_node/set_parameters')
        while updata_params_client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('SetParameters service not available, waiting again...')
        # 创建请求
        request = SetParameters.Request()
        request.parameters = parameters
        future = updata_params_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response

    def updata_parameters(self,model = 'hog'):
        params = Parameter()
        params.name = 'model'
        parameter_value = ParameterValue()
        parameter_value.type = ParameterType.PARAMETER_STRING
        parameter_value.string_value = model
        params.value = parameter_value
        response = self.call_set_parameters([params])
        for result in response.results:
            self.get_logger().info('Update parameter %d,reason: %s' % (result.successful, result.reason))



    def send_request(self):
        # 等待服务可用
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        # 创建服务请求
        request = FaceDetector.Request()
        # 将OpenCV图像转换为ROS图像消息
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        # 发送服务请求
        # 异步：先创建一个future对象，但是现在的并没有结果，需要等到服务结果返回后，才会在future对象中存储结果
        future = self.client_.call_async(request)

        # future.add_done_callback(self.show_response)  # 注册回调函数，当future对象中有结果返回时，会调用该函数处理结果

        # while future.done() is False:
        #     rclpy.spin_once(self)  # 会阻塞，直到有新的数据到来才会返回，这样对单线程不友好，有可能阻塞其他回调函数的执行
        rclpy.spin_until_future_complete(self, future)  # 会一直运行当前节点，直到future对象中有结果返回为止，更适合单线程
        response = future.result()
        # self.show_response(response)
        self.get_logger().info('received response, face count: %d, use time: %.4f seconds' % (response.count, response.use_time))

    def show_response(self,response):
        for i in range(response.count):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            # 在图像上绘制人脸位置矩形框
            cv2.rectangle(self.image, (left, top), (right, bottom), (0, 0, 255), 4)
        # 显示图像
        cv2.imshow('Face Detection Result', self.image)
        cv2.waitKey(0)  # 等待按键关闭窗口,单线程阻塞，可能会影响其他回调函数的执行，但由于这里只运行一次相应，不会有影响

def main():
    rclpy.init()
    node = FaceDetectClientNode('face_detect_client_node')
    node.updata_parameters(model='cnn')
    node.send_request()
    node.updata_parameters(model='hog')
    node.send_request()

    rclpy.spin(node)
    rclpy.shutdown()