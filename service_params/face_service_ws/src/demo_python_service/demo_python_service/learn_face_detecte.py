import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory # 获得包的share路径

def main():
    '''
    获取资源文件路径
    if you install the package 'foo' into '/home/user/ros2_ws/install' 
    and you called this function with 'foo' as the argument, 
    then it will return '/home/user/ros2_ws/install/share/foo
    '''
    resource_path = get_package_share_directory('demo_python_service') + '/resource/default.jpg'

    print("资源文件路径：", resource_path)
    # 加载图片
    # image = face_recognition.load_image_file(resource_path)
    image = cv2.imread(resource_path)
    # 检测人脸位置
    face_locations = face_recognition.face_locations(image,number_of_times_to_upsample=4,model="hog")

    # 绘制人脸框 (CV2)BGR
    for top, right, bottom, left in face_locations:
        cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 4)

    # 显示图片
    cv2.imshow("Face Detection", image)
    cv2.waitKey(0)