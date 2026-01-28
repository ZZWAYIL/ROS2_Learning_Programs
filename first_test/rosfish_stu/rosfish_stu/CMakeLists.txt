#  编译ROS2 C++节点的CMakeLists.txt文件
cmake_minimum_required(VERSION 3.8)
project("ROS2_Cpp")
add_executable(ros2_node_cpp ros2_node_cpp.cpp)

#  进行库的查找和链接
#  查找并链接rclcpp库
find_package(rclcpp REQUIRED)        # 查找rclcpp包
message(STATUS ${rclcpp_INCLUDE_DIRS})    # 输出rclcpp的头文件路径
# message(STATUS ${rclcpp_LIBRARIES})       # 输出rclcpp的库文件路径

#   添加头文件路径并链接rclcpp库
target_include_directories(ros2_node_cpp PRIVATE ${rclcpp_INCLUDE_DIRS})   # 添加头文件路径
target_link_libraries(ros2_node_cpp ${rclcpp_LIBRARIES})               # 链接rclcpp库