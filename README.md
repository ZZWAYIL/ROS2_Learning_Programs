# ROS 学习与实验工作区

本仓库用于整理和实践 ROS2 相关示例、实验与工具代码，包含多个独立工作区与学习模块。

## 目录说明

- first_test/: CMake/GCC/ROS2 入门实验与编译示例
- learning_ros2/: ROS2 学习工作区（包含 action/interface/service/tf/topic 等示例，默认被 .gitignore 忽略）
- node/: 节点相关实验工作区
- service_params/: 服务参数相关实验工作区
- tools/: 工具与 TF 相关实验工作区
- topic/: 话题/接口相关实验工作区
- turtle_params.yaml: 示例参数文件（默认被 .gitignore 忽略）


## 子工作区构建与运行

所有 ROS2 工作区在构建/运行前请先准备环境：

```bash
source /opt/ros/<distro>/setup.bash
```

### first_test/（CMake/GCC/ROS2 入门）

#### 1) first_test/编译（CMake + C++）

```bash
cd /home/zzw/ROS/ros_programs/first_test/编译
cmake -S . -B build
cmake --build build
./build/g++
```

#### 2) first_test/编译（Python）

```bash
cd /home/zzw/ROS/ros_programs/first_test/编译
python3 test.py
```

#### 3) first_test/rosfish_stu/rosfish_stu（ROS2 C++/Python 节点）

```bash
cd /home/zzw/ROS/ros_programs/first_test/rosfish_stu/rosfish_stu
mkdir -p build && cd build
cmake ..
make
./ros2_node_cpp
```

```bash
cd /home/zzw/ROS/ros_programs/first_test/rosfish_stu/rosfish_stu
python3 ros2_node_python.py
```

#### 4) first_test/rosfish_stu/rosfish_stu（参数示例）

```bash
cd /home/zzw/ROS/ros_programs/first_test/rosfish_stu/rosfish_stu
g++ cpp_args.cpp -o cpp_args.exe
./cpp_args.exe --help
```

### learning_ros2/（ROS2 学习工作区）

构建：

```bash
cd /home/zzw/ROS/ros_programs/learning_ros2
colcon build --symlink-install
source install/setup.bash
```

运行：

- test
	- `ros2 run test test_node`
- learning_action
	- `ros2 run learning_action action_move_server`
	- `ros2 run learning_action action_move_client`
- learning_service
	- `ros2 run learning_service adderServer`
	- `ros2 run learning_service addClient`
	- `ros2 run learning_service objectServer`
	- `ros2 run learning_service objectClient`
- learning_topic
	- `ros2 run learning_topic topic_helloworld_pub`
	- `ros2 run learning_topic topic_helloworld_sub`
	- `ros2 run learning_topic topic_webcam_pub`
	- `ros2 run learning_topic topic_webcam_sub`
	- `ros2 run learning_topic topic_webcam_position_pub`
	- `ros2 run learning_topic topic_webcam_position_sub`
- learning_tf
	- `ros2 run learning_tf static_tf_broadcaster`
	- `ros2 run learning_tf turtle_tf_broadcaster`
	- `ros2 run learning_tf tf_listener`
	- `ros2 run learning_tf turtle_following`
- opencv_node
	- `ros2 run opencv_node node_object_webcam`

### node/node_ws/（节点相关工作区）

构建：

```bash
cd /home/zzw/ROS/ros_programs/node/node_ws
colcon build --symlink-install
source install/setup.bash
```

运行：

- demo_cpp_pkg
	- `ros2 run demo_cpp_pkg cpp_node`
	- `ros2 run demo_cpp_pkg person_node`
	- `ros2 run demo_cpp_pkg learn_auto`
	- `ros2 run demo_cpp_pkg learn_lambda`
	- `ros2 run demo_cpp_pkg learn_functional`
	- `ros2 run demo_cpp_pkg learn_lthread`
- demo_python_pkg
	- `ros2 run demo_python_pkg python_node`
	- `ros2 run demo_python_pkg person_node`
	- `ros2 run demo_python_pkg son_node`
	- `ros2 run demo_python_pkg node_son`
	- `ros2 run demo_python_pkg thread_node`

### topic/topic_ws/（话题示例工作区）

构建：

```bash
cd /home/zzw/ROS/ros_programs/topic/topic_ws
colcon build --symlink-install
source install/setup.bash
```

运行：

- demo_cpp_topic
	- `ros2 run demo_cpp_topic turtle_circle`
	- `ros2 run demo_cpp_topic turtle_track`
- demo_python_topic
	- `ros2 run demo_python_topic novel_pub`
	- `ros2 run demo_python_topic novel_sub`

### topic/interface_ws/（接口与可视化工作区）

构建（需要 Qt5 开发库）：

```bash
cd /home/zzw/ROS/ros_programs/topic/interface_ws
colcon build --symlink-install
source install/setup.bash
```

运行：

- status_display
	- `ros2 run status_display test_qt`
	- `ros2 run status_display sys_status_qt`
- status_publisher
	- `ros2 run status_publisher sys_status_publisher`

### service_params/face_service_ws/（人脸检测服务）

构建：

```bash
cd /home/zzw/ROS/ros_programs/service_params/face_service_ws
colcon build --symlink-install
source install/setup.bash
```

运行：

- demo_python_service
	- `ros2 run demo_python_service learn_face_detecte`
	- `ros2 run demo_python_service face_detect_service_node`
	- `ros2 run demo_python_service face_detect_client_node`

### service_params/patrol_ws/（巡航服务）

构建：

```bash
cd /home/zzw/ROS/ros_programs/service_params/patrol_ws
colcon build --symlink-install
source install/setup.bash
```

运行：

- patrol_cpp_service
	- `ros2 run patrol_cpp_service turtle_control`
	- `ros2 run patrol_cpp_service patrol_client`
	- `ros2 launch patrol_cpp_service demo.launch.py`
	- `ros2 launch patrol_cpp_service actions.launch.py`

### tools/tf_ws/（TF 工具工作区）

构建：

```bash
cd /home/zzw/ROS/ros_programs/tools/tf_ws
colcon build --symlink-install
source install/setup.bash
```

运行：

- tf_cpp
	- `ros2 run tf_cpp static_tf`
	- `ros2 run tf_cpp dynamic_tf`
	- `ros2 run tf_cpp tf_listener`
- tf_python
	- `ros2 run tf_python static_tf_broadcaster`
	- `ros2 run tf_python dynamic_tf_broadcaster`
	- `ros2 run tf_python tf_listener`

## 许可证

本仓库遵循 [LICENSE](LICENSE) 中的许可协议。
