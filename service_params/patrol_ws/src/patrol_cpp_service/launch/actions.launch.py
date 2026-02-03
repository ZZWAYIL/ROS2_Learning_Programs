import launch
import launch_ros
import launch.launch_description_sources
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明
    action_arg_rqt = launch.actions.DeclareLaunchArgument("start_rqt", default_value="False")

    start_rqt = launch.substitutions.LaunchConfiguration("start_rqt", default="False")

    # action1 : 启动其他launch文件
    multisim_launch_path = [get_package_share_directory("turtlesim"),"/launch","/multisim.launch.py"]
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            multisim_launch_path
        )
    )

    # action2 : 打印包含的launch文件信息 
    action_log_info = launch.actions.LogInfo(msg=str(action_include_launch))

    # action3 ： 执行进程（即：输入一个命令行）
    # action_topic_list = launch.actions.ExecuteProcess(
    #     cmd = ["ros2","topic","list"],
    #     output = "screen"
    # )
    # if start_rqt == "True":
    #   run rqt
    action_topic_list = launch.actions.ExecuteProcess(
        condition = launch.conditions.IfCondition(start_rqt),
        cmd = ["rqt"]
    )

    # action4 : 把动作成组
    action_group = launch.actions.GroupAction(
        [
            # action5 : 定时器，顺序执行
            launch.actions.TimerAction(period=1.0,actions=[action_include_launch]),
            launch.actions.TimerAction(period=3.0,actions=[action_topic_list])
        ]
    )


    """返回固定格式"""
    return launch.LaunchDescription([
        action_arg_rqt,
        action_group,
        action_log_info
    ])