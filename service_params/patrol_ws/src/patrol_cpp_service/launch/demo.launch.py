import launch
import launch_ros

def generate_launch_description():
    """进行参数声明"""
    action_declare_arg_background_r = launch.actions.DeclareLaunchArgument(
        "background_r", default_value = "255")
    action_declare_arg_background_g = launch.actions.DeclareLaunchArgument(
        "background_g", default_value = "0")
    action_declare_arg_background_b = launch.actions.DeclareLaunchArgument(
        "background_b", default_value = "0")
    
    """
    生成launch描述文件
    生成action_node
    """
    action_node_turtlesim = launch_ros.actions.Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[{"background_r" : launch.substitutions.LaunchConfiguration("background_r")},
                    {"background_g" : launch.substitutions.LaunchConfiguration("background_g")},
                    {"background_b" : launch.substitutions.LaunchConfiguration("background_b")}],
        output="screen",
    )
    action_node_patrol_client = launch_ros.actions.Node(
        package="patrol_cpp_service",
        executable="patrol_client",
        output="log",
    )
    action_node_turtle_control = launch_ros.actions.Node(
        package="patrol_cpp_service",
        executable="turtle_control",
        output="both",
    )
    """返回固定格式"""
    return launch.LaunchDescription([
        # actions
        action_declare_arg_background_r,
        action_declare_arg_background_g,
        action_declare_arg_background_b,
        action_node_turtlesim,
        action_node_patrol_client,
        action_node_turtle_control
    ])