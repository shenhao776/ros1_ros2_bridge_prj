from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件路径
    bridge_config_path = os.path.join(
        get_package_share_directory("bridge_launcher"),  # 替换为实际包名
        "config",
        "bridge.yaml",
    )

    return LaunchDescription(
        [
            # 启动parameter_bridge（替代dynamic_bridge）
            Node(
                package="ros1_bridge",
                executable="parameter_bridge",  # 使用parameter_bridge
                name="ros1_bridge",
                output="screen",
                # 通过参数指定配置文件（ROS1参数服务器需提前加载该配置）
                parameters=[{"config_file": bridge_config_path}],  # 传递配置文件路径
            )
        ]
    )
