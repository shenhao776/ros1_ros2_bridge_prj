from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for starting the ros1_bridge dynamic_bridge
    with topic remapping.
    """
    return LaunchDescription([
        Node(
            package='ros1_bridge',
            executable='dynamic_bridge',
            name='ros1_bridge',
            output='screen',
            arguments=[
                '--bridge-all-2to1-topics'
                # '/camera/camera/color/image_raw@sensor_msgs/msg/Image@/camera/color/image_raw',

                # 示例: 将 ROS 1 的 /scan (类型: sensor_msgs/LaserScan) 桥接到 ROS 2
                # '/scan@sensor_msgs/LaserScan@{'
            ],
            remappings=[
                # 修改输入名: ('ROS 2中使用的话题名', '桥接后在ROS 1中使用的话题名')
                # ('/camera/camera/color/image_raw', '/camera/color/image_raw')
            ]
        )
    ])