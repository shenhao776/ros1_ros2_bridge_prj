from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for starting the ros1_bridge dynamic_bridge.
    """
    return LaunchDescription([
        Node(
            package='ros1_bridge',
            executable='dynamic_bridge',
            name='ros1_bridge',
            output='screen',
            arguments=[
                '--bridge-all-2to1-topics'
            ]
        )
    ])