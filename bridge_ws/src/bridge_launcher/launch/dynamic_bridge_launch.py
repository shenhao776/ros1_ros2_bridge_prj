import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Get the full path to your bridge.yaml configuration file
    bridge_config_path = os.path.join(
        get_package_share_directory("bridge_launcher"), "config", "bridge.yaml"
    )

    return LaunchDescription(
        [
            # Action 1: Load the parameters into the ROS1 parameter server.
            # This must happen before the bridge starts.
            ExecuteProcess(
                cmd=["rosparam", "load", bridge_config_path], output="screen"
            ),
            # Action 2: Run the parameter_bridge using 'ros2 run'.
            # This is launched as a simple executable process to prevent
            # ros2 launch from adding extra arguments that cause the crash.
            ExecuteProcess(
                cmd=["ros2", "run", "ros1_bridge", "parameter_bridge"], output="screen"
            ),
        ]
    )
