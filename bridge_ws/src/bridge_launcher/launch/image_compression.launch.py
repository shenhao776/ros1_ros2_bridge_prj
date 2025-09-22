from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description for starting only the image compression node."""
    raw_image_topic = "/camera/camera/color/image_raw"

    return LaunchDescription(
        [
            Node(
                package="image_transport",
                executable="republish",
                name="image_republisher",
                arguments=["raw", "compressed"],
                remappings=[
                    ("in", raw_image_topic),
                    # 'out' is automatically handled by the node based on the 'in' topic
                ],
                output="screen",
            ),
        ]
    )
