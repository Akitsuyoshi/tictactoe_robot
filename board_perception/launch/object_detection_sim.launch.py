from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("board_perception")

    rviz_config = PathJoinSubstitution([
        package_share,
        "rviz",
        "object_detection.rviz",
    ])

    model_path = PathJoinSubstitution([
        package_share,
        "weights",
        "best.onnx",
    ])
    
    return LaunchDescription([
        Node(
            package="board_perception",
            executable="object_detection",
            name="object_detection",
            output="screen",
            parameters=[
                {
                    "image_topic": "/camera1/image_raw",
                    "model_path": model_path,
                    "is_sim": True,
                }
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            parameters=[
                {
                    "is_sim": True,
                }
            ],
        ),

    ])