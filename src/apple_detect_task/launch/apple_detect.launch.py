from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_path = LaunchConfiguration("video_path")
    image_topic = LaunchConfiguration("image_topic")
    publish_period_ms = LaunchConfiguration("publish_period_ms")
    frame_id = LaunchConfiguration("frame_id")
    result_topic = LaunchConfiguration("result_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "video_path",
            default_value="/media/a/新加卷/工科学习资料/cpp/opencv-cpp/apple.mp4",
            description="Path to the video published by image_publisher_node",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/image_raw",
            description="Image topic shared by the publisher and monitor nodes",
        ),
        DeclareLaunchArgument(
            "publish_period_ms",
            default_value="0",
            description="Publishing period in milliseconds; 0 means use the video's FPS",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="camera",
            description="Frame ID attached to the published image",
        ),
        DeclareLaunchArgument(
            "result_topic",
            default_value="/apple_detector/result",
            description="Topic for the processed detection image",
        ),

        Node(
            package="apple_detect_task",
            executable="image_publisher_node",
            name="image_publisher",
            output="screen",
            parameters=[{
                "video_path": video_path,
                "image_topic": image_topic,
                "publish_period_ms": publish_period_ms,
                "frame_id": frame_id,
            }],
        ),

        Node(
            package="apple_detect_task",
            executable="monitor_node",
            name="image_monitor",
            output="screen",
            parameters=[{
                "image_topic": image_topic,
                "result_topic": result_topic,
            }],
        ),
    ])
