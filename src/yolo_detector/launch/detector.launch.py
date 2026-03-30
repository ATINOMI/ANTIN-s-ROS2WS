from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            'PYTHONPATH',
            '/home/a/ultralytics-8.3.163:'
	    '/home/a/anaconda3/envs/yolo11/lib/python3.12/site-packages:'
	    '/home/a/.local/lib/python3.12/site-packages:'
	    '/home/a/ros2_ws/install/yolo_detector/lib/python3.12/site-packages:'
	    '/home/a/ros2_ws/src/yolo_detector:'
	    '/opt/ros/jazzy/lib/python3.12/site-packages:'
	    '/home/a/ros2_ws/install/yolo_msgs/lib/python3.12/site-packages:'
        ),
        Node(
            package='yolo_detector',
            executable='detector_node',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'model_path': '/home/a/ros2_ws/src/yolo_detector/yolo_detector/yolo11n.engine',
                'input_path': '0',
            }],
        ),
    ])
