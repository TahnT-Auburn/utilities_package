from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_path = "/home/tahnt/T3_Repos/post_process_packages/ros2_ws/src/utilities_package/config/video_writer_config.yaml"
    
    return LaunchDescription([
        Node(
            package="utilities_package",
            executable="video_writer",
            name="video_writer",
            output="screen",
            emulate_tty=True,
            parameters=[config_path]
        ),
    ])