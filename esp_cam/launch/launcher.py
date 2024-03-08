from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='esp_cam',
            executable='solution.py'
        ),
        Node(
            package='esp_cam',
            executable='cam_sub.py'
        )
    ])