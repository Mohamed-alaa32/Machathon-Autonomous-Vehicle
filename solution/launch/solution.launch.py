from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='solution',
            executable='solution',
            name='solution',
            output='screen'
        ),
        # Node(
        #     package='arduino_ble',
        #     executable='control_pub',
        #     name='control_pub'
        # ),
        Node(
            package='esp32_cam',
            executable='cam_sub',
            name='cam_sub'
        )
    ])