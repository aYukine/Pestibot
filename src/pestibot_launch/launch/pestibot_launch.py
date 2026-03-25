from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hw_interface',
            executable='hw_node',
            name='hardware_bridge'
        ),
        Node(
            package='camera_pkg',
            executable='cam_ai_node',
            name='ai'
        ),
        Node(
            package='pestibot_pkg',
            executable='pestibot_node',
            name='pestibot'
        ),
    ])