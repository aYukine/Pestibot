from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hw_interface',
            executable='hw_node',
            name='hardware_bridge'
        ),
        Node (
            package='dc_gamepad',
            executable='gamepad_node',
            name='controller'
        ),
        # Node(
        #     package='camera_pkg',
        #     executable='ai_cam_node',
        #     name='ai'
        # ),
        Node(
            package='pestibot_pkg',
            executable='pestibot_control_node',
            name='pestibot'
        ),
    ])