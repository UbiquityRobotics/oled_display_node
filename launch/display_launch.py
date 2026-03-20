from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oled_display_node',
            executable='oled_display_node',
            name='oled_display_node',
            output='screen',
            parameters=[
                {'update_delay_ms': 250}
            ],
            respawn=False
        )
    ])
