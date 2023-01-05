from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_project',
            executable='visual_navigation',
            name='visual_navigation',
            output='screen',
            namespace='robot0'),
    ])
