from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_move_turtle',
            executable='publisher_exec',
            output='screen'),
        Node(
            package='ros2_move_turtle',
            executable='subscriber_exec',
            output='screen'),
    ])