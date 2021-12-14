from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description = LaunchDescription([
        Node(
            package='robotx_costmap_calculator',
            executable='costmap_calculator_node',
            name='costmap_calculator_node',
            output='screen'),
    ])
    return description