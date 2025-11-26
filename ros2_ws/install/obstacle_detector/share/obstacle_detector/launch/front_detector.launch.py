from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_detector',
            executable='obstacle_detector',
            name='front_obstacle_detector',
            output='screen',
            parameters=[
                {'angle_limit_deg': 90.0},
                {'radius': 1.0}
            ]
        )
    ])
