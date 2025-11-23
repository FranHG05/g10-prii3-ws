from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g10_prii3_nav_jetbot',
            executable='autonomous_navigation',
            name='autonomous_navigation',
            output='screen'
        )
    ])
