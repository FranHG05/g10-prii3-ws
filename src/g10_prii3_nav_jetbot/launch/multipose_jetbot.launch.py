from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g10_prii3_nav_jetbot',
            executable='multipose',
            name='multipose_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])
