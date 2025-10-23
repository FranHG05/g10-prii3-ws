from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        # Tu nodo que dibuja el 10
        Node(
            package='g10_prii3_turtlesim',
            executable='draw_10',
            name='draw10'
        )
    ])
