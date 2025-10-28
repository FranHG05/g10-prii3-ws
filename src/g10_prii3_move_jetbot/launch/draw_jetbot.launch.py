from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g10_prii3_move_jetbot',
            executable='draw_number',
            name='draw_number_node_jetbot',
            output='screen'
        ),
        Node(
            package='g10_prii3_move_jetbot',
            executable='collision_avoidance',
            name='collision_avoidance_node_jetbot',
            output='screen'
        ),
    ])