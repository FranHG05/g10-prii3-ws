from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g10_prii3_move_turtlebot',
            executable='draw_number',
            name='draw_number_node',
            output='screen'
        ),
        Node(
            package='g10_prii3_move_turtlebot',
            executable='obstacle_avoidance',
            name='obstacle_avoidance_node',
            output='screen'
        ),
    ])