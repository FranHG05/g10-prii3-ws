from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ruta del modelo del cubo
    pkg_share = get_package_share_directory('g10_prii3_move_turtlebot')
    model_path = os.path.join(pkg_share, 'models', 'box_obstacle', 'model.sdf')

    # Nodo que dibuja el número
    draw_node = Node(
        package='g10_prii3_move_turtlebot',
        executable='draw_number',
        name='draw_number_node',
        output='screen'
    )

    # Nodo de evasión de colisiones
    collision_node = Node(
        package='g10_prii3_move_turtlebot',
        executable='collision_avoidance',
        name='collision_avoidance_node',
        output='screen'
    )

    # Spawnear el cubo delante del TurtleBot
    spawn_box = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'box_obstacle',
            '-file', model_path,
            '-x', '0.5', '-y', '0.0', '-z', '0.15'
        ],
        output='screen'
    )

    return LaunchDescription([
        draw_node,
        collision_node,
        spawn_box,
    ])
