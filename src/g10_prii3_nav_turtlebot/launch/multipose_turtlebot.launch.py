from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Argumento opcional para cambiar el mapa sin modificar el archivo
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='worlds/map.yaml',
        description='Ruta al archivo de mapa YAML'
    )

    return LaunchDescription([
        map_file_arg,
        Node(
            package='g10_prii3_nav_turtlebot',
            executable='multipose',
            name='multipose_node',
            output='screen',
            parameters=[{'use_sim_time': True, 'map_yaml_file': LaunchConfiguration('map')}]
        )
    ])
