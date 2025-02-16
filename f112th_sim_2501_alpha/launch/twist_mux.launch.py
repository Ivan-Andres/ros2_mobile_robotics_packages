import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Nombre del paquete (cambiar si es necesario)
    package_name = 'f112th_sim_2501_alpha'

    # Ruta a los parámetros del joystick
    joy_params = os.path.join(get_package_share_directory(package_name), 'config', 'joystick.yaml')
    
    # Ruta a los parámetros de twist_mux
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'parms.yaml')

    # Nodo para el joystick
    joy_node = Node(
        package='joy', 
        executable='joy_node',
        parameters=[joy_params],
    )

    # Nodo para teleop joy
    teleop_node = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name="teleop_node",
        parameters=[joy_params],
        remappings=[('cmd_vel', 'joy_cmd_vel')]  # Publica en 'joy_cmd_vel'
    )

    # Nodo para twist_mux
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name="twist_mux",
        parameters=[twist_mux_params],
        remappings=[
            ('cmd_vel', 'cmd_vel_out'),  # Salida final del twist_mux
        ]
    )

    # Lanzar todos los nodos
    return LaunchDescription([
        joy_node,
        teleop_node,
        twist_mux_node,
    ])