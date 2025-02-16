# library to move between files and folders in the O.S.
import os

from ament_index_python.packages import get_package_share_directory

# libraries to define the Launch file and Function
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name='wall_following_alpha' #<--- CHANGE ME

    gap_finder = Node(
        package=package_name,  # Reemplaza 'tu_paquete' con el nombre de tu paquete
        executable='gap_finder.py',  # Nombre del ejecutable de tu nodo
        output='screen',  # Esto hace que el nodo imprima en la consola
        parameters=[{'use_sim_time': True}]  # Parametro de tiempo de simulación (si es necesario)
    )

    controller = Node(
        package=package_name,  # Reemplaza 'tu_paquete' con el nombre de tu paquete
        executable='control_alpha_gap.py',  # Nombre del ejecutable de tu nodo
        output='screen',  # Esto hace que el nodo imprima en la consola
        parameters=[{'use_sim_time': True}]  # Parametro de tiempo de simulación (si es necesario)
    )

    # Launch them all!
    return LaunchDescription([
        gap_finder,
        controller,
    ])