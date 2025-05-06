# library to move between files and folders in the O.S.
import os

from ament_index_python.packages import get_package_share_directory

# libraries to define the Launch file and Function
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='navigation' #<--- CHANGE ME
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    rviz_config = os.path.join(get_package_share_directory(package_name),'config','display_test_rviz.rviz')
    
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Manual odometry node
    manual_odom_node_slam = Node(
        package=package_name,
        executable='manual_odometry_node_slam',
        name='manual_odometry_node_slam',
        output='screen'
    )
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/cmd_vel')]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    #Agregar tu nodo laser_scan_reader aquí
    # laser_scan_reader_node = Node(
    #     package=package_name,  # Reemplaza 'tu_paquete' con el nombre de tu paquete
    #     executable='laser_scan_reader',  # Nombre del ejecutable de tu nodo
    #     output='screen',  # Esto hace que el nodo imprima en la consola
    #     parameters=[{'use_sim_time': True}]  # Parametro de tiempo de simulación (si es necesario)
    # )

    # Launch them all!
    return LaunchDescription([
        joystick,
        twist_mux_node,
        manual_odom_node_slam,
        rviz_node,
        # laser_scan_reader_node,
    ])