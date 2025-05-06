from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    navigation_dir = get_package_share_directory('navigation')
    world_path = os.path.join(navigation_dir, 'worlds', 'world_parcial.world')
    rviz_config_path = os.path.join(navigation_dir, 'config', 'display_rviz.rviz')

    return LaunchDescription([
        # 1. Lanzar Gazebo con el mundo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_dir, 'launch', 'launch_sim_ackerman.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

        # 2. Lanzar RViz con configuración personalizada
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        ),

        # 3. Lanzar SLAM y localización
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_dir, 'launch', 'slam_localization_launch.py')
            )
        ),

        # 4. Nodo de planificación A*
        Node(
            package='astar_planing',
            executable='astar_node',
            name='astar_node',
            output='screen'
        ),

        # 5. Nodo de seguimiento Pure Pursuit
        Node(
            package='pure_pursuit_tracking',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen'
        )
    ])
