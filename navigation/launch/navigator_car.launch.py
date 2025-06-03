from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    navigation_dir = get_package_share_directory('navigation')
    rviz_config_path = os.path.join(navigation_dir, 'config', 'display_rviz.rviz')
    map_path = '/home/ivan/ros2_ws_2501/final_map.yaml'
    params_file_path = os.path.join(navigation_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        # odometria desde vicon
        Node(
            package='navigation',
            executable='manual_odometry_node_slam',
            name='manual_odometry_node_slam',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
        ),

        # # localización
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation_dir, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'map': map_path,
                'params_file': params_file_path
            }.items()
        ),

        # 4. Nodo de planificación A*
        TimerAction(
        period=5.0,  # Wait to ensure map is published
        actions=[

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
        ]
        )
    ])
