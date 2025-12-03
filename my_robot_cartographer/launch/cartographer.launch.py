import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # === 1. Récupération des chemins des paquets ===
    ldlidar_pkg_dir = get_package_share_directory('ldlidar_stl_ros2')
    cartographer_pkg_dir = get_package_share_directory('my_robot_cartographer')

    # === Nœud 1 : Lancement du Pilote Lidar ===
    lidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_pkg_dir, 'launch', 'ld19.launch.py')
        )
    )

    # === Nœud 2 : Lancement de Cartographer ===
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-configuration_directory', os.path.join(cartographer_pkg_dir, 'config'),
                   '-configuration_basename', 'my_robot.lua']
    )

    # === Nœud 3 : Lancement de la Grille d'Occupation ===
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    # === Nœud 4 : Lancement du NOUVEAU Pilote Moteur (Python) ===
    motor_driver_node = Node(
        package='ugv_driver',
        executable='driver',
        name='ugv_motor_driver',
        output='screen',
        parameters=[
            {'publish_tf': False}  # <--- ICI : On désactive la TF pour Cartographer
        ]
        
    )

    # === Assemblage Final ===
    return LaunchDescription([
        lidar_driver_launch,
        cartographer_node,
        cartographer_occupancy_grid_node,
        motor_driver_node
    ])
