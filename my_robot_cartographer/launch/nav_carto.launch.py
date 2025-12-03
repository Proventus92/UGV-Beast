import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- CONFIGURATION ---
    pkg_share = get_package_share_directory('my_robot_cartographer')
    lidar_pkg = get_package_share_directory('ldlidar_stl_ros2')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    
    # Fichier Carte (celui que vous avez généré)
    pbstream_file = '/home/jetson/ma_carte.pbstream'
    
    # Fichier Config (celui qu'on vient de créer)
    lua_config = 'localization.lua'
    
    # Params Nav2 (On garde les mêmes)
    nav2_params = os.path.join(pkg_share, 'params', 'my_nav2_params.yaml')

    # --- 1. MATÉRIEL ---
    
    # Lidar
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg, 'launch', 'ld19.launch.py'))
    )
    
    # Driver Moteur : ON COUPE LA TF (publish_tf: False)
    # C'est Cartographer qui va gérer la TF
    driver = Node(
        package='ugv_driver',
        executable='driver',
        name='ugv_driver',
        output='screen',
        parameters=[{'publish_tf': True}] 
    )

    # --- 2. CARTOGRAPHER (Localisation) ---
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', os.path.join(pkg_share, 'config'),
            '-configuration_basename', lua_config,
            '-load_state_filename', pbstream_file # CHARGE LA CARTE
        ]
    )

    # Convertisseur pour que Nav2 voie les obstacles
    occupancy = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0', '-include_frozen_submaps', 'true']
    )

    # --- 3. NAV2 (Planificateur seulement) ---
    # On lance "navigation_launch.py" qui ne contient PAS AMCL ni MapServer
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_params,
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([
        lidar,
        driver,
        cartographer,
        occupancy,
        nav2
    ])
