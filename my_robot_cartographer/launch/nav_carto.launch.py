import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- CONFIGURATION ---
    pkg_share = get_package_share_directory('my_robot_cartographer')
    lidar_pkg = get_package_share_directory('ldlidar_stl_ros2')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    
    # FICHIERS
    pbstream_file = '/home/jetson/ma_carte.pbstream'
    lua_config = 'localization.lua'
    nav2_params = os.path.join(pkg_share, 'params', 'my_nav2_params.yaml')

    # 1. LIDAR
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg, 'launch', 'ld19.launch.py'))
    )
    
    # 2. DRIVER MOTEUR (Source de l'Odométrie brute + IMU)
    # On met 'publish_tf': True. C'est indispensable pour Nav2.
    # Cartographer va écouter cette TF et la corriger via le lien map->odom.
    driver = Node(
        package='ugv_driver',
        executable='driver',
        name='ugv_driver',
        output='screen',
        parameters=[{'publish_tf': True}] 
    )

    # 3. TF STATIQUE IMU (INDISPENSABLE)
    # Crée le lien physique entre le robot et l'IMU.
    # Arguments: x y z yaw pitch roll parent child
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    # 4. CARTOGRAPHER (Le Cerveau)
    # Il utilise l'odom, l'IMU et le Lidar pour calculer la vraie position.
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_share, 'config'),
            '-configuration_basename', lua_config,
            '-load_state_filename', pbstream_file
        ]
    )

    # 5. Convertisseur pour Nav2
    occupancy = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0', '-include_frozen_submaps', 'true']
    )
    
    # 6. PONT INITIAL POSE (Pour le clic Rviz)
    initial_pose_bridge = Node(
        package='my_robot_cartographer',
        executable='cartographer_initialpose', 
        name='cartographer_initialpose_bridge',
        output='screen',
        parameters=[
            {'configuration_directory': os.path.join(pkg_share, 'config')},
            {'configuration_basename': lua_config}
        ]
    )

    # 7. NAV2 (Démarrage retardé)
    nav2 = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': 'False',
                    'params_file': nav2_params,
                    'autostart': 'True'
                }.items()
            )
        ]
    )

    return LaunchDescription([
        lidar,
        driver,
        imu_tf, # Ajouté ici
        cartographer,
        occupancy,
        initial_pose_bridge,
        nav2
    ])