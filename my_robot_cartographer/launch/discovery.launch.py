import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_cartographer')
    lidar_pkg = get_package_share_directory('ldlidar_stl_ros2')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    
    # Fichiers specifiques pour la decouverte
    lua_config = 'slam.lua' 
    nav2_params = os.path.join(pkg_share, 'params', 'nav2_discovery.yaml')

    # 1. LIDAR
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg, 'launch', 'ld19.launch.py'))
    )
    
    # 2. DRIVER (Toujours publish_tf=False)
    driver = Node(
        package='ugv_driver',
        executable='driver',
        name='ugv_driver',
        output='screen',
        parameters=[{'publish_tf': False}] 
    )

    # 3. TF STATIQUE IMU
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    # 4. CARTOGRAPHER (Mode SLAM - Pas de load_state_filename)
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_share, 'config'),
            '-configuration_basename', lua_config
        ]
    )

    # 5. GRILLE
    occupancy = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    # 6. NAV2
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
        imu_tf,
        cartographer,
        occupancy,
        nav2
    ])