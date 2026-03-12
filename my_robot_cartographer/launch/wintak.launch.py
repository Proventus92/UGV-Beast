import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Recuperer le chemin du package LiDAR
    ldlidar_dir = get_package_share_directory('ldlidar_stl_ros2')

    # --- 1. LE LIDAR (LD19) ---
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ldlidar_dir, 'launch', 'ld19.launch.py'))
    )

    # --- 2. LE PILOTE WINTAK / CoT (Serveur UDP) ---
    driver = Node(
        package='ugv_driver',
        executable='wintak_driver', # Notre nouveau script sans MAVLink !
        name='wintak_driver',
        output='screen'
    )

    # --- 3. LA TF STATIQUE POUR L'IMU ---
    # Dit a ROS que l'IMU est exactement au centre du robot (base_link)
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    return LaunchDescription([
        lidar,
        driver,
        imu_tf
    ])
