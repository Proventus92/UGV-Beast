import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_lidar = get_package_share_directory('ldlidar_stl_ros2')

    # 1. LIDAR
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_lidar, 'launch', 'ld19.launch.py'))
    )

    # 2. DRIVER MAVROS (script Python qui fait le pont)
    driver = Node(
        package='ugv_driver', 
        executable='mavros_driver',
        name='mavros_driver',
        output='screen', 
        parameters=[{'publish_tf': False}] # Cartographer/Nav2 gèreront la TF plus tard
    )

    # 3. MAVROS OFFICIEL (Le traducteur UDP)
    mavros = Node(
        package='mavros', executable='mavros_node', name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': 'udp://:14551@', 
            'gcs_url': 'udp://:14550@',  # Mode écoute pour QGC
            'target_system_id': 1,
            'system_id': 1,
            'fcu_protocol': 'v2.0',
            'plugin_allowlist': ['sys_status', 'sys_time', 'command', 'setpoint_velocity', 'manual_control', 'rc_io', 'param']
        }]
    )

    # 4. TF STATIQUE IMU (Pour que l'arbre TF soit prêt)
    imu_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='base_link_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )

    # Note : Cartographer et Nav2 ont été retirés. 
    # Ils seront lancés par subprocess via 'discovery.launch.py' lors du "Start Mission" depuis QGC.

    return LaunchDescription([lidar, driver, mavros, imu_tf])
