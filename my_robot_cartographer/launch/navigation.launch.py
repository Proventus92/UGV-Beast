import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_cartographer')
    lidar_pkg = get_package_share_directory('ldlidar_stl_ros2')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    
    # Chemins
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    map_file = os.path.join(os.path.expanduser('~'), 'ma_carte.yaml')
    nav2_params = os.path.join(pkg_share, 'params', 'my_nav2_params.yaml')

    # 1. LIDAR
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg, 'launch', 'ld19.launch.py'))
    )

    # 2. DRIVER (Moteurs + IMU)
    # publish_tf = False car c'est l'EKF qui gère !
    driver = Node(
        package='ugv_driver', executable='driver', name='ugv_driver',
        output='screen', parameters=[{'publish_tf': False}] 
    )

    # 3. RF2O (Calcule la vitesse grâce au Lidar)
    rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry', output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0
        }]
    )

    # 4. EKF (Le Cerveau de Fusion)
    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odom')] # Sortie standardisée
    )

    # 5. NAV2 (AMCL + Navigation)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params,
            'use_sim_time': 'False',
            'autostart': 'True'
        }.items()
    )

    return LaunchDescription([lidar, driver, rf2o, ekf, nav2])