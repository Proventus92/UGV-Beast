import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_cartographer')
    lidar_pkg = get_package_share_directory('ldlidar_stl_ros2')
    
    # Config EKF
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # 1. LIDAR (Indispensable pour RF2O)
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg, 'launch', 'ld19.launch.py'))
    )
    
    # 2. DRIVER MOTEUR
    # IMPORTANT : publish_tf=False car c'est l'EKF qui va publier la TF odom->base_link
    driver = Node(
        package='ugv_driver',
        executable='driver',
        name='ugv_driver',
        output='screen',
        parameters=[{'publish_tf': False}] 
    )

    # 3. RF2O (Odométrie Laser)
    # Il lit /scan et publie /odom_rf2o
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False, # On laisse l'EKF faire le chef
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0
        }]
    )

    # 4. EKF (Fusion)
    # Il prend /odom_rf2o + /imu et publie la TF officielle odom->base_link
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odom')] # Sortie standard
    )

    return LaunchDescription([
        lidar,
        driver,
        rf2o,
        ekf
    ])
