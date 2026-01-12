import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_cartographer')
    lidar_pkg = get_package_share_directory('ldlidar_stl_ros2')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    
    # Chemins des configurations
    lua_config = 'slam.lua' 
    nav2_params = os.path.join(pkg_share, 'params', 'nav2_discovery.yaml')

    # 1. LIDAR
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg, 'launch', 'ld19.launch.py'))
    )
    
    # 2. DRIVER (Sans publication de TF, géré par urdf/robot_state_publisher si dispo, ou ignoré ici)
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

    # --- PARTIE CAMERA OAK-D (NOUVEAU) ---
    
    # 4. CAMERA NODE
    oakd_node = Node(
        package=None,
        executable=sys.executable,
        arguments=['/home/jetson/ros2_ws/src/oak_d_lite/oakd_pointcloud_node.py'],
        name='oakd_driver',
        output='screen'
    )

    # 5. TF STATIQUE CAMERA
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=['0.05', '0', '0.10', '-1.57', '0.0', '-1.57', 'base_link', 'oak_rgb_camera_optical_frame']
    )
    # -------------------------------------

    # 6. CARTOGRAPHER (SLAM)
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

    # 7. GRILLE OCCUPATION
    occupancy = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    # 8. NAV2 (CORRIGÉ)
    # Suppression de 'use_composition': 'True' qui faisait planter la connexion
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
        oakd_node,
        camera_tf,
        cartographer,
        occupancy,
        nav2
    ])