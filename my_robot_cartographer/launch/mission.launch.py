import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_carto = get_package_share_directory('my_robot_cartographer')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(pkg_carto, 'params', 'nav2_discovery.yaml')

    # --- 1. CAMERA OAK-D (BIEN PRISE EN COMPTE) ---
    oakd_node = Node(
        package=None,
        executable=sys.executable,
        arguments=['/home/jetson/ros2_ws/src/oak_d_lite/oakd_pointcloud_node.py'],
        name='oakd_driver',
        output='screen'
    )

    # --- 2. TF STATIQUE DE LA CAMERA ---
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=['0.05', '0', '0.10', '-1.57', '0.0', '-1.57', 'base_link', 'oak_rgb_camera_optical_frame']
    )

    # 3. CARTOGRAPHER
    cartographer = Node(
        package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
        output='screen', parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_carto, 'config'),
            '-configuration_basename', 'slam.lua'
        ]
    )

    # 4. CARTOGRAPHER GRID
    cartographer_grid = Node(
        package='cartographer_ros', executable='cartographer_occupancy_grid_node', name='cartographer_occupancy_grid_node',
        output='screen', parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    # 5. NAV2 (Retarde a 15 secondes pour laisser l'OAK-D demarrer)
    nav2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
                launch_arguments={'use_sim_time': 'False', 'params_file': nav2_params, 'autostart': 'True'}.items()
            )
        ]
    )

    # 6. EXPLORATION AUTONOME (Retarde a 22 secondes)
    script_path = '/home/jetson/ros2_ws/discovery/mavros_exploration.py'

    exploration = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', script_path],
                output='screen',
                sigterm_timeout='8', # Donne le temps de sauver la carte
                sigkill_timeout='8'
            )
        ]
    )

    return LaunchDescription([
        oakd_node, 
        camera_tf, 
        cartographer, 
        cartographer_grid, 
        nav2, 
        exploration
    ])
