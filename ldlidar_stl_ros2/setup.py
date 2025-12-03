import os
from glob import glob # <--- ASSUREZ-VOUS QUE CET IMPORT EST LÀ
from setuptools import setup

package_name = 'ldlidar_stl_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    
    # --- C'EST LA SECTION IMPORTANTE ---
    # Elle dit à ROS 2 de copier les fichiers de lancement (launch)
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # AJOUTEZ CETTE LIGNE :
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    # ------------------------------------
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'ldlidar_stl_ros2_node = ldlidar_stl_ros2.ldlidar_stl_ros2_node:main',
        ],
    },
)
