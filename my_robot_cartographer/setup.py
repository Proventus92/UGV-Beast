import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_cartographer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='proventus',
    maintainer_email='severin.etievant@procomm-mmc',
    description='Configuration Cartographer',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'my_cartographer_node = my_robot_cartographer.my_cartographer_node:main',
            'cartographer_initialpose = my_robot_cartographer.cartographer_initialpose:main',
        ],
    },
)
