# FILE NAME: setup.py (do robot_description_pkg)
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_description_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yeml]'))),
        ('share/' + package_name + '/urdf', glob(os.path.join('urdf', '*.urdf*'))),
        ('share/' + package_name + '/urdf', glob(os.path.join('urdf', '*.xacro'))),
        ('share/' + package_name + '/rviz', glob(os.path.join('rviz', '*.rviz'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = robot_description_pkg.odometry_node:main',
            'sensor_node = robot_description_pkg.sensor_node:main',
            'simple_odom = robot_description_pkg.simple_odom_publisher_node:main',
            # 'camera_publisher = robot_description_pkg.camera_publisher_node:main',
            'fake_lidar = robot_description_pkg.fake_lidar_publisher_node:main',
        ],
    },
)

