from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'imu_receiver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wilherm',
    maintainer_email='wilherm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_udp_receiver = imu_receiver.imu_udp_receiver:main',
            'udp_ros2_receiver = imu_receiver.udp_ros2_receiver:main',
        ],
    },
)
