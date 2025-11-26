from setuptools import setup
import os
from glob import glob

package_name = 'obstacle_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='test@example.com',
    description='Front obstacle detector for LIDAR',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'obstacle_detector = obstacle_detector.obstacle_detector:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
)

