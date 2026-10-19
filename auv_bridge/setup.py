import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'auv_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arash',
    maintainer_email='arash@todo.todo',
    description='Bridge ROS 2 <-> auv-fc ESP32.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge = auv_bridge.bridge_node:main',
        ],
    },
)