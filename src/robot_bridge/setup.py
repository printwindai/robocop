from setuptools import setup
import os
from glob import glob

package_name = 'robot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # installs the robot_bridge/ package dir
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include launch/config if present
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='ROS 2 bridge node in Python',
    license='MIT',
    entry_points={
        'console_scripts': [
            # executable name      module:function
            'bridge_node = robot_bridge.robot_bridge:main',
        ],
    },
)
