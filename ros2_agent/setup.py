from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add config files to the installation
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Add launch files to the installation
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'rosa'],  # Added 'rosa' dependency
    zip_safe=True,
    maintainer='AbdullagGN1',
    maintainer_email='agm.musalami@gmail.com',
    description='ROS2 Agent for ISAR robots system with Natural Language Control using Large Language Models',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'ros2_agent_node = ros2_agent.ros2_agent_node:main'
        ],
    },
)