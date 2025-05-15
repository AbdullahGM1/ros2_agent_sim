from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'drone_sim'

setup(
    # Package metadata
    name=package_name,
    version='0.0.0',
    description='A drone sim with PX4 and GZ Garden',
    maintainer='Abdullah AlMusalami',
    maintainer_email='agm.musalami@gmail.com',
    license='MIT',
    
    # Package structure
    packages=find_packages(),
    zip_safe=True,
    
    # Dependencies
    install_requires=['setuptools'],
    extras_require={
        'test': ['pytest'],
    },
    
    # Entry points for nodes
    entry_points={
        'console_scripts': [
            'offboard_control_node = drone_sim.offboard_control_node:main',
        ],
    },
    
    # Data files to install
    data_files=[
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        
        # Launch files (automatically includes all launch.py files)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Configuration files
        (os.path.join('share', package_name, 'mavros'), glob('mavros/*.yaml')),
        
        # Visualization
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        
        # Package resources
        (os.path.join('share', package_name, 'resource'), 
            ['resource/' + package_name]),
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            ['resource/' + package_name]),
    ],
)