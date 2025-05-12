from setuptools import find_packages, setup

package_name = 'drone_sim'

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdullah AlMusalami',
    maintainer_email='agm.musalami@gmail.com',
    description='A drone sim with PX4 and GZ Garden',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'offboard_control = drone_sim.offboard_control_node:main',
        ],
    },
)
