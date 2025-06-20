#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command

def generate_launch_description():
    ld = LaunchDescription()

    # ====================
    # SAR SYSTEM PARAMETERS
    # ====================
    
    # Go2 Robot Configuration
    go2_ns = 'go2'
    go2_x = '5.0'
    go2_y = '0.0' 
    go2_z = '0.5'  # Slightly elevated to avoid ground collision

    # ====================
    # INCLUDE DRONE SYSTEM
    # ====================
    
    # Include the existing drone simulation
    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drone_sim'),
                'launch',  
                'drone.launch.py'
            ])
        ])
    )

    # ====================
    # ADD UNITREE GO2 ROBOT
    # ====================
    
    # Get Unitree Go2 package paths
    unitree_go2_description = get_package_share_directory('unitree_go2_description')
    go2_urdf_path = os.path.join(unitree_go2_description, 'urdf', 'unitree_go2_robot.xacro')
    
    # Unitree Go2 Robot Description
    go2_robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='go2_robot_state_publisher',
        namespace=go2_ns,
        parameters=[{
            'robot_description': Command(['xacro ', go2_urdf_path]),
            'use_sim_time': True
        }],
        output='screen'
    )
    
    # Spawn Unitree Go2 in Gazebo
    go2_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='go2_spawn',
        arguments=[
            '-name', go2_ns,
            '-topic', f'/{go2_ns}/robot_description',
            '-x', go2_x,
            '-y', go2_y, 
            '-z', go2_z
        ],
        output='screen'
    )

    # ====================
    # SAR SYSTEM TF SETUP
    # ====================
    
    # Go2 to map TF (for coordination)
    go2_to_map_tf = Node(
        package='tf2_ros',
        name='go2_to_map_tf_node',
        executable='static_transform_publisher',
        arguments=[go2_x, go2_y, '0.0', '0.0', '0', '0', 'map', f'{go2_ns}/odom'],
    )

    # ====================
    # BUILD LAUNCH DESCRIPTION
    # ====================
    
    # Add all components to launch description
    ld.add_action(drone_launch)         # Complete drone system
    ld.add_action(go2_robot_description) # Go2 robot description
    ld.add_action(go2_spawn)            # Go2 spawning in Gazebo
    ld.add_action(go2_to_map_tf)        # Go2 TF coordination

    return ld