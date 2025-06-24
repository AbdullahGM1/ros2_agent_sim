#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    # ====================
    # SAR SYSTEM PARAMETERS
    # ====================
    
    # Go2 Robot spawn position (away from drone)
    go2_x = '5.0'
    go2_y = '0.0' 
    go2_z = '0.375'

    # ====================
    # SET GAZEBO PLUGIN PATHS FOR gz_ros2_control
    # ====================
    
    # Add your workspace libraries to Gazebo plugin path so PX4 can find gz_ros2_control
    workspace_path = '/home/user/shared_volume/ros2_ws/install/gz_ros2_control/lib'
    
    set_gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            workspace_path + ':',
            '/usr/lib/x86_64-linux-gnu/gz-sim-7/plugins:',
            '/home/user/.gz/sim/plugins'
        ]
    )
    
    # Set resource path for gz_ros2_control
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/home/user/shared_volume/ros2_ws/install:' + '/usr/share'
    )
    
    # Suppress TF warnings
    suppress_warnings = SetEnvironmentVariable(
        name='RCUTILS_LOGGING_SEVERITY_THRESHOLD',
        value='ERROR'
    )

    # ====================
    # 1. LAUNCH DRONE SYSTEM FIRST
    # ====================
    
    print("🚁 Starting drone system with PX4 Gazebo...")
    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drone_sim'),
                'launch',  
                'drone.launch.py'
            ])
        ]),
        # Add log level arguments to suppress MAVROS TF warnings
        launch_arguments={
            'log_level': 'ERROR'
        }.items()
    )

    # ====================
    # 2. LAUNCH UNITREE GO2 (DELAYED)
    # ====================
    
    print("🐕 Preparing to launch Go2 robot with ros2_control (delayed)...")
    
    # Get Unitree config files for manual controller setup
    unitree_go2_sim = '/home/user/shared_volume/ros2_ws/install/unitree_go2_sim/share/unitree_go2_sim'
    ros_control_config = unitree_go2_sim + '/config/ros_control/ros_control.yaml'
    
    # Start controller manager manually (before Unitree launch)
    controller_manager_setup = TimerAction(
        period=12.0,  # After Go2 robot description is ready
        actions=[
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                name='controller_manager',
                output='screen',
                parameters=[
                    ros_control_config,
                    {'use_sim_time': True}
                ],
            )
        ]
    )
    
    unitree_launch = TimerAction(
        period=15.0,  # 15 second delay to ensure PX4 Gazebo is fully ready
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('unitree_go2_sim'),
                        'launch',  
                        'unitree_go2_launch.py'
                    ])
                ]),
                launch_arguments={
                    'world_init_x': go2_x,
                    'world_init_y': go2_y,
                    'world_init_z': go2_z,
                    'robot_name': 'go2',
                    'rviz': 'false',  # Disable RVIZ to avoid conflicts
                }.items()
            )
        ]
    )

    # ====================
    # 3. COORDINATION SETUP
    # ====================
    
    # Static TF for SAR mission coordination
    coordination_tf = Node(
        package='tf2_ros',
        name='sar_mission_tf',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0', '0', 'map', 'mission_frame'],
    )

    # ====================
    # BUILD LAUNCH DESCRIPTION
    # ====================
    
    ld.add_action(set_gz_plugin_path)    # Set Gazebo plugin paths
    ld.add_action(set_gz_resource_path)  # Set Gazebo resource paths  
    ld.add_action(suppress_warnings)     # Suppress TF warnings
    ld.add_action(drone_launch)          # 1. Drone + PX4 Gazebo
    ld.add_action(controller_manager_setup)  # 2. Controller manager (delayed)
    ld.add_action(unitree_launch)        # 3. Go2 (delayed, with ros2_control)
    ld.add_action(coordination_tf)       # 4. Mission coordination

    return ld