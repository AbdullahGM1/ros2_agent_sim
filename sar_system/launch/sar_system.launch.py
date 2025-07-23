#!/usr/bin/env python3

import os
import launch_ros
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # ====================
    # SAR SYSTEM PARAMETERS
    # ====================
    
    # Drone spawn position
    drone_x = '0.0'
    drone_y = '0.0' 
    drone_z = '0.1'

    # Go2 Robot spawn position (away from drone)
    go2_x = '5.0'
    go2_y = '0.0' 
    go2_z = '0.375'

    # PX4 Default world path
    px4_world_path = '/home/user/shared_volume/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf'

    # Suppress warnings
    suppress_warnings = SetEnvironmentVariable(
        name='RCUTILS_LOGGING_SEVERITY_THRESHOLD',
        value='ERROR'
    )

    # ====================
    # 1. LAUNCH GAZEBO SIMULATION WITH PX4 WORLD
    # ====================
    
    print("🌍 Starting Gazebo Harmonic (GZ v8) with PX4 default world...")
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [px4_world_path, ' -r']
        }.items(),
    )

    # ====================
    # 2. SPAWN DRONE MODEL IN GAZEBO
    # ====================
    
    print("🚁 Spawning drone model in Gazebo...")
    
    spawn_drone = TimerAction(
        period=5.0,  # Wait for Gazebo to start
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-world', 'default',
                    '-file', '/home/user/shared_volume/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_camera/model.sdf',
                    '-name', 'x500_lidar_camera_1',
                    '-x', drone_x,
                    '-y', drone_y,
                    '-z', drone_z,
                    '-Y', '0.0'
                ],
            )
        ]
    )

    # ====================
    # 3. SPAWN UNITREE GO2 MODEL IN GAZEBO
    # ====================
    
    print("🐕 Spawning Unitree Go2 model in Gazebo...")
    
    # Get Unitree description paths
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description").find("unitree_go2_description")
    default_model_path = os.path.join(unitree_go2_description, "urdf/unitree_go2_robot.xacro")
    
    # Robot description publisher for Unitree (needed for topic-based spawning)
    unitree_robot_description = {"robot_description": Command(["xacro ", default_model_path])}
    
    unitree_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            unitree_robot_description,
            {"use_sim_time": True}
        ],
    )
    
    # Spawn Unitree Go2 using ros_gz_sim (GZ Harmonic compatible)
    spawn_unitree = TimerAction(
        period=8.0,  # Wait for drone to spawn first
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-world', 'default',
                    '-topic', 'robot_description',
                    '-name', 'go2',
                    '-x', go2_x,
                    '-y', go2_y,
                    '-z', go2_z,
                    '-Y', '0.0'
                ],
            )
        ]
    )

    # ====================
    # 4. LAUNCH DRONE SYSTEM (NO GZ)
    # ====================
    
    print("🚁 Starting drone system components...")
    
    drone_launch = TimerAction(
        period=10.0,  # Wait for models to spawn
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('drone_sim'),
                        'launch',  
                        'drone_no_gz.launch.py'
                    ])
                ]),
                launch_arguments={
                    'log_level': 'ERROR'
                }.items()
            )
        ]
    )

    # ====================
    # 5. LAUNCH UNITREE GO2 SYSTEM (NO GZ)
    # ====================
    
    print("🐕 Starting Unitree Go2 system components...")
    
    unitree_launch = TimerAction(
        period=15.0,  # 5 seconds after drone launch
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('unitree_go2_sim'),
                        'launch',  
                        'unitree_go2_no_gz.launch.py'
                    ])
                ]),
                launch_arguments={
                    'world_init_x': go2_x,
                    'world_init_y': go2_y,
                    'world_init_z': go2_z,
                    'robot_name': 'go2',
                    'rviz': 'false',  # Disable individual RVIZ to avoid conflicts
                }.items()
            )
        ]
    )

    # ====================
    # 6. SAR COORDINATION AND VISUALIZATION
    # ====================
    
    # Static TF for SAR mission coordination
    coordination_tf = Node(
        package='tf2_ros',
        name='sar_mission_tf',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0', '0', 'map', 'mission_frame'],
        parameters=[{"use_sim_time": True}]
    )

    # Main SAR RVIZ for monitoring both systems
    sar_rviz = TimerAction(
        period=20.0,  # After both systems are launched
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='sar_rviz2',
                output='screen',
                arguments=['-d', '/home/ab/ros2_agent_sim_shared_volume/ros2_ws/src/ros2_agent_sim/sar_system/rviz/sar.rviz'],
                parameters=[{"use_sim_time": True}]
            )
        ]
    )

    # System status monitor
    status_monitor = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='demo_nodes_cpp',
                executable='talker',
                name='sar_status',
                output='screen',
                parameters=[{"use_sim_time": True}]
            )
        ]
    )

    # ====================
    # BUILD LAUNCH DESCRIPTION
    # ====================
    
    ld.add_action(suppress_warnings)          # Suppress warnings
    ld.add_action(gz_sim)                     # 1. Gazebo Harmonic with PX4 world
    ld.add_action(unitree_robot_state_publisher)  # Robot description for spawning
    ld.add_action(spawn_drone)                # 2. Spawn drone model (GZ Harmonic)
    ld.add_action(spawn_unitree)              # 3. Spawn Unitree model (GZ Harmonic)
    ld.add_action(drone_launch)               # 4. Drone system (no GZ)
    ld.add_action(unitree_launch)             # 5. Unitree system (no GZ)
    ld.add_action(coordination_tf)            # 6. Mission coordination
    ld.add_action(sar_rviz)                   # 7. Unified visualization
    ld.add_action(status_monitor)             # 8. System status

    return ld