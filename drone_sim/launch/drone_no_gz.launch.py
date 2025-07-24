#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    ns = 'drone'

    # Node parameters for Drone
    autostart_id = '4021'
    instance_id = '1'
    xpos = '0.0'
    ypos = '0.0'
    zpos = '0.1'

    # Get PX4_DIR environment variable
    PX4_DIR = os.getenv('PX4_DIR')
    if PX4_DIR is None:
        PX4_DIR = '/home/user/shared_volume/PX4-Autopilot'  # Default path

    # PX4 SITL process (NO Gazebo - just PX4)
    px4_sitl_process = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'cd {PX4_DIR} && ' +
            f'PX4_SYS_AUTOSTART={autostart_id} ' +
            f'PX4_GZ_MODEL=x500_lidar_camera ' +
            f'PX4_MICRODDS_NS={ns} ' +
            f'PX4_GZ_MODEL_POSE=\'{xpos},{ypos},{zpos}\' ' +
            f'./build/px4_sitl_default/bin/px4 -i {instance_id}'
        ],
        shell=False,
        output='screen'
    )

    # XRCE-DDS Agent 
    xrce_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drone_sim'),
                'launch',  
                'xrce_agent.launch.py'
            ])
        ]),
        launch_arguments={
            'port': '8888'  
        }.items()
    )

    # MAVROS 
    file_name = 'drone_px4_pluginlists.yaml'
    package_share_directory = get_package_share_directory('drone_sim')
    plugins_file_path = os.path.join(package_share_directory,'mavros', file_name)
    file_name = 'drone_px4_config.yaml'
    config_file_path = os.path.join(package_share_directory,'mavros', file_name)
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drone_sim'),
                'launch',  
                'mavros.launch.py'
            ])
        ]),
        launch_arguments={
            'mavros_namespace': ns+'/mavros',
            'tgt_system': '2',  
            'fcu_url': 'udp://:14541@127.0.0.1:14557',  
            'pluginlists_yaml': plugins_file_path,
            'config_yaml': config_file_path,
            'base_link_frame': 'drone/base_link',
            'odom_frame': 'drone/odom',
            'map_frame': 'map',
            'use_sim_time': 'True'
        }.items()
    )

    # TF transformations
    odom_frame = 'odom'
    base_link_frame = 'base_link'
    map_frame = 'map'

    # Static TF map/world -> local_pose_ENU
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_' + ns + '_tf_node',
        executable='static_transform_publisher',
        arguments=[str(xpos), str(ypos), str(zpos), '0.0', '0', '0', map_frame, ns + '/' + odom_frame],
        parameters=[{'use_sim_time': True}]
    )

    # Static TF base_link -> Gimbal_Cam
    cam_tf_node = Node(
        package='tf2_ros',
        name=ns + '_base2gimbal_camera_tf_node',
        executable='static_transform_publisher',
        arguments=['0.1', '0', '0.13', '1.5708', '0', '1.5708', ns + '/' + base_link_frame, ns + '/gimbal_camera'],
        parameters=[{'use_sim_time': True}]
    )

    # Static TF base_link -> Lidar
    lidar_tf_node = Node(
        package='tf2_ros',
        name=ns + '_base2lidar_tf_node',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.295', '0', '0', '0', ns + '/' + base_link_frame, 'x500_lidar_camera_1/lidar_link/gpu_lidar'],
        parameters=[{'use_sim_time': True}]
    )

    # Connect drone/base_link to base_link
    drone_base_to_base_link_tf_node = Node(
        package='tf2_ros',
        name=ns + '_drone_base_to_base_link_tf_node', 
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', ns + '/base_link', 'base_link'],
        parameters=[{'use_sim_time': True}]
    )

    # Base link to base_link_frd (ENU to NED conversion)
    base_link_to_frd_tf_node = Node(
        package='tf2_ros',
        name='base_link_to_frd_tf_node',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '1.5708', '0', '3.1415', 'base_link', 'base_link_frd'],
        parameters=[{'use_sim_time': True}]
    )

    # Connect map to odom
    map_to_odom_tf_node = Node(
        package='tf2_ros',
        name='map_to_odom_tf_node',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    # Connect odom to odom_ned
    odom_to_odom_ned_tf_node = Node(
        package='tf2_ros',
        name='odom_to_odom_ned_tf_node',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '1.5708', '0', '3.1415', 'odom', 'odom_ned'],
        parameters=[{'use_sim_time': True}]
    )

    # === SEPARATE BRIDGES FOR STABILITY ===

    # Clock Bridge (CRITICAL - Start First)
    clock_bridge = Node(
        package='ros_gz_bridge',
        name='drone_clock_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[
            {'use_sim_time': True},
            {'qos_overrides./clock.reliability': 'reliable'},
            {'qos_overrides./clock.durability': 'transient_local'},
        ]
    )

    # Camera Bridge (Delayed for Stability)
    camera_bridge = TimerAction(
        period=3.0,  # Wait for clock bridge
        actions=[
            Node(
                package='ros_gz_bridge',
                name='drone_camera_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    
                    # CAMERA REMAPPING
                    '--ros-args', '-r', '/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/image:=' + ns + '/gimbal_camera',
                    '--ros-args', '-r', '/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/camera_info:=' + ns + '/gimbal_camera_info',
                ],
                parameters=[
                    {'use_sim_time': True},
                    {'qos_overrides./drone/gimbal_camera.reliability': 'best_effort'},
                    {'qos_overrides./drone/gimbal_camera.durability': 'volatile'},
                    {'qos_overrides./drone/gimbal_camera.history': 'keep_last'},
                    {'qos_overrides./drone/gimbal_camera.depth': '1'},
                ]
            )
        ]
    )

    # LiDAR Bridge (Delayed)
    lidar_bridge = TimerAction(
        period=5.0,  # Wait for camera bridge
        actions=[
            Node(
                package='ros_gz_bridge',
                name='drone_lidar_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                    
                    # LIDAR REMAPPING
                    '--ros-args', '-r', '/scan:=' + ns + '/scan',
                    '--ros-args', '-r', '/scan/points:=' + ns + '/scan/points',
                ],
                parameters=[
                    {'use_sim_time': True},
                    {'qos_overrides./drone/scan.reliability': 'best_effort'},
                    {'qos_overrides./drone/scan/points.reliability': 'best_effort'},
                    {'qos_overrides./drone/scan.durability': 'volatile'},
                    {'qos_overrides./drone/scan/points.durability': 'volatile'},
                ]
            )
        ]
    )

    # Sensor Bridge (IMU, GPS, etc.) - Delayed
    sensor_bridge = TimerAction(
        period=10.0,  # Wait for LiDAR bridge
        actions=[
            Node(
                package='ros_gz_bridge',
                name='drone_sensor_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/world/default/model/x500_lidar_camera_1/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/world/default/model/x500_lidar_camera_1/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure',
                    '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                    '/imu_gimbal@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    
                    # SENSOR REMAPPING
                    '--ros-args', '-r', '/world/default/model/x500_lidar_camera_1/link/base_link/sensor/imu_sensor/imu:=' + ns + '/imu',
                    '--ros-args', '-r', '/world/default/model/x500_lidar_camera_1/link/base_link/sensor/air_pressure_sensor/air_pressure:=' + ns + '/air_pressure',
                    '--ros-args', '-r', '/navsat:=' + ns + '/gps',
                    '--ros-args', '-r', '/imu_gimbal:=' + ns + '/imu_gimbal',
                ],
                parameters=[
                    {'use_sim_time': True},
                    {'qos_overrides./drone/imu.reliability': 'reliable'},
                    {'qos_overrides./drone/imu.durability': 'volatile'},
                    {'qos_overrides./drone/imu.history': 'keep_last'},
                    {'qos_overrides./drone/imu.depth': '1'},
                    {'qos_overrides./drone/gps.reliability': 'best_effort'},
                    {'qos_overrides./drone/air_pressure.reliability': 'best_effort'},
                ]
            )
        ]
    )

    # Command Bridge (Gimbal Commands) - Delayed
    command_bridge = TimerAction(
        period=9.0,  # Wait for sensor bridge
        actions=[
            Node(
                package='ros_gz_bridge',
                name='drone_command_bridge',
                executable='parameter_bridge',
                arguments=[
                    '/gimbal/cmd_yaw@std_msgs/msg/Float64]gz.msgs.Double',
                    '/gimbal/cmd_roll@std_msgs/msg/Float64]gz.msgs.Double',
                    '/gimbal/cmd_pitch@std_msgs/msg/Float64]gz.msgs.Double',
                    
                    # COMMAND REMAPPING
                    '--ros-args', '-r', '/gimbal/cmd_yaw:=' + ns + '/gimbal/cmd_yaw',
                    '--ros-args', '-r', '/gimbal/cmd_roll:=' + ns + '/gimbal/cmd_roll',
                    '--ros-args', '-r', '/gimbal/cmd_pitch:=' + ns + '/gimbal/cmd_pitch',
                ],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # Add all nodes and launches to the launch description
    ld.add_action(px4_sitl_process)           # PX4 SITL
    ld.add_action(map2pose_tf_node)           # TF transforms
    ld.add_action(cam_tf_node)
    ld.add_action(lidar_tf_node)
    ld.add_action(drone_base_to_base_link_tf_node)
    ld.add_action(base_link_to_frd_tf_node)
    ld.add_action(map_to_odom_tf_node)
    ld.add_action(odom_to_odom_ned_tf_node)
    
    # SEPARATE BRIDGES (STAGGERED TIMING)
    ld.add_action(clock_bridge)               # 1. Clock first
    ld.add_action(camera_bridge)              # 2. Camera (3s delay)
    ld.add_action(lidar_bridge)               # 3. LiDAR (5s delay)
    ld.add_action(sensor_bridge)              # 4. Sensors (7s delay)
    ld.add_action(command_bridge)             # 5. Commands (9s delay)
    
    ld.add_action(mavros_launch)              # MAVROS
    ld.add_action(xrce_agent_launch)          # XRCE agent

    return ld