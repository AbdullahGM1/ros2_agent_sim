#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from math import radians
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    ns = 'drone'

    # Node for Drone 1
    world = {'gz_world': 'default'}
    model_name = {'gz_model_name': 'x500_lidar_camera'}
    autostart_id = {'px4_autostart_id': '4021'}
    instance_id = {'instance_id': '1'}
    xpos = {'xpos': '0.0'}
    ypos = {'ypos': '0.0'}
    zpos = {'zpos': '0.1'}
    headless = {'headless': '0'}

    # Unitree Go2 Configuration
    go2_ns = 'go2'
    
    # Get Unitree Go2 package paths
    unitree_go2_description = get_package_share_directory('unitree_go2_description')
    go2_urdf_path = os.path.join(unitree_go2_description, 'urdf', 'unitree_go2_robot.xacro')
    
    # Go2 spawn position (side by side with drone)
    go2_x = '5.0'
    go2_y = '0.0' 
    go2_z = '0.5'  # Slightly elevated to avoid ground collision

    # PX4 SITL + Spawn x500_lidar_camera
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drone_sim'),
                'launch',  
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_ns': ns,
            'headless': headless['headless'],
            'gz_world': world['gz_world'],
            'gz_model_name': model_name['gz_model_name'],
            'px4_autostart_id': autostart_id['px4_autostart_id'],
            'instance_id': instance_id['instance_id'],
            'xpos': xpos['xpos'],
            'ypos': ypos['ypos'],
            'zpos': zpos['zpos']
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
            'mavros_namespace' :ns+'/mavros',
            'tgt_system': '2',
            'fcu_url': 'udp://:14541@127.0.0.1:14558',
            'pluginlists_yaml': plugins_file_path,
            'config_yaml': config_file_path,
            'base_link_frame': 'drone/base_link',
            'odom_frame': 'drone/odom',
            'map_frame': 'map',
            'use_sim_time' : 'True'

        }.items()
    )

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

    odom_frame = 'odom'
    base_link_frame = 'base_link'

    # Static TF map/world -> local_pose_ENU
    map_frame = 'map'
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_' + ns + '_tf_node',
        executable='static_transform_publisher',
        arguments=[str(xpos['xpos']), str(ypos['ypos']), str(zpos['zpos']), '0.0', '0', '0', map_frame, ns + '/' + odom_frame],
    )

    # Static TF base_link -> Gimbal_Cam
    cam_x = 0.1
    cam_y = 0
    cam_z = 0.13
    cam_roll = 1.5708
    cam_pitch = 0
    cam_yaw = 1.5708
    cam_tf_node = Node(
        package='tf2_ros',
        name=ns + '_base2gimbal_camera_tf_node',
        executable='static_transform_publisher',
        arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns + '/' + base_link_frame, ns + '/gimbal_camera'],
    )

    # Static TF base_link -> Lidar
    lidar_x = 0
    lidar_y = 0
    lidar_z = 0.295
    lidar_roll = 0
    lidar_pitch = 0
    lidar_yaw = 0
    lidar_tf_node = Node(
        package='tf2_ros',
        name=ns + '_base2lidar_tf_node',
        executable='static_transform_publisher',
        arguments=[str(lidar_x), str(lidar_y), str(lidar_z), str(lidar_yaw), str(lidar_pitch), str(lidar_roll), ns + '/' + base_link_frame, 'x500_lidar_camera_1/lidar_link/gpu_lidar'],
    )

    # Connect drone/base_link to base_link
    drone_base_to_base_link_tf_node = Node(
        package='tf2_ros',
        name=ns + '_drone_base_to_base_link_tf_node', 
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', ns + '/base_link', 'base_link'],
    )

    # Base link to base_link_frd (ENU to NED conversion)
    base_link_to_frd_tf_node = Node(
        package='tf2_ros',
        name='base_link_to_frd_tf_node',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '1.5708', '0', '3.1415', 'base_link', 'base_link_frd'],
    )

    # Connect map to odom
    map_to_odom_tf_node = Node(
        package='tf2_ros',
        name='map_to_odom_tf_node',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # Connect odom to odom_ned
    odom_to_odom_ned_tf_node = Node(
        package='tf2_ros',
        name='odom_to_odom_ned_tf_node',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '1.5708', '0', '3.1415', 'odom', 'odom_ned'],
    )

    # ROS-GZ Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        name='ros_bridge_node',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/gimbal/cmd_yaw@std_msgs/msg/Float64]ignition.msgs.Double',
            '/gimbal/cmd_roll@std_msgs/msg/Float64]ignition.msgs.Double',
            '/gimbal/cmd_pitch@std_msgs/msg/Float64]ignition.msgs.Double',
            '/imu_gimbal@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '--ros-args', '-r', '/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/image:=' + ns + '/gimbal_camera',
            '--ros-args', '-r', '/world/default/model/x500_lidar_camera_1/link/pitch_link/sensor/camera/camera_info:=' + ns + '/gimbal_camera_info',
            '--ros-args', '-r', '/gimbal/cmd_yaw:=' + ns + '/gimbal/cmd_yaw',
            '--ros-args', '-r', '/gimbal/cmd_roll:=' + ns + '/gimbal/cmd_roll',
            '--ros-args', '-r', '/gimbal/cmd_pitch:=' + ns + '/gimbal/cmd_pitch',
            '--ros-args', '-r', '/imu_gimbal:=' + ns + '/imu_gimbal',
            '--ros-args', '-r', '/scan:=' + ns + '/scan',
            '--ros-args', '-r', '/scan/points:=' + ns + '/scan/points'
        ],
    )

    # Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='sim_rviz2',
        arguments=['-d', '/home/user/shared_volume/ros2_ws/src/ros2_agent_sim/drone_sim/rviz/drone_sim.rviz']
    )

    # Add all nodes and launches to the launch description
    ld.add_action(gz_launch)
    ld.add_action(map2pose_tf_node)
    ld.add_action(cam_tf_node)
    ld.add_action(lidar_tf_node)
    ld.add_action(drone_base_to_base_link_tf_node)
    ld.add_action(base_link_to_frd_tf_node)
    ld.add_action(map_to_odom_tf_node)
    ld.add_action(odom_to_odom_ned_tf_node)
    ld.add_action(ros_gz_bridge)
    ld.add_action(mavros_launch)
    # Go2 nodes to launch description
    ld.add_action(go2_robot_description)
    ld.add_action(go2_spawn)
    ld.add_action(rviz_node)

    return ld