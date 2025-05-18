#!/usr/bin/env python3
"""Main entry point for the Multi-Robot ROS2 Agent for Search and Rescue operations."""

import rclpy
import threading
import asyncio
import time
import signal
import sys
import os
import math
import json
from datetime import datetime
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, BatteryState
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge
from rosa import ROSA

from .cli.interface import RichCLI
from .prompts.system_prompts import create_system_prompts
from .tools.robot_tools import RobotTools
from .llm.model import initialize_llm


class Ros2AgentNode(Node):
    """Main ROS2 node for ROSA Multi-Robot Agent."""
    
    def __init__(self):
        super().__init__('multi_robot_agent_node')
        
        # Mission-specific data
        self.mission_start_time = datetime.now()
        self.mission_name = "SAR_Mission"
        self.waypoints = []  # For marking points of interest
        
        # ============================= PARAMETERS =============================
        self._declare_and_get_parameters()
        
        # ============================= ROBOT REGISTRY =============================
        self._initialize_robot_registry()
        
        # ============================= INITIALIZE NODE =============================
        self._initialize_node()
        
        # ============================= SETUP AGENT =============================
        self.setup_agent()
        
        # Create logs directory if it doesn't exist
        os.makedirs('mission_logs', exist_ok=True)
        
        self.get_logger().info("Multi-Robot Agent is ready. Type a command:")
    
    def _declare_and_get_parameters(self):
        """Declare and get all ROS parameters."""
        # Agent parameters
        self.declare_parameter('mission_name', 'iSAR_Mission')
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('position_tolerance', 0.1)  # meters
        self.declare_parameter('llm_model', 'qwen3:8b')
        self.declare_parameter('robots_config_file', '')  # Path to robots config JSON
        
        # Get parameters
        self.mission_name = self.get_parameter('mission_name').value
        self.control_rate = self.get_parameter('control_rate').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.llm_model = self.get_parameter('llm_model').value
        self.robots_config_file = self.get_parameter('robots_config_file').value
        
        # Log parameters
        self.get_logger().info(f"Mission: {self.mission_name}")
        self.get_logger().info(f"Control parameters: control_rate={self.control_rate}, "
                              f"position_tolerance={self.position_tolerance}")
        self.get_logger().info(f"Using LLM model: {self.llm_model}")
    
    def _initialize_robot_registry(self):
        """Initialize the robot registry from configuration."""
        self.robots = {}
        
        # If a config file is provided, load robots from it
        if self.robots_config_file and os.path.exists(self.robots_config_file):
            try:
                with open(self.robots_config_file, 'r') as f:
                    config = json.load(f)
                    if 'robots' in config:
                        for robot in config['robots']:
                            if 'id' in robot:
                                self.robots[robot['id']] = robot
                self.get_logger().info(f"Loaded {len(self.robots)} robots from config file")
            except Exception as e:
                self.get_logger().error(f"Error loading robots config: {str(e)}")
        
        # Add default robots if none were loaded
        if not self.robots:
            self.robots = {
                "drone1": {
                    "type": "drone",
                    "namespace": "/drone1",
                    "initialized": False,
                    "active": False,
                    "topics": {
                        "pose": "/drone1/mavros/local_position/pose",
                        "battery": "/drone1/mavros/battery",
                        "camera": "/drone1/gimbal_camera",
                        "cmd_vel": "/drone1/mavros/setpoint_velocity/cmd_vel"
                    }
                }
            }
            self.get_logger().info("Using default robot configuration (drone1)")
        
        # Log the registered robots
        for robot_id, robot_info in self.robots.items():
            self.get_logger().info(f"Registered robot: {robot_id} (Type: {robot_info.get('type', 'unknown')})")
    
    def _initialize_node(self):
        """Initialize node components for all robots."""
        # Create QoS profiles
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize CV Bridge for image processing
        self.bridge = CvBridge()
        
        # Camera thread control
        self.camera_active = {}  # Dictionary to track active cameras by robot_id
        self.camera_threads = {}  # Dictionary to track camera threads by robot_id
        self.camera_lock = threading.Lock()
        
        # Initialize subscribers and publishers for each robot
        for robot_id, robot_info in self.robots.items():
            self._initialize_robot_interfaces(robot_id, robot_info)
        
        # For graceful shutdown
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def _initialize_robot_interfaces(self, robot_id, robot_info):
        """Initialize interfaces (subscribers, publishers) for a specific robot."""
        robot_type = robot_info.get('type', 'unknown')
        robot_ns = robot_info.get('namespace', f'/{robot_id}')
        
        # Set initial pose attribute
        setattr(self, f'{robot_id}_pose', PoseStamped())
        
        # Default topics based on robot type and namespace
        default_topics = {
            'drone': {
                'pose': f'{robot_ns}/mavros/local_position/pose',
                'battery': f'{robot_ns}/mavros/battery',
                'camera': f'{robot_ns}/gimbal_camera',
                'cmd_vel': f'{robot_ns}/mavros/setpoint_velocity/cmd_vel'
            },
            'wheeled': {
                'pose': f'{robot_ns}/odom',
                'battery': f'{robot_ns}/battery_state',
                'camera': f'{robot_ns}/camera/image_raw',
                'cmd_vel': f'{robot_ns}/cmd_vel'
            },
            'legged': {
                'pose': f'{robot_ns}/odom',
                'battery': f'{robot_ns}/battery_state',
                'camera': f'{robot_ns}/camera/image_raw',
                'cmd_vel': f'{robot_ns}/cmd_vel'
            }
        }
        
        # Use default topics for the robot type, if known
        if robot_type in default_topics and 'topics' not in robot_info:
            robot_info['topics'] = default_topics[robot_type]
        
        # Get topics from robot info, with fallbacks
        topics = robot_info.get('topics', {})
        pose_topic = topics.get('pose', f'{robot_ns}/odom')
        battery_topic = topics.get('battery', f'{robot_ns}/battery_state')
        camera_topic = topics.get('camera', f'{robot_ns}/camera/image_raw')
        cmd_vel_topic = topics.get('cmd_vel', f'{robot_ns}/cmd_vel')
        
        # Store topics in robot info for reference
        if 'topics' not in robot_info:
            robot_info['topics'] = {}
        robot_info['topics']['pose'] = pose_topic
        robot_info['topics']['battery'] = battery_topic
        robot_info['topics']['camera'] = camera_topic
        robot_info['topics']['cmd_vel'] = cmd_vel_topic
        
        # Create callback for position updates
        def pose_callback(msg):
            # Adjust to handle both PoseStamped and Odometry messages
            if hasattr(msg, 'pose'):
                # This is a PoseStamped message
                setattr(self, f'{robot_id}_pose', msg)
            elif hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                # This is an Odometry message, convert to PoseStamped
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose = msg.pose.pose
                setattr(self, f'{robot_id}_pose', pose_msg)
        
        # Create callback for battery updates
        def battery_callback(msg):
            setattr(self, f'{robot_id}_battery', msg)
        
        # Initialize subscribers based on message types
        if pose_topic.endswith('/odom'):
            # For robots that use Odometry messages
            setattr(self, f'{robot_id}_pose_sub', self.create_subscription(
                Odometry, pose_topic, pose_callback, self.qos_sensor
            ))
        else:
            # For robots that use PoseStamped messages (like drones)
            setattr(self, f'{robot_id}_pose_sub', self.create_subscription(
                PoseStamped, pose_topic, pose_callback, self.qos_sensor
            ))
        
        # Battery subscriber
        setattr(self, f'{robot_id}_battery_sub', self.create_subscription(
            BatteryState, battery_topic, battery_callback, self.qos_sensor
        ))
        
        # Command velocity publisher
        setattr(self, f'{robot_id}_cmd_vel_pub', self.create_publisher(
            Twist, cmd_vel_topic, self.qos_reliable
        ))
        
        # Initialize camera status
        self.camera_active[robot_id] = False
        self.camera_threads[robot_id] = None
        
        self.get_logger().info(f"Initialized interfaces for {robot_id}")
        self.get_logger().info(f"  Pose topic: {pose_topic}")
        self.get_logger().info(f"  Battery topic: {battery_topic}")
        self.get_logger().info(f"  Camera topic: {camera_topic}")
        self.get_logger().info(f"  Cmd_vel topic: {cmd_vel_topic}")
    
    def add_waypoint(self, waypoint_type, x, y, z, description="", robot_id=None):
        """Add a waypoint to the mission data."""
        waypoint = {
            "type": waypoint_type,
            "x": x,
            "y": y,
            "z": z,
            "description": description,
            "time": datetime.now().strftime("%H:%M:%S"),
            "robot_id": robot_id  # Associate waypoint with specific robot if provided
        }
        self.waypoints.append(waypoint)
        self.get_logger().info(f"Added {waypoint_type} waypoint at ({x}, {y}, {z}): {description}")
        return len(self.waypoints) - 1
    
    def signal_handler(self, sig, frame):
        """Handle SIGINT (Ctrl+C) gracefully."""
        self.get_logger().info("Shutdown signal received, stopping robots and cleaning up...")
        self.running = False
        
        # Stop all robots
        for robot_id in self.robots.keys():
            try:
                # Get the cmd_vel publisher for this robot
                cmd_vel_pub = getattr(self, f'{robot_id}_cmd_vel_pub', None)
                if cmd_vel_pub:
                    # Send stop command
                    stop_cmd = Twist()
                    cmd_vel_pub.publish(stop_cmd)
                    self.get_logger().info(f"Stopped {robot_id}")
            except Exception as e:
                self.get_logger().error(f"Error stopping {robot_id}: {str(e)}")
        
        # Stop all cameras
        with self.camera_lock:
            for robot_id in list(self.camera_active.keys()):
                if self.camera_active.get(robot_id, False):
                    self.camera_active[robot_id] = False
        
        # Join camera threads
        for robot_id, thread in self.camera_threads.items():
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
        
        # Save mission data
        self.save_mission_data()
        
        # Perform node shutdown
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    def save_mission_data(self):
        """Save mission data for analysis."""
        import json
        
        # Create data directory if it doesn't exist
        data_dir = os.path.join(os.getcwd(), 'mission_data')
        os.makedirs(data_dir, exist_ok=True)
        
        # Create timestamped file
        mission_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        data_file = os.path.join(data_dir, f'mission_{self.mission_name}_{mission_time}.json')
        
        # Prepare data
        mission_data = {
            "mission_name": self.mission_name,
            "start_time": self.mission_start_time.isoformat(),
            "end_time": datetime.now().isoformat(),
            "duration_seconds": (datetime.now() - self.mission_start_time).total_seconds(),
            "robots": list(self.robots.keys()),
            "waypoints": self.waypoints
        }
        
        # Save to file
        try:
            with open(data_file, 'w') as f:
                json.dump(mission_data, f, indent=2)
            self.get_logger().info(f"Mission data saved to {data_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save mission data: {str(e)}")
    
    def setup_agent(self):
        """Setup the ROSA agent with LLM and tools."""
        # Initialize LLM
        local_llm = initialize_llm(self.llm_model)
        
        # Create tools
        robot_tools = RobotTools(self)
        tools = robot_tools.create_tools()
        
        # Create prompts
        prompts = create_system_prompts()
        
        # Initialize ROSA
        self.agent = ROSA(
            ros_version=2,
            llm=local_llm,
            tools=tools,
            prompts=prompts
        )
        
        self.get_logger().info(f"ROSA Agent initialized with {len(tools)} tools")


def main(args=None):
    """Main function to run the Multi-Robot Agent Node with rich CLI."""
    rclpy.init(args=args)
    
    # Create the node
    node = Ros2AgentNode()
    
    # Create a separate thread for processing ROS callbacks
    def spin_thread():
        while node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    
    ros_thread = threading.Thread(target=spin_thread)
    ros_thread.daemon = True
    ros_thread.start()
    
    # Create and run the rich CLI
    cli = RichCLI(node)
    try:
        asyncio.run(cli.run())
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        # Cleanup
        node.running = False
        if ros_thread.is_alive():
            ros_thread.join(timeout=1.0)
        
        # Save mission data before shutdown
        node.save_mission_data()
        
        # Cleanup ROS
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()