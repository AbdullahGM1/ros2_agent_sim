#!/usr/bin/env python3
"""Main entry point for the ISAR System Control ROS2 Agent."""

import rclpy
import threading
import asyncio
import time
import signal
import sys
import os
import math
from datetime import datetime
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from rosa import ROSA

from .cli.interface import RichCLI
from .prompts.system_prompts import system_prompts
from .tools.robot_tools import RobotTools
from .llm.model import initialize_llm


class Ros2AgentNode(Node):
    """Main ROS2 node for ROSA Drone Agent."""
    
    def __init__(self):
        super().__init__('drone_agent_node')
        
        # Mission-specific data
        self.mission_start_time = datetime.now()
        self.mission_name = "Drone_Mission"
        self.waypoints = []  # For marking points of interest
        
        # ============================= PARAMETERS =============================
        self._declare_and_get_parameters()
        
        # ============================= INITIALIZE NODE =============================
        self._initialize_node()
        
        # ============================= SETUP AGENT =============================
        self.setup_agent()
        
        # Create logs directory if it doesn't exist
        os.makedirs('mission_logs', exist_ok=True)
        
        self.get_logger().info("Drone Agent is ready. Type a command:")
    
    def _declare_and_get_parameters(self):
        """Declare and get all ROS parameters."""
        # Drone parameters
        self.declare_parameter('namespace', '/drone/mavros')
        self.declare_parameter('camera_topic', '/drone/gimbal_camera')
        self.declare_parameter('odom_topic', '/drone/mavros/local_position/pose')
        self.declare_parameter('cmd_vel_topic', '/drone/mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('flight_speed', 2.0)  # m/s
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('position_tolerance', 0.2)  # meters
        self.declare_parameter('llm_model', 'qwen3:8b')
        
        # Get parameters
        self.namespace = self.get_parameter('namespace').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.flight_speed = self.get_parameter('flight_speed').value
        self.control_rate = self.get_parameter('control_rate').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.llm_model = self.get_parameter('llm_model').value
        
        # Log parameters
        self.get_logger().info(f"Drone Mission: {self.mission_name}")
        self.get_logger().info(f"Namespace: {self.namespace}")
        self.get_logger().info(f"Topics: odom={self.odom_topic}, cmd_vel={self.cmd_vel_topic}")
        self.get_logger().info(f"Camera topic: {self.camera_topic}")
        self.get_logger().info(f"Control parameters: flight_speed={self.flight_speed}, "
                             f"control_rate={self.control_rate}, "
                             f"position_tolerance={self.position_tolerance}")
        self.get_logger().info(f"Using LLM model: {self.llm_model}")
    
    def _initialize_node(self):
        """Initialize node components."""
        # Create QoS profile for reliable communication
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
        
        # Initialize publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            self.qos_reliable
        )
        
        # Initialize subscribers
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            self.odom_topic,
            self.pose_callback,
            self.qos_sensor
        )
        
        # Initialize CV Bridge for image processing
        self.bridge = CvBridge()
        
        # State variables
        self.current_pose = PoseStamped()
        
        # Camera thread control
        self.camera_active = False
        self.camera_thread = None
        self.camera_lock = threading.Lock()
        
        # For graceful shutdown
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def pose_callback(self, msg):
        """Callback for drone position updates."""
        self.current_pose = msg
    
    def add_waypoint(self, waypoint_type, x, y, z, description=""):
        """Add a waypoint to the mission data."""
        waypoint = {
            "type": waypoint_type,
            "x": x,
            "y": y,
            "z": z,
            "description": description,
            "time": datetime.now().strftime("%H:%M:%S")
        }
        self.waypoints.append(waypoint)
        self.get_logger().info(f"Added {waypoint_type} waypoint at ({x}, {y}, {z}): {description}")
        return len(self.waypoints) - 1
    
    def signal_handler(self, sig, frame):
        """Handle SIGINT (Ctrl+C) gracefully."""
        self.get_logger().info("Shutdown signal received, stopping drone and cleaning up...")
        self.running = False
        
        # Stop the drone
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)
        
        # Stop camera if active
        with self.camera_lock:
            if self.camera_active:
                self.camera_active = False
        
        if self.camera_thread is not None and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=1.0)
        
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
        prompts = system_prompts()
        
        # Initialize ROSA
        self.agent = ROSA(
            ros_version=2,
            llm=local_llm,
            tools=tools,
            prompts=prompts
        )
        
        self.get_logger().info(f"ROSA Agent initialized with {len(tools)} tools")


def main(args=None):
    """Main function to run the Drone Agent Node with rich CLI."""
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