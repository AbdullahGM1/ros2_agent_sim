#!/usr/bin/env python3
"""Main entry point for the Search and Rescue Multi-Robot ROS2 Agent."""

import rclpy
import threading
import asyncio
import time
import signal
import sys
import os
import logging
from datetime import datetime
from typing import Dict, List
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, NavSatFix, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
from rosa import ROSA

from .cli.interface import RichCLI
from .prompts.system_prompts import create_system_prompts
from .tools.robot_tools import RobotTools
from .llm.model import initialize_llm


class Ros2AgentNode(Node):
    """Main ROS2 node for ROSA SAR Multi-Robot Agent."""
    
    def __init__(self):
        super().__init__('sar_agent_node')
        
        # Setup logging for SAR operations
        self.setup_logging()
        
        # ============================= PARAMETERS =============================
        self._declare_and_get_parameters()
        
        # ============================= INITIALIZE NODE =============================
        self._initialize_node()
        
        # ============================= SETUP AGENT =============================
        self.setup_agent()
        
        # ============================= MISSION DATA =============================
        self.initialize_mission_data()
        
        self.get_logger().info("SAR Agent is ready. Type a command to control the robot fleet:")
    
    def setup_logging(self):
        """Set up enhanced logging for SAR operations."""
        # Create logs directory if it doesn't exist
        log_dir = os.path.join(os.getcwd(), 'sar_mission_logs')
        os.makedirs(log_dir, exist_ok=True)
        
        # Create timestamped log file for this mission
        mission_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = os.path.join(log_dir, f'sar_mission_{mission_time}.log')
        
        # Configure file handler
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        
        # Add to root logger
        root_logger = logging.getLogger()
        root_logger.addHandler(file_handler)
        root_logger.setLevel(logging.INFO)
        
        # Log mission start
        logging.info(f"SAR MISSION STARTED: {mission_time}")
        
    def _declare_and_get_parameters(self):
        """Declare and get all ROS parameters with support for multiple robots."""
        # Mission parameters
        self.declare_parameter('mission_name', 'SAR_Mission')
        self.declare_parameter('mission_area_bounds', [0.0, 0.0, 100.0, 100.0])  # [min_x, min_y, max_x, max_y]
        self.declare_parameter('base_station_coordinates', [0.0, 0.0, 0.0])  # [x, y, z]
        
        # Robot fleet parameters
        self.declare_parameter('robot_types', ['quadruped', 'drone'])
        self.declare_parameter('quadruped_ids', ['go2_1', 'go2_2'])
        self.declare_parameter('drone_ids', ['drone1', 'drone2'])
        
        # LLM parameters
        self.declare_parameter('llm_model', 'qwen3:8b')
        self.declare_parameter('llm_timeout', 30.0)
        
        # Control parameters - shared
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('position_tolerance', 0.05)  # meters
        self.declare_parameter('angle_tolerance', 0.05)  # radians
        
        # Quadruped parameters
        self.declare_parameter('quadruped_cmd_vel_topic_prefix', '/cmd_vel')
        self.declare_parameter('quadruped_odom_topic_prefix', '/odom/ground_truth')
        self.declare_parameter('quadruped_camera_topic_prefix', '/go2_rgb')
        self.declare_parameter('quadruped_linear_speed', 2.0)
        self.declare_parameter('quadruped_angular_speed', 0.8)
        
        # Drone parameters
        self.declare_parameter('drone_cmd_vel_topic_prefix', '/drone/cmd_vel')
        self.declare_parameter('drone_odom_topic_prefix', '/drone/odom')
        self.declare_parameter('drone_camera_topic_prefix', '/drone/camera/rgb')
        self.declare_parameter('drone_altitude_topic_prefix', '/drone/altitude')
        self.declare_parameter('drone_gps_topic_prefix', '/drone/gps')
        self.declare_parameter('drone_battery_topic_prefix', '/drone/battery')
        self.declare_parameter('drone_takeoff_topic_prefix', '/drone/takeoff')
        self.declare_parameter('drone_land_topic_prefix', '/drone/land')
        self.declare_parameter('drone_linear_speed', 1.0)
        self.declare_parameter('drone_angular_speed', 0.5)
        self.declare_parameter('drone_max_altitude', 50.0)  # meters
        
        # Get parameters
        self.mission_name = self.get_parameter('mission_name').value
        self.mission_area_bounds = self.get_parameter('mission_area_bounds').value
        self.base_station_coordinates = self.get_parameter('base_station_coordinates').value
        
        self.robot_types = self.get_parameter('robot_types').value
        self.quadruped_ids = self.get_parameter('quadruped_ids').value
        self.drone_ids = self.get_parameter('drone_ids').value
        
        self.llm_model = self.get_parameter('llm_model').value
        self.llm_timeout = self.get_parameter('llm_timeout').value
        
        self.control_rate = self.get_parameter('control_rate').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        
        # Quadruped parameters
        self.quadruped_cmd_vel_topic_prefix = self.get_parameter('quadruped_cmd_vel_topic_prefix').value
        self.quadruped_odom_topic_prefix = self.get_parameter('quadruped_odom_topic_prefix').value
        self.quadruped_camera_topic_prefix = self.get_parameter('quadruped_camera_topic_prefix').value
        self.quadruped_linear_speed = self.get_parameter('quadruped_linear_speed').value
        self.quadruped_angular_speed = self.get_parameter('quadruped_angular_speed').value
        
        # Drone parameters
        self.drone_cmd_vel_topic_prefix = self.get_parameter('drone_cmd_vel_topic_prefix').value
        self.drone_odom_topic_prefix = self.get_parameter('drone_odom_topic_prefix').value
        self.drone_camera_topic_prefix = self.get_parameter('drone_camera_topic_prefix').value
        self.drone_altitude_topic_prefix = self.get_parameter('drone_altitude_topic_prefix').value
        self.drone_gps_topic_prefix = self.get_parameter('drone_gps_topic_prefix').value
        self.drone_battery_topic_prefix = self.get_parameter('drone_battery_topic_prefix').value
        self.drone_takeoff_topic_prefix = self.get_parameter('drone_takeoff_topic_prefix').value
        self.drone_land_topic_prefix = self.get_parameter('drone_land_topic_prefix').value
        self.drone_linear_speed = self.get_parameter('drone_linear_speed').value
        self.drone_angular_speed = self.get_parameter('drone_angular_speed').value
        self.drone_max_altitude = self.get_parameter('drone_max_altitude').value
        
        # Log critical parameters
        self.get_logger().info(f"SAR Mission Name: {self.mission_name}")
        self.get_logger().info(f"Mission Area Bounds: {self.mission_area_bounds}")
        self.get_logger().info(f"Robot Fleet: Quadrupeds {self.quadruped_ids}, Drones {self.drone_ids}")
        self.get_logger().info(f"Using LLM model: {self.llm_model}")
    
    def _initialize_node(self):
        """Initialize node components with multi-robot support."""
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
        
        # Initialize dictionaries for robot publishers and subscribers
        self.publishers = {}
        self.subscribers = {}
        self.robot_status = {}
        self.robot_poses = {}
        
        # Initialize CV Bridge for image processing
        self.bridge = CvBridge()
        
        # Set up publishers for each robot
        self._setup_robot_publishers()
        
        # Camera thread control
        self.camera_active = {}
        self.camera_threads = {}
        self.camera_lock = threading.Lock()
        
        # For graceful shutdown
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def _setup_robot_publishers(self):
        """Set up publishers for all robots in the fleet."""
        # Setup quadruped publishers
        for robot_id in self.quadruped_ids:
            # Command velocity publisher
            cmd_vel_topic = f"{self.quadruped_cmd_vel_topic_prefix}/{robot_id}"
            self.publishers[f"{robot_id}_cmd_vel"] = self.create_publisher(
                Twist, cmd_vel_topic, self.qos_reliable
            )
            self.get_logger().info(f"Created publisher for {robot_id} on {cmd_vel_topic}")
            
            # Initialize camera active status
            self.camera_active[robot_id] = False
            
            # Initialize robot status
            self.robot_status[robot_id] = {
                "type": "quadruped",
                "active": True,
                "battery": 100.0,
                "last_update": time.time()
            }
        
        # Setup drone publishers
        for robot_id in self.drone_ids:
            # Command velocity publisher
            cmd_vel_topic = f"{self.drone_cmd_vel_topic_prefix}/{robot_id}"
            self.publishers[f"{robot_id}_cmd_vel"] = self.create_publisher(
                Twist, cmd_vel_topic, self.qos_reliable
            )
            
            # Takeoff publisher
            takeoff_topic = f"{self.drone_takeoff_topic_prefix}/{robot_id}"
            self.publishers[f"{robot_id}_takeoff"] = self.create_publisher(
                String, takeoff_topic, self.qos_reliable
            )
            
            # Land publisher
            land_topic = f"{self.drone_land_topic_prefix}/{robot_id}"
            self.publishers[f"{robot_id}_land"] = self.create_publisher(
                String, land_topic, self.qos_reliable
            )
            
            # Initialize camera active status
            self.camera_active[robot_id] = False
            
            # Initialize robot status
            self.robot_status[robot_id] = {
                "type": "drone",
                "active": True,
                "battery": 100.0,
                "altitude": 0.0,
                "last_update": time.time()
            }
            
            self.get_logger().info(f"Created publishers for drone {robot_id}")
    
    def initialize_mission_data(self):
        """Initialize SAR mission-specific data structures."""
        self.mission_start_time = datetime.now()
        self.waypoints = []  # For marking survivor locations, hazards, etc.
        self.search_areas = []  # For tracking searched areas
        self.search_patterns = {}  # For active search patterns
        self.survivor_detections = []  # For potential survivor sightings
        self.mission_log = []  # For mission-critical events
        
        # Log mission initialization
        mission_log_entry = {
            "timestamp": datetime.now().isoformat(),
            "event_type": "mission_start",
            "details": {
                "mission_name": self.mission_name,
                "area_bounds": self.mission_area_bounds,
                "robots": {
                    "quadrupeds": self.quadruped_ids,
                    "drones": self.drone_ids
                }
            }
        }
        self.mission_log.append(mission_log_entry)
        
        # Log to ROS and file
        self.get_logger().info(f"SAR Mission '{self.mission_name}' initialized with {len(self.quadruped_ids)} quadrupeds and {len(self.drone_ids)} drones")
        logging.info(f"MISSION INITIALIZED: {self.mission_name} with bounds {self.mission_area_bounds}")
    
    def signal_handler(self, sig, frame):
        """Handle SIGINT (Ctrl+C) gracefully with multi-robot support."""
        self.get_logger().info("Shutdown signal received, recalling all robots and cleaning up...")
        self.running = False
        
        # Log mission end
        mission_end_entry = {
            "timestamp": datetime.now().isoformat(),
            "event_type": "mission_end",
            "details": {
                "mission_duration": (datetime.now() - self.mission_start_time).total_seconds(),
                "waypoints_recorded": len(self.waypoints),
                "areas_searched": len(self.search_areas),
                "survivor_detections": len(self.survivor_detections)
            }
        }
        self.mission_log.append(mission_end_entry)
        
        # Stop all robot movements
        self.emergency_stop_all_robots()
        
        # Stop all camera feeds
        with self.camera_lock:
            for robot_id in self.camera_active:
                self.camera_active[robot_id] = False
        
        # Join all camera threads
        for robot_id, thread in self.camera_threads.items():
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
        
        # Save mission data
        self.save_mission_data()
        
        # Perform node shutdown
        self.destroy_node()
        rclpy.shutdown()
        
        # Final logs
        logging.info("MISSION ENDED: Shutdown complete")
        sys.exit(0)
    
    def emergency_stop_all_robots(self):
        """Emergency stop all robots in the fleet."""
        twist = Twist()
        
        # Stop all quadrupeds
        for robot_id in self.quadruped_ids:
            publisher = self.publishers.get(f"{robot_id}_cmd_vel")
            if publisher:
                for _ in range(5):  # Send multiple times for reliability
                    publisher.publish(twist)
                    time.sleep(0.01)
                self.get_logger().warn(f"Emergency stop sent to {robot_id}")
        
        # Stop and land all drones
        for robot_id in self.drone_ids:
            # Stop movement
            cmd_vel_publisher = self.publishers.get(f"{robot_id}_cmd_vel")
            if cmd_vel_publisher:
                for _ in range(5):
                    cmd_vel_publisher.publish(twist)
                    time.sleep(0.01)
            
            # Send land command
            land_publisher = self.publishers.get(f"{robot_id}_land")
            if land_publisher:
                land_msg = String()
                land_msg.data = "LAND"
                land_publisher.publish(land_msg)
                self.get_logger().warn(f"Emergency landing initiated for {robot_id}")
        
        # Log the emergency stop
        self.get_logger().error("EMERGENCY STOP: All robots halted")
        logging.warning("EMERGENCY STOP activated for all robots")
    
    def save_mission_data(self):
        """Save all mission data for later analysis."""
        import json
        
        # Create data directory if it doesn't exist
        data_dir = os.path.join(os.getcwd(), 'sar_mission_data')
        os.makedirs(data_dir, exist_ok=True)
        
        # Create timestamped data file
        mission_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        data_file = os.path.join(data_dir, f'mission_{self.mission_name}_{mission_time}.json')
        
        # Prepare data
        mission_data = {
            "mission_name": self.mission_name,
            "start_time": self.mission_start_time.isoformat(),
            "end_time": datetime.now().isoformat(),
            "duration_seconds": (datetime.now() - self.mission_start_time).total_seconds(),
            "area_bounds": self.mission_area_bounds,
            "robots": {
                "quadrupeds": self.quadruped_ids,
                "drones": self.drone_ids
            },
            "waypoints": self.waypoints,
            "search_areas": self.search_areas,
            "survivor_detections": self.survivor_detections,
            "mission_log": self.mission_log
        }
        
        # Save to file
        try:
            with open(data_file, 'w') as f:
                json.dump(mission_data, f, indent=2)
            self.get_logger().info(f"Mission data saved to {data_file}")
            logging.info(f"Mission data saved: {data_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save mission data: {str(e)}")
            logging.error(f"Failed to save mission data: {str(e)}")
    
    def setup_agent(self):
        """Setup the ROSA agent with LLM and tools for multi-robot control."""
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
        
        self.get_logger().info(f"SAR ROSA Agent initialized with {len(tools)} tools")


def main(args=None):
    """Main function to run the SAR Multi-Robot Agent with rich CLI."""
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