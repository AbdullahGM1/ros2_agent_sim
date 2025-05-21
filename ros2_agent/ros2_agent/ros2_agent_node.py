#!/usr/bin/env python3
"""Main entry point for the ISAR System Control ROS2 Agent."""

import rclpy
import threading
import asyncio
import time
import signal
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from rosa import ROSA

from .cli.interface import RichCLI
from .prompts.system_prompts import system_prompts
from .tools.robot_tools import RobotTools
from .llm.model import initialize_llm


class Ros2AgentNode(Node):
    """Main ROS2 node for ROSA Drone Agent."""
    
    def __init__(self):
        super().__init__('drone_agent_node')
        
        # ============================= PARAMETERS =============================
        self._declare_and_get_parameters()

        #To see the parameters in the terminal
        time.sleep(2)
        # ============================= INITIALIZE NODE =============================
        self._initialize_node()
        
        # ============================= SETUP AGENT =============================
        self.setup_agent()
        
        self.get_logger().info("Drone Agent is ready. Type a command:")
        time.sleep(2)

    def _declare_and_get_parameters(self):
        """Declare and get all ROS parameters."""
        # Drone parameters
        self.declare_parameter('odom_topic', '/drone/mavros/local_position/pose')
        self.declare_parameter('control_rate', 20.0)# Hz
        self.declare_parameter('llm_model', 'qwen3:8b')
        
        # Get parameters
        self.odom_topic = self.get_parameter('odom_topic').value
        self.control_rate = self.get_parameter('control_rate').value
        self.llm_model = self.get_parameter('llm_model').value
        
        # Log parameters
        self.get_logger().info(f"Odom topic: {self.odom_topic}")
        self.get_logger().info(f"Control rate: {self.control_rate}")
        self.get_logger().info(f"Using LLM model: {self.llm_model}")
    
    def _initialize_node(self):
        """Initialize node components."""
        # Create QoS profile for sensor data
        self.qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize subscribers
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            self.odom_topic,
            self.pose_callback,
            self.qos_sensor
        )
        
        # State variables
        self.current_pose = PoseStamped()
        
        # For graceful shutdown
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def pose_callback(self, msg):
        """Callback for drone position updates."""
        self.current_pose = msg
    
    def signal_handler(self, sig, frame):
        """Handle SIGINT (Ctrl+C) gracefully."""
        self.get_logger().info("Shutdown signal received, cleaning up...")
        self.running = False
        
        # Perform node shutdown
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
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
        
        # Cleanup ROS
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()