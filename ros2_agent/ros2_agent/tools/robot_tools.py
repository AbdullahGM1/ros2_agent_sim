#!/usr/bin/env python3
"""
Robot tools for the ROSA Agent.
This module contains tools for drone monitoring and control.
"""

import time
from langchain.agents import tool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, Twist


class RobotTools:
    """Collection of tools for robot monitoring and control."""
    
    def __init__(self, node):
        """Initialize with a ROS node."""
        self.node = node
        
        # Store the latest drone state
        self.latest_pose = None
        self.latest_twist = None
        
        # Initialize subscribers for drone state
        self._init_subscribers()
    
    def _init_subscribers(self):
        """Initialize all subscribers for monitoring the drone."""
        # Subscribe to odometry for pose and velocity information
        self.node.create_subscription(
            Odometry,
            '/drone/mavros/local_position/odom',
            self._odom_callback,
            10  # QoS profile depth
        )
        self.node.get_logger().info("Subscribed to odometry topic")
    
    def _odom_callback(self, msg):
        """Process incoming odometry messages."""
        self.latest_pose = msg.pose.pose
        self.latest_twist = msg.twist.twist
    
    def create_tools(self):
        """Create and return all robot tools."""
        
        # Reference to the node for use in closures
        node = self.node
        
        # Reference to self for use in closures
        robot_tools = self
        
        @tool
        def get_drone_pose() -> str:
            """
            Get the current position and orientation of the drone.
            
            Returns:
                str: Formatted string with position (x, y, z) and orientation (quaternion)
            """
            if robot_tools.latest_pose is None:
                return "Drone pose data not yet available. Waiting for odometry messages..."
            
            # Extract position
            pos = robot_tools.latest_pose.position
            
            # Extract orientation
            orient = robot_tools.latest_pose.orientation
            
            # Format the response
            return (
                f"Drone Pose:\n"
                f"Position:\n"
                f"  x: {pos.x:.2f} meters\n"
                f"  y: {pos.y:.2f} meters\n"
                f"  z: {pos.z:.2f} meters (altitude)\n"
                f"Orientation (quaternion):\n"
                f"  x: {orient.x:.2f}\n"
                f"  y: {orient.y:.2f}\n"
                f"  z: {orient.z:.2f}\n"
                f"  w: {orient.w:.2f}"
            )
        
        @tool
        def get_drone_velocity() -> str:
            """
            Get the current linear and angular velocity of the drone.
            
            Returns:
                str: Formatted string with linear and angular velocity components
            """
            if robot_tools.latest_twist is None:
                return "Drone velocity data not yet available. Waiting for odometry messages..."
            
            # Extract linear velocity
            lin = robot_tools.latest_twist.linear
            
            # Extract angular velocity
            ang = robot_tools.latest_twist.angular
            
            # Format the response
            return (
                f"Drone Velocity:\n"
                f"Linear (m/s):\n"
                f"  x: {lin.x:.2f}\n"
                f"  y: {lin.y:.2f}\n"
                f"  z: {lin.z:.2f} (vertical)\n"
                f"Angular (rad/s):\n"
                f"  x: {ang.x:.2f} (roll rate)\n"
                f"  y: {ang.y:.2f} (pitch rate)\n"
                f"  z: {ang.z:.2f} (yaw rate)"
            )
        
        # Return all tools
        return [
            get_drone_pose,
            get_drone_velocity
        ]