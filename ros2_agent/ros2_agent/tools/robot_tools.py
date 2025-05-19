#!/usr/bin/env python3
"""
Robot tools for the ROSA Agent.
This module contains tools for drone control, limited to takeoff only.
"""

import time
import threading
from geometry_msgs.msg import PoseStamped
from langchain.agents import tool
from mavros_msgs.srv import CommandBool, SetMode


class RobotTools:
    """Collection of tools for robot control."""
    
    def __init__(self, node):
        self.node = node
    
    @tool
    def takeoff(self, altitude: float = 5.0) -> str:
        """
        Command the drone to take off to the specified altitude using MAVROS.
        
        This will arm the drone, set OFFBOARD mode, and command it to ascend.
        
        Args:
            altitude: Target altitude in meters (default: 5.0)
        
        Returns:
            str: Status message about the takeoff command
        """
        
        # Validate altitude parameter
        if altitude <= 0:
            return "Invalid altitude. Please specify a positive value in meters."
        
        if altitude > 30:
            return "Requested altitude exceeds safe limits. Please specify an altitude below 30 meters."
        
        # Get current position
        current_x = self.node.current_pose.pose.position.x
        current_y = self.node.current_pose.pose.position.y
        current_z = self.node.current_pose.pose.position.z
        
        self.node.get_logger().info(f"Initiating takeoff sequence to altitude {altitude}m...")
        
        # Step 1: Create service clients for arming and mode setting
        arming_client = self.node.create_client(
            CommandBool, 
            '/drone/mavros/cmd/arming'
        )
        mode_client = self.node.create_client(
            SetMode, 
            '/drone/mavros/set_mode'
        )
        
        # Wait for services to be available
        timeout = 5.0  # seconds
        if not arming_client.wait_for_service(timeout_sec=timeout):
            return "Arming service not available. Takeoff aborted."
        
        if not mode_client.wait_for_service(timeout_sec=timeout):
            return "Set mode service not available. Takeoff aborted."
        
        # Step 2: Create a setpoint publisher for position control if it doesn't exist
        if not hasattr(self.node, 'setpoint_pub'):
            self.node.setpoint_pub = self.node.create_publisher(
                PoseStamped,
                '/drone/mavros/setpoint_position/local',
                10
            )
        
        # Create setpoint at current position but with target altitude
        setpoint = PoseStamped()
        setpoint.header.frame_id = "map"
        setpoint.pose.position.x = current_x
        setpoint.pose.position.y = current_y
        setpoint.pose.position.z = altitude  # Target altitude
        
        # Set orientation (identity quaternion - no rotation)
        setpoint.pose.orientation.w = 1.0
        setpoint.pose.orientation.x = 0.0
        setpoint.pose.orientation.y = 0.0
        setpoint.pose.orientation.z = 0.0
        
        # Step 3: Send a few setpoints before starting (required by MAVROS)
        self.node.get_logger().info("Sending initial setpoints...")
        
        for i in range(10):
            setpoint.header.stamp = self.node.get_clock().now().to_msg()
            self.node.setpoint_pub.publish(setpoint)
            time.sleep(0.1)
        
        # Step 4: Request arming
        self.node.get_logger().info("Requesting drone arming...")
        
        arm_request = CommandBool.Request()
        arm_request.value = True
        
        future = arming_client.call_async(arm_request)
        
        # Simplified wait for result
        start_time = time.time()
        while time.time() - start_time < timeout and not future.done():
            time.sleep(0.1)
        
        if not future.done():
            return "Arming request timed out. Takeoff aborted."
        
        arm_response = future.result()
        if not arm_response.success:
            return f"Arming failed. Takeoff aborted."
        
        self.node.get_logger().info("Drone armed successfully")
        
        # Step 5: Set mode to OFFBOARD
        self.node.get_logger().info("Setting OFFBOARD mode...")
        
        mode_request = SetMode.Request()
        mode_request.custom_mode = "OFFBOARD"
        
        future = mode_client.call_async(mode_request)
        
        # Simplified wait for result
        start_time = time.time()
        while time.time() - start_time < timeout and not future.done():
            time.sleep(0.1)
        
        if not future.done():
            return "Set mode request timed out. Takeoff process may be unreliable."
        
        mode_response = future.result()
        if not mode_response.mode_sent:
            return "Failed to set OFFBOARD mode. Takeoff process may be unreliable."
        
        self.node.get_logger().info("OFFBOARD mode set successfully")
        
        # Step 6: Store the current target setpoint on the node
        if not hasattr(self.node, 'target_setpoint'):
            self.node.target_setpoint = setpoint
        else:
            self.node.target_setpoint = setpoint
        
        # Step 7: Start continuous setpoint publishing if not already running
        if not hasattr(self.node, 'setpoint_thread') or not self.node.setpoint_thread.is_alive():
            def setpoint_publisher_thread():
                rate = self.node.create_rate(10)  # 10 Hz
                while self.node.running:
                    if hasattr(self.node, 'target_setpoint'):
                        # Update timestamp
                        self.node.target_setpoint.header.stamp = self.node.get_clock().now().to_msg()
                        # Publish current target
                        self.node.setpoint_pub.publish(self.node.target_setpoint)
                    
                    rate.sleep()
                
            self.node.setpoint_thread = threading.Thread(target=setpoint_publisher_thread)
            self.node.setpoint_thread.daemon = True
            self.node.setpoint_thread.start()
            self.node.get_logger().info("Started continuous setpoint publisher")
        
        return (
            f"Takeoff sequence initiated successfully!\n\n"
            f"• Armed: ✓\n"
            f"• Mode: OFFBOARD\n"
            f"• Target altitude: {altitude:.1f}m\n\n"
            f"The drone should now be climbing to the target altitude."
        )
        
    def create_tools(self):
        """Create and return all robot tools."""
        return [self.takeoff]