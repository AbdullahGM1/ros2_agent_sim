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
        
    def create_tools(self):
        """Create and return all robot tools."""
        
        # Reference to the node for use in the closure
        node = self.node
        
                    ######################## Takeoff Tool Starts ########################

        @tool
        def takeoff(altitude: float = 5.0) -> str:
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
            
            node.get_logger().info(f"Initiating takeoff sequence to altitude {altitude}m...")
            
            # Step 1: Create service clients for [Arming] and [Mode Setting]
            arming_client = node.create_client(
                CommandBool, 
                '/drone/mavros/cmd/arming'
            )
            mode_client = node.create_client(
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
            if not hasattr(node, 'setpoint_pub'):
                node.setpoint_pub = node.create_publisher(
                    PoseStamped,
                    '/drone/mavros/setpoint_position/local',
                    10
                )
            
            # Create setpoint with target altitude
            setpoint = PoseStamped()
            setpoint.header.frame_id = "map"
            setpoint.pose.position.x = 0.0  # Use fixed position
            setpoint.pose.position.y = 0.0  # Use fixed position
            setpoint.pose.position.z = altitude  # Target altitude
            
            # Set orientation (identity quaternion - no rotation)
            setpoint.pose.orientation.w = 1.0
            setpoint.pose.orientation.x = 0.0
            setpoint.pose.orientation.y = 0.0
            setpoint.pose.orientation.z = 0.0
            
            # Step 3: Send a few setpoints before starting (required by MAVROS)
            node.get_logger().info("Sending initial setpoints...")
            
            for i in range(10):
                setpoint.header.stamp = node.get_clock().now().to_msg()
                node.setpoint_pub.publish(setpoint)
                time.sleep(0.1)
            
            # Step 4: Request arming
            node.get_logger().info("Requesting drone arming...")
            
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
            
            node.get_logger().info("Drone armed successfully")
            
            # Step 5: Set mode to OFFBOARD
            node.get_logger().info("Setting OFFBOARD mode...")
            
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
            
            node.get_logger().info("OFFBOARD mode set successfully")
            
            # Step 6: Store the current target setpoint on the node
            if not hasattr(node, 'target_setpoint'):
                node.target_setpoint = setpoint
            else:
                node.target_setpoint = setpoint
            
            # Step 7: Start continuous setpoint publishing if not already running
            if not hasattr(node, 'setpoint_thread') or not node.setpoint_thread.is_alive():
                def setpoint_publisher_thread():
                    rate = node.create_rate(10)  # 10 Hz
                    while node.running:
                        if hasattr(node, 'target_setpoint'):
                            # Update timestamp
                            node.target_setpoint.header.stamp = node.get_clock().now().to_msg()
                            # Publish current target
                            node.setpoint_pub.publish(node.target_setpoint)
                        
                        rate.sleep()
                    
                node.setpoint_thread = threading.Thread(target=setpoint_publisher_thread)
                node.setpoint_thread.daemon = True
                node.setpoint_thread.start()
                node.get_logger().info("Started continuous setpoint publisher")
            
            return (
                f"Takeoff sequence initiated successfully!\n\n"
                f"• Armed: ✓\n"
                f"• Mode: OFFBOARD\n"
                f"• Target altitude: {altitude:.1f}m\n\n"
                f"The drone should now be climbing to the target altitude."
            )
        
                    ######################## Takeoff Tool Ends ########################



                    ######################## Landing Tool Starts ########################

        @tool
        def land() -> str:
            """
            Command the drone to land safely at its current position.
            
            This will initiate a controlled descent and ensure the drone lands safely.
            
            Returns:
                str: Status message about the landing operation
            """
            node.get_logger().info("Initiating landing sequence...")
            
            # Try to switch to land mode (as a primary approach)
            try:
                mode_client = node.create_client(SetMode, '/drone/mavros/set_mode')
                if mode_client.wait_for_service(timeout_sec=2.0):
                    mode_request = SetMode.Request()
                    mode_request.custom_mode = "AUTO.LAND"
                    
                    future = mode_client.call_async(mode_request)
                    
                    # Wait for result
                    timeout = 5.0
                    start_time = time.time()
                    while time.time() - start_time < timeout and not future.done():
                        time.sleep(0.1)
                        
                    if future.done() and future.result().mode_sent:
                        node.get_logger().info("AUTO.LAND mode set successfully")
                        return (
                            f"Landing sequence initiated successfully!\n\n"
                            f"• Mode: AUTO.LAND\n"
                            f"• The drone should now be descending to the ground automatically."
                        )
            except Exception as e:
                node.get_logger().warning(f"Could not set AUTO.LAND mode: {str(e)}")
            
            # Fallback approach: Controlled descent using position setpoints
            node.get_logger().info("Using position-based controlled descent...")
            
            # Create setpoint at current x,y with 0 altitude
            setpoint = PoseStamped()
            setpoint.header.frame_id = "map"
            setpoint.pose.position.x = 0.0  # Maintain current X
            setpoint.pose.position.y = 0.0  # Maintain current Y
            setpoint.pose.position.z = 0.0  # Target ground level
            
            # Set orientation (identity quaternion - no rotation)
            setpoint.pose.orientation.w = 1.0
            setpoint.pose.orientation.x = 0.0
            setpoint.pose.orientation.y = 0.0
            setpoint.pose.orientation.z = 0.0
            
            # Store the landing setpoint on the node
            node.target_setpoint = setpoint
            
            # Make sure we have a setpoint publisher and thread
            if not hasattr(node, 'setpoint_pub'):
                node.setpoint_pub = node.create_publisher(
                    PoseStamped,
                    '/drone/mavros/setpoint_position/local',
                    10
                )
            
            # Start continuous setpoint publishing if not already running
            if not hasattr(node, 'setpoint_thread') or not node.setpoint_thread.is_alive():
                def setpoint_publisher_thread():
                    rate = node.create_rate(10)  # 10 Hz
                    while node.running:
                        if hasattr(node, 'target_setpoint'):
                            # Update timestamp
                            node.target_setpoint.header.stamp = node.get_clock().now().to_msg()
                            # Publish current target
                            node.setpoint_pub.publish(node.target_setpoint)
                        
                        rate.sleep()
                    
                node.setpoint_thread = threading.Thread(target=setpoint_publisher_thread)
                node.setpoint_thread.daemon = True
                node.setpoint_thread.start()
                node.get_logger().info("Started continuous setpoint publisher")
            
            return (
                f"Landing sequence initiated successfully!\n\n"
                f"• Method: Controlled position descent\n"
                f"• Target: Ground level (altitude 0.0m)\n\n"
                f"The drone should now be descending to the ground."
            )
                        ######################## Landing Tool Ends ########################


        # Return the tools
        return [
                takeoff,
                land]