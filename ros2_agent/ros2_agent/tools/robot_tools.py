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
from geometry_msgs.msg import Twist, PoseStamped


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

            # Re-enable setpoint publishing if it was disabled by landing
            if hasattr(node, 'publish_setpoints'):
                node.publish_setpoints = True
                node.get_logger().info("Resumed position setpoint publishing")
            
            
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
    
                    # Add a control flag
                    if not hasattr(node, 'publish_setpoints'):
                        node.publish_setpoints = True
                        
                    while node.running:
                        # Check if we should be publishing
                        if hasattr(node, 'publish_setpoints') and node.publish_setpoints:
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
            Command the drone to land using PX4's native AUTO.LAND mode.
            
            Returns:
                str: Status message about the landing command
            """
            node.get_logger().info("Executing landing sequence...")
            
            # Pause the setpoint publisher before landing
            if hasattr(node, 'publish_setpoints'):
                node.publish_setpoints = False
                node.get_logger().info("Paused position setpoint publishing")
            
            # Use the PX4 native AUTO.LAND mode
            try:
                # Create service client for mode setting
                mode_client = node.create_client(SetMode, '/drone/mavros/set_mode')
                
                if mode_client.wait_for_service(timeout_sec=2.0):
                    # Request AUTO.LAND mode
                    mode_request = SetMode.Request()
                    mode_request.custom_mode = "AUTO.LAND"
                    
                    # Send request asynchronously
                    future = mode_client.call_async(mode_request)
                    
                    # Wait up to 3 seconds for response
                    timeout = 3.0
                    start_time = time.time()
                    while time.time() - start_time < timeout and not future.done():
                        time.sleep(0.1)
                    
                    # Check if AUTO.LAND mode was set successfully
                    if future.done() and future.result().mode_sent:
                        node.get_logger().info("AUTO.LAND mode set successfully")
                        return (
                            f"Landing initiated using native AUTO.LAND mode.\n\n"
                            f"• The drone's autopilot will handle the landing sequence\n"
                            f"• This is the most reliable landing method\n\n"
                            f"The drone should now be descending to the ground."
                        )
                    else:
                        return "Landing failed - could not set AUTO.LAND mode."
                else:
                    return "Landing failed - mode service not available."
            except Exception as e:
                node.get_logger().error(f"AUTO.LAND mode failed: {str(e)}")
                return f"Landing failed - error during mode change: {str(e)}"
                        ######################## Landing Tool Ends ########################


        # Return the tools
        return [
                takeoff,
                land]