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
            Command the drone to land at its current position using direct velocity control.
            This ensures the drone will descend regardless of flight mode or service availability.
            
            Returns:
                str: Status message about the landing command
            """
            node.get_logger().info("Executing landing sequence...")
            
            # Create velocity publisher if it doesn't exist
            if not hasattr(node, 'cmd_vel_publisher'):
                node.cmd_vel_publisher = node.create_publisher(
                    Twist,
                    '/drone/mavros/setpoint_velocity/cmd_vel',
                    10
                )
            
            # 1. First, try to switch to land mode (as a background attempt)
            try:
                mode_client = node.create_client(SetMode, '/drone/mavros/set_mode')
                if mode_client.wait_for_service(timeout_sec=1.0):
                    mode_request = SetMode.Request()
                    mode_request.custom_mode = "AUTO.LAND"
                    mode_client.call_async(mode_request)
                    node.get_logger().info("Land mode request sent (background)")
            except Exception as e:
                node.get_logger().warning(f"Mode switch failed: {str(e)}")
            
            # 2. Create a dedicated thread for controlled descent using velocity commands
            def controlled_descent():
                node.get_logger().info("Starting controlled descent...")
                
                # Initialize descent velocity command
                from geometry_msgs.msg import Twist
                vel_cmd = Twist()
                vel_cmd.linear.z = -0.5  # Descend at 0.5 m/s
                
                # Get control rate for sleep calculations
                control_rate = 10.0  # Hz
                if hasattr(node, 'control_rate'):
                    control_rate = node.control_rate
                sleep_time = 1.0 / control_rate
                
                # Minimum altitude to consider "landed"
                min_altitude = 0.15  # meters
                
                # Maximum time for landing (safety timeout)
                max_landing_time = 120.0  # seconds
                start_time = time.time()
                
                # Start descent loop
                while node.running:
                    # Get current altitude if available
                    current_alt = 0
                    if hasattr(node, 'current_pose') and hasattr(node.current_pose, 'pose'):
                        current_alt = node.current_pose.pose.position.z
                    
                    # Check if we're already at the ground
                    if current_alt <= min_altitude:
                        node.get_logger().info(f"Landed! Current altitude: {current_alt:.2f}m")
                        # Send stop command to ensure motors off
                        stop_cmd = Twist()
                        for _ in range(5):
                            node.cmd_vel_publisher.publish(stop_cmd)
                            time.sleep(0.05)
                        break
                    
                    # Adjust velocity based on altitude (slow down near ground)
                    if current_alt < 1.0:
                        vel_cmd.linear.z = -0.2  # Slower descent near ground
                    else:
                        vel_cmd.linear.z = -0.5  # Normal descent
                    
                    # Send descent command
                    node.cmd_vel_publisher.publish(vel_cmd)
                    
                    # Check timeout
                    if time.time() - start_time > max_landing_time:
                        node.get_logger().warning("Landing timeout exceeded, stopping descent")
                        # Send stop command
                        stop_cmd = Twist()
                        node.cmd_vel_publisher.publish(stop_cmd)
                        break
                    
                    # Sleep for control interval
                    time.sleep(sleep_time)
                
                # Final stop command to ensure motors off
                stop_cmd = Twist()
                for _ in range(5):
                    node.cmd_vel_publisher.publish(stop_cmd)
                    time.sleep(0.05)
                
                node.get_logger().info("Controlled descent completed")
            
            # Start the controlled descent thread
            descent_thread = threading.Thread(target=controlled_descent)
            descent_thread.daemon = True
            descent_thread.start()
            
            return (
                f"LANDING procedure initiated.\n\n"
                f"• Method: Controlled velocity descent\n"
                f"• The drone is now descending toward the ground at a safe speed\n\n"
                f"Landing should complete soon.\n"
            )
                        ######################## Landing Tool Ends ########################


        # Return the tools
        return [
                takeoff,
                land]