#!/usr/bin/env python3
"""
Robot tools for the ROSA Agent.
This module contains tools for drone control using MAVROS.
"""

import time
import threading
from langchain.agents import tool
from mavros_msgs.srv import CommandBool, SetMode # type: ignore
from geometry_msgs.msg import PoseStamped, Twist


class RobotTools:
    """Collection of tools for robot control."""
    
    def __init__(self, node):
        self.node = node
        
    def create_tools(self):
        """Create and return all robot tools."""
        
        # Reference to the node for use in the closure
        node = self.node
        
        @tool
        def takeoff(altitude: float = 5.0) -> str:
            """
            Command the drone to take off to the specified altitude using MAVROS.
            
            This will arm the drone, set OFFBOARD mode, and command it to ascend.
            The function creates persistent service clients and publishers if they
            don't already exist, and manages the continuous sending of setpoints.
            
            Args:
                altitude: Target altitude in meters (default: 5.0)
            
            Returns:
                str: Status message about the takeoff command, or error details if the takeoff failed
            """
            # Validate altitude parameter
            if altitude <= 0:
                return "Invalid altitude. Please specify a positive value in meters."
            
            if altitude > 30:
                return "Requested altitude exceeds safe limits. Please specify an altitude below 30 meters."
            
            node.get_logger().info(f"Initiating takeoff sequence to altitude {altitude}m...")
            
            # Step 1: Create service clients for arming and mode setting (if they don't exist)
            if not hasattr(node, 'arming_client'):
                node.arming_client = node.create_client(
                    CommandBool, 
                    '/drone/mavros/cmd/arming'
                )
                node.get_logger().info("Created arming service client")
                
            if not hasattr(node, 'mode_client'):
                node.mode_client = node.create_client(
                    SetMode, 
                    '/drone/mavros/set_mode'
                )
                node.get_logger().info("Created mode service client")
            
            # Wait for services to be available
            timeout = 5.0  # seconds
            if not node.arming_client.wait_for_service(timeout_sec=timeout):
                return "Arming service not available. Takeoff aborted."
            
            if not node.mode_client.wait_for_service(timeout_sec=timeout):
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
            
            try:
                arm_request = CommandBool.Request()
                arm_request.value = True
                
                future = node.arming_client.call_async(arm_request)
                
                # Wait for result with timeout
                start_time = time.time()
                while time.time() - start_time < timeout and not future.done():
                    time.sleep(0.1)
                
                if not future.done():
                    return "Arming request timed out. Takeoff aborted."
                
                arm_response = future.result()
                if not arm_response.success:
                    return "Arming failed. Takeoff aborted."
                
                node.get_logger().info("Drone armed successfully")
            except Exception as e:
                node.get_logger().error(f"Arming failed with error: {str(e)}")
                return f"Arming failed with error: {str(e)}"
            
            # Step 5: Set mode to OFFBOARD
            node.get_logger().info("Setting OFFBOARD mode...")
            
            try:
                mode_request = SetMode.Request()
                mode_request.custom_mode = "OFFBOARD"
                
                future = node.mode_client.call_async(mode_request)
                
                # Wait for result with timeout
                start_time = time.time()
                while time.time() - start_time < timeout and not future.done():
                    time.sleep(0.1)
                
                if not future.done():
                    return "Set mode request timed out. Takeoff process may be unreliable."
                
                mode_response = future.result()
                if not mode_response.mode_sent:
                    return "Failed to set OFFBOARD mode. Takeoff process may be unreliable."
                
                node.get_logger().info("OFFBOARD mode set successfully")
            except Exception as e:
                node.get_logger().error(f"Mode setting failed with error: {str(e)}")
                return f"Setting OFFBOARD mode failed with error: {str(e)}"
            
            # Step 6: Store the current target setpoint on the node
            node.target_setpoint = setpoint
            
            # Always ensure setpoint publishing is enabled for takeoff
            node.publish_setpoints = True
            node.get_logger().info("Enabled position control for takeoff")
                        
            # Step 7: Start continuous setpoint publishing if not already running
            if not hasattr(node, 'setpoint_thread') or not node.setpoint_thread.is_alive():
                def setpoint_publisher_thread():
                    """Thread that publishes position commands at 10Hz to maintain drone control"""
                    rate = node.create_rate(10)  # 10 Hz publication rate
                    
                    while node.running:
                        # Only publish when enabled and a target position exists
                        if getattr(node, 'publish_setpoints', False) and hasattr(node, 'target_setpoint'):
                            # Update timestamp and publish the current target position
                            node.target_setpoint.header.stamp = node.get_clock().now().to_msg()
                            node.setpoint_pub.publish(node.target_setpoint)
                        
                        # Sleep precisely to maintain 10Hz frequency
                        rate.sleep()
                
                # Create and start the publisher thread
                node.setpoint_thread = threading.Thread(target=setpoint_publisher_thread)
                node.setpoint_thread.daemon = True  # Thread will terminate when main program exits
                node.setpoint_thread.start()
                node.get_logger().info("Started position control publisher at 10Hz")
                        
            return (
                f"Takeoff sequence initiated successfully!\n\n"
                f"• Armed: ✓\n"
                f"• Mode: OFFBOARD\n"
                f"• Target altitude: {altitude:.1f}m\n\n"
                f"The drone should now be climbing to the target altitude."
            )
            
        @tool
        def land(timeout: float = 60.0) -> str:
            """
            Command the drone to land at the current location with verification.
            
            This will switch the drone to AUTO.LAND mode and actively monitor the
            drone's altitude until landing is confirmed or timeout is reached.
            
            Args:
                timeout: Maximum time in seconds to wait for landing (default: 60.0)
            
            Returns:
                str: Status message about the landing process
            """
            node.get_logger().info("Initiating landing sequence with verification...")
            
            # Check that mode client exists
            if not hasattr(node, 'mode_client'):
                node.mode_client = node.create_client(
                    SetMode, 
                    '/drone/mavros/set_mode'
                )
                node.get_logger().info("Created mode service client")
            
            # Wait for service to be available
            service_timeout = 5.0  # seconds
            if not node.mode_client.wait_for_service(timeout_sec=service_timeout):
                return "Set mode service not available. Landing aborted."
            
            # Get initial altitude
            initial_altitude = node.current_pose.pose.position.z
            node.get_logger().info(f"Current altitude before landing: {initial_altitude:.2f}m")
            
            if initial_altitude < 0.1:
                return "Drone appears to be already on the ground (altitude < 0.1m)."
            
            # Disable setpoint publishing (if active)
            if hasattr(node, 'publish_setpoints'):
                node.publish_setpoints = False
                node.get_logger().info("Disabled position control for landing")
            
            # Set mode to AUTO.LAND
            node.get_logger().info("Setting AUTO.LAND mode...")
            
            try:
                mode_request = SetMode.Request()
                mode_request.custom_mode = "AUTO.LAND"
                
                future = node.mode_client.call_async(mode_request)
                
                # Wait for result with timeout
                start_time = time.time()
                while time.time() - start_time < service_timeout and not future.done():
                    time.sleep(0.1)
                
                if not future.done():
                    return "Set mode request timed out. Landing may not have been initiated."
                
                mode_response = future.result()
                if not mode_response.mode_sent:
                    return "Failed to set AUTO.LAND mode. Landing may not have been initiated."
                
                node.get_logger().info("AUTO.LAND mode set successfully")
            except Exception as e:
                node.get_logger().error(f"Setting AUTO.LAND mode failed with error: {str(e)}")
                return f"Landing failed with error: {str(e)}"
            
            # Monitor landing progress
            start_time = time.time()
            last_altitude = initial_altitude
            progress_updates = []
            
            while time.time() - start_time < timeout:
                current_altitude = node.current_pose.pose.position.z
                
                # If altitude hasn't changed in 5 seconds and is near zero, consider landing complete
                if abs(current_altitude - last_altitude) < 0.05 and current_altitude < 0.2:
                    landing_success = True
                    break
                
                # Log progress every 1m of descent
                if last_altitude - current_altitude > 1.0:
                    progress_message = f"Descending: {current_altitude:.2f}m"
                    node.get_logger().info(progress_message)
                    progress_updates.append(progress_message)
                    last_altitude = current_altitude
                
                time.sleep(1.0)
            else:
                # Timeout occurred
                current_altitude = node.current_pose.pose.position.z
                if current_altitude < 0.5:
                    landing_success = True
                else:
                    return (
                        f"Landing timeout after {timeout:.1f} seconds.\n"
                        f"Current altitude: {current_altitude:.2f}m (started at {initial_altitude:.2f}m).\n"
                        f"The drone may still be in the process of landing."
                    )
            
            # Verify landing completed
            final_altitude = node.current_pose.pose.position.z
            descent_amount = initial_altitude - final_altitude
            
            landing_status = (
                f"Landing sequence completed!\n\n"
                f"• Starting altitude: {initial_altitude:.2f}m\n"
                f"• Current altitude: {final_altitude:.2f}m\n"
                f"• Descent amount: {descent_amount:.2f}m\n"
                f"• Mode: AUTO.LAND\n\n"
            )
            
            if final_altitude < 0.2:
                landing_status += "The drone has successfully landed and should auto-disarm."
            else:
                landing_status += f"WARNING: Drone altitude is still {final_altitude:.2f}m above ground!"
            
            return landing_status
        
        # Return the tools
        return [
            takeoff,
            land
        ]