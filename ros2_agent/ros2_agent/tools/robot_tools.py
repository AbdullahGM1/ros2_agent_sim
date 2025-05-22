#!/usr/bin/env python3
"""
Robot tools for the ROSA Agent.
This module contains tools for drone control using MAVROS.
"""

import time
import threading
from langchain.agents import tool
from mavros_msgs.srv import CommandBool, SetMode # type: ignore
from geometry_msgs.msg import PoseStamped


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
            
            Args:
                altitude: Target altitude in meters (default: 5.0)
            
            Returns:
                str: Status message about the takeoff command
            """
            # 1. Input validation
            if altitude <= 0:
                return "Error: Altitude must be positive"
            if altitude > 20:
                return "Error: Altitude exceeds safe limit (20m max)"
            
            # Get current position from the drone
            current_x = node.current_pose.pose.position.x
            current_y = node.current_pose.pose.position.y
            current_z = node.current_pose.pose.position.z
            
            node.get_logger().info(f"Current position: ({current_x:.2f}, {current_y:.2f}, {current_z:.2f})")
            node.get_logger().info(f"Taking off to: ({current_x:.2f}, {current_y:.2f}, {altitude:.2f})")
            
            node.get_logger().info(f"Initiating takeoff to altitude {altitude}m...")
            
            # 2. Create service clients with better error handling
            try:
                if not hasattr(node, 'arming_client'):
                    node.arming_client = node.create_client(CommandBool, '/drone/mavros/cmd/arming')
                    node.get_logger().info("Created arming service client")
                    
                if not hasattr(node, 'mode_client'):
                    node.mode_client = node.create_client(SetMode, '/drone/mavros/set_mode')
                    node.get_logger().info("Created mode service client")
            except Exception as e:
                return f"Error creating service clients: {str(e)}"
            
            # 3. Wait for services to be available
            timeout = 5.0
            if not node.arming_client.wait_for_service(timeout_sec=timeout):
                return "Error: Arming service not available"
            if not node.mode_client.wait_for_service(timeout_sec=timeout):
                return "Error: Mode service not available"
            
            # 4. Create setpoint publisher
            if not hasattr(node, 'setpoint_pub'):
                node.setpoint_pub = node.create_publisher(
                    PoseStamped,
                    '/drone/mavros/setpoint_position/local',
                    10
                )
                node.get_logger().info("Created setpoint publisher")
            
            # 5. Create and send initial setpoints
            setpoint = PoseStamped()
            setpoint.header.frame_id = "map"
            setpoint.pose.position.x = current_x
            setpoint.pose.position.y = current_y
            setpoint.pose.position.z = altitude
            setpoint.pose.orientation.w = 1.0
            
            node.get_logger().info("Sending initial setpoints...")
            for i in range(10):
                setpoint.header.stamp = node.get_clock().now().to_msg()
                node.setpoint_pub.publish(setpoint)
                time.sleep(0.1)
            
            # 6. Arm the drone with proper waiting
            node.get_logger().info("Requesting drone arming...")
            try:
                arm_request = CommandBool.Request()
                arm_request.value = True
                future = node.arming_client.call_async(arm_request)
                
                # Wait for arming response
                start_time = time.time()
                while time.time() - start_time < timeout and not future.done():
                    time.sleep(0.1)
                
                if not future.done():
                    return "Error: Arming request timed out"
                    
                arm_response = future.result()
                if not arm_response.success:
                    return "Error: Failed to arm drone"
                    
                node.get_logger().info("Drone armed successfully")
            except Exception as e:
                return f"Error during arming: {str(e)}"
            
            # 7. Set OFFBOARD mode with proper waiting
            node.get_logger().info("Setting OFFBOARD mode...")
            try:
                mode_request = SetMode.Request()
                mode_request.custom_mode = "OFFBOARD"
                future = node.mode_client.call_async(mode_request)
                
                # Wait for mode response
                start_time = time.time()
                while time.time() - start_time < timeout and not future.done():
                    time.sleep(0.1)
                
                if not future.done():
                    return "Error: Mode setting timed out"
                    
                mode_response = future.result()
                if not mode_response.mode_sent:
                    return "Error: Failed to set OFFBOARD mode"
                    
                node.get_logger().info("OFFBOARD mode set successfully")
            except Exception as e:
                return f"Error setting mode: {str(e)}"
            
            # 8. Store setpoint and enable publishing
            node.target_setpoint = setpoint
            node.publish_setpoints = True
            
            # 9. Start setpoint publisher thread (improved)
            if not hasattr(node, 'setpoint_thread') or not node.setpoint_thread.is_alive():
                def setpoint_publisher():
                    rate = node.create_rate(10)
                    while node.running:
                        if getattr(node, 'publish_setpoints', False) and hasattr(node, 'target_setpoint'):
                            node.target_setpoint.header.stamp = node.get_clock().now().to_msg()
                            node.setpoint_pub.publish(node.target_setpoint)
                        rate.sleep()
                
                node.setpoint_thread = threading.Thread(target=setpoint_publisher)
                node.setpoint_thread.daemon = True
                node.setpoint_thread.start()
                node.get_logger().info("Started setpoint publisher thread")
            
            return f"✅ Takeoff successful! Target altitude: {altitude}m\n• Armed: ✓\n• Mode: OFFBOARD\n• Publishing setpoints: ✓"
            
        @tool
        def land() -> str:
            """
            Land the drone at its current location.
    
            Use this tool when the user wants to:
            - Land the drone
            - Bring the drone down
            - Go to the ground
            - Descend to ground level
            - Stop flying and land
            - Return to ground
            - Touch down
            - Complete landing sequence
            
            This function disables position control allowing the drone to descend naturally
            to the ground. It monitors the landing process until the drone reaches ground
            level (altitude < 0.2m).
            
            The tool automatically handles:
            - Disabling setpoint publishing 
            - Monitoring descent progress
            - Confirming successful touchdown
            - Providing detailed landing status
            
            Returns:
                str: Detailed status of the landing process including starting altitude,
                    final altitude, descent amount, and landing confirmation.
            """
            node.get_logger().info("Initiating complete landing sequence...")
            
            # 1. Get current position for reference
            current_x = node.current_pose.pose.position.x
            current_y = node.current_pose.pose.position.y
            start_altitude = node.current_pose.pose.position.z
            
            node.get_logger().info(f"Starting landing from position: ({current_x:.2f}, {current_y:.2f}, {start_altitude:.2f})")
            
            # 2. Check if we're already on the ground
            if start_altitude < 0.2:
                node.get_logger().info("Drone appears to be already on the ground")
                return f"Drone is already on ground (altitude: {start_altitude:.2f}m)."
            
            # 3. DISABLE setpoint publishing to allow natural descent/landing
            if hasattr(node, 'publish_setpoints'):
                node.publish_setpoints = False
                node.get_logger().info("✅ Disabled position control - drone will now descend to ground")
            else:
                node.get_logger().warn("No active position control found")
                return "Warning: No position control was active"
            
            # 4. Brief pause to ensure the change takes effect
            time.sleep(0.5)
            
            # 5. Monitor descent until COMPLETE landing
            timeout = 30.0  # Reduced from 45.0 seconds
            start_time = time.time()
            last_report_time = start_time
            descent_detected = False
            
            node.get_logger().info(f"Monitoring landing progress (timeout: {timeout}s)...")
            
            while time.time() - start_time < timeout:
                current_altitude = node.current_pose.pose.position.z
                elapsed_time = time.time() - start_time
                
                # Check if we've actually landed (ground level)
                if current_altitude < 0.2:
                    landing_time = elapsed_time
                    node.get_logger().info(f"🎯 LANDING COMPLETE! Final altitude: {current_altitude:.2f}m (took {landing_time:.1f}s)")
                    break
                
                # Detect if descent has started
                if not descent_detected and start_altitude - current_altitude > 0.3:
                    descent_detected = True
                    node.get_logger().info(f"✅ Descent confirmed - continuing to ground level...")
                
                # Report progress every 2 seconds (reduced from 3)
                if time.time() - last_report_time > 2.0:
                    descent_amount = start_altitude - current_altitude
                    node.get_logger().info(f"Landing progress: {current_altitude:.2f}m (descended {descent_amount:.2f}m)")
                    last_report_time = time.time()
                
                # Check more frequently (reduced from 1.0 to 0.5 seconds)
                time.sleep(0.5)
            else:
                # Timeout occurred - check final state
                current_altitude = node.current_pose.pose.position.z
                if current_altitude < 0.2:
                    node.get_logger().info("Landing completed during timeout check")
                else:
                    node.get_logger().warn(f"Landing timeout - drone still at {current_altitude:.2f}m")
            
            # 6. Get final position
            final_altitude = node.current_pose.pose.position.z
            total_descent = start_altitude - final_altitude
            
            # 7. Return comprehensive status report
            if final_altitude < 0.2:
                status = (
                    f"🎯 LANDING SUCCESSFUL!\n\n"
                    f"• Starting altitude: {start_altitude:.2f}m\n"
                    f"• Final altitude: {final_altitude:.2f}m\n"
                    f"• Total descent: {total_descent:.2f}m\n"
                    f"• Landing position: ({current_x:.2f}, {current_y:.2f})\n"
                    f"• Position control: DISABLED\n\n"
                    f"✅ Drone has landed successfully."
                )
            elif final_altitude < 1.0:
                status = (
                    f"⚠️ PARTIAL LANDING\n\n"
                    f"• Starting altitude: {start_altitude:.2f}m\n"
                    f"• Current altitude: {final_altitude:.2f}m\n"
                    f"• Descent achieved: {total_descent:.2f}m\n"
                    f"• Position control: DISABLED\n\n"
                    f"Drone is very low but may not be fully landed. Check visually."
                )
            else:
                status = (
                    f"❌ LANDING INCOMPLETE\n\n"
                    f"• Starting altitude: {start_altitude:.2f}m\n"
                    f"• Current altitude: {final_altitude:.2f}m\n"
                    f"• Descent achieved: {total_descent:.2f}m\n"
                    f"• Position control: DISABLED\n\n"
                    f"Landing may have failed. Drone still airborne at {final_altitude:.2f}m."
                )
            
            return status
        
        @tool
        def get_drone_pose() -> str:
            """
            Get the current position and orientation of the drone.
            
            Use this tool when the user wants to:
            - Know the drone's current position
            - Check where the drone is
            - Get location information
            - Check coordinates
            - See current altitude
            - Get pose data
            
            Returns:
                str: Current drone position (x, y, z) and orientation information
            """
            node.get_logger().info("Getting current drone pose...")
            
            # Get current pose from the node
            current_pose = node.current_pose
            
            # Extract position
            x = current_pose.pose.position.x
            y = current_pose.pose.position.y
            z = current_pose.pose.position.z
            
            # Extract orientation (quaternion)
            qx = current_pose.pose.orientation.x
            qy = current_pose.pose.orientation.y
            qz = current_pose.pose.orientation.z
            qw = current_pose.pose.orientation.w
            
            # Log the pose information
            node.get_logger().info(f"Current position: ({x:.2f}, {y:.2f}, {z:.2f})")
            
            # Format response
            pose_info = (
                f"🚁 Current Drone Pose:\n\n"
                f"📍 Position:\n"
                f"  • X: {x:.2f} m\n"
                f"  • Y: {y:.2f} m\n"
                f"  • Z (Altitude): {z:.2f} m\n\n"
                f"🧭 Orientation (Quaternion):\n"
                f"  • X: {qx:.3f}\n"
                f"  • Y: {qy:.3f}\n"
                f"  • Z: {qz:.3f}\n"
                f"  • W: {qw:.3f}\n\n"
                f"📊 Status: Data retrieved successfully"
            )
            
            return pose_info
        
        # Return the tools
        return [
            takeoff,
            land,
            get_drone_pose
        ]