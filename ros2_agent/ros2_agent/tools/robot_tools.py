#!/usr/bin/env python3
"""
Robot tools for the ROSA Agent.
This module contains tools for both ground robots and aerial drones.
"""

import math
import time
import threading
import cv2
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from langchain.agents import tool
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class RobotTools:
    """Collection of tools for robot control."""
    
    def __init__(self, node):
        self.node = node
        
    def create_tools(self):
        """Create and return all robot tools."""
        
        @tool
        def get_drone_camera_image() -> dict:
            """
            Display a simple live stream from the drone's camera.
            Press 'q' in the camera window to close the stream.
            """
            # Check if camera is already active
            with self.node.camera_lock:
                if self.node.camera_active:
                    return {"message": "Camera is already running. Use 'stop_camera' to stop it first."}
                self.node.camera_active = True
            
            # Define simplified camera thread function
            def simple_camera_thread():
                subscription = None
                try:
                    # Create a simple callback
                    def callback(msg):
                        if not self.node.camera_active:
                            return
                            
                        try:
                            # Convert image and display with minimal additions
                            image = self.node.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                            cv2.imshow("Drone Camera", image)
                            key = cv2.waitKey(1)
                            if key == ord('q'):
                                with self.node.camera_lock:
                                    self.node.camera_active = False
                        except Exception as e:
                            self.node.get_logger().error(f"Image processing error: {str(e)}")
                    
                    # Subscribe to camera topic
                    subscription = self.node.create_subscription(
                        Image,
                        self.node.camera_topic,
                        callback,
                        10  # Simple QoS
                    )
                    
                    # Simple loop - just check if we should keep running
                    self.node.get_logger().info(f"Camera activated. Viewing {self.node.camera_topic}")
                    while self.node.camera_active and self.node.running:
                        time.sleep(0.1)
                        
                except Exception as e:
                    self.node.get_logger().error(f"Camera error: {str(e)}")
                    
                finally:
                    # Clean up
                    with self.node.camera_lock:
                        self.node.camera_active = False
                        
                    if subscription:
                        self.node.destroy_subscription(subscription)
                        
                    try:
                        cv2.destroyAllWindows()
                    except:
                        pass
                        
                    self.node.get_logger().info("Camera stopped")
            
            # Start thread
            thread = threading.Thread(target=simple_camera_thread)
            thread.daemon = True
            thread.start()
            self.node.camera_thread = thread
            
            return {"message": "Camera activated. Press 'q' in the camera window to exit."}

        @tool
        def stop_camera() -> dict:
            """
            Stop the currently running camera stream.
            """
            # Check if camera is active
            with self.node.camera_lock:
                if not self.node.camera_active:
                    return {"message": "No active camera to stop."}
                self.node.camera_active = False
            
            # Wait briefly for thread to notice
            time.sleep(0.2)
            
            # Destroy any windows
            try:
                cv2.destroyAllWindows()
            except:
                pass
            
            return {"message": "Camera stopped."}
        
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
            from mavros_msgs.srv import CommandBool, SetMode
            from geometry_msgs.msg import PoseStamped
            import math
            
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
            
            # Record takeoff waypoint
            self.node.add_waypoint(
                "takeoff", 
                current_x, 
                current_y, 
                altitude,
                f"Takeoff initiated to {altitude}m"
            )
            
            return (
                f"Takeoff sequence initiated successfully!\n\n"
                f"• Armed: ✓\n"
                f"• Mode: OFFBOARD\n"
                f"• Target altitude: {altitude:.1f}m\n\n"
                f"The drone should now be climbing to the target altitude.\n"
                f"Use 'get_drone_position' to monitor progress."
            )
        
        @tool
        def land() -> str:
            """
            Command the drone to land at its current position.
            
            This uses the proper MAVROS landing command, which ensures the drone
            performs a controlled descent and disarms automatically upon landing.
            """
            from mavros_msgs.srv import CommandTOL, SetMode
            
            self.node.get_logger().info("Initiating landing sequence...")
            
            # Get current position
            current_x = self.node.current_pose.pose.position.x
            current_y = self.node.current_pose.pose.position.y
            current_z = self.node.current_pose.pose.position.z
            
            # First approach: Try to use the dedicated land command
            land_client = self.node.create_client(
                CommandTOL,
                '/drone/mavros/cmd/land'
            )
            
            # Check if the landing service is available
            if land_client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info("Using land command service...")
                
                # Create land request
                request = CommandTOL.Request()
                request.min_pitch = 0.0
                request.yaw = 0.0
                request.latitude = 0.0  # Use current position
                request.longitude = 0.0  # Use current position
                request.altitude = 0.0  # Ground level
                
                # Send the land command
                future = land_client.call_async(request)
                
                # Wait for result with timeout
                timeout = 5.0
                start_time = time.time()
                while time.time() - start_time < timeout and not future.done():
                    time.sleep(0.1)
                
                if future.done():
                    result = future.result()
                    if result.success:
                        self.node.get_logger().info("Land command accepted")
                        
                        # Record landing waypoint
                        self.node.add_waypoint(
                            "landing", 
                            current_x, current_y, 0.0,
                            "Land command issued"
                        )
                        
                        return (
                            f"Land command accepted. The drone is descending to land.\n\n"
                            f"• Current altitude: {current_z:.2f}m\n"
                            f"• The drone will perform a controlled landing at the current position.\n"
                            f"• It will automatically disarm upon touchdown.\n\n"
                            f"Monitor progress with 'get_drone_position'."
                        )
                    else:
                        self.node.get_logger().warn("Land command was rejected")
                else:
                    self.node.get_logger().warn("Land command timed out")
            
            # Second approach: Try changing mode to AUTO.LAND
            self.node.get_logger().info("Trying to set AUTO.LAND mode...")
            
            mode_client = self.node.create_client(
                SetMode,
                '/drone/mavros/set_mode'
            )
            
            if mode_client.wait_for_service(timeout_sec=1.0):
                # Create mode change request
                request = SetMode.Request()
                request.custom_mode = "AUTO.LAND"
                
                # Send the mode change request
                future = mode_client.call_async(request)
                
                # Wait for result with timeout
                timeout = 5.0
                start_time = time.time()
                while time.time() - start_time < timeout and not future.done():
                    time.sleep(0.1)
                
                if future.done():
                    result = future.result()
                    if result.mode_sent:
                        self.node.get_logger().info("AUTO.LAND mode set successfully")
                        
                        # Record landing waypoint
                        self.node.add_waypoint(
                            "landing", 
                            current_x, current_y, 0.0,
                            "AUTO.LAND mode set"
                        )
                        
                        return (
                            f"Landing mode activated. The drone is descending to land.\n\n"
                            f"• Current altitude: {current_z:.2f}m\n"
                            f"• Mode: AUTO.LAND\n"
                            f"• The drone will perform a controlled landing at the current position.\n\n"
                            f"Monitor progress with 'get_drone_position'."
                        )
                    else:
                        self.node.get_logger().warn("Failed to set AUTO.LAND mode")
                else:
                    self.node.get_logger().warn("Set mode request timed out")
            
            # Third (fallback) approach: Return to manual mode and let pilot land
            # This is a final fallback that should work with most flight controllers
            self.node.get_logger().info("Trying to return to manual control mode...")
            
            if mode_client.wait_for_service(timeout_sec=1.0):
                # Create mode change request to switch to manual mode
                request = SetMode.Request()
                request.custom_mode = "MANUAL"  # Or could be "POSCTL" depending on firmware
                
                # Send the mode change request
                future = mode_client.call_async(request)
                
                # Wait for result with timeout
                timeout = 5.0
                start_time = time.time()
                while time.time() - start_time < timeout and not future.done():
                    time.sleep(0.1)
                
                if future.done():
                    result = future.result()
                    if result.mode_sent:
                        self.node.get_logger().info("Manual mode set successfully")
                        
                        return (
                            f"WARNING: Could not initiate automatic landing.\n\n"
                            f"The drone has been switched to manual control mode.\n"
                            f"Please take manual control to land the drone safely."
                        )
                    else:
                        self.node.get_logger().warn("Failed to set manual mode")
                else:
                    self.node.get_logger().warn("Set mode request timed out")
            
            # If all approaches failed, inform the user
            self.node.get_logger().error("All landing approaches failed")
            
            return (
                f"⚠️ Unable to initiate landing sequence!\n\n"
                f"All attempted landing methods failed. The drone is still in the previous flight mode.\n\n"
                f"Please check if the flight controller is responding to commands and try again, "
                f"or take manual control if necessary."
            )
        
        @tool
        def go_to_position(x: float, y: float, z: float) -> str:
            """
            Command the drone to move to a specific position (x,y,z) in meters.
            
            This will update the target setpoint to the new coordinates.
            The drone will move to this position while maintaining OFFBOARD mode.
            
            Args:
                x: Target x-coordinate in meters
                y: Target y-coordinate in meters
                z: Target altitude in meters
            
            Returns:
                str: Status message about the movement command
            """
            # Check if setpoint control is initialized
            if not hasattr(self.node, 'target_setpoint') or not hasattr(self.node, 'setpoint_pub'):
                return "Setpoint control not initialized. Use takeoff first."
            
            # Get current position
            current_x = self.node.current_pose.pose.position.x
            current_y = self.node.current_pose.pose.position.y
            current_z = self.node.current_pose.pose.position.z
            
            # Calculate distance to target
            dx = x - current_x
            dy = y - current_y
            dz = z - current_z
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Update the target setpoint
            self.node.target_setpoint.pose.position.x = x
            self.node.target_setpoint.pose.position.y = y
            self.node.target_setpoint.pose.position.z = z
            
            # Calculate estimated time to reach target (assuming average speed of 1 m/s)
            estimated_time = distance  # in seconds
            
            self.node.get_logger().info(f"Moving to position ({x:.2f}, {y:.2f}, {z:.2f}), distance: {distance:.2f}m")
            
            # Record waypoint
            self.node.add_waypoint(
                "navigation", 
                x, y, z,
                f"Moving to position ({x:.2f}, {y:.2f}, {z:.2f})"
            )
            
            return (
                f"Moving to position ({x:.2f}, {y:.2f}, {z:.2f}).\n\n"
                f"• Current position: ({current_x:.2f}, {current_y:.2f}, {current_z:.2f})\n"
                f"• Distance: {distance:.2f} meters\n"
                f"• Estimated flight time: {estimated_time:.1f} seconds\n\n"
                f"Monitor progress with 'get_drone_position'."
            )
        @tool
        def get_drone_position() -> dict:
            """
            Get the current position and orientation of the drone.
            """
            result = {
                "x": self.node.current_pose.pose.position.x,
                "y": self.node.current_pose.pose.position.y,
                "z": self.node.current_pose.pose.position.z,
            }
            
            # Get orientation as a quaternion
            qx = self.node.current_pose.pose.orientation.x
            qy = self.node.current_pose.pose.orientation.y
            qz = self.node.current_pose.pose.orientation.z
            qw = self.node.current_pose.pose.orientation.w
            
            # Convert quaternion to Euler angles (roll, pitch, yaw)
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
            
            # Add orientation to the result
            result["roll"] = roll
            result["pitch"] = pitch
            result["yaw"] = yaw
            result["roll_degrees"] = math.degrees(roll)
            result["pitch_degrees"] = math.degrees(pitch)
            result["yaw_degrees"] = math.degrees(yaw)
            
            return result
        
        def quaternion_to_euler(self, x, y, z, w):
            """Convert quaternion to Euler angles (roll, pitch, yaw)."""
            # Roll (rotation around x-axis)
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            
            # Pitch (rotation around y-axis)
            sinp = 2 * (w * y - z * x)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
            else:
                pitch = math.asin(sinp)
            
            # Yaw (rotation around z-axis)
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return roll, pitch, yaw
        
        # Make helper functions accessible to tools
        self.quaternion_to_euler = quaternion_to_euler
        
        # Return all drone tools
        return [
            takeoff,
            land,
            go_to_position,
            get_drone_position,
            get_drone_camera_image,
            stop_camera
        ]