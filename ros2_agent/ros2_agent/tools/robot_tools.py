#!/usr/bin/env python3
"""
Robot tools for the ROSA Agent.
This module contains tools for drone control, limited to takeoff only.
"""

import time
import threading
from geometry_msgs.msg import PoseStamped
from langchain.agents import tool
from mavros_msgs.srv import CommandBool, SetMode  # type: ignore
from mavros_msgs.msg import State  # type: ignore


class RobotTools:
    """Collection of tools for robot control."""
    
    def __init__(self, node, state_topic, arming_service, mode_service, setpoint_topic):
        """
        Initialize the RobotTools with necessary ROS topics and services.
        
        Args:
            node: The ROS2 node object
            state_topic: Topic for drone state information
            arming_service: Service for arming the drone
            mode_service: Service for setting flight mode
            setpoint_topic: Topic for publishing position setpoints
        """
        self.node = node
        self.state_topic = state_topic
        self.arming_service = arming_service
        self.mode_service = mode_service
        self.setpoint_topic = setpoint_topic
        
        # Initialize state subscriber if it doesn't exist
        if not hasattr(node, 'state_sub'):
            node.state_sub = node.create_subscription(
                State,
                self.state_topic,
                self._state_callback,
                10
            )
            node.current_state = State()
            node.current_state.connected = False
            node.current_state.armed = False
            node.current_state.mode = ""
            
    def _state_callback(self, msg):
        """Callback for drone state updates."""
        self.node.current_state = msg
        
    def create_tools(self):
        """Create and return all robot tools."""
        
        # Reference to the node and topic variables for use in the closure
        node = self.node
        arming_service = self.arming_service
        mode_service = self.mode_service
        setpoint_topic = self.setpoint_topic
        
                    ######################## Takeoff Tool Starts ########################

        @tool
        def takeoff(altitude: float = 5.0) -> str:
            """
            Command the drone to take off to the specified altitude using MAVROS.
            
            This will arm the drone, set OFFBOARD mode, and command it to ascend.
            The function creates persistent service clients and publishers if they
            don't already exist, and manages the continuous sending of setpoints.
            The drone will take off from its current horizontal position.
            
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
            
            # Check if we're connected to the FCU
            if not hasattr(node, 'current_state') or not node.current_state.connected:
                return "Not connected to flight controller. Please check MAVROS connection."
            
            # Get current state information
            already_armed = hasattr(node, 'current_state') and node.current_state.armed
            already_offboard = hasattr(node, 'current_state') and node.current_state.mode == "OFFBOARD"
            
            # Check if already at or above target altitude
            current_alt = 0.0
            if hasattr(node, 'current_pose'):
                current_alt = node.current_pose.pose.position.z
                if current_alt >= altitude:
                    return f"Drone is already at or above target altitude. Current altitude: {current_alt:.2f}m"
            
            # Log the takeoff plan based on current state
            if already_armed and already_offboard:
                node.get_logger().info(f"Drone already armed and in OFFBOARD mode. Commanding altitude change to {altitude}m...")
            else:
                node.get_logger().info(f"Initiating complete takeoff sequence to altitude {altitude}m...")
            
            # Step 1: Create service clients for arming and mode setting (if they don't exist)
            if not hasattr(node, 'arming_client'):
                node.arming_client = node.create_client(
                    CommandBool, 
                    arming_service
                )
                node.get_logger().info(f"Created arming service client for {arming_service}")
                
            if not hasattr(node, 'mode_client'):
                node.mode_client = node.create_client(
                    SetMode, 
                    mode_service
                )
                node.get_logger().info(f"Created mode service client for {mode_service}")
            
            # Wait for services to be available
            timeout = 5.0  # seconds
            if not node.arming_client.wait_for_service(timeout_sec=timeout):
                return f"Arming service {arming_service} not available. Takeoff aborted."
            
            if not node.mode_client.wait_for_service(timeout_sec=timeout):
                return f"Set mode service {mode_service} not available. Takeoff aborted."
            
            # Step 2: Create a setpoint publisher for position control if it doesn't exist
            if not hasattr(node, 'setpoint_pub'):
                node.setpoint_pub = node.create_publisher(
                    PoseStamped,
                    setpoint_topic,
                    10
                )
                node.get_logger().info(f"Created setpoint publisher for {setpoint_topic}")
            
            # Create setpoint with target altitude but keep current horizontal position
            setpoint = PoseStamped()
            setpoint.header.frame_id = "map"
            
            # Use current X,Y coordinates instead of fixed 0,0
            if hasattr(node, 'current_pose'):
                setpoint.pose.position.x = node.current_pose.pose.position.x
                setpoint.pose.position.y = node.current_pose.pose.position.y
            else:
                setpoint.pose.position.x = 0.0
                setpoint.pose.position.y = 0.0
                
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
            
            # Step 4: Request arming if not already armed
            if not hasattr(node, 'current_state') or not node.current_state.armed:
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
            else:
                node.get_logger().info("Drone is already armed, skipping arming step")
            
            # Step 5: Set mode to OFFBOARD if not already in OFFBOARD
            if not hasattr(node, 'current_state') or node.current_state.mode != "OFFBOARD":
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
            else:
                node.get_logger().info("Drone is already in OFFBOARD mode, skipping mode change")
            
            # Step 6: Store the current target setpoint on the node
            node.target_setpoint = setpoint #stores the target flight position on the ROS node object
                        
            # Step 7: Start continuous setpoint publishing if not already running
            if not hasattr(node, 'setpoint_thread') or not node.setpoint_thread.is_alive():
                # Initialize the control flag if it doesn't exist
                if not hasattr(node, 'publish_setpoints'):
                    node.publish_setpoints = True
                
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
            
            # Step 8: Start altitude monitoring thread
            if not hasattr(node, 'altitude_monitor_thread') or not node.altitude_monitor_thread.is_alive():
                def altitude_monitor():
                    """Monitor actual altitude vs target altitude"""
                    rate = node.create_rate(1)  # 1 Hz check rate
                    
                    # Success criteria: within 0.2m of target for 3 consecutive checks
                    success_count = 0
                    target_z = altitude
                    start_time = time.time()
                    max_wait_time = 20  # seconds
                    
                    while node.running and time.time() - start_time < max_wait_time:
                        if hasattr(node, 'current_pose'):
                            current_z = node.current_pose.pose.position.z
                            diff = abs(current_z - target_z)
                            
                            # Log progress
                            progress = min(100, int((current_z / target_z) * 100)) if target_z > 0 else 0
                            node.get_logger().info(f"Altitude: {current_z:.2f}m / {target_z:.2f}m ({progress}%)")
                            
                            if diff < 0.2:  # Within 20cm of target
                                success_count += 1
                                if success_count >= 3:
                                    node.get_logger().info(f"Target altitude reached and stable at {current_z:.2f}m")
                                    return
                            else:
                                success_count = 0
                                
                        rate.sleep()
                    
                    if time.time() - start_time >= max_wait_time:
                        node.get_logger().warning("Altitude monitoring timed out. Check drone status.")
                
                # Create and start the monitoring thread
                node.altitude_monitor_thread = threading.Thread(target=altitude_monitor)
                node.altitude_monitor_thread.daemon = True
                node.altitude_monitor_thread.start()
                node.get_logger().info("Started altitude monitoring")
                        
            return (
                f"Takeoff sequence initiated successfully!\n\n"
                f"• Armed: ✓\n"
                f"• Mode: OFFBOARD\n"
                f"• Current altitude: {current_alt:.1f}m\n"
                f"• Target altitude: {altitude:.1f}m\n"
                f"• Maintaining current horizontal position\n\n"
                f"The drone should now be climbing to the target altitude. Monitoring progress..."
            )
        
                    ######################## Takeoff Tool Ends ########################

                    ######################## Landing Tool Starts ########################
        @tool
        def land() -> str:
            """
            Command the drone to land using the AUTO.LAND flight mode.
            
            This uses the flight controller's built-in landing sequence,
            which provides a controlled descent and automatic motor shutdown
            after touchdown. The function monitors the landing progress.
            
            Returns:
                str: Status message about the landing command, or error details if landing failed
            """
            # Check if we're connected to the FCU
            if not hasattr(node, 'current_state') or not node.current_state.connected:
                return "Not connected to flight controller. Please check MAVROS connection."
            
            # Check if drone is already on the ground
            current_alt = 0.0
            if hasattr(node, 'current_pose'):
                current_alt = node.current_pose.pose.position.z
                if current_alt < 0.1:  # If drone is already very close to the ground
                    return "Drone appears to be already on the ground (altitude less than 0.1m)."
            
            # Verify that we have the mode client for changing flight modes
            if not hasattr(node, 'mode_client'):
                node.mode_client = node.create_client(
                    SetMode, 
                    mode_service
                )
                node.get_logger().info(f"Created mode service client for {mode_service}")
            
            # Wait for service to be available
            timeout = 5.0  # seconds
            if not node.mode_client.wait_for_service(timeout_sec=timeout):
                return f"Set mode service {mode_service} not available. Landing aborted."
            
            # Request mode change to AUTO.LAND
            node.get_logger().info(f"Setting AUTO.LAND mode for autonomous landing...")
            
            try:
                mode_request = SetMode.Request()
                mode_request.custom_mode = "AUTO.LAND"
                
                future = node.mode_client.call_async(mode_request)
                
                # Wait for result with timeout
                start_time = time.time()
                while time.time() - start_time < timeout and not future.done():
                    time.sleep(0.1)
                
                if not future.done():
                    return "Set mode request timed out. Landing process may be unreliable."
                
                mode_response = future.result()
                if not mode_response.mode_sent:
                    return "Failed to set AUTO.LAND mode. Landing process failed."
                
                node.get_logger().info("AUTO.LAND mode set successfully, drone is now landing autonomously")
            except Exception as e:
                node.get_logger().error(f"Mode setting failed with error: {str(e)}")
                return f"Setting AUTO.LAND mode failed with error: {str(e)}"
            
            # Start landing monitor thread if not already running
            if not hasattr(node, 'landing_monitor_thread') or not node.landing_monitor_thread.is_alive():
                def landing_monitor():
                    """Monitor landing progress"""
                    rate = node.create_rate(1)  # 1 Hz check rate
                    
                    # Success criteria: on ground (altitude near 0) and disarmed
                    initial_alt = current_alt
                    if initial_alt < 0.1:  # If we don't have a valid starting altitude
                        initial_alt = 1.0  # Assume a reasonable default
                    
                    start_time = time.time()
                    max_wait_time = 60  # seconds
                    
                    while node.running and time.time() - start_time < max_wait_time:
                        if hasattr(node, 'current_pose') and hasattr(node, 'current_state'):
                            current_z = node.current_pose.pose.position.z
                            is_armed = node.current_state.armed
                            
                            # Calculate landing progress (100% = at ground level)
                            # Ensure we don't divide by zero
                            if initial_alt > 0:
                                progress = min(100, int(100 - (current_z / initial_alt) * 100))
                            else:
                                progress = 100 if current_z < 0.1 else 0
                            
                            node.get_logger().info(f"Landing: Altitude {current_z:.2f}m ({progress}% complete)")
                            
                            # Check if we've landed and disarmed
                            if current_z < 0.1 and not is_armed:
                                node.get_logger().info("Landing complete! Drone is on the ground and disarmed.")
                                return
                            
                            # Check if we're on ground but still armed (common for some systems)
                            if current_z < 0.1 and is_armed and time.time() - start_time > 5:
                                node.get_logger().info("Drone appears to be on the ground but still armed.")
                                return
                                
                        rate.sleep()
                    
                    if time.time() - start_time >= max_wait_time:
                        node.get_logger().warning("Landing monitoring timed out. Check drone status.")
                
                # Create and start the monitoring thread
                node.landing_monitor_thread = threading.Thread(target=landing_monitor)
                node.landing_monitor_thread.daemon = True
                node.landing_monitor_thread.start()
                node.get_logger().info("Started landing monitoring")
            
            return (
                f"Landing sequence initiated successfully!\n\n"
                f"• AUTO.LAND mode activated\n"
                f"• Starting altitude: {current_alt:.1f}m\n"
                f"• Landing control handled by flight controller\n\n"
                f"The drone is now descending for landing. Progress will be reported until touchdown."
            )
        
                    ######################## Landing Tool Ends ########################
                    
        # Return the tools
        return [
                takeoff,
                land
               ]