#!/usr/bin/env python3
"""
Robot tools for the ROSA Agent.
This module contains tools for drone control, including takeoff and landing.
"""

import time
import threading
import math
from geometry_msgs.msg import PoseStamped
from langchain.agents import tool
from mavros_msgs.srv import CommandBool, SetMode  # type: ignore
from mavros_msgs.msg import State  # type: ignore


class RobotTools:
    """Collection of tools for robot control."""
    
    def __init__(self, node):
        """
        Initialize the RobotTools with necessary ROS topics and services.
        
        Args:
            node: The ROS2 node object
        """
        self.node = node
        
        # Get topic/service names from the node
        self.state_topic = node.state_topic
        self.arming_service = node.arming_service
        self.mode_service = node.mode_service
        self.setpoint_topic = node.setpoint_topic
        
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
            
            # Reset any previous landing state
            if hasattr(node, 'publish_setpoints'):
                node.publish_setpoints = True  # Ensure setpoint publishing is enabled
                
            # Force a clean mode transition if coming from a landing
            if hasattr(node, 'current_state') and node.current_state.mode in ["LAND", "RTL", "AUTO.LAND"]:
                node.get_logger().info("Transitioning from landing mode - performing reset...")
                try:
                    # First try setting a neutral mode like STABILIZED
                    mode_request = SetMode.Request()
                    mode_request.custom_mode = "STABILIZED"
                    
                    future = node.mode_client.call_async(mode_request)
                    # Wait for result
                    start_time = time.time()
                    while time.time() - start_time < timeout and not future.done():
                        time.sleep(0.1)
                        
                    # Now we'll continue with regular takeoff sequence
                    time.sleep(1.0)  # Brief pause before next steps
                except Exception as e:
                    node.get_logger().warning(f"Mode reset attempt had an issue: {str(e)} - continuing anyway")
            
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
                else:
                    # Ensure it's enabled in case it was disabled during landing
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
                # Store results to be returned to LLM
                result_message = ["Operation in progress"]  # Use a list for thread-safe modification
                operation_completed = threading.Event()  # Event to signal completion
                
                def altitude_monitor():
                    """Monitor actual altitude vs target altitude"""
                    rate = node.create_rate(1)  # 1 Hz check rate
                    
                    # Success criteria: within 0.2m of target for 3 consecutive checks
                    success_count = 0
                    target_z = altitude
                    start_time = time.time()
                    max_wait_time = 30  # Extended from 20 to 30 seconds
                    initial_alt = current_alt
                    last_progress_time = start_time
                    
                    # Check if altitude is increasing at all
                    while node.running and time.time() - start_time < max_wait_time:
                        if hasattr(node, 'current_pose'):
                            current_z = node.current_pose.pose.position.z
                            diff = abs(current_z - target_z)
                            
                            # Log progress
                            progress = min(100, int((current_z / target_z) * 100)) if target_z > 0 else 0
                            node.get_logger().info(f"Altitude: {current_z:.2f}m / {target_z:.2f}m ({progress}%)")
                            
                            # Check if making progress (altitude increasing)
                            if current_z > initial_alt + 0.2:
                                node.get_logger().info("Confirmed altitude is increasing - takeoff in progress")
                            elif time.time() - last_progress_time > 5.0:
                                # If no progress in 5 seconds, try re-issuing the command
                                node.get_logger().warning("No altitude change detected - refreshing setpoint")
                                if hasattr(node, 'setpoint_pub') and hasattr(node, 'target_setpoint'):
                                    node.target_setpoint.header.stamp = node.get_clock().now().to_msg()
                                    for _ in range(5):  # Send multiple times to ensure it's received
                                        node.setpoint_pub.publish(node.target_setpoint)
                                        time.sleep(0.1)
                                last_progress_time = time.time()
                            
                            if diff < 0.2:  # Within 20cm of target
                                success_count += 1
                                if success_count >= 3:
                                    node.get_logger().info(f"Target altitude reached and stable at {current_z:.2f}m")
                                    result_message[0] = (
                                        f"Takeoff completed successfully!\n\n"
                                        f"• Final altitude: {current_z:.2f}m\n"
                                        f"• Target altitude: {altitude:.1f}m\n"
                                        f"• Stability: Maintained for {success_count} seconds\n\n"
                                        f"The drone is now hovering at the target altitude."
                                    )
                                    operation_completed.set()
                                    return
                            else:
                                success_count = 0
                                
                        rate.sleep()
                    
                    if time.time() - start_time >= max_wait_time:
                        node.get_logger().warning("Altitude monitoring timed out. Check drone status.")
                        result_message[0] = (
                            f"Takeoff initiated but did not reach target altitude within {max_wait_time} seconds.\n\n"
                            f"• Current altitude: Approximately {current_alt:.1f}m\n"
                            f"• Target altitude: {altitude:.1f}m\n\n"
                            f"The drone may be experiencing control issues. Please check system status."
                        )
                        operation_completed.set()
                
                # Create and start the monitoring thread
                node.altitude_monitor_thread = threading.Thread(target=altitude_monitor)
                node.altitude_monitor_thread.daemon = True
                node.altitude_monitor_thread.start()
                node.get_logger().info("Started altitude monitoring")
                
                # Wait for operation to complete with a timeout
                wait_success = operation_completed.wait(timeout=30)  # Wait for up to 30 seconds
                
                if not wait_success:
                    # If event wasn't set, we hit the wait timeout
                    result_message[0] = (
                        f"Takeoff initiated but monitoring thread didn't complete in time.\n\n"
                        f"• Command sent successfully\n"
                        f"• Please check drone status\n\n"
                        f"The drone may still be climbing to the target altitude."
                    )
                
                # Return the message from the monitor thread
                return result_message[0]
        
                    ######################## Takeoff Tool Ends ########################

                    ######################## Landing Tool Starts ########################
        @tool
        def land() -> str:
            """
            Command the drone to land using the LAND flight mode.
            
            IMPORTANT: This tool MUST ALWAYS be called when a user requests to land,
            regardless of what you think the drone's current state is. Never skip
            calling this tool - it will check the drone's state itself and handle
            all necessary operations.
            
            This uses the flight controller's built-in landing sequence,
            which provides a controlled descent and automatic motor shutdown
            after touchdown. The function monitors the landing progress.
            
            IMPORTANT: This command will interrupt any ongoing takeoff sequence!
            The tool should be called immediately when the user requests landing,
            without asking for confirmation.
            
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
                    # Even if already on ground, still perform the landing sequence
                    # This is to ensure proper mode transition and to provide feedback
                    node.get_logger().info("Drone appears to be near ground level, but landing command will still be processed to ensure proper state transition.")
                    
            # CRITICAL: First, forcefully stop OFFBOARD position control if active
            if hasattr(node, 'publish_setpoints'):
                node.get_logger().info("Stopping any active position control setpoint publishing...")
                node.publish_setpoints = False  # This will stop the thread from publishing
                time.sleep(1.0)  # Increased pause to ensure publishing stops
                
            # ALTERNATIVE: Also try RTL mode if LAND doesn't work consistently
            node.get_logger().info("Attempting to initiate landing sequence...")
            
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
            
            # Try multiple landing modes in sequence if needed
            landing_modes = ["LAND", "RTL", "AUTO.LAND"]  # Try different modes that might work
            success = False
            error_messages = []
            
            for mode_name in landing_modes:
                try:
                    node.get_logger().info(f"Attempting to set {mode_name} mode...")
                    mode_request = SetMode.Request()
                    mode_request.custom_mode = mode_name
                    
                    future = node.mode_client.call_async(mode_request)
                    
                    # Wait for result with timeout
                    start_time = time.time()
                    while time.time() - start_time < timeout and not future.done():
                        time.sleep(0.1)
                    
                    if not future.done():
                        error_messages.append(f"{mode_name} mode request timed out")
                        continue
                    
                    mode_response = future.result()
                    if not mode_response.mode_sent:
                        error_messages.append(f"Failed to set {mode_name} mode")
                        continue
                    
                    node.get_logger().info(f"{mode_name} mode set successfully!")
                    success = True
                    break  # Exit the loop if successful
                    
                except Exception as e:
                    error_messages.append(f"Error setting {mode_name} mode: {str(e)}")
                    continue
            
            if not success:
                errors = "\n".join(error_messages)
                return f"Failed to set landing mode. Tried multiple modes but all failed:\n{errors}"
            
            # Start landing monitor thread if not already running
            if not hasattr(node, 'landing_monitor_thread') or not node.landing_monitor_thread.is_alive():
                # Store results to be returned to LLM
                result_message = ["Landing in progress"]  # Use a list for thread-safe modification
                operation_completed = threading.Event()  # Event to signal completion
                
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
                                result_message[0] = (
                                    f"Landing completed successfully!\n\n"
                                    f"• Drone is on the ground\n"
                                    f"• Motors disarmed\n"
                                    f"• Final altitude: {current_z:.2f}m\n\n"
                                    f"The landing sequence has finished and the drone is ready for the next command."
                                )
                                operation_completed.set()
                                return
                            
                            # Check if we're on ground but still armed (common for some systems)
                            if current_z < 0.1 and is_armed and time.time() - start_time > 5:
                                node.get_logger().info("Drone appears to be on the ground but still armed.")
                                result_message[0] = (
                                    f"Landing mostly complete.\n\n"
                                    f"• Drone is on the ground\n"
                                    f"• Motors still armed\n"
                                    f"• Final altitude: {current_z:.2f}m\n\n"
                                    f"The drone has landed but the motors remain armed. This is normal for some flight controllers."
                                )
                                operation_completed.set()
                                return
                                
                        rate.sleep()
                    
                    if time.time() - start_time >= max_wait_time:
                        node.get_logger().warning("Landing monitoring timed out. Check drone status.")
                        result_message[0] = (
                            f"Landing initiated but monitoring timed out after {max_wait_time} seconds.\n\n"
                            f"• Last altitude: {current_alt:.2f}m\n"
                            f"• Landing mode is still active\n\n"
                            f"The drone may still be in the process of landing. Please check system status."
                        )
                        operation_completed.set()
                
                # Create and start the monitoring thread
                node.landing_monitor_thread = threading.Thread(target=landing_monitor)
                node.landing_monitor_thread.daemon = True
                node.landing_monitor_thread.start()
                node.get_logger().info("Started landing monitoring")
                
                # Wait for operation to complete with a timeout
                wait_success = operation_completed.wait(timeout=60)  # Wait for up to 60 seconds for landing
                
                if not wait_success:
                    # If event wasn't set, we hit the wait timeout
                    result_message[0] = (
                        f"Landing initiated but monitoring thread didn't complete in time.\n\n"
                        f"• Landing mode activated successfully\n"
                        f"• Please check drone status\n\n"
                        f"The drone may still be in the process of landing."
                    )
                
                # Return the message from the monitor thread
                return result_message[0]
        
                    ######################## Landing Tool Ends ########################
                    
        # Return the tools
        return [
                takeoff,
                land
               ]