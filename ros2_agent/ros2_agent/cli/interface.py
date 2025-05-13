#!/usr/bin/env python3
"""
Rich command-line interface for the SAR Multi-Robot Agent.
This module provides a user-friendly interface for controlling robots in search and rescue operations.
"""

import sys
import threading
import time
import os
import signal
import logging
from queue import Queue
from datetime import datetime
from rich.console import Console
from rich.panel import Panel
from rich.markdown import Markdown
from rich.text import Text
from rich.table import Table
from rich.box import ROUNDED, HEAVY
from rich.live import Live
from rich.logging import RichHandler
from rich.layout import Layout
from rich.progress import Progress, SpinnerColumn, TextColumn
import cv2
import re
import asyncio


# Add a specialized SAR log handler
class SARLogCapture(logging.Handler):
    """Custom log handler that captures logs with SAR-specific formatting."""
    
    def __init__(self, log_queue):
        super().__init__()
        self.log_queue = log_queue
        self.setFormatter(logging.Formatter('%(levelname)s: %(message)s'))
        
    def emit(self, record):
        log_entry = self.format(record)
        timestamp = datetime.fromtimestamp(record.created).strftime('%H:%M:%S')
        
        # Add priority markers for SAR-related logs
        if any(keyword in log_entry.lower() for keyword in 
               ['survivor', 'detected', 'person', 'victim', 'found', 'heat signature']):
            styled_log = f"[bold white on red][ALERT][{timestamp}] {log_entry}[/bold white on red]"
        elif record.levelno >= logging.ERROR:
            styled_log = f"[bold red][{timestamp}] {log_entry}[/bold red]"
        elif record.levelno >= logging.WARNING:
            styled_log = f"[yellow][{timestamp}] {log_entry}[/yellow]"
        elif record.levelno >= logging.INFO:
            styled_log = f"[green][{timestamp}] {log_entry}[/green]"
        else:
            styled_log = f"[dim][{timestamp}] {log_entry}[/dim]"
        
        self.log_queue.put(styled_log)


# Existing StdoutCapture and StderrCapture classes remain unchanged

class RichCLI:
    """Rich command-line interface for the SAR Multi-Robot Agent."""
    
    def __init__(self, node):
        self.node = node
        self.console = Console()
        self.command_history = []
        self.history_index = 0
        self.log_queue = Queue()
        self.log_messages = []
        self.max_log_lines = 200  # Increased for SAR operations
        self.mission_start_time = datetime.now()
        self.waypoints = []  # Store discovered points of interest
        self.search_areas = []  # Track search patterns
        
        # Set up logging capture
        self.setup_logging_capture()
        
        # Updated examples for SAR operations
        self.examples = [
            "search the area between coordinates (5,5) and (10,10)",
            "send drone1 to scan the north perimeter", 
            "move go2_1 forward 2 meters",
            "check for heat signatures in building A",
            "show camera feed from all robots",
            "mark current location as survivor found",
            "establish search grid 20x20 meters",
            "what's the status of all robots?",
        ]
        
        # Enhanced command handlers with SAR-specific commands
        self.command_handlers = {
            "help": self.show_help,
            "status": self.show_status,
            "stop": self.emergency_stop,
            "emergency": self.emergency_stop,  # Alias for quick access
            "examples": self.show_examples,
            "clear": self.clear_screen,
            "logs": self.show_logs,
            "mission": self.show_mission_stats,  # New command
            "waypoints": self.show_waypoints,    # New command
            "coverage": self.show_search_coverage,  # New command
            "exit": self.exit_program,
            "quit": self.exit_program,
        }
        
        # Set up signal handlers
        signal.signal(signal.SIGINT, self.handle_interrupt)
        
    # Existing setup_logging_capture and process_logs methods remain unchanged
            
    def show_greeting(self):
        """Display the SAR mission greeting message."""
        greeting = Text("\n⚠️ SEARCH AND RESCUE MISSION CONTROL ⚠️\n")
        greeting.stylize("bold white on red")
        
        commands = ", ".join(sorted(self.command_handlers.keys()))
        greeting.append(f"Available commands: {commands}", style="italic cyan")
        
        # Add mission start information
        mission_info = f"\nMission started: {self.mission_start_time.strftime('%Y-%m-%d %H:%M:%S')}"
        greeting.append(mission_info, style="yellow")
        
        self.console.print(greeting)
        
    def show_help(self):
        """Display help information with SAR-specific commands."""
        try:
            help_table = Table(title="SAR Robot Commands", box=HEAVY, border_style="red")

            help_table.add_column("Command", style="cyan")
            help_table.add_column("Description", style="green")
            help_table.add_column("Example", style="yellow italic")
            
            # Basic commands
            help_table.add_row("help", "Show this help message", "help")
            help_table.add_row("status", "Show all robot statuses", "status")
            help_table.add_row("stop, emergency", "Emergency stop all robots", "stop")
            help_table.add_row("examples", "Show example commands", "examples")
            help_table.add_row("logs", "Show recent system logs", "logs")
            help_table.add_row("mission", "Show mission statistics", "mission")
            help_table.add_row("waypoints", "List marked points of interest", "waypoints")
            help_table.add_row("coverage", "Show search coverage map", "coverage")
            help_table.add_row("clear", "Clear the screen", "clear")
            help_table.add_row("exit, quit", "Exit the program", "exit")
            
            # Search commands
            help_table.add_section()
            help_table.add_row("search [area] [parameters]", "Begin search pattern", "search grid 10x10 meters")
            help_table.add_row("scan [location]", "Perform detailed scan", "scan building entrance")
            help_table.add_row("mark [type] [description]", "Mark point of interest", "mark survivor possible heat signature")
            
            # Movement commands
            help_table.add_section()
            help_table.add_row("move [robot] [direction] [distance]", "Move specified robot", "move go2_1 forward 2")
            help_table.add_row("send [robot] to [location]", "Navigate to location", "send drone1 to north perimeter")
            help_table.add_row("patrol [area] [parameters]", "Patrol an area", "patrol building A perimeter")
            
            # Sensor commands
            help_table.add_section()
            help_table.add_row("show camera [robot]", "Display robot camera feed", "show camera drone1")
            help_table.add_row("stop camera [robot]", "Stop camera feed", "stop camera go2_1")
            help_table.add_row("check [sensor] in [area]", "Use specialized sensors", "check for heat signatures in sector 2")
            
            note = "\nUse natural language to control robots. Type commands as you would speak them.\n"
            note += "Critical commands: 'emergency', 'mark survivor', 'show camera'"
            
            self.console.print(help_table)
            self.console.print(note)
        
        except Exception as e:
            self.console.print(f"[red]Error in show_help: {str(e)}[/red]")
            import traceback
            traceback.print_exc()
        
    def emergency_stop(self):
        """Perform emergency stop of all robots with enhanced visibility."""
        try:
            from geometry_msgs.msg import Twist
            twist = Twist()
            self.node.publisher_.publish(twist)
            for _ in range(5):
                self.node.publisher_.publish(twist)
                time.sleep(0.01)
                
            # Enhanced visibility for emergency stops
            self.console.print("\n")
            stop_panel = Panel(
                "⚠️ ALL ROBOT MOVEMENT HALTED ⚠️\n\nAll motors stopped. All operations suspended.",
                title="EMERGENCY STOP ACTIVATED",
                border_style="bold white on red",
                expand=False
            )
            self.console.print(stop_panel)
            
            # Log the emergency stop
            logging.warning("Emergency stop activated - all robots halted")
            
            return True
        except Exception as e:
            self.console.print(f"[red]Error during emergency stop: {str(e)}[/red]")
            return False
            
    def show_status(self):
        """Show current status of all robots with SAR-relevant information."""
        try:
            # Create a more comprehensive status display
            status_layout = Layout()
            status_layout.split_column(
                Layout(name="title"),
                Layout(name="robots", ratio=2),
                Layout(name="mission", ratio=1)
            )
            
            # Title section
            mission_duration = datetime.now() - self.mission_start_time
            hours, remainder = divmod(mission_duration.seconds, 3600)
            minutes, seconds = divmod(remainder, 60)
            
            title_text = Text("SEARCH AND RESCUE MISSION STATUS")
            title_text.stylize("bold white on blue")
            title_text.append(f"\nMission duration: {hours:02}:{minutes:02}:{seconds:02}")
            
            status_layout["title"].update(Panel(title_text))
            
            # Try to get robot information
            get_robot_pose = None
            for tool in self.node.agent.tools:
                if hasattr(tool, 'name') and 'get_robot_pose' in tool.name:
                    get_robot_pose = tool
                    break
            
            if get_robot_pose is None:
                self.console.print("[red]Error: Could not find get_robot_pose tool[/red]")
                return False
            
            # Robots status section (here we'd ideally gather status from all robots)
            robot_table = Table(box=ROUNDED, border_style="blue")
            robot_table.add_column("Robot ID", style="cyan")
            robot_table.add_column("Type", style="green")
            robot_table.add_column("Position", style="yellow")
            robot_table.add_column("Status", style="magenta")
            robot_table.add_column("Battery", style="red")
            robot_table.add_column("Current Task", style="blue")
            
            # For demonstration, let's add a simulated entry for each robot type
            # In a real implementation, you would query each robot for its status
            pose = get_robot_pose.invoke({})
            if "error" in pose:
                self.console.print(f"[red]Error getting pose: {pose['error']}[/red]")
                return False
                
            # Populate with actual robot data where available
            robot_table.add_row(
                "go2_1", 
                "Quadruped", 
                f"({pose['x']:.2f}, {pose['y']:.2f}, {pose['z']:.2f})", 
                "Active", 
                "92%", 
                "Searching sector 3"
            )
            
            # Add simulated data for additional robots
            # In a real implementation, you would get actual data from all robots
            robot_table.add_row(
                "drone1", 
                "Aerial", 
                "(12.45, 8.73, 15.20)", 
                "Active", 
                "78%", 
                "Perimeter surveillance"
            )
            
            status_layout["robots"].update(Panel(robot_table, title="Robot Status"))
            
            # Mission stats section
            mission_stats = Table(box=ROUNDED)
            mission_stats.add_column("Metric", style="cyan")
            mission_stats.add_column("Value", style="green")
            
            # These would be real metrics in a full implementation
            mission_stats.add_row("Area Covered", "1240 sq meters")
            mission_stats.add_row("Points of Interest", str(len(self.waypoints)))
            mission_stats.add_row("Potential Survivors", "3")
            mission_stats.add_row("Hazards Identified", "7")
            mission_stats.add_row("Search Completion", "64%")
            
            status_layout["mission"].update(Panel(mission_stats, title="Mission Progress"))
            
            # Render the full status display
            self.console.print(status_layout)
            
            return True
        except Exception as e:
            self.console.print(f"[red]Error retrieving status: {str(e)}[/red]")
            return False
    
    # New methods for SAR operations
    
    def show_mission_stats(self):
        """Display detailed mission statistics."""
        try:
            # This would pull from actual mission data in a full implementation
            mission_duration = datetime.now() - self.mission_start_time
            hours, remainder = divmod(mission_duration.seconds, 3600)
            minutes, seconds = divmod(remainder, 60)
            
            stats_panel = Panel(
                f"Mission Start: {self.mission_start_time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"Duration: {hours:02}:{minutes:02}:{seconds:02}\n"
                f"Search Area: 1240 sq meters\n"
                f"Coverage: 64%\n"
                f"Robots Deployed: 3\n"
                f"Points of Interest: {len(self.waypoints)}\n"
                f"Potential Survivors: 3\n"
                f"Confirmed Survivors: 1\n"
                f"Hazards Identified: 7\n",
                title="MISSION STATISTICS",
                border_style="blue",
                expand=False
            )
            self.console.print(stats_panel)
        except Exception as e:
            self.console.print(f"[red]Error showing mission stats: {str(e)}[/red]")
    
    def show_waypoints(self):
        """Show all marked points of interest."""
        if not self.waypoints:
            self.console.print("[yellow]No waypoints have been marked yet.[/yellow]")
            return
            
        waypoint_table = Table(title="Points of Interest", box=ROUNDED, border_style="green")
        waypoint_table.add_column("ID", style="cyan")
        waypoint_table.add_column("Type", style="yellow")
        waypoint_table.add_column("Coordinates", style="green")
        waypoint_table.add_column("Description", style="white")
        waypoint_table.add_column("Time", style="magenta")
        
        for i, waypoint in enumerate(self.waypoints):
            waypoint_table.add_row(
                str(i+1),
                waypoint.get("type", "Unknown"),
                f"({waypoint.get('x', '?')}, {waypoint.get('y', '?')}, {waypoint.get('z', '?')})",
                waypoint.get("description", ""),
                waypoint.get("time", "Unknown")
            )
            
        self.console.print(waypoint_table)
    
    def show_search_coverage(self):
        """Show a visualization of search coverage."""
        # In a full implementation, this would generate a visual map
        # For now, we'll just show a placeholder message
        self.console.print("[yellow]Search coverage visualization would appear here.[/yellow]")
        self.console.print("[green]This would show a map of the search area with covered regions highlighted.[/green]")
    
    # Modified existing methods
    
    def extract_thinking(self, response):
        """Extract thinking process from <think> tags and format it nicely."""
        thinking = ""
        response_text = response
        
        think_pattern = r'<think>([\s\S]*?)</think>'
        think_matches = re.findall(think_pattern, response)
        
        if think_matches:
            thinking = "\n".join(think_match.strip() for think_match in think_matches)
            response_text = re.sub(think_pattern, '', response).strip()
        
        return thinking, response_text
        
    async def process_command(self, command):
        """Process a user command with enhanced SAR feedback."""
        if not command or command.isspace():
            return
            
        self.command_history.append(command)
        self.history_index = len(self.command_history)
        
        command_lower = command.lower().strip()
        
        # Check for critical emergency stop command with any variation
        if "stop" in command_lower and any(word in command_lower for word in ["emergency", "immediately", "now", "all"]):
            self.emergency_stop()
            return
        
        # Check for waypoint marking commands
        if "mark" in command_lower and any(word in command_lower for word in ["survivor", "victim", "person", "found"]):
            # In a real implementation, this would get the current coordinates
            # and add a proper waypoint with timestamps
            self.waypoints.append({
                "type": "Survivor",
                "x": 10.5,
                "y": 15.2,
                "z": 0.0,
                "description": command.replace("mark", "").strip(),
                "time": datetime.now().strftime("%H:%M:%S")
            })
            self.console.print("[bold white on green]SURVIVOR LOCATION MARKED![/bold white on green]")
            
        # Handle regular commands through the command handlers
        if command_lower == "help":
            # Direct call instead of using handlers
            self.show_help()
            return
        elif command_lower in self.command_handlers:
            self.command_handlers[command_lower]()
            return
            
        self.console.print(f"[cyan]Processing: {command}[/cyan]")
        
        # Create progress display for SAR operations where time is critical
        with Progress(
            SpinnerColumn(),
            TextColumn("[yellow]Processing command - time is critical...[/yellow]"),
            console=self.console
        ) as progress:
            task = progress.add_task("Processing...", total=100)
            
            result = [None]
            error = [None]
            
            def process_command():
                try:
                    result[0] = self.node.agent.invoke(command)
                    # Simulate progress updates
                    for i in range(1, 101):
                        progress.update(task, completed=i)
                        time.sleep(0.01)  # Very short delay to show progress
                except Exception as e:
                    error[0] = str(e)
                    
            agent_thread = threading.Thread(target=process_command)
            agent_thread.daemon = True
            agent_thread.start()
            
            timeout = 30  # Shorter timeout for SAR operations
            start_time = time.time()
            
            while agent_thread.is_alive() and time.time() - start_time < timeout:
                await asyncio.sleep(0.1)
                
        if agent_thread.is_alive():
            self.console.print("[bold red]Response taking too long! Time-critical situation - consider emergency procedures.[/bold red]")
            return
        elif error[0]:
            self.console.print(Panel(f"Error: {error[0]}", title="Error", border_style="red"))
            return
            
        response = result[0]
        thinking, clean_response = self.extract_thinking(response)
        
        if thinking:
            # Make thinking collapsible in a SAR context where rapid response is key
            self.console.print("[dim blue]Reasoning process available (less important in emergency situations)[/dim blue]")
            
            # Check if the response contains urgent information
            if any(word in clean_response.lower() for word in ["survivor", "detected", "found", "emergency", "hazard"]):
                self.console.print(Panel(
                    Markdown(clean_response), 
                    title="⚠️ URGENT ROBOT RESPONSE ⚠️",
                    border_style="bold white on red"
                ))
            else:
                self.console.print(Panel(
                    Markdown(clean_response), 
                    title="Robot Response",
                    border_style="green"
                ))
    
    # The rest of the methods remain unchanged