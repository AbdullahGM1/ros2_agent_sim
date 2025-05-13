#!/usr/bin/env python3
"""
Rich command-line interface for the Drone Agent.
Basic implementation to fix the 'setup_logging_capture' error.
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
from rich.box import ROUNDED
import cv2
import re
import asyncio


class LogCapture(logging.Handler):
    """Custom log handler that captures logs for the rich console."""
    
    def __init__(self, log_queue):
        super().__init__()
        self.log_queue = log_queue
        self.setFormatter(logging.Formatter('%(levelname)s: %(message)s'))
        
    def emit(self, record):
        log_entry = self.format(record)
        timestamp = datetime.fromtimestamp(record.created).strftime('%H:%M:%S')
        
        if record.levelno >= logging.ERROR:
            styled_log = f"[bold red][{timestamp}] {log_entry}[/bold red]"
        elif record.levelno >= logging.WARNING:
            styled_log = f"[yellow][{timestamp}] {log_entry}[/yellow]"
        elif record.levelno >= logging.INFO:
            styled_log = f"[green][{timestamp}] {log_entry}[/green]"
        else:
            styled_log = f"[dim][{timestamp}] {log_entry}[/dim]"
        
        self.log_queue.put(styled_log)


class StdoutCapture:
    """Capture stdout for rich console."""
    
    def __init__(self, log_queue):
        self.log_queue = log_queue
        self.terminal = sys.stdout
        
    def write(self, message):
        self.terminal.write(message)
        if message and message.strip() and not message.isspace():
            timestamp = datetime.now().strftime('%H:%M:%S')
            styled_message = f"[blue][{timestamp}] {message.strip()}[/blue]"
            self.log_queue.put(styled_message)
            
    def flush(self):
        self.terminal.flush()


class StderrCapture:
    """Capture stderr for rich console."""
    
    def __init__(self, log_queue):
        self.log_queue = log_queue
        self.terminal = sys.stderr
        
    def write(self, message):
        self.terminal.write(message)
        if message and message.strip() and not message.isspace():
            timestamp = datetime.now().strftime('%H:%M:%S')
            styled_message = f"[bold red][{timestamp}] {message.strip()}[/bold red]"
            self.log_queue.put(styled_message)
            
    def flush(self):
        self.terminal.flush()


class RichCLI:
    """Rich command-line interface for the Drone Agent Node."""
    
    def __init__(self, node):
        self.node = node
        self.console = Console()
        self.command_history = []
        self.history_index = 0
        self.log_queue = Queue()
        self.log_messages = []
        self.max_log_lines = 100
        
        # Set up logging capture
        self.setup_logging_capture()
        
        # Example drone commands
        self.examples = [
            "takeoff 5.0",
            "land",
            "go to position 10.0 5.0 3.0",
            "show me what you see",
            "what's your current position?",
            "stop camera",
        ]
        
        # Command handlers
        self.command_handlers = {
            "help": self.show_help,
            "status": self.show_status,
            "stop": self.emergency_stop,
            "examples": self.show_examples,
            "clear": self.clear_screen,
            "logs": self.show_logs,
            "exit": self.exit_program,
            "quit": self.exit_program,
        }
        
        # Set up signal handlers
        signal.signal(signal.SIGINT, self.handle_interrupt)
        
    def setup_logging_capture(self):
        """Set up capture of logs and terminal output."""
        root_logger = logging.getLogger()
        for handler in root_logger.handlers[:]:
            root_logger.removeHandler(handler)
        
        log_handler = LogCapture(self.log_queue)
        root_logger.addHandler(log_handler)
        root_logger.setLevel(logging.INFO)
        
        sys.stdout = StdoutCapture(self.log_queue)
        sys.stderr = StderrCapture(self.log_queue)
        
        self.log_thread = threading.Thread(target=self.process_logs, daemon=True)
        self.log_thread.start()
        
    def process_logs(self):
        """Process incoming logs in a separate thread."""
        while True:
            try:
                log_message = self.log_queue.get()
                self.log_messages.append(log_message)
                if len(self.log_messages) > self.max_log_lines:
                    self.log_messages.pop(0)
                self.log_queue.task_done()
            except Exception:
                pass
            time.sleep(0.1)
            
    def show_greeting(self):
        """Display the greeting message."""
        greeting = Text("\n✈️ DRONE CONTROL AGENT ✈️\n")
        greeting.stylize("bold blue")
        
        commands = ", ".join(sorted(self.command_handlers.keys()))
        greeting.append(f"Available commands: {commands}", style="italic cyan")
        
        self.console.print(greeting)
        
    def show_help(self):
        """Display help information."""
        try:
            help_table = Table(title="Drone Commands", box=ROUNDED, border_style="blue")

            help_table.add_column("Command", style="cyan")
            help_table.add_column("Description", style="green")
            help_table.add_column("Example", style="yellow italic")
            
            # Basic commands
            help_table.add_row("help", "Show this help message", "help")
            help_table.add_row("status", "Show drone status", "status")
            help_table.add_row("stop", "Emergency stop the drone", "stop")
            help_table.add_row("examples", "Show example commands", "examples")
            help_table.add_row("logs", "Show recent system logs", "logs")
            help_table.add_row("clear", "Clear the screen", "clear")
            help_table.add_row("exit, quit", "Exit the program", "exit")
            
            # Drone commands
            help_table.add_section()
            help_table.add_row("takeoff [height]", "Take off to specified height", "takeoff 5.0")
            help_table.add_row("land", "Land the drone", "land")
            help_table.add_row("go to position [x] [y] [z]", "Navigate to coordinates", "go to position 10.0 5.0 3.0")
            
            # Camera commands
            help_table.add_section()
            help_table.add_row("show camera", "Display drone camera feed", "show camera")
            help_table.add_row("stop camera", "Stop camera feed", "stop camera")
            help_table.add_row("what's my position", "Show current position", "what's my position")
            
            # Original note remains the same
            note = "\nYou can use natural language to control the drone. The commands listed are just examples."
            
            self.console.print(help_table)
            self.console.print(note)
        
        except Exception as e:
            self.console.print(f"[red]Error in show_help: {str(e)}[/red]")
            import traceback
            traceback.print_exc()
        
    def show_examples(self):
        """Show example commands the user can try."""
        examples_panel = Panel(
            "\n".join([f"• {example}" for example in self.examples]),
            title="Example Commands",
            border_style="green",
            expand=False
        )
        self.console.print(examples_panel)
        
    def clear_screen(self):
        """Clear the terminal screen."""
        os.system('cls' if os.name == 'nt' else 'clear')
        
    def emergency_stop(self):
        """Perform emergency stop of the drone."""
        try:
            from geometry_msgs.msg import Twist
            twist = Twist()
            self.node.cmd_vel_publisher.publish(twist)
            for _ in range(5):
                self.node.cmd_vel_publisher.publish(twist)
                time.sleep(0.01)
                
            stop_panel = Panel(
                "⚠️ Drone movement halted. All motors stopped.",
                title="EMERGENCY STOP",
                border_style="red",
                expand=False
            )
            self.console.print(stop_panel)
            return True
        except Exception as e:
            self.console.print(f"[red]Error during emergency stop: {str(e)}[/red]")
            return False
            
    def exit_program(self):
        """Exit the program gracefully."""
        self.console.print("[yellow]Shutting down...[/yellow]")
        
        self.emergency_stop()
        
        with self.node.camera_lock:
            if self.node.camera_active:
                self.node.camera_active = False
                
        if self.node.camera_thread is not None and self.node.camera_thread.is_alive():
            self.node.camera_thread.join(timeout=1.0)
            
        try:
            cv2.destroyAllWindows()
        except:
            pass
            
        self.node.running = False
        self.console.print("[green]Goodbye![/green]")
        sys.exit(0)
        
    def show_status(self):
        """Show current drone status."""
        try:
            # Get drone position
            pos_x = self.node.current_pose.pose.position.x
            pos_y = self.node.current_pose.pose.position.y
            pos_z = self.node.current_pose.pose.position.z
            
            status_table = Table(title="Drone Status", box=ROUNDED, border_style="blue")
            status_table.add_column("Parameter", style="cyan")
            status_table.add_column("Value", style="green")
            
            # Position info
            status_table.add_row("Position X", f"{pos_x:.2f} m")
            status_table.add_row("Position Y", f"{pos_y:.2f} m")
            status_table.add_row("Position Z (Altitude)", f"{pos_z:.2f} m")
            
            timestamp = datetime.now().strftime("%H:%M:%S")
            footer = Text(f"Status as of {timestamp}")
            footer.stylize("italic")
            
            self.console.print(status_table)
            self.console.print(footer)
            return True
            
        except Exception as e:
            self.console.print(f"[red]Error retrieving status: {str(e)}[/red]")
            return False
            
    def show_logs(self):
        """Display the captured log messages."""
        if not self.log_messages:
            self.console.print("[yellow]No log messages captured yet.[/yellow]")
            return
            
        log_text = "\n".join(self.log_messages[-40:])
        log_panel = Panel(
            log_text,
            title=f"System Logs (last {min(40, len(self.log_messages))} entries)",
            border_style="blue",
            expand=False
        )
        self.console.print(log_panel)
        
    def handle_interrupt(self, sig, frame):
        """Handle SIGINT (Ctrl+C) gracefully."""
        self.console.print("\n[yellow]Interrupted. Type 'exit' to quit or 'stop' for emergency stop.[/yellow]")
        
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
        """Process a user command."""
        if not command or command.isspace():
            return
            
        self.command_history.append(command)
        self.history_index = len(self.command_history)
        
        command_lower = command.lower().strip()
        if command_lower == "help":
            # Direct call instead of using handlers
            self.show_help()
            return
        elif command_lower in self.command_handlers:
            self.command_handlers[command_lower]()
            return
            
        self.console.print(f"[cyan]Processing: {command}[/cyan]")
        
        result = [None]
        error = [None]
        
        def process_command():
            try:
                result[0] = self.node.agent.invoke(command)
            except Exception as e:
                error[0] = str(e)
                
        agent_thread = threading.Thread(target=process_command)
        agent_thread.daemon = True
        agent_thread.start()
        
        with self.console.status("[yellow]Thinking...[/yellow]", spinner="dots") as status:
            timeout = 60
            start_time = time.time()
            
            while agent_thread.is_alive() and time.time() - start_time < timeout:
                await asyncio.sleep(0.1)
                
        if agent_thread.is_alive():
            self.console.print("[red]Response taking too long! Consider using emergency stop if robot is moving.[/red]")
            return
        elif error[0]:
            self.console.print(Panel(f"Error: {error[0]}", title="Error", border_style="red"))
            return
            
        response = result[0]
        thinking, clean_response = self.extract_thinking(response)
        
        if thinking:
            self.console.print(Panel(
                Markdown(thinking), 
                title="Thinking Process",
                border_style="yellow"
            ))
            
        self.console.print(Panel(
            Markdown(clean_response), 
            title="Drone Response",
            border_style="green"
        ))
        
    async def run(self):
        """Run the main command loop."""
        self.clear_screen()
        self.show_greeting()
        
        while True:
            try:
                self.console.print("[bold green]➤ Drone >[/bold green] ", end="")
                command = input()
                await self.process_command(command)
                
            except KeyboardInterrupt:
                self.console.print("\n[yellow]Command interrupted. Type 'stop' for emergency stop.[/yellow]")
                continue
            except EOFError:
                self.exit_program()
            except Exception as e:
                self.console.print(f"[red]Error: {str(e)}[/red]")