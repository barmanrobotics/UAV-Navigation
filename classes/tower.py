"""
Tower Class for UAV Navigation

This class encapsulates all tower-related functionality including:
- Multi-drone communication and coordination
- GPS data reception and processing
- Command transmission to drones
- Obstacle avoidance coordination
- Flap control for landing platforms
- Distance calculations and collision detection
- Server management and client handling

Author: Generated from tower_pi folder functions
"""

import socket
import threading
import math
import os
import time
import json
import logging
import RPi.GPIO as GPIO
from typing import Dict, List, Tuple, Optional, Any


class Tower:
    """
    A comprehensive tower control class that handles all tower operations.
    
    This class provides methods for:
    - Managing multiple drone connections
    - Receiving and processing GPS data from drones
    - Sending commands to drones
    - Coordinating obstacle avoidance
    - Controlling landing platform flaps
    - Calculating distances and detecting potential collisions
    - Managing server connections and client handling
    """
    
    def __init__(self, 
                 host: str = '0.0.0.0',
                 ports: List[int] = None,
                 server_port: int = 6542,
                 comm_port: int = 6553,
                 debug_gps: bool = False,
                 debug_avoidance: bool = True):
        """
        Initialize the Tower class.
        
        Args:
            host: Host address to bind to
            ports: List of ports for drone connections
            server_port: Port for server communication
            comm_port: Port for command communication
            debug_gps: Enable GPS debugging
            debug_avoidance: Enable avoidance debugging
        """
        self.host = host
        self.ports = ports or [14551, 14552]
        self.server_port = server_port
        self.comm_port = comm_port
        self.debug_gps = debug_gps
        self.debug_avoidance = debug_avoidance
        
        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            datefmt='%H:%M:%S'
        )
        
        # Create a separate logger for GPS data that can be silenced
        self.gps_logger = logging.getLogger('gps_data')
        self.gps_logger.setLevel(logging.WARNING if not debug_gps else logging.DEBUG)
        
        # Data storage
        self.gps_data = {}
        self.connections = {}
        self.label_counter = 1
        
        # Control flags
        self.server_running = True
        self.avoidance_enabled = 2
        self.stage_started = time.time()
        self.disable_avoidance_detection = False
        self.force_disable_avoidance_detection = True
        
        # Update management
        self.last_full_update = 0
        self.full_update_interval = 5  # Send full update every 5 seconds
        
        # GPS logging control
        self.gps_message_counter = 0
        self.gps_log_interval = 50  # Only log every 50th GPS message
        
        # Server sockets
        self.server_socket = None
        self.server_thread = None
        
        # GPIO setup for flap control
        self.setup_gpio()
        
    def setup_gpio(self):
        """Set up GPIO pins for flap control."""
        try:
            GPIO.setmode(GPIO.BCM)
            # Define GPIO pins for stepper motor control
            self.step_pin = 17
            self.dir_pin = 18
            self.enable_pin = 27
            self.limit_switch_pin = 22
            
            # Set up pins
            GPIO.setup(self.step_pin, GPIO.OUT)
            GPIO.setup(self.dir_pin, GPIO.OUT)
            GPIO.setup(self.enable_pin, GPIO.OUT)
            GPIO.setup(self.limit_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Initialize pins
            GPIO.output(self.enable_pin, GPIO.HIGH)  # Disable stepper initially
            GPIO.output(self.step_pin, GPIO.LOW)
            GPIO.output(self.dir_pin, GPIO.LOW)
            
            print("GPIO setup completed for flap control")
            
        except Exception as e:
            print(f"Error setting up GPIO: {e}")
    
    def haversine(self, coord1: Tuple[float, float], coord2: Tuple[float, float]) -> float:
        """
        Calculate the great circle distance between two points on Earth.
        
        Args:
            coord1: First coordinate tuple (lat, lon)
            coord2: Second coordinate tuple (lat, lon)
            
        Returns:
            float: Distance in meters
        """
        R = 6371000  # Radius of Earth in meters
        lat1, lon1 = coord1
        lat2, lon2 = coord2
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = (math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c
    
    def send_gps_to_server(self, label: int, lat: float, lon: float, alt: float, vel: float, hdg: float):
        """
        Send GPS data to the dashboard server.
        
        Args:
            label: Drone label/ID
            lat: Latitude
            lon: Longitude
            alt: Altitude
            vel: Velocity
            hdg: Heading
        """
        self.gps_message_counter += 1
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.host, self.comm_port))
                gps_message = json.dumps({
                    "drone_id": label,
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "vel": vel,
                    "hdg": hdg
                })
                s.sendall(gps_message.encode())
                
                # Only log occasionally to reduce console spam
                if self.gps_message_counter % self.gps_log_interval == 0:
                    self.gps_logger.debug(f"Sent GPS update #{self.gps_message_counter} for Drone {label}")
                    
        except Exception as e:
            logging.error(f"Error sending GPS data to server: {e}")
    
    def send_all_drone_data(self):
        """Send data for all connected drones to ensure dashboard has complete information."""
        for label, data in self.gps_data.items():
            if len(data) >= 5:  # Make sure we have complete data
                lat, lon, alt, vel, hdg = data
                self.send_gps_to_server(label, lat, lon, alt, vel, hdg)
                logging.info(f"Sent refresh data for Drone {label}")
    
    def periodic_full_update(self):
        """Periodically send all drone data to the dashboard."""
        while self.server_running:
            current_time = time.time()
            if current_time - self.last_full_update > self.full_update_interval:
                self.send_all_drone_data()
                self.last_full_update = current_time
            time.sleep(1)
    
    def receive_server_commands(self):
        """Receive and process commands from the server."""
        while self.server_running:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((self.host, self.comm_port))
                    
                    while self.server_running:
                        try:
                            data = s.recv(1024)
                            if not data:
                                break
                                
                            command = data.decode().strip()
                            logging.info(f"Received server command: {command}")
                            
                            # Process the command
                            self.process_server_command(command)
                            
                        except Exception as e:
                            logging.error(f"Error receiving server command: {e}")
                            break
                            
            except Exception as e:
                logging.error(f"Error connecting to server: {e}")
                time.sleep(5)  # Wait before retrying
    
    def process_server_command(self, command: str):
        """
        Process commands received from the server.
        
        Args:
            command: Command string to process
        """
        try:
            if command.startswith("OPEN_FLAPS"):
                self.open_flaps()
            elif command.startswith("CLOSE_FLAPS"):
                self.close_flaps()
            elif command.startswith("ENABLE_AVOIDANCE"):
                self.avoidance_enabled = 1
                logging.info("Obstacle avoidance enabled")
            elif command.startswith("DISABLE_AVOIDANCE"):
                self.avoidance_enabled = 0
                logging.info("Obstacle avoidance disabled")
            elif command.startswith("SEND_COMMAND"):
                # Extract command to send to drones
                parts = command.split(" ", 2)
                if len(parts) >= 3:
                    drone_id = parts[1]
                    drone_command = parts[2]
                    self.send_command_to_drone(drone_id, drone_command)
            else:
                logging.warning(f"Unknown server command: {command}")
                
        except Exception as e:
            logging.error(f"Error processing server command: {e}")
    
    def send_command_to_drone(self, drone_id: str, command: str):
        """
        Send a command to a specific drone.
        
        Args:
            drone_id: ID of the drone to send command to
            command: Command to send
        """
        try:
            if drone_id in self.connections:
                conn = self.connections[drone_id]
                conn.sendall(command.encode())
                logging.info(f"Sent command '{command}' to drone {drone_id}")
            else:
                logging.warning(f"Drone {drone_id} not found in connections")
                
        except Exception as e:
            logging.error(f"Error sending command to drone {drone_id}: {e}")
    
    def handle_client(self, conn: socket.socket, label: int):
        """
        Handle communication with a connected drone client.
        
        Args:
            conn: Client socket connection
            label: Drone label/ID
        """
        logging.info(f"Handling client {label}")
        
        try:
            while self.server_running:
                try:
                    data = conn.recv(1024)
                    if not data:
                        logging.info(f"Client {label} disconnected")
                        break
                    
                    message = data.decode().strip()
                    
                    if message.startswith("GPS"):
                        # Process GPS data
                        self.process_gps_data(label, message)
                    else:
                        # Process other commands
                        logging.info(f"Received from drone {label}: {message}")
                        
                except Exception as e:
                    if "Bad file descriptor" in str(e):
                        logging.info(f"Client {label} connection lost")
                        break
                    else:
                        logging.error(f"Error handling client {label}: {e}")
                        break
                        
        except Exception as e:
            logging.error(f"Error in client handler for {label}: {e}")
        finally:
            # Clean up
            if label in self.connections:
                del self.connections[label]
            if label in self.gps_data:
                del self.gps_data[label]
            conn.close()
            logging.info(f"Client {label} handler finished")
    
    def process_gps_data(self, label: int, message: str):
        """
        Process GPS data received from a drone.
        
        Args:
            label: Drone label/ID
            message: GPS message string
        """
        try:
            parts = message.split()
            if len(parts) >= 6:
                lat = float(parts[1])
                lon = float(parts[2])
                alt = float(parts[3])
                vel = float(parts[4])
                hdg = float(parts[5])
                battery = float(parts[6]) if len(parts) > 6 else 0
                
                # Store GPS data
                self.gps_data[label] = [lat, lon, alt, vel, hdg, battery]
                
                # Send to dashboard
                self.send_gps_to_server(label, lat, lon, alt, vel, hdg)
                
                # Check for potential collisions if avoidance is enabled
                if self.avoidance_enabled and not self.force_disable_avoidance_detection:
                    self.check_collision_avoidance(label, lat, lon, alt)
                    
        except Exception as e:
            logging.error(f"Error processing GPS data from drone {label}: {e}")
    
    def check_collision_avoidance(self, current_drone: int, lat: float, lon: float, alt: float):
        """
        Check for potential collisions between drones and trigger avoidance if needed.
        
        Args:
            current_drone: ID of the current drone
            lat: Current latitude
            lon: Current longitude
            alt: Current altitude
        """
        try:
            current_pos = (lat, lon)
            
            for drone_id, data in self.gps_data.items():
                if drone_id == current_drone:
                    continue
                    
                if len(data) >= 3:
                    other_lat, other_lon, other_alt = data[0], data[1], data[2]
                    other_pos = (other_lat, other_lon)
                    
                    # Calculate horizontal distance
                    distance = self.haversine(current_pos, other_pos)
                    
                    # Calculate vertical distance
                    alt_diff = abs(alt - other_alt)
                    
                    # Check if drones are too close
                    if distance < 5.0 and alt_diff < 3.0:  # 5m horizontal, 3m vertical
                        if self.debug_avoidance:
                            logging.warning(f"Potential collision detected between drones {current_drone} and {drone_id}")
                            logging.warning(f"Distance: {distance:.2f}m, Alt diff: {alt_diff:.2f}m")
                        
                        # Send avoidance commands
                        self.trigger_avoidance(current_drone, drone_id)
                        
        except Exception as e:
            logging.error(f"Error in collision avoidance check: {e}")
    
    def trigger_avoidance(self, drone1: int, drone2: int):
        """
        Trigger avoidance maneuvers for two drones that are too close.
        
        Args:
            drone1: First drone ID
            drone2: Second drone ID
        """
        try:
            logging.info(f"Triggering avoidance for drones {drone1} and {drone2}")
            
            # Send avoidance commands to both drones
            if drone1 in self.connections:
                self.send_command_to_drone(str(drone1), "AVOID")
            
            if drone2 in self.connections:
                self.send_command_to_drone(str(drone2), "AVOID")
                
        except Exception as e:
            logging.error(f"Error triggering avoidance: {e}")
    
    def start_server(self, port: int = None) -> bool:
        """
        Start the tower server to accept drone connections.
        
        Args:
            port: Port to start server on (uses first port from self.ports if None)
            
        Returns:
            bool: True if server started successfully, False otherwise
        """
        if port is None:
            port = self.ports[0]
            
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, port))
            self.server_socket.listen(5)
            
            logging.info(f"Tower server started on {self.host}:{port}")
            
            # Start server thread
            self.server_thread = threading.Thread(target=self._server_loop, args=(port,))
            self.server_thread.daemon = True
            self.server_thread.start()
            
            # Start periodic update thread
            update_thread = threading.Thread(target=self.periodic_full_update)
            update_thread.daemon = True
            update_thread.start()
            
            # Start server command reception thread
            command_thread = threading.Thread(target=self.receive_server_commands)
            command_thread.daemon = True
            command_thread.start()
            
            return True
            
        except Exception as e:
            logging.error(f"Error starting server: {e}")
            return False
    
    def _server_loop(self, port: int):
        """
        Main server loop to accept and handle client connections.
        
        Args:
            port: Port the server is listening on
        """
        logging.info(f"Server loop started on port {port}")
        
        while self.server_running:
            try:
                conn, addr = self.server_socket.accept()
                logging.info(f"New connection from {addr}")
                
                # Assign label to new connection
                label = self.label_counter
                self.label_counter += 1
                
                # Store connection
                self.connections[label] = conn
                
                # Start client handler thread
                client_thread = threading.Thread(target=self.handle_client, args=(conn, label))
                client_thread.daemon = True
                client_thread.start()
                
            except Exception as e:
                if self.server_running:
                    logging.error(f"Error in server loop: {e}")
                break
        
        logging.info("Server loop ended")
    
    def move_stepper(self, steps: int, direction: int):
        """
        Move the stepper motor for flap control.
        
        Args:
            steps: Number of steps to move
            direction: Direction (1 for forward, 0 for backward)
        """
        try:
            GPIO.output(self.enable_pin, GPIO.LOW)  # Enable stepper
            GPIO.output(self.dir_pin, GPIO.HIGH if direction else GPIO.LOW)
            
            for _ in range(steps):
                GPIO.output(self.step_pin, GPIO.HIGH)
                time.sleep(0.001)  # 1ms pulse
                GPIO.output(self.step_pin, GPIO.LOW)
                time.sleep(0.001)
            
            GPIO.output(self.enable_pin, GPIO.HIGH)  # Disable stepper
            
        except Exception as e:
            logging.error(f"Error moving stepper: {e}")
    
    def extend_stepper_until_limit(self):
        """Extend the stepper motor until the limit switch is triggered."""
        try:
            GPIO.output(self.enable_pin, GPIO.LOW)  # Enable stepper
            GPIO.output(self.dir_pin, GPIO.HIGH)  # Forward direction
            
            while GPIO.input(self.limit_switch_pin) == GPIO.HIGH:
                GPIO.output(self.step_pin, GPIO.HIGH)
                time.sleep(0.001)
                GPIO.output(self.step_pin, GPIO.LOW)
                time.sleep(0.001)
            
            GPIO.output(self.enable_pin, GPIO.HIGH)  # Disable stepper
            logging.info("Stepper extended to limit")
            
        except Exception as e:
            logging.error(f"Error extending stepper: {e}")
    
    def retract_stepper_until_limit(self):
        """Retract the stepper motor until the limit switch is triggered."""
        try:
            GPIO.output(self.enable_pin, GPIO.LOW)  # Enable stepper
            GPIO.output(self.dir_pin, GPIO.LOW)  # Backward direction
            
            while GPIO.input(self.limit_switch_pin) == GPIO.HIGH:
                GPIO.output(self.step_pin, GPIO.HIGH)
                time.sleep(0.001)
                GPIO.output(self.step_pin, GPIO.LOW)
                time.sleep(0.001)
            
            GPIO.output(self.enable_pin, GPIO.HIGH)  # Disable stepper
            logging.info("Stepper retracted to limit")
            
        except Exception as e:
            logging.error(f"Error retracting stepper: {e}")
    
    def open_flaps(self):
        """Open the landing platform flaps."""
        try:
            logging.info("Opening flaps...")
            self.extend_stepper_until_limit()
            logging.info("Flaps opened successfully")
            
        except Exception as e:
            logging.error(f"Error opening flaps: {e}")
    
    def close_flaps(self):
        """Close the landing platform flaps."""
        try:
            logging.info("Closing flaps...")
            self.retract_stepper_until_limit()
            logging.info("Flaps closed successfully")
            
        except Exception as e:
            logging.error(f"Error closing flaps: {e}")
    
    def send_command(self):
        """Interactive command sending interface."""
        try:
            while self.server_running:
                print("\nAvailable commands:")
                print("1. OPEN_FLAPS - Open landing platform flaps")
                print("2. CLOSE_FLAPS - Close landing platform flaps")
                print("3. ENABLE_AVOIDANCE - Enable collision avoidance")
                print("4. DISABLE_AVOIDANCE - Disable collision avoidance")
                print("5. SEND_COMMAND <drone_id> <command> - Send command to specific drone")
                print("6. QUIT - Exit")
                
                command = input("Enter command: ").strip().upper()
                
                if command == "QUIT":
                    break
                elif command == "OPEN_FLAPS":
                    self.open_flaps()
                elif command == "CLOSE_FLAPS":
                    self.close_flaps()
                elif command == "ENABLE_AVOIDANCE":
                    self.avoidance_enabled = 1
                    print("Obstacle avoidance enabled")
                elif command == "DISABLE_AVOIDANCE":
                    self.avoidance_enabled = 0
                    print("Obstacle avoidance disabled")
                elif command.startswith("SEND_COMMAND"):
                    parts = command.split()
                    if len(parts) >= 3:
                        drone_id = parts[1]
                        drone_command = " ".join(parts[2:])
                        self.send_command_to_drone(drone_id, drone_command)
                    else:
                        print("Invalid SEND_COMMAND format. Use: SEND_COMMAND <drone_id> <command>")
                else:
                    print("Unknown command")
                    
        except KeyboardInterrupt:
            print("\nCommand interface interrupted")
        except Exception as e:
            logging.error(f"Error in command interface: {e}")
    
    def get_drone_count(self) -> int:
        """
        Get the number of connected drones.
        
        Returns:
            int: Number of connected drones
        """
        return len(self.connections)
    
    def get_drone_data(self, drone_id: int) -> Optional[List]:
        """
        Get GPS data for a specific drone.
        
        Args:
            drone_id: ID of the drone
            
        Returns:
            Optional[List]: GPS data [lat, lon, alt, vel, hdg, battery] or None if not found
        """
        return self.gps_data.get(drone_id)
    
    def get_all_drone_data(self) -> Dict[int, List]:
        """
        Get GPS data for all connected drones.
        
        Returns:
            Dict[int, List]: Dictionary mapping drone IDs to their GPS data
        """
        return self.gps_data.copy()
    
    def is_drone_connected(self, drone_id: int) -> bool:
        """
        Check if a specific drone is connected.
        
        Args:
            drone_id: ID of the drone to check
            
        Returns:
            bool: True if drone is connected, False otherwise
        """
        return drone_id in self.connections
    
    def stop(self):
        """Stop the tower server and clean up resources."""
        logging.info("Stopping tower server...")
        
        self.server_running = False
        
        # Close all client connections
        for label, conn in self.connections.items():
            try:
                conn.close()
            except:
                pass
        self.connections.clear()
        
        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        
        # Clean up GPIO
        try:
            GPIO.cleanup()
        except:
            pass
        
        logging.info("Tower server stopped")
    
    def __enter__(self):
        """Context manager entry."""
        self.start_server()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
