"""
Drone Class for UAV Navigation

This class encapsulates all drone-related functionality including:
- MAVLink communication and control
- Flight operations (takeoff, landing, waypoint navigation)
- Precision landing with ArUco markers
- Obstacle avoidance
- Return to home functionality
- GPS data transmission
- Command execution

Author: Generated from drone_pi folder functions
"""

import socket
from pymavlink import mavutil
import time
import threading
import sys
import math
import signal
import cv2
import numpy as np
import os
import json
from typing import Dict, Tuple, Optional, List, Any


class Drone:
    """
    A comprehensive drone control class that handles all drone operations.
    
    This class provides methods for:
    - Connecting to and controlling drones via MAVLink
    - Flight operations (takeoff, landing, waypoint navigation)
    - Precision landing using ArUco marker detection
    - Obstacle avoidance
    - Return to home functionality
    - GPS data transmission and command reception
    """
    
    def __init__(self, 
                 mavlink_port: int = 14551,
                 hub_ip: str = '10.203.121.89',
                 hub_port: int = 14551,
                 use_pi_camera: bool = True):
        """
        Initialize the Drone class.
        
        Args:
            mavlink_port: Port for MAVLink connection
            hub_ip: IP address of the hub/tower
            hub_port: Port for hub communication
            use_pi_camera: Whether to use Raspberry Pi camera
        """
        self.mavlink_port = mavlink_port
        self.hub_ip = hub_ip
        self.hub_port = hub_port
        self.use_pi_camera = use_pi_camera
        
        # Connection objects
        self.connection = None
        self.client_socket = None
        
        # Threading and control
        self.shutdown_event = threading.Event()
        self.mavlink_lock = threading.Lock()
        self.current_command = None
        
        # GPS and position data
        self.home_gps = {"lat": 0, "lon": 0, "alt": 0}
        self.gps_data = {}
        
        # Camera and ArUco detection
        self.camera = None
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Message rates configuration
        self.message_rates = [
            (33, 30),  # GLOBAL_POSITION_INT at 30 Hz
            (30, 50),  # ATTITUDE at 50 Hz
            (74, 5),   # VFR_HUD at 5 Hz
            (24, 5),   # GPS_RAW_INT at 5 Hz
            (32, 10),  # LOCAL_POSITION_NED at 10 Hz
        ]
        
        # Data streams configuration
        self.data_streams = [
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 50),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50),  # Attitude
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 20),
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 10),
        ]
        
        # Threads
        self.gps_thread = None
        self.command_thread = None
        
    def connect_mavlink(self) -> bool:
        """
        Connect to the drone via MAVLink.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            print(f"Connecting to MAVLink on {self.mavlink_port}...")
            self.connection = mavutil.mavlink_connection(f'udpin:127.0.0.1:{self.mavlink_port}')
            print(f"Waiting for heartbeat on {self.mavlink_port}...")
            self.connection.wait_heartbeat()
            print(f"Heartbeat received from system {self.connection.target_system}, component {self.connection.target_component}")
            
            # Set up message rates
            self._setup_message_rates()
            
            return True
        except Exception as e:
            print(f"Failed to connect to MAVLink: {e}")
            return False
    
    def _setup_message_rates(self):
        """Set up message stream rates for optimal data flow."""
        print("Setting up message stream rates...")
        for msg_id, rate in self.message_rates:
            success = self.set_message_interval(msg_id, rate)
            if not success:
                print(f"Failed to set message rate for ID {msg_id}, retrying...")
                self.set_message_interval(msg_id, rate, verify=False)
        
        # Request data streams as fallback
        for stream_id, rate in self.data_streams:
            self.request_data_stream(stream_id, rate)
    
    def set_message_interval(self, message_id: int, rate_hz: int, verify: bool = True, timeout: int = 3) -> bool:
        """
        Sets the update rate for a specific MAVLink message.
        
        Args:
            message_id: MAVLink message ID (e.g., 33 for GLOBAL_POSITION_INT)
            rate_hz: Desired frequency in Hz
            verify: Whether to verify the command was accepted
            timeout: Timeout for command verification in seconds
        
        Returns:
            bool: True if command was successful, False otherwise
        """
        if self.shutdown_event.is_set():
            return False
            
        interval_us = int(1e6 / rate_hz)
        
        print(f"Setting message ID {message_id} to {rate_hz} Hz (interval: {interval_us} μs)")
        
        with self.mavlink_lock:
            try:
                self.connection.mav.command_long_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,  # Confirmation
                    message_id,
                    interval_us,
                    0, 0, 0, 0, 0
                )
            except Exception as e:
                print(f"Error sending message interval command: {e}")
                return False
        
        if not verify:
            return True
        
        # Wait for command acknowledgment
        start_time = time.time()
        while time.time() - start_time < timeout and not self.shutdown_event.is_set():
            with self.mavlink_lock:
                try:
                    msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
                except Exception as e:
                    print(f"Error receiving command ack: {e}")
                    return False
                    
            if msg:
                if msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print(f"Message rate set successfully for ID {message_id}")
                        return True
                    else:
                        print(f"Command rejected: {msg.result}")
                        return False
        
        print(f"Timeout waiting for acknowledgment on message ID {message_id}")
        return False
    
    def request_data_stream(self, stream_id: int, rate_hz: int):
        """
        Request data streams using the legacy method as a fallback.
        
        Args:
            stream_id: MAVLink data stream ID
            rate_hz: Desired frequency in Hz
        """
        if self.shutdown_event.is_set():
            return
            
        print(f"Requesting data stream {stream_id} at {rate_hz} Hz")
        
        with self.mavlink_lock:
            try:
                self.connection.mav.request_data_stream_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    stream_id,
                    rate_hz,
                    1  # Start
                )
            except Exception as e:
                print(f"Error requesting data stream: {e}")
    
    def connect_to_hub(self) -> bool:
        """
        Connect to the hub/tower for command reception and GPS transmission.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.hub_ip, self.hub_port))
            print(f"Connected to hub at {self.hub_ip}:{self.hub_port}")
            return True
        except Exception as e:
            print(f"Failed to connect to hub: {e}")
            return False
    
    def start_gps_thread(self):
        """Start the GPS data transmission thread."""
        if self.client_socket is None:
            print("No hub connection available for GPS thread")
            return
            
        self.gps_thread = threading.Thread(target=self._send_gps_coordinates, args=(self.client_socket,))
        self.gps_thread.daemon = True
        self.gps_thread.start()
        print("GPS transmission thread started")
    
    def _send_gps_coordinates(self, client):
        """Send GPS coordinates and battery data to the tower."""
        send_gps_coordinates.last_check = time.time()
        connection_valid = True
        
        while not self.shutdown_event.is_set() and connection_valid:
            try:
                if client._closed:
                    print("Client socket is closed, stopping GPS thread")
                    break
                    
                with self.mavlink_lock:
                    try:
                        msg_pos = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
                        msg_bat = self.connection.recv_match(type='SYS_STATUS', blocking=False)
                    except Exception as e:
                        if "Bad file descriptor" in str(e):
                            print("MAVLink connection is broken, stopping GPS thread")
                            connection_valid = False
                            break
                        else:
                            print(f"Error receiving GPS/battery data: {e}")
                            time.sleep(0.5)
                            continue
                        
                if msg_pos:
                    lat = msg_pos.lat / 1e7
                    lon = msg_pos.lon / 1e7
                    alt = msg_pos.alt / 1e3
                    vx, vy, vz = msg_pos.vx, msg_pos.vy, msg_pos.vz
                    vel = (vx**2 + vy**2 + vz**2)**(1/2)
                    hdg = msg_pos.hdg / 100
                    
                    battery = 0
                    if msg_bat:
                        battery = msg_bat.battery_remaining if hasattr(msg_bat, 'battery_remaining') else 0
                        battery = max(0, min(100, battery))
                    
                    gps_data = f"GPS {lat} {lon} {alt} {vel} {hdg} {battery}"
                    
                    if not client._closed:
                        try:
                            client.sendall(gps_data.encode())
                        except (BrokenPipeError, ConnectionResetError):
                            print("Connection lost while sending GPS data")
                            break
                        except Exception as e:
                            print(f"Error sending GPS data: {e}")
                            break
                else:
                    current_time = time.time()
                    if hasattr(send_gps_coordinates, 'last_check') and current_time - send_gps_coordinates.last_check < 5:
                        continue
                        
                    send_gps_coordinates.last_check = current_time
                    print("No GPS data received for 0.5s, requesting data streams...")
                    
                    with self.mavlink_lock:
                        try:
                            self.set_message_interval(33, 50, verify=False)
                            self.set_message_interval(1, 10, verify=False)
                            self.request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_POSITION, 50)
                            self.request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 10)
                        except Exception as e:
                            if "Bad file descriptor" in str(e):
                                print("MAVLink connection is broken while requesting streams")
                                connection_valid = False
                                break
                            else:
                                print(f"Error setting up data streams: {e}")
                    
            except Exception as e:
                if "Bad file descriptor" in str(e):
                    if not hasattr(send_gps_coordinates, 'last_bad_fd_error') or \
                       time.time() - send_gps_coordinates.last_bad_fd_error > 10:
                        print("MAVLink connection has been closed, exiting GPS thread")
                        send_gps_coordinates.last_bad_fd_error = time.time()
                    connection_valid = False
                    break
                else:
                    print(f"Error in GPS data thread: {e}")
                
                time.sleep(1)
        
        print("GPS data thread exiting")
    
    def start_command_thread(self):
        """Start the command reception thread."""
        if self.client_socket is None:
            print("No hub connection available for command thread")
            return
            
        self.command_thread = threading.Thread(target=self._receive_messages, args=(self.client_socket,))
        self.command_thread.daemon = True
        self.command_thread.start()
        print("Command reception thread started")
    
    def _receive_messages(self, client_socket):
        """Receive and process commands from the hub."""
        while not self.shutdown_event.is_set():
            try:
                if client_socket._closed:
                    print("Client socket is closed, stopping command thread")
                    break
                    
                data = client_socket.recv(1024)
                if not data:
                    print("Connection closed by server")
                    break
                    
                message = data.decode().strip()
                print(f"Received command: {message}")
                
                self._process_command(client_socket, message)
                
            except Exception as e:
                if "Bad file descriptor" in str(e):
                    print("Connection lost, stopping command thread")
                    break
                else:
                    print(f"Error receiving commands: {e}")
                    time.sleep(1)
        
        print("Command thread exiting")
    
    def _process_command(self, client_socket, message):
        """Process received commands."""
        if message.startswith("GPS"):
            # GPS data received, no action needed
            pass
        else:
            # Execute the command
            self.execute_command(message)
    
    def execute_command(self, command: str):
        """
        Execute a flight command.
        
        Args:
            command: The command string to execute
        """
        if command is None:
            return
        
        print(f"Executing command: {command}")
        
        if command.startswith("TAKEOFF"):
            self._execute_takeoff(command)
        elif command.startswith("WAYPOINT"):
            self._execute_waypoint(command)
        elif command.startswith("ABSOLUTE_WAYPOINT"):
            self._execute_absolute_waypoint(command)
        elif command.startswith("RTH"):
            self._execute_rth(command)
        elif command.startswith("LAND"):
            self._execute_land(command)
        elif command.startswith("PRECISION_LAND"):
            self._execute_precision_land(command)
        elif command.startswith("AVOID"):
            self._execute_avoid(command)
        else:
            print(f"Unknown command: {command}")
    
    def _execute_takeoff(self, command: str):
        """Execute takeoff command."""
        try:
            # Get current position for home reference
            home_pos = None
            retry_count = 0
            while home_pos is None and retry_count < 5:
                home_pos = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                retry_count += 1
                
            if home_pos:
                self.home_gps["lat"] = home_pos.lat / 1e7
                self.home_gps["lon"] = home_pos.lon / 1e7
                self.home_gps["alt"] = home_pos.alt / 1e3
                print(f"Home position set to: {self.home_gps}")
            else:
                print("WARNING: Couldn't get home position")
                
            # Parse takeoff altitude
            params = command.split()
            print(f"Takeoff parameters: {params}")
            
            if len(params) == 1:
                alt = 3  # Default 3m takeoff alt
            elif len(params) > 2:
                print("Invalid takeoff command")
                return
            else:
                alt = float(params[1])
                
            print(f"Takeoff to altitude: {alt}m")

            # Get current local position
            local_pos = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)
            if local_pos:
                self.current_command = f"ABSOLUTE_WAYPOINT {local_pos.x} {local_pos.y} {local_pos.z-alt}"
                print(f"Setting next waypoint after takeoff: {self.current_command}")
            
            # Switch to GUIDED mode
            print("Switching to GUIDED mode")
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4  # GUIDED mode
            )
            
            # Wait for mode change acknowledgment
            mode_change_timeout = time.time() + 3
            while time.time() < mode_change_timeout:
                ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print("Mode change accepted")
                        break
                    else:
                        print(f"Mode change rejected: {ack.result}")
                        return
            
            # Arm the drone
            print("Arming drone...")
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # Confirmation
                1,  # Arm
                0, 0, 0, 0, 0, 0
            )
            
            # Wait for arm acknowledgment
            arm_timeout = time.time() + 5
            while time.time() < arm_timeout:
                ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print("Drone armed successfully")
                        break
                    else:
                        print(f"Arm command rejected: {ack.result}")
                        return
            
            # Send takeoff command
            print("Sending takeoff command...")
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,  # Confirmation
                0,  # Minimum pitch
                0, 0, 0, 0, 0, alt
            )
            
            print("Takeoff command sent")
            
        except Exception as e:
            print(f"Error during takeoff: {e}")
    
    def _execute_waypoint(self, command: str):
        """Execute waypoint navigation command."""
        try:
            params = command.split()
            if len(params) != 4:
                print("Invalid waypoint command format")
                return
                
            target_x = float(params[1])
            target_y = float(params[2])
            target_z = float(params[3])
            
            print(f"Navigating to waypoint: ({target_x}, {target_y}, {target_z})")
            
            # Switch to GUIDED mode if not already
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4  # GUIDED mode
            )
            
            # Send waypoint command
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,  # Confirmation
                0,  # Hold time
                0,  # Acceptance radius
                0,  # Pass radius
                0,  # Yaw
                target_x, target_y, target_z
            )
            
            print("Waypoint command sent")
            
        except Exception as e:
            print(f"Error during waypoint navigation: {e}")
    
    def _execute_absolute_waypoint(self, command: str):
        """Execute absolute waypoint navigation command."""
        try:
            params = command.split()
            if len(params) != 4:
                print("Invalid absolute waypoint command format")
                return
                
            target_x = float(params[1])
            target_y = float(params[2])
            target_z = float(params[3])
            
            print(f"Navigating to absolute waypoint: ({target_x}, {target_y}, {target_z})")
            
            # Switch to GUIDED mode if not already
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4  # GUIDED mode
            )
            
            # Send absolute waypoint command
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,  # Confirmation
                0,  # Hold time
                0,  # Acceptance radius
                0,  # Pass radius
                0,  # Yaw
                target_x, target_y, target_z
            )
            
            print("Absolute waypoint command sent")
            
        except Exception as e:
            print(f"Error during absolute waypoint navigation: {e}")
    
    def _execute_rth(self, command: str):
        """Execute return to home command."""
        try:
            print("Executing Return to Home...")
            
            # Switch to RTL mode
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6  # RTL mode
            )
            
            print("RTH command sent")
            
        except Exception as e:
            print(f"Error during RTH: {e}")
    
    def _execute_land(self, command: str):
        """Execute landing command."""
        try:
            print("Executing landing...")
            
            # Switch to LAND mode
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                9  # LAND mode
            )
            
            print("Land command sent")
            
        except Exception as e:
            print(f"Error during landing: {e}")
    
    def _execute_precision_land(self, command: str):
        """Execute precision landing command."""
        try:
            print("Executing precision landing...")
            self.precision_landing()
        except Exception as e:
            print(f"Error during precision landing: {e}")
    
    def _execute_avoid(self, command: str):
        """Execute obstacle avoidance command."""
        try:
            print("Executing obstacle avoidance...")
            self.avoid_obstacle()
        except Exception as e:
            print(f"Error during obstacle avoidance: {e}")
    
    def takeoff(self, altitude: float = 3.0) -> bool:
        """
        Take off to a specified altitude.
        
        Args:
            altitude: Takeoff altitude in meters
            
        Returns:
            bool: True if takeoff successful, False otherwise
        """
        try:
            print(f"Taking off to {altitude}m...")
            
            # Switch to GUIDED mode
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4  # GUIDED mode
            )
            
            # Arm the drone
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0
            )
            
            time.sleep(2)
            
            # Send takeoff command
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, altitude
            )
            
            print("Takeoff command sent")
            return True
            
        except Exception as e:
            print(f"Error during takeoff: {e}")
            return False
    
    def arm_disarm_drone(self, value: int) -> bool:
        """
        Arm or disarm the drone.
        
        Args:
            value: 1 to arm, 0 to disarm
            
        Returns:
            bool: True if command successful, False otherwise
        """
        try:
            action = "arm" if value == 1 else "disarm"
            print(f"Attempting to {action} drone...")
            
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, value, 0, 0, 0, 0, 0
            )
            
            print(f"{action.capitalize()} command sent")
            return True
            
        except Exception as e:
            print(f"Error during {action}: {e}")
            return False
    
    def get_yaw(self) -> float:
        """
        Get current yaw angle.
        
        Returns:
            float: Current yaw angle in degrees
        """
        try:
            msg = self.connection.recv_match(type='ATTITUDE', blocking=True, timeout=1)
            if msg:
                yaw = math.degrees(msg.yaw)
                return yaw
            else:
                print("No attitude data received")
                return 0.0
        except Exception as e:
            print(f"Error getting yaw: {e}")
            return 0.0
    
    def send_yaw_command(self, target_yaw: float, yaw_speed: float = 30.0, relative: bool = False) -> bool:
        """
        Send yaw command to the drone.
        
        Args:
            target_yaw: Target yaw angle in degrees
            yaw_speed: Yaw speed in degrees/second
            relative: Whether the yaw is relative to current position
            
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        try:
            if relative:
                current_yaw = self.get_yaw()
                target_yaw = current_yaw + target_yaw
            
            # Normalize yaw to 0-360 degrees
            target_yaw = target_yaw % 360
            
            print(f"Sending yaw command: {target_yaw}° at {yaw_speed}°/s")
            
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0, target_yaw, yaw_speed, 0, 0, 0, 0
            )
            
            return True
            
        except Exception as e:
            print(f"Error sending yaw command: {e}")
            return False
    
    def send_velocity(self, vx: float, vy: float, vz: float) -> bool:
        """
        Send velocity command to the drone.
        
        Args:
            vx: Velocity in x direction (m/s)
            vy: Velocity in y direction (m/s)
            vz: Velocity in z direction (m/s)
            
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        try:
            self.connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10, self.connection.target_system, self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
                0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
            ))
            return True
        except Exception as e:
            print(f"Error sending velocity command: {e}")
            return False
    
    def send_velocity_yaw(self, yaw: float, vx: float, vy: float, vz: float) -> bool:
        """
        Send velocity command with yaw to the drone.
        
        Args:
            yaw: Yaw angle in degrees
            vx: Velocity in x direction (m/s)
            vy: Velocity in y direction (m/s)
            vz: Velocity in z direction (m/s)
            
        Returns:
            bool: True if command sent successfully, False otherwise
        """
        try:
            self.connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10, self.connection.target_system, self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
                0, 0, 0, vx, vy, vz, 0, 0, 0, yaw, 0
            ))
            return True
        except Exception as e:
            print(f"Error sending velocity yaw command: {e}")
            return False
    
    def set_flight_mode(self, mode: str) -> bool:
        """
        Set flight mode.
        
        Args:
            mode: Flight mode string (e.g., 'GUIDED', 'RTL', 'LAND')
            
        Returns:
            bool: True if mode set successfully, False otherwise
        """
        try:
            mode_mapping = {
                'GUIDED': 4,
                'RTL': 6,
                'LAND': 9,
                'AUTO': 3,
                'MANUAL': 0
            }
            
            if mode not in mode_mapping:
                print(f"Unknown flight mode: {mode}")
                return False
            
            mode_id = mode_mapping[mode]
            print(f"Setting flight mode to {mode}...")
            
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            return True
            
        except Exception as e:
            print(f"Error setting flight mode: {e}")
            return False
    
    def get_flight_mode(self) -> str:
        """
        Get current flight mode.
        
        Returns:
            str: Current flight mode
        """
        try:
            msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                mode_mapping = {
                    4: 'GUIDED',
                    6: 'RTL',
                    9: 'LAND',
                    3: 'AUTO',
                    0: 'MANUAL'
                }
                return mode_mapping.get(msg.custom_mode, 'UNKNOWN')
            else:
                return 'UNKNOWN'
        except Exception as e:
            print(f"Error getting flight mode: {e}")
            return 'UNKNOWN'
    
    def waypoint(self, x_offset: float, y_offset: float, z_offset: float) -> bool:
        """
        Navigate to a waypoint with offsets from current position.
        
        Args:
            x_offset: X offset in meters
            y_offset: Y offset in meters
            z_offset: Z offset in meters
            
        Returns:
            bool: True if waypoint command sent successfully, False otherwise
        """
        try:
            print(f"Navigating to waypoint: ({x_offset}, {y_offset}, {z_offset})")
            
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0, 0, 0, 0, x_offset, y_offset, z_offset
            )
            
            return True
            
        except Exception as e:
            print(f"Error sending waypoint command: {e}")
            return False
    
    def rth(self, alt: float = 10.0) -> bool:
        """
        Return to home at specified altitude.
        
        Args:
            alt: Return altitude in meters
            
        Returns:
            bool: True if RTH command sent successfully, False otherwise
        """
        try:
            print(f"Returning to home at {alt}m altitude...")
            
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                0, 0, 0, 0, 0, 0, 0, alt
            )
            
            return True
            
        except Exception as e:
            print(f"Error sending RTH command: {e}")
            return False
    
    def land(self) -> bool:
        """
        Land the drone.
        
        Returns:
            bool: True if land command sent successfully, False otherwise
        """
        try:
            print("Landing drone...")
            
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            
            return True
            
        except Exception as e:
            print(f"Error sending land command: {e}")
            return False
    
    def get_home_gps(self) -> Dict[str, float]:
        """
        Get home GPS coordinates.
        
        Returns:
            Dict[str, float]: Home GPS coordinates {'lat': lat, 'lon': lon, 'alt': alt}
        """
        return self.home_gps.copy()
    
    def connect_camera(self) -> bool:
        """
        Connect to the camera for ArUco detection.
        
        Returns:
            bool: True if camera connected successfully, False otherwise
        """
        try:
            if self.use_pi_camera:
                self.camera = cv2.VideoCapture(0)
            else:
                self.camera = cv2.VideoCapture(0)
            
            if not self.camera.isOpened():
                print("Failed to open camera")
                return False
            
            print("Camera connected successfully")
            return True
            
        except Exception as e:
            print(f"Error connecting to camera: {e}")
            return False
    
    def detect_aruco_tags(self, frame) -> Tuple[List, List]:
        """
        Detect ArUco tags in a frame.
        
        Args:
            frame: Input image frame
            
        Returns:
            Tuple[List, List]: (corners, ids) of detected ArUco tags
        """
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None:
                return corners, ids
            else:
                return [], []
                
        except Exception as e:
            print(f"Error detecting ArUco tags: {e}")
            return [], []
    
    def precision_landing(self) -> bool:
        """
        Execute precision landing using ArUco marker detection.
        
        Returns:
            bool: True if precision landing successful, False otherwise
        """
        try:
            print("Starting precision landing...")
            
            if not self.connect_camera():
                print("Failed to connect camera for precision landing")
                return False
            
            # Switch to GUIDED mode
            self.set_flight_mode('GUIDED')
            
            landing_successful = False
            max_attempts = 100
            
            for attempt in range(max_attempts):
                ret, frame = self.camera.read()
                if not ret:
                    print("Failed to capture frame")
                    continue
                
                corners, ids = self.detect_aruco_tags(frame)
                
                if len(corners) > 0:
                    # ArUco marker detected
                    print(f"ArUco marker detected: {ids}")
                    
                    # Calculate marker position and adjust drone position
                    # This is a simplified version - you may need more sophisticated control
                    
                    # Send land command
                    self.land()
                    landing_successful = True
                    break
                else:
                    print("No ArUco markers detected, continuing search...")
                    time.sleep(0.1)
            
            self.camera.release()
            
            if landing_successful:
                print("Precision landing completed")
            else:
                print("Precision landing failed - no markers found")
            
            return landing_successful
            
        except Exception as e:
            print(f"Error during precision landing: {e}")
            return False
    
    def avoid_obstacle(self) -> bool:
        """
        Execute obstacle avoidance maneuver.
        
        Returns:
            bool: True if avoidance maneuver successful, False otherwise
        """
        try:
            print("Executing obstacle avoidance...")
            
            # Simple avoidance maneuver - move up and to the side
            self.send_velocity(0, 2, 1)  # Move right and up
            time.sleep(2)
            self.send_velocity(0, 0, 0)  # Stop
            
            print("Obstacle avoidance completed")
            return True
            
        except Exception as e:
            print(f"Error during obstacle avoidance: {e}")
            return False
    
    def start(self):
        """Start the drone system with all threads."""
        if not self.connect_mavlink():
            print("Failed to connect to MAVLink")
            return False
        
        if not self.connect_to_hub():
            print("Failed to connect to hub")
            return False
        
        self.start_gps_thread()
        self.start_command_thread()
        
        print("Drone system started successfully")
        return True
    
    def stop(self):
        """Stop the drone system and clean up resources."""
        print("Stopping drone system...")
        
        self.shutdown_event.set()
        
        if self.gps_thread and self.gps_thread.is_alive():
            self.gps_thread.join(timeout=2)
        
        if self.command_thread and self.command_thread.is_alive():
            self.command_thread.join(timeout=2)
        
        if self.client_socket:
            self.client_socket.close()
        
        if self.camera:
            self.camera.release()
        
        print("Drone system stopped")
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
