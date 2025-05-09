import socket
from pymavlink import mavutil  # type: ignore
import time
import threading
import sys
import math
import signal

# Client Configuration
HUB_IP = '10.203.121.89'
PORT = 14551

# Create a global event for signaling thread termination
shutdown_event = threading.Event()
mavlink_lock = threading.Lock()  # Lock for thread-safe MAVLink access

# MAVLink Connection to PX4
# Fix connection string format - should be udpin: for UDP connection
print(f"Connecting to MAVLink on {PORT}...")
connection = mavutil.mavlink_connection(f'udpin:127.0.0.1:{PORT}')
print(f"Waiting for heartbeat on {PORT}...")
connection.wait_heartbeat()
print(f"Heartbeat received from system {connection.target_system}, component {connection.target_component}")

current_command = None  # Track ongoing command
home_gps = {"lat": 0, "lon": 0, "alt": 0}

def set_message_interval(message_id, rate_hz, verify=True, timeout=3):
    """
    Sets the update rate for a specific MAVLink message using MAV_CMD_SET_MESSAGE_INTERVAL.
    
    Args:
        message_id: MAVLink message ID (e.g., 33 for GLOBAL_POSITION_INT)
        rate_hz: Desired frequency in Hz
        verify: Whether to verify the command was accepted
        timeout: Timeout for command verification in seconds
    
    Returns:
        bool: True if command was successful, False otherwise
    """
    # Skip if we're shutting down
    if shutdown_event.is_set():
        return False
        
    interval_us = int(1e6 / rate_hz)  # Convert Hz to microseconds
    
    # Send the command to set message interval
    print(f"Setting message ID {message_id} to {rate_hz} Hz (interval: {interval_us} μs)")
    
    # Use lock to prevent concurrent access to the MAVLink connection
    with mavlink_lock:
        try:
            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
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
    while time.time() - start_time < timeout and not shutdown_event.is_set():
        with mavlink_lock:
            try:
                msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
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

# Configure important messages with higher rates
# 33 = GLOBAL_POSITION_INT (position data)
# 30 = ATTITUDE (attitude data)
# 74 = VFR_HUD (airspeed, groundspeed, heading, throttle, alt, climb rate)
message_rates = [
    (33, 50),  # GLOBAL_POSITION_INT at 50 Hz
    (30, 50),  # ATTITUDE at 50 Hz
    (74, 5),  # VFR_HUD at 20 Hz
    (24, 5),  # GPS_RAW_INT at 30 Hz
    (32, 10),  # LOCAL_POSITION_NED at 30 Hz
]

print("Setting up message stream rates...")
for msg_id, rate in message_rates:
    success = set_message_interval(msg_id, rate)
    if not success:
        print(f"Failed to set message rate for ID {msg_id}, retrying...")
        set_message_interval(msg_id, rate, verify=False)  # Try again without verification

# Request data streams using the legacy method as a fallback
def request_data_stream(stream_id, rate_hz):
    if shutdown_event.is_set():
        return
        
    print(f"Requesting data stream {stream_id} at {rate_hz} Hz")
    
    with mavlink_lock:
        try:
            connection.mav.request_data_stream_send(
                connection.target_system,
                connection.target_component,
                stream_id,
                rate_hz,
                1  # Start
            )
        except Exception as e:
            print(f"Error requesting data stream: {e}")

# Request data streams as a fallback
streams = [
    (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 50),
    (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50),  # Attitude
    (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 20),
    (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 10),
]

for stream_id, rate in streams:
    request_data_stream(stream_id, rate)

def send_gps_coordinates(client):
    """Send GPS coordinates and battery data to the tower"""
    # Store last check time globally
    send_gps_coordinates.last_check = time.time()
    connection_valid = True
    
    while not shutdown_event.is_set() and connection_valid:
        try:
            # Skip if the client socket is closed
            if client._closed:
                print("Client socket is closed, stopping GPS thread")
                break
                
            # Use the lock to prevent concurrent access to the MAVLink connection
            with mavlink_lock:
                try:
                    # Get position data
                    msg_pos = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
                    
                    # Get battery data
                    msg_bat = connection.recv_match(type='SYS_STATUS', blocking=False)
                except Exception as e:
                    if "Bad file descriptor" in str(e):
                        # The MAVLink connection is broken, need to reconnect
                        print("MAVLink connection is broken, stopping GPS thread")
                        connection_valid = False
                        break
                    else:
                        print(f"Error receiving GPS/battery data: {e}")
                        time.sleep(0.5)
                        continue
                    
            # Process position data
            if msg_pos:
                lat = msg_pos.lat / 1e7
                lon = msg_pos.lon / 1e7
                alt = msg_pos.alt / 1e3
                vx, vy, vz = msg_pos.vx, msg_pos.vy, msg_pos.vz
                vel = (vx**2 + vy**2 + vz**2)**(1/2)
                hdg = msg_pos.hdg / 100
                
                # Process battery data if available
                battery = 0  # Default value
                if msg_bat:
                    # Battery remaining percentage (0-100)
                    battery = msg_bat.battery_remaining if hasattr(msg_bat, 'battery_remaining') else 0
                    # Ensure it's within 0-100 range
                    battery = max(0, min(100, battery))
                
                # Create data string with battery information
                gps_data = f"GPS {lat} {lon} {alt} {vel} {hdg} {battery}"
                
                # Only send if the socket is valid
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
                # If we didn't get a GLOBAL_POSITION_INT message, check if the stream is active
                current_time = time.time()
                if hasattr(send_gps_coordinates, 'last_check') and current_time - send_gps_coordinates.last_check < 5:
                    continue
                    
                send_gps_coordinates.last_check = current_time
                print("No GPS data received for 0.5s, requesting data streams...")
                
                # Re-request the message streams
                with mavlink_lock:
                    try:
                        set_message_interval(33, 50, verify=False)  # GLOBAL_POSITION_INT
                        set_message_interval(1, 10, verify=False)   # SYS_STATUS for battery info
                        request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_POSITION, 50)
                        request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 10)
                    except Exception as e:
                        if "Bad file descriptor" in str(e):
                            # The MAVLink connection is broken, need to reconnect
                            print("MAVLink connection is broken while requesting streams")
                            connection_valid = False
                            break
                        else:
                            print(f"Error setting up data streams: {e}")
                
        except Exception as e:
            # Don't flood with error messages about bad file descriptor
            if "Bad file descriptor" in str(e):
                if not hasattr(send_gps_coordinates, 'last_bad_fd_error') or \
                   time.time() - send_gps_coordinates.last_bad_fd_error > 10:
                    print("MAVLink connection has been closed, exiting GPS thread")
                    send_gps_coordinates.last_bad_fd_error = time.time()
                connection_valid = False
                break
            else:
                print(f"Error in GPS data thread: {e}")
            
            time.sleep(1)  # Avoid tight loop on error
    
    print("GPS data thread exiting")

def execute_command(command):
    global current_command

    if command is None:
        return
    
    print(f"Executing command: {command}")
    
    if command.startswith("TAKEOFF"):
        try:
            # Get current position for home reference
            home_pos = None
            retry_count = 0
            while home_pos is None and retry_count < 5:
                home_pos = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                retry_count += 1
                
            if home_pos:
                home_gps["lat"] = home_pos.lat / 1e7
                home_gps["lon"] = home_pos.lon / 1e7
                home_gps["alt"] = home_pos.alt / 1e3
                print(f"Home position set to: {home_gps}")
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
            local_pos = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)
            if local_pos:
                current_command = f"ABSOLUTE_WAYPOINT {local_pos.x} {local_pos.y} {local_pos.z-alt}"
                print(f"Setting next waypoint after takeoff: {current_command}")
            
            # Switch to GUIDED mode
            print("Switching to GUIDED mode")
            connection.mav.set_mode_send(
                connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4  # GUIDED mode
            )
            
            # Wait for mode change acknowledgment
            mode_change_timeout = time.time() + 3
            while time.time() < mode_change_timeout:
                ack = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    print(f"Mode change result: {ack.result}")
                    break
            
            # Arm the vehicle
            print("Arming vehicle")
            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            
            # Wait for arming
            arm_timeout = time.time() + 3
            armed = False
            while time.time() < arm_timeout:
                ack = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        armed = True
                        print("Vehicle armed successfully")
                        break
                    else:
                        print(f"Arming failed with result: {ack.result}")
                        break
            
            if not armed:
                print("WARNING: Arming acknowledgment not received, continuing anyway")
                
            # Execute takeoff command
            print(f"Sending takeoff command to {alt}m")
            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, alt
            )
            
            # Wait for takeoff acknowledgment
            takeoff_timeout = time.time() + 3
            while time.time() < takeoff_timeout:
                ack = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    print(f"Takeoff command result: {ack.result}")
                    break
            
            print("Takeoff sequence completed")
            
        except Exception as e:
            print(f"Error in takeoff: {e}")

    elif command.startswith("ABSOLUTE_WAYPOINT"):
        try:
            params = command.split()
            target_x, target_y, target_z = map(float, params[1:])
            
            print(f"Moving to absolute waypoint: x={target_x}, y={target_y}, z={target_z}")
            
            # Import only when needed to avoid circular imports
            try:
                from way_point import way_point
                way_point(connection, target_x, target_y, target_z)
            except ImportError:
                # Fallback to direct MAVLink commands if way_point module not available
                print("way_point module not found, using direct MAVLink commands")
                
                # Set position target
                connection.mav.set_position_target_local_ned_send(
                    0,       # Timestamp (0 for immediate)
                    connection.target_system,
                    connection.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111111000,  # Use position, ignore velocity and acceleration
                    target_x, target_y, target_z,  # Position
                    0, 0, 0,  # Velocity
                    0, 0, 0,  # Acceleration
                    0, 0      # Yaw and yaw rate
                )
            
        except Exception as e:
            print(f"Error executing absolute waypoint: {e}")

    elif command.startswith("WAYPOINT"):
        print("Executing WAYPOINT NAVIGATION")
        params = command.split()
        if len(params) != 4:
            print("Invalid waypoint parameters. Format: WAYPOINT x y z")
            return
        try:
            x_offset, y_offset, z_offset = map(float, params[1:])
            
            # Get current position
            current_pos = None
            retry_count = 0
            while current_pos is None and retry_count < 5:
                current_pos = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
                retry_count += 1
                
            if current_pos is None:
                print("ERROR: Could not get current position")
                return
                
            current_x = current_pos.x
            current_y = current_pos.y
            current_z = current_pos.z
            
            target_x = current_x + x_offset
            target_y = current_y + y_offset
            target_z = current_z - z_offset  # Negative because NED coordinate system

            current_command = f"ABSOLUTE_WAYPOINT {target_x} {target_y} {target_z}"
            print(f"Relative waypoint translated to: {current_command}")
            
            # Execute the absolute waypoint command
            execute_command(current_command)

        except Exception as e:
            print(f"Error executing waypoint navigation: {e}")

    elif command == "RTH":
        print("Executing Return To Home")
        try:
            # Switch to GUIDED mode first
            connection.mav.set_mode_send(
                connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4  # GUIDED mode
            )
            time.sleep(1)

            home_lat = int(home_gps["lat"] * 1e7)
            home_lon = int(home_gps["lon"] * 1e7)
            hover_altitude = 5  # Return at 5m altitude
            target_alt = int(hover_altitude)

            print(f"Returning to home: lat={home_gps['lat']}, lon={home_gps['lon']}, alt={target_alt}m")

            # Send position target in global coordinates
            connection.mav.set_position_target_global_int_send(
                0,  # Timestamp (0 for immediate)
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,  # Use position, ignore velocity and acceleration
                home_lat,
                home_lon,
                target_alt,
                0, 0, 0,  # Velocity
                0, 0, 0,  # Acceleration
                0, 0      # Yaw and yaw rate
            )

            current_command = "RTH"
            print("Return To Home command sent")

        except Exception as e:
            print(f"Error in RTH: {e}")

    elif command == "STANDBY":
        print("Entering STANDBY (Hover in place)")
        try:
            connection.mav.set_position_target_local_ned_send(
                0,  # Timestamp (0 for immediate)
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
                int(0b110111111000),  # Use position offsets
                0, 0, 0,  # No position change
                0, 0, 0,  # No velocity
                0, 0, 0,  # No acceleration
                0, 0      # No yaw change
            )
            print("STANDBY command sent")
            
        except Exception as e:
            print(f"Error in STANDBY: {e}")
            
    elif command == "PRECISION_LAND":
        print("Executing Precision Landing")
        try:
            try:
                from precision_landing import precision_land_mode
                precision_land_mode(connection)
                print("Using precision_landing module for landing")
            except ImportError:
                print("precision_landing module not found, using direct landing command")
                connection.mav.command_long_send(
                    connection.target_system,
                    connection.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
        except Exception as e:
            print(f"Error in precision landing: {e}")

    elif command == "LAND":
        print("Executing standard landing")
        try:
            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            
            # Wait for acknowledgment
            land_timeout = time.time() + 3
            while time.time() < land_timeout:
                ack = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                    print(f"Land command result: {ack.result}")
                    break
                    
            print("Land command sent")
            
        except Exception as e:
            print(f"Error in land: {e}")

    elif command == "AVOID":
        print("Executing collision avoidance maneuver")
        try:
            try:
                from avoid import avoid_obstacle
                avoid_obstacle(connection)
            except ImportError:
                print("avoid module not found, performing basic avoidance")
                # Basic avoidance - move up 2m
                current_pos = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
                if current_pos:
                    # Move up 2m from current position
                    target_z = current_pos.z - 2.0  # Negative because NED
                    
                    connection.mav.set_position_target_local_ned_send(
                        0,
                        connection.target_system,
                        connection.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        0b0000111111000111,  # Use only z position
                        current_pos.x, current_pos.y, target_z,
                        0, 0, 0,
                        0, 0, 0,
                        0, 0
                    )
            
            time.sleep(12)  # Wait for avoidance to complete
            
        except Exception as e:
            print(f"Error in avoidance: {e}")

    elif command == "STOP":
        print("Executing emergency stop")
        try:
            # Switch to brake mode
            connection.mav.command_long_send(
                connection.target_system, 
                connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                1,
                17,  # Mode 17 (brake)
                0, 0, 0, 0, 0
            )

            print("STOPPED - vehicle in brake mode")
            time.sleep(10)  # Wait in brake mode

            # Switch back to guided mode
            connection.mav.command_long_send(
                connection.target_system, 
                connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                1,
                4,  # Switch to mode 4 (guided)
                0, 0, 0, 0, 0
            )

            print("Switched back to guided mode")
            time.sleep(1)
            
        except Exception as e:
            print(f"Error in stop command: {e}")
    
    elif command == "RESUME":
        print(f"Resuming previous command: {current_command}")
        if current_command:
            execute_command(current_command)
        else:
            print("No previous command to resume")
    
    else:
        print(f"Unknown command: {command}")

# Function to handle incoming messages
def receive_messages(client_socket):
    while not shutdown_event.is_set():
        try:
            # Skip if the socket is closed
            if client_socket._closed:
                print("Client socket is closed, stopping receive thread")
                break
                
            message = client_socket.recv(1024).decode()
            if not message:
                print("Server closed connection")
                break
                
            print(f"Received command from tower: {message}")
            
            # Process the command in a separate thread to avoid blocking
            command_thread = threading.Thread(
                target=process_command,
                args=(client_socket, message),
                daemon=True
            )
            command_thread.start()
            
        except socket.timeout:
            # Timeout is normal, just continue
            continue
        except ConnectionResetError:
            print("Connection reset by tower")
            break
        except Exception as e:
            print(f"Error receiving message: {e}")
            break
            
    print("Receive messages thread exiting")

def process_command(client_socket, message):
    """Process command in a separate thread to avoid blocking the receive loop"""
    try:
        # Execute the command
        execute_command(message)
        
        # Send acknowledgment back to the tower
        try:
            client_socket.sendall("Command executed".encode())
            print(f"Sent acknowledgment for command: {message}")
        except Exception as e:
            print(f"Error sending command acknowledgment: {e}")
    except Exception as e:
        print(f"Error processing command '{message}': {e}")
        try:
            client_socket.sendall(f"Error: {str(e)}".encode())
        except:
            pass

# Update the main function for better connection management
def main():
    global connection
    reconnect_delay = 5  # Initial reconnect delay in seconds
    
    # Handle SIGINT (Ctrl+C) gracefully
    def signal_handler(sig, frame):
        print("\nReceived interrupt signal, cleaning up...")
        shutdown_event.set()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    # Try to get system parameters early
    print("Setting up MAVLink message rates...")
    for msg_id, rate in message_rates:
        set_message_interval(msg_id, rate, verify=False)
    
    while not shutdown_event.is_set():
        # Create a new socket for each connection attempt
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(5)  # Set socket timeout
        
        # Track active threads for this connection
        active_threads = []
        
        try:
            print(f"Connecting to tower at {HUB_IP}:{PORT}...")
            client.connect((HUB_IP, PORT))
            print(f"Connected to tower at {HUB_IP}:{PORT}")
            
            # Reset reconnect delay on successful connection
            reconnect_delay = 5
            
            # Start receiving messages in a separate thread
            receive_thread = threading.Thread(target=receive_messages, args=(client,))
            receive_thread.daemon = True
            receive_thread.start()
            active_threads.append(receive_thread)
            
            # Start sending GPS coordinates in a separate thread
            gps_thread = threading.Thread(target=send_gps_coordinates, args=(client,))
            gps_thread.daemon = True
            gps_thread.start()
            active_threads.append(gps_thread)
            
            # Continue main loop - send heartbeat messages
            heartbeat_interval = 10  # seconds
            last_heartbeat = time.time()
            last_stream_refresh = time.time()
            
            while not shutdown_event.is_set():
                try:
                    current_time = time.time()
                    
                    # Send periodic heartbeat
                    if current_time - last_heartbeat >= heartbeat_interval:
                        if not client._closed:
                            client.sendall("Drone status: OK".encode())
                            last_heartbeat = current_time
                        else:
                            print("Client socket closed, breaking heartbeat loop")
                            break
                    
                    # Refresh message rates periodically
                    if current_time - last_stream_refresh >= 60:  # Once a minute
                        print("Refreshing message rates...")
                        with mavlink_lock:
                            try:
                                for msg_id, rate in message_rates:
                                    set_message_interval(msg_id, rate, verify=False)
                                last_stream_refresh = current_time
                            except Exception as e:
                                print(f"Error refreshing message rates: {e}")
                    
                    # Check if threads are still alive
                    if not all(t.is_alive() for t in active_threads):
                        print("A worker thread has exited unexpectedly")
                        break
                        
                    # Sleep briefly to avoid high CPU usage
                    time.sleep(0.1)
                    
                except ConnectionResetError:
                    print("Connection reset by tower")
                    break
                except BrokenPipeError:
                    print("Connection broken")
                    break
                except Exception as e:
                    print(f"Error in main loop: {e}")
                    break
        
        except ConnectionRefusedError:
            print(f"Connection refused. Tower not running or not listening on {HUB_IP}:{PORT}")
        except socket.timeout:
            print("Connection attempt timed out")
        except Exception as e:
            print(f"Connection error: {e}")
        finally:
            # Always clean up resources
            try:
                if not client._closed:
                    client.close()
                print("Socket closed")
            except:
                pass
            
            # Wait briefly to let threads detect socket closure
            time.sleep(1)
            
            # Check for broken MAVLink connection and reconnect if needed
            with mavlink_lock:
                try:
                    # Test if the connection is still valid
                    connection.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
                except Exception as e:
                    if "Bad file descriptor" in str(e):
                        print("MAVLink connection lost, reconnecting...")
                        try:
                            connection = mavutil.mavlink_connection(f'udpin:127.0.0.1:{PORT}')
                            print("MAVLink connection re-established")
                            # Reinitialize message streams
                            for msg_id, rate in message_rates:
                                set_message_interval(msg_id, rate, verify=False)
                        except Exception as re_err:
                            print(f"Failed to reconnect to MAVLink: {re_err}")
            
        # Implement exponential backoff for reconnection attempts
        if not shutdown_event.is_set():
            print(f"Retrying connection in {reconnect_delay} seconds...")
            # Use a loop with small sleeps to check the shutdown event
            for _ in range(reconnect_delay * 10):  # 0.1 second intervals
                if shutdown_event.is_set():
                    break
                time.sleep(0.1)
                
            reconnect_delay = min(reconnect_delay * 2, 60)  # Double delay up to 60 seconds max

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Mission script terminated by user")
        shutdown_event.set()
    except Exception as e:
        print(f"Unhandled exception in main: {e}")
        shutdown_event.set()
