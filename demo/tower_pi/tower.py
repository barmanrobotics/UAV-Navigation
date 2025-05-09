import socket
import threading
import math
import os
import time
import json
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)

# Create a separate logger for GPS data that can be silenced
gps_logger = logging.getLogger('gps_data')
gps_logger.setLevel(logging.WARNING)  # Set to WARNING to hide most GPS messages

HOST = '0.0.0.0'  # Listen on all available network interfaces
PORTS = [14551, 14552]
SERVER_PORT = 6542
COMM_PORT = 6553

# Debug flag - set to False to reduce console output
DEBUG_GPS = False
DEBUG_AVOIDANCE = True

gps_data = {}
connections = {}
label_counter = 1  # Start labeling from 1
server_running = True
avoidance_enabled = 2
stage_started = time.time()
disable_avoidance_detection = False
force_disable_avoidance_detection = True

# Last time all drone data was sent to the dashboard
last_full_update = 0
FULL_UPDATE_INTERVAL = 5  # Send full update every 5 seconds

# Counter for limiting GPS log messages
gps_message_counter = 0
GPS_LOG_INTERVAL = 50  # Only log every 50th GPS message

def haversine(coord1, coord2):
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

def send_gps_to_server(label, lat, lon, alt, vel, hdg):
    global gps_message_counter
    
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, COMM_PORT))
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
            gps_message_counter += 1
            if gps_message_counter % GPS_LOG_INTERVAL == 0:
                gps_logger.debug(f"Sent GPS update #{gps_message_counter} for Drone {label}")
                
    except Exception as e:
        logging.error(f"Error sending GPS data to server: {e}")

def send_all_drone_data():
    """Send data for all connected drones to ensure dashboard has complete information"""
    for label, data in gps_data.items():
        if len(data) >= 5:  # Make sure we have complete data
            lat, lon, alt, vel, hdg = data
            send_gps_to_server(label, lat, lon, alt, vel, hdg)
            logging.info(f"Sent refresh data for Drone {label}")

def periodic_full_update():
    """Periodically send all drone data to the dashboard"""
    global last_full_update
    
    while server_running:
        current_time = time.time()
        if current_time - last_full_update > FULL_UPDATE_INTERVAL:
            send_all_drone_data()
            last_full_update = current_time
        time.sleep(1)

def receive_server_commands():
    global disable_avoidance_detection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((HOST, SERVER_PORT))
        server_socket.listen()
        logging.info(f"Listening for server commands on port {SERVER_PORT}")

        while True:
            conn, addr = server_socket.accept()
            with conn:
                command = conn.recv(1024).decode()
                if not command:
                    continue

                logging.info(f"Received command from server: {command}")
                
                # Special case: handle REFRESH command to send all drone data
                if command.strip() == "REFRESH":
                    logging.info("Received REFRESH command - sending all drone data")
                    send_all_drone_data()
                    continue
                
                parts = command.split(maxsplit=1)
                if len(parts) < 2:
                    logging.warning("Invalid command format")
                    continue

                target_label, command_data = parts
                if command_data == "LAND" or command_data == "PRECISION_LAND":
                    disable_avoidance_detection = True
                    logging.info("Avoidance detection disabled for landing")
                elif command_data == "TAKEOFF":
                    disable_avoidance_detection = False
                    logging.info("Avoidance detection enabled after takeoff")
                
                if target_label in connections:
                    try:
                        connections[target_label].sendall(command_data.encode())
                        logging.info(f"Command {command_data} sent to Drone {target_label}")
                    except Exception as e:
                        logging.error(f"Error sending command to Drone {target_label}: {e}")
                else:
                    logging.warning(f"No connection with Drone {target_label}")

def handle_client(conn, label):
    global avoidance_enabled, stage_started, disable_avoidance_detection
    try:
        logging.info(f"Connection established with Drone {label}")
        message_count = 0
        last_log_time = time.time()
        
        # Send a complete update of all drone data when a new drone connects
        if len(gps_data) > 0:
            send_all_drone_data()
        
        while True:
            try:
                message = conn.recv(1024).decode()
                if not message:
                    logging.info(f"Connection closed by Drone {label}")
                    break

                # Only respond to GPS data, don't echo other messages
                if message.startswith("GPS"):
                    message_count += 1
                    current_time = time.time()
                    
                    # Log message count every 10 seconds
                    if current_time - last_log_time > 10:
                        gps_logger.info(f"Processed {message_count} GPS messages from Drone {label} in the last 10 seconds")
                        message_count = 0
                        last_log_time = current_time
                    
                    parts = message.split()
                    if len(parts) == 6:
                        try:
                            lat, lon, alt, vel, new_hdg = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5])
                            
                            # Determine heading
                            try: 
                                if vel > 10:
                                    hdg = new_hdg
                                    if DEBUG_GPS:
                                        gps_logger.debug(f"Using velocity heading: {hdg}")
                                else:
                                    if label in gps_data and len(gps_data[label]) > 3:
                                        hdg = gps_data[label][4]  # Use stored heading
                                    else:
                                        hdg = 0
                                    if DEBUG_GPS:
                                        gps_logger.debug("Using stored heading")
                            except Exception as e:
                                if DEBUG_GPS:
                                    gps_logger.error(f"HDG ERROR: {e}")
                                hdg = 0
                                
                            # Store and forward GPS data
                            gps_data[label] = (lat, lon, alt, vel, hdg)
                            send_gps_to_server(label, lat, lon, alt, vel, hdg)
                            
                            # Check for collision avoidance if multiple drones are connected
                            if len(gps_data) > 1 and not disable_avoidance_detection and not force_disable_avoidance_detection:
                                drone_labels = list(gps_data.keys())
                                d1, d2 = drone_labels[0], drone_labels[1]
                                distance = haversine(gps_data[d1][0:2], gps_data[d2][0:2])
                                alt_diff = abs(gps_data[d1][2] - gps_data[d2][2])
                                
                                if DEBUG_AVOIDANCE:
                                    gps_logger.debug(f"Distance between drones: {distance:.1f}m, Alt diff: {alt_diff:.1f}m")
                                
                                horizontal_radius = 7
                                vertical_radius = 2

                                if (distance < horizontal_radius) and (alt_diff < vertical_radius):
                                    if avoidance_enabled == 0:
                                        logging.warning("COLLISION ALERT: Avoidance Mode 0 - Stopping both drones")
                                        connections[d1].sendall("STOP".encode())
                                        connections[d2].sendall("STOP".encode())
                                        avoidance_enabled = 1
                                        stage_started = time.time()
                                    elif avoidance_enabled == 1 and (time.time() - stage_started) >= 10:
                                        logging.warning("COLLISION ALERT: Avoidance Mode 1 - Initiating avoidance maneuver")
                                        connections[d2].sendall("AVOID".encode())
                                        avoidance_enabled = 2
                                        stage_started = time.time()
                                elif (distance > horizontal_radius or alt_diff > vertical_radius) and avoidance_enabled == 2 and (time.time() - stage_started) >= 12:
                                    logging.info("Avoidance: Mode 2 - Resuming normal operation")
                                    connections[d1].send("RESUME".encode())
                                    connections[d2].send("RESUME".encode())
                                    avoidance_enabled = 3
                                    stage_started = time.time()
                                elif (distance > horizontal_radius or alt_diff > vertical_radius) and avoidance_enabled == 3 and (time.time() - stage_started) >= 15:
                                    logging.info("Avoidance detection resumed")
                                    avoidance_enabled = 0
                                    
                        except ValueError as e:
                            logging.error(f"Invalid GPS data from Drone {label}: {e}")
            except ConnectionResetError:
                logging.warning(f"Connection reset by Drone {label}")
                break
            except Exception as e:
                logging.error(f"Error handling message from Drone {label}: {e}")
                break
    finally:
        if label in connections:
            del connections[label]
        conn.close()
        logging.info(f"Cleaned up connection for Drone {label}")

def start_server(port):
    global label_counter, server_running
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((HOST, port))
            s.listen()
            logging.info(f"Server listening on {HOST}:{port}")
            
            while server_running:
                try:
                    conn, addr = s.accept()
                    label = str(label_counter)
                    connections[label] = conn
                    logging.info(f"Drone {label} connected from {addr} on port {port}")
                    
                    # When a new drone connects, send data about all other drones
                    if len(gps_data) > 0:
                        send_all_drone_data()
                    
                    client_thread = threading.Thread(target=handle_client, args=(conn, label))
                    client_thread.daemon = True
                    client_thread.start()
                    label_counter += 1
                except Exception as e:
                    logging.error(f"Error accepting connection on port {port}: {e}")
                    if not server_running:
                        break
        except Exception as e:
            logging.error(f"Server error on port {port}: {e}")

def send_command():
    global server_running, disable_avoidance_detection
    
    # Print help information
    print("\n=== Tower Command Interface ===")
    print("Available commands:")
    print("  <label> TAKEOFF [altitude]    - Take off to specified altitude (default 3m)")
    print("  <label> WAYPOINT x y z        - Move to relative waypoint")
    print("  <label> RTH                   - Return to home")
    print("  <label> LAND                  - Land the drone")
    print("  <label> PRECISION_LAND        - Execute precision landing")
    print("  <label> STANDBY               - Hover in place")
    print("  END                           - Shut down the server")
    print("Example: 1 TAKEOFF 2.5\n")
    
    while server_running:
        try:
            input_data = input("Command > ").strip().upper()
            
            # Handle empty input
            if not input_data:
                continue
                
            input_split = input_data.split()
            
            # Check for END command first
            if input_data == "END":
                logging.info("Shutting down server...")
                server_running = False
                # Close all connections
                for label, conn in connections.items():
                    try:
                        conn.close()
                    except:
                        pass
                os._exit(0)
            
            # Check for help command
            if input_data == "HELP":
                print("\n=== Tower Command Interface ===")
                print("Available commands:")
                print("  <label> TAKEOFF [altitude]    - Take off to specified altitude (default 3m)")
                print("  <label> WAYPOINT x y z        - Move to relative waypoint")
                print("  <label> RTH                   - Return to home")
                print("  <label> LAND                  - Land the drone")
                print("  <label> PRECISION_LAND        - Execute precision landing")
                print("  <label> STANDBY               - Hover in place")
                print("  END                           - Shut down the server")
                print("  DRONES                        - List connected drones")
                print("Example: 1 TAKEOFF 2.5\n")
                continue
                
            # Check for DRONES command
            if input_data == "DRONES":
                if connections:
                    print(f"Connected drones: {list(connections.keys())}")
                else:
                    print("No drones connected")
                continue
            
            # Validate command format
            if len(input_split) < 2:
                print("Invalid input. Format: <label> <command> <params>")
                print("Type HELP for available commands")
                continue
            
            target_label = input_split[0]
            command = ' '.join(input_split[1:])
            
            # Validate command type
            valid_commands = ["TAKEOFF", "RTH", "STANDBY", "RESUME", "LAND", "PRECISION_LAND"]
            if command not in valid_commands and not command.startswith("WAYPOINT") and not command.startswith("ABSOLUTE_WAYPOINT"):
                print(f"Invalid command: {command}")
                print(f"Valid commands: {', '.join(valid_commands)}, WAYPOINT <x> <y> <z>")
                continue
            
            # Special handling for WAYPOINT command
            if command.startswith("WAYPOINT") or command.startswith("ABSOLUTE_WAYPOINT"):
                parts = command.split()
                if len(parts) != 4:
                    print("Invalid WAYPOINT format. Use: WAYPOINT <x> <y> <z>")
                    continue
                try:
                    float(parts[1]), float(parts[2]), float(parts[3])
                except ValueError:
                    print("Invalid WAYPOINT coordinates. Must be numbers.")
                    continue
            
            if command == "LAND":
                disable_avoidance_detection = True
                logging.info("Disabling avoidance detection for landing")
            elif command == "TAKEOFF":
                disable_avoidance_detection = False
                logging.info("Enabling avoidance detection after takeoff")

            # Send command to drone
            if target_label in connections:
                try:
                    connections[target_label].sendall(command.encode())
                    print(f"Sent command to Drone {target_label}: {command}")
                    
                    # Wait for response with timeout
                    connections[target_label].settimeout(5)
                    try:
                        response = connections[target_label].recv(1024).decode()
                        print(f"Response from Drone {target_label}: {response}")
                    except socket.timeout:
                        print(f"No response from Drone {target_label} (timed out)")
                    finally:
                        connections[target_label].settimeout(None)
                        
                except ConnectionResetError:
                    logging.warning(f"Connection lost with Drone {target_label}")
                    del connections[target_label]
                except Exception as e:
                    logging.error(f"Error sending command to Drone {target_label}: {e}")
            else:
                print(f"No connection with Drone {target_label}")
                if connections:
                    print(f"Available drones: {list(connections.keys())}")
                else:
                    print("No drones connected")
                
        except KeyboardInterrupt:
            print("\nShutting down server...")
            server_running = False
            for conn in connections.values():
                try:
                    conn.close()
                except:
                    pass
            os._exit(0)
        except Exception as e:
            logging.error(f"Error processing command: {e}")
            print("Please try again.")
            continue

if __name__ == "__main__":
    logging.info("Starting Tower Control System")
    
    # Start thread to handle server commands
    command_thread = threading.Thread(target=receive_server_commands, daemon=True)
    command_thread.start()
    
    # Start thread for periodic full updates
    update_thread = threading.Thread(target=periodic_full_update, daemon=True)
    update_thread.start()
    logging.info("Starting periodic data update thread")
    
    # Start server threads for each port
    for port in PORTS:
        server_thread = threading.Thread(target=start_server, args=(port,))
        server_thread.start()
        
    try:
        send_command()
    except Exception as e:
        logging.critical(f"Fatal error: {e}")
        os._exit(1)
