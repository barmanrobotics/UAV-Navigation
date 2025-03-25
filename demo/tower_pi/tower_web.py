import socket
import threading
import math
import os
import time
import json
from flask import Flask, jsonify, render_template
from flask_socketio import SocketIO
import threading

HOST = '0.0.0.0'  # Listen on all available network interfaces
PORTS = [14550, 14560]

gps_data = {}
connections = {}
label_counter = 1  # Start labeling from 1
server_running = True
avoidance_enabled = 0
avoidance_started = None

import socket
import threading
import math
import os
import time
import json
from flask import Flask, jsonify, render_template
from flask_socketio import SocketIO
import threading

HOST = '0.0.0.0'  # Listen on all available network interfaces
PORTS = [14550, 14560]

gps_data = {}
connections = {}
label_counter = 1  # Start labeling from 1
server_running = True
avoidance_enabled = 0
avoidance_started = None

# Calculate the absolute path to the template directory
template_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../dashboard/templates'))

# Initialize Flask and SocketIO with the correct template folder
app = Flask(__name__, template_folder=template_dir)
socketio = SocketIO(app, cors_allowed_origins="*")

# Add web routes
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/data')
def get_data():
    global gps_data, connections
    
    # Format data for the dashboard
    drones_data = {}
    for label, (lat, lon, alt) in gps_data.items():
        drones_data[label] = {
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "status": "ACTIVE" if label in connections else "DISCONNECTED"
        }
    
    # Tower data (could be actual tower GPS or hardcoded)
    tower_data = {
        "tower1": {
            "lat": -35.3632621,
            "lon": 149.165193
        }
    }
    
    return jsonify({
        "drones": drones_data,
        "towers": tower_data
    })

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

def handle_client(conn, label):
    global avoidance_enabled, avoidance_started
    try:
        print(f"Connection established with Drone {label}")
        while True:
            try:
                message = conn.recv(1024).decode()
                if not message:
                    print(f"Connection closed by Drone {label}")
                    break
                # print(f"Drone {label}: {message}")

                # Only respond to GPS data, don't echo other messages
                if message.startswith("GPS"):
                    parts = message.split()
                    if len(parts) == 4:
                        try:
                            lat, lon, alt = float(parts[1]), float(parts[2]), float(parts[3])
                            gps_data[label] = (lat, lon, alt)
                            fixed_coord = (-35.3632621, 149.165193)  # Arbitrary fixed GPS coordinates

                            if label in gps_data:
                                distance = haversine(gps_data[label][0:2], fixed_coord)
                                # print(f"Distance from Drone {label} to fixed point: {distance:.2f} meters")
                            
                            if len(gps_data) > 1:
                                drone_labels = list(gps_data.keys())
                                d1, d2 = drone_labels[0], drone_labels[1]
                                distance = haversine(gps_data[d1][0:2], gps_data[d2][0:2])
                                alt_diff = abs(gps_data[d1][2] - gps_data[d2][2])
                                # print(f"Altitude difference: {alt_diff}")
                                # print(f"Distance between Drone {d1} and Drone {d2}: {distance:.2f} meters")
                                if distance < 15 and alt_diff < 2:
                                    if avoidance_enabled==0:
                                        print("avoidance_enabled_0")
                                        connections[d1].sendall("STOP".encode())
                                        connections[d2].sendall("STOP".encode())
                                        avoidance_enabled = 1
                                    elif avoidance_enabled==1:
                                        print("avoidance_enabled_1")
                                        connections[d2].sendall("AVOID".encode())
                                        avoidance_enabled = 2
                                elif (distance > 15 or alt_diff > 2) and avoidance_enabled==2:
                                    print("avoidance_enabled_2")
                                    connections[d1].send("RESUME".encode())
                                    connections[d2].send("RESUME".encode())
                                    avoidance_started = time.time()
                                    avoidance_enabled = 3
                                elif avoidance_enabled == 3 and (time.time()-avoidance_started)>12:
                                    print("Avoidance detection resumed.")
                                    avoidance_enabled = 0
                                    
                        except ValueError as e:
                            print(f"Invalid GPS data from Drone {label}")
                            print(e)
            except ConnectionResetError:
                print(f"Connection reset by Drone {label}")
                break
            except Exception as e:
                print(f"Error handling message from Drone {label}: {e}")
                break
    finally:
        if label in connections:
            del connections[label]
        conn.close()
        print(f"Cleaned up connection for Drone {label}")

def start_server(port):
    global label_counter, server_running
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((HOST, port))
            s.listen()
            print(f"Server listening on {HOST}:{port}")
            while server_running:
                try:
                    conn, addr = s.accept()
                    label = str(label_counter)
                    connections[label] = conn
                    print(f"Drone {label} connected from {addr} on port {port}")
                    client_thread = threading.Thread(target=handle_client, args=(conn, label))
                    client_thread.daemon = True
                    client_thread.start()
                    label_counter += 1
                except Exception as e:
                    print(f"Error accepting connection on port {port}: {e}")
                    if not server_running:
                        break
        except Exception as e:
            print(f"Server error on port {port}: {e}")

def send_command():
    global server_running
    while server_running:
        try:
            input_data = input("Enter command (<label> <command> <params>): ").strip().upper()
            
            # Handle empty input
            if not input_data:
                continue
                
            input_split = input_data.split()
            
            # Check for END command first
            if input_data == "END":
                print("Shutting down server...")
                server_running = False
                # Close all connections
                for label, conn in connections.items():
                    try:
                        conn.close()
                    except:
                        pass
                os._exit(0)
            
            # Validate command format
            if len(input_split) < 2:
                print("Invalid input. Format: <label> <command> <params>")
                print("Available commands: TAKEOFF, WAYPOINT <x> <y> <z>, RTH, STANDBY")
                continue
            
            target_label = input_split[0]
            command = ' '.join(input_split[1:])
            
            # Validate command type
            if command not in ["TAKEOFF", "RTH", "STANDBY", "RESUME"] and not command.startswith("WAYPOINT"):
                print("Invalid command.")
                print("Available commands: TAKEOFF, WAYPOINT <x> <y> <z>, RTH, STANDBY")
                continue
            
            # Special handling for WAYPOINT command
            if command.startswith("WAYPOINT"):
                parts = command.split()
                if len(parts) != 4:
                    print("Invalid WAYPOINT format. Use: WAYPOINT <x> <y> <z>")
                    continue
                try:
                    # Verify that coordinates are numbers
                    float(parts[1]), float(parts[2]), float(parts[3])
                except ValueError:
                    print("Invalid WAYPOINT coordinates. Must be numbers.")
                    continue
            
            # Send command to drone
            if target_label in connections:
                try:
                    connections[target_label].sendall(command.encode())
                    response = connections[target_label].recv(1024).decode()
                    print(f"Response from Drone {target_label}: {response}")
                except ConnectionResetError:
                    print(f"Connection lost with Drone {target_label}")
                    del connections[target_label]
                except Exception as e:
                    print(f"Error sending command to Drone {target_label}: {e}")
            else:
                print(f"No connection with Drone {target_label}")
                print(f"Available drones: {list(connections.keys())}")
                
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
            print(f"Error processing command: {e}")
            print("Please try again.")
            continue

def broadcast_data():
    """Send data updates to connected clients"""
    global gps_data, connections, server_running
    
    while server_running:
        try:
            # Format the same data as the API endpoint
            drones_data = {}
            for label, (lat, lon, alt) in gps_data.items():
                status = "UNKNOWN"
                if label in connections:
                    status = "ACTIVE"
                
                drones_data[label] = {
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "status": status
                }
            
            tower_data = {
                "tower1": {
                    "lat": -35.3632621,
                    "lon": 149.165193
                }
            }
            
            socketio.emit('data_update', {
                "drones": drones_data,
                "towers": tower_data
            })
            
            time.sleep(1)  # Update frequency
        except Exception as e:
            print(f"Error in broadcast thread: {e}")
            time.sleep(5)  # Wait before retry

@socketio.on('connect')
def handle_connect():
    print('Web client connected')

@socketio.on('disconnect')
def handle_disconnect():
    print('Web client disconnected')

@socketio.on('send_command')
def handle_web_command(data):
    """Handle commands from the web dashboard"""
    try:
        drone_id = data['drone_id']
        command = data['command']
        
        if drone_id in connections:
            print(f"Sending command to Drone {drone_id}: {command}")
            connections[drone_id].sendall(command.encode())
            # You might want to await a response
        else:
            print(f"No connection with Drone {drone_id}")
    except Exception as e:
        print(f"Error processing web command: {e}")

# Modify the main block to start the web server
if __name__ == "__main__":
    for port in PORTS:
        server_thread = threading.Thread(target=start_server, args=(port,))
        server_thread.daemon = True
        server_thread.start()
    
    # Start the command thread
    command_thread = threading.Thread(target=send_command)
    command_thread.daemon = True
    command_thread.start()
    
    # Start the broadcast thread
    broadcast_thread = threading.Thread(target=broadcast_data)
    broadcast_thread.daemon = True
    broadcast_thread.start()
    
    # Start the web server
    socketio.run(app, host='0.0.0.0', port=5000)

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

def handle_client(conn, label):
    global avoidance_enabled, avoidance_started
    try:
        print(f"Connection established with Drone {label}")
        while True:
            try:
                message = conn.recv(1024).decode()
                if not message:
                    print(f"Connection closed by Drone {label}")
                    break
                # print(f"Drone {label}: {message}")

                # Only respond to GPS data, don't echo other messages
                if message.startswith("GPS"):
                    parts = message.split()
                    if len(parts) == 4:
                        try:
                            lat, lon, alt = float(parts[1]), float(parts[2]), float(parts[3])
                            gps_data[label] = (lat, lon, alt)
                            fixed_coord = (-35.3632621, 149.165193)  # Arbitrary fixed GPS coordinates

                            if label in gps_data:
                                distance = haversine(gps_data[label][0:2], fixed_coord)
                                # print(f"Distance from Drone {label} to fixed point: {distance:.2f} meters")
                            
                            if len(gps_data) > 1:
                                drone_labels = list(gps_data.keys())
                                d1, d2 = drone_labels[0], drone_labels[1]
                                distance = haversine(gps_data[d1][0:2], gps_data[d2][0:2])
                                alt_diff = abs(gps_data[d1][2] - gps_data[d2][2])
                                # print(f"Altitude difference: {alt_diff}")
                                # print(f"Distance between Drone {d1} and Drone {d2}: {distance:.2f} meters")
                                if distance < 15 and alt_diff < 2:
                                    if avoidance_enabled==0:
                                        print("avoidance_enabled_0")
                                        connections[d1].sendall("STOP".encode())
                                        connections[d2].sendall("STOP".encode())
                                        avoidance_enabled = 1
                                    elif avoidance_enabled==1:
                                        print("avoidance_enabled_1")
                                        connections[d2].sendall("AVOID".encode())
                                        avoidance_enabled = 2
                                elif (distance > 15 or alt_diff > 2) and avoidance_enabled==2:
                                    print("avoidance_enabled_2")
                                    connections[d1].send("RESUME".encode())
                                    connections[d2].send("RESUME".encode())
                                    avoidance_started = time.time()
                                    avoidance_enabled = 3
                                elif avoidance_enabled == 3 and (time.time()-avoidance_started)>12:
                                    print("Avoidance detection resumed.")
                                    avoidance_enabled = 0
                                    
                        except ValueError as e:
                            print(f"Invalid GPS data from Drone {label}")
                            print(e)
            except ConnectionResetError:
                print(f"Connection reset by Drone {label}")
                break
            except Exception as e:
                print(f"Error handling message from Drone {label}: {e}")
                break
    finally:
        if label in connections:
            del connections[label]
        conn.close()
        print(f"Cleaned up connection for Drone {label}")

def start_server(port):
    global label_counter, server_running
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((HOST, port))
            s.listen()
            print(f"Server listening on {HOST}:{port}")
            while server_running:
                try:
                    conn, addr = s.accept()
                    label = str(label_counter)
                    connections[label] = conn
                    print(f"Drone {label} connected from {addr} on port {port}")
                    client_thread = threading.Thread(target=handle_client, args=(conn, label))
                    client_thread.daemon = True
                    client_thread.start()
                    label_counter += 1
                except Exception as e:
                    print(f"Error accepting connection on port {port}: {e}")
                    if not server_running:
                        break
        except Exception as e:
            print(f"Server error on port {port}: {e}")

def send_command():
    global server_running
    while server_running:
        try:
            input_data = input("Enter command (<label> <command> <params>): ").strip().upper()
            
            # Handle empty input
            if not input_data:
                continue
                
            input_split = input_data.split()
            
            # Check for END command first
            if input_data == "END":
                print("Shutting down server...")
                server_running = False
                # Close all connections
                for label, conn in connections.items():
                    try:
                        conn.close()
                    except:
                        pass
                os._exit(0)
            
            # Validate command format
            if len(input_split) < 2:
                print("Invalid input. Format: <label> <command> <params>")
                print("Available commands: TAKEOFF, WAYPOINT <x> <y> <z>, RTH, STANDBY")
                continue
            
            target_label = input_split[0]
            command = ' '.join(input_split[1:])
            
            # Validate command type
            if command not in ["TAKEOFF", "RTH", "STANDBY", "RESUME"] and not command.startswith("WAYPOINT"):
                print("Invalid command.")
                print("Available commands: TAKEOFF, WAYPOINT <x> <y> <z>, RTH, STANDBY")
                continue
            
            # Special handling for WAYPOINT command
            if command.startswith("WAYPOINT"):
                parts = command.split()
                if len(parts) != 4:
                    print("Invalid WAYPOINT format. Use: WAYPOINT <x> <y> <z>")
                    continue
                try:
                    # Verify that coordinates are numbers
                    float(parts[1]), float(parts[2]), float(parts[3])
                except ValueError:
                    print("Invalid WAYPOINT coordinates. Must be numbers.")
                    continue
            
            # Send command to drone
            if target_label in connections:
                try:
                    connections[target_label].sendall(command.encode())
                    response = connections[target_label].recv(1024).decode()
                    print(f"Response from Drone {target_label}: {response}")
                except ConnectionResetError:
                    print(f"Connection lost with Drone {target_label}")
                    del connections[target_label]
                except Exception as e:
                    print(f"Error sending command to Drone {target_label}: {e}")
            else:
                print(f"No connection with Drone {target_label}")
                print(f"Available drones: {list(connections.keys())}")
                
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
            print(f"Error processing command: {e}")
            print("Please try again.")
            continue

def broadcast_data():
    """Send data updates to connected clients"""
    global gps_data, connections, server_running
    
    while server_running:
        try:
            # Format the same data as the API endpoint
            drones_data = {}
            for label, (lat, lon, alt) in gps_data.items():
                status = "UNKNOWN"
                if label in connections:
                    status = "ACTIVE"
                
                drones_data[label] = {
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "status": status
                }
            
            tower_data = {
                "tower1": {
                    "lat": -35.3632621,
                    "lon": 149.165193
                }
            }
            
            socketio.emit('data_update', {
                "drones": drones_data,
                "towers": tower_data
            })
            
            time.sleep(1)  # Update frequency
        except Exception as e:
            print(f"Error in broadcast thread: {e}")
            time.sleep(5)  # Wait before retry

@socketio.on('connect')
def handle_connect():
    print('Web client connected')

@socketio.on('disconnect')
def handle_disconnect():
    print('Web client disconnected')

@socketio.on('send_command')
def handle_web_command(data):
    """Handle commands from the web dashboard"""
    try:
        drone_id = data['drone_id']
        command = data['command']
        
        if drone_id in connections:
            print(f"Sending command to Drone {drone_id}: {command}")
            connections[drone_id].sendall(command.encode())
            # You might want to await a response
        else:
            print(f"No connection with Drone {drone_id}")
    except Exception as e:
        print(f"Error processing web command: {e}")

# Modify the main block to start the web server
if __name__ == "__main__":
    for port in PORTS:
        server_thread = threading.Thread(target=start_server, args=(port,))
        server_thread.daemon = True
        server_thread.start()
    
    # Start the command thread
    command_thread = threading.Thread(target=send_command)
    command_thread.daemon = True
    command_thread.start()
    
    # Start the broadcast thread
    broadcast_thread = threading.Thread(target=broadcast_data)
    broadcast_thread.daemon = True
    broadcast_thread.start()
    
    # Start the web server
    socketio.run(app, host='0.0.0.0', port=5000)

