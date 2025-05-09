from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO
from flask_cors import CORS
import threading
import time
import json
import socket
import math
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Mock data for initial development
drone_data = {
    "1": {
        "lat": 39.3290775819491,
        "lon": -76.62073373794556,
        "alt": 0.0,
        "vel": 0.0,
        "hdg": 0.0,
        "status": "CONNECTED"
    },
    "2": {
        "status": "NOT INITIALIZED"
    }
}

tower_data = {
    "tower1": {
        "lat": -35.3633,
        "lon": 149.1653
    }
}

TOWER_IP = "0.0.0.0"
TOWER_PORT = 6553
TOWER_COMMAND_PORT = 6542

# Earth radius in meters
EARTH_RADIUS = 6371000  

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate the great circle distance between two points on the earth"""
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    
    # Earth radius in meters
    return c * EARTH_RADIUS

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate the bearing from point 1 to point 2"""
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Calculate bearing
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(y, x)
    
    # Convert to degrees
    bearing = math.degrees(bearing)
    
    # Normalize to 0-360
    bearing = (bearing + 360) % 360
    
    return bearing

def gps_to_local_coordinates(drone_lat, drone_lon, target_lat, target_lon, drone_hdg):
    """Convert GPS coordinates to local coordinates relative to drone position and heading"""
    # Calculate distance between points
    distance = haversine_distance(drone_lat, drone_lon, target_lat, target_lon)
    
    # Calculate bearing from drone to target
    bearing = calculate_bearing(drone_lat, drone_lon, target_lat, target_lon)
    
    # Adjust bearing based on drone heading
    relative_bearing = (bearing - drone_hdg) % 360
    
    # Convert polar coordinates (distance, bearing) to cartesian (x, y)
    # In NED (North-East-Down) coordinate system:
    # x is positive towards North
    # y is positive towards East
    x = distance * math.cos(math.radians(relative_bearing))
    y = distance * math.sin(math.radians(relative_bearing))
    
    # Round to 2 decimal places for readability
    return round(x, 2), round(y, 2)

def receive_gps_data():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server_socket.bind((TOWER_IP, TOWER_PORT))
            server_socket.listen()
            logging.info(f"Listening for GPS data from tower on port {TOWER_PORT}...")

            while True:
                try:
                    conn, addr = server_socket.accept()
                    with conn:
                        data = conn.recv(1024).decode()
                        if not data:
                            continue

                        try:
                            # Check if it's a GPS message
                            if data.startswith("GPS "):
                                parts = data.split()
                                # Check if we have all needed parts (including battery)
                                if len(parts) >= 7:  # GPS lat lon alt vel hdg battery
                                    drone_id = "1"  # Default drone ID
                                    lat, lon, alt, vel, hdg = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5])
                                    
                                    # Get battery if available (new format)
                                    battery = int(parts[6]) if len(parts) >= 7 else None
                                    
                                    # Initialize drone if not present
                                    if drone_id not in drone_data:
                                        drone_data[drone_id] = {"status": "CONNECTED"}
                                        logging.info(f"New drone registered: Drone {drone_id}")
                                    else:
                                        current_status = drone_data[drone_id].get("status", "CONNECTED")
                                    
                                    # Update drone data
                                    drone_data[drone_id] = {
                                        "lat": lat,
                                        "lon": lon,
                                        "alt": alt,
                                        "vel": vel,
                                        "hdg": hdg,
                                        "status": current_status
                                    }
                                    
                                    # Add battery data if available
                                    if battery is not None:
                                        drone_data[drone_id]["battery"] = battery
                                    
                                    # Emit data update on significant position changes
                                    socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})
                                else:
                                    logging.warning(f"Invalid GPS data format: {data}")
                            else:
                                # Handle non-GPS messages
                                logging.info(f"Received non-GPS message: {data}")
                                try:
                                    # Try to parse as JSON
                                    json_data = json.loads(data)
                                    
                                    # Check if it contains drone data
                                    if 'drone_id' in json_data:
                                        drone_id = str(json_data['drone_id'])
                                        
                                        # Initialize drone if not present
                                        if drone_id not in drone_data:
                                            drone_data[drone_id] = {"status": "CONNECTED"}
                                            logging.info(f"New drone registered from JSON: Drone {drone_id}")
                                        else:
                                            # If status is NOT INITIALIZED, change it to CONNECTED when we get data
                                            current_status = drone_data[drone_id].get("status", "CONNECTED")
                                            if current_status == "NOT INITIALIZED":
                                                current_status = "CONNECTED"
                                                logging.info(f"Changing drone {drone_id} status from NOT INITIALIZED to CONNECTED")
                                            
                                            json_data["status"] = current_status
                                        
                                        # Update drone data with JSON fields
                                        for key, value in json_data.items():
                                            if key != 'drone_id':  # Already have the ID
                                                drone_data[drone_id][key] = value
                                        
                                        logging.info(f"Updated drone {drone_id} from JSON: {json_data}")
                                        
                                        # Emit data update
                                        socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})
                                except json.JSONDecodeError:
                                    # Not a valid JSON, continue with normal processing
                                    logging.info("Non-JSON message received, ignoring")
                        except json.JSONDecodeError as e:
                            logging.error(f"Invalid GPS data format received: {e}")
                        except KeyError as e:
                            logging.error(f"Missing field in GPS data: {e}")
                        except ValueError as e:
                            logging.error(f"Invalid value in GPS data: {e}")
                except Exception as e:
                    logging.error(f"Error accepting GPS connection: {e}")
        except Exception as e:
            logging.error(f"Error binding to GPS port {TOWER_PORT}: {e}")
            time.sleep(10)
            receive_gps_data()  # Retry binding

def send_command_to_tower(drone_id, command):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5)  # Set timeout for connection
            try:
                s.connect((TOWER_IP, TOWER_COMMAND_PORT))
                command_message = f"{drone_id} {command}"
                
                # Update status in local data
                if drone_id in drone_data:
                    drone_data[drone_id]["status"] = " ".join(command_message.split()[1:])
                    
                s.sendall(command_message.encode())
                logging.info(f"Command sent to tower: {command_message}")
                
                # Force update to clients
                socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})
                socketio.emit('command_ack', {"success": True, "message": f"Command '{command}' sent to Drone {drone_id}"})
                return True
            except ConnectionRefusedError:
                error_msg = f"Connection refused to tower at {TOWER_IP}:{TOWER_COMMAND_PORT}"
                logging.error(error_msg)
                socketio.emit('command_ack', {"success": False, "message": error_msg})
                return False
            except socket.timeout:
                error_msg = f"Connection to tower timed out"
                logging.error(error_msg)
                socketio.emit('command_ack', {"success": False, "message": error_msg})
                return False
    except Exception as e:
        error_msg = f"Error sending command to tower: {e}"
        logging.error(error_msg)
        socketio.emit('command_ack', {"success": False, "message": error_msg})
        return False
        
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/data')
def get_data():
    return jsonify({
        "drones": drone_data,
        "towers": tower_data
    })

@app.route('/api/waypoint', methods=['POST'])
def calculate_waypoint():
    """Calculate relative waypoint coordinates from GPS position"""
    try:
        data = request.json
        drone_id = data.get('drone_id')
        target_lat = float(data.get('target_lat'))
        target_lon = float(data.get('target_lon'))
        target_alt = float(data.get('target_alt', 0))  # Default to current altitude
        
        # Validate input
        if not drone_id or drone_id not in drone_data:
            return jsonify({"success": False, "message": "Invalid drone ID"}), 400
            
        # Get drone's current position and heading
        drone = drone_data.get(drone_id)
        if not drone or 'lat' not in drone or 'lon' not in drone:
            return jsonify({"success": False, "message": "Drone position not available"}), 400
            
        # Calculate relative coordinates
        drone_lat = drone['lat']
        drone_lon = drone['lon']
        drone_hdg = drone.get('hdg', 0)
        drone_alt = drone.get('alt', 0)
        
        # If no target altitude specified, maintain current altitude
        if target_alt == 0:
            target_alt = drone_alt
            
        # Calculate z as altitude difference (positive is up)
        z_offset = target_alt - drone_alt
        
        # Calculate x, y offsets based on GPS coordinates
        x_offset, y_offset = gps_to_local_coordinates(
            drone_lat, drone_lon, target_lat, target_lon, drone_hdg
        )
        
        return jsonify({
            "success": True,
            "waypoint": {
                "x": x_offset,
                "y": y_offset,
                "z": z_offset,
                "distance": math.sqrt(x_offset**2 + y_offset**2),
                "drone_position": {
                    "lat": drone_lat,
                    "lon": drone_lon,
                    "alt": drone_alt,
                    "hdg": drone_hdg
                },
                "target_position": {
                    "lat": target_lat,
                    "lon": target_lon,
                    "alt": target_alt
                }
            }
        })
    except Exception as e:
        logging.error(f"Error calculating waypoint: {e}")
        return jsonify({"success": False, "message": str(e)}), 500

@socketio.on('connect')
def handle_connect():
    logging.info('Client connected')
    # Send current data to the newly connected client
    socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})
    
    # Request a refresh of all drone data from the tower
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2)  # Set a short timeout
            try:
                s.connect((TOWER_IP, TOWER_COMMAND_PORT))
                s.sendall("REFRESH".encode())
                logging.info("Sent REFRESH command to tower")
            except Exception as e:
                logging.error(f"Error sending REFRESH command: {e}")
    except Exception as e:
        logging.error(f"Error creating socket for REFRESH: {e}")

@socketio.on('disconnect')
def handle_disconnect():
    logging.info('Client disconnected')

@socketio.on('send_command')
def handle_command(data):
    # Forward command to the tower system
    drone_id = data.get('drone_id')
    command = data.get('command')
    
    if not drone_id or not command:
        socketio.emit('command_ack', {"success": False, "message": "Missing drone_id or command"})
        return
        
    logging.info(f"Received command for Drone {drone_id}: {command}")
    
    # Special case for REFRESH command
    if command == 'REFRESH':
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2)  # Short timeout
                try:
                    s.connect((TOWER_IP, TOWER_COMMAND_PORT))
                    s.sendall("REFRESH".encode())
                    logging.info("Sent REFRESH command to tower")
                    socketio.emit('command_ack', {"success": True, "message": "Requested data refresh from tower"})
                    return
                except Exception as e:
                    logging.error(f"Error sending REFRESH command: {e}")
                    socketio.emit('command_ack', {"success": False, "message": f"Error refreshing data: {str(e)}"})
                    return
        except Exception as e:
            logging.error(f"Error creating socket for REFRESH: {e}")
            socketio.emit('command_ack', {"success": False, "message": f"Error creating socket: {str(e)}"})
            return
    
    if drone_id in drone_data:
        send_command_to_tower(drone_id, command)
    else:
        socketio.emit('command_ack', {"success": False, "message": f"Drone {drone_id} not found"})

@socketio.on('map_click')
def handle_map_click(data):
    """Handle map click events to generate waypoints"""
    try:
        drone_id = data.get('drone_id')
        lat = data.get('lat')
        lon = data.get('lon')
        alt = data.get('alt', 0)  # Optional altitude
        
        logging.info(f"Map click at {lat}, {lon} for Drone {drone_id}")
        
        if not drone_id or drone_id not in drone_data:
            socketio.emit('command_ack', {"success": False, "message": "Invalid drone ID"})
            return
            
        # Get drone's current position
        drone = drone_data.get(drone_id)
        if not drone or 'lat' not in drone or 'lon' not in drone:
            socketio.emit('command_ack', {"success": False, "message": "Drone position not available"})
            return
            
        # Calculate relative coordinates
        drone_lat = drone['lat']
        drone_lon = drone['lon'] 
        drone_hdg = drone.get('hdg', 0)
        drone_alt = drone.get('alt', 0)
        
        # Calculate z as altitude difference (default to maintaining current altitude)
        z_offset = alt - drone_alt if alt != 0 else 0
        
        # Calculate x, y offsets based on GPS coordinates
        x_offset, y_offset = gps_to_local_coordinates(
            drone_lat, drone_lon, lat, lon, drone_hdg
        )
        
        # Create waypoint command
        command = f"WAYPOINT {x_offset} {y_offset} {z_offset}"
        
        # Send command to tower
        send_command_to_tower(drone_id, command)
        
        # Emit waypoint info to the client
        socketio.emit('waypoint_created', {
            "drone_id": drone_id,
            "x": x_offset,
            "y": y_offset,
            "z": z_offset,
            "target": {"lat": lat, "lon": lon, "alt": alt if alt != 0 else drone_alt},
            "distance": math.sqrt(x_offset**2 + y_offset**2)
        })
        
    except Exception as e:
        error_msg = f"Error processing map click: {e}"
        logging.error(error_msg)
        socketio.emit('command_ack', {"success": False, "message": error_msg})

def background_update():
    """Send real-time updates to connected clients"""
    while True:
        # Emit current data to all clients
        socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})
        time.sleep(1)

if __name__ == '__main__':
    # Start the background thread for updates
    update_thread = threading.Thread(target=background_update)
    update_thread.daemon = True
    update_thread.start()

    gps_thread = threading.Thread(target=receive_gps_data, daemon=True)
    gps_thread.start()
    
    logging.info(f"Starting dashboard server on port 5001")
    # Start the web server with debug mode off
    socketio.run(app, host='0.0.0.0', port=5001, debug=False)