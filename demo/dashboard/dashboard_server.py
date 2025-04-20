from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
from flask_cors import CORS
import threading
import time
import json
import socket

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Mock data for initial development
drone_data = {
    "1": {
        "status": "NOT INITIALIZED"
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

TOWER_IP = "0.0.0.0"  # Changed to localhost for local testing
TOWER_PORT = 6553
TOWER_COMMAND_PORT = 6542

def receive_gps_data():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server_socket.bind(('0.0.0.0', TOWER_PORT))
            server_socket.listen()
            print(f"Listening for GPS data from tower on port {TOWER_PORT}...")

            while True:
                try:
                    conn, addr = server_socket.accept()
                    print(f"Got GPS connection from {addr}")
                    with conn:
                        data = conn.recv(1024).decode()
                        if not data:
                            continue

                        try:
                            gps_update = json.loads(data)
                            drone_id = gps_update["drone_id"]
                            lat, lon, alt = gps_update["lat"], gps_update["lon"], gps_update["alt"]
                            
                            print(f"Received GPS data: Drone {drone_id} at lat={lat}, lon={lon}, alt={alt}")
                            
                            # Initialize drone if not present
                            if drone_id not in drone_data:
                                drone_data[drone_id] = {"status": "CONNECTED"}
                            else:
                                current_status = drone_data[drone_id].get("status", "CONNECTED")
                            
                            # Make sure all required fields exist
                            drone_data[drone_id] = {
                                "lat": lat,
                                "lon": lon,
                                "alt": alt,
                                "status": current_status
                            }
                            
                            # Force an immediate update to all clients
                            socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})
                            print(f"Updated drone_data: {json.dumps(drone_data, indent=2)}")
                            
                        except json.JSONDecodeError as e:
                            print(f"Invalid GPS data format received: {e}")
                except Exception as e:
                    print(f"Error accepting GPS connection: {e}")
        except Exception as e:
            print(f"Error binding to GPS port {TOWER_PORT}: {e}")
            print("Will retry in 10 seconds...")
            time.sleep(10)
            receive_gps_data()  # Retry binding

def send_command_to_tower(drone_id, command):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5)  # Set a reasonable timeout
            try:
                s.connect((TOWER_IP, TOWER_COMMAND_PORT))
                command_message = f"{drone_id} {command}"
                # Update status before sending in case of network issues
                if drone_id in drone_data:
                    drone_data[drone_id]["status"] = " ".join(command_message.split()[1:])
                    # Ensure the UI updates when a command is sent
                    socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})
                s.sendall(command_message.encode())
                print(f"Command sent to tower: {command_message}")
                return True
            except ConnectionRefusedError:
                print(f"Connection refused - Tower not running on {TOWER_IP}:{TOWER_COMMAND_PORT}")
                return False
            except socket.timeout:
                print(f"Connection timeout - Tower not responding on {TOWER_IP}:{TOWER_COMMAND_PORT}")
                return False
    except Exception as e:
        print(f"Error sending command to tower: {e}")
        return False

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/data')
def get_data():
    print(f"API data request, returning: {json.dumps(drone_data, indent=2)}")
    return jsonify({
        "drones": drone_data,
        "towers": tower_data
    })

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    # Send current data to newly connected client
    socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('send_command')
def handle_command(data):
    # Forward command to the tower system
    drone_id = data['drone_id']
    command = data['command']
    print(f"Received command for Drone {drone_id}: {command}")
    
    if drone_id in drone_data:
        # Update status immediately even if command fails
        drone_data[drone_id]["status"] = command 
        socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})
        send_command_to_tower(drone_id, command)

def background_update():
    """Emit real-time updates to all connected clients"""
    print("Starting background update thread")
    while True:
        # Send current data to all connected clients
        socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})
        time.sleep(1)

if __name__ == '__main__':
    # Start the background thread for updates
    update_thread = threading.Thread(target=background_update)
    update_thread.daemon = True
    update_thread.start()

    gps_thread = threading.Thread(target=receive_gps_data, daemon=True)
    gps_thread.start()
    
    # Start the web server with debug mode off
    print("Starting Flask server...")
    socketio.run(app, host='0.0.0.0', port=5001, debug=False)