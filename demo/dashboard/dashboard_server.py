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
        "lat": -35.3632,
        "lon": 149.1652,
        "alt": 10,
        "status": "STANDBY"
    },
}

tower_data = {
    "tower1": {
        "lat": -35.3633,
        "lon": 149.1653
    }
}

# Tower Pi configuration (Raspberry Pi)
TOWER_IP = '10.203.181.232'  # IP address of the Raspberry Pi
TOWER_PORT = 6553            # Port for receiving GPS data
TOWER_COMMAND_PORT = 6542    # Port for sending commands

def receive_gps_data():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server_socket.bind(('0.0.0.0', TOWER_PORT))  # Listen on all interfaces
            server_socket.listen()
            print(f"Listening for GPS data from tower at {TOWER_IP} on port {TOWER_PORT}...")

            while True:
                conn, addr = server_socket.accept()
                print(f"Received connection from {addr}")
                with conn:
                    data = conn.recv(1024).decode()
                    if not data:
                        continue

                    try:
                        print(f"Received raw data: {data}")
                        gps_update = json.loads(data)
                        drone_id = gps_update["drone_id"]
                        lat, lon, alt = gps_update["lat"], gps_update["lon"], gps_update["alt"]
                        
                        print(f"Parsed GPS data - Drone {drone_id}: lat={lat}, lon={lon}, alt={alt}")
                        
                        # Create drone entry if it doesn't exist
                        if drone_id not in drone_data:
                            drone_data[drone_id] = {"status": "STANDBY"}
                            print(f"Created new drone entry for Drone {drone_id}")
                            
                        current_status = drone_data[drone_id].get("status", "STANDBY")
                        drone_data[drone_id] = {
                            "lat": lat,
                            "lon": lon,
                            "alt": alt,
                            "status": current_status
                        }
                        print(f"Updated drone {drone_id} data: {drone_data[drone_id]}")

                    except json.JSONDecodeError as e:
                        print(f"Invalid GPS data format received: {e}")
                        print(f"Raw data was: {data}")
                    except KeyError as e:
                        print(f"Missing key in GPS data: {e}")
                        print(f"Data received: {data}")
                    except Exception as e:
                        print(f"Error processing GPS data: {e}")
                        print(f"Data received: {data}")
        except Exception as e:
            print(f"Error setting up GPS receiver: {e}")
            time.sleep(5)  # Wait before retrying

def send_command_to_tower(drone_id, command):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            print(f"Connecting to tower at {TOWER_IP}:{TOWER_COMMAND_PORT}...")
            s.connect((TOWER_IP, TOWER_COMMAND_PORT))
            command_message = f"{drone_id} {command}"
            print(f"Sending command: {command_message}")
            s.sendall(command_message.encode())
            
            # Update local status
            if drone_id in drone_data:
                drone_data[drone_id]["status"] = command.split()[0]
                print(f"Updated drone {drone_id} status to {drone_data[drone_id]['status']}")
            
            # Wait for response
            try:
                s.settimeout(2)
                response = s.recv(1024).decode()
                print(f"Tower response: {response}")
            except socket.timeout:
                print("No response from tower (timeout)")
                
    except ConnectionRefusedError:
        print(f"Connection refused by tower at {TOWER_IP}:{TOWER_COMMAND_PORT}")
    except Exception as e:
        print(f"Error sending command to tower: {e}")
        
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/data')
def get_data():
    return jsonify({
        "drones": drone_data,
        "towers": tower_data
    })

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    # Send initial data to the client
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
    
    send_command_to_tower(drone_id, command)

def background_update():
    """Send real-time updates to connected clients"""
    while True:
        socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})
        time.sleep(1)

if __name__ == '__main__':
    # Start the background thread for updates
    update_thread = threading.Thread(target=background_update, daemon=True)
    update_thread.start()

    # Start the GPS data receiver thread
    gps_thread = threading.Thread(target=receive_gps_data, daemon=True)
    gps_thread.start()
    
    # Start the web server
    print("Starting dashboard server...")
    socketio.run(app, host='0.0.0.0', port=5001, debug=False)
