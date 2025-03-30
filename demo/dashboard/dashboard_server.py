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
    "2": {
        "lat": -35.3634,
        "lon": 149.1654,
        "alt": 15,
        "status": "STANDBY"
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

def receive_gps_data():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((TOWER_IP, TOWER_PORT))
        server_socket.listen()
        print(f"Listening for GPS data from tower on port {TOWER_PORT}")

        while True:
            conn, addr = server_socket.accept()
            with conn:
                data = conn.recv(1024).decode()
                if not data:
                    continue

                try:
                    gps_update = json.loads(data)
                    drone_id = gps_update["drone_id"]
                    lat, lon, alt = gps_update["lat"], gps_update["lon"], gps_update["alt"]

                    drone_data[drone_id] = {
                        "lat": lat,
                        "lon": lon,
                        "alt": alt,
                        "status": drone_data.get(drone_id, {}).get("status", "STANDBY")
                    }

                except json.JSONDecodeError:
                    print("Error: Invalid GPS data format received")

def send_command_to_tower(drone_id, command):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((TOWER_IP, TOWER_COMMAND_PORT))
            command_message = f"{drone_id} {command}"
            s.sendall(command_message.encode())
            print(f"Command sent to tower: {drone_id} {command}")
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
    print('Web client connected')

@socketio.on('disconnect')
def handle_disconnect():
    print('Web client disconnected')

@socketio.on('send_command')
def handle_command(data):
    # Forward command to the tower system
    drone_id = data['drone_id']
    command = data['command']
    print(f"Received command for Drone {drone_id}: {command}")
    
    if drone_id in drone_data:
        send_command_to_tower(drone_id, command)
        # Update status immediately in UI
        drone_data[drone_id]["status"] = command
        socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})

def background_update():
    """Send real-time updates to clients"""
    while True:
        socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})
        time.sleep(1)

if __name__ == '__main__':
    print("Dashboard server starting...")
    # Start the background thread for updates
    update_thread = threading.Thread(target=background_update)
    update_thread.daemon = True
    update_thread.start()

    gps_thread = threading.Thread(target=receive_gps_data, daemon=True)
    gps_thread.start()
    
    print("Dashboard available at http://127.0.0.1:5000")
    # Start the web server with debug mode off
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)