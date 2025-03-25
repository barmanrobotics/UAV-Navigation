from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
from flask_cors import CORS
import threading
import time
import json

# Import or access tower data
# This could be done by modifying tower_2.py to expose its data
# or by creating a data sharing mechanism

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

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('send_command')
def handle_command(data):
    # Forward command to the tower system
    drone_id = data['drone_id']
    command = data['command']
    print(f"Received command for Drone {drone_id}: {command}")
    
    # Process commands with parameters appropriately
    # For example, extract altitude from TAKEOFF command or coordinates from WAYPOINT
    command_parts = command.split()
    command_type = command_parts[0]
    
    # In a real implementation, you would send this command to the actual drone
    # For this MVP, we'll just update the status
    
    # Update the drone status in our mock data
    if drone_id in drone_data:
        drone_data[drone_id]["status"] = command
        
        # If it's a TAKEOFF command, we could simulate altitude change
        if command_type == "TAKEOFF" and len(command_parts) > 1:
            try:
                target_alt = float(command_parts[1])
                # For demonstration, set the drone's altitude to the target
                drone_data[drone_id]["alt"] = target_alt
            except ValueError:
                pass
                
        # If it's a WAYPOINT command, we could simulate position change
        elif command_type == "WAYPOINT" and len(command_parts) >= 4:
            try:
                x_offset = float(command_parts[1])
                y_offset = float(command_parts[2])
                z_offset = float(command_parts[3])
                
                # For demonstration, adjust the position slightly
                # In a real implementation, you would calculate actual GPS coordinates
                drone_data[drone_id]["lat"] += x_offset * 0.00001
                drone_data[drone_id]["lon"] += y_offset * 0.00001
                drone_data[drone_id]["alt"] += z_offset
            except ValueError:
                pass
                
        # Broadcast the updated data to all clients
        socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})

def background_update():
    """Simulate real-time updates from the tower system"""
    while True:
        # In a real implementation, this would get data from the tower
        socketio.emit('data_update', {"drones": drone_data, "towers": tower_data})
        time.sleep(1)

if __name__ == '__main__':
    # Start the background thread for updates
    update_thread = threading.Thread(target=background_update)
    update_thread.daemon = True
    update_thread.start()
    
    # Start the web server with debug mode off
    socketio.run(app, host='0.0.0.0', port=5000, debug=False) 