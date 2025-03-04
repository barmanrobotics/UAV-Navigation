# node_pi_client.py
import socket
from pymavlink import mavutil  # type: ignore
import time
import threading
import sys

# MAVLink Connection to PX4
connection = mavutil.mavlink_connection('udpin:localhost:14550')
connection.wait_heartbeat()

# Client Configuration
HUB_IP = 'localhost'
# Get the port number from command line argument
if len(sys.argv) < 2:
    print("Usage: python3 node_pi_client.py <PORT>")
    sys.exit(1)

PORT = int(sys.argv[1])  # Get the port from the command line argument

# Function to execute MAVLink commands or call external scripts
def execute_command(command):
    if command == "TAKEOFF":
        print("Executing TAKEOFF")
        connection.mav.set_mode_send(
    	    connection.target_system,
    	    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    	    4  # GUIDED mode for ArduPilot
    	)
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(1)
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, 10  # 10 meters altitude
        )
    elif command.startswith("WAYPOINT"):
        print("Executing WAYPOINT NAVIGATION")
        params = command.split()
        if len(params) != 4:  # WAYPOINT x y z
            print("Invalid waypoint parameters. Format: WAYPOINT x y z")
            return
        try:
            x_offset, y_offset, z_offset = map(float, params[1:])
            from way_point import way_point
            way_point(connection, x_offset, y_offset, z_offset)
        except Exception as e:
            print(f"Error executing waypoint navigation: {e}")
    elif command == "RTH":
        print("Executing RTH (Return to Home)")
        from rth import rth
        rth(connection)
    elif command == "STANDBY":
        print("Entering STANDBY (Hover in place)")
        msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        hover_x = msg.x
        hover_y = msg.y
        hover_z = msg.z
        
        connection.mav.set_position_target_local_ned_send(
            0,
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000),
            hover_x, hover_y, hover_z,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
    elif command == "PRECISION_LAND":
        print("Executing Precision Landing via external script")
        from precision_landing import precision_landing
        precision_landing(connection)

# Function to periodically send GPS coordinates to the hub
def send_gps_coordinates(client):
    while True:
        try:
            msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1e3
                gps_data = f"GPS {lat} {lon} {alt}"
                client.sendall(gps_data.encode())
            time.sleep(3)  # Add delay to prevent flooding
        except Exception as e:
            print(f"Error sending GPS data: {e}")
            break

def receive_messages(client_socket):
    while True:
        try:
            message = client_socket.recv(1024).decode()
            if not message:
                print("Server closed connection")
                break
            print(f"Received from server: {message}")
            execute_command(message)
            client_socket.sendall("Command executed".encode())
        except ConnectionResetError:
            print("Connection reset by server")
            break
        except Exception as e:
            print(f"Error receiving message: {e}")
            break
    print("Disconnected from server")

def main():
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect((HUB_IP, PORT))
        print(f"Connected to server at {HUB_IP}:{PORT}")
        
        # Start receive thread
        receive_thread = threading.Thread(target=receive_messages, args=(client,))
        receive_thread.daemon = True
        receive_thread.start()
        
        # Send periodic messages
        while True:
            try:
                client.send("Drone status: OK".encode())
                time.sleep(5)
            except:
                break
                
    except Exception as e:
        print(f"Connection error: {e}")
    finally:
        client.close()

if __name__ == "__main__":
    main()
