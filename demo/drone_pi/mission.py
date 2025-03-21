import socket
from pymavlink import mavutil  # type: ignore
import time
import threading
import sys

# Client Configuration
HUB_IP = 'localhost'
if len(sys.argv) < 2:
    print("Usage: python3 node_pi_client.py <PORT>")
    sys.exit(1)

PORT = int(sys.argv[1])

# MAVLink Connection to PX4
connection = mavutil.mavlink_connection(f'udpin:localhost:{PORT}')
connection.wait_heartbeat()

current_command = None  # Track ongoing command

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
            # time.sleep(0.5)  # Add delay to prevent flooding
        except Exception as e:
            print(f"Error sending GPS data: {e}")
            break

def execute_command(command):
    global current_command

    if command==None:
        return

    if command == "TAKEOFF":
        print("Executing TAKEOFF")
        msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        current_command = f"ABSOLUTE_WAYPOINT {msg.x} {msg.y} {msg.z-10}"

        print("TAKEOFF EST POS ", current_command)
        
        connection.mav.set_mode_send(
            connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4
        )
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(1)
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, 10
        )

    elif command.startswith("ABSOLUTE_WAYPOINT"):
        try:
            params = command.split()
            target_x, target_y, target_z = map(float, params[1:])
            from way_point import way_point
            
            print("CUR ABS COMMAND UPDATED", current_command)
            way_point(connection, target_x, target_y, target_z)

        except Exception as e:
            print(f"Error executing waypoint navigation: {e}")

    elif command.startswith("WAYPOINT"):
        print("Executing WAYPOINT NAVIGATION")
        params = command.split()
        if len(params) != 4:
            print("Invalid waypoint parameters. Format: WAYPOINT x y z")
            return
        try:
            x_offset, y_offset, z_offset = map(float, params[1:])
            from way_point import way_point
            
            msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)

            current_x = msg.x
            current_y = msg.y
            current_z = msg.z
            
            target_x = current_x + x_offset
            target_y = current_y + y_offset
            target_z = current_z - z_offset

            current_command = f"ABSOLUTE_WAYPOINT {target_x} {target_y} {target_z}"
            print("CUR COMMAND UPDATED", current_command)
            way_point(connection, target_x, target_y, target_z)

        except Exception as e:
            print(f"Error executing waypoint navigation: {e}")
    elif command == "RTH":
        print("Executing RTH (Return to Home)")
        current_command = "ABSOLUTE_WAYPOINT 0 0 -10"
        execute_command("ABSOLUTE_WAYPOINT 0 0 -10")
    elif command == "STANDBY":
        print("Entering STANDBY (Hover in place)")
        connection.mav.set_position_target_local_ned_send(
            0,
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            int(0b110111111000),
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
    elif command == "PRECISION_LAND":
        print("Executing Precision Landing via external script")
        from precision_landing import precision_landing
        precision_landing(connection)

    elif command == "AVOID":
        print("AVOIDNG")
        from avoid import avoid_obstacle
        avoid_obstacle(connection)

        time.sleep(12)

    elif command == "STOP":
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,
            17, # Mode 17 (brake)
            0, 0, 0, 0, 0
        )

        print("STOPPED")
        time.sleep(10)

        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,
            4, # Switch to mode 4 (guided)
            0, 0, 0, 0, 0
        )

        time.sleep(1)
    
    elif command == "RESUME":
        print("TESTING", current_command)
        print(type(current_command))
        execute_command(current_command)

# Function to handle incoming messages
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
        except Exception as e:
            print(f"Error receiving message: {e}")
            break
    print("Disconnected from server")

# Main function
def main():
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect((HUB_IP, PORT))
        print(f"Connected to server at {HUB_IP}:{PORT}")
        
        # Start receiving messages in a separate thread
        receive_thread = threading.Thread(target=receive_messages, args=(client,))
        receive_thread.daemon = True
        receive_thread.start()
        
        # Start sending GPS coordinates in a separate thread
        gps_thread = threading.Thread(target=send_gps_coordinates, args=(client,))
        gps_thread.daemon = True
        gps_thread.start()
        
        # Continue main loop
        while True:
            try:
                client.send("Drone status: OK".encode())
                time.sleep(20)
            except:
                break
    
    except Exception as e:
        print(f"Connection error: {e}")
    finally:
        client.close()

if __name__ == "__main__":
    main()
