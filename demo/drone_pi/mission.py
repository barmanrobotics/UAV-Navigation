# node_pi_client.py
import socket
from pymavlink import mavutil # type: ignore
import time


# MAVLink Connection to PX4
connection = mavutil.mavlink_connection('udpin:localhost:14550')
connection.wait_heartbeat()

# Client Configuration
HUB_IP = 'hub_ip_address_here'
PORT = 5000

# Function to execute MAVLink commands or call external scripts
def execute_command(command):
    if command == "TAKEOFF":
        print("Executing TAKEOFF")
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

# Main client loop
def main():
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((HUB_IP, PORT))
    print("Connected to Hub Pi")

    try:
        while True:
            command = client.recv(1024).decode()
            print(f"Received command: {command}")
            execute_command(command)
            client.sendall("Command executed".encode())
    except Exception as e:
        print(f"Connection error: {e}")
    finally:
        client.close()
        print("Disconnected from Hub Pi")

if __name__ == "__main__":
    main()
