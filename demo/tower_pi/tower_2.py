import socket
import threading
import math
import os
import time

HOST = '0.0.0.0'  # Listen on all available network interfaces
PORTS = [14550, 14560]

gps_data = {}
connections = {}
label_counter = 1  # Start labeling from 1
server_running = True
avoidance_enabled = 2
stage_started = time.time()

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
    global avoidance_enabled, stage_started
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
                                
                                horizontal_radius = 7
                                vertical_radius = 2
                                
                                if (distance < horizontal_radius) and (alt_diff < vertical_radius):
                                    if avoidance_enabled==0:
                                        print("avoidance_enabled_0")
                                        connections[d1].sendall("STOP".encode())
                                        connections[d2].sendall("STOP".encode())
                                        avoidance_enabled = 1
                                        stage_started = time.time()
                                    elif avoidance_enabled==1 and (time.time() - stage_started)>=10:
                                        print("avoidance_enabled_1")
                                        connections[d2].sendall("AVOID".encode())
                                        avoidance_enabled = 2
                                        stage_started = time.time()
                                elif (distance > horizontal_radius or alt_diff > vertical_radius) and avoidance_enabled==2 and (time.time() - stage_started)>=12:
                                    print("avoidance_enabled_2")
                                    connections[d1].send("RESUME".encode())
                                    connections[d2].send("RESUME".encode())
                                    avoidance_enabled = 3
                                    stage_started = time.time()
                                elif (distance > horizontal_radius or alt_diff > vertical_radius) and avoidance_enabled == 3 and (time.time()-stage_started)>=15:
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
            if command not in ["TAKEOFF", "RTH", "STANDBY", "RESUME"] and not command.startswith("WAYPOINT") and not command.startswith("ABSOLUTE_WAYPOINT"):
                print("Invalid command.")
                print("Available commands: TAKEOFF, WAYPOINT <x> <y> <z>, RTH, STANDBY")
                continue
            
            # Special handling for WAYPOINT command
            if command.startswith("WAYPOINT") or command.startswith("ABSOLUTE_WAYPOINT"):
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

if __name__ == "__main__":
    for port in PORTS:
        server_thread = threading.Thread(target=start_server, args=(port,))
        server_thread.start()
    send_command()
