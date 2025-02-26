#Owen Bartlett
import socket
import threading
import os

# Hub Pi Server Configuration
HOST = '0.0.0.0'  # Listen on all available network interfaces
PORT = 8000      # Port number for the server

# Store connections with corresponding labels
connections = {}
label_counter = 1  # Start labeling from 1

def handle_client(conn, label):
    """Handle communication with drone {label}"""
    try:
        print(f"Connection established with Drone {label}")
        while True:
            message = conn.recv(1024).decode()
            if not message:
                break
            print(f"Drone {label}: {message}")
    except ConnectionResetError:
        print(f"Connection lost with Drone {label}")
    finally:
        del connections[label]  # Remove connection when done
        conn.close()

def start_server():
    global label_counter, server_running
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print("Hub Pi Server listening for connections...")

        while server_running:  # Add check for server_running
            try:
                conn, addr = s.accept()
                # Label the connection
                label = str(label_counter)
                label_counter += 1
                connections[label] = conn
                print(f"Drone {label} connected from {addr}")
                client_thread = threading.Thread(target=handle_client, args=(conn, label))
                client_thread.start()
            except:
                break  # Exit loop if socket is closed

def send_command():
    global server_running
    while server_running:  # Add check for server_running
        input_data = input("Enter command (<label> <command> <params>): <command> = (TAKEOFF, WAYPOINT, RTH, STANDBY): ").strip().upper()
        input_split = input_data.split()

        if len(input_split) < 2 and ''.join(input_split[0]) != "END":
            print("Invalid input. Format: <label> <command> <params>")
            continue
        elif ''.join(input_split[0]) == "END":
            print("Shutting down server...")
            server_running = False  # Signal threads to stop
            # Close all connections
            for label, conn in connections.items():
                try:
                    conn.close()
                except:
                    pass
            os._exit(0)  # Force exit all threads

        target_label = input_split[0]
        command = ' '.join(input_split[1:])
        if command not in ["TAKEOFF", "WAYPOINT", "RTH", "STANDBY", "END"]:
            print("Invalid command.")
            continue
        

        if target_label in connections:
            connections[target_label].sendall(command.encode())
        else:
            print(f"No connection with Drone {target_label}")

if __name__ == "__main__":
    import os
    server_running = True  # Add global control flag
    server_thread = threading.Thread(target=start_server)
    server_thread.start()
    send_command()
