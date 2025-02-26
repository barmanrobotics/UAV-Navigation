# node_pi_client.py
import socket
import time
import threading
import sys
import random

# Client Configuration
HUB_IP = 'localhost'

# Get the port number from command line argument
if len(sys.argv) < 2:
    print("Usage: python3 node_pi_client.py <PORT>")
    sys.exit(1)

PORT = int(sys.argv[1])  # Get the port from the command line argument

# Function to generate random GPS coordinates
def generate_gps_coordinates():
    base_lat = 37.7749  # Example: San Francisco latitude
    base_lon = -122.4194  # Example: San Francisco longitude
    lat_offset = random.uniform(-0.0005, 0.0005)
    lon_offset = random.uniform(-0.0005, 0.0005)
    return base_lat + lat_offset, base_lon + lon_offset

# Function to periodically send simulated GPS coordinates to the hub
def send_gps_coordinates(client):
    while True:
        try:
            lat, lon = generate_gps_coordinates()
            gps_data = f"GPS {lat} {lon} 10"  # Altitude fixed at 10 meters for testing
            print(f"Sending GPS Data: {gps_data}")
            client.sendall(gps_data.encode())
            time.sleep(3)  # Send every 3 seconds
        except Exception as e:
            print(f"Error sending GPS data: {e}")
            break

def receive_commands(client):
    while True:
        try:
            command = client.recv(1024).decode()
            if not command:
                print("Server disconnected")
                break
                
            # Only process actual commands, not acknowledgments
            if command in ["TAKEOFF", "WAYPOINT", "RTH", "STANDBY"]:
                print(f"Received command: {command}")
                print(f"Executing command: {command}")
                client.sendall(f"Command '{command}' executed successfully.".encode())
            # Ignore other messages from server
            
        except Exception as e:
            print(f"Error receiving command: {e}")
            break

def main():
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect((HUB_IP, PORT))
        print("Connected to Tower Pi")

        # Start both threads
        cmd_thread = threading.Thread(target=receive_commands, args=(client,))
        gps_thread = threading.Thread(target=send_gps_coordinates, args=(client,))
        
        cmd_thread.daemon = True
        gps_thread.daemon = True
        
        cmd_thread.start()
        gps_thread.start()

        # Keep main thread alive and monitor connections
        while cmd_thread.is_alive() and gps_thread.is_alive():
            time.sleep(1)
            
    except Exception as e:
        print(f"Connection error: {e}")
    finally:
        print("Disconnecting from Tower Pi...")
        client.close()

if __name__ == "__main__":
    main()