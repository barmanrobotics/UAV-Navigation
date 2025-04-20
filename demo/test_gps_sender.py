#!/usr/bin/env python3
"""
Test script to simulate a drone sending GPS data to the tower
Run this to verify the entire data flow from drone to tower to dashboard
"""

import socket
import time
import json
import random
import argparse
import threading

# Default configuration
TOWER_PORT = 14551
TOWER_IP = '127.0.0.1'

def generate_random_movement(start_lat, start_lon, radius=0.0001):
    """Generate a small random movement from the starting position"""
    lat = start_lat + (random.random() * 2 - 1) * radius
    lon = start_lon + (random.random() * 2 - 1) * radius
    alt = 100 + random.random() * 10
    return lat, lon, alt

def simulate_drone(drone_id, start_lat, start_lon):
    """Simulate a drone sending GPS data to the tower"""
    print(f"Starting simulated drone {drone_id} at {start_lat}, {start_lon}")
    
    current_lat, current_lon = start_lat, start_lon
    
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5)
                print(f"Drone {drone_id}: Connecting to tower at {TOWER_IP}:{TOWER_PORT}...")
                
                try:
                    s.connect((TOWER_IP, TOWER_PORT))
                    print(f"Drone {drone_id}: Connected to tower")
                    
                    # Send initial message
                    s.sendall(f"Drone status: OK - Drone {drone_id}".encode())
                    
                    # Keep sending GPS data
                    while True:
                        # Generate movement
                        current_lat, current_lon, altitude = generate_random_movement(current_lat, current_lon)
                        
                        # Format GPS message
                        gps_message = f"GPS {current_lat} {current_lon} {altitude:.1f}"
                        print(f"Drone {drone_id}: Sending {gps_message}")
                        
                        # Send to tower
                        s.sendall(gps_message.encode())
                        
                        # Wait for response
                        try:
                            response = s.recv(1024).decode()
                            if response:
                                print(f"Drone {drone_id}: Received command: {response}")
                                # If we received a command, send acknowledgment
                                s.sendall("Command executed".encode())
                        except socket.timeout:
                            # It's okay if we don't get a response
                            pass
                            
                        time.sleep(2)
                        
                except ConnectionRefusedError:
                    print(f"Drone {drone_id}: Connection refused. Tower not running.")
                except socket.timeout:
                    print(f"Drone {drone_id}: Connection timed out")
                except Exception as e:
                    print(f"Drone {drone_id}: Connection error: {e}")
                    
                # If we got here, the connection was lost
                print(f"Drone {drone_id}: Lost connection to tower, reconnecting in 5 seconds...")
                time.sleep(5)
                
        except KeyboardInterrupt:
            print(f"Drone {drone_id}: Simulation stopped")
            break
        except Exception as e:
            print(f"Drone {drone_id}: Error: {e}")
            time.sleep(5)

def main():
    parser = argparse.ArgumentParser(description="Simulate drones sending GPS data to tower")
    parser.add_argument('--drones', type=int, default=2, help='Number of simulated drones (default: 2)')
    parser.add_argument('--tower-ip', type=str, default='127.0.0.1', help='Tower IP address (default: 127.0.0.1)')
    parser.add_argument('--tower-port', type=int, default=14551, help='Tower port (default: 14551)')
    
    args = parser.parse_args()
    
    global TOWER_IP, TOWER_PORT
    TOWER_IP = args.tower_ip
    TOWER_PORT = args.tower_port
    
    print(f"Starting {args.drones} simulated drones connecting to tower at {TOWER_IP}:{TOWER_PORT}")
    
    # Starting positions for drones (slightly separated)
    base_lat = -35.3632
    base_lon = 149.1652
    
    # Create threads for each drone
    threads = []
    for i in range(1, args.drones + 1):
        # Calculate a slightly different starting position for each drone
        start_lat = base_lat + (i - 1) * 0.0001
        start_lon = base_lon + (i - 1) * 0.0001
        
        drone_thread = threading.Thread(
            target=simulate_drone, 
            args=(str(i), start_lat, start_lon),
            daemon=True
        )
        threads.append(drone_thread)
        drone_thread.start()
    
    # Wait for keyboard interrupt
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Simulation stopped")

if __name__ == "__main__":
    main() 