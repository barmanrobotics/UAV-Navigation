#!/usr/bin/env python3
"""
Example usage of the Drone and Tower classes

This script demonstrates how to use the well-structured classes
for UAV navigation and coordination.
"""

import time
import threading
from classes import Drone, Tower


def drone_example():
    """Example of using the Drone class."""
    print("=== Drone Example ===")
    
    # Create drone instance
    drone = Drone(
        mavlink_port=14551,
        hub_ip='127.0.0.1',  # Use localhost for testing
        hub_port=14551,
        use_pi_camera=False  # Use regular camera for testing
    )
    
    try:
        # Start the drone system
        if drone.start():
            print("Drone system started successfully")
            
            # Wait a moment for connections to stabilize
            time.sleep(2)
            
            # Check if connected to MAVLink
            if drone.connection:
                print("MAVLink connection established")
                
                # Get current flight mode
                mode = drone.get_flight_mode()
                print(f"Current flight mode: {mode}")
                
                # Example flight operations (commented out for safety)
                # drone.takeoff(3.0)
                # time.sleep(5)
                # drone.waypoint(10, 10, 5)
                # time.sleep(10)
                # drone.land()
                
                print("Flight operations completed")
            else:
                print("Failed to establish MAVLink connection")
        else:
            print("Failed to start drone system")
            
    except Exception as e:
        print(f"Error in drone example: {e}")
    finally:
        # Stop the drone system
        drone.stop()
        print("Drone system stopped")


def tower_example():
    """Example of using the Tower class."""
    print("\n=== Tower Example ===")
    
    # Create tower instance
    tower = Tower(
        host='127.0.0.1',  # Use localhost for testing
        ports=[14551, 14552],
        server_port=6542,
        comm_port=6553,
        debug_gps=True,
        debug_avoidance=True
    )
    
    try:
        # Start the tower server
        if tower.start_server():
            print("Tower server started successfully")
            
            # Wait for potential drone connections
            print("Waiting for drone connections...")
            time.sleep(5)
            
            # Check connected drones
            drone_count = tower.get_drone_count()
            print(f"Connected drones: {drone_count}")
            
            if drone_count > 0:
                # Get drone data
                drone_data = tower.get_all_drone_data()
                print(f"Drone data: {drone_data}")
                
                # Example commands (commented out for safety)
                # tower.send_command_to_drone("1", "TAKEOFF 5")
                # time.sleep(5)
                # tower.send_command_to_drone("1", "WAYPOINT 10 10 5")
                
                print("Commands sent to drones")
            else:
                print("No drones connected")
            
            # Example flap control (commented out for safety)
            # tower.open_flaps()
            # time.sleep(2)
            # tower.close_flaps()
            
            print("Tower operations completed")
        else:
            print("Failed to start tower server")
            
    except Exception as e:
        print(f"Error in tower example: {e}")
    finally:
        # Stop the tower server
        tower.stop()
        print("Tower server stopped")


def multi_drone_example():
    """Example of coordinating multiple drones."""
    print("\n=== Multi-Drone Coordination Example ===")
    
    # Create tower
    tower = Tower(host='127.0.0.1')
    
    try:
        with tower:
            print("Tower started, waiting for drones...")
            time.sleep(3)
            
            # Monitor drone connections
            for i in range(10):  # Check for 10 seconds
                drone_count = tower.get_drone_count()
                print(f"Connected drones: {drone_count}")
                
                if drone_count >= 2:
                    print("Two or more drones connected!")
                    
                    # Get drone data
                    drone_data = tower.get_all_drone_data()
                    for drone_id, data in drone_data.items():
                        if len(data) >= 5:
                            lat, lon, alt, vel, hdg = data[:5]
                            print(f"Drone {drone_id}: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m")
                    
                    # Example coordinated mission (commented out for safety)
                    # print("Starting coordinated mission...")
                    # tower.send_command_to_drone("1", "TAKEOFF 5")
                    # tower.send_command_to_drone("2", "TAKEOFF 5")
                    # time.sleep(10)
                    # 
                    # tower.send_command_to_drone("1", "WAYPOINT 10 0 5")
                    # tower.send_command_to_drone("2", "WAYPOINT 0 10 5")
                    # time.sleep(15)
                    # 
                    # tower.send_command_to_drone("1", "RTH")
                    # tower.send_command_to_drone("2", "RTH")
                    
                    break
                
                time.sleep(1)
            else:
                print("No drones connected within timeout")
                
    except Exception as e:
        print(f"Error in multi-drone example: {e}")


def context_manager_example():
    """Example using context managers for automatic cleanup."""
    print("\n=== Context Manager Example ===")
    
    # Drone with context manager
    try:
        with Drone(hub_ip='127.0.0.1') as drone:
            print("Drone started with context manager")
            
            # Operations here
            time.sleep(2)
            
            print("Drone operations completed")
        # Automatically stops when exiting context
        print("Drone automatically stopped")
        
    except Exception as e:
        print(f"Error in drone context manager: {e}")
    
    # Tower with context manager
    try:
        with Tower(host='127.0.0.1') as tower:
            print("Tower started with context manager")
            
            # Operations here
            time.sleep(2)
            
            print("Tower operations completed")
        # Automatically stops when exiting context
        print("Tower automatically stopped")
        
    except Exception as e:
        print(f"Error in tower context manager: {e}")


def main():
    """Main function to run examples."""
    print("UAV Navigation Classes - Example Usage")
    print("=" * 50)
    
    # Run examples
    drone_example()
    tower_example()
    multi_drone_example()
    context_manager_example()
    
    print("\n" + "=" * 50)
    print("All examples completed!")


if __name__ == "__main__":
    main()
