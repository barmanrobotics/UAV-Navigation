from pymavlink import mavutil
import time

def connect_drone(connection_string):
    """Establish connection to the drone."""
    try:
        drone = mavutil.mavlink_connection(connection_string)
        drone.wait_heartbeat(timeout=30)
        print("Heartbeat received. Drone connected.")
        return drone
    except Exception as e:
        print(f"Connection failed: {e}")
        return None