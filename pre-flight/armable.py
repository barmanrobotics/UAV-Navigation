from pymavlink import mavutil
import time

def check_armable(drone):
    """Check if the drone is armable."""
    print("Checking arming status...")
    msg = drone.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED == 0:
            print("Drone is disarmed and safe to arm.")
            return True
    print("Drone is already armed or not ready to arm.")
    return False