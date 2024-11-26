from pymavlink import mavutil
import time

def check_gps(drone):
    """Check if GPS fix is available."""
    print("Checking GPS...")
    msg = drone.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
    if msg and msg.fix_type >= 3:  # 3D fix or better
        print(f"GPS fix acquired: {msg.fix_type}. Satellites visible: {msg.satellites_visible}")
        return True
    print("GPS fix not sufficient.")
    return False