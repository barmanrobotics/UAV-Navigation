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

def check_gps(drone):
    """Check if GPS fix is available."""
    print("Checking GPS...")
    msg = drone.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
    if msg and msg.fix_type >= 3:  # 3D fix or better
        print(f"GPS fix acquired: {msg.fix_type}. Satellites visible: {msg.satellites_visible}")
        return True
    print("GPS fix not sufficient.")
    return False

def check_battery(drone):
    """Check battery voltage and remaining capacity."""
    print("Checking battery...")
    msg = drone.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
    if msg:
        voltage = msg.voltage_battery / 1000.0
        remaining = msg.battery_remaining
        print(f"Battery voltage: {voltage}V, remaining: {remaining}%")
        if remaining > 20:  # Example threshold
            return True
    print("Battery level insufficient.")
    return False


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

def preflight_checks(drone):
    """Perform all pre-flight checks."""
    print("Starting pre-flight checks...")
    if not check_gps(drone):
        print("Pre-flight check failed: GPS.")
        return False
    if not check_battery(drone):
        print("Pre-flight check failed: Battery.")
        return False
    if not check_armable(drone):
        print("Pre-flight check failed: Arming status.")
        return False
    print("All pre-flight checks passed.")
    return True

def main():
    connection_string = "udpin:localhost:14550"  # Adjust this based on your setup
    drone = connect_drone(connection_string)
    if not drone:
        return

    if preflight_checks(drone):
        print("Ready for takeoff!")
        # Add takeoff sequence here
    else:
        print("Pre-flight checks failed. Aborting.")

if __name__ == "__main__":
    main()
