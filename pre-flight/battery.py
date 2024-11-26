from pymavlink import mavutil
import time

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