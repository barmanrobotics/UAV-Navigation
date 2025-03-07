import time
from pymavlink import mavutil  # type: ignore

def avoid_obstacle(connection):
    print("Avoid function triggered: Increasing altitude and moving forward")

    # Get current altitude
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if not msg:
        print("Failed to retrieve altitude data")
        return

    current_alt = msg.alt / 1000  # Convert from mm to meters
    target_alt = current_alt + 5  # Increase altitude by 5 meters

    print(f"Current Altitude: {current_alt}m, Target Altitude: {target_alt}m")

    # Set position target for altitude increase and forward movement
    connection.mav.set_position_target_global_int_send(
        0,  # Time_boot_ms (ignored)
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000),  # Bitmask: Ignore velocity and acceleration, use position
        0,  # Latitude (0 = unchanged)
        0,  # Longitude (0 = unchanged)
        target_alt,  # New altitude
        2.0,  # Move forward at 2 m/s
        0,  # No sideways movement
        0,  # No vertical velocity change
        0, 0, 0,  # No acceleration settings
        0,  # Yaw (unchanged)
        0   # Yaw rate (unchanged)
    )

    # Wait until the altitude has increased
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg and (msg.alt / 1000) >= target_alt - 0.5:  # Allow small tolerance
            print(f"Altitude reached: {msg.alt / 1000}m, moving forward")
            break
        time.sleep(1)

    print("Avoidance maneuver complete")
