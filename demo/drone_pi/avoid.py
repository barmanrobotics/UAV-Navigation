import time
from pymavlink import mavutil  # type: ignore

def avoid_obstacle(connection):
    print("Avoid function triggered: Increasing altitude and moving forward")

    # Get current altitude
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if not msg:
        print("Failed to retrieve altitude data")
        return

    delta_alt = 5

    current_alt = msg.alt / 1000  # Convert from mm to meters
    target_alt = current_alt + delta_alt  # Increase altitude by 5 meters

    print(f"Current Altitude: {current_alt}m, Target Altitude: {target_alt}m")
    
    connection.mav.set_position_target_local_ned_send(
        0,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
        0b110111111000,
        0, 0, -delta_alt, # Gain altitude
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

    # Wait until the altitude has increased
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg and (msg.alt / 1000) >= target_alt - 0.5: # Allow small tolerance
            print(f"Altitude reached: {msg.alt / 1000}m, moving forward")
            break
        time.sleep(1)

    print("Avoidance maneuver complete")
