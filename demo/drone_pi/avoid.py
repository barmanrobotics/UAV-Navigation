import time
from pymavlink import mavutil  # type: ignore

def avoid_obstacle(connection):
    print("Increasing altitude to avoid obstacle")
    
    # Increase altitude by 5 meters
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, 5  # Gain 5 meters altitude
    )
    
    # Wait for altitude gain
    time.sleep(3)
    print("Altitude increase complete")
