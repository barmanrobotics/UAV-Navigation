from pymavlink import mavutil # type: ignore
import time

def rth(connection):
    # Request Home Position
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0
    )
    
    # Wait for the home position message
    msg = None
    while msg is None:
        msg = connection.recv_match(type='HOME_POSITION', blocking=True)
        home_lat = msg.latitude / 1e7
        home_lon = msg.longitude / 1e7
        home_alt = msg.altitude / 1000.0  # Convert from mm to meters

    print(f"Home Coordinates: Lat={home_lat}, Lon={home_lon}, Alt={home_alt}")

    # Set to GUIDED mode for precise navigation
    connection.set_mode_guided()
    time.sleep(1)

    # Fly to home location and hover at 7m above home altitude
    target_alt = home_alt + 7  # Hover 7m above the home point

    print(f"Returning to Home and hovering at {target_alt}m")

    connection.mav.set_position_target_global_int_send(
        0,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        int(0b110111111000),
        int(home_lat * 1e7),  # Latitude (scaled)
        int(home_lon * 1e7),  # Longitude (scaled)
        target_alt,
        0, 0, 0,  # Velocity
        0, 0, 0,  # Acceleration
        0, 0
    )
    time.sleep(1)
    