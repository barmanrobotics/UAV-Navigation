# Owen Bartlett
# This fly to function uses the GLOBAL_POSITION_INT or coordinate system to determine where
# a drone is meant to go
# On the same page, the parameters it takes are also coordinates
import pymavlink as mavutil
import time
import math

connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()

def fly_to_location(lat, lon, alt):
    print(f"Flying to target location: Latitude={lat}, Longitude={lon}, Altitude={alt}")
    connection.mav.send(
        connection.mav.mission_item_int_encode(
            connection.target_system,
            connection.target_component,
            0,  # sequence
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2,  # current/auto continue
            1,  # autocontinue
            0, 0, 0, 0,
            int(lat * 1e7),  # Latitude in 1e7 degrees
            int(lon * 1e7),  # Longitude in 1e7 degrees
            alt  # Altitude in meters
        )
    )

    # Monitor distance to target location
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        current_alt = msg.relative_alt / 1000.0
        print(f"Current Position: Latitude={current_lat}, Longitude={current_lon}, Altitude={current_alt}")
        
        # Check if within 1 meter of target location
        distance_to_target = get_distance_meters(current_lat, current_lon, lat, lon)
        if distance_to_target <= 1.0 and abs(current_alt - alt) < 1.0:
            print("Reached target location.")
            break
        time.sleep(1)