# Owen Bartlett
# This fly to function uses NED coordinates (North East Down) as apposed to global positioning
# Distance and altitude are in meters where angle is in degrees where 0 is North and 90 is East, etc
# lat_takeoff, lon_takeoff are defined at takeoff using the get_location() function
import pymavlink as mavutil
import time
import math

connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()




# Fly a relative distance from the takeoff point (using NED coordinates)
def fly_relative_distance(distance, angle, alt):
    # Convert angle from degrees to radians
    angle_rad = math.radians(angle)
    
    # Calculate the change in North (N) and East (E) directions based on the angle and distance
    delta_north = distance * math.cos(angle_rad)  # NED north is positive north, negative south
    delta_east = distance * math.sin(angle_rad)   # NED east is positive east, negative west
    delta_down = -alt  # We want to fly at the given altitude, hence negative for downward motion
    
    print(f"Flying {distance} meters in the direction of {angle}° (North/East).")
    
    # Send the relative movement command (in NED coordinates)
    connection.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms, set to 0 as we are sending an instantaneous command
        connection.target_system, 
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # NED frame
        int(0b110111111000),  # type_mask to specify that we are setting position
        delta_north,  # North (meters)
        delta_east,   # East (meters)
        delta_down,   # Down (meters)
        0,            # velocity_x (not used)
        0,            # velocity_y (not used)
        0,            # velocity_z (not used)
        0,            # acceleration_x (not used)
        0,            # acceleration_y (not used)
        0,            # acceleration_z (not used)
        0,            # yaw (not used)
        0             # yaw_rate (not used)
    )
    
    # Monitor progress (waiting for the drone to complete the movement)
    start_time = time.time()
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        current_alt = msg.relative_alt / 1000.0
        print(f"Current Position: Latitude={current_lat}, Longitude={current_lon}, Altitude={current_alt}")
        
        # Check if we have moved the desired distance
        distance_travelled = get_distance_meters(current_lat, current_lon, lat_takeoff, lon_takeoff)
        if distance_travelled >= distance - 1:  # Check if we've moved the target distance
            print("Target distance reached.")
            break
        
        # Timeout after 30 seconds in case the drone doesn't reach the target
        if time.time() - start_time > 30:
            print("Timed out while trying to reach target distance.")
            break
        
        time.sleep(1)