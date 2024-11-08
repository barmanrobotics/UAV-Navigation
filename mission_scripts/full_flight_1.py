# Owen Bartlett
# This is a script designed to mimic a complete flight.
# It should take off, fly to a predetermined position, stabililze, lower in altitude,
# then return back to the original gps location. 

from pymavlink import mavutil # type: ignore
import time
import math

# Define the distance to fly (in meters) and the direction (angle from north in degrees)
# Angle is in degrees: 0 = North, 90 = East, 180 = South, 270 = West
target_distance = float(input("Enter target distance in meters: "))
target_angle = float(input("Enter target angle from north in degrees (0 = North, 90 = East): "))
target_alt = float(input("Enter target altitude (meters): "))


# Start a connection listening to a UDP port
connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")

def set_mode(mode):
    mode_id = connection.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Mode {mode} is not supported")
        return False
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    while True:
        ack = connection.recv_match(type='HEARTBEAT', blocking=True)
        if ack.custom_mode == mode_id:
            print(f"Mode changed to {mode}")
            break
        time.sleep(0.5)

# Arm the drone
def arm_drone():
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Arming...")
    # Wait until armed
    while True:
        heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True)
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Drone is armed")
            break
        time.sleep(0.5)

# Take off to a specified altitude
def takeoff(altitude):
    print(f"Taking off to {altitude} meters...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude
    )
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0
        print(f"Current altitude: {current_alt:.2f} meters")
        if current_alt >= altitude * 0.95:  # Reached 95% of target altitude
            print("Reached target altitude")
            break
        time.sleep(0.5)

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


# Function to calculate the distance between two GPS points (Haversine formula)
def get_distance_meters(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c



def get_location():
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    
    # Extract latitude, longitude, and altitude from the message
    if msg:
        latitude = msg.lat / 1e7  # Convert from 1e7 to degrees
        longitude = msg.lon / 1e7  # Convert from 1e7 to degrees
        return latitude, longitude
    else:
        return None, None
    

def return_to_launch():
    """
    Command the drone to return to its launch (takeoff) location.
    """
    print("Returning to launch (RTL)...")
    
    # Send MAVLink RTL command (MAV_CMD_NAV_RETURN_TO_LAUNCH)
    connection.mav.command_long_send(
        connection.target_system, 
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  # RTL command
        0,  # Confirmation (set to 0)
        0,  # Param 1: Not used
        0,  # Param 2: Not used
        0,  # Param 3: Not used
        0,  # Param 4: Reserved
        0,  # Param 5: Reserved
        0,  # Param 6: Reserved
        0   # Param 7: Reserved
    )


# Store takeoff position
lat_takeoff, lon_takeoff = None, None

# Execute the mission
set_mode("GUIDED")
arm_drone()
takeoff(target_alt)

# Now set the takeoff location
lat_takeoff, lon_takeoff = get_location()

# Now fly to the target relative location
fly_relative_distance(target_distance, target_angle, target_alt)

# Begin loiter, allow for specification of time and altitude
loiter_time = float(input("Enter loiter time (s): "))
time.sleep(loiter_time)

#return to launch and end mission
return_to_launch()
