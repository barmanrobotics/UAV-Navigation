import openmeteo_requests
import requests_cache
import pandas as pd
from retry_requests import retry
from pymavlink import mavutil  # type: ignore
import time
import math

# Function to get UAV location (lat, lon) from MAVLink
def get_location():
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    
    # Extract latitude, longitude, and altitude from the message
    if msg:
        latitude = msg.lat / 1e7  # Convert from 1e7 to degrees
        longitude = msg.lon / 1e7  # Convert from 1e7 to degrees
        return latitude, longitude
    else:
        return None, None

# Function to check if the weather conditions are safe for flight
def check_weather(lat_takeoff, lon_takeoff):
        # Setup the Open-Meteo API client with cache and retry on error
    cache_session = requests_cache.CachedSession('.cache', expire_after = 3600)
    retry_session = retry(cache_session, retries = 5, backoff_factor = 0.2)
    openmeteo = openmeteo_requests.Client(session = retry_session)

    # Make sure all required weather variables are listed here
    # The order of variables in hourly or daily is important to assign them correctly below
    url = "https://api.open-meteo.com/v1/forecast"
    params = {
        "latitude": lat_takeoff,
        "longitude": lon_takeoff,
        "forecast_days": 1,
        "hourly": {"visibility", "wind_speed_10m", "precipitation",}
    }
    responses = openmeteo.weather_api(url, params=params)

    # Process first location. Add a for-loop for multiple locations or weather models
    response = responses[0]
    print(f"Coordinates {response.Latitude()}°N {response.Longitude()}°E")
    print(f"Elevation {response.Elevation()} m asl")


    # Process hourly data. The order of variables needs to be the same as requested.
    hourly = response.Hourly()
    hourly_visibility = hourly.Variables(0).ValuesAsNumpy()
    hourly_wind_speed_10 = hourly.Variables(1).ValuesAsNumpy()
    hourly_precipitation = hourly.Variables(2).ValuesAsNumpy()
    print(hourly_precipitation)
    print(hourly_visibility)
    print(hourly_wind_speed_10)

    # order data to be printed
    hourly_data = {}
    hourly_data["wind speed 10m"] = hourly_wind_speed_10
    hourly_data["precipitation"] = hourly_precipitation
    hourly_data["visibility"] = hourly_visibility

    # display data
    hourly_dataframe = pd.DataFrame(data = hourly_data)
    print(hourly_dataframe)

    # calculate average (this is incredibly innacurate but should demonstrate the concept for now)
    avg_wind_speed = hourly_wind_speed_10.mean()
    avg_precipitation = hourly_precipitation.mean()
    avg_visibility = hourly_visibility.mean()
    
    # arbitrary parameters that we want to compare to
    MAX_WIND_SPEED = 15
    MAX_PRECIPITATION = 0.1
    MIN_VISIBILITY = 500

    # compare to parameters and fail if criteria is met
    if avg_wind_speed > MAX_WIND_SPEED:
        print("Wind speed is too high for safe flight. Shutting down UAV.")
        return False
    if avg_precipitation > MAX_PRECIPITATION:
        print("Precipitation detected. Shutting down UAV.")
        return False
    if avg_visibility < MIN_VISIBILITY:
        print("Visibility is too low for safe flight. Shutting down UAV.")
        return False
    
    return True

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


# Function to return to launch (RTL)
def return_to_launch():
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


connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")

lat_takeoff, lon_takeoff = get_location()

# Check if the weather is safe for flight
if check_weather(39.32, -76.62):
    # Execute the mission
    set_mode("GUIDED")
    arm_drone()
    takeoff(10)
    return_to_launch()
else:
    print("Weather conditions are not safe for flight. Shutting down UAV.")
