# Owen Bartlett
# This function sets the drone to loiter for a certain amount of turns at a specific altitude
## MAY BE FAULTY IMPLEMENTATION ##
import pymavlink as mavutil
import time
import math

connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()


def loiter_for_time(duration, altitude):
    """
    Make the drone loiter at the current location for the specified duration (in seconds).

    :param duration: Time to loiter in seconds
    :param altitude: Altitude to maintain during loitering (in meters)
    """
    print(f"Loitering for {duration} seconds at altitude {altitude} meters.")

    # Send MAVLink loiter command (MAV_CMD_NAV_LOITER_TIME)
    # Loiter command is to hold at the current position for the specified time (time in seconds)
    connection.mav.command_long_send(
        connection.target_system, 
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,  # Loiter command
        0,  # Confirmation (set to 0)
        0,  # Param 1: Latitude (0 for loiter at current position)
        0,  # Param 2: Longitude (0 for loiter at current position)
        altitude,  # Param 3: Altitude (altitude to loiter at)
        0,  # Param 4: Time in seconds (we'll use duration below)
        0,  # Param 5: Reserved (set to 0)
        0,  # Param 6: Reserved (set to 0)
        0   # Param 7: Reserved (set to 0)
    )

    # Wait for the drone to complete the loitering duration
    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        if elapsed_time >= duration:
            print("Loitering complete.")
            break
        time.sleep(1)