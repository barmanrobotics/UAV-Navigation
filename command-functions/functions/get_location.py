# Owen Bartlett
# This function gets the coordinate and altitude of the drone using GLOBAL_POSITION_INT

import pymavlink as mavutil
import time
import math

connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()


def get_location():
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    
    # Extract latitude, longitude, and altitude from the message
    if msg:
        latitude = msg.lat / 1e7  # Convert from 1e7 to degrees
        longitude = msg.lon / 1e7  # Convert from 1e7 to degrees
        altitude = msg.relative_alt / 1000.0  # Convert from mm to meters (relative altitude)
        return latitude, longitude, altitude
    else:
        return None, None, None