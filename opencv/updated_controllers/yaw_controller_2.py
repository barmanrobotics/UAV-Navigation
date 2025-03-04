import time
import math
from pymavlink import mavutil
import numpy as np

connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")

def set_yaw_target(connection, yaw_angle):
    """
    Set the yaw of a quadcopter using MAVLink 2 SET_ATTITUDE_TARGET.
    
    :param connection: MAVLink connection object
    :param yaw_angle: Desired yaw angle in degrees
    """
    # Convert yaw to radians
    yaw_rad = math.radians(yaw_angle)

    # Create a quaternion from yaw (assuming no roll/pitch change)
    q = [1, 0, 0, 0]

    q = [1, 0, 0, 0]  # Identity quaternion for no rotation (you will override yaw)

    # Build quaternion to set only the yaw angle
    q[0] = np.cos(yaw_angle / 2)  # w
    q[1] = 0  # x
    q[2] = 0  # y
    q[3] = np.sin(yaw_angle / 2)  # z

    # Send SET_ATTITUDE_TARGET message
    connection.mav.set_attitude_target_send(
        0,  # Timestamp (microseconds)
        connection.target_system,  # Target system ID
        connection.target_component,  # Target component ID
        0b00000111,  # Bitmask (ignore roll/pitch, only control yaw)
        q,  # Quaternion [x, y, z, w]
        0,  # Body roll rate
        0,  # Body pitch rate
        0,  # Body yaw rate
        0.5,  # thrust
    )
    print(f"Yaw set to {yaw_angle} degrees")


def get_yaw(master):
    """
    Get the yaw angle of a quadcopter using MAVLink.
    
    :param master: MAVLink connection object
    :return: Yaw angle in degrees
    """
    while True:
        # Receive attitude message
        msg = master.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            yaw = msg.yaw * (180 / 3.141592653589793)  # Convert radians to degrees
            return yaw
        
# Set desired yaw angle
current_yaw = get_yaw(connection)
desired_yaw = 180
error = abs(desired_yaw-current_yaw)
print(error)
while error>1:
    set_yaw_target(connection, 30)  # Set yaw to 90 degrees
    current_yaw = get_yaw(connection)
    print(current_yaw)
    desired_yaw = 180
    error = abs(desired_yaw-current_yaw)
