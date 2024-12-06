from pymavlink import mavutil 
import time
import cv2 as cv
import OF_decision6 as of
import socket
import struct
import numpy as np


the_connection = mavutil.mavlink_connection('udpin:localhost:14541')

# Wait for the first heartbeat
#  This sets the system and component ID of remot system for the link
the_connection.wait_heartbeat()
print("Heartbeat for system (system %u component %u )" % (the_connection.target_system, the_connection.target_component))

#MOVE
def get_current_acceleration():
    """
    Retrieves the current acceleration (in m/s²) from the IMU data.
    """
    try:
        # Request the HIGHRES_IMU message
        msg = the_connection.recv_match(type='HIGHRES_IMU', blocking=True)
        if msg:
            accel_x = msg.xacc  # Acceleration in the X-axis (m/s²)
            accel_y = msg.yacc  # Acceleration in the Y-axis (m/s²)
            accel_z = msg.zacc  # Acceleration in the Z-axis (m/s²)
            return accel_x, accel_y, accel_z
    except Exception as e:
        print(f"Error while reading IMU data: {e}")
    return None, None, None


def send_accel_command(accel_x, accel_y, accel_z, yaw, duration=0.1):
    start_time = time.time()
    while time.time() - start_time < duration:
        the_connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10, the_connection.target_system, the_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
                int(0b10111000111), 
                0, 0, 0,  # Position (ignored)
                accel_x, accel_y, accel_z,  # Velocity (ignored)
                0, 0, 0,  # Acceleration
                yaw, 0   # Yaw and yaw rate
            )
        )
        time.sleep(0.1)  # Send at 10 Hz

prev_mid_flow = 0

def dec2vel(decision, mid_flow, time_step=1):
    global prev_mid_flow  # Access the global variable for the previous mid_flow

    # PD control gains
    Kp = 0.01
    Kd = 0.01

    # Calculate the derivative term
    mid_flow_d = (mid_flow - prev_mid_flow) / time_step

    # PD control: Calculate acceleration components
    accel_x = 0.1 - Kp * mid_flow - Kd * mid_flow_d
    accel_z = 0  # Assuming no vertical acceleration for simplicity

    # Define acceleration commands based on decision
    if decision == 'Move forward':
        send_accel_command(0, -0.1, accel_z, 0)
    elif decision == 'Move right':
        send_accel_command(0.1, 0, accel_z, 1.57)
    elif decision == 'Move left':
        send_accel_command(-0.1, 0, accel_z, -1.57)
    elif decision == 'Stop':
        send_accel_command(0, 0, 0, 0)
    else:
        print("Invalid decision. No command sent.")

    # Update the previous mid_flow for the next call
    previous_mid_flow = mid_flow

# connect to WebotsArduVehicle
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5599))

header_size = struct.calcsize("=HH")
prev_frame = None

while True:
    # receive header
    header = s.recv(header_size)
    if len(header) != header_size:
        print("Header size mismatch")
        break

    # parse header
    width, height = struct.unpack("=HH", header)

    # receive image
    bytes_to_read = width * height
    img = bytes()
    while len(img) < bytes_to_read:
        img += s.recv(min(bytes_to_read - len(img), 4096))

    # convert incoming bytes to a numpy array (a grayscale image)
    current_frame = np.frombuffer(img, np.uint8).reshape((height, width))
    decision, mid_flow = of.process_frame(current_frame)
    print(decision, mid_flow)
    dec2vel(decision, mid_flow)
    
    # Check current acceleration
    #accel_x, accel_y, accel_z = get_current_acceleration()
    #if accel_x is not None:
        #print(f"Current acceleration - X: {accel_x:.2f}, Y: {accel_y:.2f}, Z: {accel_z:.2f}")

    time.sleep(0.1)
