from pymavlink import mavutil 
import time
import cv2 as cv
import OF_decision as of
import socket
import struct
import numpy as np

the_connection = mavutil.mavlink_connection('udpin:localhost:14541')

# Wait for the first heartbeat
#  This sets the system and component ID of remot system for the link
the_connection.wait_heartbeat()
print("Heartbeat for system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

#MOVE

def send_velocity_command(velocity_x, velocity_y, velocity_z, yaw):
    """
    Sends a velocity command for a specified duration.
    
    :param velocity_x: Velocity in the North direction (m/s)
    :param velocity_y: Velocity in the East direction (m/s)
    :param velocity_z: Velocity in the Down direction (m/s, positive down)
    :param duration: Duration to maintain the velocity (in seconds)
    """
    # the_connection.mav.set_position_target_local_ned_send(
    #         0,  # time_boot_ms (not used)
    #         the_connection.target_system,  # target_system
    #         the_connection.target_component,  # target_component
    #         mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
    #         int(0b100111111000),  # type_mask (only velocity control enabled)
    #         0, 0, 0,  # x, y, z positions (not used)
    #         velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
    #         0, 0, 0,  # x, y, z acceleration (not used)
    #         yaw, 0  # yaw, yaw_rate (not used)
    #     )
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, the_connection.target_system,the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
            int(0b10111000111), 0, 0, 0, velocity_x, velocity_y, velocity_z, 0, 0, 0, yaw, 0))
    time.sleep(0.1)

def OF_process(prev_frame, current_frame):
    if prev_frame.ndim == 3:  # Check if the image has 3 channels
    	prvs_gray = cv.cvtColor(prev_frame, cv.COLOR_BGR2GRAY)
    else:
    	prvs_gray = prev_frame
    if current_frame.ndim == 3:  # Check if the image has 3 channels
    	next_gray = cv.cvtColor(current_frame, cv.COLOR_BGR2GRAY)
    else:
    	next_gray = current_frame    	
    	
    direction = of.OF_cal(prvs_gray, next_gray)
    
    return direction

def dec2vel(decision):
    if decision == 'Move forward':
        send_velocity_command(1.0, 0, 0, 0) 
    elif decision == 'Move backward':
        send_velocity_command(-1.0, 0, 0, 3.14) 
    elif decision == 'Move right':
        send_velocity_command(0, 1.0, 0, 1.57) 
    elif decision == 'Move left':
        send_velocity_command(0, -1.0, 0, -1.57)
    elif decision == 'stop':
        send_velocity_command(0, 0, 0, 5)

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

    if prev_frame is None:
        prev_frame = current_frame
        continue
    
    direction = OF_process(prev_frame, current_frame)
    decision = of.choose_direction(direction)

    dec2vel(decision)

    prev_frame = current_frame

    time.sleep(0.1)
