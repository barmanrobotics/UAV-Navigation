import cv2
import numpy as np
import time
from simple_pid import PID
from pymavlink import mavutil

# Establish MAVLink connection
def connect_mavlink(connection_string="udp:127.0.0.1:14550"):
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("MAVLink connection established.")
    return master

# Send velocity commands to the drone
def send_velocity_command(master, vx, vy, vz):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,  # Position
        vx, vy, vz,  # Velocity
        0, 0, 0,  # Acceleration
        0, 0  # Yaw, yaw rate
    )

# ArUco tag tracking parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

# PID Controllers with optimized values
pid_x = PID(0.8, 0.02, 0.1, setpoint=0, output_limits=(-1.0, 1.0))
pid_y = PID(0.8, 0.02, 0.1, setpoint=0, output_limits=(-1.0, 1.0))

# Open camera
cap = cv2.VideoCapture(0)

# Connect to the drone
master = connect_mavlink()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    if ids is not None:
        for i in range(len(ids)):
            corner = corners[i][0]
            center_x = int((corner[0][0] + corner[2][0]) / 2)
            center_y = int((corner[0][1] + corner[2][1]) / 2)
            frame_height, frame_width, _ = frame.shape
            
            error_x = (center_x - frame_width / 2) / (frame_width / 2)
            error_y = (center_y - frame_height / 2) / (frame_height / 2)
            
            vx = -pid_y(error_y)
            vy = -pid_x(error_x)
            vz = -0.2  # Constant descent rate
            
            send_velocity_command(master, vx, vy, vz)
            
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
    
    cv2.imshow("Frame", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
