import csv
import os
import cv2
import cv2.aruco as aruco
import socket
import struct
import numpy as np
import math
import time
from pymavlink import mavutil
import random

# connect to WebotsArduVehicle
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5599))

connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")


def get_unique_filename(base_name):
    #Stores the csv files with new names without overwriting
    counter = 1
    file_name = f"{base_name}.csv"
    while os.path.isfile(file_name):
        file_name = f"{base_name}_{counter}.csv"
        counter += 1
    return file_name

def save_marker_positions(marker_positions):
    #Stores the csv file
    for marker_id, positions in marker_positions.items():
        file_name = get_unique_filename(f"marker_{marker_id}")
        with open(file_name, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["X", "Y", "Z", "A_X", "A_Y", "V_Z"])  # Write header
            writer.writerows(positions)
        print(f"Saved positions for marker {marker_id} in {file_name}")


def set_message_interval(rate_hz):
    """
    Sets the update rate for a specific MAVLink message using MAV_CMD_SET_MESSAGE_INTERVAL.

    :param connection: MAVLink connection object
    :param message_id: MAVLink message ID (e.g., 30 for ATTITUDE)
    :param rate_hz: Desired frequency in Hz
    """
    interval_us = int(1e6 / rate_hz)  # Convert Hz to microseconds
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,  # Confirmation
        30,
        interval_us,
        0, 0, 0, 0, 0
    )


def get_drone_attitude():
    """
    Connects to the drone via MAVLink and retrieves attitude (roll, pitch, yaw).
    
    :param connection_string: MAVLink connection string (e.g., 'udp:127.0.0.1:14550' or 'COM3' for serial)
    :return: Tuple (roll_deg, pitch_deg, yaw_deg) in degrees
    """
    # Wait for an ATTITUDE message
    msg = connection.recv_match(type='ATTITUDE', blocking=True, timeout= 0.02)
    if msg:
        roll = msg.roll # Convert radians to degrees
        pitch = msg.pitch
        yaw = msg.yaw

        return roll, pitch, yaw
    return None

def arm_disarm_drone(value):                      #set value to 1 to arm, 0 to disarm
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        value, 0, 0, 0, 0, 0, 0
    )
    if value == 1:
        print("Arming... ")
        while True:
            heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True)
            if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Drone is armed")
                break
            time.sleep(0.5)
    else:
        print("Disarming ...")
        msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
    # Wait until armed

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
        time.sleep(0.1)

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


def send_yaw_command(master, target_yaw, yaw_speed, direction, relative):
    """
    Sends a MAV_CMD_CONDITION_YAW command to control the drone's yaw.

    :param master: MAVLink connection object
    :param target_yaw: Desired yaw angle (degrees)
    :param yaw_speed: Yaw rotation speed (degrees/sec)
    :param direction: 1 for clockwise, -1 for counterclockwise
    :param relative: 1 for relative yaw, 0 for absolute yaw
    """
    master.mav.command_long_send(
        master.target_system,    # Target system ID
        master.target_component, # Target component ID
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # Command ID
        0,  # Confirmation
        target_yaw,  # Target yaw angle in degrees
        yaw_speed,   # Yaw speed in degrees/sec
        direction,   # Direction: 1 = CW, -1 = CCW
        relative,    # Relative (1) or absolute (0)
        0, 0, 0)     # Unused parameters

def send_velocity(vx,vy,vz):
    # Send velocity command in the drone's NED frame
    connection.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # Frame of reference (Body frame)
        0b100111000111,  # Control velocity only
        0, 0, 0,  # Position x, y, z (not used)
        vx, vy, vz,  # Velocity x, y, z (used)
        0, 0, 0,  # Acceleration (not used)
        0.75, 0  # yaw, yaw_rate (not used)
    )

def send_velocity_yaw(yaw,vx,vy,vz):
    # Send velocity command in the drone's NED frame
    connection.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        1,  # Frame of reference (Body frame)
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Control velocity only
        0, 0, 0,  # Position x, y, z (not used)
        vx, vy, vz,  # Velocity x, y, z
        0, 0, 0,  # Acceleration (not used)
        yaw, 0  # yaw, yaw_rate (not used)
        )
    print('setting yaw')

def send_acceleration(ax, ay, vz):
    # Send velocity command in the drone's NED frame
    connection.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Frame of reference (Body frame)
        0b110000000000,  # Control velocity only
        0, 0, 0,  # Position x, y, z (not used)
        0, 0, vz,  # Velocity x, y, z
        ax, ay, 0,  # Acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )
    
def set_flight_mode(mode):
    """
    Change the flight mode of the vehicle.
    
    Args:
        mode (str): The desired flight mode (e.g., "LAND", "GUIDED", "STABILIZE").
    """
    # Ensure mode is available in the MAVLink vehicle modes
    mode_id = connection.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Unknown mode: {mode}")
        return

    # Send command to change flight mode
    connection.mav.command_long_send(
        connection.target_system,  # target system
        connection.target_component,  # target component
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # command
        0,  # confirmation
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        mode_id,  # mode to switch to
        0, 0, 0, 0, 0  # unused parameters
    )

    # Wait for the mode to change
    while True:
        # Receive a heartbeat message to check the current mode
        msg = connection.recv_match(type='HEARTBEAT', blocking=True)
        current_mode = mavutil.mode_string_v10(msg)
        if current_mode == mode:
            print(f"Mode changed to {mode}")
            break
        print(f"Waiting for mode change... (Current mode: {current_mode})")
        time.sleep(1)
   

def get_flight_mode():
    # Request the vehicle's heartbeat
    msg = connection.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        # Get the flight mode
        mode = mavutil.mode_string_v10(msg)
        print(f"Current flight mode: {mode}")
        return mode
    else:
        print("Failed to receive flight mode.")
        return None
    
def set_land_mode():
    """
    Function to set the vehicle to LAND mode using pymavlink.
    """
    # Get the mode ID for LAND mode
    mode_id = connection.mode_mapping().get("LAND")
    
    if mode_id is None:
        print("LAND mode not available")
        return

    # Send the command to set the mode to LAND
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Enable custom mode
        mode_id,  # Set to LAND mode ID
        0, 0, 0, 0, 0  # Unused parameters
    )
    print("LAND mode command sent")

def euler_to_quaternion(roll=0, pitch=0, yaw=0):
    """Convert Euler angles to a quaternion (w, x, y, z)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (w, x, y, z)

def send_yaw_command(master, target_yaw, yaw_speed, direction, relative):
    """
    Sends a MAV_CMD_CONDITION_YAW command to control the drone's yaw.

    :param master: MAVLink connection object
    :param target_yaw: Desired yaw angle (degrees)
    :param yaw_speed: Yaw rotation speed (degrees/sec)
    :param direction: 1 for clockwise, -1 for counterclockwise
    :param relative: 1 for relative yaw, 0 for absolute yaw
    """
    master.mav.command_long_send(
        master.target_system,    # Target system ID
        master.target_component, # Target component ID
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # Command ID
        0,  # Confirmation
        target_yaw,  # Target yaw angle in degrees
        yaw_speed,   # Yaw speed in degrees/sec
        direction,   # Direction: 1 = CW, -1 = CCW
        relative,    # Relative (1) or absolute (0)
        0, 0, 0)     # Unused parameters

def send_land_message_v1(x_rad=0, y_rad=0, dist_m=0, time_usec=0, target_num=0):
        connection.mav.landing_target_send(
        time_usec,          # time target data was processed, as close to sensor capture as possible
        target_num,          # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
        x_rad,          # X-axis angular offset, in radians
        y_rad,          # Y-axis angular offset, in radians
        dist_m,          # distance, in meters
        0,          # Target x-axis size, in radians
        0,          # Target y-axis size, in radians
    )

def send_land_message_v2(x_rad=0, y_rad=0, dist_m=0, x_m=0,y_m=0,z_m=0, time_usec=0, target_num=0,yaw_rad=1.54):
        
        # Convert yaw angle to quaternion
        q = euler_to_quaternion(roll=0, pitch=0, yaw=yaw_rad)

        connection.mav.landing_target_send(
        time_usec,          # time target data was processed, as close to sensor capture as possible
        target_num,          # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_FRD, # frame, not used
        x_rad,          # X-axis angular offset, in radians
        y_rad,          # Y-axis angular offset, in radians
        dist_m,          # distance, in meters
        0,          # Target x-axis size, in radians
        0,          # Target y-axis size, in radians
        x_m,          # x	float	X Position of the landing target on MAV_FRAME
        y_m,          # y	float	Y Position of the landing target on MAV_FRAME
        z_m,          # z	float	Z Position of the landing target on MAV_FRAME
        q,  # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        2,          # type of landing target: 2 = Fiducial marker
        1,          # position_valid boolean
    )


def precision_land_acc():
    # ArUco setup
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    marker_positions = {}

    #distance from the center of the tag along x or y axes where you'd like the center of the camera to land
    #if you want the center of the camera to land at the center of the tag, set it to 0.

    #marker_size = float(input("Please enter the marker size in cm: "))
    #offset_x = float(input("Please enter the x offset distance in cm. Back is positive: "))
    #offset_y = float(input("Please enter the y offset distance in cm. Right is positive: "))
    marker_size = 20
    offset_x = 0
    offset_y = 0
    #DESCENT_VELOCITY = float(input("Enter initial descent velocity: ")) 
    #ERROR_THRESHOLD = float(input("Enter threshold error: "))
    #marker_track = float(input("WHich marker ID you want to track? "))
    DESCENT_VELOCITY = 0.0
    ERROR_THRESHOLD = 0.0 # land within 1 cm of target
    marker_track = 0


    horizontal_fov = 66*(math.pi/180) #horizontal_fov in radians for normal raspi camera module 3
    vertical_fov = 41*(math.pi/180) #vertical_fov in radians for normal raspi camera module 3
    x_res= 640 # pixels
    y_res = 480 # pixels

    LANDING_THRESHOLD_ANGLE = 0.034 #2 degrees in radians

    camera_matrix = np.array([
    [917.1777059, 0.00000000, 323.46713801],
    [0.00000000, 926.27018107, 240.68578702],
    [0.00000000, 0.00000000, 1.00000000]
    ])

    dist_coefficients= np.array([
    [-4.21053311e-01],
    [6.10894559e+00],
    [1.08524770e-03],
    [-3.40935112e-02],
    [-6.30249662e+01]
    ])

     #wide angle camera module 3
    camera_matrix_2 = np.array(
    [[533.19512447, 0.0, 345.74213966],
    [  0.0, 532.58952811, 223.34142568],
    [  0.0, 0.0, 1.0]])
    
    dist_coefficients_2 = np.array(
    [[ 0.01978526],[-0.49556434],[-0.00970043],[0.0188091],[1.46700603]])
    
    detected_markers = {}
    z_0 = 0
    z_5 = 200

    header_size = struct.calcsize("=HH")

    mode = get_flight_mode()
    if mode != 'LAND':
        set_flight_mode('LAND')
        print ('mode set to LAND')

    
    

    try:

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
            frame = bytes()
            while len(frame) < bytes_to_read:
                frame += s.recv(min(bytes_to_read - len(frame), 4096))

            # convert to numpy array
            frame = np.frombuffer(frame, np.uint8).reshape((height, width))

            # Convert the frame to grayscale
            #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            # Detect the markers in the frame
            corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
            aruco.drawDetectedMarkers(frame, corners)
            corners = np.array(corners)
            # Draw detected markers on the frame
            #ids = ids.flatten
            if ids is not None:
                ids = ids.flatten()
                aruco.drawDetectedMarkers(frame, corners, ids=None, borderColor=(0, 255, 0))
                for (markerCorner, markerID) in zip(corners, ids):

                    if markerID==0:
                        marker_size=20
                    if markerID==5:
                        marker_size=8
                
                    ret = aruco.estimatePoseSingleMarkers(markerCorner, marker_size, camera_matrix_2, dist_coefficients_2)
                    (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                    x, y, z = -tvec[1], tvec[0], tvec[2]
                    detected_markers[markerID] = (x, y, z)
                    #send_land_message_v1()
                    #send_land_message_v2(dist_m=z*0.01, x_m= x*0.01,y_m=y*0.01,z_m=z*0.01)
                    #print (x, x_pitch_error)

                    if markerID not in marker_positions:
                        marker_positions[markerID] = []
                    marker_positions[markerID].append([x, y, z, DESCENT_VELOCITY])
                    

                    marker_position = f'MARKER {markerID}: x={x:.2f} y={y:.2f} z={z:.2f}'

                    # extract the marker corners (which are always returned in
                    # top-left, top-right, bottom-right, and bottom-left order)
                
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    # convert each of the (x, y)-coordinate pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    # compute and draw the center (x, y)-coordinates of the ArUco
                    # marker
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                    angle_x = math.atan2(x,z)
                    angle_y = math.atan2(y,z)

                    x_ang = (cX - x_res * 0.5)* horizontal_fov/x_res
                    y_ang = (cY - y_res * 0.5)* vertical_fov/y_res
                                  

                    #send_land_message_v2(dist_m=z*0.01, x_m=x*0.01,y_m=y*0.01,z_m=z*0.01)
                    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                    
                    # Draw marker ID and position
                    cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, marker_position, (10, 50 + 20 * markerID), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (102, 0, 255), 2)

                for key in detected_markers.keys():
                    if key == 5:
                        z_5 = detected_markers[5][2]
                        x_5 = detected_markers[5][0]
                        y_5 = detected_markers[5][1]

                    if key == 0:
                        z_0 = detected_markers[0][2]
                        x_0 = detected_markers[0][0]
                        y_0 = detected_markers[0][1]

                print ('tage 5 height',z_0,'tag 0 height',z_5)

                #send_land_message_v1(x_rad= angle_y, y_rad= -angle_x, dist_m=z*0.01)
                if z_5>150 and z_0!=0:
                    send_land_message_v2(x_m=x_0*0.01, y_m=y_0*0.01, z_m=z_0*0.01, dist_m=z_0*0.01)
                if z_5<150:
                    send_land_message_v2(x_m=x_5*0.01, y_m=y_5*0.01, z_m=z_5*0.01, dist_m=z_5*0.01)
                    
            else:
                print("aruco not detected")
                print("continuing descent")

                
            # Display the frame
            cv2.imshow('ArUco Pose Estimation', frame)
                
            # Break the loop when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Saving data before exiting...")
    finally:
        # Save all marker positions once per execution
        save_marker_positions(marker_positions)
        cv2.destroyAllWindows()

    # Release the video capture and close all OpenCV windows
    s.release()
    cv2.destroyAllWindows()

# Example usage
#detect_aruco_tags()

#set_land_mode()
#connection_mode = get_flight_mode()
#print(connection_mode)

#mavlink20 = 'MAVLINK20' in os.environ
#print(mavlink20)
#send_velocity(0,3,0)
#time.sleep(1)
set_message_interval(50)
set_flight_mode('GUIDED')
time.sleep(2)
arm_disarm_drone(1)
time.sleep(2)
takeoff(8)
time.sleep(2)
#current_yaw = get_yaw(connection)
#if current_yaw<0:
    #direction = 1
#else:
    #direction = 0
#send_yaw_command(connection, 180,120,direction,0)
#time.sleep(7)
precision_land_acc()
#arm_disarm_drone(0)
#send_velocity(0, 0.5, 0)
