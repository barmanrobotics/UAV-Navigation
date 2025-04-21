# precision_landing.py
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
from pymavlink import mavutil
import os
import csv

try:
    from picamera2 import Picamera2

    x_res= 640 # pixels  #A resolution of 1014 x 760 works well and gives the same 30fps output
    y_res = 480# pixels
    FPS = 120

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        raw={"size": (1536, 864)},
        main={"format": 'RGB888', "size": (x_res,y_res)},
        controls={"FrameRate": FPS}
    )
    # Apply the configuration to the camera
    picam2.configure(camera_config)
    picam2.start()
    #picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #comment this out if you don't want autofocus
    print("Waiting for camera to start")
    time.sleep(1)
    print("Camera Started")
except:
    print("Camera failed to initialize.")


connection = None

"Author: Arnab Chatterjee"
"Version: 0.1"



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
            writer.writerow(["X", "X_compensated", "Y","Y_compensated", "Z", "Z_compensated"])  # Write header
            writer.writerows(positions)
        print(f"Saved positions for marker {marker_id} in {file_name}")

def set_message_interval(rate_hz,code):
    """
    Sets the update rate for a specific MAVLink message using MAV_CMD_SET_MESSAGE_INTERVAL.

    :param connection: MAVLink connection object
    :param message_id: MAVLink message ID (e.g., 30 for ATTITUDE)
    :param rate_hz: Desired frequency in Hz
    """

    # Note that this would not work out of the box. You need to set the SERIALX
    # options bitmask in the drone or SITL to ignore
    # streamrate or else it will keep defaulting back to 4 Hz

    interval_us = int(1e6 / rate_hz)  # Convert Hz to microseconds
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,  # Confirmation
        code,
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
    msg = connection.recv_match(type='ATTITUDE', blocking=True)
    if msg:
        roll = msg.roll # Convert radians to degrees
        pitch = msg.pitch
        yaw = msg.yaw

        return roll, pitch, yaw
    return None

def takeoff(altitude):
    message_rate = 50
    code = 33
    set_message_interval(message_rate,code)
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

def get_yaw(master):
    """
    Get the yaw angle of a quadcopter using MAVLink.
    
    :param master: MAVLink connection object
    :return: Yaw angle in degrees
    """
    message_rate = 50
    code = 30
    set_message_interval(message_rate,code)
    time.sleep(1)
    while True:
        # Receive attitude message
        msg = master.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            yaw = msg.yaw * (180 / 3.141592653589793)  # Convert radians to degrees
            return yaw

def send_yaw_command(master, target_yaw, yaw_speed, relative):
    """
    Sends a MAV_CMD_CONDITION_YAW command to control the drone's yaw.

    :param master: MAVLink connection object
    :param target_yaw: Desired yaw angle (degrees)
    :param yaw_speed: Yaw rotation speed (degrees/sec)
    :param direction: 1 for clockwise, -1 for counterclockwise
    :param relative: 1 for relative yaw, 0 for absolute yaw
    """

    current_yaw = get_yaw(connection)
    if current_yaw<0:
        direction = 1
    else:
        direction = 0
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
    

def send_land_message_v2(x_rad=0, y_rad=0, dist_m=0, x_m=0,y_m=0,z_m=0, time_usec=0, target_num=0):
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
        (1,0,0,0),  # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        2,          # type of landing target: 2 = Fiducial marker
        1,          # position_valid boolean
    )


def precision_land_mode():

    message_rate = 50
    code = 30
    set_message_interval(message_rate,code)
    time.sleep(1)
    attitude = get_drone_attitude()

    send_yaw_command(connection, 0,120,0)
    time.sleep(7)

    # ArUco setup
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    marker_positions = {}

    #distance from the center of the tag along x or y axes where you'd like the center of the camera to land
    #if you want the center of the camera to land at the center of the tag, set offsets to 0.

    #offset_x = float(input("Please enter the x offset distance in cm. Back is positive: "))
    #offset_y = float(input("Please enter the y offset distance in cm. Right is positive: "))
    marker_size = 19.2
    offset_x = -3.6 # Enter the x offset distance in cm. a positive value will make the camera land infornt of the tag and vice versa:
    offset_y = 0    # Enter the y offset distance in cm. a positive value will make the camera land right of the tag and vice versa:
    threshold = 1 #cm if error is within this, don't correct the quadcopter

    cam_dist = 14.754 #Distance of camera from cg in x axes

    camera_matrix = np.array([[570.41873064,0.0,285.59706468],
    [   0.0,564.78457011,225.93404959],
    [  0.0,0.0,1.0]])

    dist_coefficients = np.array([[ 0.10980456], [-0.32288555], [-0.00354651], [-0.02758834], [-1.85357048]])

    #Changes the mode to land if mode is not land

    mode = get_flight_mode()
    if mode != 'LAND':
        set_flight_mode('LAND')
        print ('mode set to LAND')
    

    try:

        while True:

            attitude = get_drone_attitude()
            img = picam2.capture_array()
    
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

            # Detect the markers in the frame
            corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
            corners = np.array(corners)
            np_id  = np.array(ids)
            found = np.isin(0, np_id) #if marker 0 and only one marker is detected then proceed
            if found == True and len(corners)==1:
                ids = ids.flatten()
                aruco.drawDetectedMarkers(frame, corners, ids=None, borderColor=(0, 255, 0))
                for (markerCorner, markerID) in zip(corners, ids):
                
                    ret = aruco.estimatePoseSingleMarkers(markerCorner, marker_size, camera_matrix, dist_coefficients)
                    (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                    x, y, z = -tvec[1], tvec[0], tvec[2]
                    roll = attitude[0]
                    pitch = attitude[1]

                    x_error_1 = z*math.tan(pitch) # When pitch is positive (i.e nose up), error is positive and added to the aruco x co-ordinate
                                             # When pitch is negative (i.e nose down), error is removed from the aruco x co-ordinate
                    y_error_1 = z*math.tan(-roll)  # Roll right is postive, the error should be removed from the y-co-ordinates as roll right increases the y co-ordinate

                    x_error_2 = cam_dist*(1-math.cos(pitch))

                    if pitch<0:
                        x_error_2 = -x_error_2 

                    x_compensated = x + x_error_1 + x_error_2
                    y_compensated = y + y_error_1
                    z_compensated = z - cam_dist*math.sin(pitch)

                    cor_x = x_compensated + offset_x
                    cor_y = y_compensated + offset_y
                    cor_z = z_compensated

                    #if the x and y errors are within threshold, do not move the drone
                    if abs(x_compensated + offset_x)<threshold:
                        cor_x = 0
                    if abs(y_compensated + offset_y)<threshold:
                        cor_y = 0
                    
                    #send_land_message_v2(x_rad= angle_y, y_rad= -angle_x, dist_m=z*0.01)
                    #send_land_message_v2(x_m=(x+offset_x)*0.01, y_m=(y+offset_y)*0.01, z_m=z*0.01, dist_m=z*0.01)
                
                    send_land_message_v2(x_m=(cor_x)*0.01, y_m=(cor_y)*0.01, z_m=cor_z*0.01, dist_m=cor_z*0.01)

                    #saves the error values to debug later
                    if markerID not in marker_positions:
                        marker_positions[markerID] = []
                    marker_positions[markerID].append([x, x_compensated, y, y_compensated, z, z_compensated])

                    #print ("X is: ", x, "X compensated is:", x_compensated, "Pitch is:", pitch, " radians")
                    #print ("Y is: ", y, "Y compensated is:", y_compensated, "Roll is:", roll, " radians")
                    #print ("Z is: ", z, "Z compensated is:", z_compensated, "Pitch is:", pitch, " radians")

            else:
                print("aruco not detected. Continuing normal descent")
                print("")
            
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Saving data before exiting...")
    finally:
        # Save all marker positions once per execution
        save_marker_positions(marker_positions)

