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
            writer.writerow(["X", "Y", "Z"])  # Write header
            writer.writerows(positions)
        print(f"Saved positions for marker {marker_id} in {file_name}")

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

def send_velocity(vx,vy,vz):
    #This function can take only velocity input
    # Send velocity command in the drone's NED frame
    connection.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # Frame of reference (Body frame)
        0b110111000111,  # Control velocity only
        0, 0, 0,  # Position x, y, z (not used)
        vx, vy, vz,  # Velocity x, y, z (used)
        0, 0, 0,  # Acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )


def send_velocity_yaw(yaw,vx,vy,vz):
    #This function can take velocity as well as yaw input. Yaw input is global and is in radians
    #0 is North. pi is South etc
    # Send velocity and yaw command in the drone's NED frame
    connection.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        1,  # Frame of reference (Body frame)
        2503,  # Control velocity only
        0, 0, 0,  # Position x, y, z (not used)
        vx, vy, vz,  # Velocity x, y, z
        0, 0, 0,  # Acceleration (not used)
        yaw, 0  # yaw, yaw_rate (not used)
        )

def send_acceleration(ax, ay, vz):
    #This function can take position, acceleration as well as velocity inputs
    #Currently it takes ax, ay and vz (velocity of decend or climb depending upon sign)
    # Send velocity command in the drone's NED frame
    connection.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # Frame of reference (Body frame)
        0b110000000000,  # Control velocity only
        0, 0, 0,  # Position x, y, z (not used)
        0, 0, vz,  # Velocity x, y, z
        ax, ay, 0,  # Acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )

def precision_land_velocity():
    # ArUco setup
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    marker_positions = {}

    #distance from the center of the tag along x or y axes where you'd like the center of the camera to land
    #if you want the center of the camera to land at the center of the tag, set it to 0.

    marker_size = float(input("Please enter the marker size in cm: "))
    offset_x = float(input("Please enter the x offset distance in cm. Back is positive: "))
    offset_y = float(input("Please enter the y offset distance in cm. Right is positive: "))
    #DESCENT_VELOCITY = float(input("Enter initial descent velocity: ")) 
    #ERROR_THRESHOLD = float(input("Enter threshold error: "))
    #marker_track = float(input("WHich marker ID you want to track? "))
    DESCENT_VELOCITY = 0.2
    ERROR_THRESHOLD = 0.01
    marker_track = 0

    #x_res= 640 # pixels
    #y_res = 480 # pixels

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

    np_camera_matrix = np.array(camera_matrix)
    np_dist_coefficients = np.array(dist_coefficients)

    header_size = struct.calcsize("=HH")
    
    i = 1

    kp = 0.2
    kd = 0.065
    ki = 0.0

    previous_time = time.time()
    previous_error_x = 0.0
    previous_error_y = 0.0
    error_sum_x = 0.0
    error_sum_y = 0.0

    try:

        while True:
            time.sleep(0.1)
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
                
                    ret = aruco.estimatePoseSingleMarkers(markerCorner, marker_size, camera_matrix, dist_coefficients)
                    (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                    x, y, z = -tvec[1], tvec[0], tvec[2]
                        
                    if markerID not in marker_positions:
                        marker_positions[markerID] = []
                    marker_positions[markerID].append([x, y, z])

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
                    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

                    #calcualte x_ang and y_ang

                    #x_ang = (cX - x_res * 0.5)* horizontal_fov/x_res
                    #y_ang = (cY - y_res * 0.5)* vertical_fov/y_res
                    #print(x_ang, y_ang)

                    #print(f"x_ang: {x_ang}, y_ang: {y_ang}")

                    if marker_track == 0:
                
                        # storing the altitude
                        ar_alt = tvec[2]

                        #offsetting the error in x direction to land offset distance away from center of tag
                        error_offset_x = -tvec[1]+offset_x
                        error_offset_y = tvec[0]+offset_y
                        error_x = error_offset_x/100.0 #error in m
                        error_y = error_offset_y/100.0 #error in m
                        

                        p_out_x = kp*error_x
                        p_out_y = kp*error_y

                        current_time = time.time()
                        if i==1:
                            dt = 0.1
                            previous_error_x = error_x
                            previous_error_y = error_y
                            i+=1
                        else:
                            dt = current_time - previous_time
                        
                        previous_time = current_time

                        derivative_x = (error_x - previous_error_x)/dt
                        derivative_y = (error_y - previous_error_y)/dt
                        previous_error_x = error_x
                        previous_error_y = error_y

                        d_out_x = kd*derivative_x
                        d_out_y = kd*derivative_y

                        error_sum_x += error_x*dt
                        error_sum_y += error_y*dt

                        i_out_x = ki*error_sum_x
                        i_out_y = ki*error_sum_y


                        a_x = p_out_x + d_out_x + i_out_x
                        a_y = p_out_y + d_out_y + i_out_y


                        #ax = 0.01 + kp*(e_y) + kd*(e_y_t)
                        #ay = 0.01 + kp*(e_x) + kd*(e_x_t)

                        #x_ang_old = x_ang
                        #y_ang_old = y_ang

                        #Sprint(f"e_x: {x_ang}, e_x_t: {e_x_t}")
                        
                        
                        max_velocity = 0.4
                        min_velocity = 0.01

                        a_x = a_x if abs(error_x) > ERROR_THRESHOLD else 0
                        a_y = a_y if abs(error_y) > ERROR_THRESHOLD else 0

                        def clamp(value):
                            if value > 0:
                                return max(min_velocity, min(value, max_velocity))
                            elif value < 0:
                                return min(-min_velocity, max(value, -max_velocity))
                            return 0  # Ensures zero remains zero

                        a_x, a_y = clamp(a_x), clamp(a_y)
                        if ar_alt<100:
                            DESCENT_VELOCITY = 0.15

                        if random.random() < 0.2:  # 20% chance to trigger a gust
                            a_x = random.uniform(-0.1, 0.1)
                            a_y = random.uniform(-0.1, 0.1)
                            print("gust of", a_x, a_y)
                            send_acceleration(a_x,a_y,0)

                        else:
                            send_velocity(a_x,a_y, DESCENT_VELOCITY)
                        print (f"v_y: {a_y} v_x: {a_x}")
                        print("")

                        
                        
                        #yaw value of 0 is given to align the drone to true north
                        

                    
                    # Draw marker ID and position
                    cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, marker_position, (10, 50 + 20 * markerID), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (102, 0, 255), 2)
                    
            else:
                print("aruco not detected")
                print("continuing descent")       
                send_velocity(0, 0, 0.05)

                
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
#run precision land
precision_land_velocity()
#arm_disarm_drone(0)
#send_velocity(0, 0.5, 0)
