#Author: Arnab Chatterjee

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
import socket
import struct
from pymavlink import mavutil
from picamera2 import Picamera2
from libcamera import controls

timestamp = time.strftime("%Y%m%d_%H%M%S")
output_filename = f'output_{timestamp}.mp4'

x_res = 640
y_res = 480
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_filename, fourcc, 60, (x_res, y_res))

print(f"Video output initialized: {output_filename}")

# connect to Picamera
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
#picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #comment this out if you don't want autofocusy
time.sleep(1)  # Allow time for autofocus adjustment
print("Waiting for camera to start")
time.sleep(2)
print("Camera Started")

connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")

camera_matrix_1 = np.array([
    [917.1777059, 0.00000000, 323.46713801],
    [0.00000000, 926.27018107, 240.68578702],
    [0.00000000, 0.00000000, 1.00000000]
])

dist_coefficients_1= np.array([
    [-4.21053311e-01],
    [6.10894559e+00],
    [1.08524770e-03],
    [-3.40935112e-02],
    [-6.30249662e+01]
])

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
        time.sleep(0.5)


def alt():
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    current_alt = msg.relative_alt / 1000.0
    print(f"Current altitude: {current_alt:.2f} meters")
    return current_alt


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

def send_velocity(ax, ay, vz):
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

def detect_aruco_tags():
    # ArUco setup
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()

    horizontal_fov = 66*(math.pi/180) #horizontal_fov in radians for normal raspi camera module 3
    vertical_fov = 41*(math.pi/180) #vertical_fov in radians for normal raspi camera module 3

    ## The camera matrix is calculated for raspberry pi camera module 3 based on its specification sheet
    ## found here https://www.raspberrypi.com/documentation/accessories/camera.html

    DESCENT_VELOCITY = 0.3 #m/s
    INITIAL_ERROR = 0.01

    s_x = 6.45  # mm
    s_y = 3.63  # mm
    focal_length = 4.74 # mm
    f_x = (x_res/s_x)*focal_length 
    f_y = (y_res/s_y)*focal_length
    o_x = (x_res/2) + 0.5
    o_y = (y_res/2) + 0.5
    camera_matrix = [[f_x,0,o_x],[0,f_y,o_y],[0,0,1]]
    dist_coefficients =[0.0,0.0,0.0,0.0,0.0]

    np_camera_matrix = camera_matrix_1
    np_dist_coefficients = dist_coefficients_1

    marker_size = 18#cm
    start_time = time.time()
    counter = 0

    kp = 0.5
    kd = 0.07
    ki = 0.0

    previous_error_x = 0.0
    previous_error_y = 0.0
    error_sum_x = 0.0
    error_sum_y = 0.0
    
    i = 1

    while True:
        # receive header
        # receive header
        img = picam2.capture_array()
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        counter += 1
        cur_time = time.time()
        elapsed_time = cur_time-start_time
        frame_rate = counter/elapsed_time
        #print("I'm here")

        # Detect the markers in the frame
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        corners = np.array(corners)
        # Draw detected markers on the frame
        #ids = ids.flatten
        np_id  = np.array(ids)
        found = np.isin(0, np_id)
        if found == True and len(corners)==1:
            print(found)
            ids = ids.flatten()
            aruco.drawDetectedMarkers(img, corners, ids=None, borderColor=(0, 255, 0))
            for (markerCorner, markerID) in zip(corners, ids):
            
                ret = aruco.estimatePoseSingleMarkers(corners,marker_size,np_camera_matrix,np_dist_coefficients)
                (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                x = '{:.2f}'.format(tvec[0])
                y = '{:.2f}'.format(tvec[1])
                z = '{:.2f}'.format(tvec[2])
                fps = '{:.2f}'.format(frame_rate)

                marker_postion = 'MARKER POSITION: x='+x+' y= '+y+' z= '+z+' fps= '+fps

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
                cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(img, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

                cv2.putText(img, str(marker_postion),
                    (10,25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (102, 0, 255), 2)

                ar_alt = tvec[2]


                error_x = tvec[1]/100.0
                error_y = tvec[0]/100.0
                
                p_out_x = kp*error_x
                p_out_y = kp*error_y

                current_time = time.time()
                if i==1:
                    dt = 0.1
                    previous_error_x = error_x
                    previous_error_y = error_y
                    i = 2
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


                a_x= -a_x if abs(error_x) > INITIAL_ERROR else 0 #Found out that the axes on the controller is opposite of the simulation. Cheers!
                a_y= a_y if abs(error_y) > INITIAL_ERROR else 0 #Flip the negatve sign if following simulation

                if a_x<-0.4:
                    a_x=-0.4
                if a_x>0.4:
                    a_x=0.4
                if a_y<-0.4:
                    a_y=-0.4
                if a_y>0.4:
                    a_y=0.4
                print (f"a_y: {a_y} a_x: {a_x}")
                print("")

                if ar_alt<200:
                    DESCENT_VELOCITY = 0.2
                else:
                    DESCENT_VELOCITY = 0.3
                
                send_velocity(a_x, a_y, DESCENT_VELOCITY)


                # draw the ArUco marker ID on the image
                
                acc_x = '{:.2f}'.format(a_x)
                acc_y = '{:.2f}'.format(a_y)

                acc_values = 'Acceleration: a_x='+acc_x+' a_y= '+acc_y

                cv2.putText(img, str(acc_values),
                    (10,50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (102, 0, 255), 2)
                
        else:
            print("aruco not detected")
            print("continuing descent")       
            send_velocity(0, 0, 0.15)

            
        #Write the frme toa video file
        out.write(img)
    
    # Release the video capture and close all OpenCV windows
    picam2.stop()
    picam2.close()
    cv2.destroyAllWindows()

def detect_aruco_tags_2():
    # ArUco setup
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()

    horizontal_fov = 66*(math.pi/180) #horizontal_fov in radians for normal raspi camera module 3
    vertical_fov = 41*(math.pi/180) #vertical_fov in radians for normal raspi camera module 3

    ## The camera matrix is calculated for raspberry pi camera module 3 based on its specification sheet
    ## found here https://www.raspberrypi.com/documentation/accessories/camera.html

    s_x = 6.45  # mm
    s_y = 3.63  # mm
    focal_length = 4.74 # mm
    f_x = (x_res/s_x)*focal_length 
    f_y = (y_res/s_y)*focal_length
    o_x = (x_res/2) + 0.5
    o_y = (y_res/2) + 0.5
    camera_matrix = [[f_x,0,o_x],[0,f_y,o_y],[0,0,1]]
    #print (camera_matrix)
    time.sleep(2)
    #camera_matrix = [[3304.74173631,0.00000000,2250.15430808],[0.00000000,3317.15405795,1108.87277414],[0.00000000,0.00000000,1.00000000]]
    dist_coefficients =[0.0,0.0,0.0,0.0,0.0]
    #dist_coefficients = [0.00548862,0.12807064,-0.01048389,-0.00508286,-0.27971602]

    np_camera_matrix = np.array(camera_matrix)
    np_dist_coefficients = np.array(dist_coefficients)

    marker_size = 18#cm
    start_time = time.time()
    counter = 0

    while True:
        # receive header
        img = picam2.capture_array()

        frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        counter += 1
        cur_time = time.time()
        elapsed_time = cur_time-start_time
        frame_rate = counter/elapsed_time
        print(frame_rate)

        # Detect the markers in the frame
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        corners = np.array(corners)
        # Draw detected markers on the frame
        #ids = ids.flatten
        np_id  = np.array(ids)
        found = np.isin(0, np_id)
        if found == True and len(corners)==1:
            ids = ids.flatten()
            aruco.drawDetectedMarkers(img, corners, ids=None, borderColor=(0, 255, 0))
            for (markerCorner, markerID) in zip(corners, ids):
            
                ret = aruco.estimatePoseSingleMarkers(corners,marker_size,np_camera_matrix,np_dist_coefficients)
                (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                x = '{:.2f}'.format(tvec[0])
                y = '{:.2f}'.format(tvec[1])
                z = '{:.2f}'.format(tvec[2])
                fps = '{:.2f}'.format(frame_rate)

                marker_position = 'MARKER POSITION: x='+x+' y= '+y+' z= '+z+' fps= '+fps

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
                cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(img, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

                cv2.putText(img, str(marker_position),
                    (10,50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (102, 0, 255), 2)

            # draw the bounding box of the ArUCo detection
            
        # Display the frame
        #cv2.imshow('ArUco Pose Estimation', img)
        out.write(img)

    # Release the video capture and close all OpenCV windows
    
    picam2.stop()
    picam2.close()
    

# Example usage
#detect_aruco_tags()
<<<<<<< Updated upstream
#connection_mode = get_flight_mode()
#print(connection_mode)
#time.sleep(3)
#set_flight_mode("GUIDED_NOGPS")
#connection_mode = get_flight_mode()
#print(connection_mode)
#arm_disarm_drone(1)
#takeoff(6)
#time.sleep(5)
#set_flight_mode("LAND")
=======
connection_mode = get_flight_mode()
set_flight_mode("GUIDED")
arm_disarm_drone(1)
takeoff(2)
print(connection_mode)
>>>>>>> Stashed changes
detect_aruco_tags()
#time.sleep(3)
#picam2.stop()
#picam2.close()
#server_socket.close()