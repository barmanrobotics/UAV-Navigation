import cv2
import cv2.aruco as aruco
import socket
import struct
import numpy as np
import math
import time
from pymavlink import mavutil

# connect to WebotsArduVehicle
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5599))

connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")

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

def detect_aruco_tags():
    # ArUco setup
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()

    horizontal_fov = 66*(math.pi/180) #horizontal_fov in radians for normal raspi camera module 3
    vertical_fov = 41*(math.pi/180) #vertical_fov in radians for normal raspi camera module 3

    ## The camera matrix is calculated for raspberry pi camera module 3 based on its specification sheet
    ## found here https://www.raspberrypi.com/documentation/accessories/camera.html

    LANDING_THRESHOLD_ANGLE = 0.02 #radians
    INITIAL_ANGLE = 0.1
    DESCENT_VELOCITY = 0.5 #m/s
    INITIAL_ERROR = 0.01
    LANDING_THRESHOLD_ERROR = 0.01
    vel = 0.05

    x_res= 1536 # pixels
    y_res = 864 # pixels
    s_x = 6.45  # mm
    s_y = 3.63  # mm
    focal_length = 4.74 # mm
    f_x = (x_res/s_x)*focal_length 
    f_y = (y_res/s_y)*focal_length
    o_x = (x_res/2) + 0.5
    o_y = (y_res/2) + 0.5
    camera_matrix = [[f_x,0,o_x],[0,f_y,o_y],[0,0,1]]
    #camera_matrix = [[3304.74173631,0.00000000,2250.15430808],[0.00000000,3317.15405795,1108.87277414],[0.00000000,0.00000000,1.00000000]]
    dist_coefficients =[0.0,0.0,0.0,0.0,0.0]
    #dist_coefficients = [0.00548862,0.12807064,-0.01048389,-0.00508286,-0.27971602]

    np_camera_matrix = np.array(camera_matrix)
    np_dist_coefficients = np.array(dist_coefficients)

    marker_size = 20 #cm

    header_size = struct.calcsize("=HH")
    
    i = 1

    x_max = 0.576
    y_max = 0.358

    x_min = 0.017
    y_min = 0.017

    a_min = 0.1
    a_max = 1

    h_min = 40
    h_max = 800

    c1 = x_max/x_min
    c2 = y_max/y_min

    k1_x = a_max/x_max
    k1_y = a_max/y_max

    kp = 0.5
    kd = 0.07
    ki = 0.0


    #k2_x = (a_max - a_min*c1)/(h_max + h_min*c1)
    #k1_x = (a_min + k2_x*h_min)/x_min

    #k2_y = (a_max - a_min*c2)/(h_max + h_min*c2)
    #k1_y = (a_min + k2_y*h_min)/y_min

    #print (k1_x, k2_x)

    #ax = k1_x*x_max - k2_x*h_max
    #ay = k1_y*0.04 - k2_y*8

    #print(ax, ay)

    previous_time = time.time()
    previous_error_x = 0.0
    previous_error_y = 0.0
    error_sum_x = 0.0
    error_sum_y = 0.0

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
            
                ret = aruco.estimatePoseSingleMarkers(corners,marker_size,np_camera_matrix,np_dist_coefficients)
                (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                x = '{:.2f}'.format(tvec[0])
                y = '{:.2f}'.format(tvec[1])
                z = '{:.2f}'.format(tvec[2])

                marker_postion = 'MARKER POSITION: x='+x+' y= '+y+' z= '+z

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
            
                # Check if within the landing threshold
                ar_alt = tvec[2]


                error_x = tvec[1]/100.0
                error_y = tvec[0]/100.0
                
                print(f"error_y: {error_y}")
                

                p_out_x = kp*error_x
                p_out_y = kp*error_y

                current_time = time.time()
                if i==1:
                    dt = 0.1
                    previous_error_x = error_x
                    previous_error_y = error_y
                else:
                    dt = current_time - previous_time
                
                print(f"dt: {dt}")
                previous_time = current_time

                derivative_x = (error_x - previous_error_x)/dt
                derivative_y = (error_y - previous_error_y)/dt
                print(f"derivative_y: {derivative_y}")
                previous_error_x = error_x
                previous_error_y = error_y

                d_out_x = kd*derivative_x
                d_out_y = kd*derivative_y

                error_sum_x += error_x*dt
                error_sum_y += error_y*dt

                i_out_x = ki*error_sum_x
                i_out_y = ki*error_sum_y


                print(f"d_out_y: {d_out_y}")
                print(f"p_out_y: {p_out_y}")

                a_x = p_out_x + d_out_x + i_out_x
                a_y = p_out_y + d_out_y + i_out_y


                #ax = 0.01 + kp*(e_y) + kd*(e_y_t)
                #ay = 0.01 + kp*(e_x) + kd*(e_x_t)

                #x_ang_old = x_ang
                #y_ang_old = y_ang

               #Sprint(f"e_x: {x_ang}, e_x_t: {e_x_t}")
                a_x= -a_x if abs(error_x) > INITIAL_ERROR else 0
                a_y= a_y if abs(error_y) > INITIAL_ERROR else 0

                if a_x<-0.6:
                    a_x=-0.6
                if a_x>0.6:
                    a_x=0.6
                if a_y<-0.6:
                    a_y=-0.6
                if a_y>0.6:
                    a_y=0.6
                print (f"a_y: {a_y} a_x: {a_x}")
                print("")

                if ar_alt<200:
                    DESCENT_VELOCITY = 0.1
                
                send_velocity(a_x, a_y, DESCENT_VELOCITY)

                

                if ar_alt<150:
                    DESCENT_VELOCITY = 0.1

                #if abs(x_ang) > INITIAL_ANGLE or abs(y_ang) > INITIAL_ANGLE:
                    #ax = -ax if y_ang > INITIAL_ANGLE else (ax if y_ang < -INITIAL_ANGLE else 0)
                    #ay = ay if x_ang > INITIAL_ANGLE else (-ay if x_ang < -INITIAL_ANGLE else 0)

                    #print (ax, ay)
                    #send_velocity(ax, ay, 0)

                #if abs(error_y) > INITIAL_ERROR or abs(error_x) > INITIAL_ERROR:
                    #ax = -a_x if error_y > INITIAL_ERROR else (ax if error_y < -INITIAL_ANGLE else 0)
                    #ay = 0

                    #print (ax, ay)
                    #send_velocity(ax, ay, 0)    

                #if abs(x_ang) < LANDING_THRESHOLD_ANGLE and abs(y_ang) < LANDING_THRESHOLD_ANGLE:
                    # Descend straight down
                    #send_velocity(0, 0, DESCENT_VELOCITY)
                #else:
                    # Adjust horizontal movement based on x_ang and y_ang
                    #ax = -a_x if y_ang > LANDING_THRESHOLD_ANGLE else (ax if y_ang < -LANDING_THRESHOLD_ANGLE else 0)
                    #ay = ay if x_ang > LANDING_THRESHOLD_ANGLE else (-ay if x_ang < -LANDING_THRESHOLD_ANGLE else 0)
                    #print(f"ax: {ax}, ay: {ay}")
                    #send_velocity(ax, ay, DESCENT_VELOCITY)
                


                # draw the ArUco marker ID on the image
                cv2.putText(frame, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

                cv2.putText(frame, str(marker_postion),
                    (10,50), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (102, 0, 255), 2)
                
                i = i+1
        else:
            print("aruco not detected")
            print("continuing descent")       
            send_velocity(0, 0, 0.05)

            
        # Display the frame
        cv2.imshow("ArUco Tag Detection", frame)
        #print("Detected IDs:", ids)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and close all OpenCV windows
    s.release()
    cv2.destroyAllWindows()

# Example usage
#detect_aruco_tags()
connection_mode = get_flight_mode()
print(connection_mode)
detect_aruco_tags()
#arm_disarm_drone(0)
#send_velocity(0, 0.5, 0)
