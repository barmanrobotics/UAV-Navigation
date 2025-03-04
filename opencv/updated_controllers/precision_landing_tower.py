import cv2
import cv2.aruco as aruco
import socket
import struct
import numpy as np
import time
from pymavlink import mavutil

# connect to WebotsArduVehicle
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5599))

connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")

# Camera calibration data
camera_matrix = np.array([
    [917.1777059, 0.00000000, 323.46713801],
    [0.00000000, 926.27018107, 240.68578702],
    [0.00000000, 0.00000000, 1.00000000]
])

dist_coefficients = np.array([
    [-4.21053311e-01],
    [6.10894559e+00],
    [1.08524770e-03],
    [-3.40935112e-02],
    [-6.30249662e+01]
])

def get_altitude():
    """
    Get the altitude of the quadcopter using MAVLink 2.
    :return: Altitude in meters
    """
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            return msg.relative_alt / 1000.0  # Convert from mm to meters

def send_acceleration(ax, ay, vz):
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

def send_velocity(vz):
    # Send velocity command in the drone's NED frame
    connection.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # Frame of reference (Body frame)
        0b110111000111,  # Control velocity only
        0, 0, 0,  # Position x, y, z (not used)
        0, 0, vz,  # Velocity x, y, z (used)
        0, 0, 0,  # Acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )

def send_yaw(yaw,ax,ay,vz):
    # Send velocity command in the drone's NED frame
    connection.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        1,  # Frame of reference (Body frame)
        2503,  # Control velocity only
        0, 0, 0,  # Position x, y, z (not used)
        ax, ay, vz,  # Velocity x, y, z
        0, 0, 0,  # Acceleration (not used)
        yaw, 0  # yaw, yaw_rate (not used)
        )
    print('setting yaw')

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

def precision_land():
    # ArUco setup
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    
    marker_size = 8  # cm #Make sure this size matches the size of the aruco tag
    header_size = struct.calcsize("=HH")
    detected_markers = {}

    error_threshold = 0.01

    previous_time = time.time()
    previous_error_x = 0.0
    previous_error_y = 0.0
    error_sum_x = 0.0
    error_sum_y = 0.0
    i = 1
    z_1 = 0
    z_2 = 200

    offset_1 = 0
    offset_2 = 11.5

    desired_yaw = 0 #set the desired yaw of the drone to start landing from
    yaw_speed = 20

    current_yaw = get_yaw(connection)
    yaw_error = abs(desired_yaw-current_yaw)
    if yaw_error>2:
        if current_yaw<0:
            send_yaw_command(connection, desired_yaw, yaw_speed, 1, 0)
            time.sleep(10)
        else:
            send_yaw_command(connection, desired_yaw, yaw_speed, -1, 0)
            time.sleep(10)
    
    

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
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        
        # Detect the markers in the frame
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        
        if ids is not None:
            ids = ids.flatten()
            aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))
            
            
            for markerCorner, markerID in zip(corners, ids):
                if markerID==0:
                    marker_size=5
                else:
                    marker_size=15
                
                ret = aruco.estimatePoseSingleMarkers(markerCorner, marker_size, camera_matrix, dist_coefficients)
                (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                x, y, z = -tvec[1], tvec[0], tvec[2]
                detected_markers[markerID] = (x, y, z)

                marker_position = f'MARKER {markerID}: x={x:.2f} y={y:.2f} z={z:.2f}'
                
                # Extract and convert marker corners
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topLeft, topRight, bottomRight, bottomLeft = map(lambda pt: (int(pt[0]), int(pt[1])), [topLeft, topRight, bottomRight, bottomLeft])
                
                # Compute and draw the center of the ArUco marker
                cX, cY = int((topLeft[0] + bottomRight[0]) / 2.0), int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                
                # Draw marker ID and position
                cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, marker_position, (10, 50 + 20 * markerID), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (102, 0, 255), 2)

            for key in detected_markers.keys():
                if key == 5:
                    print('ID 5 found')
                    found = 'TRUE'
                    z_1 = detected_markers[5][2]
                    print (z_1)
                if key == 0:
                    z_2 = detected_markers[0][2]
                    print (z_2)
            print (z_1,z_2)

            if z_1>360 or z_2>300:
                coordinates = detected_markers.get(5)
                print(coordinates)
                values = precision_alignment(coordinates,previous_error_x,previous_error_y,error_sum_x,error_sum_y,i,previous_time,error_threshold,offset_1)
                previous_error_x = values[0]
                previous_error_y = values[1]
                error_sum_x = values[2]
                error_sum_y = values[3]
                i = values[4]
                previous_time = values[5]
                print('tracking tag 5')
            else:
                coordinates = detected_markers.get(0)
                values = precision_alignment(coordinates,previous_error_x,previous_error_y,error_sum_x,error_sum_y,i,previous_time,error_threshold,offset_2)
                previous_error_x = values[0]
                previous_error_y = values[1]
                error_sum_x = values[2]
                error_sum_y = values[3]
                i = values[4]
                previous_time = values[5]
                print('tracking tag 0')

        else:
            print('No tag detected')
            print ('Altitude is below 1 m, decending down')
            send_velocity(0.15)
        

        # Display the frame
        cv2.imshow('ArUco Pose Estimation', frame)
        
        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    cv2.destroyAllWindows()

# Example usage

def precision_alignment(coordinates,previous_error_x,previous_error_y,error_sum_x,error_sum_y,i,previous_time,error_threshold,offset):

    marker_id1 = 5
    marker_id2 = 0
    desired_heading = 0

    kp = 0.4
    kd = 0.07
    ki = 0.0

    DESCENT_VELOCITY = 0.2

    #offset_1 = -12.5 for aruco on top of the tower
    #Get aruco positon dictionary

    
    x = coordinates[0]
    y = coordinates[1]
    z = coordinates[2]

    
    error_x = (x + offset)/100.0
    error_y = y/100.0

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
    

    derivative_x = (error_x - previous_error_x)/dt
    derivative_y = (error_y - previous_error_y)/dt

    previous_error_x = error_x
    previous_error_y = error_y
    previous_time = current_time

    d_out_x = kd*derivative_x
    d_out_y = kd*derivative_y

    error_sum_x += error_x*dt
    error_sum_y += error_y*dt

    i_out_x = ki*error_sum_x
    i_out_y = ki*error_sum_y


    a_x = p_out_x + d_out_x + i_out_x
    a_y = p_out_y + d_out_y + i_out_y

    threshold_acc = 0.4
    #Sprint(f"e_x: {x_ang}, e_x_t: {e_x_t}")
    a_x= a_x if abs(error_x) > error_threshold else 0
    a_y= a_y if abs(error_y) > error_threshold else 0

    if a_x<-threshold_acc:
        a_x=-threshold_acc
    if a_x>threshold_acc:
        a_x=threshold_acc
    if a_y<-threshold_acc:
        a_y=-threshold_acc
    if a_y>threshold_acc:
        a_y=threshold_acc
    print (f"a_y: {a_y} a_x: {a_x}")
    print("")

    yaw = 0
    if z<100:
        DESCENT_VELOCITY = 0.15
    send_acceleration(a_x,a_y,DESCENT_VELOCITY)
    return previous_error_x, previous_error_y, error_sum_x, error_sum_y, i,previous_time
    
precision_land()
