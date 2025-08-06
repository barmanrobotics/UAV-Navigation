# Bill Wang

# Modified code from Arnab Chatterje and Owen Bartlett




import time
import cv2
import numpy as np
import apriltag
import socket
import struct
from pymavlink import mavutil # type: ignore
import time
import math
import pymavlink.dialects.v20.all as dialect
# from picamera2 import Picamera2
# from libcamera import controls

tag_size = .5

def april_tag_land(tag_size, use_display = True):

    x_res= 640 # pixels
    y_res = 480 # pixels

    FPS = 120

    # connect to WebotsArduVehicle
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("127.0.0.1", 5599))
    print("Connected to socket")

    ## Picam code

    # picam2 = Picamera2()
    # camera_config = picam2.create_preview_configuration(
    #     raw={"size": (1536, 864)},
    #     main={"format": 'RGB888', "size": (x_res,y_res)},
    #     controls={"FrameRate": FPS}
    # )
    # # Apply the configuration to the camera
    # picam2.configure(camera_config)
    # picam2.start()

    # set yaw heading in rad
    def set_yaw(yaw_rad):
        yaw = yaw_rad/math.pi*180
        # print(f"set yaw to {yaw}\n")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            1, yaw, 10, 0, 0, 0, 0, 0
        )

    def set_mode(mode):
        mode_id = connection.mode_mapping().get(mode)
        if mode_id is None:
            print(f"Mode {mode} is not supported")
            return False
        connection.mav.set_mode_send(
            connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        while True:
            ack = connection.recv_match(type='HEARTBEAT', blocking=True)
            if ack.custom_mode == mode_id:
                print(f"Mode changed to {mode}")
                break
            time.sleep(0.5)

    def align(fwd, side, down, yaw):
        connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms, set to 0 as we are sending an instantaneous command
            connection.target_system, 
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Local frame, everything relative to current position
            int(0b100111111000),  # type_mask (0 = use, 1 = don't use)
            fwd,   # +forward/-back (m)
            side,   # +right/-left (m)
            down,   # Down (meters)
            0,            # velocity_x
            0,            # velocity_y
            0,            # velocity_z
            0,            # acceleration_x (not used)
            0,            # acceleration_y (not used)
            0,            # acceleration_z (not used)
            yaw,          # yaw (rad)
            0             # yaw_rate
        )


    # draw axes on tag
    def draw_axis(img, rvec, t, K, center):
        # unit is mm
        points = np.float32([[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 0]]).reshape(-1, 3)
        axisPoints, _ = cv2.projectPoints(points, rvec, t, K, (0, 0, 0, 0))
        axisPoints = axisPoints.astype(int)

        cv2.line(img, center, axisPoints[0].ravel(), (255,0,0), 3)
        cv2.line(img, center, axisPoints[1].ravel(), (0,255,0), 3)
        cv2.line(img, center, axisPoints[2].ravel(), (0,0,255), 3)

    # standard camera module 3
    camera_matrix = np.array([
        [917.1777059, 0.00000000, 323.46713801],
        [0.00000000, 926.27018107, 240.68578702],
        [0.00000000, 0.00000000, 1.00000000],
    ],dtype=np.float64)

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

    half_size = tag_size / 2.0

    object_points = np.array([
        [-half_size,  half_size, 0],
        [ half_size,  half_size, 0],
        [ half_size, -half_size, 0],
        [-half_size, -half_size, 0]
    ], dtype=np.float32)

    # AprilTag detector

    # Initialize the AprilTag detector
    options = apriltag.DetectorOptions(families='tag36h11')  # default family
    detector = apriltag.Detector(options)
    header_size = struct.calcsize("=HH")

    timeout = 0 # sets a delay for the yaw command

    yaw_set = False
    target_yaw = 0
    landed = False
    y_offset = .32 # account for april tag not aligned with box
    land_height = 2 # height to switch to landing mode
    align_height = 7 # height to start precision alignment

    while not landed:
        # print(timeout, end="\r")

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
        img = np.frombuffer(img, np.uint8).reshape((height, width))

        ## Picamera code

        # frame = picam2.capture_array()
        # if frame is None:
        #     print ("No frame received! Exiting")
        #     break
        # gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # gray_color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        ## Webots cam
        gray = img
        gray_color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        detections = detector.detect(gray)

        for det in detections:

            ## OpenCV pnp solver
            ## Estimate pose using corner points
            # image_points = det.corners.astype(np.float32)
            # success, rvec, M = cv2.solvePnP(object_points, image_points,
            #                                    camera_matrix, dist_coefficients,
            #                                    flags=cv2.SOLVEPNP_ITERATIVE)


            # create an apriltag_detection_info_t struct using your known parameters.
            fx = camera_matrix_2[0,0]
            fy = camera_matrix_2[1,1]
            camera_params = [fx, fy, x_res//2, y_res//2]

            M, init_error, final_error = detector.detection_pose(det, camera_params, tag_size)

            # if M exists (rotation matrix + translation vector)
            if M.any():
                rvec, _ = cv2.Rodrigues(M[:3,:3])
                tvec = M[:3, 3]
                M_flat = M.flatten()
                x = -M_flat[3]
                y = M_flat[7]
                z = M_flat[11]
                cx, cy = map(int, det.center)

                if use_display:
                    cv2.circle(gray_color, (cx, cy), 5, (0, 0, 255), -1)

                    ## Display X,Y offset
                    cv2.putText(gray_color, f"ID:{det.tag_id} X:{x:.2f} Y:{y:.2f} Z:{z:.2f}m",
                                (40, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    
                    ## Display rotation matrix values
                    cv2.putText(gray_color, f"{rvec[0]} \n {rvec[1]} \n {rvec[2]}",
                                (40, 70),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)

                    center = tuple([cx, cy])
                    
                    draw_axis(gray_color, rvec, tvec, camera_matrix_2, center)

                    # Draw box around tag
                    for i in range(4):
                        pt1 = tuple(det.corners[i].astype(int))
                        pt2 = tuple(det.corners[(i + 1) % 4].astype(int))
                        cv2.line(gray_color, pt1, pt2, (0, 255, 0), 2)


                if (time.time() > timeout):
                    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                    if msg:
                        hdg = msg.hdg
                        curr_yaw = hdg/180*math.pi/100
                        timeout = time.time() + .5
                        z_rot = rvec[2,0]
                        print(f"x:{-x:.3f} y:{y:.3f} desr yaw: {target_yaw:.3f} curr yaw:{curr_yaw:.3f} z rot:{z_rot:.3f}", end="\r")
                        timeout += 5
                        if z > align_height:
                            delta_h = 1 + (z - align_height) / 1.5 # logarithmically approach precision landing height
                            align(-y/2-y_offset,-x/2,delta_h,z_rot)
                        elif (z > land_height):
                            align(-y-y_offset,-x,z/3,z_rot)
                        else:
                            print("\n")
                            set_mode("LAND")
                            landed = True

        if use_display:
            cv2.imshow("AprilTag Pose", gray_color)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


    
    # Cleanup
    if use_display:
        cv2.destroyAllWindows()
    # picam2.stop()

    s.close()
    print("\nSocket closed")

# Define the distance to fly (in meters) and the direction (angle from north in degrees)
# Angle is in degrees: 0 = North, 90 = East, 180 = South, 270 = West
# target_distance = float(input("Enter target distance in meters: "))
# target_angle = float(input("Enter target angle from north in degrees (0 = North, 90 = East): "))
# target_alt = float(input("Enter target altitude (meters): "))
# loiter_time = float(input("Enter loiter time (s): "))

target_distance = 10
target_angle = 27
target_alt = 15
loiter_time = 1


# Start a connection listening to a UDP port
connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")


def set_mode(mode):
    mode_id = connection.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Mode {mode} is not supported")
        return False
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    while True:
        ack = connection.recv_match(type='HEARTBEAT', blocking=True)
        if ack.custom_mode == mode_id:
            print(f"Mode changed to {mode}")
            break
        time.sleep(0.5)

# Arm the drone
def arm_drone():
    print("Arming...")
    # Wait until armed
    while True:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1, 1, 0, 0, 0, 0, 0, 0
        )
        heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True)
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Drone is armed")
            break
        time.sleep(2)

# Take off to a specified altitude
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
        # print(f"Current altitude: {current_alt:.2f} meters")
        if current_alt >= altitude * 0.95:  # Reached 95% of target altitude
            print("Reached target altitude")
            break
        time.sleep(0.5)

# Fly a relative distance from the takeoff point (using NED coordinates)
def fly_relative_distance(distance, angle, alt):
    # Convert angle from degrees to radians
    angle_rad = math.radians(angle)
    
    # Calculate the change in North (N) and East (E) directions based on the angle and distance
    delta_north = distance * math.cos(angle_rad)  # NED north is positive north, negative south
    delta_east = distance * math.sin(angle_rad)   # NED east is positive east, negative west
    delta_down = -alt  # We want to fly at the given altitude, hence negative for downward motion
    
    print(f"Flying {distance} meters in the direction of {angle}° (North/East).")
    
    # Send the relative movement command (in NED coordinates)
    connection.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms, set to 0 as we are sending an instantaneous command
        connection.target_system, 
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # NED frame
        int(0b110111111000),  # type_mask to specify that we are setting position
        delta_north,  # North (meters)
        delta_east,   # East (meters)
        delta_down,   # Down (meters)
        0,            # velocity_x (not used)
        0,            # velocity_y (not used)
        0,            # velocity_z (not used)
        0,            # acceleration_x (not used)
        0,            # acceleration_y (not used)
        0,            # acceleration_z (not used)
        0,            # yaw (not used)
        0             # yaw_rate (not used)
    )
    
    # Monitor progress (waiting for the drone to complete the movement)
    start_time = time.time()
    while True:
        timestamp = time.time() - start_time 
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        current_alt = msg.relative_alt / 1000.0

        
        # Check if we have moved the desired distance
        distance_travelled = get_distance_meters(current_lat, current_lon, lat_takeoff, lon_takeoff)
        print(f"Distance: {distance_travelled}", end="\r")
        if distance_travelled >= distance - 1:  # Check if we've moved the target distance
            print("\nTarget distance reached.")
            break
        
        # Timeout after 30 seconds in case the drone doesn't reach the target
        if time.time() - start_time > 30:
            print("Timed out while trying to reach target distance.")
            break

        time.sleep(.5)


# Function to calculate the distance between two GPS points (Haversine formula)
def get_distance_meters(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def get_location():
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    
    # Extract latitude, longitude, and altitude from the message
    if msg:
        latitude = msg.lat / 1e7  # Convert from 1e7 to degrees
        longitude = msg.lon / 1e7  # Convert from 1e7 to degrees
        return latitude, longitude
    else:
        return None, None
    
# set yaw heading in degrees
def set_yaw(yaw):
    error = 0.2
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        1, yaw, 10, 0, 0, 0, 0, 0
    )
    yaw_rad = yaw/180*math.pi
    print(yaw_rad)
    while(True):
        # msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        msg = connection.recv_match(type='ATTITUDE', blocking=True)
        hdg = msg.yaw
        print(f"Heading: {hdg}", end="\r")
        if (hdg > yaw_rad - error) and (hdg < yaw_rad + error):
            break
        time.sleep(.5)
    print("\naligned, resuming")


def return_to_launch():
    """
    Command the drone to return to its launch (takeoff) location.
    """
    print("Returning to launch (RTL)...")
    
    # Send MAVLink RTL command (MAV_CMD_NAV_RETURN_TO_LAUNCH)
    connection.mav.command_long_send(
        connection.target_system, 
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  # RTL command
        0,  # Confirmation (set to 0)
        0,  # Param 1: Not used
        0,  # Param 2: Not used
        0,  # Param 3: Not used
        0,  # Param 4: Reserved
        0,  # Param 5: Reserved
        0,  # Param 6: Reserved
        0   # Param 7: Reserved
    )

# return to lat/lon location and align yaw
def yaw_return(lat, lon):
    print("Returning to takeoff location")
    # Send the relative movement command (in NED coordinates)
    # create return home message
    message = dialect.MAVLink_mission_item_int_message(
        target_system=connection.target_system,
        target_component=connection.target_component,
        seq=0,
        frame=dialect.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=dialect.MAV_CMD_NAV_WAYPOINT,
        current=2,
        autocontinue=0,
        param1=0,
        param2=2,
        param3=0,
        param4=0,
        x=int(lat * 1e7),
        y=int(lon * 1e7),
        z=target_alt
    )

    # send target location command to the connection
    connection.mav.send(message)

    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        dist = get_distance_meters(lat, lon, current_lat, current_lon)
        print(f"Distance to takeoff: {dist}", end="\r")
        if(dist < 1):
            break
        time.sleep(.5)
    print("\nTakeoff location reached")
    # set_yaw(180)
    # # return_to_launch()
    # print("Landing")
    # set_mode("LAND")

# Store takeoff position
lat_takeoff, lon_takeoff = None, None

lat_box, lon_box = -35.3632571, 149.1652256

# Execute the mission
set_mode("GUIDED")
arm_drone()
takeoff(target_alt)

# Now set the takeoff location
lat_takeoff, lon_takeoff = get_location()

# Now fly to the target relative location
fly_relative_distance(target_distance, target_angle, target_alt)

# Begin loiter
time.sleep(loiter_time)

#return to launch and end mission
yaw_return(lat_box, lon_box)
# yaw_return(lat_takeoff, lon_takeoff)


april_tag_land(tag_size, False)