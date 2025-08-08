# Bill Wang

# Modified code from Arnab Chatterjee
# Added precision landing



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



# Start a connection listening to a UDP port
connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")

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


                # Precision landing logic:
                # Once the AprilTag is detected, start to align yaw and move xy to center while 
                # descending so the tag so that it does not go out of frame while aligning the yaw. 
                # After a certain altitude is reached, become more precise. Descend slower 
                # and go for final alignment.
                # Under a final altitude threshold (lowest the drone can safely go while 
                # still having the whole tag visible), set to land mode
                if (time.time() > timeout):
                    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                    if msg:
                        hdg = msg.hdg
                        curr_yaw = hdg/180*math.pi/100
                        timeout = time.time() + .5
                        z_rot = rvec[2,0]
                        print(f"x:{-x:.3f} y:{y:.3f} curr yaw:{curr_yaw:.3f} z rot:{z_rot:.3f}", end="\r")
                        timeout += 5
                        # if z > align_height:
                        #     delta_h = 1 + (z - align_height) / 1.5 # Calculate how much to descend, without going below precision landing height
                        #     align(-y / 2 - y_offset, -x / 2, delta_h, z_rot)
                        # elif (z > land_height):
                        if (z > land_height):
                            align(-y - y_offset, -x, z / 3, z_rot) # y_offset to account for box position
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
    
april_tag_land(tag_size, False)