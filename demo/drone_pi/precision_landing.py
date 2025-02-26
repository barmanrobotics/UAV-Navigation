import cv2 # type: ignore   
import cv2.aruco as aruco # type: ignore
import numpy as np # type: ignore   
import math
import time
from pymavlink import mavutil # type: ignore
from picamera2 import Picamera2 # type: ignore  

# Camera Configuration
x_res = 640
y_res = 480
FPS = 120

picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(
    main={"format": 'RGB888', "size": (x_res, y_res)},
    controls={"FrameRate": FPS}
)
picam2.configure(camera_config)
picam2.start()
time.sleep(2)  # Allow camera to warm up

# Camera Calibration
camera_matrix_1 = np.array([
    [917.1777059, 0.00000000, 323.46713801],
    [0.00000000, 926.27018107, 240.68578702],
    [0.00000000, 0.00000000, 1.00000000]
])

dist_coefficients_1 = np.array([
    [-4.21053311e-01],
    [6.10894559e+00],
    [1.08524770e-03],
    [-3.40935112e-02],
    [-6.30249662e+01]
])

# PID Coefficients
kp = 0.5
kd = 0.07
ki = 0.0

INITIAL_ERROR = 0.01
marker_size = 18  # cm
DESCENT_VELOCITY = 0.3  # m/s

# Velocity Control
def send_velocity(connection, ax, ay, vz):
    connection.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (not used)
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # Frame of reference (Body frame)
        0b110000000000,  # Control velocity only
        0, 0, 0,  # Position x, y, z (not used)
        0, 0, vz,  # Velocity x, y, z
        ax, ay, 0,  # Acceleration (not used)
        0, 0  # yaw, yaw_rate (not used)
    )

# Main Precision Landing Function
def precision_landing(connection):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()

    np_camera_matrix = camera_matrix_1
    np_dist_coefficients = dist_coefficients_1

    previous_error_x = 0.0
    previous_error_y = 0.0
    error_sum_x = 0.0
    error_sum_y = 0.0
    i = 1

    while True:
        # Capture frame
        img = picam2.capture_array()
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        corners = np.array(corners)
        
        if ids is not None and 0 in ids and len(corners) == 1:
            ids = ids.flatten()
            for idx, markerID in enumerate(ids):
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, np_camera_matrix, np_dist_coefficients)
                (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])

                error_x = tvec[1] / 100.0
                error_y = tvec[0] / 100.0

                p_out_x = kp * error_x
                p_out_y = kp * error_y

                current_time = time.time()
                if i == 1:
                    dt = 0.1
                    previous_error_x = error_x
                    previous_error_y = error_y
                    i = 2
                else:
                    dt = current_time - previous_time
                previous_time = current_time

                derivative_x = (error_x - previous_error_x) / dt
                derivative_y = (error_y - previous_error_y) / dt
                previous_error_x = error_x
                previous_error_y = error_y

                d_out_x = kd * derivative_x
                d_out_y = kd * derivative_y

                error_sum_x += error_x * dt
                error_sum_y += error_y * dt

                i_out_x = ki * error_sum_x
                i_out_y = ki * error_sum_y

                a_x = p_out_x + d_out_x + i_out_x
                a_y = p_out_y + d_out_y + i_out_y

                # Error Correction
                a_x = -a_x if abs(error_x) > INITIAL_ERROR else 0
                a_y = a_y if abs(error_y) > INITIAL_ERROR else 0

                # Limit acceleration
                a_x = max(min(a_x, 0.4), -0.4)
                a_y = max(min(a_y, 0.4), -0.4)

                print(f"a_y: {a_y} a_x: {a_x}")

                # Adjust descent velocity
                ar_alt = tvec[2]
                DESCENT_VELOCITY = 0.2 if ar_alt < 200 else 0.3

                send_velocity(connection, a_x, a_y, DESCENT_VELOCITY)

        else:
            print("Aruco not detected, continuing descent")       
            send_velocity(connection, 0, 0, 0.15)

# Main Entry Point
if __name__ == "__main__":
    connection = mavutil.mavlink_connection('udpin:localhost:14551')
    connection.wait_heartbeat()
    print("Connected to the vehicle")
    precision_landing(connection)
