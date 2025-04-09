# precision_landing.py
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
from picamera2 import Picamera2
from pymavlink import mavutil
import struct
import os
import csv

def precision_land(connection):
    picam2 = Picamera2()
    x_res, y_res, FPS = 640, 480, 120

    camera_config = picam2.create_preview_configuration(
        raw={"size": (1536, 864)},
        main={"format": 'RGB888', "size": (x_res, y_res)},
        controls={"FrameRate": FPS}
    )
    picam2.configure(camera_config)
    picam2.start()
    print("Waiting for camera to start...")
    time.sleep(1)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    marker_positions = {}

    marker_size = 19.2  # cm
    marker_track = 0

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

    def save_marker_positions(marker_positions):
        for marker_id, positions in marker_positions.items():
            file_name = f"marker_{marker_id}.csv"
            with open(file_name, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["X", "Y", "Z"])
                writer.writerows(positions)
            print(f"Saved positions for marker {marker_id} in {file_name}")

    def set_flight_mode(mode):
        mode_id = connection.mode_mapping().get(mode)
        if mode_id is None:
            print(f"Unknown mode: {mode}")
            return
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
            0, 0, 0, 0, 0
        )
        while True:
            msg = connection.recv_match(type='HEARTBEAT', blocking=True)
            current_mode = mavutil.mode_string_v10(msg)
            if current_mode == mode:
                print(f"Mode changed to {mode}")
                break