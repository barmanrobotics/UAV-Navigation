import csv
import os
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

#wide angle camera module 3
camera_matrix_2 = np.array(
[[533.19512447, 0.0, 345.74213966],
 [  0.0, 532.58952811, 223.34142568],
 [  0.0, 0.0, 1.0]])
 
dist_coefficients_2 = np.array(
[[ 0.01978526],[-0.49556434],[-0.00970043],[0.0188091],[1.46700603]])

def get_unique_filename(base_name):
    counter = 1
    file_name = f"{base_name}.csv"
    while os.path.isfile(file_name):
        file_name = f"{base_name}_{counter}.csv"
        counter += 1
    return file_name

def save_marker_positions(marker_positions):
    for marker_id, positions in marker_positions.items():
        file_name = get_unique_filename(f"marker_{marker_id}")
        with open(file_name, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["X", "Y", "Z"])  # Write header
            writer.writerows(positions)
        print(f"Saved positions for marker {marker_id} in {file_name}")

def detect_aruco_tags():
    # ArUco setup
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    
    marker_size = 8  # cm
    header_size = struct.calcsize("=HH")
    marker_positions = {}

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
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            
            # Detect the markers in the frame
            corners, ids, _ = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
            
            if ids is not None:
                ids = ids.flatten()
                aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))
                
                for markerCorner, markerID in zip(corners, ids):
                    if markerID==0:
                        marker_size=20
                    if markerID==5:
                        marker_size=8
                    if markerID==8:
                        marker_size=15
                    
                    ret = aruco.estimatePoseSingleMarkers(markerCorner, marker_size, camera_matrix_2, dist_coefficients_2)
                    (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                    x, y, z = -tvec[1], tvec[0], tvec[2]
                    
                    if markerID not in marker_positions:
                        marker_positions[markerID] = []
                    marker_positions[markerID].append([x, y, z])

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

# Example usage
aruco_positions = detect_aruco_tags()
