import cv2
import socket
import struct
import cv2.aruco as aruco
import numpy as np
from drone_control import connect_drone, arm_disarm_drone, send_velocity
from aruco_detection import detect_aruco_tags, aruco_spec

# Connect to the drone
connection = connect_drone()
arm_disarm_drone(connection, 1)

# Connect to the Webots simulation
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5600))

header_size = struct.calcsize("=HH")
while True:
    header = s.recv(header_size)
    if len(header) != header_size:
        print("Header size mismatch")
        break
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

    corners, ids, _ = detect_aruco_tags(frame)
    if ids is not None:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
            ret = aruco.estimatePoseSingleMarkers(corners, 20, np.eye(3), np.zeros(5))
            x_ang, y_ang, ar_alt = aruco_spec(corners, ret, markerID, frame)
            send_velocity(connection, -0.1 if y_ang > 0.02 else 0.1, 0.1 if x_ang > 0.02 else -0.1, 0.1)
    else:
        send_velocity(connection, 0, 0, 0.02)
    
    cv2.imshow("ArUco Tag Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

s.close()
cv2.destroyAllWindows()
