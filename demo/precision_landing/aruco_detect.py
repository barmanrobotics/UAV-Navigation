import cv2
import cv2.aruco as aruco
import numpy as np
import struct
import socket
import math

camera_matrix = None
dist_coefficients = None
marker_size = 10

def connect_camera(use_pi_camera=True):
    global camera_matrix, dist_coefficients
    """Connects to either the Pi Camera or the simulation camera."""
    if use_pi_camera:
        from picamera2 import Picamera2
        from libcamera import controls
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(raw={"size": (1536, 864)},main={"format": 'RGB888', "size": (x_res, y_res)}))
        picam2.start()
        picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        x_res, y_res = 640, 480
        camera_matrix = np.array([
            [917.1777059, 0.0, 323.46713801],
            [0.0, 926.27018107, 240.68578702],
            [0.0, 0.0, 1.0]
        ])
        dist_coefficients = np.zeros(5)

        return lambda: picam2.capture_array()
    else:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("127.0.0.1", 5600))
        horizontal_fov = 66*(math.pi/180) #horizontal_fov in radians for normal raspi camera module 3
        vertical_fov = 41*(math.pi/180) #vertical_fov in radians for normal raspi camera module 3
        # x_res= 1536 # pixels
        #y_res = 864 # pixels
        x_res = 640
        y_res = 480
        s_x = 6.45  # mm
        s_y = 3.63  # mm
        focal_length = 4.74 # mm
        f_x = (x_res/s_x)*focal_length 
        f_y = (y_res/s_y)*focal_length
        o_x = (x_res/2) + 0.5
        o_y = (y_res/2) + 0.5
        camera_matrix = np.array([[f_x,0,o_x],[0,f_y,o_y],[0,0,1]])
        #camera_matrix = [[3304.74173631,0.00000000,2250.15430808],[0.00000000,3317.15405795,1108.87277414],[0.00000000,0.00000000,1.00000000]]
        dist_coefficients = np.array([0.0,0.0,0.0,0.0,0.0])
        #dist_coefficients = [0.00548862,0.12807064,-0.01048389,-0.00508286,-0.27971602]

        return lambda: get_simulation_frame(s)

def get_simulation_frame(s):
    """Receives a frame from the simulation."""
    header_size = struct.calcsize("=HH")
    try:
        header = s.recv(header_size)
        if len(header) != header_size:
            return None
        width, height = struct.unpack("=HH", header)
        frame = s.recv(width * height, socket.MSG_WAITALL)
        return np.frombuffer(frame, np.uint8).reshape((height, width))
    except Exception as e:
        print(f"Error receiving simulation frame: {e}")
        return None

def detect_aruco_tags(frame):
    """Detects ArUco tags in the given frame and returns position."""
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)
    corners = np.array(corners)
        # Draw detected markers on the frame
        #ids = ids.flatten
    np_id  = np.array(ids)
    found = np.isin(0, np_id)
    print(found)
    img = frame.copy()
    if found == True and len(corners)==1:
        ids = ids.flatten()
        aruco.drawDetectedMarkers(img, corners, ids=None, borderColor=(0, 255, 0))
        for (markerCorner, markerID) in zip(corners, ids):
        
            ret = aruco.estimatePoseSingleMarkers(corners,marker_size,camera_matrix,dist_coefficients)
            (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
            x = '{:.2f}'.format(tvec[0])
            y = '{:.2f}'.format(tvec[1])
            z = '{:.2f}'.format(tvec[2])
            marker_postion = 'MARKER POSITION: x='+x+' y= '+y+' z= '+z
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            #Find the center
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
            cv2.putText(img, str(markerID), (int(topLeft[0]), int(topLeft[1]) - 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.putText(img, str(marker_postion),
                (10,50), cv2.FONT_HERSHEY_SIMPLEX,
                1, (102, 0, 255), 2)
            # Display the frame
            cv2.imshow("ArUco Tag Detection", img)
        return tvec

    return None

