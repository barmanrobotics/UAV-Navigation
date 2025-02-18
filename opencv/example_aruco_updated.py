#!/usr/bin/env python3

#
# An example script that receives images from a WebotsArduVehicle on port 5599 
# and displays them overlayed with any ArUco markers using OpenCV.
# Requires opencv-python (`pip3 install opencv-python`)
#

import cv2
import socket
import struct
import numpy as np
import math
import cv2.aruco as aruco

# connect to WebotsArduVehicle
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5599))

# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

horizontal_fov = 66*(math.pi*180) #horizontal_fov in radians for normal raspi camera module 3
vertical_fov = 41*(math.pi*180) #vertical_fov in radians for normal raspi camera module 3

found_count = 0
notfound_count = 0

## The camera matrix is calculated for raspberry pi camera module 3 based on its specification sheet
## found here https://www.raspberrypi.com/documentation/accessories/camera.html

x_res= 1536
y_res = 864
s_x = 6.45
s_y = 3.63
focal_length = 4.74
f_x = (x_res/s_x)*focal_length
f_y = (y_res/s_y)*focal_length
o_x = (x_res/2) + 0.5
o_y = (y_res/2) + 0.5
camera_matrix = [[f_x,0,o_x],[0,f_y,o_y],[0,0,1]]
dist_coefficients =[0.0,0.0,0.0,0.0,0.0]

np_camera_matrix = np.array(camera_matrix)
np_dist_coefficients = np.array(dist_coefficients)

marker_size = 20


header_size = struct.calcsize("=HH")
while True:
    # receive header
    header = s.recv(header_size)
    if len(header) != header_size:
        print("Header size mismatch")
        break

    # parse header
    width, height = struct.unpack("=HH", header)



    # for CV applications we may want camera intrinsics such as focal length: 
    # https://stackoverflow.com/questions/61555182/webot-camera-default-parameters-like-pixel-size-and-focus
    # cam_focal_length = 2 * np.arctan(np.tan(cam_fov * 0.5) / (cam_width / cam_height))

    # receive image
    bytes_to_read = width * height
    img = bytes()
    while len(img) < bytes_to_read:
        img += s.recv(min(bytes_to_read - len(img), 4096))

    # convert to numpy array
    img = np.frombuffer(img, np.uint8).reshape((height, width))
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # detect ArUco markers
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    corners, ids, rejected = detector.detectMarkers(img)


    if len(corners) > 0:
        # flatten the ArUco IDs list
        
        ids = ids.flatten()
        # loop over the detected ArUCo corners

        ret = aruco.estimatePoseSingleMarkers(corners,marker_size,np_camera_matrix,np_dist_coefficients)
        (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])

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

            # draw the bounding box of the ArUCo detection
            cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)
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
                (10,50), cv2.FONT_HERSHEY_SIMPLEX,
                1, (102, 0, 255), 2)

    # display image
    cv2.imshow("image", img)
    if cv2.waitKey(1) == ord("q"):
        break

s.close()