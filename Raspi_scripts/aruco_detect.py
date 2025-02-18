import cv2
import cv2.aruco as aruco
import socket
import struct
import numpy as np
from picamera2 import Picamera2
from libcamera import controls
import math
import time

# Camera calibration data
camera_matrix = np.array([
    [3304.74173631, 0.00000000, 2250.15430808],
    [0.00000000, 3317.15405795, 1108.87277414],
    [0.00000000, 0.00000000, 1.00000000]
])

distortion_coefficients = np.array([
    [0.00548862],
    [0.12807064],
    [-0.01048389],
    [-0.00508286],
    [-0.27971602]
])

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(raw={"size":(2304,1296)},main={"format":'RGB888',"size": (1536,864)}))
picam2.start()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #comment this out if you don't want autofocusy
print("Waiting for camera to start")
time.sleep(2)


def detect_aruco_tags():
    # ArUco setup
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()

    horizontal_fov = 66*(math.pi/180) #horizontal_fov in radians for normal raspi camera module 3
    vertical_fov = 41*(math.pi/180) #vertical_fov in radians for normal raspi camera module 3

    ## The camera matrix is calculated for raspberry pi camera module 3 based on its specification sheet
    ## found here https://www.raspberrypi.com/documentation/accessories/camera.html

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
    #print (camera_matrix)
    #camera_matrix = [[3304.74173631,0.00000000,2250.15430808],[0.00000000,3317.15405795,1108.87277414],[0.00000000,0.00000000,1.00000000]]
    dist_coefficients =[0.0,0.0,0.0,0.0,0.0]
    #dist_coefficients = [0.00548862,0.12807064,-0.01048389,-0.00508286,-0.27971602]

    np_camera_matrix = np.array(camera_matrix)
    np_dist_coefficients = np.array(dist_coefficients)

    marker_size = 10#cm


    while True:
        # receive header
        img = picam2.capture_array()

        frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #print("I'm here")
        

        # Detect the markers in the frame
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        aruco.drawDetectedMarkers(img, corners)
        corners = np.array(corners)
        # Draw detected markers on the frame
        #ids = ids.flatten
        np_id  = np.array(ids)
        found = np.isin(0, np_id)
        print(found)
        if found == True and len(corners)==1:
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
                cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(img, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

                cv2.putText(img, str(marker_postion),
                    (10,50), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (102, 0, 255), 2)

            # draw the bounding box of the ArUCo detection
            
        # Display the frame
        cv2.imshow("ArUco Tag Detection", img)
        #print("Detected IDs:", ids)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.0084)

    # Release the video capture and close all OpenCV windows
    
    picam2.stop()
    picam2.close()
    cv2.destroyAllWindows()

# Example usage
detect_aruco_tags()
