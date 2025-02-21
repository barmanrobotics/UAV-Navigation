import cv2
import cv2.aruco as aruco
import numpy as np
import math

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
camera_matrix = [[f_x,0,o_x],[0,f_y,o_y],[0,0,1]]
#camera_matrix = [[3304.74173631,0.00000000,2250.15430808],[0.00000000,3317.15405795,1108.87277414],[0.00000000,0.00000000,1.00000000]]
dist_coefficients =[0.0,0.0,0.0,0.0,0.0]
#dist_coefficients = [0.00548862,0.12807064,-0.01048389,-0.00508286,-0.27971602]

np_camera_matrix = np.array(camera_matrix)
np_dist_coefficients = np.array(dist_coefficients)

marker_size = 20 #cm

def detect_aruco_tags(frame):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
    return corners, ids, rejected

def aruco_spec(corners, ret, markerID, frame):
    (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
    x = '{:.2f}'.format(tvec[0])
    y = '{:.2f}'.format(tvec[1])
    z = '{:.2f}'.format(tvec[2])

    marker_postion = 'MARKER POSITION: x='+x+' y= '+y+' z= '+z
    corners = np.array(corners)
    corners = corners.reshape((4, 2))
    (topLeft, topRight, bottomRight, bottomLeft) = corners
	# convert each of the (x, y)-coordinate pairs to integers
    topRight = (int(topRight[0]), int(topRight[1]))
    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    topLeft = (int(topLeft[0]), int(topLeft[1]))

    #Find the center
    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
    cY = int((topLeft[1] + bottomRight[1]) / 2.0)

    # Calculate the ground error and current altitude
    x_ang = (cX - x_res * 0.5)* horizontal_fov/x_res
    y_ang = (cY - y_res * 0.5)* vertical_fov/y_res
    ar_alt = tvec[2]

    #draw center
    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
    # draw the ArUco marker ID on the image
    cv2.putText(frame, str(markerID),
        (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (0, 255, 0), 2)

    cv2.putText(frame, str(marker_postion),
        (10,50), cv2.FONT_HERSHEY_SIMPLEX,
        1, (102, 0, 255), 2)

    return x_ang, y_ang, ar_alt
