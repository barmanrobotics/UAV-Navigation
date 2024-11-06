import cv2
import cv2.aruco as aruco
import socket
import struct
import numpy as np
import math

# connect to WebotsArduVehicle
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5599))



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
    dist_coefficients =[0.0,0.0,0.0,0.0,0.0]

    np_camera_matrix = np.array(camera_matrix)
    np_dist_coefficients = np.array(dist_coefficients)

    marker_size = 20 #cm

    header_size = struct.calcsize("=HH")

    while True:
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

        # Convert the frame to grayscale
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # Detect the markers in the frame
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        aruco.drawDetectedMarkers(frame, corners)
        corners = np.array(corners)
        # Draw detected markers on the frame
        #ids = ids.flatten
        ids = ids.flatten()
        if ids is not None:
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
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(frame, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

                cv2.putText(frame, str(marker_postion),
                    (10,50), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (102, 0, 255), 2)

            # draw the bounding box of the ArUCo detection
            
        # Display the frame
        cv2.imshow("ArUco Tag Detection", frame)
        #print("Detected IDs:", ids)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and close all OpenCV windows
    s.release()
    cv2.destroyAllWindows()

# Example usage
detect_aruco_tags()
