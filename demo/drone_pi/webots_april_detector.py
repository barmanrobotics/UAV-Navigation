# Bill Wang

# Modified code from Arnab Chatterjee


import time
import cv2
import numpy as np
import apriltag
import socket
import struct
# from picamera2 import Picamera2
# from libcamera import controls

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

# Webots camera
webots_cam_matrix = np.array([
    [384.715,   0.    , 320.],
    [  0.    , 384.715, 240.],
    [  0.    ,   0.    , 1. ]
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

# --- AprilTag parameters ---
tag_size = .5  # tag side length in meters (e.g., 20 cm)

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

while True:

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

        # Estimate pose using corner points
        # image_points = det.corners.astype(np.float32)
        # success, rvec, M = cv2.solvePnP(object_points, image_points,
        #                                    camera_matrix, dist_coefficients,
        #                                    flags=cv2.SOLVEPNP_ITERATIVE)

        # First create an apriltag_detection_info_t struct using your known parameters.
        fx = 384.715
        fy = fx
        camera_params = [fx, fy, x_res//2, y_res//2]

        M, init_error, final_error = detector.detection_pose(det, camera_params, tag_size)

        if M.any():

            # R, _ = cv2.Rodrigues(rvec)
            rvec, _ = cv2.Rodrigues(M[:3,:3])
            tvec = M[:3, 3]
            M_flat = M.flatten()
            x = -M_flat[3]
            y = M_flat[7]
            z = M_flat[11]
            cx, cy = map(int, det.center)
            cv2.circle(gray_color, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(gray_color, f"ID:{det.tag_id} X:{x:.2f} Y:{y:.2f} Z:{z:.2f}m",
                        (40, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            ## Display rotation matrix values
            cv2.putText(gray_color, f"{rvec[0]} \n {rvec[1]} \n {rvec[2]}",
                        (40, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 2)

            # print(f"[ID {det.tag_id}] X: {x:.3f} cm, Y: {y:.3f} cm, Z: {z:.3f} cm", end="\r")'

            center = tuple([cx, cy])
            
            draw_axis(gray_color, rvec, tvec, webots_cam_matrix, center)

            # Draw box around tag
            for i in range(4):
                pt1 = tuple(det.corners[i].astype(int))
                pt2 = tuple(det.corners[(i + 1) % 4].astype(int))
                cv2.line(gray_color, pt1, pt2, (0, 255, 0), 2)

            

    time.sleep(.1)
    
    # Display frame (optional – comment out if headless)
    cv2.imshow("AprilTag Pose", gray_color)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cv2.destroyAllWindows()
# picam2.stop()

s.close()
print("\nSocket closed")