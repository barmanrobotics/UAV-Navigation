import cv2
import numpy as np
from picamera2 import Picamera2

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

# Replacement function for estimatePoseSingleMarkers
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([
        [-marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0]
    ], dtype=np.float32)
    
    rvecs, tvecs, trash = [], [], []
    for c in corners:
        _, rvec, tvec = cv2.solvePnP(marker_points, c, mtx, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(rvec)
        tvecs.append(tvec)
        trash.append(None)  # Placeholder for compatibility
    return rvecs, tvecs, trash

# Draw the coordinate axes on the image
def draw_axes(img, corners, rvec, tvec, camera_matrix, distortion_coefficients):
    axis = np.float32([[0.03, 0, 0], [0, 0.03, 0], [0, 0, -0.03]]).reshape(-1, 3)
    imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, distortion_coefficients)
    corner = tuple(corners[0][0].astype(int))
    img = cv2.line(img, corner, tuple(imgpts[0].ravel().astype(int)), (255, 0, 0), 3)  # X-axis in red
    img = cv2.line(img, corner, tuple(imgpts[1].ravel().astype(int)), (0, 255, 0), 3)  # Y-axis in green
    img = cv2.line(img, corner, tuple(imgpts[2].ravel().astype(int)), (0, 0, 255), 3)  # Z-axis in blue
    return img

# Main function
if __name__ == "__main__":
    # Define ArUco dictionary and detector parameters
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detectorParams = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)

    # Initialize the Raspberry Pi Camera V3 using Picamera2
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
    picam2.start()

    print("Press 'q' to quit.")

    while True:
        # Capture a frame
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        marker_corners, marker_ids, rejected_candidates = detector.detectMarkers(gray)

        if marker_ids is not None:
            # Draw markers
            cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)

            # Estimate pose for each detected marker using the new function
            rvecs, tvecs, _ = my_estimatePoseSingleMarkers(
                marker_corners, 0.03, camera_matrix, distortion_coefficients
            )

            for i in range(len(marker_ids)):
                rvec = rvecs[i]
                tvec = tvecs[i]

                # Draw axes
                frame = draw_axes(frame, marker_corners[i], rvec, tvec, camera_matrix, distortion_coefficients)

                # Display rotation and translation vectors
                print(f"Marker ID: {marker_ids[i][0]}")
                print(f"Rotation Vector:\n{rvec}")
                print(f"Translation Vector:\n{tvec}\n")

        # Display the frame
        cv2.imshow('ArUco Pose Estimation', frame)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()
