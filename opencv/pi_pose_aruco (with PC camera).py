import cv2
import cv2.aruco as aruco
import numpy as np

# Function to load calibration data from a file
def load_calibration_data(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
        camera_matrix = np.array([list(map(float, lines[1].split())),
                                   list(map(float, lines[2].split())),
                                   list(map(float, lines[3].split()))])
        dist_coeffs = np.array(list(map(float, lines[6].split()))).reshape(-1, 1)
    return camera_matrix, dist_coeffs

# Draw the coordinate axes on the image
def draw_axes(img, corners, imgpts):
    corner = tuple(corners[0].ravel().astype(int))
    imgpts = [tuple(pt.ravel().astype(int)) for pt in imgpts]
    img = cv2.line(img, corner, imgpts[0], (0, 0, 255), 3)  # X-axis
    img = cv2.line(img, corner, imgpts[1], (0, 255, 0), 3)  # Y-axis
    img = cv2.line(img, corner, imgpts[2], (255, 0, 0), 3)  # Z-axis
    return img

# Main function
if __name__ == "__main__":
    # Load calibration data
    camera_matrix, dist_coeffs = load_calibration_data("cal_img_data.txt")
    axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)

    # Define ArUco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector_params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, detector_params)

    # Open camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    marker_size = 3  # Real-world marker size

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        marker_corners, marker_ids, _ = detector.detectMarkers(gray)

        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)

            for i, corners in enumerate(marker_corners):
                obj_points = np.array([[0, 0, 0],
                                       [marker_size, 0, 0],
                                       [marker_size, marker_size, 0],
                                       [0, marker_size, 0]], dtype=np.float32)

                # Estimate pose
                corners = corners.reshape(-1, 2)
                ret, rvec, tvec = cv2.solvePnP(obj_points, corners, camera_matrix, dist_coeffs)

                if ret:
                    # Draw axes
                    imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
                    frame = draw_axes(frame, corners, imgpts)

                    # Print pose
                    print(f"Marker ID: {marker_ids[i][0]}")
                    print(f"Rotation Vector:\n{rvec}")
                    print(f"Translation Vector:\n{tvec}\n")

        # Display frame
        cv2.imshow('ArUco Pose Estimation', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
