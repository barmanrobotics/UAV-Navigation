import numpy as np
import cv2 as cv
import glob

# Define directory for calibration images
dir_name = 'checkerboard_with_distance'

# Given intrinsic camera matrix and distortion coefficients
camera_matrix = np.array([[1.92425010e+03, 0.00000000e+00, 2.32528385e+03],
                          [0.00000000e+00, 1.93361262e+03, 1.31522043e+03],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist_coeffs = np.array([[-0.07092664, 0.12813373, 0.00166277, 0.00033397, -0.07259955]])

# Termination criteria for corner refinement
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points for an 8x6 checkerboard
objp = np.zeros((6 * 8, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2) * 30 

# Load images from the specified directory
fnames = glob.glob('./' + dir_name + '/*.jpg')
print(f"Found {len(fnames)} images for pose estimation.")

# Process each image to estimate the pose
for fname in fnames:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv.findChessboardCorners(gray, (8, 6), None)

    if ret:
        # Refine the corner locations
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # Estimate the pose using solvePnP
        ret, rvecs, tvecs = cv.solvePnP(objp, corners2, camera_matrix, dist_coeffs)

        # Print the pose values (rotation and translation vectors)
        print(f"Pose for {fname}:")
        print("Rotation Vector:\n", rvecs)
        print("Translation Vector:\n", tvecs)

        # Optionally, draw the axes on the image
        axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)
        imgpts, _ = cv.projectPoints(axis, rvecs, tvecs, camera_matrix, dist_coeffs)

        # Draw the axis on the image
        corner = tuple(corners2[0].ravel().astype(int))
        imgpts = [tuple(pt.ravel().astype(int)) for pt in imgpts]

        # Draw X, Y, Z axes (RGB: Red, Green, Blue)
        img = cv.line(img, corner, imgpts[0], (255, 0, 0), 5)  # X-axis
        img = cv.line(img, corner, imgpts[1], (0, 255, 0), 5)  # Y-axis
        img = cv.line(img, corner, imgpts[2], (0, 0, 255), 5)  # Z-axis

        # Display the image with pose estimation
        cv.imshow('Pose Estimation', img)
        cv.waitKey(0)

cv.destroyAllWindows()
