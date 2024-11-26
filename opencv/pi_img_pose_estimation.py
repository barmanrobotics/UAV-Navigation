import numpy as np
import cv2 as cv
import glob

# Function to load calibration data from a file
def load_calibration_data(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
        # Extract and parse the camera matrix
        camera_matrix = np.array([list(map(float, lines[1].split())),
                                   list(map(float, lines[2].split())),
                                   list(map(float, lines[3].split()))])
        # Extract and parse the distortion coefficients
        dist_coeffs = np.array(list(map(float, lines[6].split()))).reshape(-1, 1)
    return camera_matrix, dist_coeffs

# Draw X, Y, Z axes (RGB: Red, Green, Blue)
def draw(img, corners, imgpts):
    # Draw the axis on the image
    corner = tuple(corners[0].ravel().astype(int))
    imgpts = [tuple(pt.ravel().astype(int)) for pt in imgpts]
    
    img = cv.line(img, corner, tuple(imgpts[0]), (0,0,255), 5)
    img = cv.line(img, corner, tuple(imgpts[1]), (0,255,0), 5)
    img = cv.line(img, corner, tuple(imgpts[2]), (255,0,0), 5)
    return img

# Load the camera matrix and distortion coefficients
# pi_v3 file name: cal_img_data.txt
# pi_v3 wide file name: cal_img_wide_data.txt
camera_matrix, dist_coeffs = load_calibration_data("cal_img_data.txt")

# Termination criteria for corner refinement
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points for an 8x6 checkerboard
objp = np.zeros((6 * 8, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)

axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)


# Define the directory containing calibration images
dir_name = 'checkerboard_with_distance'

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
        imgpts, _ = cv.projectPoints(axis, rvecs, tvecs, camera_matrix, dist_coeffs)

        # Draw X, Y, Z axes (RGB: Red, Green, Blue)
        img = draw(img, corners2, imgpts)

        # Display the image with pose estimation
        cv.imshow('Pose Estimation', img)
        cv.waitKey(0)

cv.destroyAllWindows()
