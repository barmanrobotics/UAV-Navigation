import numpy as np
import glob
import cv2
import pickle
import matplotlib.pyplot as plt

# Select the img directory to calibrate
#   cal_img Directory: Images taken from Raspberry Pi v3
#   cal_img_wide Directory: Images taken from Raspberry Pi v3 wide
dir_name = 'cal_img'

def Calibrate(num_intersections_in_x=8, num_intersections_in_y=6, square_size=0.03):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    obj_points = []
    img_points = []
    
    object_points = np.zeros((num_intersections_in_x * num_intersections_in_y, 3), np.float32)
    object_points[:, :2] = np.mgrid[0:num_intersections_in_x, 0:num_intersections_in_y].T.reshape(-1, 2) * square_size

    fnames = glob.glob('./' + dir_name + '/*.jpg')
    print(f"Found {len(fnames)} images for calibration.")
    
    for fname in fnames:
        img = cv2.imread(fname)
        img_size = (img.shape[1], img.shape[0])
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (num_intersections_in_x, num_intersections_in_y), None)
        
        if ret:
            obj_points.append(object_points)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1, -1), criteria)
            img_points.append(corners)
			# Draw and display the corners
            # cv2.drawChessboardCorners(img, (6,6), corners2, ret)
            # cv2.imshow('img', img)
            # cv2.waitKey(100)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, img_size, None, None)
    img = cv2.imread(fnames[1])
    dst = cv2.undistort(img, mtx, dist, None, mtx)
    f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
    ax1.imshow(img)
    ax1.set_title('Original Image', fontsize=15)
    ax2.imshow(dst)
    ax2.set_title('Undistorted Image', fontsize=15)
    plt.show()

    # Find the mean error for validation
    mean_error = 0
    for i in range(len(obj_points)):
        imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(img_points[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error

    print( "total error: {}".format(mean_error/len(obj_points)) )

    return ret, mtx, dist, rvecs, tvecs

def save_calibration_data_txt(mtx, dist, filename=dir_name + "_data.txt"):
    with open(filename, "w") as f:
        f.write("Camera Matrix:\n")
        np.savetxt(f, mtx, fmt='%0.8f')
        f.write("\nDistortion Coefficients:\n")
        np.savetxt(f, dist, fmt='%0.8f')

if __name__ == "__main__":
    ret, mtx, dist, rvecs, tvecs = Calibrate()
    print("Return value:" + str(ret) + "\n")
    print("Camera Matrix:\n", mtx)
    print("Distortion Coefficients:\n", dist)
    save_calibration_data_txt(mtx, dist)
