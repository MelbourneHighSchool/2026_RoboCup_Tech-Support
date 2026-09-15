import glob
from pathlib import Path

import cv2 as cv
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
CALIBRATION_FILE = SCRIPT_DIR / "calibration_data.npz"
CHESS_IMAGES = str(SCRIPT_DIR / "chess" / "*.jpg")

# Check if calibration file already exists
if CALIBRATION_FILE.exists():
    print("Loading calibration data from file...")
    calibration_data = np.load(CALIBRATION_FILE, allow_pickle=True)
    mtx = calibration_data['mtx']
    dist = calibration_data['dist']
    rvecs = calibration_data['rvecs']
    tvecs = calibration_data['tvecs']
    print("Calibration data loaded successfully")
    print("Camera matrix:\n", mtx)
    print("Distortion coefficients:\n", dist)
else:
    print("Calibration file not found. Performing calibration...")
    
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    grid_size = (9, 6)
    objp = np.zeros((grid_size[0] * grid_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:grid_size[0], 0:grid_size[1]].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob(CHESS_IMAGES)

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, grid_size, None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            cv.drawChessboardCorners(img, grid_size, corners2, ret)

    if objpoints:
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        if ret:
            print("Calibration successful")
            print("Camera matrix:\n", mtx)
            print("Distortion coefficients:\n", dist)
            
            # Save calibration results to file
            np.savez(CALIBRATION_FILE, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
            print(f"Calibration data saved to {CALIBRATION_FILE}")
        else:
            print("Calibration failed")
    else:
        print("No corners found in any images. Check the grid_size and image paths.")

cv.destroyAllWindows()
