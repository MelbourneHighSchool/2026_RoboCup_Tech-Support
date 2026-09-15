import os
import sys
from pathlib import Path

import cv2 as cv
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
CALIBRATION_FILE = SCRIPT_DIR / "calibration_data.npz"

# Check if filename argument was provided
if len(sys.argv) < 2:
    print("Usage: python undistort_image.py <FILE_NAME>")
    sys.exit(1)

input_filename = sys.argv[1]

# Load calibration data from file
if CALIBRATION_FILE.exists():
    print("Loading calibration data from file...")
    calibration_data = np.load(CALIBRATION_FILE, allow_pickle=True)
    mtx = calibration_data['mtx']
    dist = calibration_data['dist']
    rvecs = calibration_data['rvecs']
    tvecs = calibration_data['tvecs']
    print("Calibration data loaded successfully")
    
    # Load input image
    if not os.path.exists(input_filename):
        print(f"Error: Input file '{input_filename}' not found.")
        sys.exit(1)
    
    img = cv.imread(input_filename)
    if img is None:
        print(f"Error: Could not read image from '{input_filename}'.")
        sys.exit(1)
    
    h, w = img.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    # undistort
    dst = cv.undistort(img, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    
    # Generate output filename: <FILE_NAME>_undistorted.<extension>
    base_name, ext = os.path.splitext(os.path.basename(input_filename))
    output_filename = f"{base_name}_undistorted{ext}"
    
    cv.imwrite(output_filename, dst)
    print(f"Undistorted image saved to '{output_filename}'")
else:
    print(f"Error: Calibration file '{CALIBRATION_FILE}' not found.")
    print("Please run find_chess_corners.py first to generate calibration data.")
    sys.exit(1)
