#
# calibrate - Calibrate a camera using ChArUco boards
#
# fp
# January 2025
#

import numpy as np
import cv2 as cv
import cv2.aruco as aruco
import glob
import os

# debug path crap
print("Current Working Directory:", os.getcwd())

# Define the ArUco dictionary (Use a larger dictionary for safety)
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

# Define ChArUco board parameters
squaresX = 19   # Number of squares along X (horizontal)
squaresY = 11   # Number of squares along Y (vertical)
squareLength = 0.04  # Checker size in meters (40mm)
markerLength = 0.03  # Marker size in meters (30mm)

# Create the ChArUco board object
board = aruco.CharucoBoard((squaresX, squaresY), squareLength, markerLength, dictionary)

# Save the board as an image
board_image = board.generateImage((2000, 1000))  # Adjust resolution as needed
cv.imwrite("src/img/charuco_board.png", board_image)

# Set ArUco detector parameters
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

# Lists to store detected points for calibration
all_charuco_corners = []
all_charuco_ids = []
image_size = None

# Get list of calibration images
images = glob.glob('src/img/*.jpeg')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Step 1: Detect ArUco markers
    corners, ids, rejected = detector.detectMarkers(gray)

    # Visualize detected markers
    img_with_markers = img.copy()
    if ids is not None:
        # Resize for visualization (scales down to 1080p while preserving aspect ratio)
        scale_factor = 1080 / img.shape[0]  # Scale to 1080px height
        scaled_img = cv.resize(img, None, fx=scale_factor, fy=scale_factor, interpolation=cv.INTER_AREA)

    # Draw markers on the scaled image
    aruco.drawDetectedMarkers(scaled_img, corners, ids)
    
    cv.imshow("Detected ArUco Markers", img_with_markers)
    cv.waitKey(500)  # Pause to inspect results

    # Step 2: Interpolate ChArUco corners (subpixel refinement)
    if ids is not None and len(ids) > 0:
        ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(corners, ids, gray, board)

        # Step 3: If enough corners are found, store them
        if ret > 4:  # At least 4 points needed for calibration
            all_charuco_corners.append(charuco_corners)
            all_charuco_ids.append(charuco_ids)

            # Save image size (only once)
            if image_size is None:
                image_size = gray.shape[::-1]

            scale_factor = 1080 / img.shape[0]  # Scale to 1080px height
            scaled_img = cv.resize(img, None, fx=scale_factor, fy=scale_factor, interpolation=cv.INTER_AREA)

            # Scale detected corner locations to match resized image
            scaled_corners = charuco_corners.copy()  # Copy to avoid modifying original data
            scaled_corners[:, 0, 0] *= scale_factor  # Scale x-coordinates
            scaled_corners[:, 0, 1] *= scale_factor  # Scale y-coordinates

            # Draw chessboard corners on the **scaled image**
            cv.drawChessboardCorners(scaled_img, (squaresX-1, squaresY-1), scaled_corners, True)

            cv.imshow("Scaled Chessboard Corners", scaled_img)
            cv.waitKey(0)


cv.destroyAllWindows()

# Step 4: Perform Camera Calibration
if len(all_charuco_corners) > 0:
    print(f"Found {len(all_charuco_corners)} valid ChArUco detections for calibration.")

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
        charucoCorners=all_charuco_corners,
        charucoIds=all_charuco_ids,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None
    )

    # Print results
    print("Camera Matrix:\n", camera_matrix)
    print("Distortion Coefficients:\n", dist_coeffs)
else:
    print("❌ Not enough ChArUco detections for calibration.")