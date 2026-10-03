#!/usr/bin/env python3
"""
Camera Calibration Script
Uses chessboard images to compute camera intrinsic parameters.
Based on OpenCV camera calibration tutorial:
https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

Usage:
    python3 calibrate_camera.py

Output:
    - Prints camera_matrix and distortion_coefficients
    - Saves results to camera_calibration.yaml
"""

import numpy as np
import cv2
import glob
import os
import yaml

# ============================================================
# CONFIGURATION — adjust these to match your chessboard
# ============================================================
# Number of INTERNAL corners (not squares!)
# For a 10x7 square chessboard, internal corners = 9x6
CHESSBOARD_HEIGHT = 9
CHESSBOARD_WIDTH = 6

# Size of one square in meters (measure your actual chessboard!)
# If unknown, set to 1.0 and tvec will be in "square units"
SQUARE_SIZE = 0.035  # 35mm = 0.035m, measured actual board

# Path to calibration images
IMAGE_PATH = "saved_camera_images/chessboard_images_new/"

# Output file path
OUTPUT_PATH = "src/asclinic_pkg/Pitt_A1/camera_calibration.yaml"

# ============================================================

def main():
    # Termination criteria for cornerSubPix
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points: (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
    # Scaled by SQUARE_SIZE so tvec is in meters
    objp = np.zeros((CHESSBOARD_HEIGHT * CHESSBOARD_WIDTH, 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_HEIGHT, 0:CHESSBOARD_WIDTH].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    # Find all jpg images (exclude aruco images)
    all_images = sorted(glob.glob(os.path.join(IMAGE_PATH, "*.jpg")))
    images = [img for img in all_images if "aruco" not in os.path.basename(img)]

    if not images:
        print(f"ERROR: No images found in {IMAGE_PATH}")
        return

    print(f"Found {len(images)} calibration images")
    print(f"Searching for {CHESSBOARD_HEIGHT}x{CHESSBOARD_WIDTH} internal corners")
    print("-" * 50)

    gray = None
    successful_images = []
    failed_images = []

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"  SKIP: Cannot read {os.path.basename(fname)}")
            failed_images.append(fname)
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, (CHESSBOARD_HEIGHT, CHESSBOARD_WIDTH), flags=flags)

        if ret:
            # Refine corner locations to sub-pixel accuracy
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)
            successful_images.append(fname)
            print(f"  OK:   {os.path.basename(fname)}")
        else:
            failed_images.append(fname)
            print(f"  FAIL: {os.path.basename(fname)} — chessboard not detected")

    print("-" * 50)
    print(f"Successfully detected: {len(successful_images)}/{len(images)} images")

    if len(successful_images) < 5:
        print("ERROR: Need at least 5 successful images for reliable calibration.")
        return

    # ---- Calibrate ----
    print("\nRunning calibration...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    # ---- Results ----
    print("\n" + "=" * 50)
    print("CALIBRATION RESULTS")
    print("=" * 50)

    print(f"\nReprojection error (RMS): {ret:.4f} pixels")
    if ret < 0.5:
        print("  -> Excellent!")
    elif ret < 1.0:
        print("  -> Good")
    else:
        print("  -> Consider recalibrating with better images")

    print(f"\nCamera Matrix:\n{camera_matrix}")
    print(f"\nDistortion Coefficients:\n{dist_coeffs}")

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    print(f"\nExtracted parameters:")
    print(f"  fx = {fx:.2f}")
    print(f"  fy = {fy:.2f}")
    print(f"  cx = {cx:.2f}")
    print(f"  cy = {cy:.2f}")
    print(f"  k1 = {dist_coeffs[0][0]:.6f}")
    print(f"  k2 = {dist_coeffs[0][1]:.6f}")
    print(f"  p1 = {dist_coeffs[0][2]:.6f}")
    print(f"  p2 = {dist_coeffs[0][3]:.6f}")
    print(f"  k3 = {dist_coeffs[0][4]:.6f}")

    # ---- Per-image reprojection error ----
    print("\nPer-image reprojection error:")
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
        print(f"  {os.path.basename(successful_images[i])}: {error:.4f}")
    mean_error = total_error / len(objpoints)
    print(f"  Mean error: {mean_error:.4f}")

    # ---- Code snippet for aruco_detector.py ----
    print("\n" + "=" * 50)
    print("COPY THIS INTO aruco_detector.py (line ~326):")
    print("=" * 50)
    print(f"self.intrinic_camera_matrix = np.array([[{fx:.2f}, 0, {cx:.2f}], [0, {fy:.2f}, {cy:.2f}], [0, 0, 1]], dtype=float)")
    print(f"self.intrinic_camera_distortion = np.array([[{dist_coeffs[0][0]:.6e}, {dist_coeffs[0][1]:.6e}, {dist_coeffs[0][2]:.6e}, {dist_coeffs[0][3]:.6e}, {dist_coeffs[0][4]:.6e}]], dtype=float)")

    # ---- Save to YAML ----
    calibration_data = {
        "image_width": int(gray.shape[1]),
        "image_height": int(gray.shape[0]),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": dist_coeffs.tolist(),
        "reprojection_error_rms": float(ret),
        "num_images_used": len(successful_images),
        "chessboard_size": [CHESSBOARD_HEIGHT, CHESSBOARD_WIDTH],
        "square_size_meters": SQUARE_SIZE,
    }

    with open(OUTPUT_PATH, "w") as f:
        yaml.dump(calibration_data, f, default_flow_style=False)
    print(f"\nCalibration saved to: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
