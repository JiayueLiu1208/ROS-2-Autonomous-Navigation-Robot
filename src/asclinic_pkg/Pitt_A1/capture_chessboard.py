#!/usr/bin/env python3
"""
Interactive chessboard image capture for camera calibration.

Usage:
    python3 capture_chessboard.py

Controls:
    SPACE  - Capture and save current frame (only if chessboard detected)
    Q/ESC  - Quit

Images are saved to saved_camera_images/chessboard_images_new/
"""

import cv2
import numpy as np
import os

# ============================================================
# CONFIGURATION
# ============================================================
SAVE_DIR = "saved_camera_images/chessboard_images_new/"
CHESSBOARD_HEIGHT = 9  # internal corners
CHESSBOARD_WIDTH = 6

CAMERA_DEVICE = 0
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080
# ============================================================

def main():
    os.makedirs(SAVE_DIR, exist_ok=True)

    # Count existing images to continue numbering
    existing = [f for f in os.listdir(SAVE_DIR) if f.endswith(".jpg")]
    img_count = len(existing)

    cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 0)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera opened: {actual_w}x{actual_h}")
    print(f"Save directory: {SAVE_DIR}")
    print(f"Looking for {CHESSBOARD_HEIGHT}x{CHESSBOARD_WIDTH} internal corners")
    print("---")
    print("SPACE = capture | Q/ESC = quit")
    print("---")

    # For corner refinement
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Coverage tracking: divide frame into 3x3 grid
    grid_rows, grid_cols = 3, 3
    coverage = np.zeros((grid_rows, grid_cols), dtype=int)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        display = frame.copy()

        # Draw 3x3 grid overlay
        h, w = display.shape[:2]
        for r in range(1, grid_rows):
            y = r * h // grid_rows
            cv2.line(display, (0, y), (w, y), (100, 100, 100), 1)
        for c in range(1, grid_cols):
            x = c * w // grid_cols
            cv2.line(display, (x, 0), (x, h), (100, 100, 100), 1)

        # Show coverage count in each cell
        for r in range(grid_rows):
            for c in range(grid_cols):
                cx = c * w // grid_cols + w // (2 * grid_cols)
                cy = r * h // grid_rows + 25
                color = (0, 255, 0) if coverage[r, c] >= 2 else (0, 165, 255) if coverage[r, c] >= 1 else (0, 0, 255)
                cv2.putText(display, str(coverage[r, c]), (cx - 10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Try to detect chessboard
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        found, corners = cv2.findChessboardCorners(gray, (CHESSBOARD_HEIGHT, CHESSBOARD_WIDTH), flags=flags)

        chessboard_center = None
        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display, (CHESSBOARD_HEIGHT, CHESSBOARD_WIDTH), corners2, found)

            # Compute chessboard center
            center = corners2.mean(axis=0)[0]
            chessboard_center = center
            cv2.circle(display, (int(center[0]), int(center[1])), 10, (255, 0, 255), -1)

            status_text = f"DETECTED - Press SPACE to capture (saved: {img_count})"
            cv2.putText(display, status_text, (10, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            status_text = f"No chessboard found (saved: {img_count})"
            cv2.putText(display, status_text, (10, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Resize for display
        display_resized = cv2.resize(display, (960, 540))
        cv2.imshow("Chessboard Capture", display_resized)

        key = cv2.waitKey(30) & 0xFF

        if key == ord(' ') and found:
            img_count += 1
            fname = os.path.join(SAVE_DIR, f"calib_{img_count:03d}.jpg")
            cv2.imwrite(fname, frame)  # Save original, not the annotated one
            print(f"  Saved: {fname}")

            # Update coverage grid
            if chessboard_center is not None:
                gc = int(chessboard_center[0] / (w / grid_cols))
                gr = int(chessboard_center[1] / (h / grid_rows))
                gc = min(gc, grid_cols - 1)
                gr = min(gr, grid_rows - 1)
                coverage[gr, gc] += 1

            # Print coverage summary
            print(f"  Coverage grid (target >=2 per cell):\n{coverage}")

        elif key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

    print(f"\nDone. Total images saved: {img_count}")
    print(f"Coverage:\n{coverage}")
    total_cells = grid_rows * grid_cols
    covered = np.sum(coverage >= 1)
    well_covered = np.sum(coverage >= 2)
    print(f"Cells with >=1 image: {covered}/{total_cells}")
    print(f"Cells with >=2 images: {well_covered}/{total_cells}")
    if well_covered < total_cells:
        print("TIP: Try to get at least 2 images per grid cell for best results.")


if __name__ == "__main__":
    main()
