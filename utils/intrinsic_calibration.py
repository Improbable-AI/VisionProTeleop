#!/usr/bin/env python3
"""
Single-step live stereo calibration script (AUTO-CAPTURE).

Usage:
    python stereo_live_calibrate_auto.py

What it does:
- Opens a video stream (stereo camera with left/right side-by-side).
- Continuously shows the live feed, split into left/right.
- When it detects a checkerboard in BOTH left and right views:
    - If enough time has passed since the last capture AND
      the pose is sufficiently different (corners moved enough),
      it automatically captures that pair.
- Once MIN_SAMPLES samples are collected, it:
    - Calibrates LEFT and RIGHT cameras separately (intrinsics + distortion).
    - Runs stereoCalibrate (R, T).
    - Saves everything to "stereo_calibration_live.npz".

Keys:
- q or ESC: quit early (no calibration if not enough samples)
"""

import cv2
import numpy as np
import time

# =======================
# CONFIG
# =======================

# Checkerboard inner corner count: (columns, rows)
CHECKERBOARD = (9, 6)

# Physical size of one checkerboard square (meters).
# Measure on your printed checkerboard, e.g., 0.024 for 24mm.
SQUARE_SIZE = 0.024

# Minimum number of good stereo samples to collect before calibrating.
MIN_SAMPLES = 20

# Which camera index to open
CAMERA_INDEX = 0

# Whether the stereo frame is side-by-side (left | right)
SIDE_BY_SIDE = True

# Optional: requested capture resolution (depends on driver support)
CAPTURE_WIDTH = 2560
CAPTURE_HEIGHT = 960

# Auto-capture parameters
MIN_CAPTURE_INTERVAL = 0.7  # seconds between auto captures
MIN_CORNER_MOVEMENT = 15.0  # pixels (mean movement vs last captured pose)


def prepare_object_points(board_size, square_size):
    """Prepare a single template of 3D points for the checkerboard pattern."""
    cols, rows = board_size
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size
    return objp


def find_corners(gray, board_size):
    """Find checkerboard corners with sub-pixel refinement."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ret, corners = cv2.findChessboardCorners(gray, board_size, flags)

    if not ret:
        return False, None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        1e-3,
    )
    corners_subpix = cv2.cornerSubPix(
        gray,
        corners,
        winSize=(11, 11),
        zeroZone=(-1, -1),
        criteria=criteria,
    )
    return True, corners_subpix


def calibrate_single_camera(objpoints, imgpoints, image_size):
    """Run mono camera calibration and return intrinsics + distortion."""
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objectPoints=objpoints,
        imagePoints=imgpoints,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None,
    )
    return ret, K, dist, rvecs, tvecs


def mean_corner_movement(corners_a, corners_b):
    """
    Compute mean Euclidean distance between two sets of corners.
    corners_a, corners_b: Nx1x2 arrays.
    """
    if corners_a is None or corners_b is None:
        return float("inf")
    if corners_a.shape != corners_b.shape:
        return float("inf")

    diff = corners_a - corners_b
    diff = diff.reshape(-1, 2)
    dists = np.linalg.norm(diff, axis=1)
    return float(np.mean(dists))


def main():
    # Prepare checkerboard 3D points (same for every capture)
    objp = prepare_object_points(CHECKERBOARD, SQUARE_SIZE)

    objpoints_left = []
    objpoints_right = []
    imgpoints_left = []
    imgpoints_right = []

    print("Opening camera...")
    cap = cv2.VideoCapture(CAMERA_INDEX)

    if CAPTURE_WIDTH is not None and CAPTURE_HEIGHT is not None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)

    if not cap.isOpened():
        print("ERROR: Could not open camera.")
        return

    print("Camera opened. Move the camera around the checkerboard.")
    print("Checkerboard will be auto-captured when visible in BOTH views,")
    print("with enough pose difference and time gap.")
    print(f"Need at least {MIN_SAMPLES} samples before calibration.")
    print("Press 'q' or ESC to quit early.\n")

    img_size_left = None
    img_size_right = None

    last_capture_time = 0.0
    last_corners_left = None
    last_corners_right = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("WARNING: Failed to read frame from camera.")
            break

        h, w = frame.shape[:2]

        if SIDE_BY_SIDE:
            w2 = w // 2
            left_frame = frame[:, :w2]
            right_frame = frame[:, w2:]
        else:
            print("SIDE_BY_SIDE=False not implemented in this script.")
            break

        gray_left = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)

        ok_left, corners_left = find_corners(gray_left, CHECKERBOARD)
        ok_right, corners_right = find_corners(gray_right, CHECKERBOARD)

        vis_left = left_frame.copy()
        vis_right = right_frame.copy()

        if ok_left:
            cv2.drawChessboardCorners(vis_left, CHECKERBOARD, corners_left, ok_left)
        if ok_right:
            cv2.drawChessboardCorners(vis_right, CHECKERBOARD, corners_right, ok_right)

        vis_combined = np.hstack((vis_left, vis_right))

        # Status text
        samples_count = len(objpoints_left)
        text = f"samples: {samples_count} / {MIN_SAMPLES}"
        color = (0, 255, 0) if samples_count >= MIN_SAMPLES else (0, 255, 255)
        cv2.putText(
            vis_combined,
            text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            color,
            2,
            cv2.LINE_AA,
        )

        if ok_left and ok_right:
            cv2.putText(
                vis_combined,
                "Checkerboard detected in BOTH views (auto-capturing).",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                vis_combined,
                "Need checkerboard visible in BOTH views.",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow("Stereo Calibration (AUTO) - left | right", vis_combined)

        key = cv2.waitKey(10) & 0xFF
        if key == 27 or key == ord("q"):  # ESC or q
            print("User requested quit.")
            break

        # AUTO-CAPTURE LOGIC
        now = time.time()
        if ok_left and ok_right:
            # Check time gap
            enough_time = (now - last_capture_time) >= MIN_CAPTURE_INTERVAL

            # Check pose difference
            movement_left = mean_corner_movement(corners_left, last_corners_left)
            movement_right = mean_corner_movement(corners_right, last_corners_right)
            movement = min(movement_left, movement_right)  # conservative

            enough_movement = movement >= MIN_CORNER_MOVEMENT

            if enough_time and enough_movement:
                objpoints_left.append(objp)
                imgpoints_left.append(corners_left)
                objpoints_right.append(objp)
                imgpoints_right.append(corners_right)

                img_size_left = (gray_left.shape[1], gray_left.shape[0])
                img_size_right = (gray_right.shape[1], gray_right.shape[0])

                last_capture_time = now
                last_corners_left = corners_left.copy()
                last_corners_right = corners_right.copy()

                print(
                    f"Auto-captured sample #{len(objpoints_left)} "
                    f"(movement ~ {movement:.1f}px)."
                )

        # Auto-calibrate once we have enough samples
        if len(objpoints_left) >= MIN_SAMPLES:
            print("\nCollected enough samples. Running calibration...")
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(objpoints_left) < MIN_SAMPLES:
        print(
            f"\nNot enough samples collected ({len(objpoints_left)} < {MIN_SAMPLES}). "
            "No calibration performed."
        )
        return

    # =======================
    # Mono calibration: LEFT
    # =======================
    print("\n=== Calibrating LEFT camera ===")
    ret_L, K_L, dist_L, rvecs_L, tvecs_L = calibrate_single_camera(
        objpoints_left, imgpoints_left, img_size_left
    )

    print(f"Left reprojection error: {ret_L}")
    print("Left intrinsic matrix K_L:\n", K_L)
    print("Left distortion coefficients:\n", dist_L.ravel())

    # =======================
    # Mono calibration: RIGHT
    # =======================
    print("\n=== Calibrating RIGHT camera ===")
    ret_R, K_R, dist_R, rvecs_R, tvecs_R = calibrate_single_camera(
        objpoints_right, imgpoints_right, img_size_right
    )

    print(f"Right reprojection error: {ret_R}")
    print("Right intrinsic matrix K_R:\n", K_R)
    print("Right distortion coefficients:\n", dist_R.ravel())

    # =======================
    # Stereo calibration (R, T)
    # =======================
    print("\n=== Stereo calibration (R, T) ===")
    criteria = (
        cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS,
        100,
        1e-5,
    )

    rms, K_L_st, dist_L_st, K_R_st, dist_R_st, R, T, E, F = cv2.stereoCalibrate(
        objectPoints=objpoints_left,
        imagePoints1=imgpoints_left,
        imagePoints2=imgpoints_right,
        cameraMatrix1=K_L,
        distCoeffs1=dist_L,
        cameraMatrix2=K_R,
        distCoeffs2=dist_R,
        imageSize=img_size_left,
        criteria=criteria,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )

    print(f"Stereo RMS reprojection error: {rms}")
    print("R (rotation from left to right):\n", R)
    print("T (translation from left to right):\n", T)

    # =======================
    # Save everything
    # =======================
    out_file = "stereo_calibration_live.npz"
    np.savez(
        out_file,
        K_L=K_L,
        dist_L=dist_L,
        K_R=K_R,
        dist_R=dist_R,
        R=R,
        T=T,
        image_size_left=img_size_left,
        image_size_right=img_size_right,
        reproj_error_left=ret_L,
        reproj_error_right=ret_R,
        stereo_rms=rms,
    )

    print(f"\nCalibration saved to {out_file}")
    print("You now have K_L and K_R (intrinsic matrices) and distortion terms.")


if __name__ == "__main__":
    main()