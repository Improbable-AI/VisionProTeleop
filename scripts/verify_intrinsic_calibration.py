#!/usr/bin/env python3
"""
Verify intrinsic calibration by detecting a 2x2 ArUco tag board and measuring
3D distances between tag centers.

This script:
1. Loads intrinsic calibration from JSON
2. Detects 4 ArUco tags in a 2x2 pattern
3. Solves PnP for each tag to get 3D poses
4. Computes distances between tag centers in 3D space
5. Compares measured distances to ground truth (from printed board)
"""

import cv2
import numpy as np
import argparse
import json
from pathlib import Path


def load_calibration(calib_path: str, camera: str = "left"):
    """Load intrinsic calibration from JSON file."""
    with open(calib_path, 'r') as f:
        data = json.load(f)
    
    # Handle both stereo and mono calibration formats
    if camera in data:
        # Stereo format
        cam_data = data[camera]
        mtx = np.array(cam_data["intrinsic_matrix"])
        dist = np.array(cam_data["distortion_coefficients"])
    elif "intrinsic_matrix" in data:
        # Mono format
        mtx = np.array(data["intrinsic_matrix"])
        dist = np.array(data["distortion_coefficients"])
    else:
        raise ValueError(f"Could not find calibration data in {calib_path}")
    
    return mtx, dist


def detect_aruco_tags(image, dictionary_id=cv2.aruco.DICT_4X4_50):
    """Detect ArUco tags in an image."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    corners, ids, rejected = detector.detectMarkers(gray)
    
    return corners, ids


def solve_tag_pose(corners, tag_size, camera_matrix, dist_coeffs):
    """
    Solve PnP to get the pose of a single ArUco tag.
    Returns rotation vector, translation vector, and tag center in 3D.
    """
    # Define object points for the tag (centered at origin)
    half_size = tag_size / 2
    obj_points = np.array([
        [-half_size, half_size, 0],   # top-left
        [half_size, half_size, 0],    # top-right
        [half_size, -half_size, 0],   # bottom-right
        [-half_size, -half_size, 0],  # bottom-left
    ], dtype=np.float32)
    
    # Solve PnP
    success, rvec, tvec = cv2.solvePnP(
        obj_points,
        corners.reshape(-1, 2),
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE  # Optimized for square markers
    )
    
    if not success:
        return None, None, None
    
    # Tag center in camera frame is just the translation vector
    center_3d = tvec.flatten()
    
    return rvec, tvec, center_3d


def compute_distances(tag_centers):
    """
    Compute all pairwise distances between tag centers.
    Returns dict of distances.
    """
    distances = {}
    tag_ids = list(tag_centers.keys())
    
    for i, id1 in enumerate(tag_ids):
        for id2 in tag_ids[i+1:]:
            dist = np.linalg.norm(tag_centers[id1] - tag_centers[id2])
            distances[(id1, id2)] = dist
    
    return distances


def compute_planeness(tag_centers):
    """
    Compute how well the tag centers lie on a single plane.
    
    Uses SVD to fit a plane to the points and returns:
    - rmse: Root mean squared error of distances from the fitted plane (in meters)
    - max_dev: Maximum deviation from the plane (in meters)
    - normal: The plane normal vector
    
    Lower RMSE = more planar = better calibration / flatter surface.
    
    For 4 coplanar points on a flat printed board:
    - RMSE < 1mm: Excellent
    - RMSE < 3mm: Good  
    - RMSE < 5mm: Acceptable
    - RMSE > 5mm: Poor (check calibration or paper flatness)
    """
    if len(tag_centers) < 3:
        return None, None, None
    
    # Stack points into matrix
    points = np.array(list(tag_centers.values()))
    
    # Compute centroid
    centroid = np.mean(points, axis=0)
    
    # Center the points
    centered = points - centroid
    
    # SVD to find the best-fit plane
    # The plane normal is the singular vector with smallest singular value
    U, S, Vt = np.linalg.svd(centered)
    
    # Normal vector (last row of Vt corresponds to smallest singular value)
    normal = Vt[-1]
    
    # Compute signed distances from each point to the plane
    # Plane equation: normal · (point - centroid) = 0
    distances = np.dot(centered, normal)
    
    # RMSE and max deviation
    rmse = np.sqrt(np.mean(distances**2))
    max_dev = np.max(np.abs(distances))
    
    return rmse, max_dev, normal


def get_expected_distances(tag_size_m, margin_m, tag_ids=[0, 1, 2, 3], cols=2, rows=2):
    """
    Compute expected distances based on board geometry.
    
    For a grid of cols x rows tags:
    - Adjacent tags (horizontal/vertical) are center_dist apart
    - Diagonal tags are sqrt(dx^2 + dy^2) apart
    
    Args:
        tag_size_m: Size of each tag in meters
        margin_m: Margin between tags in meters
        tag_ids: List of tag IDs in row-major order
        cols: Number of columns
        rows: Number of rows
    """
    center_dist = tag_size_m + margin_m  # Center-to-center distance for adjacent tags
    
    expected = {}
    
    # Build position map: tag_id -> (col, row)
    positions = {}
    for idx, tag_id in enumerate(tag_ids[:cols * rows]):
        col = idx % cols
        row = idx // cols
        positions[tag_id] = (col, row)
    
    # Compute pairwise distances based on grid positions
    tag_list = list(positions.keys())
    for i, id1 in enumerate(tag_list):
        for id2 in tag_list[i+1:]:
            pos1 = positions[id1]
            pos2 = positions[id2]
            
            # Grid distance in units of center_dist
            dx = abs(pos1[0] - pos2[0])
            dy = abs(pos1[1] - pos2[1])
            
            # Euclidean distance
            dist = center_dist * np.sqrt(dx**2 + dy**2)
            expected[(id1, id2)] = dist
    
    return expected


def get_tag_image_center(corners, ids, tag_id):
    """Get the 2D image center of a tag given its corners."""
    if ids is None:
        return None
    for i, id_arr in enumerate(ids):
        if id_arr[0] == tag_id:
            # Average of 4 corners
            center = np.mean(corners[i][0], axis=0)
            return tuple(center.astype(int))
    return None


def draw_verification_overlay(image, corners, ids, tag_centers, measured_distances, expected_distances, camera_matrix, dist_coeffs, tag_size):
    """Draw detection results and distance measurements on image."""
    vis = image.copy()
    
    # Draw detected markers
    if ids is not None and len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)
        
        # Draw axes for each tag
        for i, corner in enumerate(corners):
            tag_id = ids[i][0]
            if tag_id in tag_centers:
                rvec, tvec, _ = solve_tag_pose(corner, tag_size, camera_matrix, dist_coeffs)
                if rvec is not None:
                    cv2.drawFrameAxes(vis, camera_matrix, dist_coeffs, rvec, tvec, tag_size * 0.5)
    
    # Draw lines between detected tag centers with distance labels
    for pair, measured in measured_distances.items():
        # Get 2D image positions of both tags
        pt1 = get_tag_image_center(corners, ids, pair[0])
        pt2 = get_tag_image_center(corners, ids, pair[1])
        
        if pt1 is not None and pt2 is not None:
            # Find expected distance for color coding
            expected = None
            for exp_pair, exp_dist in expected_distances.items():
                if set(pair) == set(exp_pair):
                    expected = exp_dist
                    break
            
            # Color based on error
            if expected is not None:
                error_pct = abs(measured - expected) / expected * 100
                if error_pct < 1:
                    color = (0, 255, 0)  # Green - excellent
                elif error_pct < 3:
                    color = (0, 255, 255)  # Yellow - good
                elif error_pct < 5:
                    color = (0, 165, 255)  # Orange - acceptable
                else:
                    color = (0, 0, 255)  # Red - poor
            else:
                color = (255, 255, 255)  # White - unknown
            
            # Draw line between tags
            cv2.line(vis, pt1, pt2, color, 2)
            
            # Draw distance label at midpoint
            mid_pt = ((pt1[0] + pt2[0]) // 2, (pt1[1] + pt2[1]) // 2)
            label = f"{measured*1000:.1f}mm"
            
            # Add background for readability
            (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            cv2.rectangle(vis, (mid_pt[0] - 2, mid_pt[1] - text_h - 2), 
                         (mid_pt[0] + text_w + 2, mid_pt[1] + 2), (0, 0, 0), -1)
            cv2.putText(vis, label, mid_pt, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    
    # Draw distance measurements text overlay
    y_offset = 30
    cv2.putText(vis, "Distance Verification (mm):", (10, y_offset), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    y_offset += 30
    
    for pair, measured in measured_distances.items():
        # Find expected distance
        expected = None
        for exp_pair, exp_dist in expected_distances.items():
            if set(pair) == set(exp_pair):
                expected = exp_dist
                break
        
        if expected is not None:
            error = abs(measured - expected) * 1000  # Convert to mm
            error_pct = abs(measured - expected) / expected * 100
            
            # Color based on error
            if error_pct < 1:
                color = (0, 255, 0)  # Green - excellent
            elif error_pct < 3:
                color = (0, 255, 255)  # Yellow - good
            elif error_pct < 5:
                color = (0, 165, 255)  # Orange - acceptable
            else:
                color = (0, 0, 255)  # Red - poor
            
            text = f"{pair[0]}-{pair[1]}: {measured*1000:.1f}mm (exp: {expected*1000:.1f}mm, err: {error:.1f}mm / {error_pct:.1f}%)"
            cv2.putText(vis, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            y_offset += 22
    
    return vis


def run_verification(
    camera_id: int,
    calib_file: str,
    config_file: str = None,
    tag_size_m: float = 0.05,
    margin_m: float = 0.03,
    tag_ids: list = [0, 1, 2, 3],
    use_stereo: bool = False,
    camera: str = "left"
):
    """
    Run live verification of intrinsic calibration.
    """
    # Load calibration
    print(f"Loading calibration from: {calib_file}")
    camera_matrix, dist_coeffs = load_calibration(calib_file, camera)
    print(f"Camera matrix:\n{camera_matrix}")
    print(f"Distortion coefficients: {dist_coeffs}")
    
    # Load config if provided
    if config_file and Path(config_file).exists():
        with open(config_file, 'r') as f:
            config = json.load(f)
        tag_size_m = config.get("tag_size_m", tag_size_m)
        margin_m = config.get("margin_m", margin_m)
        tag_ids = config.get("tag_ids", tag_ids)
        cols = config.get("cols", 2)
        rows = config.get("rows", 2)
        print(f"Loaded board config: tag_size={tag_size_m*1000:.1f}mm, margin={margin_m*1000:.1f}mm, grid={cols}x{rows}")
    else:
        cols = 2
        rows = 2
    
    # Compute expected distances
    expected_distances = get_expected_distances(tag_size_m, margin_m, tag_ids, cols, rows)
    print("\nExpected distances (mm):")
    for pair, dist in expected_distances.items():
        print(f"  {pair[0]}-{pair[1]}: {dist*1000:.1f}")
    
    # Open camera
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_id}")
        return
    
    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840 if use_stereo else 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080 if use_stereo else 1080)
    
    print("\nControls:")
    print("  'q' - Quit")
    print("  's' - Save current measurement to file")
    print("  'u' - Toggle undistortion")
    if use_stereo:
        print("  'l' - Toggle left/right camera")
    
    apply_undistort = False
    current_camera = camera  # Track current camera for stereo toggle
    measurement_log = []
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Handle stereo
        if use_stereo:
            h, w = frame.shape[:2]
            if current_camera == "left":
                frame = frame[:, :w//2]
            else:
                frame = frame[:, w//2:]
        
        # Optionally undistort
        if apply_undistort:
            frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
        
        # Detect ArUco tags
        corners, ids = detect_aruco_tags(frame)
        
        tag_centers = {}
        measured_distances = {}
        
        if ids is not None:
            # Get 3D pose for each detected tag
            for i, corner in enumerate(corners):
                tag_id = ids[i][0]
                if tag_id in tag_ids:
                    rvec, tvec, center = solve_tag_pose(
                        corner, tag_size_m, camera_matrix, dist_coeffs
                    )
                    if center is not None:
                        tag_centers[tag_id] = center
            
            # Compute distances if we have enough tags
            if len(tag_centers) >= 2:
                measured_distances = compute_distances(tag_centers)
        
        # Draw visualization
        vis = draw_verification_overlay(
            frame, corners, ids, tag_centers, 
            measured_distances, expected_distances,
            camera_matrix, dist_coeffs, tag_size_m
        )
        
        # Show detection status
        n_detected = len(tag_centers)
        status_color = (0, 255, 0) if n_detected == 4 else (0, 165, 255)
        status_text = f"Detected: {n_detected}/4 tags | Undistort: {'ON' if apply_undistort else 'OFF'}"
        if use_stereo:
            status_text += f" | Camera: {current_camera.upper()}"
        cv2.putText(vis, status_text, (10, vis.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        # Show average error and planeness if all 4 tags detected
        if len(measured_distances) == 6:
            errors = []
            for pair, measured in measured_distances.items():
                for exp_pair, expected in expected_distances.items():
                    if set(pair) == set(exp_pair):
                        errors.append(abs(measured - expected) * 1000)
            avg_error = np.mean(errors)
            max_error = np.max(errors)
            
            error_text = f"Avg Error: {avg_error:.2f}mm | Max Error: {max_error:.2f}mm"
            error_color = (0, 255, 0) if avg_error < 2 else ((0, 255, 255) if avg_error < 5 else (0, 0, 255))
            cv2.putText(vis, error_text, (10, vis.shape[0] - 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, error_color, 2)
            
            # Compute and display planeness
            if len(tag_centers) >= 4:
                plane_rmse, plane_max, plane_normal = compute_planeness(tag_centers)
                if plane_rmse is not None:
                    plane_rmse_mm = plane_rmse * 1000
                    plane_max_mm = plane_max * 1000
                    
                    # Color based on planeness quality
                    if plane_rmse_mm < 1:
                        plane_color = (0, 255, 0)  # Green - excellent
                    elif plane_rmse_mm < 3:
                        plane_color = (0, 255, 255)  # Yellow - good
                    elif plane_rmse_mm < 5:
                        plane_color = (0, 165, 255)  # Orange - acceptable
                    else:
                        plane_color = (0, 0, 255)  # Red - poor
                    
                    plane_text = f"Planeness: RMSE={plane_rmse_mm:.2f}mm Max={plane_max_mm:.2f}mm"
                    cv2.putText(vis, plane_text, (10, vis.shape[0] - 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, plane_color, 2)
        
        cv2.imshow("Calibration Verification", vis)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('u'):
            apply_undistort = not apply_undistort
            print(f"Undistortion: {'ON' if apply_undistort else 'OFF'}")
        elif key == ord('l') and use_stereo:
            # Toggle between left and right camera
            current_camera = "right" if current_camera == "left" else "left"
            # Reload calibration for the new camera
            camera_matrix, dist_coeffs = load_calibration(calib_file, current_camera)
            print(f"Switched to {current_camera.upper()} camera")
        elif key == ord('s'):
            if len(measured_distances) > 0:
                import time
                entry = {
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "tag_centers": {str(k): v.tolist() for k, v in tag_centers.items()},
                    "measured_distances": {f"{p[0]}-{p[1]}": d for p, d in measured_distances.items()},
                    "undistorted": apply_undistort
                }
                measurement_log.append(entry)
                print(f"Saved measurement {len(measurement_log)}")
    
    cap.release()
    cv2.destroyAllWindows()
    
    # Save measurements if any
    if measurement_log:
        log_path = "verification_measurements.json"
        with open(log_path, 'w') as f:
            json.dump({
                "calibration_file": calib_file,
                "expected_distances": {f"{p[0]}-{p[1]}": d for p, d in expected_distances.items()},
                "measurements": measurement_log
            }, f, indent=2)
        print(f"\nSaved {len(measurement_log)} measurements to {log_path}")


def analyze_from_image(
    image_path: str,
    calib_file: str,
    config_file: str = None,
    tag_size_m: float = 0.05,
    margin_m: float = 0.03,
    tag_ids: list = [0, 1, 2, 3],
):
    """
    Analyze a single image for verification (non-live mode).
    """
    print(f"Analyzing image: {image_path}")
    
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image {image_path}")
        return
    
    # Load calibration
    camera_matrix, dist_coeffs = load_calibration(calib_file)
    
    # Load config if provided
    if config_file and Path(config_file).exists():
        with open(config_file, 'r') as f:
            config = json.load(f)
        tag_size_m = config.get("tag_size_m", tag_size_m)
        margin_m = config.get("margin_m", margin_m)
        tag_ids = config.get("tag_ids", tag_ids)
        cols = config.get("cols", 2)
        rows = config.get("rows", 2)
    else:
        cols = 2
        rows = 2
    
    expected_distances = get_expected_distances(tag_size_m, margin_m, tag_ids, cols, rows)
    
    # Detect and analyze
    corners, ids = detect_aruco_tags(image)
    
    tag_centers = {}
    if ids is not None:
        for i, corner in enumerate(corners):
            tag_id = ids[i][0]
            if tag_id in tag_ids:
                rvec, tvec, center = solve_tag_pose(
                    corner, tag_size_m, camera_matrix, dist_coeffs
                )
                if center is not None:
                    tag_centers[tag_id] = center
    
    print(f"\nDetected {len(tag_centers)}/4 tags")
    
    if len(tag_centers) >= 2:
        measured_distances = compute_distances(tag_centers)
        
        print("\nMeasurement Results:")
        print("-" * 60)
        total_error = 0
        count = 0
        
        for pair, measured in measured_distances.items():
            for exp_pair, expected in expected_distances.items():
                if set(pair) == set(exp_pair):
                    error = abs(measured - expected) * 1000
                    error_pct = abs(measured - expected) / expected * 100
                    total_error += error
                    count += 1
                    status = "✓" if error_pct < 3 else "✗"
                    print(f"  {status} {pair[0]}-{pair[1]}: {measured*1000:.2f}mm (expected: {expected*1000:.1f}mm, error: {error:.2f}mm / {error_pct:.2f}%)")
        
        if count > 0:
            avg_error = total_error / count
            print("-" * 60)
            print(f"  Average Error: {avg_error:.2f}mm")
            
            if avg_error < 1:
                print("  ✓ Excellent calibration!")
            elif avg_error < 2:
                print("  ✓ Good calibration")
            elif avg_error < 5:
                print("  ⚠ Acceptable calibration, consider recalibrating")
            else:
                print("  ✗ Poor calibration, recalibration recommended")
    
    # Show result
    vis = draw_verification_overlay(
        image, corners, ids, tag_centers,
        compute_distances(tag_centers) if len(tag_centers) >= 2 else {},
        expected_distances, camera_matrix, dist_coeffs, tag_size_m
    )
    
    cv2.imshow("Verification Result", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Verify intrinsic calibration using ArUco board")
    parser.add_argument("--camera", type=int, default=0, help="Camera index for live mode")
    parser.add_argument("--image", type=str, default=None, help="Image path for single-image analysis")
    parser.add_argument("--calibration", type=str, default="intrinsic_calibration.json", help="Calibration JSON file")
    parser.add_argument("--config", type=str, default=None, help="Board config JSON file")
    parser.add_argument("--tag-size", type=float, default=0.05, help="Tag size in meters")
    parser.add_argument("--margin", type=float, default=0.03, help="Margin between tags in meters")
    parser.add_argument("--tag-ids", type=int, nargs=4, default=[0, 1, 2, 3], help="Tag IDs on the board")
    parser.add_argument("--stereo", action="store_true", help="Use stereo camera (side-by-side)")
    parser.add_argument("--camera-side", choices=["left", "right"], default="left", help="Which camera to use for stereo")
    
    args = parser.parse_args()
    
    if args.image:
        analyze_from_image(
            args.image,
            args.calibration,
            args.config,
            args.tag_size,
            args.margin,
            args.tag_ids,
        )
    else:
        run_verification(
            args.camera,
            args.calibration,
            args.config,
            args.tag_size,
            args.margin,
            args.tag_ids,
            args.stereo,
            args.camera_side,
        )
