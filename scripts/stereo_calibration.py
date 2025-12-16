#!/usr/bin/env python3
"""
Stereo Camera Calibration Script for Vision Pro

This script performs:
1. Intrinsic calibration using checkerboard pattern (6x5 pattern of 10mm squares)
2. Extrinsic calibration (head-to-camera matrix) using ArUco markers

The extrinsic calibration solves:
    marker_pose_arkit = head_pose_arkit @ X @ marker_pose_camera
    
Where X is the head-to-camera transformation matrix.

We use Kabsch algorithm to solve for X by matching translation components.
"""

import argparse
import json
import os
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares


def load_session_info(session_dir: Path) -> dict:
    """Load session info from JSON file."""
    with open(session_dir / "session_info.json", "r") as f:
        return json.load(f)


def load_intrinsic_calibration(session_dir: Path) -> Optional[dict]:
    """Load existing intrinsic calibration if available."""
    calib_file = session_dir / "intrinsic_calibration.json"
    if calib_file.exists():
        with open(calib_file, "r") as f:
            return json.load(f)
    return None


def build_camera_matrix(calib: dict) -> np.ndarray:
    """Build 3x3 camera matrix from calibration params."""
    return np.array([
        [calib["fx"], 0, calib["cx"]],
        [0, calib["fy"], calib["cy"]],
        [0, 0, 1]
    ], dtype=np.float64)


def build_distortion_coeffs(calib: dict) -> np.ndarray:
    """Build distortion coefficients from calibration params."""
    return np.array(calib["distortion_coeffs"], dtype=np.float64)


def split_stereo_image(img: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Split a stereo side-by-side image into left and right images."""
    h, w = img.shape[:2]
    mid = w // 2
    return img[:, :mid], img[:, mid:]


def calibrate_intrinsic(
    session_dir: Path,
    pattern_size: tuple[int, int] = (5, 4),  # inner corners (one less than squares)
    square_size_mm: float = 10.0,
    max_frames: int = 100,
    show_detections: bool = False
) -> dict:
    """
    Perform intrinsic calibration using checkerboard pattern.
    
    Args:
        session_dir: Path to session directory
        pattern_size: (cols, rows) of inner corners 
        square_size_mm: Size of each square in mm
        max_frames: Maximum number of frames to use for calibration
        show_detections: Whether to display detected corners
        
    Returns:
        Dictionary with calibration results for left and right cameras
    """
    intrinsic_dir = session_dir / "intrinsic"
    if not intrinsic_dir.exists():
        raise ValueError(f"Intrinsic directory not found: {intrinsic_dir}")
    
    # Prepare object points (3D points in world coordinates)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size_mm / 1000.0  # Convert to meters
    
    # Storage for calibration data
    objpoints = []  # 3D points in world coordinate system
    imgpoints_left = []  # 2D points in left image
    imgpoints_right = []  # 2D points in right image
    
    # Find all JPEG frames
    frame_files = sorted(intrinsic_dir.glob("frame_*.jpg"))
    print(f"Found {len(frame_files)} intrinsic frames")
    
    # Sample frames evenly if we have too many
    if len(frame_files) > max_frames:
        indices = np.linspace(0, len(frame_files) - 1, max_frames, dtype=int)
        frame_files = [frame_files[i] for i in indices]
    
    img_size = None
    successful_detections = 0
    
    for frame_file in frame_files:
        # Read stereo image
        img = cv2.imread(str(frame_file))
        if img is None:
            continue
            
        # Split into left and right
        left_img, right_img = split_stereo_image(img)
        
        if img_size is None:
            img_size = (left_img.shape[1], left_img.shape[0])
        
        # Convert to grayscale
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
        
        # Find checkerboard corners
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret_left, corners_left = cv2.findChessboardCorners(left_gray, pattern_size, flags)
        ret_right, corners_right = cv2.findChessboardCorners(right_gray, pattern_size, flags)
        
        if ret_left and ret_right:
            # Refine corner detection
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_left = cv2.cornerSubPix(left_gray, corners_left, (11, 11), (-1, -1), criteria)
            corners_right = cv2.cornerSubPix(right_gray, corners_right, (11, 11), (-1, -1), criteria)
            
            objpoints.append(objp)
            imgpoints_left.append(corners_left)
            imgpoints_right.append(corners_right)
            successful_detections += 1
            
            if show_detections:
                # Draw and display corners
                left_vis = cv2.drawChessboardCorners(left_img.copy(), pattern_size, corners_left, ret_left)
                right_vis = cv2.drawChessboardCorners(right_img.copy(), pattern_size, corners_right, ret_right)
                combined = np.hstack([left_vis, right_vis])
                cv2.imshow("Checkerboard Detection", combined)
                if cv2.waitKey(100) == ord('q'):
                    break
    
    if show_detections:
        cv2.destroyAllWindows()
    
    print(f"Successfully detected checkerboard in {successful_detections} frames")
    
    if successful_detections < 10:
        raise ValueError(f"Insufficient checkerboard detections: {successful_detections}")
    
    # Calibrate left camera
    print("Calibrating left camera...")
    ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
        objpoints, imgpoints_left, img_size, None, None
    )
    print(f"Left camera RMS error: {ret_left:.4f}")
    
    # Calibrate right camera
    print("Calibrating right camera...")
    ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
        objpoints, imgpoints_right, img_size, None, None
    )
    print(f"Right camera RMS error: {ret_right:.4f}")
    
    # Build result dictionary
    result = {
        "image_width": img_size[0],
        "image_height": img_size[1],
        "left": {
            "fx": float(mtx_left[0, 0]),
            "fy": float(mtx_left[1, 1]),
            "cx": float(mtx_left[0, 2]),
            "cy": float(mtx_left[1, 2]),
            "distortion_coeffs": dist_left.flatten().tolist(),
            "rms_error": float(ret_left)
        },
        "right": {
            "fx": float(mtx_right[0, 0]),
            "fy": float(mtx_right[1, 1]),
            "cx": float(mtx_right[0, 2]),
            "cy": float(mtx_right[1, 2]),
            "distortion_coeffs": dist_right.flatten().tolist(),
            "rms_error": float(ret_right)
        }
    }
    
    return result


def detect_aruco_marker(
    img: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_size: float,
    target_ids: list[int] = [0, 2, 3],
    return_details: bool = False,
    min_depth: float = 0.05,  # Minimum depth in meters (50mm)
    max_depth: float = 2.0    # Maximum depth in meters (2000mm)
) -> dict[int, np.ndarray] | tuple[dict, dict, dict]:
    """
    Detect ArUco markers and estimate their poses.
    
    Args:
        return_details: If True, also return corners and rvec/tvec for visualization
        min_depth: Minimum valid depth in meters (reject closer detections)
        max_depth: Maximum valid depth in meters (reject farther detections)
    
    Returns:
        If return_details=False: Dictionary mapping marker ID to 4x4 transformation matrix
        If return_details=True: (transforms, corners, rvecs_tvecs)
    """
    # Use DICT_4X4_50 as it's common for 4x4 markers
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    
    # Detect markers
    corners, ids, rejected = detector.detectMarkers(img)
    
    result = {}
    corners_dict = {}
    rvecs_tvecs = {}
    
    if ids is None or len(ids) == 0:
        if return_details:
            return result, corners_dict, rvecs_tvecs
        return result
    
    # Estimate pose for each detected marker
    for i, marker_id in enumerate(ids.flatten()):
        if marker_id not in target_ids:
            continue
            
        # Get marker corners
        marker_corners = corners[i]
        
        # Estimate pose using solvePnP
        obj_points = np.array([
            [-marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0]
        ], dtype=np.float32)
        
        success, rvec, tvec = cv2.solvePnP(
            obj_points, marker_corners.reshape(-1, 2),
            camera_matrix, dist_coeffs
        )
        
        if success:
            depth = tvec[2][0]  # Z component is depth
            
            # Reject detections with unreasonable depth (false positives)
            if depth < min_depth or depth > max_depth:
                # Skip this detection but still store corners for visualization
                corners_dict[marker_id] = marker_corners
                continue
            
            # Convert to 4x4 transformation matrix
            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = tvec.flatten()
            result[marker_id] = T
            corners_dict[marker_id] = marker_corners
            rvecs_tvecs[marker_id] = (rvec, tvec)
    
    if return_details:
        return result, corners_dict, rvecs_tvecs
    return result


def visualize_marker_detection(
    img: np.ndarray,
    corners_dict: dict,
    rvecs_tvecs: dict,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_size: float,
    label: str = ""
) -> np.ndarray:
    """
    Draw marker detection visualization on image.
    
    Args:
        img: Input image (will be copied)
        corners_dict: Dictionary of marker ID -> corners
        rvecs_tvecs: Dictionary of marker ID -> (rvec, tvec)
        camera_matrix: Camera intrinsic matrix
        dist_coeffs: Distortion coefficients
        marker_size: Marker size in meters
        label: Optional label to draw on image
        
    Returns:
        Image with visualization overlay
    """
    vis = img.copy()
    
    for marker_id, corners in corners_dict.items():
        # Draw marker outline
        corners_int = corners.reshape(-1, 2).astype(int)
        cv2.polylines(vis, [corners_int], True, (0, 255, 0), 2)
        
        # Draw corner points
        for j, pt in enumerate(corners_int):
            color = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 255, 0)][j]
            cv2.circle(vis, tuple(pt), 5, color, -1)
        
        # Draw marker ID
        center = corners_int.mean(axis=0).astype(int)
        cv2.putText(vis, f"ID:{marker_id}", (center[0] - 20, center[1] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Draw pose axes if available
        if marker_id in rvecs_tvecs:
            rvec, tvec = rvecs_tvecs[marker_id]
            cv2.drawFrameAxes(vis, camera_matrix, dist_coeffs, rvec, tvec, marker_size * 0.5)
            
            # Draw depth info
            depth = tvec[2][0] * 1000  # Convert to mm
            cv2.putText(vis, f"Z:{depth:.0f}mm", (center[0] - 30, center[1] + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    # Draw label
    if label:
        cv2.putText(vis, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    
    # Draw "No detection" if no markers
    if not corners_dict:
        cv2.putText(vis, "NO MARKER DETECTED", (img.shape[1]//4, img.shape[0]//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
    
    return vis


def kabsch_algorithm(P: np.ndarray, Q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Kabsch algorithm to find optimal rotation and translation
    that aligns point set P to point set Q.
    
    Args:
        P: Nx3 array of source points
        Q: Nx3 array of target points
        
    Returns:
        R: 3x3 rotation matrix
        t: 3x1 translation vector
    """
    # Center the point sets
    centroid_P = np.mean(P, axis=0)
    centroid_Q = np.mean(Q, axis=0)
    
    P_centered = P - centroid_P
    Q_centered = Q - centroid_Q
    
    # Compute covariance matrix
    H = P_centered.T @ Q_centered
    
    # SVD
    U, S, Vt = np.linalg.svd(H)
    
    # Compute rotation
    R = Vt.T @ U.T
    
    # Handle reflection case
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Compute translation
    t = centroid_Q - R @ centroid_P
    
    return R, t


def rvec_tvec_to_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Convert rotation vector and translation vector to 4x4 transformation matrix."""
    R, _ = cv2.Rodrigues(np.array(rvec).reshape(3, 1))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array(tvec).flatten()
    return T


def matrix_to_rvec_tvec(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Convert 4x4 transformation matrix to rotation and translation vectors."""
    R = T[:3, :3]
    rvec, _ = cv2.Rodrigues(R)
    tvec = T[:3, 3]
    return rvec.flatten(), tvec


def joint_calibration_cost(
    params: np.ndarray,
    observations: list,
    T_b_v_list: list[np.ndarray],
    marker_positions_gt: dict[int, np.ndarray],
    marker_3d_points: np.ndarray,
    optimize_intrinsics: bool = True
) -> np.ndarray:
    """
    Position-only cost function for joint intrinsic/extrinsic calibration.
    
    The constraint is:
        T_b^v(t) @ T_v^c @ p_c^m = p_b^m (ground truth)
    
    Where:
        T_b^v(t) is the SLAM/ARKit head pose at time t
        T_v^c is the head-to-camera transform we're solving for
        p_c^m is the marker position in camera frame (from solvePnP)
        p_b^m is the ground truth marker position in base/world frame
    
    Args:
        params: Parameter vector:
            If optimize_intrinsics: [fx, fy, cx, cy, k1, k2, p1, p2, rvec_vc(3), tvec_vc(3)]
            Otherwise: [rvec_vc(3), tvec_vc(3)]
        observations: List of (marker_id, corners, frame_idx, K, D) tuples
            K, D are only used if not optimizing intrinsics
        T_b_v_list: List of 4x4 head poses indexed by frame_idx
        marker_positions_gt: Dict mapping marker_id to ground truth position (3D vector)
        marker_3d_points: 4x3 array of marker corner positions in marker frame
        optimize_intrinsics: Whether to optimize intrinsics or just extrinsics
        
    Returns:
        Residual vector (flat array of position errors)
    """
    if optimize_intrinsics:
        # Unpack intrinsics
        fx, fy, cx, cy = params[:4]
        k1, k2, p1, p2 = params[4:8]
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        D = np.array([k1, k2, p1, p2], dtype=np.float64)
        rvec_vc = params[8:11]
        tvec_vc = params[11:14]
    else:
        rvec_vc = params[:3]
        tvec_vc = params[3:6]
        # Will use K, D from observations
        K, D = None, None
    
    T_v_c = rvec_tvec_to_matrix(rvec_vc, tvec_vc)
    
    residuals = []
    
    for obs in observations:
        if optimize_intrinsics:
            marker_id, corners, frame_idx = obs[:3]
        else:
            marker_id, corners, frame_idx, K, D = obs
        
        # Get marker center position from PnP
        success, rvec_cm, tvec_cm = cv2.solvePnP(
            marker_3d_points, corners.reshape(-1, 2),
            K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        
        if not success:
            continue
        
        # Marker position in camera frame (just the translation)
        p_c_m = tvec_cm.flatten()
        
        # Transform to Vision Pro (head) frame
        p_v_m = T_v_c[:3, :3] @ p_c_m + T_v_c[:3, 3]
        
        # Transform to base/world frame using head pose
        T_b_v = T_b_v_list[frame_idx]
        p_b_m = T_b_v[:3, :3] @ p_v_m + T_b_v[:3, 3]
        
        # Ground truth position
        p_b_m_gt = marker_positions_gt[marker_id]
        
        # Position error (3D)
        error = p_b_m - p_b_m_gt
        residuals.extend(error)
    
    return np.array(residuals)


    return np.array(residuals)


def joint_stereo_residuals(
    params: np.ndarray,
    left_observations: list,
    right_observations: list,
    T_b_v_list: list[np.ndarray],
    marker_positions_gt: dict[int, np.ndarray],
    marker_3d_points: np.ndarray
) -> np.ndarray:
    """
    Compute residuals for joint stereo calibration (using PnP).
    Returns vector of residuals for use with least_squares (LM).
    """
    # Unpack shared intrinsics
    f, cx, cy, k1, k2 = params[:5]
    K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float64)
    D = np.array([k1, k2, 0, 0], dtype=np.float64)
    
    # Unpack extrinsics
    rvec_L = params[5:8]
    tvec_L = params[8:11]
    T_v_c_L = rvec_tvec_to_matrix(rvec_L, tvec_L)
    
    rvec_R = params[11:14]
    tvec_R = params[14:17]
    T_v_c_R = rvec_tvec_to_matrix(rvec_R, tvec_R)
    
    residuals = []
    
    def process(observations, T_v_c):
        for marker_id, corners, frame_idx in observations:
            success, rvec_cm, tvec_cm = cv2.solvePnP(
                marker_3d_points, corners.reshape(-1, 2),
                K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not success:
                continue
                
            p_c_m = tvec_cm.flatten()
            p_v_m = T_v_c[:3, :3] @ p_c_m + T_v_c[:3, 3]
            
            T_b_v = T_b_v_list[frame_idx]
            if T_b_v is None:
                continue
                
            p_b_m = T_b_v[:3, :3] @ p_v_m + T_b_v[:3, 3]
            p_b_m_gt = marker_positions_gt[marker_id]
            
            residuals.extend(p_b_m - p_b_m_gt)
            
    process(left_observations, T_v_c_L)
    process(right_observations, T_v_c_R)
    
    return np.array(residuals)


def ray_stereo_residuals(
    params: np.ndarray,
    left_observations: list,
    right_observations: list,
    T_b_v_list: list[np.ndarray],
    marker_positions_gt: dict[int, np.ndarray]
) -> np.ndarray:
    """
    Compute residuals for ray-based calibration.
    Returns vector of residuals (perpendicular distance components) for least_squares (LM).
    """
    # Unpack shared intrinsics
    f, cx, cy, k1, k2 = params[:5]
    K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float64)
    D = np.array([k1, k2, 0, 0], dtype=np.float64)
    
    # Unpack extrinsics
    rvec_L = params[5:8]
    tvec_L = params[8:11]
    T_v_c_L = rvec_tvec_to_matrix(rvec_L, tvec_L)
    
    rvec_R = params[11:14]
    tvec_R = params[14:17]
    T_v_c_R = rvec_tvec_to_matrix(rvec_R, tvec_R)
    
    residuals = []
    
    def process(observations, T_v_c):
        for marker_id, corners, frame_idx in observations:
            T_b_v = T_b_v_list[frame_idx]
            if T_b_v is None:
                continue
            
            center_px = corners.reshape(-1, 2).mean(axis=0)
            center_undist = cv2.undistortPoints(
                center_px.reshape(1, 1, 2).astype(np.float64), K, D, P=K
            ).reshape(2)
            
            ray_c = np.array([(center_undist[0] - cx)/f, (center_undist[1] - cy)/f, 1.0])
            ray_c = ray_c / np.linalg.norm(ray_c)
            
            T_b_c = T_b_v @ T_v_c
            ray_b = T_b_c[:3, :3] @ ray_c
            origin_b = T_b_c[:3, 3]
            
            p_gt = marker_positions_gt[marker_id]
            
            # Perpendicular vector component
            v = p_gt - origin_b
            parallel = np.dot(v, ray_b) * ray_b
            perpendicular = v - parallel
            
            residuals.extend(perpendicular)
            
    process(left_observations, T_v_c_L)
    process(right_observations, T_v_c_R)
    
    return np.array(residuals)


def calibrate_joint_stereo(
    session_dir: Path,
    intrinsic_calib: dict,
    max_iterations: int = 200,
    use_ray_cost: bool = False
) -> dict:
    """
    Hybrid Joint Stereo Calibration (Shared Intrinsics):
    1. Global Optimization (Differential Evolution) to find basin of attraction
    2. Local Optimization (Levenberg-Marquardt) to refine solution
    """
    from scipy.optimize import differential_evolution, least_squares
    
    extrinsic_dir = session_dir / "extrinsic"
    session_info = load_session_info(session_dir)
    marker_size = session_info.get("marker_size_meters", 0.055)
    
    img_width = intrinsic_calib.get("image_width", 640)
    img_height = intrinsic_calib.get("image_height", 480)
    
    marker_3d_points = np.array([
        [-marker_size/2, marker_size/2, 0], [marker_size/2, marker_size/2, 0],
        [marker_size/2, -marker_size/2, 0], [-marker_size/2, -marker_size/2, 0]
    ], dtype=np.float32)
    
    # Collect observations (same as before)
    left_observations = []
    right_observations = []
    T_b_v_list = []
    marker_positions_gt = {}
    
    sample_dirs = sorted(extrinsic_dir.glob("sample_*"))
    for frame_idx, sample_dir in enumerate(sample_dirs):
        if not (sample_dir/"data.json").exists(): continue
        with open(sample_dir/"data.json", "r") as f: data = json.load(f)
        if "arkit_marker_pose" not in data: continue
        
        while len(T_b_v_list) <= frame_idx: T_b_v_list.append(None)
        T_b_v_list[frame_idx] = np.array(data["head_pose"])
        marker_id = data.get("marker_id", 0)
        if marker_id not in marker_positions_gt:
            marker_positions_gt[marker_id] = np.array(data["arkit_marker_pose"])[:3, 3]
            
        img_file = sample_dir / "frame_stereo.jpg"
        if img_file.exists():
            img = cv2.imread(str(img_file))
            if img is not None:
                left, right = split_stereo_image(img)
                det = cv2.aruco.ArucoDetector(
                    cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                    cv2.aruco.DetectorParameters()
                )
                corners_L, ids_L, _ = det.detectMarkers(left)
                if ids_L is not None and marker_id in ids_L:
                    idx = list(ids_L.flatten()).index(marker_id)
                    left_observations.append((marker_id, corners_L[idx], frame_idx))
                    
                corners_R, ids_R, _ = det.detectMarkers(right)
                if ids_R is not None and marker_id in ids_R:
                    idx = list(ids_R.flatten()).index(marker_id)
                    right_observations.append((marker_id, corners_R[idx], frame_idx))

    print(f"\nCollected {len(left_observations)} Left + {len(right_observations)} Right observations")
    
    if len(left_observations) < 5: raise ValueError("Insufficient observations")

    # Define bounds
    bounds_limits = [
        (img_width*0.3, img_width*2.0), # f
        (img_width*0.2, img_width*0.8), # cx
        (img_height*0.2, img_height*0.8), # cy
        (-0.5, 0.5), (-0.5, 0.5),       # k1, k2
        (-np.pi, np.pi), (-np.pi, np.pi), (-np.pi, np.pi), # L rot
        (-0.3, 0.3), (-0.3, 0.3), (-0.3, 0.3),             # L transl
        (-np.pi, np.pi), (-np.pi, np.pi), (-np.pi, np.pi), # R rot
        (-0.3, 0.3), (-0.3, 0.3), (-0.3, 0.3)              # R transl
    ]

    # Select cost function
    residual_func = ray_stereo_residuals if use_ray_cost else joint_stereo_residuals
    args = (left_observations, right_observations, T_b_v_list, marker_positions_gt)
    if not use_ray_cost: args += (marker_3d_points,)

    def scalar_cost(params):
        res = residual_func(params, *args)
        return np.sum(res**2)

    print("\n--- STAGE 1: Global Optimization (Differential Evolution) ---")
    print("Finding basin of attraction...")
    # Use fewer iterations for DE since we'll refine with LM
    de_iters = max(10, max_iterations // 5) 
    
    result_de = differential_evolution(
        scalar_cost,
        bounds_limits,
        maxiter=de_iters,
        popsize=10,        # Increased from default for better exploration
        tol=1e-3,          # Looser tolerance for Stage 1
        workers=1,
        disp=True
    )
    print(f"Stage 1 cost: {result_de.fun:.6f}")

    print("\n--- STAGE 2: Local Refinement (Trust Region Reflective) ---")
    print("Polishing solution with bounds enforcement...")
    
    # Convert bounds for least_squares (lower_array, upper_array)
    lower_bounds = np.array([b[0] for b in bounds_limits])
    upper_bounds = np.array([b[1] for b in bounds_limits])
    
    result_lm = least_squares(
        residual_func,
        result_de.x,
        bounds=(lower_bounds, upper_bounds),
        args=args,
        method='trf',  # 'lm' doesn't support bounds, 'trf' does
        max_nfev=2000,
        verbose=1,
        x_scale='jac', # improving scaling
        ftol=1e-8,
        xtol=1e-8,
        gtol=1e-8
    )
    
    final_params = result_lm.x
    final_cost = np.sum(result_lm.fun**2)
    print(f"Stage 2 final cost: {final_cost:.6f}")
    
    # Extract Results
    f_opt, cx_opt, cy_opt, k1_opt, k2_opt = final_params[:5]
    rvec_L, tvec_L = final_params[5:8], final_params[8:11]
    rvec_R, tvec_R = final_params[11:14], final_params[14:17]
    
    T_v_c_L = rvec_tvec_to_matrix(rvec_L, tvec_L)
    T_v_c_R = rvec_tvec_to_matrix(rvec_R, tvec_R)
    
    print("\nFINAL RESULTS:")
    print(f"Shared Focal Length: {f_opt:.1f}")
    print(f"Left Translation: {tvec_L * 1000} mm")
    print(f"Right Translation: {tvec_R * 1000} mm")
    
    # Return formatted validation results
    intrinsics = {
        "fx": float(f_opt), "fy": float(f_opt),
        "cx": float(cx_opt), "cy": float(cy_opt),
        "distortion_coeffs": [float(k1_opt), float(k2_opt), 0.0, 0.0]
    }
    
    # Error metrics are computed from residuals
    residuals = result_lm.fun.reshape(-1, 3)
    errors = np.linalg.norm(residuals, axis=1)
    
    return {
        "left": {
            "intrinsics": intrinsics,
            "head_to_camera": T_v_c_L.tolist(),
            "mean_error_meters": float(np.mean(errors)),
            "max_error_meters": float(np.max(errors)),
            "sample_count": len(left_observations)
        },
        "right": {
            "intrinsics": intrinsics,
            "head_to_camera": T_v_c_R.tolist(),
            "mean_error_meters": float(np.mean(errors)), # Shared error metric approximation
            "max_error_meters": float(np.max(errors)),
            "sample_count": len(right_observations)
        },
        "shared_intrinsics": intrinsics,
        "optimization_success": result_lm.success,
        "optimization_cost": float(final_cost)
    }


def calibrate_joint(
    session_dir: Path,
    intrinsic_calib: dict,
    optimize_intrinsics: bool = True,
    max_iterations: int = 100,
    use_global_optimizer: bool = True
) -> dict:
    """
    Perform joint intrinsic/extrinsic calibration using position-only constraints.
    
    This method uses multiple markers at different locations to simultaneously
    solve for camera intrinsics and the head-to-camera extrinsic transform.
    
    Uses differential evolution (global optimizer) to avoid local minima.
    
    The cost function minimizes:
        sum_t sum_i || T_b^v(t) @ T_v^c @ solvePnP(corners_i(t), K, D) - p_b^{m_i} ||²
    
    Where p_b^{m_i} is the ground truth marker position in world frame.
    
    Args:
        session_dir: Path to session directory
        intrinsic_calib: Initial intrinsic calibration dictionary (used for image size, not values)
        optimize_intrinsics: Whether to also optimize intrinsics or just extrinsics
        max_iterations: Maximum iterations for the optimizer
        use_global_optimizer: If True, use differential evolution; else use LM
        
    Returns:
        Dictionary with optimized calibration results
    """
    from scipy.optimize import differential_evolution
    
    extrinsic_dir = session_dir / "extrinsic"
    if not extrinsic_dir.exists():
        raise ValueError(f"Extrinsic directory not found: {extrinsic_dir}")
    
    # Load session info for marker size
    session_info = load_session_info(session_dir)
    marker_size = session_info.get("marker_size_meters", 0.055)
    
    # Get image size from intrinsic calib
    img_width = intrinsic_calib.get("image_width", 640)
    img_height = intrinsic_calib.get("image_height", 480)
    
    # Marker 3D points in marker frame (centered at origin)
    marker_3d_points = np.array([
        [-marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0]
    ], dtype=np.float32)
    
    # Collect observations for both cameras
    results = {}
    
    for side in ["left", "right"]:
        print(f"\n{'='*60}")
        print(f"JOINT CALIBRATION - {side.upper()} CAMERA")
        print(f"{'='*60}")
        
        # Collect observations
        observations = []  # (marker_id, corners, frame_idx)
        T_b_v_list = []    # head poses indexed by frame number
        marker_positions_gt = {}  # marker_id -> ground truth position
        
        sample_dirs = sorted(extrinsic_dir.glob("sample_*"))
        
        for frame_idx, sample_dir in enumerate(sample_dirs):
            data_file = sample_dir / "data.json"
            if not data_file.exists():
                continue
            
            with open(data_file, "r") as f:
                data = json.load(f)
            
            # Skip samples without ARKit marker pose
            if "arkit_marker_pose" not in data:
                continue
            
            # Get poses
            head_pose = np.array(data["head_pose"])
            arkit_marker_pose = np.array(data["arkit_marker_pose"])
            marker_id = data.get("marker_id", 0)
            
            # Store head pose
            while len(T_b_v_list) <= frame_idx:
                T_b_v_list.append(None)
            T_b_v_list[frame_idx] = head_pose
            
            # Store marker ground truth position (from ARKit)
            if marker_id not in marker_positions_gt:
                marker_positions_gt[marker_id] = arkit_marker_pose[:3, 3]
            
            # Load and detect marker in image
            img_file = sample_dir / "frame_stereo.jpg"
            if not img_file.exists():
                continue
            
            img = cv2.imread(str(img_file))
            if img is None:
                continue
            
            left_img, right_img = split_stereo_image(img)
            side_img = left_img if side == "left" else right_img
            
            # Detect marker
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            params = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            corners, ids, _ = detector.detectMarkers(side_img)
            
            if ids is not None and marker_id in ids.flatten():
                idx = list(ids.flatten()).index(marker_id)
                marker_corners = corners[idx]
                observations.append((marker_id, marker_corners, frame_idx))
        
        print(f"Collected {len(observations)} observations")
        print(f"Found {len(marker_positions_gt)} unique markers: {list(marker_positions_gt.keys())}")
        
        if len(observations) < 10:
            print(f"Warning: Insufficient observations for joint calibration")
            continue
        
        # Define reasonable bounds based on image size
        # Focal length: typically 0.5x to 2x image width for normal cameras
        f_min, f_max = img_width * 0.3, img_width * 2.0
        # Principal point: should be within image, typically near center
        cx_min, cx_max = img_width * 0.2, img_width * 0.8
        cy_min, cy_max = img_height * 0.2, img_height * 0.8
        # Distortion coefficients: typically small
        dist_min, dist_max = -0.5, 0.5
        # Rotation: full range
        rot_min, rot_max = -np.pi, np.pi
        # Translation: reasonable range in meters (headset to camera)
        t_min, t_max = -0.3, 0.3  # +/- 300mm
        
        if optimize_intrinsics:
            # Bounds: [fx, fy, cx, cy, k1, k2, p1, p2, rx, ry, rz, tx, ty, tz]
            bounds = [
                (f_min, f_max),    # fx
                (f_min, f_max),    # fy
                (cx_min, cx_max),  # cx
                (cy_min, cy_max),  # cy
                (dist_min, dist_max),  # k1
                (dist_min, dist_max),  # k2
                (-0.1, 0.1),       # p1 (tangential, usually smaller)
                (-0.1, 0.1),       # p2
                (rot_min, rot_max),    # rx
                (rot_min, rot_max),    # ry
                (rot_min, rot_max),    # rz
                (t_min, t_max),    # tx
                (t_min, t_max),    # ty
                (t_min, t_max),    # tz
            ]
            print(f"\nOptimizing intrinsics + extrinsics (14 parameters)")
            print(f"  Focal length bounds: [{f_min:.0f}, {f_max:.0f}]")
            print(f"  Principal point bounds: cx=[{cx_min:.0f}, {cx_max:.0f}], cy=[{cy_min:.0f}, {cy_max:.0f}]")
            print(f"  Translation bounds: [{t_min*1000:.0f}, {t_max*1000:.0f}] mm")
        else:
            # Extrinsics only
            K_init = build_camera_matrix(intrinsic_calib[side])
            D_init = build_distortion_coeffs(intrinsic_calib[side])
            bounds = [
                (rot_min, rot_max),    # rx
                (rot_min, rot_max),    # ry
                (rot_min, rot_max),    # rz
                (t_min, t_max),    # tx
                (t_min, t_max),    # ty
                (t_min, t_max),    # tz
            ]
            print(f"\nOptimizing extrinsics only (6 parameters)")
        
        # Cost function wrapper for differential evolution (needs scalar output)
        def scalar_cost(params):
            residuals = joint_calibration_cost(
                params, observations, T_b_v_list, marker_positions_gt,
                marker_3d_points, optimize_intrinsics
            )
            return np.sum(residuals ** 2)
        
        if use_global_optimizer:
            print(f"\nRunning Differential Evolution (global optimizer)...")
            print(f"This may take a few minutes...")
            
            result = differential_evolution(
                scalar_cost,
                bounds,
                maxiter=max_iterations,
                tol=1e-6,
                seed=42,
                workers=1,  # Single-threaded (nested function can't be pickled for multiprocessing)
                disp=True,
                polish=True  # Refine with L-BFGS-B at the end
            )
            
            print(f"\nOptimization completed: {result.message}")
            print(f"Final cost: {result.fun:.6f}")
            print(f"Number of function evaluations: {result.nfev}")
            
            opt_params = result.x
            success = result.success
            final_cost = result.fun
        else:
            # Fall back to LM (needs initial guess)
            # Use sensible defaults: focal length ~ image width, principal point at center
            if optimize_intrinsics:
                x0 = np.array([
                    img_width * 0.8,  # fx - reasonable estimate
                    img_width * 0.8,  # fy - same as fx (square pixels)
                    img_width / 2,    # cx - image center
                    img_height / 2,   # cy - image center
                    0.0, 0.0, 0.0, 0.0,  # distortion (start at zero)
                    0.0, 0.0, 0.0,       # rotation (identity)
                    0.05, 0.0, -0.1      # translation (rough guess: 5cm right, 10cm forward)
                ])
            else:
                x0 = np.array([0.0, 0.0, 0.0, 0.05, 0.0, -0.1])
            
            print(f"\nRunning Levenberg-Marquardt optimization...")
            result = least_squares(
                joint_calibration_cost,
                x0,
                args=(observations, T_b_v_list, marker_positions_gt, marker_3d_points, optimize_intrinsics),
                method='lm',
                max_nfev=max_iterations * len(x0),
                verbose=1
            )
            
            print(f"\nOptimization completed: {result.message}")
            print(f"Final cost: {result.cost:.6f}")
            print(f"Number of function evaluations: {result.nfev}")
            
            opt_params = result.x
            success = result.success
            final_cost = result.cost
        
        # Extract optimized parameters
        if optimize_intrinsics:
            fx_opt, fy_opt, cx_opt, cy_opt = opt_params[:4]
            k1_opt, k2_opt, p1_opt, p2_opt = opt_params[4:8]
            rvec_opt = opt_params[8:11]
            tvec_opt = opt_params[11:14]
            
            K_opt = np.array([[fx_opt, 0, cx_opt], [0, fy_opt, cy_opt], [0, 0, 1]])
            D_opt = np.array([k1_opt, k2_opt, p1_opt, p2_opt])
        else:
            rvec_opt = opt_params[:3]
            tvec_opt = opt_params[3:6]
            fx_opt, fy_opt = K_init[0, 0], K_init[1, 1]
            cx_opt, cy_opt = K_init[0, 2], K_init[1, 2]
            D_opt = D_init
        
        T_v_c_opt = rvec_tvec_to_matrix(rvec_opt, tvec_opt)
        
        # Compute final position errors
        final_residuals = joint_calibration_cost(
            opt_params, observations, T_b_v_list, marker_positions_gt, 
            marker_3d_points, optimize_intrinsics
        )
        final_residuals = final_residuals.reshape(-1, 3)
        position_errors = np.linalg.norm(final_residuals, axis=1)
        
        print(f"\nFinal Results:")
        print(f"  Optimized intrinsics: fx={fx_opt:.1f}, fy={fy_opt:.1f}, "
              f"cx={cx_opt:.1f}, cy={cy_opt:.1f}")
        print(f"  fx/fy ratio: {fx_opt/fy_opt:.3f} (should be ~1.0 for square pixels)")
        print(f"  Distortion: k1={k1_opt:.4f}, k2={k2_opt:.4f}, p1={p1_opt:.4f}, p2={p2_opt:.4f}" if optimize_intrinsics else "")
        print(f"  Optimized extrinsic translation: [{tvec_opt[0]*1000:.1f}, {tvec_opt[1]*1000:.1f}, {tvec_opt[2]*1000:.1f}] mm")
        print(f"  Mean position error: {np.mean(position_errors) * 1000:.2f} mm")
        print(f"  Max position error: {np.max(position_errors) * 1000:.2f} mm")
        print(f"  Median position error: {np.median(position_errors) * 1000:.2f} mm")
        
        # Store results
        results[side] = {
            "intrinsics": {
                "fx": float(fx_opt),
                "fy": float(fy_opt),
                "cx": float(cx_opt),
                "cy": float(cy_opt),
                "distortion_coeffs": D_opt.tolist()
            },
            "head_to_camera": T_v_c_opt.tolist(),
            "mean_error_meters": float(np.mean(position_errors)),
            "max_error_meters": float(np.max(position_errors)),
            "median_error_meters": float(np.median(position_errors)),
            "sample_count": len(observations),
            "optimization_success": success,
            "optimization_cost": float(final_cost)
        }
    
    return results




def calibrate_extrinsic(
    session_dir: Path,
    intrinsic_calib: dict,
    recalculate_opencv_poses: bool = True
) -> dict:
    """
    Perform extrinsic calibration to find head-to-camera transformation.
    
    The relationship is:
        marker_pose_arkit = head_pose_arkit @ X @ marker_pose_camera
        
    We solve for X by matching translation components using Kabsch algorithm.
    
    Args:
        session_dir: Path to session directory
        intrinsic_calib: Intrinsic calibration dictionary
        recalculate_opencv_poses: Whether to recalculate marker poses from images
        
    Returns:
        Dictionary with extrinsic calibration results
    """
    extrinsic_dir = session_dir / "extrinsic"
    if not extrinsic_dir.exists():
        raise ValueError(f"Extrinsic directory not found: {extrinsic_dir}")
    
    # Load session info for marker size
    session_info = load_session_info(session_dir)
    marker_size = session_info.get("marker_size_meters", 0.055)
    
    # Build camera matrices
    left_mtx = build_camera_matrix(intrinsic_calib["left"])
    left_dist = build_distortion_coeffs(intrinsic_calib["left"])
    right_mtx = build_camera_matrix(intrinsic_calib["right"])
    right_dist = build_distortion_coeffs(intrinsic_calib["right"])
    
    # Find all sample directories
    sample_dirs = sorted(extrinsic_dir.glob("sample_*"))
    print(f"Found {len(sample_dirs)} extrinsic samples")
    
    # Collect data for both cameras
    left_data = []  # (marker_pos_arkit, marker_pos_camera)
    right_data = []
    
    for sample_dir in sample_dirs:
        data_file = sample_dir / "data.json"
        if not data_file.exists():
            continue
            
        with open(data_file, "r") as f:
            data = json.load(f)
        
        # Skip samples that don't have ARKit marker pose
        if "arkit_marker_pose" not in data:
            continue
        
        # Get poses from data (already row-major 4x4)
        head_pose = np.array(data["head_pose"])
        arkit_marker_pose = np.array(data["arkit_marker_pose"])
        marker_id = data.get("marker_id", 0)
        
        if recalculate_opencv_poses:
            # Load image and detect markers
            img_file = sample_dir / "frame_stereo.jpg"
            if not img_file.exists():
                continue
                
            img = cv2.imread(str(img_file))
            if img is None:
                continue
                
            left_img, right_img = split_stereo_image(img)
            
            # Detect markers
            left_markers = detect_aruco_marker(
                left_img, left_mtx, left_dist, marker_size, [marker_id]
            )
            right_markers = detect_aruco_marker(
                right_img, right_mtx, right_dist, marker_size, [marker_id]
            )
            
            if marker_id in left_markers:
                opencv_left = left_markers[marker_id]
            else:
                opencv_left = None
                
            if marker_id in right_markers:
                opencv_right = right_markers[marker_id]
            else:
                opencv_right = None
        else:
            # Use pre-computed OpenCV poses from data (already row-major)
            if "opencv_marker_pose_left" in data:
                opencv_left = np.array(data["opencv_marker_pose_left"])
            else:
                opencv_left = None
                
            if "opencv_marker_pose_right" in data:
                opencv_right = np.array(data["opencv_marker_pose_right"])
            else:
                opencv_right = None
        
        # Compute expected marker position in head frame
        # marker_arkit = head_arkit @ X @ marker_camera
        # marker_arkit @ marker_camera^-1 = head_arkit @ X
        # head_arkit^-1 @ marker_arkit @ marker_camera^-1 = X
        
        # We collect: marker position in ARKit (world) frame
        # and: marker position in camera frame
        # Then use Kabsch to find X that maps camera frame markers to head-relative markers
        
        if opencv_left is not None:
            # marker_in_head = head^-1 @ marker_arkit
            head_inv = np.linalg.inv(head_pose)
            marker_in_head = head_inv @ arkit_marker_pose
            
            # marker_in_camera = opencv_left
            left_data.append({
                "marker_in_head": marker_in_head,
                "marker_in_camera": opencv_left,
                "marker_id": marker_id
            })
        
        if opencv_right is not None:
            head_inv = np.linalg.inv(head_pose)
            marker_in_head = head_inv @ arkit_marker_pose
            
            right_data.append({
                "marker_in_head": marker_in_head,
                "marker_in_camera": opencv_right,
                "marker_id": marker_id
            })
    
    print(f"Collected {len(left_data)} left camera samples")
    print(f"Collected {len(right_data)} right camera samples")
    
    # Solve for head-to-camera transformation using Kabsch with RANSAC
    def solve_head_to_camera(
        data_list: list,
        ransac_iterations: int = 100,
        inlier_threshold: float = 0.03  # 30mm
    ) -> tuple[np.ndarray, float, float]:
        """
        Solve for head-to-camera transformation X such that:
        marker_in_head = X @ marker_in_camera
        
        Uses RANSAC with Kabsch algorithm for robust outlier rejection.
        """
        if len(data_list) < 3:
            raise ValueError(f"Insufficient samples: {len(data_list)}")
        
        # Extract translation components
        # P: marker positions in camera frame
        # Q: marker positions in head frame
        P = np.array([d["marker_in_camera"][:3, 3] for d in data_list])
        Q = np.array([d["marker_in_head"][:3, 3] for d in data_list])
        
        n_points = len(P)
        best_inliers = None
        best_inlier_count = 0
        
        # RANSAC iterations
        for _ in range(ransac_iterations):
            # Randomly select 3 points (minimum for Kabsch)
            idx = np.random.choice(n_points, size=min(3, n_points), replace=False)
            P_sample = P[idx]
            Q_sample = Q[idx]
            
            # Compute transformation using Kabsch
            R, t = kabsch_algorithm(P_sample, Q_sample)
            
            # Compute errors for all points
            P_transformed = (R @ P.T).T + t
            errors = np.linalg.norm(P_transformed - Q, axis=1)
            
            # Count inliers
            inliers = errors < inlier_threshold
            inlier_count = np.sum(inliers)
            
            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_inliers = inliers
        
        # Refine using all inliers
        if best_inliers is not None and best_inlier_count >= 3:
            P_inliers = P[best_inliers]
            Q_inliers = Q[best_inliers]
            R, t = kabsch_algorithm(P_inliers, Q_inliers)
            print(f"  RANSAC: {best_inlier_count}/{n_points} inliers ({best_inlier_count/n_points*100:.1f}%)")
        else:
            # Fall back to using all points
            R, t = kabsch_algorithm(P, Q)
            print(f"  RANSAC: Using all {n_points} points (no good inlier set found)")
            best_inliers = np.ones(n_points, dtype=bool)
        
        # Build 4x4 transformation matrix
        X = np.eye(4)
        X[:3, :3] = R
        X[:3, 3] = t
        
        # Compute residual error on inliers only
        P_inliers = P[best_inliers]
        Q_inliers = Q[best_inliers]
        P_transformed = (R @ P_inliers.T).T + t
        errors = np.linalg.norm(P_transformed - Q_inliers, axis=1)
        mean_error = np.mean(errors)
        max_error = np.max(errors)
        
        print(f"  Mean translation error (inliers): {mean_error * 1000:.2f} mm")
        print(f"  Max translation error (inliers): {max_error * 1000:.2f} mm")
        
        return X, mean_error, max_error
    
    print("\nSolving left camera head-to-camera transformation:")
    X_left, left_mean_err, left_max_err = solve_head_to_camera(left_data)
    
    print("\nSolving right camera head-to-camera transformation:")
    X_right, right_mean_err, right_max_err = solve_head_to_camera(right_data)
    
    # Build result
    result = {
        "left": {
            "head_to_camera": X_left.tolist(),
            "mean_error_meters": float(left_mean_err),
            "max_error_meters": float(left_max_err),
            "sample_count": len(left_data)
        },
        "right": {
            "head_to_camera": X_right.tolist(),
            "mean_error_meters": float(right_mean_err),
            "max_error_meters": float(right_max_err),
            "sample_count": len(right_data)
        }
    }
    
    return result


def print_transform_summary(label: str, T: np.ndarray):
    """Print a summary of a 4x4 transformation matrix."""
    # Extract rotation as Euler angles
    R = T[:3, :3]
    rot = Rotation.from_matrix(R)
    euler = rot.as_euler('xyz', degrees=True)
    
    # Extract translation
    t = T[:3, 3]
    
    print(f"\n{label}:")
    print(f"  Translation: x={t[0]*1000:.1f}mm, y={t[1]*1000:.1f}mm, z={t[2]*1000:.1f}mm")
    print(f"  Rotation (XYZ Euler): roll={euler[0]:.1f}°, pitch={euler[1]:.1f}°, yaw={euler[2]:.1f}°")


def main():
    parser = argparse.ArgumentParser(
        description="Stereo camera calibration for Vision Pro"
    )
    parser.add_argument(
        "session_dir",
        type=str,
        help="Path to calibration session directory"
    )
    parser.add_argument(
        "--pattern-size",
        type=str,
        default="5,4",
        help="Checkerboard inner corner size as 'cols,rows' (default: 5,4 for 6x5 squares)"
    )
    parser.add_argument(
        "--square-size",
        type=float,
        default=10.0,
        help="Checkerboard square size in mm (default: 10.0)"
    )
    parser.add_argument(
        "--marker-size",
        type=float,
        default=55.0,
        help="ArUco marker size in mm (default: 55.0)"
    )
    parser.add_argument(
        "--skip-intrinsic",
        action="store_true",
        help="Skip intrinsic calibration and use existing"
    )
    parser.add_argument(
        "--show-detections",
        action="store_true",
        help="Display checkerboard detections during intrinsic calibration"
    )
    parser.add_argument(
        "--use-stored-opencv-poses",
        action="store_true",
        help="Use pre-computed OpenCV poses from data.json instead of re-detecting"
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output file for calibration results (default: calibration_result.json in session dir)"
    )
    
    args = parser.parse_args()
    
    session_dir = Path(args.session_dir)
    if not session_dir.exists():
        raise ValueError(f"Session directory not found: {session_dir}")
    
    # Parse pattern size
    pattern_cols, pattern_rows = map(int, args.pattern_size.split(","))
    pattern_size = (pattern_cols, pattern_rows)
    
    # Load or compute intrinsic calibration
    if args.skip_intrinsic:
        intrinsic_calib = load_intrinsic_calibration(session_dir)
        if intrinsic_calib is None:
            raise ValueError("No existing intrinsic calibration found")
        print("Using existing intrinsic calibration")
    else:
        print("=" * 60)
        print("INTRINSIC CALIBRATION")
        print("=" * 60)
        intrinsic_calib = calibrate_intrinsic(
            session_dir,
            pattern_size=pattern_size,
            square_size_mm=args.square_size,
            show_detections=args.show_detections
        )
        
        # Print summary
        print("\nIntrinsic Calibration Results:")
        for side in ["left", "right"]:
            print(f"\n{side.upper()} Camera:")
            print(f"  Focal length: fx={intrinsic_calib[side]['fx']:.2f}, fy={intrinsic_calib[side]['fy']:.2f}")
            print(f"  Principal point: cx={intrinsic_calib[side]['cx']:.2f}, cy={intrinsic_calib[side]['cy']:.2f}")
            print(f"  RMS error: {intrinsic_calib[side].get('rms_error', 'N/A')}")
    
    # Extrinsic calibration
    print("\n" + "=" * 60)
    print("EXTRINSIC CALIBRATION")
    print("=" * 60)
    
    extrinsic_calib = calibrate_extrinsic(
        session_dir,
        intrinsic_calib,
        recalculate_opencv_poses=not args.use_stored_opencv_poses
    )
    
    # Print extrinsic results
    print_transform_summary(
        "LEFT Camera Head-to-Camera Transform",
        np.array(extrinsic_calib["left"]["head_to_camera"])
    )
    print_transform_summary(
        "RIGHT Camera Head-to-Camera Transform",
        np.array(extrinsic_calib["right"]["head_to_camera"])
    )
    
    # Combine results
    result = {
        "intrinsic": intrinsic_calib,
        "extrinsic": extrinsic_calib
    }
    
    # Save results
    output_file = args.output or str(session_dir / "calibration_result.json")
    with open(output_file, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nCalibration results saved to: {output_file}")

def visualize_detections(
    session_dir: Path,
    intrinsic_calib: dict,
    output_dir: Path = None
) -> None:
    """
    Generate visualization overlays for all extrinsic samples.
    
    Args:
        session_dir: Path to session directory
        intrinsic_calib: Intrinsic calibration dictionary
        output_dir: Output directory for visualizations (default: session_dir/visualizations)
    """
    extrinsic_dir = session_dir / "extrinsic"
    if not extrinsic_dir.exists():
        raise ValueError(f"Extrinsic directory not found: {extrinsic_dir}")
    
    # Output directory
    if output_dir is None:
        output_dir = session_dir / "visualizations"
    output_dir.mkdir(exist_ok=True)
    
    # Load session info for marker size
    session_info = load_session_info(session_dir)
    marker_size = session_info.get("marker_size_meters", 0.055)
    
    # Build camera matrices
    left_mtx = build_camera_matrix(intrinsic_calib["left"])
    left_dist = build_distortion_coeffs(intrinsic_calib["left"])
    right_mtx = build_camera_matrix(intrinsic_calib["right"])
    right_dist = build_distortion_coeffs(intrinsic_calib["right"])
    
    # Find all sample directories
    sample_dirs = sorted(extrinsic_dir.glob("sample_*"))
    print(f"Generating visualizations for {len(sample_dirs)} samples...")
    
    for i, sample_dir in enumerate(sample_dirs):
        data_file = sample_dir / "data.json"
        if not data_file.exists():
            continue
        
        with open(data_file, "r") as f:
            data = json.load(f)
        
        marker_id = data.get("marker_id", 0)
        has_arkit = "arkit_marker_pose" in data
        
        # Load image
        img_file = sample_dir / "frame_stereo.jpg"
        if not img_file.exists():
            continue
        
        img = cv2.imread(str(img_file))
        if img is None:
            continue
        
        left_img, right_img = split_stereo_image(img)
        
        # Detect markers with details
        _, left_corners, left_rvecs = detect_aruco_marker(
            left_img, left_mtx, left_dist, marker_size, [marker_id], return_details=True
        )
        _, right_corners, right_rvecs = detect_aruco_marker(
            right_img, right_mtx, right_dist, marker_size, [marker_id], return_details=True
        )
        
        # Create visualizations
        arkit_status = "ARKit:YES" if has_arkit else "ARKit:NO"
        left_vis = visualize_marker_detection(
            left_img, left_corners, left_rvecs, left_mtx, left_dist, marker_size,
            f"LEFT - Sample {i+1} - Marker {marker_id} - {arkit_status}"
        )
        right_vis = visualize_marker_detection(
            right_img, right_corners, right_rvecs, right_mtx, right_dist, marker_size,
            f"RIGHT - Sample {i+1} - Marker {marker_id} - {arkit_status}"
        )
        
        # Combine side by side
        combined = np.hstack([left_vis, right_vis])
        
        # Add detection status bar at bottom
        status_bar = np.zeros((40, combined.shape[1], 3), dtype=np.uint8)
        left_det = "LEFT: DETECTED" if left_corners else "LEFT: MISSED"
        right_det = "RIGHT: DETECTED" if right_corners else "RIGHT: MISSED"
        left_color = (0, 255, 0) if left_corners else (0, 0, 255)
        right_color = (0, 255, 0) if right_corners else (0, 0, 255)
        cv2.putText(status_bar, left_det, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, left_color, 2)
        cv2.putText(status_bar, right_det, (combined.shape[1]//2 + 10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, right_color, 2)
        
        combined = np.vstack([combined, status_bar])
        
        # Save
        output_file = output_dir / f"sample_{i+1:04d}.jpg"
        cv2.imwrite(str(output_file), combined)
        
        if (i + 1) % 10 == 0:
            print(f"  Processed {i+1}/{len(sample_dirs)} samples")
    
    print(f"\nVisualizations saved to: {output_dir}")
    print(f"You can view them with: open {output_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Stereo camera calibration for Vision Pro"
    )
    parser.add_argument(
        "session_dir",
        type=str,
        help="Path to calibration session directory"
    )
    parser.add_argument(
        "--pattern-size",
        type=str,
        default="5,4",
        help="Checkerboard inner corner size as 'cols,rows' (default: 5,4 for 6x5 squares)"
    )
    parser.add_argument(
        "--square-size",
        type=float,
        default=10.0,
        help="Checkerboard square size in mm (default: 10.0)"
    )
    parser.add_argument(
        "--marker-size",
        type=float,
        default=55.0,
        help="ArUco marker size in mm (default: 55.0)"
    )
    parser.add_argument(
        "--skip-intrinsic",
        action="store_true",
        help="Skip intrinsic calibration and use existing"
    )
    parser.add_argument(
        "--show-detections",
        action="store_true",
        help="Display checkerboard detections during intrinsic calibration"
    )
    parser.add_argument(
        "--use-stored-opencv-poses",
        action="store_true",
        help="Use pre-computed OpenCV poses from data.json instead of re-detecting"
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Generate marker detection visualizations for all extrinsic samples"
    )
    parser.add_argument(
        "--joint-calibration",
        action="store_true",
        help="Use joint intrinsic/extrinsic optimization with position-only cost (optimizes cameras independently)"
    )
    parser.add_argument(
        "--joint-stereo",
        action="store_true",
        help="RECOMMENDED: Joint stereo calibration with shared intrinsics (most robust, enforces physical constraints)"
    )
    parser.add_argument(
        "--extrinsic-only",
        action="store_true",
        help="When using --joint-calibration, only optimize extrinsics (keep intrinsics fixed)"
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output file for calibration results (default: calibration_result.json in session dir)"
    )
    parser.add_argument(
        "--max-iter",
        type=int,
        default=200,
        help="Maximum iterations for differential evolution optimizer (default: 200)"
    )
    parser.add_argument(
        "--use-ray-cost",
        action="store_true",
        help="Use ray-based cost function instead of PnP position error (more robust to noisy rotation)"
    )
    
    args = parser.parse_args()
    
    session_dir = Path(args.session_dir)
    if not session_dir.exists():
        raise ValueError(f"Session directory not found: {session_dir}")
    
    # Parse pattern size
    pattern_cols, pattern_rows = map(int, args.pattern_size.split(","))
    pattern_size = (pattern_cols, pattern_rows)
    
    # Load or compute intrinsic calibration
    if args.skip_intrinsic or args.visualize or args.joint_calibration or args.joint_stereo:
        intrinsic_calib = load_intrinsic_calibration(session_dir)
        if intrinsic_calib is None:
            # Need to run intrinsic first
            print("=" * 60)
            print("INTRINSIC CALIBRATION")
            print("=" * 60)
            intrinsic_calib = calibrate_intrinsic(
                session_dir,
                pattern_size=pattern_size,
                square_size_mm=args.square_size,
                show_detections=args.show_detections
            )
            # Save it for later use
            with open(session_dir / "intrinsic_calibration.json", "w") as f:
                json.dump(intrinsic_calib, f, indent=2)
            print(f"Intrinsic calibration saved to: {session_dir / 'intrinsic_calibration.json'}")
        else:
            print("Using existing intrinsic calibration")
    else:
        print("=" * 60)
        print("INTRINSIC CALIBRATION")
        print("=" * 60)
        intrinsic_calib = calibrate_intrinsic(
            session_dir,
            pattern_size=pattern_size,
            square_size_mm=args.square_size,
            show_detections=args.show_detections
        )
        
        # Print summary
        print("\nIntrinsic Calibration Results:")
        for side in ["left", "right"]:
            print(f"\n{side.upper()} Camera:")
            print(f"  Focal length: fx={intrinsic_calib[side]['fx']:.2f}, fy={intrinsic_calib[side]['fy']:.2f}")
            print(f"  Principal point: cx={intrinsic_calib[side]['cx']:.2f}, cy={intrinsic_calib[side]['cy']:.2f}")
            print(f"  RMS error: {intrinsic_calib[side].get('rms_error', 'N/A')}")
    
    # If --visualize flag, just generate visualizations and exit
    if args.visualize:
        print("\n" + "=" * 60)
        print("GENERATING VISUALIZATIONS")
        print("=" * 60)
        visualize_detections(session_dir, intrinsic_calib)
        exit(0)
    
    # Choose calibration method
    if args.joint_stereo:
        # RECOMMENDED: Joint stereo with shared intrinsics
        joint_result = calibrate_joint_stereo(
            session_dir,
            intrinsic_calib,
            max_iterations=args.max_iter,
            use_ray_cost=args.use_ray_cost
        )
        
        # Print results
        for side in ["left", "right"]:
            if side in joint_result:
                print_transform_summary(
                    f"{side.upper()} Camera Head-to-Camera Transform (Joint Stereo)",
                    np.array(joint_result[side]["head_to_camera"])
                )
        
        # Build final result with shared intrinsics
        final_intrinsic = {
            "image_width": intrinsic_calib.get("image_width", 0),
            "image_height": intrinsic_calib.get("image_height", 0),
            "left": joint_result["shared_intrinsics"],
            "right": joint_result["shared_intrinsics"]
        }
        
        extrinsic_calib = {
            side: {
                "head_to_camera": joint_result[side]["head_to_camera"],
                "mean_error_meters": joint_result[side]["mean_error_meters"],
                "max_error_meters": joint_result[side]["max_error_meters"],
                "sample_count": joint_result[side]["sample_count"]
            }
            for side in ["left", "right"]
        }
        
        result = {
            "intrinsic": final_intrinsic,
            "extrinsic": extrinsic_calib,
            "method": "joint_stereo_shared_intrinsics"
        }
    elif args.joint_calibration:
        # Joint optimization approach (cameras independently)
        print("\n" + "=" * 60)
        print("JOINT CALIBRATION (Position-Only Cost)")
        print("=" * 60)
        
        optimize_intrinsics = not args.extrinsic_only
        joint_result = calibrate_joint(
            session_dir,
            intrinsic_calib,
            optimize_intrinsics=optimize_intrinsics
        )
        
        # Print results
        for side in ["left", "right"]:
            if side in joint_result:
                print_transform_summary(
                    f"{side.upper()} Camera Head-to-Camera Transform (Joint)",
                    np.array(joint_result[side]["head_to_camera"])
                )
        
        # Build final result - use joint-optimized intrinsics if available
        if optimize_intrinsics:
            final_intrinsic = {
                "image_width": intrinsic_calib.get("image_width", 0),
                "image_height": intrinsic_calib.get("image_height", 0),
            }
            for side in ["left", "right"]:
                if side in joint_result:
                    final_intrinsic[side] = joint_result[side]["intrinsics"]
                else:
                    final_intrinsic[side] = intrinsic_calib[side]
        else:
            final_intrinsic = intrinsic_calib
        
        extrinsic_calib = {
            side: {
                "head_to_camera": joint_result[side]["head_to_camera"],
                "mean_error_meters": joint_result[side]["mean_error_meters"],
                "max_error_meters": joint_result[side]["max_error_meters"],
                "median_error_meters": joint_result[side]["median_error_meters"],
                "sample_count": joint_result[side]["sample_count"]
            }
            for side in joint_result
        }
        
        result = {
            "intrinsic": final_intrinsic,
            "extrinsic": extrinsic_calib,
            "method": "joint_position_only",
            "optimize_intrinsics": optimize_intrinsics
        }
    else:
        # Standard Kabsch approach
        print("\n" + "=" * 60)
        print("EXTRINSIC CALIBRATION (Kabsch)")
        print("=" * 60)
        
        extrinsic_calib = calibrate_extrinsic(
            session_dir,
            intrinsic_calib,
            recalculate_opencv_poses=not args.use_stored_opencv_poses
        )
        
        # Print extrinsic results
        print_transform_summary(
            "LEFT Camera Head-to-Camera Transform",
            np.array(extrinsic_calib["left"]["head_to_camera"])
        )
        print_transform_summary(
            "RIGHT Camera Head-to-Camera Transform",
            np.array(extrinsic_calib["right"]["head_to_camera"])
        )
        
        result = {
            "intrinsic": intrinsic_calib,
            "extrinsic": extrinsic_calib,
            "method": "kabsch"
        }
    
    # Save results
    output_file = args.output or str(session_dir / "calibration_result.json")
    with open(output_file, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nCalibration results saved to: {output_file}")

