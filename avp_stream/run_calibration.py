#!/usr/bin/env python3
"""
Camera Calibration Runner
=========================

Interactive tool for complete camera calibration workflow:
1. Camera discovery and selection
2. Intrinsic calibration (checkerboard/ChArUco)
3. Intrinsic verification (ArUco distance check)
4. Extrinsic calibration (with Vision Pro)

Usage:
    python -m avp_stream.run_calibration
    python -m avp_stream.run_calibration --camera 0 --stereo

Prerequisites:
    - Print calibration sheets: python -m avp_stream.prepare_calibration
    - Have Vision Pro running Tracking Streamer app for extrinsic calibration
"""

import cv2
import numpy as np
import time
import argparse
import sys
import os
import threading
from typing import List, Dict, Optional, Tuple, Union, Any
from dataclasses import dataclass, field
import json
from threading import Thread, Lock

# Calibration parameters (matching prepare_calibration.py)
CHARUCO_ROWS = 7
CHARUCO_COLS = 5
CHARUCO_SQUARE_SIZE = 0.030  # 30mm
CHARUCO_MARKER_SIZE = 0.023  # 23mm

VERIFICATION_TAG_SIZE = 0.100   # 100mm
VERIFICATION_MARGIN = 0.060     # 60mm
VERIFICATION_TAG_IDS = [0, 2, 3]

EXTRINSIC_MARKER_SIZE_PRINT = 0.100   # 100mm for paper mode
EXTRINSIC_MARKER_SIZE_IPHONE = 0.055  # 55mm for iPhone mode
EXTRINSIC_MARKER_IDS = [0, 2, 3]

# iPhone Mode Parameters
IPHONE_CHECKER_COLS = 6
IPHONE_CHECKER_ROWS = 12
IPHONE_CHECKER_SIZE = 0.010  # 10mm

IPHONE_VERIF_TAG_SIZE = 0.040    # 40mm
IPHONE_VERIF_TAG_GAP = 0.006     # 6mm gap between markers
IPHONE_VERIF_TAG_SPACING = 0.046 # 46mm center-to-center (40mm + 6mm)
IPHONE_VERIF_TAG_IDS = [0, 1, 2]  # 3 tags vertically

# Extrinsic is same for both modes


@dataclass
class CalibrationResult:
    """Stores calibration results for a single camera."""
    camera_matrix: Optional[np.ndarray] = None
    distortion: Optional[np.ndarray] = None
    image_size: Optional[Tuple[int, int]] = None
    reprojection_error: float = float('inf')
    n_samples: int = 0


@dataclass 
class ExtrinsicSample:
    """A single extrinsic calibration sample."""
    T_b_h: np.ndarray  # Head pose from AVP (body frame)
    T_c_m: np.ndarray  # Marker pose from camera (OpenCV)
    T_b_m: np.ndarray  # Marker pose from ARKit (body frame)
    marker_id: int
    timestamp: float


# =============================================================================
# Camera Discovery
# =============================================================================

def discover_cameras(max_index: int = 10) -> List[Dict]:
    """
    Discover available cameras.
    
    Args:
        max_index: Maximum camera index to probe
    
    Returns:
        List of dicts with camera info: {index, width, height, name}
    """
    cameras = []
    
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            # Try to get camera name (platform-dependent)
            name = f"Camera {i}"
            
            cameras.append({
                "index": i,
                "width": width,
                "height": height,
                "name": name,
            })
            cap.release()
    
    return cameras


def select_camera(cameras: List[Dict]) -> Tuple[Optional[int], bool]:
    """
    Interactive camera selection with simultaneous multi-camera preview grid.
    
    Returns:
        Tuple of (camera_index, is_stereo) or (None, False) if cancelled
    """
    if not cameras:
        print("No cameras found!")
        return None, False
    
    print("\n" + "=" * 60)
    print("CAMERA SELECTION")
    print("=" * 60)
    print("All cameras displayed simultaneously.")
    print("Press number key (1-9) or click on camera to select.")
    print("Press 'q' or ESC to cancel.")
    print()
    
    # Open all cameras with LOW resolution for fast preview
    caps = []
    resolutions = []  # Store actual resolutions
    for cam in cameras:
        cap = cv2.VideoCapture(cam["index"])
        if cap.isOpened():
            # Use LOW resolution for fast grid preview
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            # Reduce buffer size for lower latency
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            resolutions.append((actual_w, actual_h))
        else:
            resolutions.append((0, 0))
        caps.append(cap)
    
    # Calculate grid layout
    n_cameras = len(cameras)
    if n_cameras == 1:
        grid_cols, grid_rows = 1, 1
    elif n_cameras == 2:
        grid_cols, grid_rows = 2, 1
    elif n_cameras <= 4:
        grid_cols, grid_rows = 2, 2
    elif n_cameras <= 6:
        grid_cols, grid_rows = 3, 2
    else:
        grid_cols, grid_rows = 3, 3
    
    # Smaller cell size for faster rendering
    cell_width = 480
    cell_height = 360
    
    selected_idx = None
    mouse_selection = [None]  # Use list for closure
    
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Determine which cell was clicked
            col = x // cell_width
            row = y // cell_height
            idx = row * grid_cols + col
            if idx < len(cameras):
                mouse_selection[0] = idx
    
    cv2.namedWindow("Camera Selection - Click or Press Number", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Camera Selection - Click or Press Number", mouse_callback)
    
    # Pre-allocate canvas
    canvas_width = grid_cols * cell_width
    canvas_height = grid_rows * cell_height
    canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)
    
    while True:
        # Clear canvas
        canvas.fill(0)
        
        # Capture and display each camera
        for i, (cap, cam) in enumerate(zip(caps, cameras)):
            row = i // grid_cols
            col = i % grid_cols
            
            x_offset = col * cell_width
            y_offset = row * cell_height
            
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    # Resize to fit cell
                    frame = cv2.resize(frame, (cell_width, cell_height))
                else:
                    frame = np.zeros((cell_height, cell_width, 3), dtype=np.uint8)
                    cv2.putText(frame, "No Signal", (cell_width//2 - 80, cell_height//2),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                frame = np.zeros((cell_height, cell_width, 3), dtype=np.uint8)
                cv2.putText(frame, "Failed to Open", (cell_width//2 - 100, cell_height//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            actual_w, actual_h = resolutions[i]
            
            # Draw overlay with camera info
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (cell_width, 60), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
            
            # Camera number (large, prominent)
            num_text = f"[{i + 1}]"
            cv2.putText(frame, num_text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            
            # Camera info with stereo hint
            aspect = actual_w / actual_h if actual_h > 0 else 0
            stereo_hint = " (STEREO?)" if aspect > 1.8 else ""
            info_text = f"Cam {cam['index']} | {actual_w}x{actual_h}{stereo_hint}"
            cv2.putText(frame, info_text, (80, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            
            # Border
            cv2.rectangle(frame, (0, 0), (cell_width - 1, cell_height - 1), (100, 100, 100), 2)
            
            # Place in canvas
            canvas[y_offset:y_offset + cell_height, x_offset:x_offset + cell_width] = frame
        
        cv2.imshow("Camera Selection - Click or Press Number", canvas)
        
        # Check mouse selection
        if mouse_selection[0] is not None:
            selected_idx = mouse_selection[0]
            print(f"\nSelected camera index {cameras[selected_idx]['index']} (clicked)")
            break
        
        key = cv2.waitKey(1) & 0xFF  # Faster polling
        
        # Number keys 1-9
        if ord('1') <= key <= ord('9'):
            idx = key - ord('1')
            if idx < len(cameras):
                selected_idx = idx
                print(f"\nSelected camera index {cameras[idx]['index']} (key {idx + 1})")
                break
        
        # Cancel
        elif key in [ord('q'), ord('Q'), 27]:  # Q or ESC
            print("\nCamera selection cancelled")
            break
    
    # Cleanup
    for cap in caps:
        if cap.isOpened():
            cap.release()
    cv2.destroyAllWindows()
    
    if selected_idx is None:
        return None, False
    
    camera_index = cameras[selected_idx]["index"]
    actual_w, actual_h = resolutions[selected_idx]
    
    # Detect stereo based on aspect ratio (2:1 or close)
    aspect = actual_w / actual_h if actual_h > 0 else 0
    is_likely_stereo = aspect > 1.8  # 2:1 aspect ratio suggests side-by-side stereo
    
    # Ask user to confirm stereo mode
    print()
    if is_likely_stereo:
        print(f"Detected aspect ratio {aspect:.2f}:1 - this looks like a STEREO camera.")
        response = input("Is this a stereo (side-by-side) camera? [Y/n]: ").strip().lower()
        is_stereo = response != 'n'
    else:
        print(f"Detected aspect ratio {aspect:.2f}:1 - this looks like a MONO camera.")
        response = input("Is this a stereo (side-by-side) camera? [y/N]: ").strip().lower()
        is_stereo = response == 'y'
    
    return camera_index, is_stereo




# =============================================================================
# Intrinsic Calibration
# =============================================================================

def split_stereo_image(img: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Split side-by-side stereo image into left and right."""
    h, w = img.shape[:2]
    mid = w // 2
    return img[:, :mid], img[:, mid:]


class IntrinsicCalibrator:
    """
    Handles intrinsic camera calibration using ChArUco boards or plain checkerboards.
    
    Features:
    - Auto-capture based on board movement
    - Real-time reprojection error feedback
    - Convergence detection
    - Stereo camera support
    - Dual mode: ChArUco (print) or Checkerboard (iPhone)
    """
    
    def __init__(
        self,
        camera_id: int,
        stereo: bool = False,
        mode: str = "charuco",  # "charuco" or "checkerboard"
        rows: int = CHARUCO_ROWS,
        cols: int = CHARUCO_COLS,
        square_size: float = CHARUCO_SQUARE_SIZE,
        marker_size: float = CHARUCO_MARKER_SIZE,
    ):
        self.camera_id = camera_id
        self.stereo = stereo
        self.mode = mode
        self.rows = rows
        self.cols = cols
        self.square_size = square_size
        self.marker_size = marker_size
        
        # Mode-specific setup
        if mode == "checkerboard":
            # Plain checkerboard: rows/cols are total squares, inner corners = (cols-1, rows-1)
            self.pattern_size = (cols - 1, rows - 1)
            # Generate object points
            self.objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
            self.objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
            self.objp *= square_size
            self.board = None
            self.dictionary = None
            self.detector = None
        else:
            # ChArUco board
            self.pattern_size = None
            self.objp = None
            self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.board = cv2.aruco.CharucoBoard(
                (cols, rows), square_size, marker_size, self.dictionary
            )
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary, cv2.aruco.DetectorParameters()
            )
        
        # Storage for collected samples
        self.collections = {
            "left": {"imgpoints": [], "objpoints": [], "ids": [], "size": None},
            "right": {"imgpoints": [], "objpoints": [], "ids": [], "size": None},
        }
        
        # Calibration state
        self.current_result = {"left": CalibrationResult(), "right": CalibrationResult()}
        self.fx_history = {"left": [], "right": []}
        
        # Auto-capture settings
        self.auto_capture_interval = 0.5  # seconds
        self.min_corner_movement = 15.0   # pixels
        self.last_capture_time = {"left": 0, "right": 0}
        self.last_corners = {"left": None, "right": None}
        
        # Background calibration
        self._calibration_lock = Lock()
        self._calibrating = {"left": False, "right": False}
    
    def _corner_movement(self, corners_new, corners_old) -> float:
        """Calculate average corner movement."""
        if corners_old is None or corners_new is None:
            return float('inf')
        if len(corners_new) != len(corners_old):
            return float('inf')
        try:
            diff = np.linalg.norm(
                corners_new.reshape(-1, 2) - corners_old.reshape(-1, 2), axis=1
            )
            return np.mean(diff)
        except:
            return float('inf')
    
    def _compute_convergence(self, history: List[float]) -> Tuple[float, bool]:
        """Compute calibration convergence from fx history."""
        if len(history) < 5:
            return 0, False
        
        recent = history[-10:] if len(history) >= 10 else history
        mean_fx = np.mean(recent)
        std_fx = np.std(recent)
        
        cv = (std_fx / mean_fx * 100) if mean_fx > 0 else 100
        stability = max(0, min(100, 100 - cv * 20))
        ready = stability > 90 and len(history) >= 20
        
        return stability, ready
    
    def _run_background_calibration(self, cam_name: str):
        """Run calibration in background thread."""
        try:
            # Make a copy of data under lock
            with self._calibration_lock:
                data = self.collections[cam_name]
                if len(data["imgpoints"]) < 5:
                    self._calibrating[cam_name] = False
                    return
                
                # Deep copy lists to avoid race conditions with main thread appending
                imgpoints = list(data["imgpoints"])
                objpoints = list(data["objpoints"]) if data["objpoints"] else []
                ids = list(data["ids"]) if data["ids"] else []
                image_size = data["size"]
            
            if self.mode == "checkerboard":
                # Standard checkerboard calibration
                ret, mtx, dist, _, _ = cv2.calibrateCamera(
                    objpoints,
                    imgpoints,
                    image_size,
                    None, None
                )
            else:
                # ChArUco calibration
                ret, mtx, dist, _, _ = cv2.aruco.calibrateCameraCharuco(
                    imgpoints,
                    ids,
                    self.board,
                    image_size,
                    None, None
                )
            
            with self._calibration_lock:
                self.current_result[cam_name].camera_matrix = mtx
                self.current_result[cam_name].distortion = dist
                self.current_result[cam_name].reprojection_error = ret
                self.current_result[cam_name].image_size = image_size
                self.current_result[cam_name].n_samples = len(imgpoints)
                self.fx_history[cam_name].append(mtx[0, 0])
        except Exception as e:
            # print(f"Calibration failed: {e}")
            pass
        finally:
            self._calibrating[cam_name] = False
    
    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Process a single frame for calibration.
        
        Returns:
            vis_frame: Visualization frame with overlays
            status: Dict with detection and calibration status
        """
        vis_frame = frame.copy()
        status = {"detected": {}, "captured": {}, "convergence": {}}
        
        # Split if stereo
        views = {}
        if self.stereo:
            left, right = split_stereo_image(frame)
            views["left"] = left
            views["right"] = right
        else:
            views["left"] = frame
        
        current_time = time.time()
        
        for name, img in views.items():
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Update size (safe, just overwrites tuple)
            self.collections[name]["size"] = gray.shape[::-1]
            
            detected = False
            detected_corners = None
            charuco_ids = None
            
            if self.mode == "checkerboard":
                # Plain checkerboard detection
                ret, corners = cv2.findChessboardCorners(
                    gray, self.pattern_size,
                    cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                )
                
                if ret:
                    # Subpixel refinement
                    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    detected_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
                    detected = True
                    
                    # Draw on visualization
                    draw_offset = 0 if not self.stereo else (0 if name == "left" else frame.shape[1] // 2)
                    draw_region = vis_frame[:, draw_offset:draw_offset + img.shape[1]]
                    cv2.drawChessboardCorners(draw_region, self.pattern_size, detected_corners, True)
            else:
                # ChArUco detection
                corners, ids, _ = self.detector.detectMarkers(gray)
                
                if len(corners) > 0:
                    try:
                        ret_val, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                            corners, ids, gray, self.board
                        )
                        
                        if ret_val >= 4:
                            # Subpixel refinement
                            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                            detected_corners = cv2.cornerSubPix(
                                gray, charuco_corners, (5, 5), (-1, -1), term
                            )
                            detected = True
                            
                            # Draw on visualization
                            draw_offset = 0 if not self.stereo else (0 if name == "left" else frame.shape[1] // 2)
                            draw_region = vis_frame[:, draw_offset:draw_offset + img.shape[1]]
                            cv2.aruco.drawDetectedCornersCharuco(
                                draw_region, detected_corners, charuco_ids, (0, 0, 255)
                            )
                    except Exception:
                        pass
            
            status["detected"][name] = detected
            status["captured"][name] = False
            
            # Auto-capture logic
            if detected and detected_corners is not None:
                movement = self._corner_movement(detected_corners, self.last_corners[name])
                time_since_last = current_time - self.last_capture_time[name]
                
                should_capture = (
                    time_since_last >= self.auto_capture_interval and
                    (self.last_corners[name] is None or movement >= self.min_corner_movement)
                )
                
                if should_capture:
                    # Protect append with lock
                    with self._calibration_lock:
                        self.collections[name]["imgpoints"].append(detected_corners)
                        if self.mode == "checkerboard":
                            self.collections[name]["objpoints"].append(self.objp)
                        else:
                            self.collections[name]["ids"].append(charuco_ids)
                    
                    self.last_capture_time[name] = current_time
                    self.last_corners[name] = detected_corners.copy()
                    status["captured"][name] = True
                    
                    # Trigger background calibration
                    # Use lock to read length safe-ishly (though standard int read is atomic enough, being safe is better)
                    with self._calibration_lock:
                        n_samples = len(self.collections[name]["imgpoints"])
                    
                    if n_samples >= 5 and not self._calibrating[name]:
                        self._calibrating[name] = True
                        thread = Thread(
                            target=self._run_background_calibration,
                            args=(name,)
                        )
                        thread.daemon = True
                        thread.start()
            
            # Compute convergence
            stability, ready = self._compute_convergence(self.fx_history[name])
            status["convergence"][name] = {"stability": stability, "ready": ready}
        
        # Draw status overlay
        self._draw_status_overlay(vis_frame, status)
        
        return vis_frame, status
    
    def _draw_status_overlay(self, frame: np.ndarray, status: Dict):
        """Draw calibration status on frame."""
        y = 30
        
        def error_color(err):
            if err == float('inf'):
                return (128, 128, 128)
            if err < 0.5:
                return (0, 255, 0)
            if err < 1.0:
                return (0, 255, 255)
            if err < 2.0:
                return (0, 165, 255)
            return (0, 0, 255)
        
        # Title
        targets = ["left", "right"] if self.stereo else ["left"]
        counts = [len(self.collections[t]["imgpoints"]) for t in targets]
        title = f"ChArUco Calibration | Samples: L={counts[0]}"
        if self.stereo:
            title += f" R={counts[1]}"
        cv2.putText(frame, title, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        y += 30
        
        # Per-camera stats
        for name in targets:
            with self._calibration_lock:
                result = self.current_result[name]
            
            if result.camera_matrix is not None:
                fx = result.camera_matrix[0, 0]
                fy = result.camera_matrix[1, 1]
                err = result.reprojection_error
                
                text = f"{name.upper()}: fx={fx:.1f} fy={fy:.1f} err={err:.3f}"
                cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, error_color(err), 2)
                y += 25
        
        # Convergence
        for name in targets:
            conv = status["convergence"][name]
            text = f"{name.upper()} Stability: {conv['stability']:.0f}%"
            if conv["ready"]:
                text += " - READY"
                color = (0, 255, 0)
            else:
                color = (0, 255, 255)
            cv2.putText(frame, text, (10, frame.shape[0] - 40 + (0 if name == "left" else 25)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Instructions
        cv2.putText(frame, "Press 'q' to finish, 'r' to reset",
                   (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    def get_results(self) -> Dict[str, CalibrationResult]:
        """Get current calibration results."""
        with self._calibration_lock:
            return {k: CalibrationResult(
                camera_matrix=v.camera_matrix.copy() if v.camera_matrix is not None else None,
                distortion=v.distortion.copy() if v.distortion is not None else None,
                image_size=v.image_size,
                reprojection_error=v.reprojection_error,
                n_samples=v.n_samples,
            ) for k, v in self.current_result.items()}
    
    def reset(self):
        """Reset all collected samples."""
        for name in self.collections:
            self.collections[name]["imgpoints"] = []
            self.collections[name]["ids"] = []
        self.last_corners = {"left": None, "right": None}
        self.last_capture_time = {"left": 0, "right": 0}
        self.fx_history = {"left": [], "right": []}
        with self._calibration_lock:
            self.current_result = {"left": CalibrationResult(), "right": CalibrationResult()}


def run_intrinsic_calibration(
    camera_id: int,
    stereo: bool = False,
) -> Optional[Dict[str, CalibrationResult]]:
    """
    Run interactive intrinsic calibration.
    
    Returns:
        Dict of CalibrationResult per camera, or None if cancelled
    """
    print("\n" + "=" * 60)
    print("INTRINSIC CALIBRATION")
    print("=" * 60)
    print("Hold the ChArUco board in front of the camera.")
    print("Move it around slowly to capture different angles.")
    print("Press 'q' when convergence is high (>90%).")
    print("Press 'r' to reset and start over.")
    print()
    
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_id}")
        return None
    
    # Set high resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080 if stereo else 2160)
    
    calibrator = IntrinsicCalibrator(camera_id, stereo)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        vis_frame, status = calibrator.process_frame(frame)
        
        cv2.imshow("Intrinsic Calibration", vis_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            print("Resetting calibration...")
            calibrator.reset()
    
    cap.release()
    cv2.destroyAllWindows()
    
    results = calibrator.get_results()
    
    # Check if we have valid results
    valid = any(r.camera_matrix is not None for r in results.values())
    if not valid:
        print("No valid calibration obtained.")
        return None
    
    return results


# =============================================================================
# Intrinsic Verification
# =============================================================================

def run_intrinsic_verification(
    camera_id: int,
    calibration: Dict[str, CalibrationResult],
    stereo: bool = False,
) -> bool:
    """
    Verify intrinsic calibration using ArUco distance check.
    
    Returns:
        True if verification passed, False otherwise
    """
    print("\n" + "=" * 60)
    print("INTRINSIC VERIFICATION")
    print("=" * 60)
    print("Hold the 2x2 ArUco verification grid in front of the camera.")
    print("Check that measured distances match expected values.")
    print("Press 'q' to continue, 'r' to retry calibration.")
    print()
    
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_id}")
        return False
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080 if stereo else 2160)
    
    # Get calibration for current view
    result = calibration["left"]
    if result.camera_matrix is None:
        print("No calibration available for verification!")
        cap.release()
        return False
    
    camera_matrix = result.camera_matrix
    dist_coeffs = result.distortion
    
    # Expected distances
    center_dist = VERIFICATION_TAG_SIZE + VERIFICATION_MARGIN
    expected_adjacent = center_dist  # Adjacent tags (horizontal/vertical)
    expected_diagonal = center_dist * np.sqrt(2)  # Diagonal tags
    
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    
    passed = False
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Use left view for stereo
        if stereo:
            frame = frame[:, :frame.shape[1]//2]
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        
        vis_frame = frame.copy()
        cv2.aruco.drawDetectedMarkers(vis_frame, corners, ids)
        
        # Solve poses and compute distances
        tag_centers = {}
        if ids is not None:
            for i, corner in enumerate(corners):
                tag_id = ids[i][0]
                if tag_id in VERIFICATION_TAG_IDS:
                    # Solve PnP
                    half = VERIFICATION_TAG_SIZE / 2
                    obj_pts = np.array([
                        [-half, half, 0], [half, half, 0],
                        [half, -half, 0], [-half, -half, 0]
                    ], dtype=np.float32)
                    
                    success, rvec, tvec = cv2.solvePnP(
                        obj_pts, corner.reshape(-1, 2),
                        camera_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    
                    if success:
                        tag_centers[tag_id] = tvec.flatten()
                        cv2.drawFrameAxes(vis_frame, camera_matrix, dist_coeffs,
                                         rvec, tvec, VERIFICATION_TAG_SIZE * 0.5)
        
        # Compute and display distances
        y = 30
        cv2.putText(vis_frame, f"Verification: {len(tag_centers)}/4 tags detected",
                   (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        y += 30
        
        errors = []
        if len(tag_centers) >= 2:
            tag_ids_found = sorted(tag_centers.keys())
            for i, id1 in enumerate(tag_ids_found):
                for id2 in tag_ids_found[i+1:]:
                    dist = np.linalg.norm(tag_centers[id1] - tag_centers[id2])
                    
                    # Determine expected distance
                    # Adjacent if same row or same column
                    pos1 = (id1 % 2, id1 // 2)
                    pos2 = (id2 % 2, id2 // 2)
                    dx, dy = abs(pos1[0] - pos2[0]), abs(pos1[1] - pos2[1])
                    
                    if dx + dy == 1:  # Adjacent
                        expected = expected_adjacent
                    else:  # Diagonal
                        expected = expected_diagonal
                    
                    error_mm = abs(dist - expected) * 1000
                    error_pct = abs(dist - expected) / expected * 100
                    errors.append(error_pct)
                    
                    color = (0, 255, 0) if error_pct < 3 else ((0, 255, 255) if error_pct < 5 else (0, 0, 255))
                    text = f"{id1}-{id2}: {dist*1000:.1f}mm (exp: {expected*1000:.1f}mm, err: {error_mm:.1f}mm)"
                    cv2.putText(vis_frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    y += 22
        
        if errors:
            avg_error = np.mean(errors)
            status = "PASS" if avg_error < 5 else "FAIL"
            color = (0, 255, 0) if avg_error < 5 else (0, 0, 255)
            cv2.putText(vis_frame, f"Avg Error: {avg_error:.1f}% - {status}",
                       (10, vis_frame.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            passed = avg_error < 5
        
        cv2.putText(vis_frame, "Press 'q' to continue, 'r' to retry calibration",
                   (10, vis_frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        cv2.imshow("Intrinsic Verification", vis_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            cap.release()
            cv2.destroyAllWindows()
            return False  # Signal to retry calibration
    
    cap.release()
    cv2.destroyAllWindows()
    return True


# =============================================================================
# Extrinsic Calibration
# =============================================================================

class ExtrinsicCalibrator:
    """
    Collects synchronized data for head-to-camera (T_h_c) calibration.
    
    For each marker observation:
    - T_b_h: Head pose from VisionOS (body frame)
    - T_c_m: Marker pose from camera (OpenCV)
    - T_b_m: Marker pose from ARKit (body frame)
    
    Uses Kabsch algorithm on positions only for robustness.
    Runs continuous background optimization thread.
    """
    
    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        marker_size: float,
    ):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_size = marker_size
        
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary, cv2.aruco.DetectorParameters()
        )
        
        self.samples: List[ExtrinsicSample] = []
        self.last_loss: Optional[float] = None
        self.latest_result: Optional[np.ndarray] = None
        
        # Background optimization thread
        self._running = True
        self._samples_lock = threading.Lock()
        self._calibration_thread = threading.Thread(target=self._run_background_calibration, daemon=True)
        self._calibration_thread.start()
    
    def _run_background_calibration(self):
        """Background thread that continuously optimizes T_h_c."""
        last_sample_count = 0
        while self._running:
            time.sleep(0.5)  # Run at ~2Hz
            
            with self._samples_lock:
                current_count = len(self.samples)
                if current_count < 10 or current_count == last_sample_count:
                    continue
                samples_copy = list(self.samples)
            
            last_sample_count = current_count
            
            try:
                # Compute T_h_c from samples
                result = self._compute_T_h_c_internal(samples_copy)
                if result is not None:
                    self.latest_result = result
            except Exception as e:
                pass  # Silent fail
    
    def stop(self):
        """Stop the background thread."""
        self._running = False
        if self._calibration_thread.is_alive():
            self._calibration_thread.join(timeout=1.0)
    
    def detect_marker_opencv(
        self, 
        frame: np.ndarray, 
        target_id: int
    ) -> Optional[np.ndarray]:
        """
        Detect a specific marker in the camera frame.
        
        Returns:
            4x4 transformation matrix T_c_m, or None if not detected
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        if ids is None:
            return None
        
        for i, corner in enumerate(corners):
            if ids[i][0] == target_id:
                half = self.marker_size / 2
                obj_pts = np.array([
                    [-half, half, 0], [half, half, 0],
                    [half, -half, 0], [-half, -half, 0]
                ], dtype=np.float32)
                
                success, rvec, tvec = cv2.solvePnP(
                    obj_pts, corner.reshape(-1, 2),
                    self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                if success:
                    R, _ = cv2.Rodrigues(rvec)
                    T = np.eye(4)
                    T[:3, :3] = R
                    T[:3, 3] = tvec.flatten()
                    return T
        
        return None
    
    def collect_sample(
        self,
        frame: np.ndarray, 
        marker_id: int,
        T_b_h: np.ndarray,
        T_b_m: np.ndarray,
        save: bool = True,
    ) -> Optional[ExtrinsicSample]:
        """
        Detect marker and optionally collect sample.
        
        Args:
            frame: Image frame
            marker_id: Target marker ID
            T_b_h: Head pose from VisionOS
            T_b_m: Marker pose from ARKit
            save: Whether to append sample to collection
        
        Returns:
            ExtrinsicSample if successful (detected), None otherwise
        """
        T_c_m = self.detect_marker_opencv(frame, marker_id)
        if T_c_m is None:
            return None
        
        sample = ExtrinsicSample(
            T_b_h=T_b_h.copy(),
            T_c_m=T_c_m.copy(),
            T_b_m=T_b_m.copy(),
            marker_id=marker_id,
            timestamp=time.time(),
        )
        
        if save:
            with self._samples_lock:
                self.samples.append(sample)
        
        return sample
    
    def compute_T_h_c(self) -> Optional[np.ndarray]:
        """
        Compute head-to-camera transformation using Kabsch algorithm.
        
        Uses only position data (not orientation) for robustness.
        
        Returns:
            4x4 transformation matrix T_h_c, or None if insufficient samples
        """
        with self._samples_lock:
            if len(self.samples) < 10:
                print(f"Need at least 10 samples, have {len(self.samples)}")
                return None
            samples_copy = list(self.samples)
        
        return self._compute_T_h_c_internal(samples_copy)
    
    def _compute_T_h_c_internal(self, samples: List[ExtrinsicSample]) -> Optional[np.ndarray]:
        """Internal compute function used by both main and background thread.
        
        Uses iterative outlier rejection to improve robustness against noisy ARKit detections.
        """
        if len(samples) < 10:
            return None
        
        # Work with a copy for iterative refinement
        working_samples = list(samples)
        max_outlier_ratio = 0.20  # Remove at most 20% of samples
        error_threshold_mm = 50.0  # Target mean error
        max_iterations = 10
        
        for iteration in range(max_iterations):
            if len(working_samples) < 10:
                print(f"Too few samples remaining after outlier rejection: {len(working_samples)}")
                break
            
            # Extract positions
            P_h = []  # Marker positions in head frame
            P_c = []  # Marker positions in camera frame
            
            for s in working_samples:
                # Marker position in head frame: p_h = T_h_b @ T_b_m[:3, 3]
                T_h_b = np.linalg.inv(s.T_b_h)
                p_h = (T_h_b @ np.append(s.T_b_m[:3, 3], 1))[:3]
                P_h.append(p_h)
                
                # Marker position in camera frame
                P_c.append(s.T_c_m[:3, 3])
            
            P_h = np.array(P_h)
            P_c = np.array(P_c)
            
            # Center the point clouds
            centroid_h = P_h.mean(axis=0)
            centroid_c = P_c.mean(axis=0)
            
            P_h_centered = P_h - centroid_h
            P_c_centered = P_c - centroid_c
            
            # Kabsch algorithm: find optimal rotation
            H = P_c_centered.T @ P_h_centered
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            # Handle reflection case
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            # Compute translation
            t = centroid_h - R @ centroid_c
            
            # Build transformation matrix
            T_h_c = np.eye(4)
            T_h_c[:3, :3] = R
            T_h_c[:3, 3] = t
            
            # Compute per-sample residual errors
            errors = []
            for s in working_samples:
                T_h_b = np.linalg.inv(s.T_b_h)
                p_h_expected = (T_h_b @ np.append(s.T_b_m[:3, 3], 1))[:3]
                p_h_predicted = (T_h_c @ np.append(s.T_c_m[:3, 3], 1))[:3]
                errors.append(np.linalg.norm(p_h_expected - p_h_predicted))
            
            errors = np.array(errors)
            mean_error_mm = np.mean(errors) * 1000
            median_error = np.median(errors)
            
            # Check if we've converged to acceptable error
            if mean_error_mm < error_threshold_mm:
                print(f"Kabsch alignment: {len(working_samples)} samples (iter {iteration+1}), mean error: {mean_error_mm:.2f}mm (converged)")
                self.last_loss = mean_error_mm
                return T_h_c
            
            # Check if we've removed too many samples
            removed_count = len(samples) - len(working_samples)
            if removed_count >= len(samples) * max_outlier_ratio:
                print(f"Kabsch alignment: {len(working_samples)} samples, mean error: {mean_error_mm:.2f}mm (max outliers reached)")
                self.last_loss = mean_error_mm
                return T_h_c
            
            # Identify outliers: samples with error > 3x median
            outlier_threshold = max(median_error * 3.0, 0.05)  # At least 5cm threshold
            outlier_mask = errors > outlier_threshold
            n_outliers = np.sum(outlier_mask)
            
            if n_outliers == 0:
                # No more outliers found
                print(f"Kabsch alignment: {len(working_samples)} samples (iter {iteration+1}), mean error: {mean_error_mm:.2f}mm (no outliers)")
                self.last_loss = mean_error_mm
                return T_h_c
            
            # Remove worst outliers (at most 5% per iteration to avoid aggressive rejection)
            max_remove_per_iter = max(1, int(len(working_samples) * 0.05))
            if n_outliers > max_remove_per_iter:
                # Only remove the worst ones
                sorted_indices = np.argsort(errors)[::-1]  # Descending
                outlier_mask = np.zeros(len(errors), dtype=bool)
                outlier_mask[sorted_indices[:max_remove_per_iter]] = True
            
            # Filter samples
            working_samples = [s for s, is_outlier in zip(working_samples, outlier_mask) if not is_outlier]
            print(f"  Iter {iteration+1}: removed {np.sum(outlier_mask)} outliers, {len(working_samples)} remaining")
        
        # Final result after all iterations
        mean_error_mm = np.mean(errors) * 1000 if len(errors) > 0 else float('inf')
        print(f"Kabsch alignment: {len(working_samples)} samples, mean error: {mean_error_mm:.2f}mm")
        self.last_loss = mean_error_mm
        
        return T_h_c
    
    def clear_samples(self):
        """Clear all collected samples."""
        with self._samples_lock:
            self.samples = []
    
    def save_samples(self, filepath: str):
        """Save all collected samples to a JSON file for debugging.
        
        Args:
            filepath: Path to save the JSON file
        """
        with self._samples_lock:
            samples_copy = list(self.samples)
        
        data = {
            "camera_matrix": self.camera_matrix.tolist(),
            "dist_coeffs": self.dist_coeffs.tolist(),
            "marker_size": self.marker_size,
            "num_samples": len(samples_copy),
            "samples": []
        }
        
        for s in samples_copy:
            sample_data = {
                "T_b_h": s.T_b_h.tolist(),  # Head pose from VisionOS
                "T_c_m": s.T_c_m.tolist(),  # Marker pose from camera (OpenCV)
                "T_b_m": s.T_b_m.tolist(),  # Marker pose from ARKit
                "marker_id": s.marker_id,
                "timestamp": s.timestamp,
            }
            data["samples"].append(sample_data)
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"Saved {len(samples_copy)} samples to {filepath}")
    
    def load_samples(self, filepath: str) -> int:
        """Load samples from a JSON file.
        
        Args:
            filepath: Path to the JSON file
            
        Returns:
            Number of samples loaded
        """
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        loaded_samples = []
        for s in data.get("samples", []):
            sample = ExtrinsicSample(
                T_b_h=np.array(s["T_b_h"]),
                T_c_m=np.array(s["T_c_m"]),
                T_b_m=np.array(s["T_b_m"]),
                marker_id=s["marker_id"],
                timestamp=s["timestamp"],
            )
            loaded_samples.append(sample)
        
        with self._samples_lock:
            self.samples = loaded_samples
        
        print(f"Loaded {len(loaded_samples)} samples from {filepath}")
        return len(loaded_samples)


def run_extrinsic_calibration(
    camera_id: int,
    intrinsic_calibration: Dict[str, CalibrationResult],
    avp_ip: str,
    stereo: bool = False,
) -> Optional[np.ndarray]:
    """
    Run extrinsic calibration with Vision Pro.
    
    Returns:
        4x4 T_h_c transformation, or None if failed
    """
    print("\n" + "=" * 60)
    print("EXTRINSIC CALIBRATION")
    print("=" * 60)
    print()
    print("Prerequisites:")
    print("  1. Wear the Vision Pro with the camera attached")
    print("  2. Launch Tracking Streamer app on Vision Pro")
    print("  3. Enable Marker Detection in settings")
    print(f"  4. Make sure AVP IP is correct: {avp_ip}")
    print()
    print("Process:")
    print("  - Place each marker (IDs 11, 12, 13) on a flat surface")
    print("  - Look at the marker with both camera and Vision Pro")
    print("  - Move your head around while keeping marker in view")
    print("  - Samples are collected automatically")
    print()
    
    # Import VisionProStreamer
    try:
        from avp_stream import VisionProStreamer
    except ImportError:
        print("Error: Could not import VisionProStreamer")
        return None
    
    # Get intrinsic calibration
    result = intrinsic_calibration["left"]
    if result.camera_matrix is None:
        print("Error: No intrinsic calibration available!")
        return None
    
    # Connect to Vision Pro
    print(f"Connecting to Vision Pro at {avp_ip}...")
    try:
        streamer = VisionProStreamer(ip=avp_ip, verbose=True)
    except Exception as e:
        print(f"Failed to connect to Vision Pro: {e}")
        return None
    
    print("Connected! Waiting for tracking data...")
    time.sleep(2)
    
    # Open camera
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_id}")
        return None
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080 if stereo else 2160)
    
    calibrator = ExtrinsicCalibrator(
        result.camera_matrix,
        result.distortion,
    )
    
    current_marker_idx = 0
    sample_interval = 0.1  # Collect samples every 100ms
    last_sample_time = 0
    
    print("\nStarting extrinsic calibration...")
    print(f"Looking for marker ID {EXTRINSIC_MARKER_IDS[current_marker_idx]}")
    print("Press 'n' to move to next marker, 'q' to finish")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Use left view for stereo
        cam_frame = frame[:, :frame.shape[1]//2] if stereo else frame
        
        current_marker_id = EXTRINSIC_MARKER_IDS[current_marker_idx]
        current_time = time.time()
        
        # Get data from Vision Pro
        latest = streamer.get_latest()
        markers = streamer.get_markers()
        
        vis_frame = cam_frame.copy()
        
        # Detect marker in camera
        T_c_m = calibrator.detect_marker_opencv(cam_frame, current_marker_id)
        camera_detected = T_c_m is not None
        
        # Check ARKit detection
        arkit_detected = current_marker_id in markers
        
        # Draw detection status
        y = 30
        cv2.putText(vis_frame, f"Marker {current_marker_idx + 1}/3 (ID {current_marker_id})",
                   (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        y += 35
        
        cam_color = (0, 255, 0) if camera_detected else (0, 0, 255)
        cv2.putText(vis_frame, f"Camera: {'DETECTED' if camera_detected else 'NOT FOUND'}",
                   (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, cam_color, 2)
        y += 25
        
        arkit_color = (0, 255, 0) if arkit_detected else (0, 0, 255)
        cv2.putText(vis_frame, f"ARKit: {'DETECTED' if arkit_detected else 'NOT FOUND'}",
                   (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, arkit_color, 2)
        y += 25
        
        # Collect sample if both detect the marker
        sample_collected = False
        if (camera_detected and arkit_detected and latest is not None and
            current_time - last_sample_time >= sample_interval):
            
            T_b_h = latest["head"][0]
            T_b_m = markers[current_marker_id]["pose"]
            
            sample = calibrator.collect_sample(cam_frame, current_marker_id, T_b_h, T_b_m)
            if sample is not None:
                last_sample_time = current_time
                sample_collected = True
        
        # Show sample count
        cv2.putText(vis_frame, f"Samples: {len(calibrator.samples)}",
                   (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y += 25
        
        if sample_collected:
            cv2.putText(vis_frame, "SAMPLE COLLECTED!",
                       (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Instructions
        cv2.putText(vis_frame, "Press 'n' for next marker, 'q' to compute T_h_c",
                   (10, vis_frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        cv2.imshow("Extrinsic Calibration", vis_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('n'):
            current_marker_idx = (current_marker_idx + 1) % len(EXTRINSIC_MARKER_IDS)
            print(f"Now looking for marker ID {EXTRINSIC_MARKER_IDS[current_marker_idx]}")
    
    cap.release()
    cv2.destroyAllWindows()
    
    # Compute T_h_c
    print("\nComputing head-to-camera transformation...")
    T_h_c = calibrator.compute_T_h_c()
    
    return T_h_c


# =============================================================================
# Main Entry Point
# =============================================================================

def save_calibration(
    output_path: str,
    intrinsic: Dict[str, CalibrationResult],
    T_h_c: Optional[np.ndarray] = None,
    stereo: bool = False,
):
    """Save calibration results to JSON."""
    data = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "stereo": stereo,
    }
    
    # Intrinsic calibration
    for name in ["left", "right"] if stereo else ["left"]:
        result = intrinsic[name]
        if result.camera_matrix is not None:
            data[f"intrinsic_{name}"] = {
                "camera_matrix": result.camera_matrix.tolist(),
                "distortion": result.distortion.tolist(),
                "image_size": list(result.image_size) if result.image_size else None,
                "reprojection_error": result.reprojection_error,
                "n_samples": result.n_samples,
            }
    
    # Extrinsic calibration
    if T_h_c is not None:
        data["extrinsic"] = {
            "T_head_camera": T_h_c.tolist(),
        }
    
    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"\nCalibration saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Complete camera calibration workflow",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python -m avp_stream.run_calibration
  python -m avp_stream.run_calibration --camera 0 --stereo
  python -m avp_stream.run_calibration --avp-ip 192.168.1.100
        """
    )
    
    parser.add_argument("--camera", "-c", type=int, default=None,
                        help="Camera index (default: interactive selection)")
    parser.add_argument("--stereo", action="store_true",
                        help="Enable stereo camera mode (side-by-side)")
    parser.add_argument("--avp-ip", type=str, default=None,
                        help="Vision Pro IP for extrinsic calibration")
    parser.add_argument("--output", "-o", type=str, default="camera_calibration.json",
                        help="Output calibration file (default: camera_calibration.json)")
    parser.add_argument("--skip-intrinsic", action="store_true",
                        help="Skip intrinsic calibration (load from existing file)")
    parser.add_argument("--skip-extrinsic", action="store_true",
                        help="Skip extrinsic calibration")
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("CAMERA CALIBRATION WORKFLOW")
    print("=" * 60)
    print()
    
    # Camera selection
    stereo = args.stereo  # Default from CLI
    if args.camera is None:
        print("Discovering cameras...")
        cameras = discover_cameras()
        camera_id, detected_stereo = select_camera(cameras)
        if camera_id is None:
            print("No camera selected, exiting.")
            return
        # Use detected stereo unless explicitly set via CLI
        if not args.stereo:
            stereo = detected_stereo
    else:
        camera_id = args.camera
    
    print(f"\nUsing camera index: {camera_id}")
    print(f"Stereo mode: {'enabled' if stereo else 'disabled'}")
    
    intrinsic_calibration = None

    
    # Step 1: Intrinsic calibration
    if not args.skip_intrinsic:
        while True:
            intrinsic_calibration = run_intrinsic_calibration(camera_id, stereo)
            
            if intrinsic_calibration is None:
                print("Intrinsic calibration failed or cancelled.")
                retry = input("Retry? [y/N]: ").strip().lower()
                if retry != 'y':
                    return
                continue
            
            # Step 2: Verification
            passed = run_intrinsic_verification(camera_id, intrinsic_calibration, stereo)
            
            if passed:
                print("\nIntrinsic calibration verified!")
                break
            else:
                print("\nVerification failed, retrying calibration...")
    else:
        # Load existing calibration
        if Path(args.output).exists():
            with open(args.output, 'r') as f:
                data = json.load(f)
            intrinsic_calibration = {}
            for name in ["left", "right"]:
                key = f"intrinsic_{name}"
                if key in data:
                    d = data[key]
                    intrinsic_calibration[name] = CalibrationResult(
                        camera_matrix=np.array(d["camera_matrix"]),
                        distortion=np.array(d["distortion"]),
                        image_size=tuple(d["image_size"]) if d["image_size"] else None,
                        reprojection_error=d["reprojection_error"],
                        n_samples=d["n_samples"],
                    )
                else:
                    intrinsic_calibration[name] = CalibrationResult()
            print(f"Loaded intrinsic calibration from {args.output}")
        else:
            print(f"Error: No existing calibration file found at {args.output}")
            return
    
    # Step 3: Extrinsic calibration
    T_h_c = None
    if not args.skip_extrinsic:
        if args.avp_ip is None:
            avp_ip = input("\nEnter Vision Pro IP address: ").strip()
        else:
            avp_ip = args.avp_ip
        
        if avp_ip:
            T_h_c = run_extrinsic_calibration(
                camera_id, intrinsic_calibration, avp_ip, stereo
            )
            
            if T_h_c is not None:
                print("\nExtrinsic calibration complete!")
                print(f"T_head_camera:\n{T_h_c}")
            else:
                print("\nExtrinsic calibration failed or skipped.")
    
    # Save results
    save_calibration(args.output, intrinsic_calibration, T_h_c, stereo)
    
    print("\n" + "=" * 60)
    print("CALIBRATION COMPLETE")
    print("=" * 60)


if __name__ == "__main__":
    main()
