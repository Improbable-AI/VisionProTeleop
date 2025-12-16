import numpy as np 
import cv2
from typing import Tuple, Dict

class VerificationEngine:
    """
    Handles calibration verification using ArUco grids.
    
    Print mode: 2x2 grid, IDs 0-3, 50mm tags, 80mm center-to-center
    iPhone mode: 1x3 vertical, IDs 0-2, 30mm tags, 40mm center-to-center
    """
    def __init__(self, camera_matrix, dist_coeffs, mode: str = "print"):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.mode = mode
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        if mode == "iphone":
            # iPhone mode: 1x3 vertical layout
            # 40mm markers with 6mm gap between = 46mm center-to-center
            self.marker_size = 0.040  # 40mm
            self.valid_ids = [0, 1, 2]
            self.VALID_PAIRS = {
                (0, 1): 0.046,  # 46mm vertical spacing (40mm + 6mm gap)
                (1, 2): 0.046,
                (0, 2): 0.092,  # Top to bottom (46mm * 2)
            }
        else:
            # Print mode: IDs 0, 2, 3, 100mm tags
            self.marker_size = 0.100  # 100mm
            self.valid_ids = [0, 2, 3]
            self.VALID_PAIRS = {
                (2, 3): 0.160,  # Horizontal
                (0, 2): 0.160,  # Vertical
                (0, 3): 0.2262, # Diagonal (sqrt(2) * 0.16)
            }
    
    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict]:
        """Verify calibration on a single frame."""
        vis_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        stats = {
            "detected": False,
            "mean_error": None,
            "max_error": None,
            "valid_pairs": 0
        }
        
        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(vis_frame, corners, ids)
            
            # Estimate poses for each marker to get 3D centers
            # Note: We use a simplified estimation just to get centers
            # For 2D verification we might just use pixel distances if we knew depth,
            # but better to estimate pose of each marker and measure 3D distance.
            
            marker_size = self.marker_size
            
            # center points map: id -> 3d_center
            centers = {}
            
            # estimatePoseSingleMarkers is not in new opencv obj detection API, 
            # use solvePnP for each
            obj_points = np.array([
                [-marker_size/2, marker_size/2, 0],
                [marker_size/2, marker_size/2, 0],
                [marker_size/2, -marker_size/2, 0],
                [-marker_size/2, -marker_size/2, 0]
            ], dtype=np.float32)
            
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in self.valid_ids:
                    continue
                    
                _, rvec, tvec = cv2.solvePnP(
                    obj_points, corners[i][0], 
                    self.camera_matrix, self.dist_coeffs
                )
                centers[marker_id] = tvec.flatten()
                
                # Draw axis
                cv2.drawFrameAxes(vis_frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)
            
            # Calculate distances
            errors = []
            measurements = []  # For web UI display
            
            for (id1, id2), expected_dist in self.VALID_PAIRS.items():
                if id1 in centers and id2 in centers:
                    dist = np.linalg.norm(centers[id1] - centers[id2])
                    err_mm = abs(dist - expected_dist) * 1000
                    measured_mm = dist * 1000
                    expected_mm = expected_dist * 1000
                    errors.append(err_mm)
                    
                    # Store for web UI
                    measurements.append({
                        "pair": f"{id1}↔{id2}",
                        "measured": round(measured_mm, 1),
                        "expected": round(expected_mm, 0),
                        "error": round(err_mm, 1)
                    })
                    
                    # Draw line
                    p1 = np.mean(corners[list(ids.flatten()).index(id1)][0], axis=0).astype(int)
                    p2 = np.mean(corners[list(ids.flatten()).index(id2)][0], axis=0).astype(int)
                    
                    color = (0, 255, 0) if err_mm < 2.0 else (0, 165, 255) if err_mm < 5.0 else (0, 0, 255)
                    cv2.line(vis_frame, tuple(p1), tuple(p2), color, 2)
                    
                    # Minimal overlay - just show the pair ID
                    mid = ((p1 + p2) / 2).astype(int)
                    cv2.putText(vis_frame, f"{id1}-{id2}", tuple(mid), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            if errors:
                stats["detected"] = True
                stats["mean_error"] = float(np.mean(errors))
                stats["max_error"] = float(np.max(errors))
                stats["valid_pairs"] = len(errors)
                stats["measurements"] = measurements
        
        return vis_frame, stats
