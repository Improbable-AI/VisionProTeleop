import cv2
import numpy as np
import argparse
import json
import time

def split_stereo_image(img):
    """Split side-by-side stereo image into left and right images."""
    h, w = img.shape[:2]
    mid = w // 2
    left = img[:, :mid]
    right = img[:, mid:]
    return left, right

def run_calibration(
    camera_id: int, 
    use_stereo: bool,
    board_type: str,
    rows: int, 
    cols: int, 
    square_size: float, 
    marker_size: float, 
    output_file: str
):
    print(f"Starting {board_type} calibration...")
    print(f"Grid: {rows}x{cols} squares | Square: {square_size*1000:.1f}mm")
    
    # Setup Board Logic
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    charuco_board = None
    checkerboard_pattern_size = None
    checkerboard_objpoints = None
    
    if board_type == "charuco":
        charuco_board = cv2.aruco.CharucoBoard((cols, rows), square_size, marker_size, dictionary)
        aruco_params = cv2.aruco.DetectorParameters()
        aruco_detector = cv2.aruco.ArucoDetector(dictionary, aruco_params)
    elif board_type == "checkerboard":
        # OpenCV findChessboardCorners looks for INNER corners
        # If board is 6x5 squares, inner corners are 5x4
        checkerboard_pattern_size = (cols - 1, rows - 1)
        
        # Prepare object points (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((checkerboard_pattern_size[0] * checkerboard_pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:checkerboard_pattern_size[0], 0:checkerboard_pattern_size[1]].T.reshape(-1, 2)
        objp = objp * square_size
        checkerboard_objpoints = objp
        print(f"Looking for checkerboard inner corners: {checkerboard_pattern_size}")
    
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_id}")
        return
        
    # Set high resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080 if use_stereo else 2160)
    
    # Storage
    # For ChArUco: 'corners' store interpolated corners
    # For Checkerboard: 'corners' store image points, 'objpoints' store 3D points
    collections = {
        "left": {"imgpoints": [], "objpoints": [], "ids": [], "size": None},
        "right": {"imgpoints": [], "objpoints": [], "ids": [], "size": None}
    }
    
    print("\nControls:")
    print("  'c' - Capture frame (must detect board)")
    print("  'q' - Quit and Calibrate")
    print("  'r' - Reset/Clear captures")
    
    # Auto-capture settings
    auto_capture_interval = 0.5  # seconds between auto-captures
    min_corner_movement = 15.0    # pixels - minimum movement to trigger new capture
    last_capture_time = {"left": 0, "right": 0}
    last_corners = {"left": None, "right": None}
    
    # Incremental calibration tracking
    current_reproj_error = {}  # Current reprojection error per camera
    current_fx = {}  # Current focal length (fx) per camera
    current_fy = {}  # Current focal length (fy) per camera
    current_cx = {}  # Current optical center (cx) per camera
    current_cy = {}  # Current optical center (cy) per camera
    current_dist = {}  # Current distortion coefficients per camera
    calibration_running = {"left": False, "right": False}  # Background calibration in progress
    
    # Convergence tracking - track history of fx values
    fx_history = {"left": [], "right": []}
    CONVERGENCE_WINDOW = 10  # Number of recent calibrations to analyze
    
    def compute_convergence(history):
        """Compute convergence score based on fx stability over recent samples.
        Returns (stability_pct, ready_to_stop) where stability_pct is 0-100."""
        if len(history) < 5:
            return 0, False
        
        recent = history[-CONVERGENCE_WINDOW:] if len(history) >= CONVERGENCE_WINDOW else history
        mean_fx = np.mean(recent)
        std_fx = np.std(recent)
        
        # Coefficient of variation (relative std dev)
        cv = (std_fx / mean_fx * 100) if mean_fx > 0 else 100
        
        # Stability score: 100% = perfectly stable, 0% = highly variable
        # A CV of 0.5% or less means ~99% stable, CV of 5% means ~50% stable
        stability = max(0, min(100, 100 - cv * 20))
        
        # Ready to stop if: stable (>90%), low error, and enough samples
        ready = stability > 90 and len(history) >= 20
        
        return stability, ready
    
    def corner_movement(corners_new, corners_old):
        """Calculate average corner movement between two detections."""
        if corners_old is None or corners_new is None:
            return float('inf')
        if len(corners_new) != len(corners_old):
            return float('inf')
        # Calculate mean distance
        try:
            diff = np.linalg.norm(corners_new.reshape(-1, 2) - corners_old.reshape(-1, 2), axis=1)
            return np.mean(diff)
        except:
            return float('inf')
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
            
        vis_frame = frame.copy()
        
        views = {}
        if use_stereo:
            left, right = split_stereo_image(frame)
            views["left"] = left
            views["right"] = right
        else:
            views["left"] = frame 
            
        current_detections = {}
        
        for name, img in views.items():
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            collections[name]["size"] = gray.shape[::-1] 
            
            detected = False
            
            if board_type == "charuco":
                corners, ids, rejected = aruco_detector.detectMarkers(gray)
                if len(corners) > 0:
                    # Always draw detected markers (green squares) for feedback
                    draw_img = vis_frame if not use_stereo else (vis_frame[:, :vis_frame.shape[1]//2] if name == "left" else vis_frame[:, vis_frame.shape[1]//2:])
                    cv2.aruco.drawDetectedMarkers(draw_img, corners)
                    cv2.putText(draw_img, f"Raw Markers: {len(corners)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    try:
                        ret_val, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                            corners, ids, gray, charuco_board
                        )
                    except Exception as e:
                        print(f"Interpolation error: {e}")
                        ret_val = 0
                    
                    if ret_val >= 4: # Threshold: require at least 4 corners for valid pose/detection
                        # Subpixel refinement for better accuracy
                        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                        charuco_corners = cv2.cornerSubPix(gray, charuco_corners, (5, 5), (-1, -1), term)
                        
                        detected = True
                        current_detections[name] = {"corners": charuco_corners, "ids": charuco_ids}
                        cv2.aruco.drawDetectedCornersCharuco(draw_img, charuco_corners, charuco_ids, (0, 0, 255)) # red dots
                        
                        # Show count
                        cv2.putText(draw_img, f"Corners: {len(charuco_corners)}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    else:
                        cv2.putText(draw_img, f"Markers: {len(corners)} (Board Mismatch?)", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            elif board_type == "checkerboard":
                found = False
                corners = None
                try:
                    found, corners = cv2.findChessboardCorners(gray, checkerboard_pattern_size, 
                        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
                except cv2.error as e:
                    # OpenCV requires inner corners > 2x2. If 4x3 squares -> 3x2 inner -> Error.
                    pass
                
                if found:
                    # Subpixel refinement
                    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
                    
                    detected = True
                    current_detections[name] = {"corners": corners2, "objpoints": checkerboard_objpoints}
                    
                    # Draw
                    draw_img = vis_frame if not use_stereo else (vis_frame[:, :vis_frame.shape[1]//2] if name == "left" else vis_frame[:, vis_frame.shape[1]//2:])
                    cv2.drawChessboardCorners(draw_img, checkerboard_pattern_size, corners2, found)

        # Auto-capture logic
        current_time = time.time()
        targets = ["left", "right"] if use_stereo else ["left"]
        
        for name in targets:
            if name in current_detections:
                det = current_detections[name]
                movement = corner_movement(det["corners"], last_corners[name])
                time_since_last = current_time - last_capture_time[name]
                
                # Auto-capture if: enough time passed AND (first capture OR significant movement)
                should_capture = (time_since_last >= auto_capture_interval and 
                                  (last_corners[name] is None or movement >= min_corner_movement))
                
                if should_capture:
                    collections[name]["imgpoints"].append(det["corners"])
                    if board_type == "charuco":
                        collections[name]["ids"].append(det["ids"])
                    else:
                        collections[name]["objpoints"].append(det["objpoints"])
                    
                    last_capture_time[name] = current_time
                    last_corners[name] = det["corners"].copy()
                    
                    # Trigger background calibration (non-blocking)
                    count = len(collections[name]["imgpoints"])
                    if count >= 5 and not calibration_running[name]:
                        # Start calibration in background thread
                        calibration_running[name] = True
                        
                        def run_calibration_bg(cam_name, imgpts, ids_list, objpts, size, board):
                            try:
                                if board_type == "charuco":
                                    ret, mtx, dist, _, _ = cv2.aruco.calibrateCameraCharuco(
                                        imgpts.copy(), 
                                        ids_list.copy(), 
                                        board, 
                                        size, 
                                        None, None
                                    )
                                else:
                                    ret, mtx, dist, _, _ = cv2.calibrateCamera(
                                        objpts.copy(),
                                        imgpts.copy(),
                                        size,
                                        None, None
                                    )
                                current_reproj_error[cam_name] = ret
                                current_fx[cam_name] = mtx[0, 0]
                                current_fy[cam_name] = mtx[1, 1]
                                current_cx[cam_name] = mtx[0, 2]
                                current_cy[cam_name] = mtx[1, 2]
                                current_dist[cam_name] = dist.ravel()
                                # Track fx history for convergence
                                fx_history[cam_name].append(mtx[0, 0])
                            except Exception as e:
                                pass  # Calibration failed, keep previous values
                            finally:
                                calibration_running[cam_name] = False
                        
                        import threading
                        thread = threading.Thread(
                            target=run_calibration_bg,
                            args=(name, 
                                  list(collections[name]["imgpoints"]),
                                  list(collections[name]["ids"]) if board_type == "charuco" else None,
                                  list(collections[name]["objpoints"]) if board_type != "charuco" else None,
                                  collections[name]["size"],
                                  charuco_board)
                        )
                        thread.daemon = True
                        thread.start()
                    
                    print(f"Captured! Total: L={len(collections['left']['imgpoints'])} R={len(collections['right']['imgpoints'])}")

        # Overlay status
        count_L = len(collections["left"]["imgpoints"])
        count_R = len(collections["right"]["imgpoints"])
        
        # Color based on reprojection error quality
        def error_color(err):
            if err is None: return (128, 128, 128)  # Gray
            if err < 0.5: return (0, 255, 0)  # Green - excellent
            if err < 1.0: return (0, 255, 255)  # Yellow - good
            if err < 2.0: return (0, 165, 255)  # Orange - acceptable
            return (0, 0, 255)  # Red - poor
        
        # Compute convergence
        conv_L, ready_L = compute_convergence(fx_history["left"])
        conv_R, ready_R = compute_convergence(fx_history["right"])
        
        # Build display lines
        y_offset = 30
        
        # Title line
        title = f"{board_type.upper()} | L={count_L}" + (f" R={count_R}" if use_stereo else "") + " | AUTO"
        cv2.putText(vis_frame, title, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        y_offset += 35
        
        # Left camera full intrinsics
        err_L = current_reproj_error.get("left", None)
        if err_L is not None:
            fx_L = current_fx.get("left", 0)
            fy_L = current_fy.get("left", 0)
            cx_L = current_cx.get("left", 0)
            cy_L = current_cy.get("left", 0)
            dist_L = current_dist.get("left", [])
            
            # Line 1: focal length and error
            line_L1 = f"L: fx={fx_L:.1f} fy={fy_L:.1f} cx={cx_L:.1f} cy={cy_L:.1f} err={err_L:.3f}"
            cv2.putText(vis_frame, line_L1, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.65, error_color(err_L), 2)
            y_offset += 28
            
            # Line 2: distortion coefficients
            if len(dist_L) >= 5:
                line_L2 = f"   k1={dist_L[0]:.4f} k2={dist_L[1]:.4f} p1={dist_L[2]:.4f} p2={dist_L[3]:.4f} k3={dist_L[4]:.4f}"
                cv2.putText(vis_frame, line_L2, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)
                y_offset += 25
        else:
            cv2.putText(vis_frame, "L: collecting...", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (128, 128, 128), 2)
            y_offset += 30
        
        # Right camera full intrinsics (if stereo)
        if use_stereo:
            err_R = current_reproj_error.get("right", None)
            if err_R is not None:
                fx_R = current_fx.get("right", 0)
                fy_R = current_fy.get("right", 0)
                cx_R = current_cx.get("right", 0)
                cy_R = current_cy.get("right", 0)
                dist_R = current_dist.get("right", [])
                
                # Line 1: focal length and error
                line_R1 = f"R: fx={fx_R:.1f} fy={fy_R:.1f} cx={cx_R:.1f} cy={cy_R:.1f} err={err_R:.3f}"
                cv2.putText(vis_frame, line_R1, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.65, error_color(err_R), 2)
                y_offset += 28
                
                # Line 2: distortion coefficients
                if len(dist_R) >= 5:
                    line_R2 = f"   k1={dist_R[0]:.4f} k2={dist_R[1]:.4f} p1={dist_R[2]:.4f} p2={dist_R[3]:.4f} k3={dist_R[4]:.4f}"
                    cv2.putText(vis_frame, line_R2, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)
                    y_offset += 25
            else:
                cv2.putText(vis_frame, "R: collecting...", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (128, 128, 128), 2)
                y_offset += 30
        
        # Convergence status at bottom
        if conv_L > 0 or conv_R > 0:
            conv_text = f"Stability: L={conv_L:.0f}%"
            if use_stereo:
                conv_text += f" R={conv_R:.0f}%"
            if (ready_L and not use_stereo) or (ready_L and ready_R):
                conv_text += " - READY (press q)"
                conv_color = (0, 255, 0)
            else:
                conv_color = (0, 255, 255)
        else:
            conv_text = "Stability: collecting..."
            conv_color = (128, 128, 128)
        
        cv2.putText(vis_frame, conv_text, (10, vis_frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, conv_color, 2)
        
        cv2.imshow("Intrinsic Calibration", vis_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            # Manual capture (force)
            added = False
            for name in targets:
                if name in current_detections:
                    det = current_detections[name]
                    collections[name]["imgpoints"].append(det["corners"])
                    if board_type == "charuco":
                        collections[name]["ids"].append(det["ids"])
                    else:
                        collections[name]["objpoints"].append(det["objpoints"])
                    last_corners[name] = det["corners"].copy()
                    last_capture_time[name] = current_time
                    added = True
            
            if added:
                print(f"Manual Capture! Total: L={len(collections['left']['imgpoints'])} R={len(collections['right']['imgpoints'])}")
                cv2.imshow("Intrinsic Calibration", np.ones_like(vis_frame)*255)
                cv2.waitKey(50)
        elif key == ord('r'):
            print("Resetting captured frames.")
            for k in collections:
                collections[k]["imgpoints"] = []
                collections[k]["objpoints"] = []
                collections[k]["ids"] = []
            last_corners = {"left": None, "right": None}
            last_capture_time = {"left": 0, "right": 0}
            current_reproj_error = {}
            current_fx = {}
            current_fy = {}
            current_cx = {}
            current_cy = {}
            current_dist = {}
            fx_history = {"left": [], "right": []}
                
    cap.release()
    cv2.destroyAllWindows()
    
    # Perform Calibration
    print("\nRunnning Calibration...")
    
    final_output = {
        "method": board_type,
        "grid_rows": rows,
        "grid_cols": cols,
        "square_size": square_size,
        "marker_size": marker_size,
        "date": time.strftime("%Y-%m-%d %H:%M:%S")
    }
    
    targets = ["left", "right"] if use_stereo else ["left"]
    
    for name in targets:
        data = collections[name]
        n_samples = len(data["imgpoints"])
        if n_samples < 5:
            print(f"Warning: Not enough samples for {name} camera (need > 5). Skipping.")
            continue
            
        print(f"Calibrating {name.upper()} camera with {n_samples} samples...")
        
        try:
            ret, mtx, dist, rvecs, tvecs = None, None, None, None, None
            
            # Calibration criteria - doubled iterations (60 vs default 30)
            calib_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
            
            # Initial calibration
            if board_type == "charuco":
                ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
                    data["imgpoints"], data["ids"], charuco_board, data["size"], 
                    None, None, criteria=calib_criteria
                )
            elif board_type == "checkerboard":
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                    data["objpoints"], data["imgpoints"], data["size"], 
                    None, None, criteria=calib_criteria
                )
            
            print(f"  Initial Reproj Error: {ret:.4f}")
            
            # === Outlier Detection and Removal ===
            # Compute per-sample reprojection errors
            if rvecs is not None and len(rvecs) > 0:
                per_sample_errors = []
                for i in range(len(rvecs)):
                    if board_type == "charuco":
                        # Get object points for detected corners
                        objpts = charuco_board.getChessboardCorners()[data["ids"][i].flatten()]
                        imgpts_reproj, _ = cv2.projectPoints(objpts, rvecs[i], tvecs[i], mtx, dist)
                        error = np.sqrt(np.mean((data["imgpoints"][i].reshape(-1, 2) - imgpts_reproj.reshape(-1, 2))**2))
                    else:
                        imgpts_reproj, _ = cv2.projectPoints(data["objpoints"][i], rvecs[i], tvecs[i], mtx, dist)
                        error = np.sqrt(np.mean((data["imgpoints"][i].reshape(-1, 2) - imgpts_reproj.reshape(-1, 2))**2))
                    per_sample_errors.append(error)
                
                per_sample_errors = np.array(per_sample_errors)
                mean_error = np.mean(per_sample_errors)
                std_error = np.std(per_sample_errors)
                
                # Reject samples with error > mean + 2*std
                threshold = mean_error + 2 * std_error
                inlier_mask = per_sample_errors < threshold
                n_outliers = np.sum(~inlier_mask)
                
                if n_outliers > 0 and np.sum(inlier_mask) >= 10:
                    print(f"  Removing {n_outliers} outliers (threshold: {threshold:.3f})")
                    
                    # Filter data
                    filtered_imgpoints = [data["imgpoints"][i] for i in range(len(inlier_mask)) if inlier_mask[i]]
                    if board_type == "charuco":
                        filtered_ids = [data["ids"][i] for i in range(len(inlier_mask)) if inlier_mask[i]]
                    else:
                        filtered_objpoints = [data["objpoints"][i] for i in range(len(inlier_mask)) if inlier_mask[i]]
                    
                    # Re-calibrate without outliers
                    if board_type == "charuco":
                        ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
                            filtered_imgpoints, filtered_ids, charuco_board, data["size"], 
                            None, None, criteria=calib_criteria
                        )
                    else:
                        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                            filtered_objpoints, filtered_imgpoints, data["size"], 
                            None, None, criteria=calib_criteria
                        )
                    
                    n_samples = len(filtered_imgpoints)
                    print(f"  After outlier removal: {n_samples} samples, Reproj Error: {ret:.4f}")
            
            print(f"  Final Reproj Error: {ret:.4f}")
            print(f"  Matrix:\n{mtx}")
            print(f"  Dist: {dist.ravel()}")
            
            res_dict = {
                "intrinsic_matrix": mtx.tolist(),
                "distortion_coefficients": dist.ravel().tolist(),
                "reprojection_error": ret,
                "n_samples": n_samples
            }
            
            if name == "left":
                final_output["image_width"] = data["size"][0]
                final_output["image_height"] = data["size"][1]
                if not use_stereo:
                    final_output.update(res_dict)
                else:
                    final_output["left"] = res_dict
            elif name == "right":
                 final_output["right"] = res_dict
                    
        except Exception as e:
            print(f"Calibration failed for {name}: {e}")
            import traceback
            traceback.print_exc()
            
    # Save results
    with open(output_file, 'w') as f:
        json.dump(final_output, f, indent=2)
    print(f"\nCalibration saved to {output_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calibrate intrinsics using ChArUco or Checkerboard")
    parser.add_argument("--camera", type=int, default=0, help="Camera index")
    parser.add_argument("--stereo", action="store_true", help="Enable side-by-side stereo splitting")
    parser.add_argument("--type", choices=['charuco', 'checkerboard'], default='charuco', help="Board type")
    
    # Board params
    parser.add_argument("--rows", type=int, default=7, help="Number of rows")
    parser.add_argument("--cols", type=int, default=5, help="Number of cols")
    parser.add_argument("--square", type=float, default=0.030, help="Square size in meters")
    parser.add_argument("--marker", type=float, default=0.023, help="Marker size in meters (for ChArUco)")
    
    parser.add_argument("--output", type=str, default="intrinsic_calibration.json", help="Output JSON file")
    
    args = parser.parse_args()
    
    run_calibration(
        args.camera,
        args.stereo,
        args.type,
        args.rows,
        args.cols,
        args.square,
        args.marker,
        args.output
    )

