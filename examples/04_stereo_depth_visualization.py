"""Visualize hand tracking with stereo depth perception using binocular disparity."""

from avp_stream.streamer import VisionProStreamer
import cv2
import numpy as np
import time
import argparse


def create_stereo_depth_visualizer(streamer, disparity_scale=50.0):
    """
    Create a stereo visualizer that uses disparity to show depth.
    
    Args:
        streamer: VisionProStreamer instance for getting hand tracking data
        disparity_scale: Controls the strength of the stereo effect (default: 100.0)
                        Higher values = more pronounced depth effect
                        Lower values = more subtle depth effect
                        Recommended range: 50-200
    
    Disparity principle:
    - Objects closer to the viewer have MORE disparity (larger separation between left/right)
    - Objects farther away have LESS disparity (smaller separation)
    - Negative Z = closer to user, Positive Z = farther away
    """
    
    def generate_stereo_frame(blank_frame):
        h, w = blank_frame.shape[:2]
        
        # This will be side-by-side stereo (left half = left eye, right half = right eye)
        half_w = w // 2
        

        # Get latest hand tracking data
        latest = streamer.get_latest()
        if latest is None:
            # Draw "waiting" message in center of each eye
            msg = "Waiting for hand data..."
            for eye_offset in [quarter_w, half_w + quarter_w]:
                cv2.putText(blank_frame, msg, (eye_offset - 150, h//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            return blank_frame
        
        # Virtual 3D space parameters
        center_y = h // 2
        quarter_w = half_w // 2  # Center of each eye view
        
        
        # Scale factors for projection
        xy_scale = 800  # How much to scale X/Y coordinates (doubled for bigger hands)
        z_scale = disparity_scale  # How much Z affects disparity (controllable)
        
        def project_point_stereo(world_pos):
            """
            Project a 3D point to stereo 2D coordinates.
            Returns (left_x, left_y, right_x, right_y, depth_z)
            """
            x, y, z = world_pos[0, 3], world_pos[1, 3], world_pos[2, 3]
            
            # Calculate disparity based on Z depth
            # Negative Z (closer) = more disparity, Positive Z (farther) = less disparity
            # INVERT the sign: closer objects need MORE separation
            disparity = -(z_scale * z)  # Negative Z creates positive disparity
            
            # Y is the same for both eyes
            screen_y = int(center_y - float(y) * xy_scale)
            
            # Left eye: shift left by half the disparity
            left_x = int(quarter_w + float(x) * xy_scale - disparity/2)
            
            # Right eye: shift right by half the disparity (plus half_w offset)
            right_x = int(half_w + quarter_w + float(x) * xy_scale + disparity/2)
            
            return left_x, screen_y, right_x, screen_y, float(z)
        
        def draw_hand_stereo(wrist_matrix, fingers_matrix, color, label):
            """Draw a hand in stereo with proper depth perception"""
            if wrist_matrix is None or fingers_matrix is None:
                return
            
            # Transform fingers from local to world space (same as original example)
            fingers_world = wrist_matrix @ fingers_matrix
            
            # Project wrist (use the wrist transform's position)
            left_x, left_y, right_x, right_y, depth = project_point_stereo(wrist_matrix[0])
            
            # Draw wrist circle in both eyes (scaled up)
            cv2.circle(blank_frame, (left_x, left_y), 25, color, -1)
            cv2.circle(blank_frame, (right_x, right_y), 25, color, -1)
            
            # Draw label (larger font)
            cv2.putText(blank_frame, label, (left_x - 12, left_y + 8), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(blank_frame, label, (right_x - 12, right_y + 8), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # Draw all 25 finger joints with proper skeleton connections
            # Hand skeleton structure: 5 fingers × 5 joints each = 25 total
            # Each finger: [base, joint1, joint2, joint3, tip]
            finger_color = tuple(int(c * 0.6) for c in color)
            
            for finger_idx in range(5):  # 5 fingers
                finger_start = finger_idx * 5
                finger_joints = []
                
                # Project all joints in this finger
                for joint_idx in range(5):
                    joint_i = finger_start + joint_idx
                    if joint_i < fingers_world.shape[0]:
                        fx_l, fy_l, fx_r, fy_r, f_depth = project_point_stereo(fingers_world[joint_i])
                        finger_joints.append(((fx_l, fy_l), (fx_r, fy_r)))
                        
                        # Draw joint circles (larger)
                        cv2.circle(blank_frame, (fx_l, fy_l), 6, finger_color, -1)
                        cv2.circle(blank_frame, (fx_r, fy_r), 6, finger_color, -1)
                
                # Connect wrist to finger base (thicker lines)
                if len(finger_joints) > 0:
                    cv2.line(blank_frame, (left_x, left_y), finger_joints[0][0], finger_color, 2)
                    cv2.line(blank_frame, (right_x, right_y), finger_joints[0][1], finger_color, 2)
                
                # Connect joints within the finger (base -> joint1 -> joint2 -> joint3 -> tip)
                for j in range(len(finger_joints) - 1):
                    cv2.line(blank_frame, finger_joints[j][0], finger_joints[j+1][0], finger_color, 2)
                    cv2.line(blank_frame, finger_joints[j][1], finger_joints[j+1][1], finger_color, 2)
            
            return depth
        
        # Draw coordinate axes reference (centered, no disparity - at z=0)
        left_center = (quarter_w, center_y)
        right_center = (half_w + quarter_w, center_y)
        
        # X axis (red)
        cv2.line(blank_frame, left_center, (left_center[0] + 100, left_center[1]), (0, 0, 255), 2)
        cv2.line(blank_frame, right_center, (right_center[0] + 100, right_center[1]), (0, 0, 255), 2)
        cv2.putText(blank_frame, "X", (left_center[0] + 110, left_center[1]), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(blank_frame, "X", (right_center[0] + 110, right_center[1]), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Y axis (green)
        cv2.line(blank_frame, left_center, (left_center[0], left_center[1] - 100), (0, 255, 0), 2)
        cv2.line(blank_frame, right_center, (right_center[0], right_center[1] - 100), (0, 255, 0), 2)
        cv2.putText(blank_frame, "Y", (left_center[0], left_center[1] - 110), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(blank_frame, "Y", (right_center[0], right_center[1] - 110), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw hands with stereo depth
        left_wrist = latest.get("left_wrist")
        right_wrist = latest.get("right_wrist")
        left_fingers = latest.get("left_fingers")
        right_fingers = latest.get("right_fingers")
        left_pinch = latest.get("left_pinch_distance", 0)
        right_pinch = latest.get("right_pinch_distance", 0)
        
        # Draw left hand (blue/cyan)
        left_depth = None
        if left_wrist is not None and left_fingers is not None:
            color = (0, 255, 255) if left_pinch < 0.02 else (100, 100, 255)
            left_depth = draw_hand_stereo(left_wrist, left_fingers, color, "L")
        
        # Draw right hand (red/magenta)
        right_depth = None
        if right_wrist is not None and right_fingers is not None:
            color = (255, 0, 255) if right_pinch < 0.02 else (255, 100, 100)
            right_depth = draw_hand_stereo(right_wrist, right_fingers, color, "R")
        
        # Display info panel (only on left eye to avoid clutter)
        info_y = 30
        cv2.putText(blank_frame, "STEREO DEPTH VISUALIZATION", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += 30
        
        if left_depth is not None:
            cv2.putText(blank_frame, f"Left Hand Depth: {left_depth:+.3f}m", (10, info_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 255), 1)
            info_y += 25
        
        if right_depth is not None:
            cv2.putText(blank_frame, f"Right Hand Depth: {right_depth:+.3f}m", (10, info_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 255), 1)
            info_y += 25
        
        cv2.putText(blank_frame, f"Left Pinch: {left_pinch:.3f}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        info_y += 25
        cv2.putText(blank_frame, f"Right Pinch: {right_pinch:.3f}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        info_y += 25
        
        # Add depth perception guide
        cv2.putText(blank_frame, "Move hands forward/back to see depth!", (10, h - 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 255, 150), 1)
        cv2.putText(blank_frame, "Closer = Wider separation | Farther = Narrower", (10, h - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 255, 150), 1)
        
        # Draw center divider line for reference
        cv2.line(blank_frame, (half_w, 0), (half_w, h), (80, 80, 80), 2)
        cv2.putText(blank_frame, "LEFT EYE", (quarter_w - 50, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
        cv2.putText(blank_frame, "RIGHT EYE", (half_w + quarter_w - 50, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
        
        return blank_frame
    
    return generate_stereo_frame


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Stereo vision depth perception visualization with hand tracking"
    )
    parser.add_argument("--ip", type=str, required=True, 
                       help="Vision Pro IP address")
    parser.add_argument("--resolution", type=str, default="2000x1000",
                       help="Resolution for side-by-side stereo (default: 1920x1080)")
    parser.add_argument("--fps", type=int, default=30,
                       help="Frame rate (default: 30)")
    parser.add_argument("--disparity", type=float, default=100.0,
                       help="Disparity scale factor for stereo effect (default: 100.0, range: 50-200)")
    args = parser.parse_args()
    
    # Create streamer
    print("=" * 70)
    print("STEREO DEPTH PERCEPTION VISUALIZATION")
    print("=" * 70)
    print()
    print("This example demonstrates binocular disparity for depth perception.")
    print("Move your hands forward and backward to see the stereo effect!")
    print()
    
    streamer = VisionProStreamer(ip=args.ip)
    
    # Register stereo frame callback with custom disparity scale
    streamer.register_frame_callback(create_stereo_depth_visualizer(streamer, disparity_scale=args.disparity))
    
    # Configure video streaming with stereo enabled
    print(f"Starting stereo video stream at {args.resolution}, {args.fps} fps...")
    streamer.configure_video(
        fps=args.fps,
        size=args.resolution,     # Side-by-side stereo resolution
        stereo=True,              # Enable stereo video mode
    )
    streamer.start_webrtc(port=9999)
    
    
    try:
        while True:
            time.sleep(1/30.0)
    except KeyboardInterrupt:
        print("\n\nStopping stereo visualization...")
        print("Done!")
