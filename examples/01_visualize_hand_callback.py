"""Visualize hand tracking data in 2D using frame callbacks."""

from avp_stream.streamer import VisionProStreamer
import cv2
import numpy as np
import time

def hand_tracking_visualizer(streamer):
    """Visualize hand tracking data in 3D space"""
    
    def generate_frame(blank_frame):
        h, w = blank_frame.shape[:2]
        
        # Dark background
        blank_frame[:] = [20, 20, 20]
        
        # Get latest hand tracking data
        latest = streamer.get_latest()
        if latest is None:
            cv2.putText(blank_frame, "Waiting for hand data...", (w//2 - 200, h//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            return blank_frame
        
        # Draw virtual 3D space
        center_x, center_y = w // 2, h // 2
        
        # Draw coordinate axes
        cv2.line(blank_frame, (center_x, center_y), (center_x + 100, center_y), (0, 0, 255), 2)  # X
        cv2.line(blank_frame, (center_x, center_y), (center_x, center_y - 100), (0, 255, 0), 2)  # Y
        cv2.putText(blank_frame, "X", (center_x + 110, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(blank_frame, "Y", (center_x, center_y - 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Project and draw hand positions
        left_wrist = latest.get("left_wrist")
        right_wrist = latest.get("right_wrist")
        left_fingers = latest.get("left_fingers")
        right_fingers = latest.get("right_fingers")
        
        # Draw left hand
        if left_wrist is not None and left_fingers is not None:
            # Transform fingers to world space
            left_fingers_world = left_wrist @ left_fingers
            
            # Draw wrist
            lx, ly = int(center_x + float(left_wrist[0, 0, 3]) * 400), int(center_y - float(left_wrist[0, 1, 3]) * 400)
            left_pinch = latest.get("left_pinch_distance", 0)
            color = (0, 255, 0) if left_pinch < 0.02 else (100, 100, 255)
            cv2.circle(blank_frame, (lx, ly), 15, color, -1)
            cv2.putText(blank_frame, "L", (lx - 8, ly + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Draw all 25 finger joints
            for i in range(left_fingers_world.shape[0]):
                fx = int(center_x + float(left_fingers_world[i, 0, 3]) * 400)
                fy = int(center_y - float(left_fingers_world[i, 1, 3]) * 400)
                cv2.circle(blank_frame, (fx, fy), 3, (150, 150, 255), -1)
        
        # Draw right hand
        if right_wrist is not None and right_fingers is not None:
            # Transform fingers to world space
            right_fingers_world = right_wrist @ right_fingers
            
            # Draw wrist
            rx, ry = int(center_x + float(right_wrist[0, 0, 3]) * 400), int(center_y - float(right_wrist[0, 1, 3]) * 400)
            right_pinch = latest.get("right_pinch_distance", 0)
            color = (0, 255, 0) if right_pinch < 0.02 else (255, 100, 100)
            cv2.circle(blank_frame, (rx, ry), 15, color, -1)
            cv2.putText(blank_frame, "R", (rx - 8, ry + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Draw all 25 finger joints
            for i in range(right_fingers_world.shape[0]):
                fx = int(center_x + float(right_fingers_world[i, 0, 3]) * 400)
                fy = int(center_y - float(right_fingers_world[i, 1, 3]) * 400)
                cv2.circle(blank_frame, (fx, fy), 3, (255, 150, 150), -1)
        
        # Display pinch info
        left_pinch = latest.get("left_pinch_distance", 0)
        right_pinch = latest.get("right_pinch_distance", 0)
        cv2.putText(blank_frame, f"Left Pinch: {left_pinch:.3f}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(blank_frame, f"Right Pinch: {right_pinch:.3f}", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return blank_frame
    
    return generate_frame



if __name__ == "__main__":

    import argparse 
    parser = argparse.ArgumentParser(description="Synthetic Video Streamer for VisionPro")
    parser.add_argument("--ip", type=str, default = None)
    parser.add_argument("--room", type=str, default = None)
    parser.add_argument(
        "--hand-tracking-backend",
        type=str,
        default="grpc",
        choices=["grpc", "webrtc"],
        help="Transport used for hand tracking data (default: grpc)",
    )
    args = parser.parse_args()
    # Create streamer
    if args.ip is not None:
        streamer = VisionProStreamer(ip=args.ip, ht_backend=args.hand_tracking_backend)
    elif args.room is not None:
        streamer = VisionProStreamer(room=args.room, ht_backend=args.hand_tracking_backend)
    else:
        raise ValueError("Must provide either --ip or --room")
    
    streamer.register_frame_callback(hand_tracking_visualizer(streamer))
    
    streamer.configure_video(
        fps=60,            
        size="1280x720",
    )
    streamer.start_webrtc(port=9999)
    
    print("Streaming synthetic video at 60fps...")
    print("Press Ctrl+C to stop")
    cnt = 0 
    try:
        while True:
            time.sleep(1/60.)
    except KeyboardInterrupt:
        print("\nStopping...")
