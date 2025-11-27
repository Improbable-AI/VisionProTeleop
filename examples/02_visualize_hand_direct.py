"""
Example: Visualizing hand tracking using direct frame updates

This example shows how to use the update_frame() method to directly
push frames to the VisionPro, giving you full control over when and
how frames are generated.
"""

from avp_stream.streamer import VisionProStreamer
import cv2
import numpy as np
import time

def generate_hand_visualization(streamer, width=1280, height=720):
    """Generate a frame visualizing hand tracking data"""
    
    # Create blank frame
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:] = [20, 20, 20]  # Dark background
    
    # Get latest hand tracking data
    latest = streamer.get_latest()
    if latest is None:
        cv2.putText(frame, "Waiting for hand data...", (width//2 - 200, height//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return frame
    
    # Draw virtual 3D space
    center_x, center_y = width // 2, height // 2
    
    # Draw coordinate axes
    cv2.line(frame, (center_x, center_y), (center_x + 100, center_y), (0, 0, 255), 2)  # X
    cv2.line(frame, (center_x, center_y), (center_x, center_y - 100), (0, 255, 0), 2)  # Y
    cv2.putText(frame, "X", (center_x + 110, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(frame, "Y", (center_x, center_y - 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
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
        cv2.circle(frame, (lx, ly), 15, color, -1)
        cv2.putText(frame, "L", (lx - 8, ly + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Draw all 25 finger joints
        for i in range(left_fingers_world.shape[0]):
            fx = int(center_x + float(left_fingers_world[i, 0, 3]) * 400)
            fy = int(center_y - float(left_fingers_world[i, 1, 3]) * 400)
            cv2.circle(frame, (fx, fy), 3, (150, 150, 255), -1)
    
    # Draw right hand
    if right_wrist is not None and right_fingers is not None:
        # Transform fingers to world space
        right_fingers_world = right_wrist @ right_fingers
        
        # Draw wrist
        rx, ry = int(center_x + float(right_wrist[0, 0, 3]) * 400), int(center_y - float(right_wrist[0, 1, 3]) * 400)
        right_pinch = latest.get("right_pinch_distance", 0)
        color = (0, 255, 0) if right_pinch < 0.02 else (255, 100, 100)
        cv2.circle(frame, (rx, ry), 15, color, -1)
        cv2.putText(frame, "R", (rx - 8, ry + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Draw all 25 finger joints
        for i in range(right_fingers_world.shape[0]):
            fx = int(center_x + float(right_fingers_world[i, 0, 3]) * 400)
            fy = int(center_y - float(right_fingers_world[i, 1, 3]) * 400)
            cv2.circle(frame, (fx, fy), 3, (255, 150, 150), -1)
    
    # Display pinch info
    left_pinch = latest.get("left_pinch_distance", 0)
    right_pinch = latest.get("right_pinch_distance", 0)
    cv2.putText(frame, f"Left Pinch: {left_pinch:.3f}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, f"Right Pinch: {right_pinch:.3f}", (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    return frame



if __name__ == "__main__":
    import argparse 
    parser = argparse.ArgumentParser(description="Direct Hand Visualization for VisionPro")
    parser.add_argument("--ip", type=str, required=True)
    args = parser.parse_args()
    
    # Create streamer
    streamer = VisionProStreamer(ip=args.ip)
    
    streamer.configure_video(
        format=None,  
        fps=60,             
        size="1280x720"    
    )

    streamer.start_webrtc()
    
    print("Streaming hand visualization at 60fps using direct frame updates...")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            # Generate frame with hand visualization
            frame = generate_hand_visualization(streamer, width=1280, height=720)
            # Directly update the frame being streamed
            streamer.update_frame(frame)
            # Control update rate
            time.sleep(1/60.)
    except KeyboardInterrupt:
        print("\nStopping...")
