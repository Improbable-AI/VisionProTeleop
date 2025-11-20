"""
Example: Using custom frame processing with VisionProStreamer

This example shows how to register a callback function to process
video frames before they are sent via WebRTC to the Vision Pro.
"""

from avp_stream.streamer import VisionProStreamer
import cv2
import numpy as np

# Replace with your Vision Pro's IP address
VISION_PRO_IP = '10.29.249.251'


# Example 1: Simple text overlay
def add_text_overlay(frame):
    """Add text overlay to the frame"""
    cv2.putText(frame, "Hello VisionPro!", (50, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    return frame


# Example 2: Draw hand tracking data on video
def overlay_hand_tracking(streamer):
    """Returns a callback that overlays hand tracking data"""
    def process_frame(frame):
        latest = streamer.get_latest()
        if latest is None:
            return frame
        
        # Draw hand position indicators
        h, w = frame.shape[:2]
        
        # Get wrist positions (these are 4x4 transformation matrices)
        left_wrist = latest.get("left_wrist")
        right_wrist = latest.get("right_wrist")
        
        # Draw pinch status
        left_pinch = latest.get("left_pinch_distance", 0)
        right_pinch = latest.get("right_pinch_distance", 0)
        
        # Visualize pinch state
        left_color = (0, 255, 0) if left_pinch < 0.02 else (0, 0, 255)
        right_color = (0, 255, 0) if right_pinch < 0.02 else (0, 0, 255)
        
        cv2.putText(frame, f"L Pinch: {left_pinch:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, left_color, 2)
        cv2.putText(frame, f"R Pinch: {right_pinch:.3f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, right_color, 2)
        
        # Draw circles to indicate pinch state
        cv2.circle(frame, (w - 50, 30), 15, left_color, -1)
        cv2.circle(frame, (w - 50, 60), 15, right_color, -1)
        
        return frame
    return process_frame


# Example 3: Edge detection filter
def edge_detection_filter(frame):
    """Apply edge detection to the frame"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 100, 200)
    # Convert back to BGR for display
    return cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)


# Example 4: Composite multiple effects
def fancy_overlay(frame):
    """Multiple effects combined"""
    # Add a semi-transparent overlay
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (frame.shape[1], 80), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.3, frame, 0.7, 0)
    
    # Add timestamp
    import time
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    cv2.putText(frame, timestamp, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # Add a border
    cv2.rectangle(frame, (0, 0), (frame.shape[1]-1, frame.shape[0]-1), 
                  (0, 255, 0), 2)
    
    return frame


if __name__ == "__main__":
    # Create streamer
    streamer = VisionProStreamer(ip=VISION_PRO_IP)
    
    # Choose one of the examples:
    
    # Example 1: Simple text
    # streamer.register_frame_callback(add_text_overlay)
    
    # Example 2: Hand tracking overlay
    streamer.register_frame_callback(overlay_hand_tracking(streamer))
    
    # Example 3: Edge detection
    # streamer.register_frame_callback(edge_detection_filter)
    
    # Example 4: Fancy overlay
    # streamer.register_frame_callback(fancy_overlay)
    
    # Example 5: Lambda for quick tests
    # streamer.register_frame_callback(
    #     lambda frame: cv2.flip(frame, 1)  # Horizontal flip
    # )
    
    # Start video streaming with the registered callback
    streamer.start_video_streaming(
        device="0:none",
        format="avfoundation",
        size = "640x480",
        fps = 30, 
        port=9999
    )
    
    # Keep running
    print("Streaming with custom frame processing...")
    print("Press Ctrl+C to stop")
    try:
        while True:
            import time
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
