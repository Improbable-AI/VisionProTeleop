"""Apply custom frame processing (overlays, filters) to camera video before streaming."""

from avp_stream.streamer import VisionProStreamer
import cv2
import numpy as np


# Example 1: Simple text overlay
def add_text_overlay(frame):
    """Add text overlay to the frame"""
    cv2.putText(frame, "Hello VisionPro!", (50, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    return frame


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


    import argparse
    parser = argparse.ArgumentParser(description="Custom Frame Processing with VisionProStreamer")
    parser.add_argument("--ip", type=str, required = True, help="IP address of the Vision Pro device")
    args = parser.parse_args()

    # Create streamer
    streamer = VisionProStreamer(ip=args.ip)
    
    # Choose one of the examples:
    
    # Example 1: Simple text
    streamer.register_frame_callback(add_text_overlay)
    
    # Example 2: Edge detection
    # streamer.register_frame_callback(edge_detection_filter)
    
    # Example 3: Fancy overlay
    # streamer.register_frame_callback(fancy_overlay)
    
    # Example 4: Lambda for quick tests
    # streamer.register_frame_callback(
    #     lambda frame: cv2.flip(frame, 1)  # Horizontal flip
    # )
    
    # Configure video streaming with the registered callback
    streamer.configure_video(
        device="0:none",
        format="avfoundation",
        size="640x480",
        fps=30,
    )
    streamer.start_webrtc(port=9999)
    
    # Keep running
    print("Streaming with custom frame processing...")
    print("Press Ctrl+C to stop")
    try:
        while True:
            import time
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
