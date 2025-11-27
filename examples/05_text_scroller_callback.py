"""
Example: Text scroller using frame callbacks

This example shows how to use register_frame_callback() to create
a scrolling text animation. The callback is invoked automatically
by the video streaming system.
"""

from avp_stream.streamer import VisionProStreamer
import cv2
import numpy as np
import time


def text_scroller_callback():
    """Create a scrolling text animation callback"""
    start_time = time.time()
    message = "Hello VisionPro! This is a synthetic video stream without a camera!"
    
    def generate_frame(blank_frame):
        elapsed = time.time() - start_time
        h, w = blank_frame.shape[:2]
        
        # Gradient background
        for y in range(h):
            color = int(255 * y / h)
            blank_frame[y, :] = [color, color // 2, 255 - color]
        
        # Scrolling text
        text_x = int(w - (elapsed * 100) % (w + 1000))
        cv2.putText(blank_frame, message, (text_x, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
        
        return blank_frame
    
    return generate_frame


def animated_gradient_callback():
    """Create an animated gradient effect callback"""
    start_time = time.time()
    
    def generate_frame(blank_frame):
        elapsed = time.time() - start_time
        h, w = blank_frame.shape[:2]
        
        # Create animated gradient
        for y in range(h):
            for x in range(w):
                r = int(127 + 127 * np.sin(elapsed + x * 0.01))
                g = int(127 + 127 * np.sin(elapsed + y * 0.01))
                b = int(127 + 127 * np.sin(elapsed + (x + y) * 0.01))
                blank_frame[y, x] = [b, g, r]
        
        # Add timestamp
        cv2.putText(blank_frame, f"Time: {elapsed:.2f}s", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return blank_frame
    
    return generate_frame


def moving_shapes_callback():
    """Create moving geometric shapes callback"""
    start_time = time.time()
    
    def generate_frame(blank_frame):
        elapsed = time.time() - start_time
        h, w = blank_frame.shape[:2]
        
        # Black background
        blank_frame[:] = [0, 0, 0]
        
        # Moving circle
        circle_x = int(w/2 + w/3 * np.sin(elapsed))
        circle_y = int(h/2 + h/3 * np.cos(elapsed))
        cv2.circle(blank_frame, (circle_x, circle_y), 50, (0, 255, 0), -1)
        
        # Moving rectangle
        rect_x = int(w/2 + w/3 * np.cos(elapsed * 1.5))
        rect_y = int(h/2 + h/3 * np.sin(elapsed * 1.5))
        cv2.rectangle(blank_frame, 
                      (rect_x - 30, rect_y - 30),
                      (rect_x + 30, rect_y + 30),
                      (255, 0, 0), -1)
        
        # Add FPS counter
        cv2.putText(blank_frame, f"Generated @ 60fps", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return blank_frame
    
    return generate_frame


if __name__ == "__main__":
    import argparse 
    parser = argparse.ArgumentParser(description="Callback Text Scroller for VisionPro")
    parser.add_argument("--ip", type=str, required=True)
    parser.add_argument("--mode", type=str, default="scroller", 
                        choices=["scroller", "gradient", "shapes"],
                        help="Animation mode: scroller, gradient, or shapes")
    args = parser.parse_args()
    
    # Create streamer
    streamer = VisionProStreamer(ip=args.ip)
    
    # Choose callback based on mode
    if args.mode == "scroller":
        callback = text_scroller_callback()
    elif args.mode == "gradient":
        callback = animated_gradient_callback()
    else:
        callback = moving_shapes_callback()
    
    # Register the callback
    streamer.register_frame_callback(callback)
    
    # Configure video streaming without device (synthetic mode)
    streamer.configure_video(
        fps=60,            
        size="1280x720",
    )
    streamer.start_webrtc(port=9999)
    
    print(f"Streaming {args.mode} animation at 60fps using callbacks...")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
