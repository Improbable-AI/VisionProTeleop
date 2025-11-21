"""
Example: Text scroller using direct frame updates

This example shows how to use the update_frame() method to create
a scrolling text animation with full control over frame generation.
"""

from avp_stream.streamer import VisionProStreamer
import cv2
import numpy as np
import time


def generate_text_scroller_frame(elapsed, width=1280, height=720):
    """Generate a frame with scrolling text"""
    message = "Hello VisionPro! This is a synthetic video stream without a camera!"
    
    # Create blank frame
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Gradient background
    for y in range(height):
        color = int(255 * y / height)
        frame[y, :] = [color, color // 2, 255 - color]
    
    # Scrolling text
    text_x = int(width - (elapsed * 100) % (width + 1000))
    cv2.putText(frame, message, (text_x, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
    
    return frame


def generate_animated_gradient_frame(elapsed, width=1280, height=720):
    """Generate a frame with animated gradient effect"""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Create animated gradient
    for y in range(height):
        for x in range(width):
            r = int(127 + 127 * np.sin(elapsed + x * 0.01))
            g = int(127 + 127 * np.sin(elapsed + y * 0.01))
            b = int(127 + 127 * np.sin(elapsed + (x + y) * 0.01))
            frame[y, x] = [b, g, r]
    
    # Add timestamp
    cv2.putText(frame, f"Time: {elapsed:.2f}s", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    return frame


def generate_moving_shapes_frame(elapsed, width=1280, height=720):
    """Generate a frame with moving geometric shapes"""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Black background
    frame[:] = [0, 0, 0]
    
    # Moving circle
    circle_x = int(width/2 + width/3 * np.sin(elapsed))
    circle_y = int(height/2 + height/3 * np.cos(elapsed))
    cv2.circle(frame, (circle_x, circle_y), 50, (0, 255, 0), -1)
    
    # Moving rectangle
    rect_x = int(width/2 + width/3 * np.cos(elapsed * 1.5))
    rect_y = int(height/2 + height/3 * np.sin(elapsed * 1.5))
    cv2.rectangle(frame, 
                  (rect_x - 30, rect_y - 30),
                  (rect_x + 30, rect_y + 30),
                  (255, 0, 0), -1)
    
    # Add FPS counter
    cv2.putText(frame, f"Generated @ 60fps", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    return frame


if __name__ == "__main__":
    import argparse 
    parser = argparse.ArgumentParser(description="Direct Text Scroller for VisionPro")
    parser.add_argument("--ip", type=str, required=True)
    parser.add_argument("--mode", type=str, default="scroller", 
                        choices=["scroller", "gradient", "shapes"],
                        help="Animation mode: scroller, gradient, or shapes")
    args = parser.parse_args()
    
    # Create streamer
    streamer = VisionProStreamer(ip=args.ip)
    
    # Start video streaming without device (synthetic mode)
    streamer.start_video_streaming(
        device=None,        
        format=None,       
        fps=60,            
        size="1280x720",  
        port=9999
    )
    
    print(f"Streaming {args.mode} animation at 60fps using direct frame updates...")
    print("Press Ctrl+C to stop")
    
    # Choose generator function based on mode
    if args.mode == "scroller":
        generator = generate_text_scroller_frame
    elif args.mode == "gradient":
        generator = generate_animated_gradient_frame
    else:
        generator = generate_moving_shapes_frame
    
    start_time = time.time()
    
    try:
        while True:
            elapsed = time.time() - start_time
            
            # Generate frame
            frame = generator(elapsed, width=1280, height=720)
            
            # Directly update the frame being streamed
            streamer.update_frame(frame)
            
            # Control update rate
            time.sleep(1/60.)
    except KeyboardInterrupt:
        print("\nStopping...")
