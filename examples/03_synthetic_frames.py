"""
Example: Generating synthetic video streams without a camera

This example shows how to create video streams programmatically
without any physical camera device. The frame callback is called
at the specified FPS to generate each frame.
"""

from avp_stream.streamer import VisionProStreamer
import cv2
import numpy as np
import time

# Replace with your Vision Pro's IP address
VISION_PRO_IP = '192.168.86.21'


# Example 1: Animated gradient
def animated_gradient():
    """Create an animated gradient effect"""
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


# Example 2: Moving shapes
def moving_shapes():
    """Create moving geometric shapes"""
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


# Example 3: Visualize hand tracking data
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
        
        if left_wrist is not None:
            # Extract position from transformation matrix (assume last column)
            lx, ly = int(center_x + left_wrist[0, 3] * 200), int(center_y - left_wrist[1, 3] * 200)
            left_pinch = latest.get("left_pinch_distance", 0)
            color = (0, 255, 0) if left_pinch < 0.02 else (100, 100, 255)
            cv2.circle(blank_frame, (lx, ly), 20, color, -1)
            cv2.putText(blank_frame, "L", (lx - 10, ly + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if right_wrist is not None:
            rx, ry = int(center_x + right_wrist[0, 3] * 200), int(center_y - right_wrist[1, 3] * 200)
            right_pinch = latest.get("right_pinch_distance", 0)
            color = (0, 255, 0) if right_pinch < 0.02 else (255, 100, 100)
            cv2.circle(blank_frame, (rx, ry), 20, color, -1)
            cv2.putText(blank_frame, "R", (rx - 10, ry + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Display pinch info
        left_pinch = latest.get("left_pinch_distance", 0)
        right_pinch = latest.get("right_pinch_distance", 0)
        cv2.putText(blank_frame, f"Left Pinch: {left_pinch:.3f}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(blank_frame, f"Right Pinch: {right_pinch:.3f}", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return blank_frame
    
    return generate_frame


# Example 4: Plasma effect
def plasma_effect():
    """Classic plasma demo effect"""
    start_time = time.time()
    
    def generate_frame(blank_frame):
        elapsed = time.time() - start_time
        h, w = blank_frame.shape[:2]
        
        for y in range(h):
            for x in range(w):
                # Plasma formula
                value = np.sin(x * 0.04 + elapsed)
                value += np.sin(y * 0.03 + elapsed)
                value += np.sin((x + y) * 0.03 + elapsed)
                value += np.sin(np.sqrt(x*x + y*y) * 0.02 + elapsed)
                
                # Convert to color
                color_val = int((value + 4) / 8 * 255)
                blank_frame[y, x] = [color_val, 255 - color_val, 128]
        
        return blank_frame
    
    return generate_frame


# Example 5: Text scroller
def text_scroller():
    """Scrolling text display"""
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


if __name__ == "__main__":
    # Create streamer
    streamer = VisionProStreamer(ip=VISION_PRO_IP)
    
    # Choose one of the examples:
    
    # Example 1: Animated gradient
    # streamer.register_frame_callback(animated_gradient())
    
    # Example 2: Moving shapes
    # streamer.register_frame_callback(moving_shapes())
    
    # Example 3: Hand tracking visualizer
    # streamer.register_frame_callback(hand_tracking_visualizer(streamer))
    
    # Example 4: Plasma effect
    streamer.register_frame_callback(plasma_effect())
    
    # Example 5: Text scroller
    # streamer.register_frame_callback(text_scroller())
    
    # Start synthetic video streaming (NO CAMERA!)
    streamer.start_video_streaming(
        device=None,        # No camera device!
        format=None,        # Not needed without camera
        fps=60,            # Generate at 60fps
        size="1280x720",   # HD resolution
        port=9999
    )
    
    # Keep running
    print("Streaming synthetic video at 60fps...")
    print("Press Ctrl+C to stop")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
