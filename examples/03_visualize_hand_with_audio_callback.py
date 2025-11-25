"""
Example: Hand tracking visualization with audio feedback

This example visualizes hand tracking data and plays a beep sound
whenever a pinch gesture is detected on either hand. The audio is
generated programmatically and streamed in real-time to Vision Pro.
"""

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
        
        # Get latest hand tracking data (use cache for consistency with audio)
        latest = streamer.get_latest(use_cache=True, cache_ms=10)
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
        
        # Display pinch info (streamlined for performance)
        left_pinch = latest.get("left_pinch_distance", 0)
        right_pinch = latest.get("right_pinch_distance", 0)
        
        # Simple pinch indicators (minimal rendering overhead)
        if left_pinch < 0.02:
            cv2.putText(blank_frame, "L", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        if right_pinch < 0.02:
            cv2.putText(blank_frame, "R", (w - 50, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        
        return blank_frame
    
    return generate_frame


def beep_audio_generator(streamer):
    """Generate beep audio when pinch is detected (highly optimized)"""
    import math
    import numpy as np
    
    # Audio state
    sample_rate = 48000  # Standard audio sample rate
    beep_frequency = 800  # Hz (beep tone)
    beep_duration = 0.15  # seconds
    samples_per_beep = int(sample_rate * beep_duration)
    
    # Pre-generate sine wave lookup tables for both tones (much faster than math.sin)
    left_freq = beep_frequency - 100
    right_freq = beep_frequency + 100
    
    # Generate one full period of each sine wave
    left_samples = int(sample_rate / left_freq)
    right_samples = int(sample_rate / right_freq)
    left_sine_lut = np.sin(2 * np.pi * np.arange(left_samples) / left_samples) * 0.3
    right_sine_lut = np.sin(2 * np.pi * np.arange(right_samples) / right_samples) * 0.3
    
    # Track pinch state
    left_was_pinching = False
    right_was_pinching = False
    
    # Beep state
    left_beep_samples_remaining = 0
    right_beep_samples_remaining = 0
    
    # Phase counters for LUT indexing
    left_phase = 0
    right_phase = 0
    
    # Pre-allocate silence buffer (reused for efficiency)
    silence_buffer = np.zeros(1024, dtype=np.int16)  # Max typical audio frame size
    
    def generate_audio(audio_frame):
        nonlocal left_was_pinching, right_was_pinching
        nonlocal left_beep_samples_remaining, right_beep_samples_remaining
        nonlocal left_phase, right_phase
        
        # Get latest hand tracking data with caching (only check every ~20ms, not every audio frame)
        latest = streamer.get_latest(use_cache=True, cache_ms=20)
        
        if latest is None:
            return audio_frame
        
        # Check pinch state
        left_pinch = latest.get("left_pinch_distance", 1.0)
        right_pinch = latest.get("right_pinch_distance", 1.0)
        
        left_pinching = left_pinch < 0.02
        right_pinching = right_pinch < 0.02
        
        # Trigger beep on pinch start
        if left_pinching and not left_was_pinching:
            left_beep_samples_remaining = samples_per_beep
            print("🔊 Left pinch")
        
        if right_pinching and not right_was_pinching:
            right_beep_samples_remaining = samples_per_beep
            print("🔊 Right pinch")
        
        left_was_pinching = left_pinching
        right_was_pinching = right_pinching
        
        num_samples = audio_frame.samples
        
        # Fast path: if no beeps, use pre-allocated silence buffer
        if left_beep_samples_remaining == 0 and right_beep_samples_remaining == 0:
            audio_bytes = silence_buffer[:num_samples].tobytes()
        else:
            # Use vectorized NumPy operations (much faster than Python loops)
            output = np.zeros(num_samples, dtype=np.float32)
            
            if left_beep_samples_remaining > 0:
                count = min(left_beep_samples_remaining, num_samples)
                # Use lookup table with wrapping
                indices = (left_phase + np.arange(count)) % left_samples
                envelope = np.minimum(1.0, left_beep_samples_remaining / (sample_rate * 0.01))
                output[:count] += left_sine_lut[indices] * envelope
                left_phase = (left_phase + count) % left_samples
                left_beep_samples_remaining -= count
            
            if right_beep_samples_remaining > 0:
                count = min(right_beep_samples_remaining, num_samples)
                indices = (right_phase + np.arange(count)) % right_samples
                envelope = np.minimum(1.0, right_beep_samples_remaining / (sample_rate * 0.01))
                output[:count] += right_sine_lut[indices] * envelope
                right_phase = (right_phase + count) % right_samples
                right_beep_samples_remaining -= count
            
            # Convert to int16 (vectorized)
            audio_data = np.clip(output * 32767, -32768, 32767).astype(np.int16)
            audio_bytes = audio_data.tobytes()
        
        # Update audio frame
        for plane in audio_frame.planes:
            plane.update(audio_bytes)
        
        return audio_frame
    
    return generate_audio


if __name__ == "__main__":
    import argparse 
    parser = argparse.ArgumentParser(description="Hand Tracking Visualizer with Audio Feedback")
    parser.add_argument("--ip", type=str, required=True, help="Vision Pro IP address")
    args = parser.parse_args()
    
    # Create streamer
    streamer = VisionProStreamer(ip=args.ip)
    
    # Register video callback for hand visualization
    streamer.register_frame_callback(hand_tracking_visualizer(streamer))
    
    # Register audio callback for beep generation
    streamer.register_audio_callback(beep_audio_generator(streamer))
    
    # Start video and audio streaming
    streamer.start_streaming(
        device=None,           # No camera - synthetic video
        format=None,           
        fps=60,                # 60fps video for low latency (matches 01_visualize_hand_callback.py)
        size="1280x720",       # HD resolution
        port=9999,
        audio_device=None,     # No microphone - synthetic audio
        audio_format=None
    )
    
    print("=" * 60)
    print("Streaming hand tracking visualization with audio feedback")
    print("=" * 60)
    print()
    print("📹 Video: Hand tracking visualization at 60fps (low latency)")
    print("🔊 Audio: Beep sounds when pinching (synced with video)")
    print("   - Left hand pinch: Lower tone beep")
    print("   - Right hand pinch: Higher tone beep")
    print()
    print("Make sure Vision Pro app is running and connected!")
    print("Press Ctrl+C to stop")
    print()
    
    try:
        while True:
            time.sleep(1/60.)
    except KeyboardInterrupt:
        print("\n\nStopping...")
        print("Done!")
