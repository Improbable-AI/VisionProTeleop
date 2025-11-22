"""
Example 5: Stream video with audio to Vision Pro

This example demonstrates how to stream both video and audio from your computer
to the Vision Pro headset using WebRTC.

Requirements:
- A camera device (or use device=None for programmatic frames)
- A microphone device for audio input
- Vision Pro connected to the same network
"""

from avp_stream import VisionProStreamer
import time

# Configuration
VISION_PRO_IP = "10.31.129.206"  # Replace with your Vision Pro's IP address

# Create streamer instance
streamer = VisionProStreamer(ip=VISION_PRO_IP)

# Optional: Add a frame processing callback
def process_frame(frame):
    """
    Process video frames before streaming.
    
    Args:
        frame: numpy array in BGR24 format
    
    Returns:
        Processed frame in BGR24 format
    """
    import cv2
    # Add timestamp overlay
    timestamp = time.strftime("%H:%M:%S")
    cv2.putText(frame, f"Time: {timestamp}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    return frame

streamer.register_frame_callback(process_frame)

# Optional: Add an audio processing callback
def process_audio(audio_frame):
    """
    Process audio frames before streaming.
    
    Args:
        audio_frame: AudioFrame with properties:
            - samples: numpy array of audio samples
            - sample_rate: audio sample rate
            - channels: number of audio channels
    
    Returns:
        Processed AudioFrame
    """
    # Example: Apply volume boost
    audio_frame.samples = audio_frame.samples * 1.2
    return audio_frame

# Uncomment to enable audio processing:
# streamer.register_audio_callback(process_audio)

# Start streaming with audio
print("Starting video and audio streaming...")
print("Make sure to start the VisionOS app and press 'Start'")
print()

streamer.start_video_streaming(
    device="0:none",           # Video device (macOS camera)
    format="avfoundation",     # Video format for macOS
    fps=30,                    # Frame rate
    size="640x480",            # Video resolution
    port=9999,                 # WebRTC server port
    stereo_video=False,        # Set to True for side-by-side stereo video
    stereo_audio=False,        # Set to True for stereo audio
    audio_device=":0",         # Audio device (macOS default microphone)
    audio_format="avfoundation"  # Audio format for macOS
)

print("✓ WebRTC server started")
print("  - Video streaming enabled")
print("  - Audio streaming enabled")
print()
print("Press Ctrl+C to stop")

# Keep the script running
try:
    while True:
        time.sleep(1)
        
        # You can access hand tracking data while streaming
        latest = streamer.get_latest()
        if latest:
            # Print hand positions
            left_pos = latest['left_wrist'][0, :3, 3]
            right_pos = latest['right_wrist'][0, :3, 3]
            print(f"Left: [{left_pos[0]:.2f}, {left_pos[1]:.2f}, {left_pos[2]:.2f}]  "
                  f"Right: [{right_pos[0]:.2f}, {right_pos[1]:.2f}, {right_pos[2]:.2f}]", 
                  end='\r')
            
except KeyboardInterrupt:
    print("\n\nStopping...")
    print("Done!")
