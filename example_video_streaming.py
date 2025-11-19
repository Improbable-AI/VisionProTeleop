#!/usr/bin/env python3
"""
Example script demonstrating video streaming from Python to VisionOS.

Usage:
1. Launch the VisionOS app on Vision Pro
2. Press "Start" with video streaming toggled ON
3. Run this script: python example_video_streaming.py
4. The video feed should appear in the VisionOS app
"""

from avp_stream.streamer import VisionProStreamer
import time

def main():
    # Replace with your Vision Pro's IP address
    # You can find it in the app's ContentView
    VISION_PRO_IP = "169.254.45.56"  # UPDATE THIS!
    
    print("=" * 60)
    print("VisionProTeleop - Video Streaming Example")
    print("=" * 60)
    print("\nBefore running this script, make sure:")
    print("  1. VisionOS app is launched")
    print("  2. 'Start' button is pressed in the app")
    print("  3. Video streaming toggle is enabled")
    print(f"  4. Vision Pro IP ({VISION_PRO_IP}) is correct")
    print("=" * 60)
    
    
    print(f"\n1. Connecting to Vision Pro at {VISION_PRO_IP}...")
    
    # Create streamer instance - this establishes gRPC connection
    streamer = VisionProStreamer(ip=VISION_PRO_IP, record=True)
    
    print("✓ Connected to Vision Pro")
    print("\n2. Starting WebRTC video streaming server...")
    
    # Start video streaming
    # This will:
    # - Start WebRTC server on port 9999
    # - Start HTTP info server on port 8888
    # - VisionOS will automatically discover and connect
    streamer.start_video_streaming(
        video_device="0:none",          # macOS webcam (device 0)
        format="avfoundation",           # macOS video format
        options={
            "video_size": "640x480",     # Resolution
            "framerate": "30"            # FPS
        }
    )
    
    print("✓ Video streaming server started")
    print("\n3. Streaming video to Vision Pro...")
    print("   Press Ctrl+C to stop\n")
    
    try:
        # Keep the script running and show hand tracking data
        while True:
            latest = streamer.get_latest()
            
            # Optional: print hand tracking info
            if latest:

                # print("latest arrived")
                pass 
                left_pos = latest["left_wrist"][0, :3, 3]
                right_pos = latest["right_wrist"][0, :3, 3]
                # print(left_pos)
                # print(f"\rLeft hand: [{left_pos[0]:.2f}, {left_pos[1]:.2f}, {left_pos[2]:.2f}] | "
                #       f"Right hand: [{right_pos[0]:.2f}, {right_pos[1]:.2f}, {right_pos[2]:.2f}]", 
                #       end="", flush=True)
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\n✓ Stopped streaming")
        print("Recording saved with", len(streamer.get_recording()), "frames")

if __name__ == "__main__":
    main()
