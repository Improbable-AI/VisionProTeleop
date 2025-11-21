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
    VISION_PRO_IP = "192.168.86.21" 
    
    streamer = VisionProStreamer(ip=VISION_PRO_IP, record=True)
    
    streamer.start_video_streaming(
        device="0:none",
        format="avfoundation",
        size = "640x480",
        fps = 30, 
        port=9999
    )
    
    try:
        # Keep the script running and show hand tracking data
        while True:
            latest = streamer.get_latest()
            
            if latest:

                pass 
                left_pos = latest["left_wrist"][0, :3, 3]
                right_pos = latest["right_wrist"][0, :3, 3]
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\n✓ Stopped streaming")

if __name__ == "__main__":
    main()
