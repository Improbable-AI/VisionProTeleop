#!/usr/bin/env python3
"""Stream video from a local camera to Vision Pro via WebRTC."""

from avp_stream.streamer import VisionProStreamer
import time

def main(args):
    streamer = VisionProStreamer(ip=args.ip, record=True)
    
    streamer.configure_video(
        device="0:none",
        format="avfoundation",
        size="640x480",
        fps=30,
    )
    streamer.start_webrtc(port=9999)
    
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
    import argparse
    parser = argparse.ArgumentParser(
        description="Stream video from Python to VisionOS with hand tracking."
    )
    parser.add_argument("--ip", type=str, required=True)
    args = parser.parse_args()

    main(args)
