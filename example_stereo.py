"""
Example script demonstrating stereo video streaming to Vision Pro.

This script shows how to enable stereo mode where a side-by-side 
stereo video stream is sent to the Vision Pro and rendered separately
to the left and right eyes.
"""

from avp_stream import VisionProStreamer
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='VisionPro Stereo Video Streaming Example')
    parser.add_argument('--ip', type=str, required=True, help='Vision Pro IP address')
    parser.add_argument('--video-device', type=str, default='0:none', 
                        help='Video device identifier (default: "0:none" for macOS)')
    parser.add_argument('--width', type=int, default=1280, 
                        help='Video width (default: 1280 for side-by-side stereo)')
    parser.add_argument('--height', type=int, default=480, 
                        help='Video height (default: 480)')
    parser.add_argument('--framerate', type=int, default=30, 
                        help='Video framerate (default: 30)')
    parser.add_argument('--port', type=int, default=9999, 
                        help='WebRTC server port (default: 9999)')
    parser.add_argument('--stereo', action='store_true', 
                        help='Enable stereo mode (side-by-side video)')
    args = parser.parse_args()
    
    print("=" * 60)
    print("VisionPro Stereo Video Streaming")
    print("=" * 60)
    print(f"Vision Pro IP: {args.ip}")
    print(f"Video Device: {args.video_device}")
    print(f"Resolution: {args.width}x{args.height}")
    print(f"Framerate: {args.framerate}")
    print(f"Stereo Mode: {'ENABLED' if args.stereo else 'DISABLED'}")
    print("=" * 60)
    
    # Initialize streamer (for hand tracking data)
    streamer = VisionProStreamer(ip=args.ip, record=False)
    
    # Start video streaming
    print("\nStarting video streaming...")
    streamer.start_video_streaming(
        video_device=args.video_device,
        format="avfoundation",
        options={
            "video_size": f"{args.width}x{args.height}",
            "framerate": str(args.framerate)
        },
        port=args.port,
        stereo=args.stereo  # NEW: Enable stereo mode
    )
    
    print("\n✓ Video streaming started!")
    if args.stereo:
        print("  Note: Ensure your video source is side-by-side stereo format")
        print("  Left eye image should be on the left half, right eye on the right half")
    
    print("\nStreaming hand tracking data...")
    print("Press Ctrl+C to stop\n")
    
    # Stream hand tracking data
    try:
        while True:
            latest = streamer.get_latest()
            if latest:
                head_pos = latest['head'][:, :3, -1]
                right_wrist_pos = latest['right_wrist'][:, :3, -1]
                print(f"Head: {head_pos.flatten()}, Right Wrist: {right_wrist_pos.flatten()}", end='\r')
    except KeyboardInterrupt:
        print("\n\nStopping...")
