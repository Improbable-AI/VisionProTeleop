"""Stream hand tracking data from Vision Pro and print the latest tracking state."""

from avp_stream import VisionProStreamer
import time 

if __name__ == "__main__":

    import argparse 
    parser = argparse.ArgumentParser(description="Synthetic Video Streamer for VisionPro")
    parser.add_argument("--ip", type=str, required=True)
    args = parser.parse_args()
    # Create streamer
    streamer = VisionProStreamer(ip=args.ip)
    
    
    try:
        while True:
            print(streamer.get_latest())
            time.sleep(1/60.)
    except KeyboardInterrupt:
        print("\nStopping...")
