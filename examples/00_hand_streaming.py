from avp_stream import VisionProStreamer
import time 

if __name__ == "__main__":

    import argparse 
    parser = argparse.ArgumentParser(description="Synthetic Video Streamer for VisionPro")
    parser.add_argument("--ip", type=str, required=True)
    args = parser.parse_args()
    # Create streamer
    streamer = VisionProStreamer(ip=args.ip)
    
    
    print("Streaming synthetic video at 60fps...")
    print("Press Ctrl+C to stop")
    cnt = 0 
    try:
        while True:
            print(streamer.latest)
            time.sleep(1/60.)
    except KeyboardInterrupt:
        print("\nStopping...")
