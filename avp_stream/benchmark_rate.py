import time
import argparse
from avp_stream.streamer import VisionProStreamer


class BenchmarkStreamer(VisionProStreamer):
    def __init__(self, *args, **kwargs):
        self.message_count = 0
        self.unique_count = 0
        self.last_position = None
        self.measuring = False
        super().__init__(*args, **kwargs)
        
    def _process_hand_update(self, hand_update, source="grpc"):
        super()._process_hand_update(hand_update, source)
        
        if not self.measuring:
            return
            
        self.message_count += 1
        
        current_pos = (
            hand_update.right_hand.wristMatrix.m03,
            hand_update.right_hand.wristMatrix.m13,
            hand_update.right_hand.wristMatrix.m23
        )
        
        if self.last_position != current_pos:
            self.unique_count += 1
            self.last_position = current_pos


def main():
    parser = argparse.ArgumentParser(description="Benchmark Hand Tracking Data Rate")
    parser.add_argument("--ip", type=str, required=True, help="Vision Pro IP address")
    parser.add_argument("--duration", type=float, default=10.0, help="Duration of the benchmark in seconds")
    args = parser.parse_args()

    print(f"Connecting to Vision Pro at {args.ip}...")
    streamer = BenchmarkStreamer(ip=args.ip, record=False)
    
        
    print(f"Collecting data for {args.duration} seconds...")
    streamer.measuring = True
    start_time = time.time()
    time.sleep(args.duration)
    streamer.measuring = False
    duration = time.time() - start_time
    
    print(f"\nResults:")
    print(f"Duration: {duration:.2f} seconds")
    print(f"Total Messages: {streamer.message_count}")
    print(f"Unique Messages: {streamer.unique_count}")
    print(f"Message Rate: {streamer.message_count / duration:.2f} Hz")
    print(f"Unique Data Rate: {streamer.unique_count / duration:.2f} Hz")

if __name__ == "__main__":
    main()
