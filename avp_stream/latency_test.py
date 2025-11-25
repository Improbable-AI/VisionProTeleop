import time
import cv2
import numpy as np
import argparse
import json
import os
from datetime import datetime
from tqdm import tqdm
from avp_stream.streamer import VisionProStreamer

def inject_benchmark_payload(frame, sequence_id, timestamp_ms):
    """
    Injects a 8x9 grid of 16x16 blocks encoding the benchmark payload.
    
    Payload Format (72 bits):
    - Bits 0-7:   Magic Byte (0x5A)
    - Bits 8-39:  Sequence ID (32-bit, Little Endian)
    - Bits 40-71: Timestamp (32-bit, Little Endian)
    """
    rows = 8
    cols = 9
    block_size = 16
    margin = 8
    magic = 0x5A
    
    # Construct the 72-bit payload
    # We'll use a list of 0s and 1s
    bits = []
    
    # 1. Magic Byte (8 bits) - MSB first for the byte itself? 
    # Swift: 
    # var magic: UInt8 = 0
    # for bit in bits[0..<8] { magic = (magic << 1) | bit }
    # So Swift reads MSB first (big-endian bit order within the byte).
    for i in range(7, -1, -1):
        bits.append((magic >> i) & 1)
        
    # 2. Sequence ID (32 bits)
    # Swift:
    # var sequence: UInt32 = 0
    # for bit in bits[8..<(8 + 32)] { sequence = (sequence << 1) | UInt32(bit) }
    # So Swift reads MSB first.
    for i in range(31, -1, -1):
        bits.append((sequence_id >> i) & 1)
        
    # 3. Timestamp (32 bits)
    # Swift:
    # var sentTimestamp: UInt32 = 0
    # for bit in bits[(8 + 32)..<(8 + 64)] { sentTimestamp = (sentTimestamp << 1) | UInt32(bit) }
    # So Swift reads MSB first.
    for i in range(31, -1, -1):
        bits.append((timestamp_ms >> i) & 1)
        
    # Draw the grid
    # Swift reads row by row, then col by col:
    # bits[row * benchmarkCols + col]
    
    # Draw background (black) for the ROI to ensure clean reading
    roi_width = cols * block_size + 2 * margin
    roi_height = rows * block_size + 2 * margin
    cv2.rectangle(frame, (0, 0), (roi_width, roi_height), (0, 0, 0), -1)
    
    for r in range(rows):
        for c in range(cols):
            idx = r * cols + c
            if idx < len(bits) and bits[idx] == 1:
                x = margin + c * block_size
                y = margin + r * block_size
                # Draw white block
                cv2.rectangle(frame, (x, y), (x + block_size, y + block_size), (255, 255, 255), -1)
                
    # Add some text for human debugging
    cv2.putText(frame, f"Seq: {sequence_id}", (10, roi_height + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(frame, f"Ts: {timestamp_ms}", (10, roi_height + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return frame

def main():
    parser = argparse.ArgumentParser(description="Glass-to-Glass Latency Tester")
    parser.add_argument("--ip", type=str, default="169.254.220.107", help="IP address of the Vision Pro")
    parser.add_argument("--fps", type=int, default=30, help="Streaming FPS")
    parser.add_argument("--resolution", type=str, default="640x480", help="Streaming resolution (WIDTHxHEIGHT)")
    parser.add_argument("--sweep", action="store_true", help="Sweep over common resolutions")
    parser.add_argument("--trials", type=int, default=1000, help="Number of trials per resolution in sweep mode")
    parser.add_argument("--warmup", type=int, default=100, help="Number of warmup frames before collecting data")
    parser.add_argument("--stereo", action="store_true", help="Enable stereo video streaming (doubles width)")
    args = parser.parse_args()

    print(f"Connecting to Vision Pro at {args.ip}...")
    # Enable quiet mode for benchmarks when sweeping to avoid interfering with tqdm
    streamer = VisionProStreamer(ip=args.ip, record=False, benchmark_quiet=True)
    
    # State for the callback
    class LatencyState:
        def __init__(self):
            self.sequence_id = 0
            self.streamer = streamer
            self.latencies = []
            self.collecting = False
            self.target_trials = 0
            self.current_resolution = ""
            self.last_frame_time = 0
            self.warmup_frames = 0
            self.in_warmup = False
            
    state = LatencyState()    
    
    def frame_callback(frame):
        # 1. Get current timestamp
        now = time.perf_counter()
        if state.sequence_id == 0:
            state.streamer.reset_benchmark_epoch(now)
            state.last_frame_time = now
        else:
            # Sleep to maintain target FPS (30fps = 33.33ms per frame)
            elapsed = now - state.last_frame_time
            target_frame_time = 1.0 / args.fps
            # if elapsed < target_frame_time:
            #     time.sleep(target_frame_time - elapsed)
            now = time.perf_counter()
            state.last_frame_time = now
            
        timestamp_ms = int((now - state.streamer._benchmark_epoch) * 1000)
        
        # 2. Inject payload
        inject_benchmark_payload(frame, state.sequence_id, timestamp_ms)
        
        # 3. Increment sequence
        state.sequence_id += 1
        
        return frame

    # Register the callback BEFORE starting streaming
    streamer.register_frame_callback(frame_callback)
    
    # Define resolutions for sweep
    if args.stereo:
        # Double the width for stereo mode
        resolutions = [
            ("240p", "852x240"),
            ("360p", "1280x360"),
            ("720p", "2560x720"),
            ("1080p", "3840x1080"),
            ("2160p", "7680x2160")
        ]
    else:
        resolutions = [
            ("240p", "426x240"),
            ("360p", "640x360"),
            ("720p", "1280x720"),
            ("1080p", "1920x1080"),
            ("2160p", "3840x2160")
        ]
    
    if not args.sweep:
        # Single resolution mode
        resolution = args.resolution
        if args.stereo:
            # Double the width for stereo
            width, height = map(int, resolution.split('x'))
            resolution = f"{width * 2}x{height}"
        streamer.start_streaming(device=None, size=resolution, fps=args.fps, stereo_video=args.stereo)
        print("Streaming started. Measuring latency...")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping...")
    else:
        # Sweep mode
        # Start with the first resolution
        first_res_name, first_res_size = resolutions[0]
        streamer.start_streaming(device=None, size=first_res_size, fps=args.fps, stereo_video=args.stereo)
        
        print(f"Starting sweep over {len(resolutions)} resolutions: {[r[0] for r in resolutions]}")
        
        # Prepare results dictionary
        all_results = {
            "timestamp": datetime.now().isoformat(),
            "test_config": {
                "ip": args.ip,
                "fps": args.fps,
                "trials_per_resolution": args.trials,
                "warmup_frames": args.warmup,
                "stereo": args.stereo
            },
            "resolutions": {}
        }
        
        for res_name, res_size in resolutions:
            print(f"\n--- Testing {res_name} ({res_size}) ---")
            
            # Update resolution if not the first one (already started with it)
            if res_size != first_res_size:
                streamer.update_stream_resolution(res_size)
                # Give it a moment to settle
                time.sleep(2)
            
            state.current_resolution = res_name
            state.latencies = []
            state.collecting = True
            state.target_trials = args.trials
            state.warmup_frames = args.warmup
            state.in_warmup = True
            
            # Warmup phase
            print(f"🔥 Warming up ({args.warmup} frames)...")
            warmup_start_seq = state.sequence_id
            warmup_pbar = tqdm(total=args.warmup, desc="Warmup", unit="frames")
            
            warmup_check_seq = warmup_start_seq
            warmup_collected = 0
            
            while warmup_collected < args.warmup:
                event = streamer.wait_for_benchmark_event(warmup_check_seq, timeout=1.0)
                
                if event:
                    warmup_collected += 1
                    warmup_pbar.update(1)
                    warmup_check_seq += 1
                else:
                    # Check if we should skip
                    if state.sequence_id > warmup_check_seq + 100:
                        warmup_pbar.write(f"⚠️  Skipping missing warmup sequence {warmup_check_seq}")
                        warmup_check_seq += 1
            
            warmup_pbar.close()
            state.in_warmup = False
            print(f"✅ Warmup complete. Starting data collection...")
            
            # Collect samples
            # We need to capture the return events.
            # The streamer prints them, but we need to intercept or query them.
            # streamer.wait_for_benchmark_event removes them from the dict.
            # So we should actively poll for events corresponding to the sequences we sent.
            
            start_seq = state.sequence_id
            collected_count = 0
            
            # Wait until we have enough samples
            # We'll poll for events starting from start_seq
            current_check_seq = start_seq
            
            # Create progress bar
            pbar = tqdm(total=args.trials, desc=f"{res_name}", unit="samples", 
                       bar_format='{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed}<{remaining}] RT: {postfix[0]:.1f}ms ± {postfix[1]:.1f}ms',
                       postfix=[0.0, 0.0])
            
            try:
                while collected_count < args.trials:
                    # Wait for the next event with a timeout
                    event = streamer.wait_for_benchmark_event(current_check_seq, timeout=1.0)
                    
                    if event:
                        if event["round_trip_ms"] is not None:
                            state.latencies.append(event["round_trip_ms"])
                            collected_count += 1
                            
                            # Update progress bar with current stats
                            if len(state.latencies) > 0:
                                current_mean = np.mean(state.latencies)
                                current_std = np.std(state.latencies)
                                pbar.postfix[0] = current_mean
                                pbar.postfix[1] = current_std
                            pbar.update(1)
                        current_check_seq += 1
                    else:
                        # Timeout - maybe packet loss or just slow?
                        # Skip this sequence and try next, or just keep waiting?
                        # If we skip too many, we might get stuck.
                        # Let's check if the sequence_id has advanced significantly
                        if state.sequence_id > current_check_seq + 100:
                            pbar.write(f"⚠️  Skipping missing sequence {current_check_seq} (current sent: {state.sequence_id})")
                            current_check_seq += 1
                        else:
                            # Just wait a bit more
                            pass
                            
            except KeyboardInterrupt:
                pbar.close()
                print("\nSweep interrupted.")
                return
            finally:
                pbar.close()

            # Calculate stats
            if state.latencies:
                mean_latency = np.mean(state.latencies)
                jitter = np.std(state.latencies)
                min_lat = np.min(state.latencies)
                max_lat = np.max(state.latencies)
                p50 = np.percentile(state.latencies, 50)
                p95 = np.percentile(state.latencies, 95)
                p99 = np.percentile(state.latencies, 99)
                
                print(f"📊 Results for {res_name}:")
                print(f"   Mean:   {mean_latency:.2f} ms")
                print(f"   Jitter: {jitter:.2f} ms")
                print(f"   Min:    {min_lat} ms")
                print(f"   Max:    {max_lat} ms")
                
                # Store results
                all_results["resolutions"][res_name] = {
                    "resolution": res_size,
                    "mean_ms": float(mean_latency),
                    "std_ms": float(jitter),
                    "min_ms": float(min_lat),
                    "max_ms": float(max_lat),
                    "p50_ms": float(p50),
                    "p95_ms": float(p95),
                    "p99_ms": float(p99),
                    "samples_collected": len(state.latencies),
                    "raw_latencies": [float(x) for x in state.latencies]
                }
            else:
                print(f"⚠️  No data collected for {res_name}")
                all_results["resolutions"][res_name] = {
                    "resolution": res_size,
                    "error": "No data collected"
                }

        # Save results to JSON file
        os.makedirs("benchmarks", exist_ok=True)
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"benchmarks/{timestamp_str}.json"
        
        with open(filename, 'w') as f:
            json.dump(all_results, f, indent=2)
        
        print(f"\n✅ Sweep complete. Results saved to: {filename}")

if __name__ == "__main__":
    main()
