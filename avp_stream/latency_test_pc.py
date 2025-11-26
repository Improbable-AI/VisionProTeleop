import time
import numpy as np
import argparse
import json
import os
from datetime import datetime
from tqdm import tqdm
from avp_stream.streamer import VisionProStreamer

def generate_random_pc(n_points):
    """Generate random points in a unit cube and random colors."""
    points = np.random.rand(n_points, 3).astype(np.float32)
    # Center around 0,0,0 and scale
    points = (points - 0.5) * 0.5
    colors = np.random.randint(0, 255, (n_points, 3), dtype=np.uint8)
    return points, colors

def main():
    parser = argparse.ArgumentParser(description="Point Cloud Latency Benchmark")
    parser.add_argument("--ip", type=str, default="10.29.230.57", help="IP address of the Vision Pro")
    parser.add_argument("--trials", type=int, default=1000, help="Number of trials per point count")
    parser.add_argument("--warmup", type=int, default=100, help="Number of warmup frames")
    parser.add_argument("--quantize", action="store_true", help="Enable Float16 quantization")
    parser.add_argument("--draco", action="store_true", help="Enable Draco compression")
    args = parser.parse_args()

    print(f"Connecting to Vision Pro at {args.ip}...")
    # Enable quiet mode for benchmarks to avoid interfering with tqdm
    streamer = VisionProStreamer(ip=args.ip, record=False, benchmark_quiet=True)
    
    # Configure minimal video to keep connection alive
    # streamer.configure_video(device=None, size="640x480", fps=30)
    streamer.enable_point_cloud()
    streamer.serve()
    
    print("Waiting for WebRTC connection...")
    
    # Wait for point cloud channel to be ready
    for i in range(100): # Wait up to 10 seconds
        if (streamer._point_cloud_channel is not None and 
            streamer._point_cloud_channel.readyState == "open"):
            print("✅ Point cloud channel is open!")
            break
        time.sleep(0.1)
        if i % 10 == 0:
            print(f"   Waiting for channel... ({i*0.1:.1f}s)")
    else:
        print("⚠️ Timeout waiting for point cloud channel. Attempting to start anyway (might fail)...") 
    
    point_counts = [100000]
    
    all_results = {
        "timestamp": datetime.now().isoformat(),
        "test_config": {
            "ip": args.ip,
            "trials_per_count": args.trials,
            "warmup_frames": args.warmup,
            "quantize": args.quantize,
            "draco": args.draco,
        },
        "point_counts": {}
    }
    
    sequence_id = 0
    streamer.reset_benchmark_epoch()
    
    for n_points in point_counts:
        print(f"\n--- Testing {n_points} points ---")
        
        latencies = []
        proc_times = []
        trans_times = []
        jitters = []
        # Store encoding times: {seq_id: encoding_time_ms}
        encoding_times = {}

        try:
            # Warmup
            print(f"🔥 Warming up for {args.warmup} frames...")
            for _ in range(args.warmup):
                points, colors = generate_random_pc(n_points)
                now_ms = (time.perf_counter() - streamer._benchmark_epoch) * 1000
                streamer.send_benchmark_pc(points, colors, sequence_id, now_ms, quantize=args.quantize, use_draco=args.draco)
                sequence_id += 1
                time.sleep(1/30.0) # Limit to ~30fps
            
            # Data Collection
            print(f"✅ Starting data collection...")
            pbar = tqdm(total=args.trials, desc=f"{n_points} pts", unit="samples",
                       bar_format='{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed}<{remaining}] {postfix}')
            
            collected_count = 0
            current_check_seq = sequence_id
            
            while collected_count < args.trials:
                # Send a frame
                points, colors = generate_random_pc(n_points)
                now_ms = (time.perf_counter() - streamer._benchmark_epoch) * 1000
                enc_time = streamer.send_benchmark_pc(points, colors, sequence_id, now_ms, quantize=args.quantize, use_draco=args.draco)
                encoding_times[sequence_id] = enc_time
                
                # Send
                sequence_id += 1
                
                # Check for events
                while True:
                    event = streamer.wait_for_benchmark_event(current_check_seq, timeout=0.001) # Very short timeout to not block sending
                    if event:
                        if event["rtt"] is not None:
                            rtt = event["rtt"]
                            proc_time = event.get("proc_time", 0.0)
                            
                            # Get encoding time for this sequence
                            seq_recv = current_check_seq
                            enc_time_val = encoding_times.get(seq_recv, 0.0)
                            
                            # Net latency = RTT - Proc - Encode
                            net_time = rtt - proc_time - enc_time_val
                            
                            latencies.append(rtt)
                            proc_times.append(proc_time)
                            trans_times.append(net_time)
                            
                            # Cleanup
                            if seq_recv in encoding_times:
                                del encoding_times[seq_recv]
                            
                            collected_count += 1
                            
                            if len(latencies) > 0:
                                pbar.set_postfix({
                                    "RT": f"{np.mean(latencies):.1f}ms",
                                    "Enc": f"{np.mean(list(encoding_times.values())) if encoding_times else 0.0:.1f}ms", # Approx
                                    "Net": f"{np.mean(trans_times):.1f}ms",
                                    "Proc": f"{np.mean(proc_times):.1f}ms"
                                })
                            pbar.update(1)
                        current_check_seq += 1
                    else:
                        # No event yet, break to send next frame
                        break
                
                # Sleep to maintain ~30fps
                time.sleep(1/30.0)
            
        except KeyboardInterrupt:
            print("\nInterrupted.")
            return
        finally:
            if 'pbar' in locals():
                pbar.close()
            
        # Stats
        if latencies:
            mean_latency = np.mean(latencies)
            jitter = np.std(latencies)
            min_lat = np.min(latencies)
            max_lat = np.max(latencies)
            p50 = np.percentile(latencies, 50)
            p95 = np.percentile(latencies, 95)
            
            mean_proc_time = np.mean(proc_times) if proc_times else 0.0
            mean_trans_time = np.mean(trans_times) if trans_times else 0.0
            
            print(f"📊 Results for {n_points} points:")
            print(f"   Mean RTT:   {mean_latency:.2f} ms")
            print(f"   Jitter:     {jitter:.2f} ms")
            print(f"   Min RTT:    {min_lat} ms")
            print(f"   Max RTT:    {max_lat} ms")
            print(f"   Mean Proc:  {mean_proc_time:.2f} ms")
            print(f"   Mean Net:   {mean_trans_time:.2f} ms")
            
            all_results["point_counts"][n_points] = {
                "mean_rtt_ms": float(mean_latency),
                "std_rtt_ms": float(jitter),
                "min_rtt_ms": float(min_lat),
                "max_rtt_ms": float(max_lat),
                "p50_rtt_ms": float(p50),
                "p95_rtt_ms": float(p95),
                "mean_processing_ms": float(mean_proc_time),
                "mean_network_ms": float(mean_trans_time),
                "samples_collected": len(latencies)
            }
        else:
            print(f"⚠️ No data collected for {n_points} points")
            
    # Save
    os.makedirs("benchmarks", exist_ok=True)
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"benchmarks/pc_latency_{timestamp_str}.json"
    
    with open(filename, 'w') as f:
        json.dump(all_results, f, indent=2)
    
    print(f"\n✅ Benchmark complete. Results saved to: {filename}")

if __name__ == "__main__":
    main()
