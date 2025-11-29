"""
Simulation Streaming Latency Test

This script measures the end-to-end latency of MuJoCo simulation streaming to Vision Pro.
It injects timestamp data into the pose updates and measures round-trip time when
the Vision Pro echoes it back.

The test works by:
1. Including a special "__benchmark__" body in the pose data with encoded timestamps
2. Swift detects this and echoes back via the hand tracking data channel
3. Python measures the round-trip latency

For the echo mechanism to work, Swift needs to be updated to detect the benchmark
body and send it back. Alternatively, this test measures one-way latency by
checking if poses are being received and tracking update frequency.

Example:
    python -m avp_stream.sim_latency_test --ip 192.168.86.21

    # With sweep over different update rates
    python -m avp_stream.sim_latency_test --ip 192.168.86.21 --sweep

Requirements:
    - MuJoCo
    - Vision Pro with Tracking Streamer app running
"""

import time
import numpy as np
import argparse
import json
import os
import mujoco
from datetime import datetime
from tqdm import tqdm
from pathlib import Path
from threading import Thread, Lock, Event
from copy import deepcopy

from avp_stream import VisionProStreamer

# Get the directory containing demo assets
ASSETS_DIR = Path(__file__).resolve().parent.parent / "assets" / "mujoco_demos"


class SimLatencyTester:
    """
    Measures simulation streaming latency to Vision Pro.
    
    This class injects benchmark timestamps into the pose data and measures
    the round-trip time when Vision Pro echoes them back via the hand tracking
    data channel.
    
    Attributes:
        streamer: VisionProStreamer instance
        sequence_id: Current benchmark sequence number
        latencies: List of measured latencies
        epoch: Time reference for timestamp calculations
    """
    
    # Magic number to identify benchmark pose data
    BENCHMARK_MAGIC = 0.123456789
    
    def __init__(self, streamer: VisionProStreamer, benchmark_quiet: bool = True):
        """
        Initialize the latency tester.
        
        Args:
            streamer: VisionProStreamer instance configured for simulation
            benchmark_quiet: Suppress per-sample printouts
        """
        self.streamer = streamer
        self.sequence_id = 0
        self.latencies = []
        self.epoch = None
        self.benchmark_quiet = benchmark_quiet
        
        # Thread-safe data structures
        self._lock = Lock()
        self._sent_timestamps = {}  # sequence_id -> sent_timestamp_ms
        self._pending_events = {}   # sequence_id -> event dict
        self._event = Event()       # For signaling new events
        
        # Stats tracking
        self.update_count = 0
        self.update_times = []
        self.last_update_time = None
        
    def reset(self):
        """Reset the latency tester state for a new test run."""
        with self._lock:
            self.sequence_id = 0
            self.latencies = []
            self._sent_timestamps = {}
            self._pending_events = {}
            self.update_count = 0
            self.update_times = []
            self.last_update_time = None
        self.epoch = time.perf_counter()
        self.streamer.reset_benchmark_epoch(self.epoch)
    
    def inject_benchmark_into_poses(self, poses: dict) -> dict:
        """
        Inject benchmark timestamp data into pose dictionary.
        
        Adds a special "__benchmark__" body with encoded sequence ID and timestamp.
        The format is designed to be detected by Swift and echoed back.
        
        Args:
            poses: Dictionary of body poses {name: {"xpos": [x,y,z], "xquat": [w,x,y,z]}}
            
        Returns:
            Modified poses dictionary with benchmark data
        """
        if self.epoch is None:
            self.epoch = time.perf_counter()
            
        now = time.perf_counter()
        timestamp_ms = int((now - self.epoch) * 1000)
        
        # Store sent timestamp for round-trip calculation
        with self._lock:
            self._sent_timestamps[self.sequence_id] = timestamp_ms
        
        # Encode benchmark data in a special body
        # Position encodes: [magic, sequence_id, timestamp_ms]
        # Quaternion encodes: [sequence_high, sequence_low, timestamp_high, timestamp_low]
        seq_high = (self.sequence_id >> 16) & 0xFFFF
        seq_low = self.sequence_id & 0xFFFF
        ts_high = (timestamp_ms >> 16) & 0xFFFF
        ts_low = timestamp_ms & 0xFFFF
        
        poses["__benchmark__"] = {
            "xpos": [self.BENCHMARK_MAGIC, float(self.sequence_id), float(timestamp_ms)],
            "xquat": [float(seq_high) / 65535.0, float(seq_low) / 65535.0, 
                      float(ts_high) / 65535.0, float(ts_low) / 65535.0]
        }
        
        self.sequence_id += 1
        return poses
    
    def handle_benchmark_response(self, event: dict):
        """
        Handle a benchmark response from Vision Pro.
        
        This is called when Swift echoes back the benchmark data.
        
        Args:
            event: Dictionary with sequence_id, sent_timestamp_ms, etc.
        """
        seq_id = event.get("sequence_id")
        if seq_id is None:
            return
            
        with self._lock:
            sent_ts = self._sent_timestamps.get(seq_id)
            if sent_ts is not None:
                now = time.perf_counter()
                receive_ms = int((now - self.epoch) * 1000)
                round_trip_ms = receive_ms - sent_ts
                
                event["sent_timestamp_ms"] = sent_ts
                event["python_receive_ms"] = receive_ms
                event["round_trip_ms"] = round_trip_ms
                
                self._pending_events[seq_id] = event
                del self._sent_timestamps[seq_id]
                
        self._event.set()
        
        if not self.benchmark_quiet:
            rt = event.get("round_trip_ms")
            if rt is not None:
                print(f"🧪 Sim benchmark seq={seq_id} round-trip={rt} ms")
    
    def record_update(self):
        """Record a pose update for frequency tracking."""
        now = time.perf_counter()
        self.update_count += 1
        
        if self.last_update_time is not None:
            dt = now - self.last_update_time
            self.update_times.append(dt)
            # Keep only last 100 samples
            if len(self.update_times) > 100:
                self.update_times.pop(0)
                
        self.last_update_time = now
    
    def get_update_frequency(self) -> float:
        """Get the current update frequency in Hz."""
        if len(self.update_times) < 2:
            return 0.0
        avg_dt = np.mean(self.update_times)
        return 1.0 / avg_dt if avg_dt > 0 else 0.0
    
    def wait_for_event(self, sequence_id: int, timeout: float = 1.0) -> dict:
        """
        Wait for a specific benchmark event.
        
        Args:
            sequence_id: The sequence ID to wait for
            timeout: Maximum time to wait in seconds
            
        Returns:
            Event dictionary or None if timeout
        """
        deadline = time.time() + timeout
        
        while time.time() < deadline:
            with self._lock:
                if sequence_id in self._pending_events:
                    return self._pending_events.pop(sequence_id)
            
            remaining = deadline - time.time()
            if remaining > 0:
                self._event.wait(timeout=min(0.1, remaining))
                self._event.clear()
        
        return None
    
    def get_stats(self) -> dict:
        """Get current latency statistics."""
        if not self.latencies:
            return {
                "count": 0,
                "mean_ms": 0.0,
                "std_ms": 0.0,
                "min_ms": 0.0,
                "max_ms": 0.0,
                "p50_ms": 0.0,
                "p95_ms": 0.0,
                "p99_ms": 0.0,
                "update_hz": self.get_update_frequency()
            }
        
        return {
            "count": len(self.latencies),
            "mean_ms": float(np.mean(self.latencies)),
            "std_ms": float(np.std(self.latencies)),
            "min_ms": float(np.min(self.latencies)),
            "max_ms": float(np.max(self.latencies)),
            "p50_ms": float(np.percentile(self.latencies, 50)),
            "p95_ms": float(np.percentile(self.latencies, 95)),
            "p99_ms": float(np.percentile(self.latencies, 99)),
            "update_hz": self.get_update_frequency()
        }


class BenchmarkSimulation:
    """
    A minimal MuJoCo simulation for benchmarking purposes.
    
    This creates a simple scene with a few bodies that can be used
    to measure streaming latency without complex physics.
    """
    
    def __init__(self, num_bodies: int = 10):
        """
        Initialize the benchmark simulation.
        
        Args:
            num_bodies: Number of bodies to include in the simulation
        """
        self.num_bodies = num_bodies
        self.model = None
        self.data = None
        self.xml_path = None
        self._create_benchmark_scene()
    
    def _create_benchmark_scene(self):
        """Create a minimal MuJoCo scene for benchmarking."""
        # Create a temporary XML file with simple bodies
        xml_content = f'''<?xml version="1.0"?>
<mujoco model="benchmark_scene">
    <option gravity="0 0 0"/>
    <worldbody>
        <light pos="0 0 3" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="10 10 0.1" rgba="0.8 0.8 0.8 1"/>
        
        <!-- Benchmark bodies -->
        {''.join([f"""
        <body name="body_{i}" pos="{(i % 5) * 0.2} {(i // 5) * 0.2} 0.1">
            <geom type="box" size="0.05 0.05 0.05" rgba="{0.2 + 0.1*i} 0.3 0.8 1"/>
        </body>""" for i in range(self.num_bodies)])}
    </worldbody>
</mujoco>'''
        
        # Write to a temporary file
        benchmark_dir = ASSETS_DIR / "scenes" / "benchmark"
        benchmark_dir.mkdir(parents=True, exist_ok=True)
        self.xml_path = str(benchmark_dir / "benchmark_scene.xml")
        
        with open(self.xml_path, 'w') as f:
            f.write(xml_content)
        
        # Load the model
        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
    
    def step(self):
        """Advance the simulation by one step."""
        mujoco.mj_step(self.model, self.data)
    
    def randomize_poses(self):
        """Randomize body positions for realistic test data."""
        for i in range(self.num_bodies):
            body_id = self.model.body(f"body_{i}").id
            # Apply small random perturbation
            self.data.xpos[body_id] += np.random.uniform(-0.01, 0.01, 3)


def measure_streaming_frequency(streamer: VisionProStreamer, 
                                 model, data,
                                 duration: float = 5.0,
                                 target_hz: float = 120.0) -> dict:
    """
    Measure the actual streaming frequency achieved.
    
    Args:
        streamer: VisionProStreamer instance
        model: MuJoCo model
        data: MuJoCo data
        duration: Test duration in seconds
        target_hz: Target update rate in Hz
        
    Returns:
        Dictionary with frequency statistics
    """
    update_count = 0
    update_times = []
    last_time = None
    
    target_dt = 1.0 / target_hz
    start_time = time.perf_counter()
    
    while time.perf_counter() - start_time < duration:
        mujoco.mj_step(model, data)
        streamer.update_sim()
        update_count += 1
        
        now = time.perf_counter()
        if last_time is not None:
            update_times.append(now - last_time)
        last_time = now
        
        # Sleep to maintain target rate
        elapsed = time.perf_counter() - start_time
        expected_updates = int(elapsed * target_hz)
        if update_count > expected_updates:
            sleep_time = (update_count / target_hz) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    total_time = time.perf_counter() - start_time
    actual_hz = update_count / total_time
    
    return {
        "target_hz": target_hz,
        "actual_hz": actual_hz,
        "update_count": update_count,
        "duration_s": total_time,
        "mean_dt_ms": float(np.mean(update_times)) * 1000 if update_times else 0,
        "std_dt_ms": float(np.std(update_times)) * 1000 if update_times else 0,
    }


def run_frequency_sweep(args):
    """Run a sweep over different update frequencies."""
    print(f"🔌 Connecting to Vision Pro at {args.ip}...")
    streamer = VisionProStreamer(ip=args.ip, record=False, benchmark_quiet=True)
    
    # Use existing scene or create benchmark scene
    if args.scene:
        xml_path = args.scene
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
    else:
        # Use Franka scene for realistic benchmark
        xml_path = str(ASSETS_DIR / "scenes" / "franka_emika_panda" / "scene_blockpush.xml")
        if os.path.exists(xml_path):
            model = mujoco.MjModel.from_xml_path(xml_path)
            data = mujoco.MjData(model)
        else:
            # Fall back to benchmark scene
            sim = BenchmarkSimulation(num_bodies=args.num_bodies)
            model = sim.model
            data = sim.data
            xml_path = sim.xml_path
    
    print(f"📦 Using scene: {xml_path}")
    print(f"   Bodies: {model.nbody}")
    
    # Configure simulation streaming
    attach_to = [0.2, 1.0, 0.7, -90]
    streamer.configure_sim(
        xml_path=xml_path,
        model=model,
        data=data,
        relative_to=attach_to,
        grpc_port=args.port,
    )
    
    # Start WebRTC
    print(f"🌐 Starting WebRTC connection...")
    streamer.start_webrtc()
    
    # Wait for sim channel to be ready
    print(f"⏳ Waiting for sim-poses channel...")
    if not streamer.wait_for_sim_channel(timeout=30.0):
        print("❌ Failed to establish sim-poses channel")
        return
    
    print(f"✅ Sim-poses channel ready!")
    
    # Define test frequencies
    frequencies = [30, 60, 90, 120, 180, 240]
    if args.frequencies:
        frequencies = [int(f) for f in args.frequencies.split(',')]
    
    results = {
        "timestamp": datetime.now().isoformat(),
        "test_config": {
            "ip": args.ip,
            "scene": xml_path,
            "num_bodies": model.nbody,
            "duration_per_test": args.duration,
            "warmup": args.warmup,
        },
        "frequencies": {}
    }
    
    print(f"\n🚀 Starting frequency sweep: {frequencies} Hz")
    print(f"   Duration per test: {args.duration}s")
    print(f"   Warmup: {args.warmup}s")
    
    for target_hz in frequencies:
        print(f"\n--- Testing {target_hz} Hz ---")
        
        # Reset simulation
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        
        # Warmup
        if args.warmup > 0:
            print(f"🔥 Warming up ({args.warmup}s)...")
            warmup_start = time.perf_counter()
            while time.perf_counter() - warmup_start < args.warmup:
                mujoco.mj_step(model, data)
                streamer.update_sim()
                time.sleep(1.0 / target_hz)
        
        # Run test
        print(f"📊 Measuring...")
        freq_result = measure_streaming_frequency(
            streamer, model, data,
            duration=args.duration,
            target_hz=target_hz
        )
        
        results["frequencies"][str(target_hz)] = freq_result
        
        print(f"   Target: {target_hz} Hz")
        print(f"   Actual: {freq_result['actual_hz']:.1f} Hz")
        print(f"   Mean dt: {freq_result['mean_dt_ms']:.2f} ms")
        print(f"   Std dt:  {freq_result['std_dt_ms']:.2f} ms")
    
    # Save results
    os.makedirs("benchmarks", exist_ok=True)
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"benchmarks/sim_freq_{timestamp_str}.json"
    
    with open(filename, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\n✅ Frequency sweep complete. Results saved to: {filename}")
    
    # Print summary
    print("\n📊 Summary:")
    print(f"{'Target Hz':>12} {'Actual Hz':>12} {'Efficiency':>12} {'Mean dt':>12}")
    print("-" * 52)
    for target_hz, data in results["frequencies"].items():
        efficiency = data['actual_hz'] / float(target_hz) * 100
        print(f"{target_hz:>12} {data['actual_hz']:>12.1f} {efficiency:>11.1f}% {data['mean_dt_ms']:>11.2f}ms")


def run_body_count_sweep(args):
    """Run a sweep over different body counts to measure scalability."""
    print(f"🔌 Connecting to Vision Pro at {args.ip}...")
    streamer = VisionProStreamer(ip=args.ip, record=False, benchmark_quiet=True)
    
    body_counts = [5, 10, 20, 50, 100, 200]
    if args.body_counts:
        body_counts = [int(b) for b in args.body_counts.split(',')]
    
    results = {
        "timestamp": datetime.now().isoformat(),
        "test_config": {
            "ip": args.ip,
            "target_hz": args.hz,
            "duration_per_test": args.duration,
            "warmup": args.warmup,
        },
        "body_counts": {}
    }
    
    print(f"\n🚀 Starting body count sweep: {body_counts} bodies")
    print(f"   Target rate: {args.hz} Hz")
    print(f"   Duration per test: {args.duration}s")
    
    first_run = True
    
    for num_bodies in body_counts:
        print(f"\n--- Testing {num_bodies} bodies ---")
        
        # Create benchmark simulation with specified body count
        sim = BenchmarkSimulation(num_bodies=num_bodies)
        
        # Configure simulation streaming
        attach_to = [0.2, 1.0, 0.7, -90]
        streamer.configure_sim(
            xml_path=sim.xml_path,
            model=sim.model,
            data=sim.data,
            relative_to=attach_to,
            grpc_port=args.port,
            force_reload=True,  # Force reload for each body count
        )
        
        if first_run:
            # Start WebRTC only on first run
            print(f"🌐 Starting WebRTC connection...")
            streamer.start_webrtc()
            
            # Wait for sim channel to be ready
            print(f"⏳ Waiting for sim-poses channel...")
            if not streamer.wait_for_sim_channel(timeout=30.0):
                print("❌ Failed to establish sim-poses channel")
                return
            print(f"✅ Sim-poses channel ready!")
            first_run = False
        else:
            # Give time for new scene to load
            time.sleep(2.0)
        
        # Warmup
        if args.warmup > 0:
            print(f"🔥 Warming up ({args.warmup}s)...")
            warmup_start = time.perf_counter()
            while time.perf_counter() - warmup_start < args.warmup:
                mujoco.mj_step(sim.model, sim.data)
                streamer.update_sim()
                time.sleep(1.0 / args.hz)
        
        # Run test
        print(f"📊 Measuring...")
        freq_result = measure_streaming_frequency(
            streamer, sim.model, sim.data,
            duration=args.duration,
            target_hz=args.hz
        )
        
        # Estimate message size
        # Each body: 7 floats (x,y,z,qx,qy,qz,qw) * 4 bytes = 28 bytes
        # Plus JSON overhead ~20 bytes per body
        estimated_msg_bytes = num_bodies * 48 + 50  # Rough estimate
        bandwidth_kbps = estimated_msg_bytes * freq_result['actual_hz'] * 8 / 1000
        
        results["body_counts"][str(num_bodies)] = {
            **freq_result,
            "estimated_msg_bytes": estimated_msg_bytes,
            "estimated_bandwidth_kbps": bandwidth_kbps,
        }
        
        print(f"   Bodies: {num_bodies}")
        print(f"   Actual Hz: {freq_result['actual_hz']:.1f}")
        print(f"   Est. bandwidth: {bandwidth_kbps:.1f} kbps")
    
    # Save results
    os.makedirs("benchmarks", exist_ok=True)
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"benchmarks/sim_bodies_{timestamp_str}.json"
    
    with open(filename, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\n✅ Body count sweep complete. Results saved to: {filename}")
    
    # Print summary
    print("\n📊 Summary:")
    print(f"{'Bodies':>10} {'Actual Hz':>12} {'Est. BW':>15} {'Efficiency':>12}")
    print("-" * 52)
    for body_count, data in results["body_counts"].items():
        efficiency = data['actual_hz'] / args.hz * 100
        print(f"{body_count:>10} {data['actual_hz']:>12.1f} {data['estimated_bandwidth_kbps']:>13.1f}kb/s {efficiency:>11.1f}%")


def run_continuous_test(args):
    """Run a continuous streaming test for monitoring."""
    print(f"🔌 Connecting to Vision Pro at {args.ip}...")
    streamer = VisionProStreamer(ip=args.ip, record=False, benchmark_quiet=True, verbose=args.verbose)
    
    # Use existing scene or create benchmark scene
    if args.scene:
        xml_path = args.scene
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
    else:
        # Use Franka scene
        xml_path = str(ASSETS_DIR / "scenes" / "franka_emika_panda" / "scene_blockpush.xml")
        if os.path.exists(xml_path):
            model = mujoco.MjModel.from_xml_path(xml_path)
            data = mujoco.MjData(model)
        else:
            sim = BenchmarkSimulation(num_bodies=args.num_bodies)
            model = sim.model
            data = sim.data
            xml_path = sim.xml_path
    
    print(f"📦 Using scene: {xml_path}")
    print(f"   Bodies: {model.nbody}")
    
    # Configure simulation streaming
    attach_to = [0.2, 1.0, 0.7, -90]
    streamer.configure_sim(
        xml_path=xml_path,
        model=model,
        data=data,
        relative_to=attach_to,
        grpc_port=args.port,
    )
    
    # Start WebRTC
    print(f"🌐 Starting WebRTC connection...")
    streamer.start_webrtc()
    
    # Wait for sim channel to be ready
    print(f"⏳ Waiting for sim-poses channel...")
    if not streamer.wait_for_sim_channel(timeout=30.0):
        print("❌ Failed to establish sim-poses channel")
        return
    
    print(f"✅ Sim-poses channel ready!")
    print(f"\n🎮 Streaming at {args.hz} Hz. Press Ctrl+C to stop.\n")
    
    target_dt = 1.0 / args.hz
    update_count = 0
    update_times = []
    start_time = time.perf_counter()
    last_report = start_time
    
    try:
        while True:
            loop_start = time.perf_counter()
            
            mujoco.mj_step(model, data)
            streamer.update_sim()
            update_count += 1
            
            now = time.perf_counter()
            update_times.append(now - loop_start)
            
            # Keep only last 1000 samples
            if len(update_times) > 1000:
                update_times.pop(0)
            
            # Report every 5 seconds
            if now - last_report >= 5.0:
                elapsed = now - start_time
                actual_hz = update_count / elapsed
                recent_hz = len(update_times) / sum(update_times) if update_times else 0
                mean_dt = np.mean(update_times) * 1000
                std_dt = np.std(update_times) * 1000
                
                print(f"📊 Updates: {update_count:,} | "
                      f"Avg Hz: {actual_hz:.1f} | "
                      f"Recent Hz: {recent_hz:.1f} | "
                      f"dt: {mean_dt:.2f}±{std_dt:.2f}ms")
                last_report = now
            
            # Sleep to maintain target rate
            elapsed_in_loop = time.perf_counter() - loop_start
            sleep_time = target_dt - elapsed_in_loop
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        total_time = time.perf_counter() - start_time
        print(f"\n\n🛑 Stopped after {total_time:.1f}s")
        print(f"   Total updates: {update_count:,}")
        print(f"   Average Hz: {update_count / total_time:.1f}")


def run_roundtrip_test(args):
    """
    Run a round-trip latency test.
    
    This test enables benchmark mode in the streamer which includes timestamps
    in each pose update. Swift echoes these back, allowing measurement of
    true round-trip latency.
    """
    print(f"🔌 Connecting to Vision Pro at {args.ip}...")
    streamer = VisionProStreamer(ip=args.ip, record=False, benchmark_quiet=True, verbose=args.verbose)
    
    # Use existing scene or create benchmark scene
    if args.scene:
        xml_path = args.scene
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
    else:
        # Use Franka scene
        xml_path = str(ASSETS_DIR / "scenes" / "franka_emika_panda" / "scene_blockpush.xml")
        if os.path.exists(xml_path):
            model = mujoco.MjModel.from_xml_path(xml_path)
            data = mujoco.MjData(model)
        else:
            sim = BenchmarkSimulation(num_bodies=args.num_bodies)
            model = sim.model
            data = sim.data
            xml_path = sim.xml_path
    
    print(f"📦 Using scene: {xml_path}")
    print(f"   Bodies: {model.nbody}")
    
    # Configure simulation streaming
    attach_to = [0.2, 1.0, 0.7, -90]
    streamer.configure_sim(
        xml_path=xml_path,
        model=model,
        data=data,
        relative_to=attach_to,
        grpc_port=args.port,
    )
    
    # Start WebRTC
    print(f"🌐 Starting WebRTC connection...")
    streamer.start_webrtc()
    
    # Wait for sim channel to be ready
    print(f"⏳ Waiting for sim-poses channel...")
    if not streamer.wait_for_sim_channel(timeout=30.0):
        print("❌ Failed to establish sim-poses channel")
        return
    
    print(f"✅ Sim-poses channel ready!")
    
    # Enable benchmark mode
    streamer.enable_sim_benchmark(True)
    streamer.reset_benchmark_epoch()
    
    target_dt = 1.0 / args.hz
    
    # Warmup
    if args.warmup > 0:
        print(f"🔥 Warming up ({args.warmup}s)...")
        warmup_start = time.perf_counter()
        while time.perf_counter() - warmup_start < args.warmup:
            mujoco.mj_step(model, data)
            streamer.update_sim()
            time.sleep(target_dt)
        # Drain any warmup events
        streamer.reset_benchmark_epoch()
    
    print(f"\n🧪 Measuring round-trip latency at {args.hz} Hz...")
    print(f"   Trials: {args.trials}")
    
    latencies = []
    current_seq = 0
    collected = 0
    skipped = 0
    
    # Create progress bar
    pbar = tqdm(total=args.trials, desc="RTT", unit="samples",
                bar_format='{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed}<{remaining}] RTT: {postfix[0]:.1f}ms ± {postfix[1]:.1f}ms',
                postfix=[0.0, 0.0])
    
    try:
        while collected < args.trials:
            # Step simulation and update
            loop_start = time.perf_counter()
            mujoco.mj_step(model, data)
            streamer.update_sim()
            
            # Wait for benchmark echo (use source="sim" for sim-poses channel)
            event = streamer.wait_for_benchmark_event(current_seq, timeout=0.5, source="sim")
            
            if event and event.get("round_trip_ms") is not None:
                latencies.append(event["round_trip_ms"])
                collected += 1
                
                # Update progress bar
                if len(latencies) > 0:
                    current_mean = np.mean(latencies)
                    current_std = np.std(latencies)
                    pbar.postfix[0] = current_mean
                    pbar.postfix[1] = current_std
                pbar.update(1)
            else:
                skipped += 1
                if skipped > 10 and collected == 0:
                    pbar.write("⚠️  No benchmark echoes received yet. Is the Swift app updated?")
            
            current_seq += 1
            
            # Maintain target rate
            elapsed = time.perf_counter() - loop_start
            sleep_time = target_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        pbar.close()
        print("\n⚠️  Test interrupted")
    finally:
        pbar.close()
        streamer.enable_sim_benchmark(False)
    
    # Calculate and display results
    if latencies:
        mean_latency = np.mean(latencies)
        std_latency = np.std(latencies)
        min_lat = np.min(latencies)
        max_lat = np.max(latencies)
        p50 = np.percentile(latencies, 50)
        p95 = np.percentile(latencies, 95)
        p99 = np.percentile(latencies, 99)
        
        print(f"\n📊 Round-Trip Latency Results:")
        print(f"   Samples:  {len(latencies)}")
        print(f"   Mean:     {mean_latency:.2f} ms")
        print(f"   Std:      {std_latency:.2f} ms")
        print(f"   Min:      {min_lat:.2f} ms")
        print(f"   Max:      {max_lat:.2f} ms")
        print(f"   P50:      {p50:.2f} ms")
        print(f"   P95:      {p95:.2f} ms")
        print(f"   P99:      {p99:.2f} ms")
        print(f"   Skipped:  {skipped}")
        
        # Save results
        results = {
            "timestamp": datetime.now().isoformat(),
            "test_config": {
                "ip": args.ip,
                "hz": args.hz,
                "scene": xml_path,
                "num_bodies": model.nbody,
                "warmup": args.warmup,
                "trials": args.trials,
            },
            "results": {
                "mean_ms": float(mean_latency),
                "std_ms": float(std_latency),
                "min_ms": float(min_lat),
                "max_ms": float(max_lat),
                "p50_ms": float(p50),
                "p95_ms": float(p95),
                "p99_ms": float(p99),
                "samples_collected": len(latencies),
                "samples_skipped": skipped,
                "raw_latencies": [float(x) for x in latencies]
            }
        }
        
        os.makedirs("benchmarks", exist_ok=True)
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"benchmarks/sim_rtt_{timestamp_str}.json"
        
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"\n✅ Results saved to: {filename}")
    else:
        print("\n❌ No latency data collected")
        print("   Make sure the Vision Pro app is updated to echo benchmark data.")


def main():
    parser = argparse.ArgumentParser(
        description="Simulation Streaming Latency and Performance Tester",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Continuous streaming test at 120 Hz
    python -m avp_stream.sim_latency_test --ip 192.168.86.21
    
    # Round-trip latency test (requires updated Swift app)
    python -m avp_stream.sim_latency_test --ip 192.168.86.21 --roundtrip
    
    # Sweep over different update frequencies
    python -m avp_stream.sim_latency_test --ip 192.168.86.21 --sweep
    
    # Sweep over different body counts
    python -m avp_stream.sim_latency_test --ip 192.168.86.21 --sweep-bodies
    
    # Custom frequencies
    python -m avp_stream.sim_latency_test --ip 192.168.86.21 --sweep --frequencies 30,60,120
"""
    )
    
    parser.add_argument(
        "--ip",
        default="192.168.86.21",
        help="Vision Pro IP address",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=50051,
        help="MuJoCo gRPC port for USDZ transfer (default: 50051)",
    )
    parser.add_argument(
        "--hz",
        type=int,
        default=120,
        help="Target update rate in Hz (default: 120)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Test duration in seconds for sweep tests (default: 10)",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=2.0,
        help="Warmup duration in seconds (default: 2)",
    )
    parser.add_argument(
        "--trials",
        type=int,
        default=1000,
        help="Number of trials for round-trip test (default: 1000)",
    )
    parser.add_argument(
        "--roundtrip",
        action="store_true",
        help="Run round-trip latency test (measures actual RTT)",
    )
    parser.add_argument(
        "--sweep",
        action="store_true",
        help="Sweep over different update frequencies",
    )
    parser.add_argument(
        "--frequencies",
        type=str,
        help="Comma-separated list of frequencies for sweep (e.g., '30,60,120')",
    )
    parser.add_argument(
        "--sweep-bodies",
        action="store_true",
        help="Sweep over different body counts",
    )
    parser.add_argument(
        "--body-counts",
        type=str,
        help="Comma-separated list of body counts for sweep (e.g., '10,50,100')",
    )
    parser.add_argument(
        "--num-bodies",
        type=int,
        default=20,
        help="Number of bodies in benchmark scene (default: 20)",
    )
    parser.add_argument(
        "--scene",
        type=str,
        help="Path to custom MuJoCo scene XML",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose output",
    )
    
    args = parser.parse_args()
    
    if args.roundtrip:
        run_roundtrip_test(args)
    elif args.sweep:
        run_frequency_sweep(args)
    elif args.sweep_bodies:
        run_body_count_sweep(args)
    else:
        run_continuous_test(args)


if __name__ == "__main__":
    main()
