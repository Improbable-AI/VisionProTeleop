"""Latency benchmark between Python streamer and VisionOS client.

This script streams the Big Buck Bunny video with embedded markers that the VisionOS
application detects. When a marker is seen, the VisionOS side echoes a benchmark
signal back over the gRPC stream, allowing this script to compute the round-trip
latency and jitter between the Python and Swift processes.
"""

from __future__ import annotations

import argparse
import array
import json
import subprocess
import sys
import tempfile
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from avp_stream.streamer import VisionProStreamer


DEFAULT_SWEEP_RESOLUTIONS: List[Tuple[str, str]] = [
    ("426x240", "240p"),
    ("640x360", "360p"),
    ("854x480", "480p"),
    ("1280x720", "720p"),
    ("1920x1080", "1080p"),
    ("2560x1440", "1440p"),
    ("3840x2160", "2160p"),
]

DEFAULT_SWEEP_BACKENDS: List[str] = ["grpc", "webrtc"]

VIDEO_URL = "http://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4"


def download_video(url: str, output_path: Path) -> Path:
    """Download video file if it doesn't exist."""
    if output_path.exists():
        print(f"✓ Using cached video: {output_path}")
        return output_path
    
    print(f"📥 Downloading video from {url}...")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        urllib.request.urlretrieve(url, output_path)
        print(f"✓ Video downloaded to {output_path}")
        return output_path
    except Exception as e:
        print(f"❌ Failed to download video: {e}")
        raise


@dataclass
class BenchmarkPayload:
    sequence_id: int
    sent_timestamp_ms: int


class BenchmarkFrameGenerator:
    """Generate frames that encode benchmark payloads as bright/dark squares overlaid on video."""

    MAGIC = 0x5A
    ROWS = 8
    COLS = 9

    def __init__(self, width: int, height: int, video_path: Path, block_size: int = 16, margin: int = 8) -> None:
        self.width = width
        self.height = height
        self.block_size = block_size
        self.margin = margin
        self._lock = Lock()
        self._payload: Optional[BenchmarkPayload] = None
        
        # Open video file
        self.video_cap = cv2.VideoCapture(str(video_path))
        if not self.video_cap.isOpened():
            raise RuntimeError(f"Failed to open video file: {video_path}")
        
        # Get video properties
        self.video_fps = self.video_cap.get(cv2.CAP_PROP_FPS)
        self.video_frame_count = int(self.video_cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.current_frame_idx = 0
        
        print(f"✓ Video loaded: {self.video_frame_count} frames at {self.video_fps:.2f} fps")

        required_width = margin * 2 + self.COLS * block_size
        required_height = margin * 2 + self.ROWS * block_size
        if required_width > width or required_height > height:
            raise ValueError(
                "Benchmark bit pattern does not fit in the configured frame. "
                "Lower --block-size or increase --resolution."
            )

    def activate_payload(self, payload: BenchmarkPayload) -> None:
        with self._lock:
            self._payload = payload

    def clear_payload(self) -> None:
        with self._lock:
            self._payload = None

    def __call__(self, _frame: np.ndarray) -> np.ndarray:
        # Read next video frame
        ret, video_frame = self.video_cap.read()
        if not ret:
            # Loop back to start
            self.video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, video_frame = self.video_cap.read()
            if not ret:
                # Fallback to blank frame if video fails
                video_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Resize video frame to match target resolution
        if video_frame.shape[:2] != (self.height, self.width):
            video_frame = cv2.resize(video_frame, (self.width, self.height))
        
        with self._lock:
            payload = self._payload

        if payload is None:
            return video_frame

        return self._render_payload(payload, video_frame)

    def _render_payload(self, payload: BenchmarkPayload, base_frame: np.ndarray) -> np.ndarray:
        # Use video frame as base
        frame = base_frame.copy()
        bits = self._encode_payload(payload)

        for idx, bit in enumerate(bits):
            row = idx // self.COLS
            col = idx % self.COLS
            x0 = self.margin + col * self.block_size
            y0 = self.margin + row * self.block_size
            x1 = min(x0 + self.block_size, self.width)
            y1 = min(y0 + self.block_size, self.height)
            if bit:
                frame[y0:y1, x0:x1] = (255, 255, 255)
            else:
                frame[y0:y1, x0:x1] = (0, 0, 0)

        # Draw a border so it is easy to verify visually in debug builds.
        border_thickness = 4
        frame[self.margin:self.margin + border_thickness, self.margin:self.margin + self.COLS * self.block_size] = (0, 255, 255)
        frame[self.margin + self.ROWS * self.block_size - border_thickness:self.margin + self.ROWS * self.block_size, self.margin:self.margin + self.COLS * self.block_size] = (0, 255, 255)
        frame[self.margin:self.margin + self.ROWS * self.block_size, self.margin:self.margin + border_thickness] = (0, 255, 255)
        frame[self.margin:self.margin + self.ROWS * self.block_size, self.margin + self.COLS * self.block_size - border_thickness:self.margin + self.COLS * self.block_size] = (0, 255, 255)

        return frame

    def _encode_payload(self, payload: BenchmarkPayload) -> List[int]:
        bits: List[int] = []
        bits.extend(self._value_to_bits(self.MAGIC, 8))
        bits.extend(self._value_to_bits(payload.sequence_id & 0xFFFFFFFF, 32))
        bits.extend(self._value_to_bits(payload.sent_timestamp_ms & 0xFFFFFFFF, 32))
        return bits

    @staticmethod
    def _value_to_bits(value: int, bit_length: int) -> List[int]:
        return [(value >> (bit_length - 1 - i)) & 1 for i in range(bit_length)]


def parse_resolution(resolution: str) -> tuple[int, int]:
    try:
        width_str, height_str = resolution.lower().split("x", 1)
        return int(width_str), int(height_str)
    except ValueError as exc:  # pragma: no cover - defensive programming
        raise argparse.ArgumentTypeError(f"Invalid resolution '{resolution}'. Use WIDTHxHEIGHT format.") from exc


def create_audio_streamer(video_path: Path, stereo: bool = False):
    """Create an audio callback that streams audio from the video file."""
    try:
        from pydub import AudioSegment
    except ImportError:
        print("⚠️ pydub not installed. Audio streaming disabled.")
        print("   Install with: pip install pydub")
        return None
    
    try:
        print(f"🎵 Loading audio from video: {video_path}")
        
        # Extract audio from video file
        audio = AudioSegment.from_file(str(video_path))
        
        # Convert to 48kHz with specified channel count
        channels = 2 if stereo else 1
        audio = audio.set_frame_rate(48000).set_channels(channels)
        
        channel_mode = "stereo" if stereo else "mono"
        print(f"✓ Audio loaded ({channel_mode}):")
        print(f"  - Duration: {len(audio) / 1000:.2f} seconds")
        print(f"  - Sample rate: {audio.frame_rate} Hz")
        print(f"  - Channels: {audio.channels}")
        
        # Get raw audio data as int16 samples
        audio_samples = np.array(audio.get_array_of_samples(), dtype=np.int16)
        total_samples = len(audio_samples)
        
        # Track position in audio file
        current_position = [0]  # Use list for mutable closure
        
        def generate_audio(audio_frame):
            # Number of samples per frame PER CHANNEL
            num_samples_per_channel = audio_frame.samples
            
            # Total samples needed
            num_samples_needed = num_samples_per_channel * channels
            
            # Extract samples for this frame
            end_position = current_position[0] + num_samples_needed
            
            # Handle looping
            if end_position >= total_samples:
                samples_before_end = total_samples - current_position[0]
                samples_after_wrap = num_samples_needed - samples_before_end
                
                frame_data = np.concatenate([
                    audio_samples[current_position[0]:],
                    audio_samples[:samples_after_wrap]
                ])
                
                current_position[0] = samples_after_wrap
            else:
                frame_data = audio_samples[current_position[0]:end_position]
                current_position[0] = end_position
            
            # Convert numpy array to bytes
            audio_array = array.array('h', frame_data.tolist())
            audio_bytes = audio_array.tobytes()
            
            # Update the audio frame's plane data
            for plane in audio_frame.planes:
                plane.update(audio_bytes)
            
            return audio_frame
        
        return generate_audio
    
    except Exception as e:
        print(f"⚠️ Failed to load audio: {e}")
        print("   Continuing without audio...")
        return None


def compute_summary(latencies: List[float]) -> Dict[str, float]:
    if not latencies:
        return {}

    values = np.array(latencies, dtype=np.float64)
    count = values.size
    summary: Dict[str, float] = {
        "samples": float(count),
        "mean_ms": float(values.mean()),
        "median_ms": float(np.median(values)),
        "min_ms": float(values.min()),
        "max_ms": float(values.max()),
        "p95_ms": float(np.percentile(values, 95)),
        "stddev_ms": float(values.std(ddof=1)) if count > 1 else 0.0,
    }
    summary["jitter_ms"] = summary["stddev_ms"]
    return summary


def run_benchmark(args: argparse.Namespace) -> Tuple[Dict[str, float], List[Dict[str, Optional[float]]]]:
    width, height = parse_resolution(args.resolution)

    # Download video if needed
    video_cache_dir = Path.home() / ".cache" / "avp_stream"
    video_path = video_cache_dir / "BigBuckBunny.mp4"
    download_video(VIDEO_URL, video_path)

    streamer = VisionProStreamer(ip=args.ip, record=False, ht_backend=args.ht_backend)
    frame_generator = BenchmarkFrameGenerator(
        width=width, 
        height=height, 
        video_path=video_path,
        block_size=args.block_size
    )
    streamer.register_frame_callback(frame_generator)
    
    # Set up audio streaming
    audio_callback = create_audio_streamer(video_path, stereo=args.stereo_audio)
    if audio_callback:
        streamer.register_audio_callback(audio_callback)

    print("⏳ Waiting for VisionOS client to establish WebRTC connection...")
    streamer.start_video_streaming(
        device=None, 
        size=args.resolution, 
        fps=args.fps,
        stereo_audio=args.stereo_audio,
        audio_device=None if audio_callback else None,
    )
    time.sleep(args.warmup)

    epoch = streamer.reset_benchmark_epoch()
    print(f"🚀 Benchmark epoch established at t0={epoch:.6f}s")

    if args.initial_delay > 0:
        print(f"⏳ Waiting an additional {args.initial_delay:.1f}s before first sample...")
        time.sleep(args.initial_delay)

    results: List[Dict[str, Optional[float]]] = []

    try:
        for sample_idx in range(1, args.samples + 1):
            send_time = time.perf_counter()
            sent_ms = int((send_time - epoch) * 1000)
            payload = BenchmarkPayload(sequence_id=sample_idx, sent_timestamp_ms=sent_ms)

            frame_generator.activate_payload(payload)
            event = streamer.wait_for_benchmark_event(payload.sequence_id, timeout=args.timeout)
            frame_generator.clear_payload()

            if event is None:
                print(f"⚠️ Sample {sample_idx} timed out after {args.timeout:.1f}s")
                results.append({
                    "sequence_id": sample_idx,
                    "status": "timeout",
                })
            else:
                status = "ok" if event.get("round_trip_ms") is not None else "invalid"
                recorded_sent = event.get("sent_timestamp_ms")
                if recorded_sent is not None and abs(int(recorded_sent) - sent_ms) > 1:
                    status = "mismatch"
                    print(
                        f"⚠️ Sample {sample_idx:03d}: reported sent timestamp {recorded_sent} ms "
                        f"differs from expected {sent_ms} ms"
                    )

                event.update({
                    "sequence_id": sample_idx,
                    "status": status,
                })

                if status == "ok":
                    latency = event["round_trip_ms"]
                    print(f"✅ Sample {sample_idx:03d}: {latency:.2f} ms round-trip")
                else:
                    print(f"⚠️ Sample {sample_idx:03d}: event missing timing metadata")

                results.append(event)

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("⏹️ Benchmark interrupted by user")

    successful_latencies = [
        float(entry["round_trip_ms"]) for entry in results if entry.get("status") == "ok"
    ]
    summary = compute_summary(successful_latencies)

    if summary:
        print("\n📊 Latency Summary")
        print(f"  Samples: {int(summary['samples'])}")
        print(f"  Mean:    {summary['mean_ms']:.2f} ms")
        print(f"  Median:  {summary['median_ms']:.2f} ms")
        print(f"  P95:     {summary['p95_ms']:.2f} ms")
        print(f"  Min/Max: {summary['min_ms']:.2f} / {summary['max_ms']:.2f} ms")
        print(f"  Jitter:  {summary['jitter_ms']:.2f} ms (stddev)")
    else:
        print("❌ No successful samples recorded; check connectivity and try again.")

    if args.output:
        payload = {
            "config": {
                "ip": args.ip,
                "samples": args.samples,
                "timeout_s": args.timeout,
                "interval_s": args.interval,
                "resolution": args.resolution,
                "ht_backend": args.ht_backend,
                "fps": args.fps,
            },
            "summary": summary,
            "results": results,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(payload, indent=2))
        print(f"💾 Saved detailed results to {args.output}")

    return summary, results


def build_sweep_resolution_list(args: argparse.Namespace) -> List[Tuple[str, str]]:
    if args.sweep_resolutions:
        resolutions: List[Tuple[str, str]] = []
        for entry in args.sweep_resolutions:
            value = entry.strip().lower()
            if not value:
                continue
            resolutions.append((value, entry.strip()))
        return resolutions
    return DEFAULT_SWEEP_RESOLUTIONS


def build_sweep_backend_list(args: argparse.Namespace) -> List[str]:
    if args.sweep_backends:
        backends: List[str] = []
        for entry in args.sweep_backends:
            backend = entry.strip().lower()
            if backend:
                backends.append(backend)
        if backends:
            return backends
    return DEFAULT_SWEEP_BACKENDS


def run_sweep(args: argparse.Namespace) -> Tuple[List[Dict[str, object]], List[Tuple[str, str]], List[str]]:
    sweep_resolutions = build_sweep_resolution_list(args)
    sweep_backends = build_sweep_backend_list(args)
    sweep_results: List[Dict[str, object]] = []

    if not sweep_resolutions:
        print("⚠️ No resolutions provided for sweep; aborting.")
        return sweep_results, sweep_resolutions, sweep_backends

    if not sweep_backends:
        print("⚠️ No backends provided for sweep; aborting.")
        return sweep_results, sweep_resolutions, sweep_backends

    abort_sweep = False
    SWEEP_REPEATS = 3

    for backend in sweep_backends:
        if abort_sweep:
            break
        print(f"\n##### Backend: {backend.upper()} #####")
        for resolution, label in sweep_resolutions:
            accumulated_means = []
            accumulated_jitters = []
            combined_results = []
            last_config = {}

            for i in range(SWEEP_REPEATS):
                if abort_sweep:
                    break

                print(f"\n===== Running benchmark at {label} ({resolution}) [Run {i+1}/{SWEEP_REPEATS}] =====")

                with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tmp_file:
                    tmp_path = Path(tmp_file.name)

                cmd = [
                    sys.executable,
                    "-m",
                    "avp_stream.utils.latency_benchmark",
                    "--ip",
                    args.ip,
                    "--samples",
                    str(args.samples),
                    "--timeout",
                    str(args.timeout),
                    "--interval",
                    str(args.interval),
                    "--warmup",
                    str(args.warmup),
                    "--resolution",
                    resolution,
                    "--fps",
                    str(args.fps),
                    "--block-size",
                    str(args.block_size),
                    "--ht-backend",
                    backend,
                    "--output",
                    str(tmp_path),
                ]

                if args.initial_delay is not None:
                    cmd.extend(["--initial-delay", f"{args.initial_delay}"])
                
                if args.stereo_audio:
                    cmd.append("--stereo-audio")

                result = subprocess.run(cmd, check=False)

                if result.returncode != 0:
                    print(
                        f"❌ Benchmark failed for backend {backend} at resolution {resolution} "
                        f"(exit code {result.returncode})."
                    )
                    tmp_path.unlink(missing_ok=True)
                    abort_sweep = True
                    break

                try:
                    data = json.loads(tmp_path.read_text())
                except Exception as exc:  # pragma: no cover - defensive
                    print(f"❌ Failed to parse benchmark output for {resolution} / {backend}: {exc}")
                    tmp_path.unlink(missing_ok=True)
                    abort_sweep = True
                    break
                finally:
                    tmp_path.unlink(missing_ok=True)

                summary = data.get("summary") or {}
                if summary:
                    mean = summary.get("mean_ms")
                    jitter = summary.get("jitter_ms")
                    if mean is not None:
                        accumulated_means.append(float(mean))
                        accumulated_jitters.append(float(jitter or 0.0))

                combined_results.extend(data.get("results", []))
                last_config = data.get("config", {})

                # Give VisionOS a brief breather between runs.
                time.sleep(1.0)

            if abort_sweep:
                break

            final_summary = {}
            if accumulated_means:
                avg_mean = sum(accumulated_means) / len(accumulated_means)
                avg_jitter = sum(accumulated_jitters) / len(accumulated_jitters)
                print(f"→ {label} [{backend}]: avg_mean={avg_mean:.2f} ms, avg_jitter={avg_jitter:.2f} ms")
                final_summary = {
                    "mean_ms": avg_mean,
                    "jitter_ms": avg_jitter,
                    "samples": len(combined_results),
                }
            else:
                print(f"→ {label} [{backend}]: no successful samples")

            sweep_results.append(
                {
                    "resolution": resolution,
                    "label": label,
                    "backend": backend,
                    "summary": final_summary,
                    "results": combined_results,
                    "config": last_config,
                }
            )

    return sweep_results, sweep_resolutions, sweep_backends


def plot_sweep_results(
    sweep_results: List[Dict[str, object]],
    sweep_resolutions: List[Tuple[str, str]],
    sweep_backends: List[str],
    output_path: Path,
) -> None:
    if not sweep_results:
        print("⚠️ No sweep data available to plot.")
        return

    try:
        import matplotlib.pyplot as plt
    except ImportError:  # pragma: no cover - plotting optional
        print("⚠️ matplotlib is not installed; skipping plot generation.")
        return

    label_order = [label for _, label in sweep_resolutions]
    label_to_index = {label: idx for idx, label in enumerate(label_order)}
    x_positions = np.arange(len(label_order))

    plt.figure(figsize=(12, 4))

    color_cycle = plt.rcParams.get("axes.prop_cycle", None)
    colors = None
    if color_cycle is not None:
        colors = [c["color"] for c in color_cycle]

    series_data: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    for backend in sweep_backends:
        means = np.full(len(label_order), np.nan, dtype=float)
        jitters = np.zeros(len(label_order), dtype=float)
        series_data[backend] = (means, jitters)

    for entry in sweep_results:
        backend = str(entry.get("backend", ""))
        if backend not in series_data:
            continue
        label = str(entry.get("label", entry.get("resolution", "?")))
        if label not in label_to_index:
            continue
        summary = entry.get("summary") or {}
        mean = summary.get("mean_ms")
        jitter = summary.get("jitter_ms", 0.0)
        if mean is None:
            continue
        idx = label_to_index[label]
        series_data[backend][0][idx] = float(mean)
        series_data[backend][1][idx] = float(jitter)

    any_series = False
    for idx_backend, backend in enumerate(sweep_backends):
        means, jitters = series_data.get(backend, (None, None))
        if means is None:
            continue
        valid_mask = ~np.isnan(means)
        if not np.any(valid_mask):
            continue
        color = None
        if colors is not None:
            color = colors[idx_backend % len(colors)]
        label = f"Hand tracking stream with: {backend}"
        plt.plot(x_positions[valid_mask], means[valid_mask], marker="o", label=label, color=color)
        lower = np.maximum(0.0, means[valid_mask] - jitters[valid_mask])
        upper = means[valid_mask] + jitters[valid_mask]
        plt.fill_between(
            x_positions[valid_mask],
            lower,
            upper,
            color=color,
            alpha=0.2,
        )
        any_series = True

    if not any_series:
        print("⚠️ Sweep summaries are empty; skipping plot.")
        return

    plt.xticks(x_positions, label_order)
    plt.ylabel("Round-trip latency (ms)")
    plt.xlabel("Resolution")
    plt.title("VisionOS round-trip latency sweep")
    plt.grid(axis="y", linestyle="--", alpha=0.3)
    plt.legend(loc="best")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output_path)
    plt.close()
    print(f"🖼️ Saved latency plot to {output_path}")


def write_sweep_output(
    args: argparse.Namespace,
    sweep_results: List[Dict[str, object]],
    sweep_resolutions: List[Tuple[str, str]],
    sweep_backends: List[str],
) -> None:
    if not args.output:
        return

    payload = {
        "config": {
            "ip": args.ip,
            "samples": args.samples,
            "timeout_s": args.timeout,
            "interval_s": args.interval,
            "warmup_s": args.warmup,
            "initial_delay_s": args.initial_delay,
            "fps": args.fps,
            "block_size": args.block_size,
            "sweep": True,
            "resolutions": [res for res, _ in sweep_resolutions],
            "labels": [label for _, label in sweep_resolutions],
            "backends": sweep_backends,
        },
        "results": sweep_results,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2))
    print(f"💾 Saved sweep results to {args.output}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Benchmark VisionOS round-trip latency via WebRTC + gRPC")
    parser.add_argument("--ip", required=True, help="Vision Pro IP address")
    parser.add_argument("--samples", type=int, default=30, help="Number of benchmark iterations")
    parser.add_argument("--timeout", type=float, default=2.0, help="Seconds to wait for each sample before timing out")
    parser.add_argument("--interval", type=float, default=0.3, help="Seconds to wait between samples")
    parser.add_argument("--warmup", type=float, default=3.0, help="Seconds to wait after starting the WebRTC server")
    parser.add_argument("--initial-delay", dest="initial_delay", type=float, help="Extra seconds to wait after warmup before the first sample")
    parser.add_argument("--initial", dest="initial_delay", type=float, help="Alias for --initial-delay")
    parser.add_argument("--resolution", default="640x480", help="Frame size used for the video stream (WIDTHxHEIGHT)")
    parser.add_argument("--ht-backend", default="grpc", choices=["grpc", "webrtc"], help="Hand-tracking backend to use (single run)")
    parser.add_argument("--fps", type=int, default=30, help="Frame rate for the video stream")
    parser.add_argument("--block-size", type=int, default=16, help="Pixel width/height of each encoded bit block")
    parser.add_argument("--stereo-audio", action="store_true", help="Stream audio in stereo (2 channels) instead of mono")
    parser.add_argument("--output", type=Path, help="Optional path to store benchmark results as JSON")
    parser.add_argument("--plot", type=Path, help="Optional path to save latency plot (sweep only)")
    parser.add_argument("--sweep", action="store_true", help="Run the benchmark across multiple common resolutions")
    parser.add_argument("--sweep-resolutions", nargs="+", help="Override the default sweep resolutions (format WIDTHxHEIGHT)")
    parser.add_argument("--sweep-backends", nargs="+", help="Override the default backends tested during a sweep")
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    if args.initial_delay is None:
        args.initial_delay = 5.0
    
    # Default stereo_audio to False if not set
    if not hasattr(args, 'stereo_audio'):
        args.stereo_audio = False

    if args.sweep:
        sweep_results, sweep_resolutions, sweep_backends = run_sweep(args)
        if args.plot:
            plot_sweep_results(sweep_results, sweep_resolutions, sweep_backends, args.plot)
        write_sweep_output(args, sweep_results, sweep_resolutions, sweep_backends)
        return

    if args.plot:
        print("⚠️ --plot is only supported together with --sweep; ignoring plot request.")

    run_benchmark(args)


if __name__ == "__main__":
    main()
