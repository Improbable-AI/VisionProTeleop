"""
Example: Point Cloud Streaming with Draco Compression

This example demonstrates how to stream point cloud data using Google Draco
compression over WebRTC data channels.

Requirements:
    pip install DracoPy

Usage:
    python 09_stream_pointcloud.py --ip <vision_pro_ip>
    
Note: 
    This example demonstrates the Python-side encoding/decoding of point clouds.
    Full integration with VisionOS requires additional Swift-side implementation
    to handle the point cloud data channel.
"""

import numpy as np
import time
import argparse
from typing import Optional

# Import point cloud utilities
try:
    from avp_stream.pointcloud_streamer import (
        PointCloudEncoder,
        PointCloudDecoder,
        create_sample_pointcloud,
        is_draco_available
    )
except ImportError:
    # If running from examples directory
    import sys
    sys.path.insert(0, '..')
    from avp_stream.pointcloud_streamer import (
        PointCloudEncoder,
        PointCloudDecoder,
        create_sample_pointcloud,
        is_draco_available
    )


def generate_animated_sphere(frame: int, num_points: int = 5000) -> np.ndarray:
    """
    Generate a point cloud forming an animated sphere.
    
    The sphere rotates and pulses over time.
    """
    # Generate points on a unit sphere using fibonacci spiral
    indices = np.arange(num_points, dtype=np.float32)
    phi = np.arccos(1 - 2 * (indices + 0.5) / num_points)
    theta = np.pi * (1 + 5**0.5) * indices
    
    # Add time-based animation
    t = frame * 0.05
    radius = 0.5 + 0.1 * np.sin(t * 2 + phi)  # Pulsing radius
    
    # Rotate around Y axis
    rotation_angle = t
    cos_r, sin_r = np.cos(rotation_angle), np.sin(rotation_angle)
    
    # Convert spherical to cartesian
    x = radius * np.sin(phi) * np.cos(theta)
    y = radius * np.sin(phi) * np.sin(theta)
    z = radius * np.cos(phi)
    
    # Apply Y-axis rotation
    x_rot = x * cos_r - z * sin_r
    z_rot = x * sin_r + z * cos_r
    
    points = np.column_stack([x_rot, y, z_rot]).astype(np.float32)
    return points


def generate_depth_pointcloud(width: int = 640, height: int = 480) -> np.ndarray:
    """
    Generate a simulated depth camera point cloud.
    
    Creates a point cloud that might come from a depth sensor like RealSense.
    """
    # Simulate depth values (distance from camera)
    u = np.linspace(-1, 1, width, dtype=np.float32)
    v = np.linspace(-1, 1, height, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)
    
    # Simulate a surface with some noise
    depth = 2.0 + 0.3 * np.sin(uu * 3) * np.cos(vv * 3) + np.random.randn(height, width) * 0.05
    depth = depth.astype(np.float32)
    
    # Convert to 3D points (pinhole camera model)
    fx, fy = 500, 500  # Focal lengths
    cx, cy = width / 2, height / 2  # Principal point
    
    x = (uu * width - cx) / fx * depth
    y = (vv * height - cy) / fy * depth
    z = depth
    
    # Stack and reshape to (N, 3)
    points = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    
    # Remove invalid points (where depth is too far or invalid)
    valid_mask = (z.flatten() > 0.5) & (z.flatten() < 5.0)
    points = points[valid_mask]
    
    return points


def demo_encoding_decoding():
    """Demonstrate basic point cloud encoding and decoding."""
    print("=" * 60)
    print("Point Cloud Encoding/Decoding Demo")
    print("=" * 60)
    
    if not is_draco_available():
        print("ERROR: DracoPy is not installed. Run: pip install DracoPy")
        return
    
    # Create encoder with different settings
    print("\n1. Testing compression levels:")
    points, _ = create_sample_pointcloud(num_points=10000)
    original_size = points.nbytes
    
    for qbits in [10, 14, 16]:
        for clevel in [1, 5, 10]:
            encoder = PointCloudEncoder(quantization_bits=qbits, compression_level=clevel)
            encoded = encoder.encode(points)
            ratio = original_size / len(encoded)
            print(f"   qbits={qbits:2d}, clevel={clevel:2d}: {len(encoded):6d} bytes ({ratio:.2f}x compression)")
    
    # Test round-trip
    print("\n2. Round-trip accuracy test:")
    encoder = PointCloudEncoder(quantization_bits=16, compression_level=1)
    decoder = PointCloudDecoder()
    
    encoded = encoder.encode(points)
    result = decoder.decode(encoded)
    decoded_points = result['points']
    
    # Since Draco may reorder points, we compare sets
    original_set = set(map(tuple, points.round(4)))
    decoded_set = set(map(tuple, decoded_points.round(4)))
    
    print(f"   Original points: {len(points)}")
    print(f"   Decoded points: {len(decoded_points)}")
    print(f"   Points preserved: {len(original_set & decoded_set)}/{len(original_set)}")
    
    # Test with colors
    print("\n3. Point cloud with colors:")
    points, colors = create_sample_pointcloud(num_points=5000, include_colors=True)
    encoder = PointCloudEncoder(quantization_bits=14)
    
    encoded = encoder.encode(points, colors=colors)
    result = decoder.decode(encoded)
    
    print(f"   Points shape: {result['points'].shape}")
    print(f"   Colors shape: {result['colors'].shape if result['colors'] is not None else 'None'}")
    print(f"   Timestamp: {result['timestamp_ns']}")
    
    # Benchmark throughput
    print("\n4. Encoding throughput benchmark:")
    encoder = PointCloudEncoder(quantization_bits=14, compression_level=1)
    
    test_sizes = [1000, 5000, 10000, 50000, 100000]
    for size in test_sizes:
        points, _ = create_sample_pointcloud(num_points=size)
        
        # Warm up
        encoder.encode(points)
        
        # Time encoding
        start = time.perf_counter()
        num_iterations = 10
        for _ in range(num_iterations):
            encoder.encode(points)
        elapsed = time.perf_counter() - start
        
        fps = num_iterations / elapsed
        print(f"   {size:6d} points: {fps:.1f} encodes/sec ({elapsed/num_iterations*1000:.2f} ms/encode)")


def demo_streaming_simulation():
    """Simulate streaming point clouds as would happen in real usage."""
    print("\n" + "=" * 60)
    print("Point Cloud Streaming Simulation")
    print("=" * 60)
    
    if not is_draco_available():
        print("ERROR: DracoPy is not installed.")
        return
    
    encoder = PointCloudEncoder(quantization_bits=14, compression_level=1)
    decoder = PointCloudDecoder()
    
    print("\nSimulating 30 FPS point cloud stream for 5 seconds...")
    print("(Press Ctrl+C to stop early)\n")
    
    target_fps = 30
    frame_time = 1.0 / target_fps
    start_time = time.perf_counter()
    frame_count = 0
    total_bytes = 0
    
    try:
        while frame_count < 150:  # 5 seconds at 30 fps
            frame_start = time.perf_counter()
            
            # Generate animated point cloud
            points = generate_animated_sphere(frame_count, num_points=5000)
            
            # Encode
            encoded = encoder.encode(points)
            total_bytes += len(encoded)
            
            # Simulate network transmission (in real usage, this would go over WebRTC)
            # decoded = decoder.decode(encoded)
            
            frame_count += 1
            
            # Print stats every second
            if frame_count % 30 == 0:
                elapsed = time.perf_counter() - start_time
                actual_fps = frame_count / elapsed
                bandwidth_mbps = (total_bytes * 8) / (elapsed * 1e6)
                print(f"   Frame {frame_count:3d}: {actual_fps:.1f} FPS, "
                      f"{len(encoded):5d} bytes/frame, "
                      f"{bandwidth_mbps:.2f} Mbps bandwidth")
            
            # Sleep to maintain target frame rate
            elapsed_frame = time.perf_counter() - frame_start
            if elapsed_frame < frame_time:
                time.sleep(frame_time - elapsed_frame)
                
    except KeyboardInterrupt:
        print("\n   Stopped by user")
    
    # Final stats
    elapsed = time.perf_counter() - start_time
    print(f"\n   Final: {frame_count} frames in {elapsed:.2f}s = {frame_count/elapsed:.1f} FPS")
    print(f"   Average: {total_bytes/frame_count:.0f} bytes/frame")
    print(f"   Compression ratio: {encoder.get_compression_ratio():.2f}x")


def demo_depth_camera_simulation():
    """Simulate depth camera point cloud streaming."""
    print("\n" + "=" * 60)
    print("Depth Camera Point Cloud Simulation")
    print("=" * 60)
    
    if not is_draco_available():
        print("ERROR: DracoPy is not installed.")
        return
    
    encoder = PointCloudEncoder(quantization_bits=12, compression_level=1)
    
    print("\nSimulating depth camera (640x480) point cloud...")
    
    # Generate depth point cloud
    points = generate_depth_pointcloud(640, 480)
    
    print(f"   Raw points: {len(points)} (from 640x480 depth image)")
    print(f"   Original size: {points.nbytes:,} bytes")
    
    encoded = encoder.encode(points)
    print(f"   Encoded size: {len(encoded):,} bytes")
    print(f"   Compression ratio: {points.nbytes / len(encoded):.2f}x")
    
    # Measure encoding time
    start = time.perf_counter()
    for _ in range(10):
        encoder.encode(points)
    elapsed = (time.perf_counter() - start) / 10
    
    print(f"   Encoding time: {elapsed*1000:.2f} ms")
    print(f"   Max FPS: {1/elapsed:.1f}")


def main():
    parser = argparse.ArgumentParser(
        description="Point Cloud Streaming Demo with Draco Compression"
    )
    parser.add_argument(
        "--ip", 
        type=str, 
        default=None,
        help="Vision Pro IP address (for future WebRTC integration)"
    )
    parser.add_argument(
        "--demo",
        choices=["all", "encode", "stream", "depth"],
        default="all",
        help="Which demo to run (default: all)"
    )
    args = parser.parse_args()
    
    print("\n" + "=" * 60)
    print("Point Cloud Streaming with Google Draco Compression")
    print("=" * 60)
    
    if not is_draco_available():
        print("\nERROR: DracoPy is not installed.")
        print("Please install it with: pip install DracoPy")
        return
    
    print("\n✓ DracoPy is available")
    
    if args.demo in ["all", "encode"]:
        demo_encoding_decoding()
    
    if args.demo in ["all", "stream"]:
        demo_streaming_simulation()
    
    if args.demo in ["all", "depth"]:
        demo_depth_camera_simulation()
    
    print("\n" + "=" * 60)
    print("Demo complete!")
    print("=" * 60)
    
    if args.ip:
        print(f"\nNote: WebRTC streaming to Vision Pro ({args.ip}) requires")
        print("additional VisionOS-side implementation to handle the point cloud")
        print("data channel. See the documentation for integration details.")


if __name__ == "__main__":
    main()
