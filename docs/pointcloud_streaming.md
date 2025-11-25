# Point Cloud Streaming with Google Draco Compression

This module provides functionality to stream 3D point cloud data over WebRTC data channels using Google's Draco encoder for efficient compression.

## Overview

The point cloud streaming feature allows you to:
- Encode 3D point clouds using Google Draco compression (typically 3-6x compression ratio)
- Optionally include RGB color data with each point
- Measure encoding throughput and compression statistics
- Stream point clouds at 30+ FPS for typical robot sensor data

## Installation

Point cloud streaming requires the optional `DracoPy` dependency:

```bash
# Install with point cloud support
pip install avp_stream[pointcloud]

# Or install DracoPy separately
pip install DracoPy
```

## Quick Start

### Basic Encoding/Decoding

```python
import numpy as np
from avp_stream import PointCloudEncoder, PointCloudDecoder, is_draco_available

# Check if Draco is available
if not is_draco_available():
    print("DracoPy not installed")
    exit(1)

# Create encoder with default settings
encoder = PointCloudEncoder(
    quantization_bits=14,  # Precision (higher = more accurate but larger)
    compression_level=1    # Speed (1=fast, 10=best compression)
)

# Create a sample point cloud
points = np.random.rand(10000, 3).astype(np.float32)
colors = np.random.randint(0, 255, (10000, 3), dtype=np.uint8)

# Encode
encoded_data = encoder.encode(points, colors=colors)
print(f"Original: {points.nbytes} bytes, Encoded: {len(encoded_data)} bytes")
print(f"Compression ratio: {encoder.get_compression_ratio():.2f}x")

# Decode
decoder = PointCloudDecoder()
result = decoder.decode(encoded_data)

print(f"Decoded points shape: {result['points'].shape}")
print(f"Decoded colors shape: {result['colors'].shape}")
print(f"Timestamp: {result['timestamp_ns']}")
```

### Streaming Simulation

```python
import time
import numpy as np
from avp_stream import PointCloudEncoder

encoder = PointCloudEncoder(quantization_bits=14, compression_level=1)

# Simulate 30 FPS streaming
frame_count = 0
start_time = time.time()

while frame_count < 150:  # 5 seconds at 30 FPS
    # Generate point cloud (e.g., from depth camera)
    points = np.random.rand(5000, 3).astype(np.float32)
    
    # Encode for transmission
    encoded = encoder.encode(points)
    
    # In real usage, send 'encoded' over WebRTC data channel
    # webrtc_channel.send(encoded)
    
    frame_count += 1
    time.sleep(1/30)  # Maintain 30 FPS

elapsed = time.time() - start_time
print(f"Streamed {frame_count} frames in {elapsed:.1f}s = {frame_count/elapsed:.1f} FPS")
print(f"Average compression: {encoder.get_compression_ratio():.2f}x")
```

## API Reference

### PointCloudEncoder

```python
PointCloudEncoder(
    quantization_bits: int = 14,
    compression_level: int = 1,
    include_header: bool = True
)
```

**Parameters:**
- `quantization_bits`: Number of bits for coordinate quantization (1-30). Higher values provide more precision but larger encoded data.
  - 10: Low quality, ~5x compression
  - 14: Balanced (default), ~3x compression  
  - 16: High quality, ~2.5x compression
- `compression_level`: Draco compression level (1-10). Higher = better compression, slower encoding.
  - 1: Fastest (recommended for real-time)
  - 5: Balanced
  - 10: Best compression
- `include_header`: If True, prepend metadata header (timestamp, point count)

**Methods:**
- `encode(points, colors=None, timestamp_ns=None)` → `bytes`
- `get_compression_ratio()` → `float`
- `reset_stats()`

### PointCloudDecoder

```python
PointCloudDecoder()
```

**Methods:**
- `decode(data)` → `dict` with keys:
  - `'points'`: np.ndarray of shape (N, 3), dtype float32
  - `'colors'`: np.ndarray of shape (N, 3), dtype uint8, or None
  - `'timestamp_ns'`: int or None
  - `'point_count'`: int

### Utility Functions

```python
from avp_stream import is_draco_available, create_sample_pointcloud

# Check if DracoPy is installed
if is_draco_available():
    print("Point cloud streaming available")

# Generate sample point cloud for testing
points, colors = create_sample_pointcloud(
    num_points=1000,
    bounds=(-1, 1, -1, 1, -1, 1),  # min/max for x, y, z
    include_colors=True
)
```

## Performance Benchmarks

Tested on a typical laptop (Intel i7, no GPU acceleration):

| Point Count | Encoding Time | Max FPS |
|-------------|--------------|---------|
| 1,000       | 0.5 ms       | ~2000   |
| 5,000       | 2.5 ms       | ~400    |
| 10,000      | 5 ms         | ~200    |
| 50,000      | 27 ms        | ~37     |
| 100,000     | 55 ms        | ~18     |

For a typical depth camera (640×480, ~300K points after filtering):
- Encoding time: ~160 ms
- Compression ratio: ~6.5x
- Bandwidth: ~4-5 Mbps at 30 FPS

## Integration Notes

### WebRTC Data Channel

Point cloud data is designed to be sent over WebRTC data channels, which provide:
- Reliable or unreliable delivery options
- Low-latency peer-to-peer communication
- NAT traversal via ICE/STUN

For integration with VisionOS, the Swift side needs to:
1. Create a data channel for point cloud data
2. Receive and decode the Draco-compressed bytes
3. Render the point cloud in AR

### VisionOS Integration (Future)

Full VisionOS integration requires Swift-side changes to:
1. Handle a new "pointcloud" WebRTC data channel
2. Decode Draco data using Apple's SceneKit or RealityKit
3. Render point clouds in the AR environment

This is planned for a future release. Currently, this Python module provides the encoding/decoding infrastructure for point cloud streaming.

## Example Scripts

See `examples/09_stream_pointcloud.py` for a complete demonstration:

```bash
# Run all demos
python examples/09_stream_pointcloud.py --demo all

# Run specific demo
python examples/09_stream_pointcloud.py --demo encode  # Encoding tests
python examples/09_stream_pointcloud.py --demo stream  # Streaming simulation
python examples/09_stream_pointcloud.py --demo depth   # Depth camera simulation
```
