# Glass-to-Glass Latency Benchmark

We performed comprehensive glass-to-glass latency measurements across multiple resolutions to evaluate the end-to-end performance of our video streaming system. The tests were conducted on both wired (Ubuntu) and wireless (Mac) connections, streaming from a computer to Vision Pro over the same local network.

## Measurement Methodology

> **Glass-to-Glass Latency Testing:** Our benchmark system measures true end-to-end latency by embedding a visual payload (8×9 grid of binary blocks encoding sequence ID and timestamp) into synthetic video frames on the sender side. The Vision Pro's camera captures these frames from its display, and our Swift decoder extracts the payload to calculate round-trip time:
> 
> `Python (encode timestamp) → WebRTC → Vision Pro (decode + display) → Camera capture → Swift decoder → Latency calculation`
> 
> This "glass-to-glass" approach captures all latency sources including encoding, network transmission, decoding, display rendering, and optical capture, providing a realistic measurement of what users actually experience during teleoperation.

## Test Configuration

- **Trials per resolution:** 1,000 samples
- **Warmup frames:** 100 frames (to stabilize encoder/decoder)
- **Frame rate:** 30 FPS
- **Network setups:**
  - **Wired (Ubuntu):** Vision Pro connected via USB-C developer strap using `setup-avp-wired`
  - **Wireless (Mac):** Vision Pro connected over WiFi 6
- **Test modes:**
  - **Mono:** Single camera view
  - **Stereo:** Side-by-side stereo view (width doubled)

## Running the Benchmark

### Prerequisites

1. Install the package:
   ```bash
   pip install --upgrade avp_stream
   ```

2. For wired connection, set up the USB bridge:
   ```bash
   setup-avp-wired
   ```
   This command configures the network bridge for USB-C connection with Vision Pro using the developer strap.

### Running Tests

**Single resolution test:**
```bash
python avp_stream/latency_test.py --ip <VISION_PRO_IP> --resolution 1280x720
```

**Full resolution sweep (mono):**
```bash
python avp_stream/latency_test.py --ip <VISION_PRO_IP> --sweep --trials 1000
```

**Stereo mode test:**
```bash
python avp_stream/latency_test.py --ip <VISION_PRO_IP> --sweep --stereo --trials 1000
```

**Custom configuration:**
```bash
python avp_stream/latency_test.py \
  --ip <VISION_PRO_IP> \
  --sweep \
  --trials 1000 \
  --warmup 100 \
  --fps 30 \
  --stereo
```

### Plotting Results

After running benchmarks, visualize the results:

```bash
# Single benchmark
python avp_stream/plot_benchmarks.py benchmarks/your_result.json --output plot.png

# Compare multiple benchmarks
python avp_stream/plot_benchmarks.py benchmarks/*.json --output comparison.png
```

## Results

![Glass-to-Glass Latency Results](../comparison.png)

### Key Findings

**Mono Video Streaming:**
- **Wired connection (Ubuntu):**
  - 240p (426×240): **~20ms** mean latency
  - 720p (1280×720): **~20ms** mean latency
  - 2160p (3840×2160): **~50ms** mean latency

- **Wireless connection (Mac WiFi):**
  - 240p (426×240): **~57ms** mean latency
  - 720p (1280×720): **~90ms** mean latency
  - 2160p (3840×2160): **~98ms** mean latency

**Stereo Video Streaming (doubled width):**
- **Wired connection (Ubuntu):**
  - 240p (852×240): **~20ms** mean latency
  - 720p (2560×720): **~20ms** mean latency
  - 2160p (7680×2160): **~60ms** mean latency

- **Wireless connection (Mac WiFi):**
  - 240p (852×240): **~57ms** mean latency
  - 720p (2560×720): **~60ms** mean latency
  - 2160p (7680×2160): **~118ms** mean latency

### Analysis

The results demonstrate several important characteristics of our streaming system:

1. **Wired vs Wireless:** Wired connection via USB-C developer strap provides consistently lower latency (~20ms) and more stable performance compared to wireless (~50-100ms).

2. **Resolution Scaling:** Lower resolutions (240p-720p) maintain very low latency even in wireless mode, while 4K streaming shows moderate increases due to encoding/decoding overhead.

3. **Stereo Impact:** Stereo mode (doubling the horizontal resolution) has minimal impact on latency in wired mode but shows increased jitter in wireless mode at higher resolutions.

4. **Stability:** The shaded regions (±1 standard deviation) indicate stable and predictable latency characteristics across all tested configurations, making the system suitable for real-time teleoperation tasks.

5. **Practical Performance:** Even in the most demanding scenario (4K stereo over wireless), the system maintains sub-120ms latency, which is acceptable for most teleoperation applications.

## Implementation Details

The benchmark system consists of two main components:

### Python Sender (`avp_stream/latency_test.py`)
- Generates synthetic video frames with embedded binary payload
- Encodes sequence ID (32-bit) and timestamp (32-bit) as a visual grid
- Streams via WebRTC using FFmpeg and aiortc
- Waits for detection events from Vision Pro

### Swift Receiver (`ImmersiveView.swift`)
- Captures video frames from WebRTC stream
- Extracts binary payload using Accelerate framework (vImage)
- Calculates round-trip latency from embedded timestamp
- Reports results back to Python via gRPC

The payload encoding uses a robust 8×9 grid of 16×16 pixel blocks, with each block representing a single bit (white=1, black=0). This approach is resilient to compression artifacts and display characteristics.

## References

For more details on the video streaming implementation, see:
- [Video Streaming Documentation](video_streaming.rst)
- [Quick Start Guide](quickstart.rst)
- [API Documentation](api/)
