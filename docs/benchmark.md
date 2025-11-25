# Round-Trip Video Latency Benchmark

We performed comprehensive round-trip latency measurements across multiple resolutions to evaluate the end-to-end performance of our video streaming system. The tests were conducted on both wired (Ubuntu) and wireless (Mac) connections, streaming from a computer to Vision Pro over the same local network.

## Measurement Methodology

Our benchmark system measures end-to-end round-trip latency by embedding a visual payload (8×9 grid of binary blocks encoding sequence ID and timestamp) into synthetic video frames on the sender side. The Vision Pro decodes this payload from the received video stream and sends the timing information back to Python via gRPC:
 
```
Python (encode timestamp) → WebRTC (forward path) → Vision Pro (decode from video) 
                                                    ↓
Python (calculate latency) ← gRPC (return path) ← Vision Pro
```

This round-trip measurement captures the forward video path (encoding, network transmission, decoding, display rendering) plus the return gRPC response time. While not a pure "glass-to-glass" measurement (which would calculate latency entirely on the Vision Pro side), this approach provides a conservative upper bound on actual user-experienced latency. The gRPC return path adds only a few milliseconds to the dominant forward video latency, making these measurements representative of real-world teleoperation performance.

## Test Configuration

- **Trials per resolution:** 1,000 samples
- **Network setups:**
  - **Wired:** Vision Pro connected via USB-C developer strap using `setup-avp-wired`
  - **Wireless:** Vision Pro connected over WiFi 6
- **Client Machines**: MacBook Pro M2. 

## Running the Benchmark


```bash
python avp_stream/latency_test.py --ip <VISION_PRO_IP> --sweep
```


### Plotting Results

After running benchmarks, visualize the results:

```bash
python avp_stream/plot_benchmarks.py benchmarks/*.json --output comparison.png
```

## Results

![round trip Latency Results](../comparison.png)

