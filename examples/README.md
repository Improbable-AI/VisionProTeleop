# VisionProTeleop with Video Streaming:  Examples

This directory contains examples demonstrating how to stream video to Apple Vision Pro using the `avp_stream` library. The examples showcase two different approaches: **callback-based** and **direct frame updates**.

## Table of Contents

0. [Overview](#callback-vs-direct-methods)
1. [Hand Tracking Visualization](#1-hand-tracking-visualization)
2. [Text Scroller and Animations](#2-text-scroller-and-animations)
3. [Camera Streaming](#3-camera-streaming)
4. [Frame Processing](#4-frame-processing)


---

## Callback vs Direct Methods

### Overview

The library supports two paradigms for video frame generation:

| Aspect | Callback Method | Direct Method |
|--------|----------------|---------------|
| **Control** | Automatic invocation | Explicit control |
| **Timing** | Decoupled from main loop | Synchronized with main loop |
| **Style** | Functional (closures) | Procedural (pure functions) |
| **Main Loop** | Minimal/idle | Active frame generation |
| **Frame Rate** | Internal to video system | Controlled by your code |
| **Use Case** | Background video generation | Sync with control loops |



### Callback Method: `register_frame_callback()`

**How it works:**
1. Register a callback function that processes/generates frames
2. Video streaming system calls your callback automatically at the specified fps
3. Your main loop can focus on other tasks (control logic, data processing, etc.)

**Pros:**
- ✅ **Decoupled timing**: Video frame rate is independent of main loop speed
- ✅ **Ideal for control loops**: Main loop can run at different rate than video (e.g., control at 100Hz, video at 60Hz)
- ✅ **Multithreading advantage**: Frame generation happens in video thread
- ✅ **Consistent frame rate**: Not affected by main loop delays
- ✅ **Clean separation**: Video logic separated from control logic

**Cons:**
- ❌ **Less explicit**: Frame generation timing is hidden
- ❌ **Harder to debug**: Callback runs in separate thread
- ❌ **State management**: Need closures to maintain state
- ❌ **No frame-by-frame control**: Can't easily skip frames or change rate dynamically

**Best for:**
- Robot control loops that need different timing than video
- Background video generation while doing other work
- When you want consistent frame rates regardless of main loop activity
- Streaming camera feed with processing

**Example use case:**
```python
# Robot control running at 100Hz
streamer.register_frame_callback(visualizer_callback())
streamer.start_video_streaming(device=None, fps=60)

while True:
    # Control loop runs at 100Hz
    robot_state = get_robot_state()
    action = compute_action(robot_state)
    robot.execute(action)
    time.sleep(1/100.)  # Control rate: 100Hz
    
    # Video automatically streams at 60Hz in background
```


### Direct Method: `update_frame()`

**How it works:**
1. Generate frames in your main loop
2. Explicitly push each frame using `streamer.update_frame(frame)`
3. You control timing, frame rate, and when frames are sent

**Pros:**
- ✅ **Explicit control**: You decide when to generate and send frames
- ✅ **Synchronized timing**: Frame rate matches your loop rate perfectly
- ✅ **Easy debugging**: All logic in main thread
- ✅ **Dynamic frame rate**: Can change rate on-the-fly
- ✅ **Frame-accurate**: Precise control over which frames are sent
- ✅ **State in main scope**: No need for closures

**Cons:**
- ❌ **Coupled timing**: Frame rate tied to main loop speed
- ❌ **Main loop overhead**: Frame generation happens in main thread
- ❌ **Variable frame rate**: Main loop delays affect video timing
- ❌ **Blocking**: Frame generation can slow down other main loop tasks

**Best for:**
- Visualization that must sync with specific events
- Frame-by-frame recording or replay
- When you need precise control over timing
- Debugging and development
- Simple single-threaded applications

**Example use case:**
```python
streamer.start_video_streaming(device=None, fps=60)

while True:
    # Get data
    hand_data = streamer.get_latest()
    
    # Process data
    processed_data = process(hand_data)
    
    # Generate visualization synced with data processing
    frame = visualize(processed_data)
    
    # Send frame immediately after processing
    streamer.update_frame(frame)
    
    time.sleep(1/60.)  # Everything runs at 60Hz together
```

---

## Choosing the Right Method

### Use **Callback Method** when:
- Your main loop needs to run at a different rate than video
- You're implementing a robot control loop
- You want video generation to not block other operations
- You're streaming camera feed with processing
- You want guaranteed consistent frame rates

### Use **Direct Method** when:
- You need frame-level synchronization with events
- You're debugging or developing
- Your application is single-threaded
- You need dynamic frame rate control
- You want explicit, easy-to-follow control flow

### Hybrid Approach

You can even combine both! Use callbacks for base processing and direct updates for overrides:

```python
# Register callback for normal operation
streamer.register_frame_callback(base_visualizer())
streamer.start_video_streaming(device=None, fps=60)

while True:
    # Do your main work
    do_work()
    
    # Occasionally override with special frames
    if special_event:
        special_frame = generate_special_frame()
        streamer.update_frame(special_frame)  # Overrides callback
    
    time.sleep(1/100.)
```


---

## 1. Hand Tracking Visualization

These examples visualize hand tracking data from the Vision Pro in real-time, projecting 3D hand positions onto a 2D video stream.

### `01_visualize_hand_callback.py` (Callback Method)

**What it does:**
- Receives hand tracking data from Vision Pro via gRPC
- Renders hand positions (wrists and 25 finger joints per hand) in a virtual 3D space
- Color-codes hands based on pinch gestures (green when pinching, blue/red otherwise)
- Streams the visualization back to Vision Pro at 60fps

**How it works:**
```python
def hand_tracking_visualizer(streamer):
    def generate_frame(blank_frame):
        # Get latest hand data
        latest = streamer.get_latest()
        
        # Draw visualization on blank_frame
        # ... drawing code ...
        
        return blank_frame
    return generate_frame

# Register the callback
streamer.register_frame_callback(hand_tracking_visualizer(streamer))
streamer.start_video_streaming(device=None, fps=60, size="1280x720")

# Main loop just waits
while True:
    time.sleep(1/60.)
```

**Key characteristics:**
- Frame generation happens automatically in the video streaming thread
- Main loop is minimal - just keeps program alive
- Frame rate is controlled by the `fps` parameter in `start_video_streaming()`
- Decoupled from main loop timing

**Run it:**
```bash
python examples/01_visualize_hand_callback.py --ip YOUR_VISION_PRO_IP
```

---

### `01_visualize_hand_direct.py` (Direct Method)

**What it does:**
- Same visualization as the callback version
- Gives explicit control over when frames are generated and sent

**How it works:**
```python
def generate_hand_visualization(streamer, width=1280, height=720):
    # Create frame from scratch
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Get latest hand data
    latest = streamer.get_latest()
    
    # Draw visualization on frame
    # ... drawing code ...
    
    return frame

# Start streaming
streamer.start_video_streaming(device=None, fps=60, size="1280x720")

# Main loop explicitly generates and pushes frames
while True:
    frame = generate_hand_visualization(streamer, width=1280, height=720)
    streamer.update_frame(frame)  # Push frame directly
    time.sleep(1/60.)
```

**Key characteristics:**
- Frame generation is explicit in the main loop
- You control exactly when frames are created and sent
- Frame rate controlled by `time.sleep()` in your main loop
- Perfect for syncing with control loops or other timing logic

**Run it:**
```bash
python examples/01_visualize_hand_direct.py --ip YOUR_VISION_PRO_IP
```

---

## 2. Text Scroller and Animations

These examples demonstrate synthetic video generation without any camera input, showcasing three different animation types.

### `02_text_scroller_callback.py` (Callback Method)

**What it does:**
- Generates synthetic animations from scratch
- Three modes: scrolling text, animated gradient, or moving shapes
- No camera required - purely programmatic frame generation

**Available modes:**
- `--mode scroller`: Scrolling text with gradient background
- `--mode gradient`: Animated rainbow gradient with timestamp
- `--mode shapes`: Moving circle and rectangle

**How it works:**
```python
def text_scroller_callback():
    start_time = time.time()
    
    def generate_frame(blank_frame):
        elapsed = time.time() - start_time
        
        # Draw gradient background
        for y in range(h):
            color = int(255 * y / h)
            blank_frame[y, :] = [color, color // 2, 255 - color]
        
        # Draw scrolling text
        text_x = int(w - (elapsed * 100) % (w + 1000))
        cv2.putText(blank_frame, message, (text_x, h // 2), ...)
        
        return blank_frame
    
    return generate_frame

# Register and start
streamer.register_frame_callback(text_scroller_callback())
streamer.start_video_streaming(device=None, fps=60, size="1280x720")
```

**Key characteristics:**
- Uses closure to maintain state (start_time)
- Functional programming style
- Time tracking internal to callback
- Automatic invocation at specified fps

**Run it:**
```bash
# Scrolling text
python examples/02_text_scroller_callback.py --ip YOUR_VISION_PRO_IP --mode scroller

# Animated gradient
python examples/02_text_scroller_callback.py --ip YOUR_VISION_PRO_IP --mode gradient

# Moving shapes
python examples/02_text_scroller_callback.py --ip YOUR_VISION_PRO_IP --mode shapes
```

---

### `02_text_scroller_direct.py` (Direct Method)

**What it does:**
- Same animations as callback version
- Direct control over frame generation timing

**How it works:**
```python
def generate_text_scroller_frame(elapsed, width=1280, height=720):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Draw gradient background
    for y in range(height):
        color = int(255 * y / height)
        frame[y, :] = [color, color // 2, 255 - color]
    
    # Draw scrolling text
    text_x = int(width - (elapsed * 100) % (width + 1000))
    cv2.putText(frame, message, (text_x, height // 2), ...)
    
    return frame

# Start streaming
streamer.start_video_streaming(device=None, fps=60, size="1280x720")

start_time = time.time()

# Main loop controls everything
while True:
    elapsed = time.time() - start_time
    frame = generate_text_scroller_frame(elapsed, width=1280, height=720)
    streamer.update_frame(frame)
    time.sleep(1/60.)
```

**Key characteristics:**
- Pure functions (no closures needed)
- Time tracking in main loop
- Procedural style
- Explicit control over timing

**Run it:**
```bash
# Same usage as callback version
python examples/02_text_scroller_direct.py --ip YOUR_VISION_PRO_IP --mode scroller
```

---

## 3. Camera Streaming

### `03_stream_from_camera.py`

**What it does:**
- Streams real camera feed to Vision Pro
- macOS uses AVFoundation for camera access
- Can apply frame processing callbacks

**How it works:**
```python
streamer = VisionProStreamer(ip=VISION_PRO_IP)

# Optional: Add processing
def process_frame(frame):
    # Modify frame (add text, filters, etc.)
    return frame

streamer.register_frame_callback(process_frame)

# Stream from camera device
streamer.start_video_streaming(
    device="0:none",           # Camera device ID (macOS)
    format="avfoundation",     # macOS camera format
    fps=30,
    size="640x480"
)
```

**Run it:**
```bash
python examples/03_stream_from_camera.py
```

---

## 4. Frame Processing

### `04_process_frames.py`

**What it does:**
- Demonstrates frame processing with camera input
- Shows how to add overlays, filters, and effects to camera frames
- Uses callback method to process frames in real-time

**Run it:**
```bash
python examples/04_process_frames.py
```


---

## Common Patterns

### Pattern 1: Camera with Overlay (Callback)
```python
def add_overlay(frame):
    cv2.putText(frame, f"FPS: {fps}", (10, 30), ...)
    return frame

streamer.register_frame_callback(add_overlay)
streamer.start_video_streaming(device="0:none", format="avfoundation")
```

### Pattern 2: Synthetic Animation (Direct)
```python
streamer.start_video_streaming(device=None, fps=60)

while True:
    frame = generate_animation_frame(time.time())
    streamer.update_frame(frame)
    time.sleep(1/60.)
```

### Pattern 3: Data Visualization Synced with Control (Direct)
```python
streamer.start_video_streaming(device=None, fps=30)

while True:
    # Control loop
    sensor_data = read_sensors()
    action = controller.compute(sensor_data)
    robot.execute(action)
    
    # Visualization synced with control
    viz_frame = visualize_state(sensor_data, action)
    streamer.update_frame(viz_frame)
    
    time.sleep(1/30.)  # 30Hz control + video
```

### Pattern 4: High-Speed Control with Background Video (Callback)
```python
streamer.register_frame_callback(video_generator())
streamer.start_video_streaming(device=None, fps=30)  # 30Hz video

while True:
    # 1kHz control loop
    fast_control_loop()
    time.sleep(1/1000.)
    
    # Video streams at 30Hz independently
```

---

## Tips and Best Practices

1. **Frame Resolution**: Higher resolutions (1920x1080) require more bandwidth. Start with 1280x720 for testing.

2. **Frame Rate**: 60fps provides smooth visualization but uses more resources. 30fps is often sufficient.

3. **Color Format**: Frames should be in BGR24 format (OpenCV default).

4. **Error Handling**: Wrap frame generation in try-except to handle missing data gracefully.

5. **Performance**: For real-time applications, profile your frame generation code to ensure it completes within your target frame time.

6. **Network**: Ensure Vision Pro and computer are on the same network with good connectivity.

---

## Troubleshooting

**No video appearing on Vision Pro?**
- Check IP address is correct
- Verify both devices are on same network
- Check firewall settings (ports 12345, 9999, 8888)
- Look for connection messages in terminal

**Choppy/laggy video?**
- Reduce frame rate
- Reduce resolution
- Simplify frame generation code
- Check network bandwidth

**"Waiting for hand data..." message?**
- Make sure Vision Pro app is running
- Press "Start" in the VisionOS app
- Check gRPC connection (port 12345)

---

## Next Steps

1. Start with `01_visualize_hand_callback.py` to see basic hand tracking
2. Try `01_visualize_hand_direct.py` to compare the direct approach
3. Experiment with `02_text_scroller_callback.py` different modes
4. Try `03_stream_from_camera.py` for real camera streaming
5. Check out `04_process_frames.py` for frame processing examples
6. Build your own visualizations combining these patterns
7. Integrate with your robot control system

For more information, see the main repository README and documentation.
