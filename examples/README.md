# VisionProTeleop with Video Streaming:  Examples



## Feature Comparison

| Example | Description | Hardware Camera | Update Method | Audio | Stereo Video |
|---------|-------------|----------------|--------------|-------|--------------|
| **[01_visualize_hand_callback.py](01_visualize_hand_callback.py)** | Hand tracking visualization with callback method | ❌ | Callback | ❌ | ❌ |
| **[02_visualize_hand_direct.py](02_visualize_hand_direct.py)** | Hand tracking visualization with direct frame updates | ❌ | Direct | ❌ | ❌ |
| **[03_visualize_hand_with_audio_callback.py](03_visualize_hand_with_audio_callback.py)** | Hand tracking with beep sounds on pinch gestures | ❌ | Callback | ✅ | ❌ |
| **[04_stereo_depth_visualization.py](04_stereo_depth_visualization.py)** | Stereoscopic 3D hand tracking with depth perception | ❌ | Callback | ❌ | ✅ |
| **[05_text_scroller_callback.py](05_text_scroller_callback.py)** | Animated text and graphics without camera input | ❌ | Callback | ❌ | ❌ |
| **[06_stream_from_camera.py](06_stream_from_camera.py)** | Stream live camera feed to Vision Pro | ✅ | Callback | ❌ | ❌ |
| **[07_process_frames.py](07_process_frames.py)** | Camera streaming with custom frame processing | ✅ | Callback | ❌ | ❌ |
| **[08_stream_audio_file.py](08_stream_audio_file.py)** | Hand tracking with looping audio file playback | ❌ | Callback | ✅ | ❌ |
| **[09_stream_pointcloud.py](09_stream_pointcloud.py)** | Point cloud streaming with Draco compression (experimental) | ❌ | N/A | ❌ | ❌ |

**Legend:**
- **Hardware Camera**: Whether the example requires a physical camera device (✅) or generates synthetic video (❌)
- **Update Method**: Whether the example uses callback-based or direct frame update method
- **Audio**: Whether the example includes audio streaming (✅) or not (❌)
- **Stereo Video**: Whether the example demonstrates stereoscopic 3D video (✅) or standard video (❌)


---

## Callback vs Direct Methods

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


**Example use case:**
```python
# Robot control running at 100Hz
streamer.register_frame_callback(visualizer_callback())
streamer.start_streaming(device=None, fps=60)

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


**Example use case:**
```python
streamer.start_streaming(device=None, fps=60)

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

