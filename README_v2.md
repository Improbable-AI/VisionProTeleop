<!-- omit in toc -->
VisionProTeleop
===========

<div align="center">
  <img width="340" src="assets/vptv2.png">
</div>
<p align="center">
  <a href="https://pypi.org/project/avp_stream/">
    <img src="https://img.shields.io/pypi/v/avp_stream" alt="CI">
  </a>
  <a href="https://pypi.org/project/avp_stream/">
    <img src="https://img.shields.io/pypi/dm/avp_stream" alt="CI">
  </a>  
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="CI">
  </a>
</p>

A complete ecosystem for using Apple Vision Pro in robotics research — from **real-world teleoperation** to **simulation teleoperation** to **egocentric dataset recording**. Stream hand/head tracking from Vision Pro, send video/audio/simulation back, and record everything to the cloud.

> **For a more detailed explanation, check out this short [paper](./assets/short_paper_new.pdf).**

> The recently updated App Store version of Tracking Streamer requires python library `avp_stream` over 2.50.0. It will show a warning message on the app if the library is outdated. You can upgrade the library by running `pip install --upgrade avp_stream`.


<!-- omit in toc -->
## Table of Contents

- [Overview](#overview)
- [Use Case 1: Real-World Teleoperation](#use-case-1-real-world-teleoperation)
  - [Video \& Audio Streaming](#video--audio-streaming)
  - [Video Configuration Examples](#video-configuration-examples)
  - [Audio Configuration Examples](#audio-configuration-examples)
- [Use Case 2: Simulation Teleoperation](#use-case-2-simulation-teleoperation)
  - [MuJoCo Streaming](#mujoco-streaming)
  - [Positioning Your Simulation in AR](#positioning-your-simulation-in-ar)
  - [Hand Tracking Coordinate Frame](#hand-tracking-coordinate-frame)
- [Use Case 3: Egocentric Video Dataset Recording](#use-case-3-egocentric-video-dataset-recording)
  - [Camera Mounting on AVP](#camera-mounting-on-avp)
  - [Camera Calibration](#camera-calibration)
- [Recording \& Cloud Storage](#recording--cloud-storage)
  - [Automatic Cloud Sync](#automatic-cloud-sync)
  - [Companion iOS App: Tracking Manager](#companion-ios-app-tracking-manager)
  - [Public Dataset Sharing](#public-dataset-sharing)
- [API Reference](#api-reference)
  - [Available Data](#available-data)
  - [Configuration Reference](#configuration-reference)
  - [Axis Convention](#axis-convention)
  - [Hand Skeleton](#hand-skeleton)
- [Performance](#performance)
- [Examples](#examples)
- [Appendix](#appendix)
  - [Citation](#citation)
  - [Acknowledgements](#acknowledgements)


## Overview

This project provides:

1. **Tracking Streamer**: A **VisionOS** app that 
  - streams hand/head tracking data to Python client
  - receive video/audio streams with low latency from Python client
  - present and receive simulation scenes (MuJoCo and Isaac) and its updates with native AR rendering using RealityKit
  - record egocentric video with hand tracking with arbitrary UVC camera connected to Vision Pro
  - (optionally) record every sessions to user's personal cloud storage 
2. **Tracking Manager**: A companion **iOS** app for 
  - managing and viewing recordings on their personal cloud storage
  - configuring app settings for VisionOS app
  - calibrating camera mounted on Vision Pro
  - sharing recorded datasets with others
  - viewing publicly shared datasets
3. **avp_stream**: A **Python** library for 
  - receiving tracking data from Vision Pro
  - streaming video/audio/simulation back to Vision Pro


Together, they enable three major workflows for robotics research:

| Use Case | Description | Primary Tools |
|----------|-------------|---------------|
| **Real-World Teleoperation** | Control physical robots with hand tracking while viewing robot camera feeds | `avp_stream` + WebRTC streaming (or UVC camera) |
| **Simulation Teleoperation** | Control simulated robots with MuJoCo/Isaac Lab rendered directly in AR | `avp_stream` + MuJoCo/Isaac Lab streaming |
| **Egocentric Dataset Recording** | Record first-person manipulation videos with synchronized tracking | UVC camera + Developer Strap |



---

## Use Case 1: Real-World Teleoperation

Stream your robot's camera feed to Vision Pro while receiving hand/head tracking data for control. Perfect for teleoperating physical robots with visual feedback.

### Video & Audio Streaming

```python
from avp_stream import VisionProStreamer

avp_ip = "10.31.181.201"  # Vision Pro IP (shown in the app)
s = VisionProStreamer(ip=avp_ip)

# Configure video streaming from robot camera
s.configure_video(device="/dev/video0", format="v4l2", size="1280x720", fps=30)
s.start_webrtc()

while True:
    r = s.latest
    # Use tracking data to control your robot
    head_pose = r['head']
    right_wrist = r['right_wrist']
    right_fingers = r['right_fingers']
```

### Video Configuration Examples

**Camera with overlay processing:**
```python
def add_overlay(frame):
    return cv2.putText(frame, "Robot View", (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

s = VisionProStreamer(ip=avp_ip)
s.register_frame_callback(add_overlay)
s.configure_video(device="/dev/video0", format="v4l2", size="640x480", fps=30)
s.start_webrtc()
```

**Stereo camera (side-by-side 3D):**
```python
s = VisionProStreamer(ip=avp_ip)
s.configure_video(device="/dev/video0", format="v4l2", size="1920x1080", fps=30, stereo=True)
s.start_webrtc()
```

**Synthetic video (generated frames):**
```python
s = VisionProStreamer(ip=avp_ip)
s.register_frame_callback(render_visualization)  # Your rendering function
s.configure_video(size="1280x720", fps=60)  # No device = synthetic mode
s.start_webrtc()
```

### Audio Configuration Examples

**With microphone input:**
```python
s = VisionProStreamer(ip=avp_ip)
s.configure_video(device="/dev/video0", format="v4l2", size="1280x720", fps=30)
s.configure_audio(device=":0", stereo=True)  # Default mic
s.start_webrtc()
```

**With synthetic audio (feedback sounds):**
```python
def beep_on_pinch(audio_frame):
    # Generate audio based on hand tracking state
    return audio_frame

s = VisionProStreamer(ip=avp_ip)
s.register_audio_callback(beep_on_pinch)
s.configure_video(size="1280x720", fps=60)
s.configure_audio(sample_rate=48000, stereo=True)
s.start_webrtc()
```


---

## Use Case 2: Simulation Teleoperation

Render MuJoCo physics simulations directly in AR on Vision Pro. The simulation is converted to USD and rendered natively using RealityKit, with real-time pose updates streamed via WebRTC. Control simulated robots with your hands in a mixed-reality environment.

![](assets/diagram-mjar3.png)

https://github.com/user-attachments/assets/7e6a3b6a-34f8-472a-ac6f-0f032fc0eae5

### MuJoCo Streaming

```python
import mujoco
from avp_stream import VisionProStreamer

model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

s = VisionProStreamer(ip=avp_ip)
s.configure_sim("robot.xml", model, data, relative_to=[0, 0, 0.8, 90])
s.start_webrtc()

while True:
    # Your control logic using hand tracking
    r = s.latest
    # ... update robot based on hand positions ...
    
    mujoco.mj_step(model, data)
    s.update_sim()  # Stream updated poses to Vision Pro
```

### Positioning Your Simulation in AR

Since AR blends your simulation with the real world, you need to decide where the simulation's `world` frame should be placed in your physical space. Use `relative_to` parameter:

- **4-dim**: `[x, y, z, yaw°]` — translation + rotation around z-axis (degrees)
- **7-dim**: `[x, y, z, qw, qx, qy, qz]` — full quaternion orientation

```python
# Place world frame 0.8m above ground, rotated 90° around z-axis
s.configure_sim("robot.xml", model, data, relative_to=[0, 0, 0.8, 90])
```

**Default Behavior**: VisionOS automatically detects the physical ground and places the origin there (below your feet if standing, below your chair if sitting).

| Examples from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | [Unitree G1](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_g1/scene.xml) | [Google Robot](https://github.com/google-deepmind/mujoco_menagerie/tree/main/google_robot/scene.xml) | [ALOHA 2](https://github.com/google-deepmind/mujoco_menagerie/blob/main/aloha/scene.xml) |
|-------|---------|----------|----------|
| Visualization of `world` frame | ![](assets/unitree_g1.png) | ![](assets/google_robot.png) | ![](assets/aloha2.png) |
| | `world` frame on ground | `world` frame on ground | `world` frame on table |
| Recommended `relative_to` | Default | Default | Offset in z-axis |

### Hand Tracking Coordinate Frame

When using MuJoCo simulation streaming, you often want hand tracking data in the simulation's coordinate frame (not Vision Pro's native frame). By default, calling `configure_sim()` automatically sets `origin="sim"`.

```python
s = VisionProStreamer(ip=avp_ip)
s.configure_sim("robot.xml", model, data, relative_to=[0, 0, 0.8, 90])
# origin is now "sim" — hand tracking is in simulation coordinates

# You can switch manually:
s.set_origin("avp")  # Vision Pro's native coordinate frame
s.set_origin("sim")  # Simulation's coordinate frame
```

| Origin | Hand Tracking Frame | Use Case |
|--------|---------------------|----------|
| `"avp"` | Vision Pro ground frame | General hand tracking |
| `"sim"` | Simulation world frame | Teleoperation, robot control |


---

## Use Case 3: Egocentric Video Dataset Recording

Record **egocentric human manipulation video datasets** with synchronized hand and head tracking data. This is invaluable for learning from video, human behavior anaylsis, etc. 


> **Why we built this**: Vision Pro has multiple high-quality RGB cameras, but **Apple doesn't let individual developers access them** — you need an Enterprise account and a complicated approval process. Meta's Project Aria glasses have similar restrictions unless you're officially affiliated with Meta.  So we built a workaround: **connect any standard UVC camera via the Developer Strap**. This gives you full control over your camera choice (wide-angle, high-res, stereo,  whatever your research needs), direct access to raw frames, and precise synchronization with Vision Pro's hand/head tracking. No approval process required.


### Camera Mounting on AVP

We provide CAD models for 3D printing camera mounting brackets in [`assets/adapters/`](assets/adapters/):

| File | Description |
|------|-------------|
| `attachment_left.step` | Left-side camera mount bracket |
| `attachment_right.step` | Right-side camera mount bracket |
| `camera_head.step` | Camera head adapter |

📺 **Video Tutorial**: Watch our [camera attachment tutorial on YouTube](https://youtu.be/vGd3XjLV0kw) for step-by-step assembly instructions.

### Camera Calibration

After mounting the camera, you need to calibrate it to align video frames with tracking data:

1. **Intrinsic Calibration** — Determines camera's internal parameters (focal length, distortion)
2. **Extrinsic Calibration** — Determines camera's position/orientation relative to Vision Pro

Both calibrations can be performed using **Tracking Manager**, our iOS companion app.  For detailed instructions and math behind these calibrations, see the [Camera Calibration Guide](docs/camera_calibration.md).


---

## Recording & Cloud Storage

Any session — whether real-world teleoperation, simulation control, or egocentric recording — can be saved to cloud storage for easy access and sharing.

### Automatic Cloud Sync

Configure cloud storage in the Tracking Streamer app settings:

| Provider | Features |
|----------|----------|
| **iCloud Drive** | Automatic sync to your Apple devices |
| **Dropbox** | Cross-platform access, easy sharing |
| **Google Drive** | Integration with Google Workspace |

Recordings include:
- Video file (H.264/H.265 encoded)
- Tracking data (JSON format with all hand/head poses)
- Metadata (timestamps, calibration info, session details)
- Simulation data (if using MuJoCo streaming)

### Companion iOS App: Tracking Manager

The **Tracking Manager** iOS app provides a complete interface for managing your recordings:

| Feature | Description |
|---------|-------------|
| **Personal Recordings** | Browse and manage your recordings from cloud storage |
| **Playback & Inspection** | View synchronized video + 3D skeleton visualization |
| **Calibration** | Perform camera calibration with visual guidance |
| **Vision Pro Settings** | Configure Tracking Streamer settings remotely |
| **Public Sharing** | Share recordings with the research community |

### Public Dataset Sharing

Want to contribute to the research community? The Tracking Manager app allows you to:

1. Select recordings to share publicly
2. Add metadata (task description, environment info)
3. Upload to a shared CloudKit database
4. Browse and download others' public recordings

This creates a growing community dataset of egocentric manipulation videos with tracking data!


---

## API Reference

### Available Data

```python
r = s.latest  # or s.get_latest()
```

| Key | Type | Description |
|-----|------|-------------|
| `head` | `(1,4,4) ndarray` | Head pose matrix (Z-up frame) |
| `left_wrist` / `right_wrist` | `(1,4,4) ndarray` | Wrist pose matrices |
| `left_fingers` / `right_fingers` | `(25,4,4) ndarray` | Finger joints in wrist frame |
| `left_arm` / `right_arm` | `(27,4,4) ndarray` | Full skeleton (includes forearm) |
| `left_pinch_distance` / `right_pinch_distance` | `float` | Thumb-index pinch distance (m) |
| `left_wrist_roll` / `right_wrist_roll` | `float` | Axial wrist rotation (rad) |

### Configuration Reference

**Video** (`configure_video`):
| Parameter | Description | Example |
|-----------|-------------|---------|
| `device` | Camera device (`None` for synthetic) | `"/dev/video0"`, `"0:none"` (macOS) |
| `format` | Video format | `"v4l2"` (Linux), `"avfoundation"` (macOS) |
| `size` | Resolution | `"640x480"`, `"1280x720"`, `"1920x1080"` |
| `fps` | Frame rate | `30`, `60` |
| `stereo` | Side-by-side stereo | `True`, `False` |

**Audio** (`configure_audio`):
| Parameter | Description | Example |
|-----------|-------------|---------|
| `device` | Audio device (`None` for synthetic) | `":0"` (default mic on macOS) |
| `sample_rate` | Sample rate (Hz) | `48000` |
| `stereo` | Stereo or mono | `True`, `False` |

**Simulation** (`configure_sim`):
| Parameter | Description | Example |
|-----------|-------------|---------|
| `xml_path` | MuJoCo XML path | `"scene.xml"` |
| `model` | MuJoCo model | `mujoco.MjModel` |
| `data` | MuJoCo data | `mujoco.MjData` |
| `relative_to` | Scene placement | `[0, 0, 0.8, 90]` |

### Axis Convention

![](assets/axis_convention.png)

### Hand Skeleton

![](assets/hand_skeleton_convention.png)

The 27-joint skeleton order (with forearm tracking):
- `[0]` forearmArm, `[1]` forearmWrist, `[2]` wrist
- `[3-6]` thumb (knuckle, intermediateBase, intermediateTip, tip)
- `[7-11]` index, `[12-16]` middle, `[17-21]` ring, `[22-26]` little


---

## Performance

We performed comprehensive round-trip latency measurements. The system consistently achieves:

| Configuration | Latency |
|---------------|---------|
| Wireless, resolution ≤720p | < 100ms |
| Wired, stereo 4K | ~50ms stable |

For detailed methodology and results, see [Benchmark Documentation](docs/benchmark.md).

![](comparison.png)


---

## Examples

The [examples/](examples/) folder contains 13 examples:

| # | Example | Use Case |
|---|---------|----------|
| 00 | `hand_streaming.py` | Basic hand tracking |
| 01 | `visualize_hand_callback.py` | Synthetic video with hand viz |
| 02 | `visualize_hand_direct.py` | Direct frame generation |
| 03 | `visualize_hand_with_audio_callback.py` | Audio feedback on pinch |
| 04 | `stereo_depth_visualization.py` | Stereo depth demo |
| 05 | `text_scroller_callback.py` | Text overlay example |
| 06 | `stream_from_camera.py` | Camera streaming |
| 07 | `process_frames.py` | Frame processing |
| 08 | `stream_audio_file.py` | Audio file streaming |
| 09 | `mujoco_streaming.py` | MuJoCo AR simulation |
| 10 | `teleop_osc_franka.py` | Franka teleoperation |
| 11 | `diffik_aloha.py` | ALOHA diff IK control |
| 12 | `diffik_shadow_hand.py` | Shadow hand control |


---

## Appendix

### Citation

If you use this project in your research:

```bibtex
@software{park2024avp,
    title={Using Apple Vision Pro to Train and Control Robots},
    author={Park, Younghyo and Agrawal, Pulkit},
    year={2024},
    url={https://github.com/Improbable-AI/VisionProTeleop},
}
```

### Acknowledgements

We acknowledge support from Hyundai Motor Company and ARO MURI grant number W911NF-23-1-0277.

[![Star History Chart](https://api.star-history.com/svg?repos=improbable-ai/visionproteleop&type=date&legend=top-left)](https://www.star-history.com/#improbable-ai/visionproteleop&type=date&legend=top-left)
