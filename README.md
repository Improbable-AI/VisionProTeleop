<!-- omit in toc -->
VisionProTeleop
===========


> **🎉 V2 UPDATE: Low-Latency Video/Audio/Simulation Streaming!** Now stream video, audio, and MuJoCo simulations back to Vision Pro via WebRTC — alongside the original hand tracking stream, from any machine on the network. Update the app, `pip install --upgrade avp_stream`, and you're ready!


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



This VisionOS app and python library allows anyone to get Head + Wrist + Hand Tracking from Vision Pro, and **stream back stereo video / audio feeds from your robot to Vision Pro.** 

> **For a more detailed explanation, check out this short [paper](./assets/short_paper_new.pdf).*

<!-- omit in toc -->
## Table of Contents

- [Key Features](#key-features)
- [Egocentric Video Dataset Recording](#egocentric-video-dataset-recording)
  - [What Gets Recorded](#what-gets-recorded)
  - [Why We Built This](#why-we-built-this)
  - [Hardware Requirements](#hardware-requirements)
  - [Camera Mounting](#camera-mounting)
  - [Camera Calibration](#camera-calibration)
- [Examples](#examples)
- [Getting Started](#getting-started)
  - [Installation](#installation)
  - [Basic Usage](#basic-usage)
- [Streaming Guide](#streaming-guide)
  - [Video Streaming](#video-streaming)
  - [Audio Streaming](#audio-streaming)
  - [MuJoCo Simulation Streaming](#mujoco-simulation-streaming)
    - [Positioning Your Simulation in AR (`relative_to`)](#positioning-your-simulation-in-ar-relative_to)
    - [Hand Tracking Coordinate Frame (`origin`)](#hand-tracking-coordinate-frame-origin)
  - [Configuration Reference](#configuration-reference)
- [API Reference](#api-reference)
  - [Available Data](#available-data)
  - [Axis Convention](#axis-convention)
  - [Hand Skeleton](#hand-skeleton)
- [Performance](#performance)
  - [Latency Benchmark Results](#latency-benchmark-results)
- [Appendix](#appendix)
  - [Citation](#citation)
  - [Acknowledgements](#acknowledgements)



## Key Features 

1. **Bilateral Data Streams**: 
    - AVP → Python:  Wrist, Fingers and Head Tracking
    - Python → AVP:  Low-Latency Streaming of Video, Audio,  MuJoCo environments presented in AR with RealityKit
2. **Direct UVC Camera Connection**: It supports directly connecting a UVC camera to Vision Pro using its Developer Strap, which can be useful for: 
    - extremely low latency video streaming experience  
    - record **human manipulation videos with egocentric RGB video with accurate hand/head tracking data**, bypassing the limited access to RGB camera feeds on AVP.
3. **Recording to iCloud Drive**: The app can record the incoming/outgoing data streams (video + hand tracking) to personal iCloud Drive for easy data syncing and sharing. 
4. **Wired Network Connection** : It also supports wired network connection with a computer that's running our Python client via Developer Strap for low latency communication. 

## Egocentric Video Dataset Recording

One unique capability of this system is recording **egocentric human manipulation video datasets** with synchronized hand and head tracking data. This is particularly valuable for imitation learning, human behavior analysis, and robotics research.

### What Gets Recorded

When recording in egocentric video mode, the system captures:
- **Video frames** from a connected UVC camera (egocentric RGB view)
- **Hand tracking data** — full 25-joint hand skeleton for both hands
- **Head tracking data** — 6-DoF head pose in world coordinates
- **Wrist tracking data** — wrist poses and roll angles
- **Timestamps** — for accurate synchronization between all modalities

All data can be recorded directly to **iCloud Drive** for easy syncing and sharing.

### Why We Built This

Although Vision Pro has multiple high-quality RGB cameras built-in, **Apple does not grant individual developers access to these camera feeds**. Access requires an Apple Enterprise account with a complicated approval process, making it impractical for most researchers and developers.

Similarly, **Meta's Project Aria glasses** offer egocentric recording capabilities, but they also require going through a complex approval process that's difficult to obtain unless you're officially affiliated with Meta.

**Our solution bypasses these limitations** by using an external UVC camera connected via the Developer Strap. This approach offers several advantages:
- **No approval process** — works with any standard UVC camera
- **Camera flexibility** — use any camera that fits your research needs (wide-angle, high-resolution, stereo, etc.)
- **Full control** — direct access to raw video frames for custom processing
- **Synchronized data** — video frames are perfectly synced with Vision Pro's precise hand/head tracking

### Hardware Requirements

To record egocentric video datasets, you'll need three components:

| Component | Description | 
|-----------|-------------|
| **1. UVC Camera** | Any USB Video Class compatible camera. We recommend compact cameras with wide field-of-view for capturing hand manipulation. |
| **2. Developer Strap** | Apple's [Vision Pro Developer Strap](https://www.apple.com/shop/product/MW3N3LL/A/apple-vision-pro-developer-strap) provides the USB-C port needed to connect the camera. |
| **3. 3D Printed Brackets** | Custom mounting brackets to attach the camera to the Developer Strap securely. |

### Camera Mounting

We provide CAD models for 3D printing camera mounting brackets in the [`assets/adapters/`](assets/adapters/) folder:

| File | Description |
|------|-------------|
| `attachment_left.step` | Left-side camera mount bracket |
| `attachment_right.step` | Right-side camera mount bracket |
| `camera_head.step` | Camera head adapter |

📺 **Video Tutorial**: Watch our [camera attachment tutorial on YouTube](https://youtu.be/vGd3XjLV0kw) for step-by-step assembly instructions.

### Camera Calibration

After mounting the camera, you'll need to calibrate it to accurately align the video frames with the tracking data. The calibration process includes:

1. **Intrinsic Calibration** — Determines the camera's internal parameters (focal length, distortion coefficients)
2. **Extrinsic Calibration** — Determines the camera's position and orientation relative to the Vision Pro head frame

Both calibrations can be performed using the companion **iOS app (Tracking Manager)** or directly on the Vision Pro app. For detailed instructions, see the [Camera Calibration Guide](docs/camera_calibration.md).

## Examples

The best way to learn about this app is to go through the [examples](examples/) folder. It contains 13 examples covering hand tracking, video/audio streaming, and sample teleoperation script using AR simulation streaming feature. 
 

## Getting Started

### Installation

**Step 1. Install the app on Vision Pro**

This app is officially on VisionOS App Store! You can search for **[Tracking Streamer](https://apps.apple.com/us/app/tracking-streamer/id6478969032)** from the App Store and install the app. 

> If you want to play around with the app, you can build/install the app yourself too. To learn how to do that, take a look at this [documentation](/how_to_install.md). This requires (a) Apple Developer Account, (b) Vision Pro Developer Strap, and (c) a Mac with Xcode installed. 

After installation, click and open the app on Vision Pro. It should show something like this. Click **START with Video Streaming** if you want to stream back videos from your robot over webRTC. Otherwise, click the left button. That's it!  


**Step 2. Install Python Library**

The following python package allows you to receive the data stream from any device that's connected to the same WiFi network. Install the package: 

```
pip install --upgrade avp_stream 
```

### Basic Usage

Add this code snippet to any of your projects: 

```python
from avp_stream import VisionProStreamer
avp_ip = "10.31.181.201"   # example IP 
s = VisionProStreamer(ip = avp_ip)

while True:
    r = s.latest
    print(r['head'], r['right_wrist'], r['right_fingers'])
```


## Streaming Guide

Stream your robot's video feed, audio, and even MuJoCo simulations back to Vision Pro via WebRTC. Make sure you upgrade both the python library and the VisionOS app.

### Video Streaming

```python
from avp_stream import VisionProStreamer
avp_ip = "10.31.181.201"   # Vision Pro IP (shown in the app)
s = VisionProStreamer(ip = avp_ip)

# Configure and start video streaming
s.configure_video(device="/dev/video0", format="v4l2", size="640x480", fps=30)
s.start_webrtc()  # Start streaming to Vision Pro

while True:
    r = s.get_latest()
    print(r['head'], r['right_wrist'], r['right_fingers'])
```

**Example 1: Camera with custom processing:**
```python
s = VisionProStreamer(ip=avp_ip)
s.register_frame_callback(my_image_processor)  # Your processing function
s.configure_video(device="/dev/video0", format="v4l2", size="640x480", fps=30)
s.start_webrtc()
```

**Example 2: Stereo camera (side-by-side):**
```python
s = VisionProStreamer(ip=avp_ip)
s.configure_video(device="/dev/video0", format="v4l2", size="1920x1080", fps=30, stereo=True)
s.start_webrtc()
```

**Example 3: Synthetic video:**
```python
s = VisionProStreamer(ip=avp_ip)
s.register_frame_callback(render_simulation_frame)  # Generate frames programmatically
s.configure_video(size="1280x720", fps=60)  # No device = synthetic mode
s.start_webrtc()
```

### Audio Streaming

```python
s = VisionProStreamer(ip=avp_ip)
s.register_frame_callback(visualizer)
s.register_audio_callback(audio_generator)  # Generate or process audio
s.configure_video(size="1280x720", fps=60)
s.configure_audio(sample_rate=48000, stereo=True)
s.start_webrtc()
```

### MuJoCo Simulation Streaming

Stream your MuJoCo physics simulations directly into AR! The simulation scene is converted to USD and rendered natively on Vision Pro using RealityKit, with real-time pose updates streamed via webRTC.

![](assets/diagram-mjar3.png)

https://github.com/user-attachments/assets/7e6a3b6a-34f8-472a-ac6f-0f032fc0eae5

```python
import mujoco
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

s = VisionProStreamer(ip=avp_ip)
s.configure_sim("robot.xml", model, data, relative_to=[0, 1, 0.5, -90])
s.start_webrtc()

while True:
    mujoco.mj_step(model, data)
    s.update_sim()  # Stream updated poses to Vision Pro
```

#### Positioning Your Simulation in AR (`relative_to`)

Since AR blends your simulation with the real world, you need to decide where the simulation's `world` frame should be placed in your physical space. Use `relative_to` parameter:

- **4-dim**: `[x, y, z, yaw°]` — translation + rotation around z-axis (degrees)
- **7-dim**: `[x, y, z, qw, qx, qy, qz]` — full quaternion orientation

```python
# Place world frame 0.8m above ground, rotated 90° around z-axis
s.configure_sim("robot.xml", model, data, relative_to=[0, 0, 0.8, 90])
```

**Default Behavior**: VisionOS automatically detects the physical ground and places the origin there (below your feet if standing, below your chair if sitting).

| Examples from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | [Unitree G1 XML](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_g1/scene.xml) | [Google Robot XML](https://github.com/google-deepmind/mujoco_menagerie/tree/main/google_robot/scene.xml) | [ALOHA 2 XML](https://github.com/google-deepmind/mujoco_menagerie/blob/main/aloha/scene.xml) |
|-------|---------|----------|----------|
| Visualization of `world` frame | ![](assets/unitree_g1.png)  | ![](assets/google_robot.png)     | ![](assets/aloha2.png)     |
|  | `world` frame is attached on a "ground".     | `world` frame is attached on a "ground".     | `world` frame is attached on a "table".     |
| Recommended `attach_to` | Default Setting    | Default Setting     | Offset in `z-axis`, that can bring up the table surface to reasonable height in your real world.    |

#### Hand Tracking Coordinate Frame (`origin`)

When using MuJoCo simulation streaming, you often want hand tracking data in the simulation's coordinate frame (not Vision Pro's native frame). By default, calling `configure_sim()` automatically sets `origin="sim"`, so hand positions are relative to your scene's `world` frame.

```python
s = VisionProStreamer(ip=avp_ip)
s.configure_sim("robot.xml", model, data, relative_to=[0, 0, 0.8, 90])
# origin is now "sim" — hand tracking is in simulation coordinates

# You can also switch manually:
s.set_origin("avp")  # Vision Pro's native coordinate frame
s.set_origin("sim")  # Simulation's coordinate frame (relative to attach_to)
```

| Origin | Hand Tracking Frame | Use Case |
|--------|---------------------|----------|
| `"avp"` | Vision Pro ground frame | Default, general hand tracking |
| `"sim"` | Simulation world frame | Teleoperation, robot control in sim |



### Configuration Reference

**Video Configuration** (`configure_video`):
| Parameter | Description | Example |
|-----------|-------------|---------|
| `device` | Camera device. `None` for synthetic frames | `"/dev/video0"`, `"0:none"` (macOS), `None` |
| `format` | Video format | `"v4l2"` (Linux), `"avfoundation"` (macOS) |
| `size` | Resolution as "WxH" | `"640x480"`, `"1280x720"`, `"1920x1080"` |
| `fps` | Frame rate | `30`, `60` |
| `stereo` | Side-by-side stereo video | `True`, `False` |

**Audio Configuration** (`configure_audio`):
| Parameter | Description | Example |
|-----------|-------------|---------|
| `device` | Audio device. `None` for synthetic audio | `":0"` (default mic on macOS), `None` |
| `sample_rate` | Sample rate in Hz | `48000` |
| `stereo` | Stereo or mono audio | `True`, `False` |

**Simulation Configuration** (`configure_sim`):
| Parameter | Description | Example |
|-----------|-------------|---------|
| `xml_path` | Path to MuJoCo XML scene | `"scene.xml"` |
| `model` | MuJoCo model object | `mujoco.MjModel` |
| `data` | MuJoCo data object | `mujoco.MjData` |
| `relative_to` | Scene placement [x, y, z, yaw°] or [x, y, z, qw, qx, qy, qz] | `[0, 1, 0.5, -90]` |

**Note:** Finding the right combination of `device`, `format`, `size`, and `fps` can be tricky since cameras only support certain combinations. Use this script to find valid configurations:

```bash
python test_video_devices.py --live
``` 


## API Reference

### Available Data

```python
r = s.latest
```

`r` is a dictionary containing the following data streamed from AVP: 

```python
r['head']: np.ndarray  
  # shape (1,4,4) / measured from ground frame
r['right_wrist']: np.ndarray 
  # shape (1,4,4) / measured from ground frame
r['left_wrist']: np.ndarray 
  # shape (1,4,4) / measured from ground frame
r['right_fingers']: np.ndarray 
  # shape (25,4,4) / measured from right wrist frame 
r['left_fingers']: np.ndarray 
  # shape (25,4,4) / measured from left wrist frame 
r['right_pinch_distance']: float  
  # distance between right index tip and thumb tip 
r['left_pinch_distance']: float  
  # distance between left index tip and thumb tip 
r['right_wrist_roll']: float 
  # rotation angle of your right wrist around your arm axis
r['left_wrist_roll']: float 
 # rotation angle of your left wrist around your arm axis
```




### Axis Convention

Refer to the image below to see how the axis are defined for your head, wrist, and fingers. 

![](assets/axis_convention.png)

### Hand Skeleton

![](assets/hand_skeleton_convention.png)

Refer to the image above to see what order the joints are represented in each hand's skeleton. 


## Performance

### Latency Benchmark Results

We performed comprehensive round-trip latency measurements to benchmark our video streaming system. The measurement captures the full cycle: 
1. Python encodes a timestamp into a video frame as a marker
2. WebRTC transmission happens over the network
3. Vision Pro decodes the image, and reads the marker ID 
4. sends the marker ID back to Python.
5. Python calculates latency. 

This provides a conservative upper bound on user-experienced latency. According to our own testing, the system can consistently hit under 100ms both in wired mode and wireless mode for resolution under 720p. When wired up (requires developer strap), you can get stable 50ms latency even for **stereo 4K streaming**. 

For detailed methodology, test configurations, and complete results, see the **[Benchmark Documentation](docs/benchmark.md)**.

![](comparison.png)


## Appendix

### Citation

If you use this repository in your work, consider citing:

    @software{park2024avp,
        title={Using Apple Vision Pro to Train and Control Robots},
        author={Park, Younghyo and Agrawal, Pulkit},
        year={2024},
        url = {https://github.com/Improbable-AI/VisionProTeleop},
    }

### Acknowledgements 

We acknowledge support from Hyundai Motor Company and ARO MURI grant number W911NF-23-1-0277. 

[![Star History Chart](https://api.star-history.com/svg?repos=improbable-ai/visionproteleop&type=date&legend=top-left)](https://www.star-history.com/#improbable-ai/visionproteleop&type=date&legend=top-left)
