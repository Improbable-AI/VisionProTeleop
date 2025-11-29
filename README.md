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



This VisionOS app and python library streams your Head + Wrist + Hand Tracking result via gRPC over a WiFi network, so any robots connected to the same wifi network can subscribe and use. **It can also stream stereo (or mono) video / audio feeds from your robot, back to the Vision Pro.** 

> **For a more detailed explanation, check out this short [paper](./assets/short_paper_new.pdf).*

## Supported Features

- [x] Wrist / Fingers / Head Tracking
- [x] Video Streaming
- [x] Audio Streaming
- [x] MuJoCo Environment Streaming in AR
- [x] Wired/Wireless Connection 


## Latency Benchmark Results

We performed comprehensive round-trip latency measurements to benchmark our video streaming system. The measurement captures the full cycle: 
1. Python encodes a timestamp into a video frame as a marker
2. WebRTC transmission happens over the network
3. Vision Pro decodes the image, and reads the marker ID 
4. sends the marker ID back to Python.
5. Python calculates latency. 

This provides a conservative upper bound on user-experienced latency. According to our own testing, the system can consistently hit under 100ms both in wired mode and wireless mode for resolution under 720p. When wired up (requires developer strap), you can get stable 50ms latency even for **stereo 4K streaming**. 

For detailed methodology, test configurations, and complete results, see the **[Benchmark Documentation](docs/benchmark.md)**.

![](comparison.png)



## How to Use

If you use this repository in your work, consider citing:

    @software{park2024avp,
        title={Using Apple Vision Pro to Train and Control Robots},`
        author={Park, Younghyo and Agrawal, Pulkit},
        year={2024},
        url = {https://github.com/Improbable-AI/VisionProTeleop},
    }

### Step 1. Install the app on Vision Pro 


This app is officially on VisionOS App Store! You can search for **[Tracking Streamer](https://apps.apple.com/us/app/tracking-streamer/id6478969032)** from the App Store and install the app. 


> If you want to play around with the app, you can build/install the app yourself too. To learn how to do that, take a look at this [documentation](/how_to_install.md). This requires (a) Apple Developer Account, (b) Vision Pro Developer Strap, and (c) a Mac with Xcode installed. 


### Step 2. Run the app on Vision Pro 

After installation, click and open the app on Vision Pro. It should show something like this. Click **START with Video Streaming** if you want to stream back videos from your robot over webRTC. Otherwise, click the left button. That's it!  

Vision Pro is now streaming the tracking data over your wifi network via gRPC, and ready to receive video stream via webRTC. 


![](assets/app-screenshot.png)



### Step 3. Receive the hand tracking data from anywhere

The following python package allows you to receive the data stream from any device that's connected to the same WiFi network. First, install the package: 

```
pip install --upgrade avp_stream 
```

Then, add this code snippet to any of your projects you were developing: 

```python
from avp_stream import VisionProStreamer
avp_ip = "10.31.181.201"   # example IP 
s = VisionProStreamer(ip = avp_ip)

while True:
    r = s.latest
    print(r['head'], r['right_wrist'], r['right_fingers'])
```

### Step 4. [🎉V2 Update🎉] Stream video/audio/simulation back to Vision Pro! 

Stream your robot's video feed, audio, and even MuJoCo simulations back to Vision Pro via WebRTC. Make sure you upgrade both the python library and the VisionOS app.

#### Quick Start: Video Streaming

```python
from avp_stream import VisionProStreamer
avp_ip = "10.31.181.201"   # Vision Pro IP (shown in the app)
s = VisionProStreamer(ip = avp_ip)

# Configure and start video streaming
s.configure_video(device="/dev/video0", format="v4l2", size="640x480", fps=30)
s.start_webrtc()  # Start streaming to Vision Pro

while True:
    r = s.latest
    print(r['head'], r['right_wrist'], r['right_fingers'])
```

#### Configuration Options

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

#### Example: Streaming Modes

**1. Camera with custom processing:**
```python
s = VisionProStreamer(ip=avp_ip)
s.register_frame_callback(my_image_processor)  # Your processing function
s.configure_video(device="/dev/video0", format="v4l2", size="640x480", fps=30)
s.start_webrtc()
```

**2. Stereo camera (side-by-side):**
```python
s = VisionProStreamer(ip=avp_ip)
s.configure_video(device="/dev/video0", format="v4l2", size="1920x1080", fps=30, stereo=True)
s.start_webrtc()
```

**3. Synthetic video (no camera - e.g., simulation rendering):**
```python
s = VisionProStreamer(ip=avp_ip)
s.register_frame_callback(render_simulation_frame)  # Generate frames programmatically
s.configure_video(size="1280x720", fps=60)  # No device = synthetic mode
s.start_webrtc()
```

**4. Video + Audio:**
```python
s = VisionProStreamer(ip=avp_ip)
s.register_frame_callback(visualizer)
s.register_audio_callback(audio_generator)  # Generate or process audio
s.configure_video(size="1280x720", fps=60)
s.configure_audio(sample_rate=48000, stereo=True)
s.start_webrtc()
```

**5. MuJoCo Simulation AR Streaming:**

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

**6. Everything together (Video + Audio + Simulation):**
```python
s = VisionProStreamer(ip=avp_ip)
s.register_frame_callback(camera_overlay)
s.register_audio_callback(audio_feedback)
s.configure_video(size="1280x720", fps=30)
s.configure_audio(stereo=True)
s.configure_sim("scene.xml", model, data, relative_to=[0, 1, 0.5, 0])
s.start_webrtc()
```

More examples in the [examples](examples) folder.

**Note:** Finding the right combination of `device`, `format`, `size`, and `fps` can be tricky since cameras only support certain combinations. Use this script to find valid configurations:

```bash
python test_video_devices.py --live
``` 







## Available Data

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


### Hand Skeleton used in VisionOS

![](assets/hand_skeleton_convention.png)

Refer to the image above to see what order the joints are represented in each hand's skeleton. 


## App Features


Status window is a good way to monitor the networking status, and control the life cycle of the app. It is shown in full view when you first start the app, but automatically gets minimized when video frame comes in. You can also minimize / maximize as you need. 

![](assets/status-window.png)

You can also modify the video viewport -- where and how the streamed video is presented in your AR environment. 




## Acknowledgements 

We acknowledge support from Hyundai Motor Company and ARO MURI grant number W911NF-23-1-0277. 

[![Star History Chart](https://api.star-history.com/svg?repos=improbable-ai/visionproteleop&type=date&legend=top-left)](https://www.star-history.com/#improbable-ai/visionproteleop&type=date&legend=top-left)
