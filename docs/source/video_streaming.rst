Video Streaming
===============

VisionProTeleop supports low-latency video streaming from your robot back to Vision Pro using WebRTC.

Overview
--------

The video streaming feature allows you to:

- Stream camera feeds from your robot to Vision Pro
- Support both mono and stereo camera configurations
- Process frames before streaming
- Generate synthetic frames (e.g., from simulation)
- Achieve low latency (typically 50-150ms) over WiFi

Basic Setup
-----------

Enable Video on Vision Pro
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Start the app with video streaming enabled by clicking **"START with Video Streaming"** (right button).

Start Streaming from Python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from avp_stream import VisionProStreamer
   
   avp_ip = "10.31.181.201"
   s = VisionProStreamer(ip=avp_ip)
   
   s.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30,
       stereo=False
   )

Parameters
^^^^^^^^^^

``device``
   Path to the video device (e.g., ``"/dev/video0"`` on Linux, ``0`` on Windows)

``format``
   Video capture format (``"v4l2"`` for Linux, ``"dshow"`` for Windows, ``"avfoundation"`` for macOS)

``size``
   Frame dimensions as string (e.g., ``"640x480"``, ``"1280x720"``)

``fps``
   Frames per second (typically 30 or 60)

``stereo``
   ``True`` for side-by-side stereo, ``False`` for mono

Finding Camera Configuration
-----------------------------

Use the Test Script
^^^^^^^^^^^^^^^^^^^

The repository includes a helper script to identify the correct camera parameters:

.. code-block:: bash

   python test_video_devices.py --live

This script will:

1. List all available video devices
2. Test different format/size/fps combinations
3. Show a live preview when a valid configuration is found

Example Output
^^^^^^^^^^^^^^

.. code-block:: text

   Found devices:
     /dev/video0 - USB Camera
     /dev/video2 - Integrated Webcam
   
   Testing /dev/video0...
     ✓ v4l2 @ 640x480 @ 30fps - Working!
     ✓ v4l2 @ 1280x720 @ 30fps - Working!
     ✗ v4l2 @ 1920x1080 @ 60fps - Failed

Common Configurations
^^^^^^^^^^^^^^^^^^^^^

**Linux with USB Camera:**

.. code-block:: python

   s.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30
   )

**Windows with USB Camera:**

.. code-block:: python

   s.start_video_streaming(
       device="0",
       format="dshow",
       size="640x480",
       fps=30
   )

**macOS with Built-in Camera:**

.. code-block:: python

   s.start_video_streaming(
       device="0",
       format="avfoundation",
       size="1280x720",
       fps=30
   )

Advanced Usage
--------------

Stereo Camera Streaming
^^^^^^^^^^^^^^^^^^^^^^^

For stereo cameras that output side-by-side images:

.. code-block:: python

   s.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="1280x480",  # Width is double for side-by-side
       fps=30,
       stereo=True
   )

The streamer expects the frame to be horizontally concatenated: ``[left_image | right_image]``

Frame Processing
^^^^^^^^^^^^^^^^

Add custom processing to frames before streaming:

.. code-block:: python

   def add_overlay(frame):
       """Add information overlay to frame."""
       import cv2
       
       # frame is a numpy array (H, W, 3) in RGB format
       cv2.putText(frame, "Robot View", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
       
       # Draw bounding boxes, add filters, etc.
       return frame
   
   s.register_frame_callback(add_overlay)
   s.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30
   )

Multiple Processing Steps
^^^^^^^^^^^^^^^^^^^^^^^^^^

Chain multiple processing functions:

.. code-block:: python

   def preprocessing(frame):
       """Adjust brightness/contrast."""
       import cv2
       alpha = 1.2  # Contrast
       beta = 10    # Brightness
       return cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
   
   def add_info(frame):
       """Add information overlay."""
       import cv2
       import time
       timestamp = time.strftime("%H:%M:%S")
       cv2.putText(frame, timestamp, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
       return frame
   
   # Apply preprocessing first
   processed = preprocessing(frame)
   # Then add overlay
   final = add_info(processed)

Synthetic Frame Generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Generate frames without a physical camera (useful for simulation):

.. code-block:: python

   import numpy as np
   
   def generate_synthetic_frame():
       """Generate a test pattern or simulation render."""
       # Create a synthetic image (720p RGB)
       height, width = 720, 1280
       frame = np.zeros((height, width, 3), dtype=np.uint8)
       
       # Draw something (example: gradient)
       for i in range(height):
           frame[i, :, 0] = int(255 * i / height)  # Red gradient
       
       return frame
   
   s.register_frame_callback(generate_synthetic_frame)
   s.start_video_streaming(
       device=None,    # No physical device
       format=None,    # No format needed
       size="1280x720",
       fps=60
   )

Integration with Simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Example with Isaac Gym or other simulators:

.. code-block:: python

   def get_simulation_frame():
       """Get rendered frame from simulator."""
       # Get camera image from simulator
       camera_image = sim.render_camera()  # Your simulator API
       
       # Convert to RGB numpy array if needed
       frame = np.array(camera_image)
       
       return frame
   
   s.register_frame_callback(get_simulation_frame)
   s.start_video_streaming(
       device=None,
       format=None,
       size="1280x720",
       fps=30
   )

Performance Optimization
------------------------

Resolution Guidelines
^^^^^^^^^^^^^^^^^^^^^

Choose resolution based on your use case:

- **640x480**: Low bandwidth, good for monitoring, ~1-2 Mbps
- **1280x720**: Balance of quality and bandwidth, ~3-5 Mbps
- **1920x1080**: High quality, requires good WiFi, ~8-12 Mbps

Frame Rate Guidelines
^^^^^^^^^^^^^^^^^^^^^

- **30 fps**: Standard, works well for most applications
- **60 fps**: Smoother motion, requires 2x bandwidth
- **15-20 fps**: Acceptable for slow-moving scenarios, saves bandwidth

Reducing Latency
^^^^^^^^^^^^^^^^

1. **Use a 5GHz WiFi network** (lower congestion)
2. **Reduce resolution** if latency is high
3. **Ensure good signal strength** between Vision Pro and router
4. **Minimize WiFi network traffic** from other devices
5. **Use wired connection** for the robot if possible

Monitoring Performance
^^^^^^^^^^^^^^^^^^^^^^

Check the Vision Pro status window for:

- Current frame rate
- Connection quality
- Latency indicators

Troubleshooting
---------------

No Video Appears
^^^^^^^^^^^^^^^^

1. Verify you clicked "START with Video Streaming" on Vision Pro
2. Check that ``start_video_streaming()`` was called successfully
3. Ensure camera is not being used by another application
4. Check the camera parameters are correct

Poor Video Quality
^^^^^^^^^^^^^^^^^^

1. Increase resolution (if bandwidth allows)
2. Check WiFi signal strength
3. Reduce other network traffic
4. Verify camera supports the requested quality

High Latency
^^^^^^^^^^^^

1. Switch to 5GHz WiFi if available
2. Reduce resolution or frame rate
3. Move closer to WiFi router
4. Check for network congestion

Frame Drops
^^^^^^^^^^^

1. Reduce frame rate or resolution
2. Check CPU usage (processing callbacks may be too heavy)
3. Verify WiFi stability
4. Simplify frame processing callbacks

Examples
--------

See the ``examples/`` directory for complete working examples:

- ``01_stream_from_camera.py``: Basic camera streaming
- ``02_process_frames.py``: Frame processing pipeline
- ``03_synthetic_frames.py``: Synthetic frame generation
