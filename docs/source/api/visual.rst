Visual Streaming
================

Client and server modules for video streaming via WebRTC.

Overview
--------

The visual module provides WebRTC-based video streaming functionality, allowing low-latency video transmission from your robot to Vision Pro.

Client Module
-------------

.. automodule:: avp_stream.visual.client
   :members:
   :undoc-members:
   :show-inheritance:

The client module handles video capture and encoding on the robot side.

Server Module
-------------

.. automodule:: avp_stream.visual.server
   :members:
   :undoc-members:
   :show-inheritance:

The server module handles WebRTC signaling and connection management.

Architecture
------------

The video streaming system uses WebRTC for peer-to-peer video transmission:

1. **Client (Robot)**: Captures video frames and encodes them
2. **Signaling**: Exchanges connection information via HTTP
3. **WebRTC**: Establishes direct peer-to-peer connection
4. **Server (Vision Pro)**: Receives and displays video stream

Components
^^^^^^^^^^

Video Capture
~~~~~~~~~~~~~

Supports multiple video sources:

- Physical cameras (V4L2, DirectShow, AVFoundation)
- Synthetic frame generation
- Custom frame providers via callbacks

Frame Processing
~~~~~~~~~~~~~~~~

Optional frame processing pipeline:

- Color space conversion
- Overlay rendering
- Image filters
- Custom transformations

WebRTC Streaming
~~~~~~~~~~~~~~~~

Low-latency streaming features:

- H.264 video encoding
- Adaptive bitrate
- Network resilience
- Sub-200ms latency on WiFi

Usage Examples
--------------

Basic Client Usage
^^^^^^^^^^^^^^^^^^

Stream from a camera::

   from avp_stream.visual.client import VideoStreamClient

   client = VideoStreamClient(
       vision_pro_ip="10.31.181.201",
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30
   )

   client.start()

   try:
       while True:
           # Streaming happens in background
           pass
   except KeyboardInterrupt:
       client.stop()

With Frame Processing
^^^^^^^^^^^^^^^^^^^^^

Add custom frame processing::

   def add_overlay(frame):
       import cv2
       cv2.putText(frame, "Robot View", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
       return frame

   client = VideoStreamClient(
       vision_pro_ip="10.31.181.201",
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30,
       frame_callback=add_overlay
   )

   client.start()

Synthetic Frames
^^^^^^^^^^^^^^^^

Generate frames without a camera::

   import numpy as np

   def generate_frame():
       # Generate a test pattern
       frame = np.zeros((720, 1280, 3), dtype=np.uint8)
       # ... generate content ...
       return frame

   client = VideoStreamClient(
       vision_pro_ip="10.31.181.201",
       device=None,
       format=None,
       size="1280x720",
       fps=60,
       frame_callback=generate_frame
   )

   client.start()

Advanced Configuration
----------------------

Video Encoding Settings
^^^^^^^^^^^^^^^^^^^^^^^

Customize encoding parameters::

   client = VideoStreamClient(
       vision_pro_ip="10.31.181.201",
       device="/dev/video0",
       format="v4l2",
       size="1920x1080",
       fps=30,
       bitrate=5000000,  # 5 Mbps
       codec="h264"
   )

Stereo Configuration
^^^^^^^^^^^^^^^^^^^^

Stream stereo video (side-by-side)::

   client = VideoStreamClient(
       vision_pro_ip="10.31.181.201",
       device="/dev/video0",
       format="v4l2",
       size="1280x480",  # Double width
       fps=30,
       stereo=True
   )

Network Configuration
^^^^^^^^^^^^^^^^^^^^^

Adjust network settings::

   client = VideoStreamClient(
       vision_pro_ip="10.31.181.201",
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30,
       stun_servers=["stun:stun.l.google.com:19302"],
       ice_transport_policy="all"  # or "relay"
   )

Performance Tuning
------------------

Latency Optimization
^^^^^^^^^^^^^^^^^^^^

Minimize latency:

1. Use lower resolution (e.g., 640x480)
2. Reduce frame rate if acceptable (e.g., 30fps)
3. Use 5GHz WiFi network
4. Minimize frame processing overhead

Bandwidth Optimization
^^^^^^^^^^^^^^^^^^^^^^

Reduce bandwidth usage:

1. Lower resolution
2. Reduce frame rate
3. Adjust bitrate settings
4. Use more aggressive compression

Quality Optimization
^^^^^^^^^^^^^^^^^^^^

Improve video quality:

1. Increase resolution (up to 1920x1080)
2. Increase bitrate
3. Ensure good lighting conditions
4. Use higher quality camera

Troubleshooting
---------------

Connection Issues
^^^^^^^^^^^^^^^^^

If WebRTC connection fails:

1. Verify both devices on same network
2. Check firewall settings
3. Ensure Vision Pro app is running
4. Try restarting both client and app

Poor Video Quality
^^^^^^^^^^^^^^^^^^

If video quality is poor:

1. Increase bitrate
2. Improve WiFi signal strength
3. Check camera settings
4. Verify frame processing isn't too heavy

High Latency
^^^^^^^^^^^^

If latency is too high:

1. Reduce resolution
2. Switch to 5GHz WiFi
3. Minimize network traffic
4. Check processing overhead

See Also
--------

- :doc:`streamer` - Main VisionProStreamer interface
- :doc:`../video_streaming` - Video streaming guide
- :doc:`../examples` - Complete examples
