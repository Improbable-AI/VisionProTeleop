Usage Guide
===========

This guide covers the main features and workflows for using VisionProTeleop.

Basic Workflow
--------------

The typical workflow consists of three steps:

1. Start the VisionOS app on your Vision Pro
2. Initialize the VisionProStreamer in your Python code
3. Access tracking data and optionally stream video

Step 1: Start the Vision Pro App
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open the **Tracking Streamer** app on your Vision Pro. You'll see two options:

- **START** (left button): For hand tracking only
- **START with Video Streaming** (right button): For hand tracking + video streaming

The app will display:

- Your Vision Pro's IP address
- Connection status
- Data streaming status

Step 2: Initialize the Streamer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In your Python code:

.. code-block:: python

   from avp_stream import VisionProStreamer
   
   avp_ip = "10.31.181.201"  # Replace with your Vision Pro's IP
   streamer = VisionProStreamer(ip=avp_ip)

Step 3: Access Tracking Data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``latest`` property provides real-time tracking data:

.. code-block:: python

   while True:
       data = streamer.latest
       
       # Access different tracking components
       head_pose = data['head']
       right_wrist_pose = data['right_wrist']
       left_wrist_pose = data['left_wrist']
       right_fingers = data['right_fingers']
       left_fingers = data['left_fingers']
       
       # Access pinch distances
       right_pinch = data['right_pinch_distance']
       left_pinch = data['left_pinch_distance']
       
       # Access wrist roll angles
       right_roll = data['right_wrist_roll']
       left_roll = data['left_wrist_roll']

Video Streaming Options
-----------------------

Basic Video Streaming
^^^^^^^^^^^^^^^^^^^^^

Stream from a physical camera:

.. code-block:: python

   streamer.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30,
       stereo=False
   )

Stereo Video Streaming
^^^^^^^^^^^^^^^^^^^^^^

For stereo cameras (side-by-side format):

.. code-block:: python

   streamer.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="1280x480",  # Double width for stereo
       fps=30,
       stereo=True
   )

Frame Processing
^^^^^^^^^^^^^^^^

Add custom processing to frames before streaming:

.. code-block:: python

   def process_frame(frame):
       # Add overlays, filters, etc.
       # frame is a numpy array
       return processed_frame
   
   streamer.register_frame_callback(process_frame)
   streamer.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30
   )

Synthetic Frames
^^^^^^^^^^^^^^^^

Generate frames without a physical camera:

.. code-block:: python

   def generate_frame():
       # Generate or render a frame
       # Return a numpy array (H, W, 3) in RGB format
       return synthetic_frame
   
   streamer.register_frame_callback(generate_frame)
   streamer.start_video_streaming(
       device=None,
       format=None,
       size="1280x720",
       fps=60
   )

App Features
------------

Status Window
^^^^^^^^^^^^^

The status window displays:

- Network connection status
- IP address
- Frame rate
- Streaming status

The window automatically minimizes when video streaming starts but can be manually toggled.

Video Viewport Control
^^^^^^^^^^^^^^^^^^^^^^^

You can adjust the video viewport in AR space:

- Position: Move the video display in 3D space
- Size: Adjust the virtual screen size
- Distance: Set viewing distance

These controls are accessible through the app's menu system.

Troubleshooting
---------------

Camera Not Found
^^^^^^^^^^^^^^^^

If you get a "camera not found" error:

1. Use the test script to identify available cameras:

   .. code-block:: bash

      python test_video_devices.py --live

2. Verify the camera is properly connected
3. Check that no other application is using the camera

Connection Issues
^^^^^^^^^^^^^^^^^

If you can't connect to Vision Pro:

1. Verify both devices are on the same WiFi network
2. Check the IP address matches what's shown in the app
3. Ensure no firewall is blocking gRPC or WebRTC traffic
4. Try restarting both the app and your Python script

Low Video Frame Rate
^^^^^^^^^^^^^^^^^^^^

If video streaming is slow:

1. Reduce the resolution (``size`` parameter)
2. Lower the frame rate (``fps`` parameter)
3. Check your WiFi connection quality
4. Ensure your camera supports the requested configuration

Best Practices
--------------

1. **Always close the streamer properly** to release resources:

   .. code-block:: python

      try:
          while True:
              data = streamer.latest
              # Your code
      except KeyboardInterrupt:
          streamer.stop()

2. **Test camera parameters** before full deployment using ``test_video_devices.py``

3. **Handle connection drops** gracefully in production code

4. **Use appropriate video resolution** - higher isn't always better due to network constraints

5. **Monitor frame rates** to ensure real-time performance
