Quick Start
===========

This guide will help you get started with VisionProTeleop in just a few minutes.

Basic Hand Tracking
-------------------

1. **Start the App on Vision Pro**: Open the **Tracking Streamer** app on your Vision Pro and click "START" (left button for tracking only).

2. **Connect from Python**: Create a simple Python script:

   .. code-block:: python

      from avp_stream import VisionProStreamer
      
      # Replace with your Vision Pro's IP address
      avp_ip = "10.31.181.201"
      s = VisionProStreamer(ip=avp_ip)
      
      # Access the latest tracking data
      while True:
          r = s.latest
          print(f"Head position: {r['head']}")
          print(f"Right wrist: {r['right_wrist']}")
          print(f"Right fingers: {r['right_fingers']}")


With Video Streaming
--------------------

To enable video streaming from your robot back to Vision Pro:

1. **Start the App with Video**:  Click "START with Video Streaming" (right button) on your Vision Pro.

2. **Configure Video Streaming**

   .. code-block:: python

      from avp_stream import VisionProStreamer
      
      avp_ip = "10.31.181.201"
      s = VisionProStreamer(ip=avp_ip)
      
      # Start video streaming from a camera
      s.start_streaming(
          device="/dev/video0",
          format="v4l2",
          size="640x480",
          fps=30,
          stereo_video=False
      )
      
      # Now access tracking data as usual
      while True:
          r = s.latest
          # Your robot control code here

          time.sleep(1/30.) # Maintain desired control rate

Finding the Right Camera Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use the provided test script to find the correct camera parameters:

.. code-block:: bash

   python test_video_devices.py --live

This will help you identify the correct ``device``, ``format``, ``size``, and ``fps`` combination for your camera.

