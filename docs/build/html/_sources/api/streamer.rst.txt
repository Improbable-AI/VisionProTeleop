VisionProStreamer
=================

The main interface for receiving tracking data and streaming video.

.. automodule:: avp_stream.streamer
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

Classes
-------

.. autoclass:: avp_stream.streamer.VisionProStreamer
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

   .. automethod:: __init__
   .. automethod:: start_video_streaming
   .. automethod:: register_frame_callback
   .. automethod:: stop
   .. autoproperty:: latest

Main Methods
^^^^^^^^^^^^

.. py:method:: __init__(ip: str, port: int = 50051)

   Initialize the Vision Pro streamer.

   :param ip: IP address of the Vision Pro device
   :type ip: str
   :param port: gRPC port number (default: 50051)
   :type port: int

.. py:method:: start_video_streaming(device, format, size, fps, stereo=False)

   Start streaming video back to Vision Pro.

   :param device: Path to video device (e.g., "/dev/video0") or None for synthetic
   :type device: str or None
   :param format: Video format (e.g., "v4l2", "dshow", "avfoundation") or None
   :type format: str or None
   :param size: Frame size as string (e.g., "640x480")
   :type size: str
   :param fps: Frames per second
   :type fps: int
   :param stereo: Whether to stream stereo video (side-by-side format)
   :type stereo: bool

.. py:method:: register_frame_callback(callback)

   Register a function to process frames before streaming.

   :param callback: Function that takes a frame (numpy array) and returns processed frame
   :type callback: callable

.. py:method:: stop()

   Stop the streamer and release resources.

.. py:property:: latest

   Get the latest tracking data.

   :return: Dictionary containing tracking data
   :rtype: dict

   Dictionary keys:

   - ``head``: (1, 4, 4) numpy array - head pose in ground frame
   - ``right_wrist``: (1, 4, 4) numpy array - right wrist pose in ground frame
   - ``left_wrist``: (1, 4, 4) numpy array - left wrist pose in ground frame
   - ``right_fingers``: (25, 4, 4) numpy array - right finger joints in wrist frame
   - ``left_fingers``: (25, 4, 4) numpy array - left finger joints in wrist frame
   - ``right_pinch_distance``: float - distance between right thumb and index finger
   - ``left_pinch_distance``: float - distance between left thumb and index finger
   - ``right_wrist_roll``: float - rotation angle of right wrist around arm axis
   - ``left_wrist_roll``: float - rotation angle of left wrist around arm axis

Example Usage
^^^^^^^^^^^^^

Basic usage::

   from avp_stream import VisionProStreamer

   avp_ip = "10.31.181.201"
   streamer = VisionProStreamer(ip=avp_ip)

   while True:
       data = streamer.latest
       print(data['head'], data['right_wrist'])

With video streaming::

   streamer = VisionProStreamer(ip=avp_ip)
   streamer.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30
   )

   while True:
       data = streamer.latest
       # Process tracking data

With frame processing::

   def process_frame(frame):
       # Add overlay or processing
       return frame

   streamer = VisionProStreamer(ip=avp_ip)
   streamer.register_frame_callback(process_frame)
   streamer.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30
   )
