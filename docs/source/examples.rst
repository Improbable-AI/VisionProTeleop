Examples
========

This page provides detailed examples for common use cases.

Complete Examples
-----------------

The repository includes several complete example scripts in the ``examples/`` directory:

Example 1: Basic Camera Streaming
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``examples/01_stream_from_camera.py``

This example demonstrates basic video streaming from a physical camera.

.. code-block:: python

   from avp_stream import VisionProStreamer
   import time
   
   # Vision Pro IP address
   avp_ip = "10.31.181.201"
   
   # Initialize streamer
   streamer = VisionProStreamer(ip=avp_ip)
   print("Connected to Vision Pro")
   
   # Start video streaming from camera
   streamer.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30,
       stereo=False
   )
   print("Video streaming started")
   
   try:
       while True:
           # Get latest tracking data
           data = streamer.latest
           
           # Print head position
           head_pos = data['head'][0, :3, 3]
           print(f"Head: x={head_pos[0]:.2f}, y={head_pos[1]:.2f}, z={head_pos[2]:.2f}")
           
           time.sleep(0.1)
   
   except KeyboardInterrupt:
       print("Stopping...")
       streamer.stop()

Example 2: Frame Processing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``examples/02_process_frames.py``

This example shows how to process frames before streaming them to Vision Pro.

.. code-block:: python

   from avp_stream import VisionProStreamer
   import cv2
   import numpy as np
   import time
   
   avp_ip = "10.31.181.201"
   
   def process_frame(frame):
       """Add overlay and processing to video frame."""
       # frame is RGB numpy array (H, W, 3)
       
       # Add timestamp
       timestamp = time.strftime("%H:%M:%S")
       cv2.putText(frame, timestamp, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
       
       # Add a robot status indicator
       status = "ACTIVE"
       cv2.putText(frame, f"Status: {status}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
       
       # Draw a crosshair in the center
       h, w = frame.shape[:2]
       cv2.line(frame, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 0, 0), 2)
       cv2.line(frame, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 0, 0), 2)
       
       return frame
   
   # Initialize and register callback
   streamer = VisionProStreamer(ip=avp_ip)
   streamer.register_frame_callback(process_frame)
   
   # Start streaming with processing
   streamer.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30
   )
   
   try:
       while True:
           data = streamer.latest
           # Your robot control logic here
           time.sleep(0.1)
   except KeyboardInterrupt:
       streamer.stop()

Example 3: Synthetic Frames
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**File:** ``examples/03_synthetic_frames.py``

This example generates synthetic frames without a physical camera.

.. code-block:: python

   from avp_stream import VisionProStreamer
   import numpy as np
   import time
   
   avp_ip = "10.31.181.201"
   
   # State for animation
   frame_count = 0
   
   def generate_synthetic_frame():
       """Generate an animated test pattern."""
       global frame_count
       
       height, width = 720, 1280
       frame = np.zeros((height, width, 3), dtype=np.uint8)
       
       # Create animated gradient
       t = frame_count / 60.0  # Time in seconds
       
       for i in range(height):
           for j in range(width):
               # Animated color pattern
               r = int(127 + 127 * np.sin(t + i * 0.01))
               g = int(127 + 127 * np.sin(t + j * 0.01))
               b = int(127 + 127 * np.sin(t + (i + j) * 0.005))
               frame[i, j] = [r, g, b]
       
       frame_count += 1
       return frame
   
   # Initialize and register callback
   streamer = VisionProStreamer(ip=avp_ip)
   streamer.register_frame_callback(generate_synthetic_frame)
   
   # Start streaming synthetic frames
   streamer.start_video_streaming(
       device=None,
       format=None,
       size="1280x720",
       fps=60
   )
   
   try:
       while True:
           data = streamer.latest
           time.sleep(0.1)
   except KeyboardInterrupt:
       streamer.stop()

Use Case Examples
-----------------

Robot Teleoperation
^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from avp_stream import VisionProStreamer
   import robot_interface  # Your robot's API
   
   avp_ip = "10.31.181.201"
   streamer = VisionProStreamer(ip=avp_ip)
   robot = robot_interface.Robot()
   
   # Stream robot's camera back to Vision Pro
   streamer.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="640x480",
       fps=30
   )
   
   while True:
       data = streamer.latest
       
       # Use right hand to control gripper
       gripper_opening = data['right_pinch_distance']
       robot.set_gripper(gripper_opening)
       
       # Use right wrist to control end-effector pose
       target_pose = data['right_wrist'][0]
       robot.move_to_pose(target_pose)

Hand Gesture Recognition
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from avp_stream import VisionProStreamer
   import numpy as np
   
   def detect_pinch_gesture(data, threshold=0.02):
       """Detect if user is pinching."""
       right_pinch = data['right_pinch_distance']
       left_pinch = data['left_pinch_distance']
       
       return {
           'right_pinching': right_pinch < threshold,
           'left_pinching': left_pinch < threshold
       }
   
   def detect_fist(data):
       """Detect if hand is in a fist."""
       # Check if all fingers are close to palm
       fingers = data['right_fingers']
       # Implement your fist detection logic
       return False  # Placeholder
   
   avp_ip = "10.31.181.201"
   streamer = VisionProStreamer(ip=avp_ip)
   
   while True:
       data = streamer.latest
       
       gestures = detect_pinch_gesture(data)
       if gestures['right_pinching']:
           print("Right hand pinching - trigger action!")
       
       if gestures['left_pinching']:
           print("Left hand pinching - trigger action!")

Bilateral Teleoperation
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from avp_stream import VisionProStreamer
   import robot_interface
   
   avp_ip = "10.31.181.201"
   streamer = VisionProStreamer(ip=avp_ip)
   robot = robot_interface.DualArmRobot()
   
   # Stream stereo camera from robot
   streamer.start_video_streaming(
       device="/dev/video0",
       format="v4l2",
       size="1280x480",  # Side-by-side stereo
       fps=30,
       stereo=True
   )
   
   while True:
       data = streamer.latest
       
       # Control both arms independently
       right_target = data['right_wrist'][0]
       left_target = data['left_wrist'][0]
       
       robot.move_right_arm(right_target)
       robot.move_left_arm(left_target)
       
       # Control both grippers
       robot.set_right_gripper(data['right_pinch_distance'])
       robot.set_left_gripper(data['left_pinch_distance'])

Simulation Integration
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from avp_stream import VisionProStreamer
   import simulation_env  # Your simulation environment
   
   avp_ip = "10.31.181.201"
   streamer = VisionProStreamer(ip=avp_ip)
   sim = simulation_env.Simulation()
   
   def get_sim_camera():
       """Get rendered frame from simulation."""
       return sim.render_camera()
   
   # Register simulation camera
   streamer.register_frame_callback(get_sim_camera)
   streamer.start_video_streaming(
       device=None,
       format=None,
       size="1280x720",
       fps=30
   )
   
   while True:
       data = streamer.latest
       
       # Update simulation with tracked poses
       sim.set_avatar_head(data['head'][0])
       sim.set_avatar_hand(data['right_wrist'][0])
       
       # Step simulation
       sim.step()

Data Recording
^^^^^^^^^^^^^^

.. code-block:: python

   from avp_stream import VisionProStreamer
   import numpy as np
   import json
   from datetime import datetime
   
   avp_ip = "10.31.181.201"
   streamer = VisionProStreamer(ip=avp_ip)
   
   # Start recording
   recording = []
   start_time = datetime.now()
   
   try:
       while True:
           data = streamer.latest
           
           # Save tracking data with timestamp
           timestamp = (datetime.now() - start_time).total_seconds()
           
           recording.append({
               'timestamp': timestamp,
               'head': data['head'].tolist(),
               'right_wrist': data['right_wrist'].tolist(),
               'left_wrist': data['left_wrist'].tolist(),
               'right_fingers': data['right_fingers'].tolist(),
               'left_fingers': data['left_fingers'].tolist(),
               'right_pinch': data['right_pinch_distance'],
               'left_pinch': data['left_pinch_distance'],
           })
   
   except KeyboardInterrupt:
       # Save recording
       output_file = f"recording_{start_time.strftime('%Y%m%d_%H%M%S')}.json"
       with open(output_file, 'w') as f:
           json.dump(recording, f)
       print(f"Saved recording to {output_file}")
       streamer.stop()

Multi-Modal Control
^^^^^^^^^^^^^^^^^^^

Combine head tracking with hand tracking for complex control schemes:

.. code-block:: python

   from avp_stream import VisionProStreamer
   import robot_interface
   import numpy as np
   
   avp_ip = "10.31.181.201"
   streamer = VisionProStreamer(ip=avp_ip)
   robot = robot_interface.Robot()
   
   while True:
       data = streamer.latest
       
       # Use head orientation to control robot base
       head_matrix = data['head'][0]
       head_forward = head_matrix[:3, 2]  # Z-axis
       
       # Project to ground plane
       forward_2d = head_forward[:2] / np.linalg.norm(head_forward[:2])
       robot.set_base_direction(forward_2d)
       
       # Use right hand for manipulation
       robot.set_arm_pose(data['right_wrist'][0])
       robot.set_gripper(data['right_pinch_distance'])
       
       # Use left hand for mode switching
       if data['left_pinch_distance'] < 0.02:
           robot.switch_mode()  # Toggle between modes

Tips and Best Practices
------------------------

1. **Always handle KeyboardInterrupt** to properly stop the streamer
2. **Test camera parameters** before deployment using ``test_video_devices.py``
3. **Monitor performance** - check frame rates in the Vision Pro status window
4. **Use appropriate coordinate frames** - remember fingers are in wrist frame
5. **Calibrate pinch threshold** based on your specific use case
6. **Add error handling** for network disconnections
7. **Log data** for debugging and analysis
8. **Start with low resolution** and increase as needed
9. **Use callbacks efficiently** - heavy processing can drop frames
10. **Test on target hardware** - performance varies by platform
