Data Format
===========

This page describes the structure and format of tracking data returned by VisionProTeleop.

Tracking Data Dictionary
-------------------------

The ``latest`` property returns a dictionary with the following keys:

.. code-block:: python

   data = streamer.latest
   # Returns a dict with these keys:

Head Tracking
^^^^^^^^^^^^^

``data['head']``
   - Type: ``numpy.ndarray``
   - Shape: ``(1, 4, 4)``
   - Description: 4x4 homogeneous transformation matrix representing the head pose
   - Reference frame: Ground frame

Wrist Tracking
^^^^^^^^^^^^^^

``data['right_wrist']``
   - Type: ``numpy.ndarray``
   - Shape: ``(1, 4, 4)``
   - Description: 4x4 homogeneous transformation matrix for right wrist pose
   - Reference frame: Ground frame

``data['left_wrist']``
   - Type: ``numpy.ndarray``
   - Shape: ``(1, 4, 4)``
   - Description: 4x4 homogeneous transformation matrix for left wrist pose
   - Reference frame: Ground frame

Finger Tracking
^^^^^^^^^^^^^^^

``data['right_fingers']``
   - Type: ``numpy.ndarray``
   - Shape: ``(25, 4, 4)``
   - Description: Array of 25 transformation matrices, one for each joint in the right hand
   - Reference frame: Right wrist frame

``data['left_fingers']``
   - Type: ``numpy.ndarray``
   - Shape: ``(25, 4, 4)``
   - Description: Array of 25 transformation matrices, one for each joint in the left hand
   - Reference frame: Left wrist frame

Pinch Distance
^^^^^^^^^^^^^^

``data['right_pinch_distance']``
   - Type: ``float``
   - Description: Euclidean distance between right index fingertip and thumb tip
   - Units: Meters

``data['left_pinch_distance']``
   - Type: ``float``
   - Description: Euclidean distance between left index fingertip and thumb tip
   - Units: Meters

Wrist Roll
^^^^^^^^^^

``data['right_wrist_roll']``
   - Type: ``float``
   - Description: Rotation angle of the right wrist around the arm axis
   - Units: Radians

``data['left_wrist_roll']``
   - Type: ``float``
   - Description: Rotation angle of the left wrist around the arm axis
   - Units: Radians

Coordinate Systems
------------------

Axis Convention
^^^^^^^^^^^^^^^

VisionProTeleop uses the following axis conventions:

- **X-axis**: Points to the right
- **Y-axis**: Points upward
- **Z-axis**: Points backward (following right-hand rule)

For detailed axis orientations for head, wrist, and fingers, refer to the axis convention diagram in the repository.

Transformation Matrices
^^^^^^^^^^^^^^^^^^^^^^^

All poses are represented as 4x4 homogeneous transformation matrices:

.. math::

   T = \begin{bmatrix}
   R_{11} & R_{12} & R_{13} & t_x \\
   R_{21} & R_{22} & R_{23} & t_y \\
   R_{31} & R_{32} & R_{33} & t_z \\
   0 & 0 & 0 & 1
   \end{bmatrix}

Where:
- The upper-left 3x3 block is the rotation matrix R
- The rightmost column (top 3 elements) is the translation vector t
- All translations are in meters

Hand Skeleton Structure
-----------------------

Each hand has 25 joints tracked in the following order:

1. **Wrist** (joint 0)
2. **Thumb** (joints 1-4): Metacarpal → Proximal → Intermediate → Distal
3. **Index Finger** (joints 5-8): Metacarpal → Proximal → Intermediate → Distal
4. **Middle Finger** (joints 9-12): Metacarpal → Proximal → Intermediate → Distal
5. **Ring Finger** (joints 13-16): Metacarpal → Proximal → Intermediate → Distal
6. **Pinky** (joints 17-20): Metacarpal → Proximal → Intermediate → Distal
7. **Palm** (joints 21-24): Additional palm tracking points

See the hand skeleton diagram in the repository for a visual reference.

Working with Transformation Matrices
-------------------------------------

Extracting Position
^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   import numpy as np
   
   data = streamer.latest
   head_matrix = data['head'][0]  # Remove batch dimension
   
   # Extract position (translation)
   position = head_matrix[:3, 3]
   print(f"Head position: x={position[0]}, y={position[1]}, z={position[2]}")

Extracting Rotation
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Extract rotation matrix
   rotation = head_matrix[:3, :3]
   
   # Convert to Euler angles (example using scipy)
   from scipy.spatial.transform import Rotation as R
   r = R.from_matrix(rotation)
   euler_angles = r.as_euler('xyz', degrees=True)
   print(f"Euler angles: {euler_angles}")

Transforming Points
^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Transform a point from wrist frame to ground frame
   finger_in_wrist = data['right_fingers'][5]  # Index finger metacarpal
   wrist_in_ground = data['right_wrist'][0]
   
   # Combine transformations
   finger_in_ground = wrist_in_ground @ finger_in_wrist

Computing Relative Poses
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Compute right wrist pose relative to head
   head_inv = np.linalg.inv(data['head'][0])
   wrist_relative_to_head = head_inv @ data['right_wrist'][0]

Units and Scaling
-----------------

- **Distance/Position**: Meters
- **Rotation**: Radians (for wrist roll) or rotation matrices
- **Time**: Data is streamed in real-time, timestamps depend on network latency

Frame Rates
-----------

- **Tracking data**: Typically 30-90 Hz depending on Vision Pro's update rate
- **Video streaming**: Configurable, typically 30-60 fps
- **Network latency**: Usually 20-50ms on good WiFi networks

Data Quality Considerations
----------------------------

1. **Occlusion**: Hand tracking may be less accurate when hands are not in the Vision Pro's field of view
2. **Lighting**: Very dark environments may affect tracking quality
3. **Network**: WiFi quality affects both tracking data rate and video stream quality
4. **Distance**: Tracking is most accurate within 0.5-2 meters from the Vision Pro
