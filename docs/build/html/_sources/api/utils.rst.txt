Utilities
=========

Utility modules for coordinate transformations and gRPC communication.

gRPC Utils
----------

.. automodule:: avp_stream.utils.grpc_utils
   :members:
   :undoc-members:
   :show-inheritance:

Handles gRPC communication with Vision Pro.

Isaac Utils
-----------

.. automodule:: avp_stream.utils.isaac_utils
   :members:
   :undoc-members:
   :show-inheritance:

Utilities for Isaac Gym integration.

SE(3) Transformations
---------------------

.. automodule:: avp_stream.utils.se3_utils
   :members:
   :undoc-members:
   :show-inheritance:

Functions for working with SE(3) transformation matrices.

Key Functions
^^^^^^^^^^^^^

Transformation matrix operations::

   from avp_stream.utils.se3_utils import *

   # Extract position from transformation matrix
   position = get_position(transform_matrix)

   # Extract rotation matrix
   rotation = get_rotation(transform_matrix)

   # Compose transformations
   combined = compose_transforms(T1, T2)

   # Invert transformation
   T_inv = invert_transform(T)

Constants
---------

.. automodule:: avp_stream.utils.trn_constants
   :members:
   :undoc-members:
   :show-inheritance:

Constants used throughout the package.

Common Utilities
----------------

Working with Transformation Matrices
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Example: Extract position and orientation::

   import numpy as np
   from avp_stream import VisionProStreamer

   streamer = VisionProStreamer(ip="10.31.181.201")
   data = streamer.latest

   # Get head transformation matrix
   head_matrix = data['head'][0]

   # Extract position (translation)
   position = head_matrix[:3, 3]
   print(f"Position: {position}")

   # Extract rotation matrix
   rotation = head_matrix[:3, :3]

   # Convert to Euler angles (requires scipy)
   from scipy.spatial.transform import Rotation as R
   r = R.from_matrix(rotation)
   euler = r.as_euler('xyz', degrees=True)
   print(f"Euler angles: {euler}")

Coordinate Frame Transformations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Transform between different reference frames::

   import numpy as np

   # Transform finger from wrist frame to ground frame
   finger_in_wrist = data['right_fingers'][5]  # Index finger metacarpal
   wrist_in_ground = data['right_wrist'][0]

   # Compose transformations
   finger_in_ground = wrist_in_ground @ finger_in_wrist

Relative Pose Computation
^^^^^^^^^^^^^^^^^^^^^^^^^^

Compute relative poses between tracked entities::

   # Compute right wrist pose relative to head
   head_matrix = data['head'][0]
   wrist_matrix = data['right_wrist'][0]

   # Invert head matrix
   head_inv = np.linalg.inv(head_matrix)

   # Compute relative pose
   wrist_relative_to_head = head_inv @ wrist_matrix

Distance Calculations
^^^^^^^^^^^^^^^^^^^^^

Compute distances between tracked points::

   # Distance between two finger tips
   thumb_tip = data['right_fingers'][4]  # Thumb distal joint
   index_tip = data['right_fingers'][8]  # Index distal joint

   # Extract positions in wrist frame
   thumb_pos = thumb_tip[:3, 3]
   index_pos = index_tip[:3, 3]

   # Compute Euclidean distance
   distance = np.linalg.norm(thumb_pos - index_pos)
   print(f"Pinch distance: {distance:.4f} meters")
