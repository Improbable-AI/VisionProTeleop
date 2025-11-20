Isaac Environment
=================

Integration with NVIDIA Isaac Gym for simulation.

.. automodule:: avp_stream.isaac_env
   :members:
   :undoc-members:
   :show-inheritance:

Classes and Functions
---------------------

.. autoclass:: avp_stream.isaac_env.IsaacEnv
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

Overview
--------

The Isaac environment module provides integration between VisionProTeleop and NVIDIA Isaac Gym simulation environments. This allows you to control simulated robots using Vision Pro hand tracking data.

Example Usage
-------------

Basic Isaac Gym integration::

   from avp_stream import VisionProStreamer
   from avp_stream.isaac_env import IsaacEnv

   # Initialize Vision Pro streamer
   avp_ip = "10.31.181.201"
   streamer = VisionProStreamer(ip=avp_ip)

   # Create Isaac environment
   env = IsaacEnv(streamer)

   # Training/control loop
   while True:
       # Get tracking data
       data = streamer.latest
       
       # Update simulation with tracking data
       env.update(data)
       
       # Step simulation
       env.step()

See Also
--------

- :doc:`streamer` - Main streamer interface
- :doc:`utils` - Utility functions for Isaac integration
