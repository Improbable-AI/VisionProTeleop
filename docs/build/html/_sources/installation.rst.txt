Installation
============

Python Package
--------------

The easiest way to install the VisionProTeleop Python package is via pip:

.. code-block:: bash

   pip install --upgrade avp_stream

Requirements
^^^^^^^^^^^^

The package requires Python 3.7+ and installs the following dependencies:

- numpy
- grpcio
- grpcio-tools
- matplotlib
- opencv-python
- aiortc
- av
- requests
- pyyaml

Vision Pro App
--------------

App Store Installation
^^^^^^^^^^^^^^^^^^^^^^^

The official way to install the app is through the VisionOS App Store:

1. Open the App Store on your Vision Pro
2. Search for **Tracking Streamer**
3. Download and install the app

Manual Installation (Development)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you want to build and install the app yourself for development purposes:

**Requirements:**

- Apple Developer Account
- Vision Pro Developer Strap
- Mac with Xcode installed

**Steps:**

See the `how_to_install.md <https://github.com/Improbable-AI/VisionProTeleop/blob/main/how_to_install.md>`_ for detailed instructions on building and installing the app manually.

Network Setup
-------------

Both your Vision Pro and the device running the Python package must be connected to the same WiFi network. No additional network configuration is required.

Verifying Installation
-----------------------

To verify your Python installation:

.. code-block:: python

   from avp_stream import VisionProStreamer
   print("avp_stream installed successfully!")

To verify your Vision Pro app installation, open the **Tracking Streamer** app on your Vision Pro. You should see the main interface with options to start streaming.
