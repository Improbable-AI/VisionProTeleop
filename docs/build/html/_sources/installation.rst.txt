Installation
============

Python Package
--------------

The easiest way to install the VisionProTeleop Python package is via pip:

.. code-block:: bash

   pip install --upgrade avp_stream


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

If you want to build and install the app yourself for development purposes, you need: 

- Apple Developer Account
- Vision Pro Developer Strap
- Mac with Xcode installed

See `here <https://github.com/Improbable-AI/VisionProTeleop/blob/main/how_to_install.md>`_ for detailed instructions on building and installing the app manually.

Network Setup
--------------

WiFi Connection
^^^^^^^^^^^^^^^

For wireless operation, both your Vision Pro and the device running the Python package must be connected to the same WiFi network. No additional network configuration is required.

Wired Connection 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For optimal performance with the lowest latency, you can connect your Vision Pro via a wired USB-C connection. This requires:

- **Vision Pro Developer Strap** (available from Apple)
- USB-C cable connected from the developer strap to your computer

After physically connecting your Vision Pro via the developer strap, run the wired setup command:

.. code-block:: bash

   setup-avp-wired

This command (provided by the ``avp_stream`` package) configures the network bridge to enable communication over the USB-C connection. Wired connections typically provide significantly lower latency compared to WiFi - see the `benchmark documentation <https://github.com/Improbable-AI/VisionProTeleop/blob/main/docs/benchmark.md>`_ for detailed performance comparisons.

.. note::
   On the Vision Pro, the app displays the IP addresses registered on different network interfaces. You'll see separate IPs for WiFi and wired (USB-C) connections when both are available. Make sure to use the correct IP address corresponding to your connection type when connecting from Python.

