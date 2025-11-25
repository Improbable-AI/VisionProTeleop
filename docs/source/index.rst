VisionProTeleop Documentation
==============================

.. image:: ../../assets/new-logo.png
   :width: 340
   :align: center

|

Welcome to VisionProTeleop's documentation! This VisionOS app and Python library streams your Head + Wrist + Hand Tracking result via gRPC over a WiFi network, enabling any robots connected to the same WiFi network to subscribe and use the data. **It can also stream stereo (or mono) camera feeds from your robot back to the Vision Pro.**

.. note::
   🎉 **UPDATE:** Now supporting Low-Latency Video Streaming! You can now stream back your robot's camera feed to Vision Pro via WebRTC protocol, alongside the original hand tracking data stream.

Installation
------------

Install the Python package:

.. code-block:: bash

   pip install --upgrade avp_stream

For Vision Pro app installation, search for **Tracking Streamer** on the VisionOS App Store.

Quick Start
-----------

.. code-block:: python

   from avp_stream import VisionProStreamer
   
   avp_ip = "10.31.181.201"  # example IP
   s = VisionProStreamer(ip=avp_ip)
   
   # Optional: Start video streaming
   s.start_streaming(device="/dev/video0", format="v4l2",
                          size="640x480", fps=30, stereo=False)
   
   while True:
       r = s.latest
       print(r['head'], r['right_wrist'], r['right_fingers'])

Citation
--------

If you use this repository in your work, please cite:

.. code-block:: bibtex

   @software{park2024avp,
       title={Using Apple Vision Pro to Train and Control Robots},
       author={Park, Younghyo and Agrawal, Pulkit},
       year={2024},
       url = {https://github.com/Improbable-AI/VisionProTeleop},
   }

Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   installation
   quickstart

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/modules

.. toctree::
   :maxdepth: 1
   :caption: Additional Information

   acknowledgements
   license

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
