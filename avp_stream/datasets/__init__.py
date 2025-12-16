"""
avp_stream.datasets - Access public recordings shared via CloudKit.

This module provides functions to query and download public recordings
that users have shared through the VisionProTeleop app.

Quick Start
-----------
>>> from avp_stream.datasets import list_public_recordings, download_recording
>>> 
>>> # List all public recordings
>>> recordings = list_public_recordings()
>>> for rec in recordings:
...     print(f"{rec.title} - {rec.provider}")
>>>
>>> # Download a recording
>>> if recordings:
...     download_recording(recordings[0], dest_dir="./data")

API Token Setup
---------------
To use this module, you need a CloudKit API token. See the docstring in
`cloudkit.py` for detailed setup instructions, or set the environment variable:

    export CLOUDKIT_API_TOKEN="your-api-token"

"""

from .models import PublicRecording
from .cloudkit import list_public_recordings, get_recording
from .downloaders import download_recording

__all__ = [
    "PublicRecording",
    "list_public_recordings",
    "get_recording", 
    "download_recording",
]
