import grpc
from avp_stream.grpc_msg import * 
from threading import Thread, Lock, Condition
from avp_stream.utils.grpc_utils import * 
import time 
import numpy as np
import asyncio
import socket
from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import fractions 
import math
import cv2
import os
import shutil
import re
import traceback
import logging
import atexit
import signal
from copy import deepcopy
from typing import Optional, Tuple, List, Dict, Any, Callable, TYPE_CHECKING
from pathlib import Path
from aiortc import VideoStreamTrack, AudioStreamTrack
from av import VideoFrame, AudioFrame
from avp_stream.mujoco_msg import mujoco_ar_pb2, mujoco_ar_pb2_grpc
from scipy.spatial.transform import Rotation as R

# Suppress noisy aioice TURN channel bind errors (non-fatal, connection still works via STUN)
logging.getLogger("aioice.turn").setLevel(logging.ERROR)

# Global list to track active streamers for cleanup
_active_streamers = []

def _cleanup_all_streamers():
    """Clean up all active streamers on process exit."""
    for streamer in list(_active_streamers):
        try:
            streamer.cleanup()
        except Exception as e:
            print(f"[CLEANUP] Error during cleanup: {e}")

def _signal_handler(signum, frame):
    """Handle SIGINT/SIGTERM to ensure clean shutdown."""
    print(f"\n[SIGNAL] Received signal {signum}, cleaning up...")
    _cleanup_all_streamers()
    # Re-raise to allow normal exit
    raise KeyboardInterrupt()

# Register cleanup handlers
atexit.register(_cleanup_all_streamers)

# Only register signal handlers in main thread
try:
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
except ValueError:
    # Signal handlers can only be registered in main thread
    pass


# =============================================================================
# Version Information for Compatibility Checking
# =============================================================================
# Version is encoded as: major * 10000 + minor * 100 + patch
# Example: 2.2.2 -> 20202, 3.0.0 -> 30000
# This allows visionOS to compare versions and enforce minimum requirements
LIBRARY_VERSION = "2.50.0"
LIBRARY_VERSION_CODE = 25000  # 2*10000 + 50*100 + 0

YUP2ZUP = np.array([[[1, 0, 0, 0], 
                    [0, 0, -1, 0], 
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]]], dtype = np.float64)


# =============================================================================
# Tracking Data Wrapper Classes
# =============================================================================

# Joint name to index mapping for 27-joint skeleton
# New ordering: wrist(0), thumb(1-4), index(5-9), middle(10-14), ring(15-19), little(20-24), forearm(25-26)
JOINT_NAMES = {
    # Wrist
    "wrist": 0,
    
    # Thumb (4 joints)
    "thumbKnuckle": 1,
    "thumbIntermediateBase": 2,
    "thumbIntermediateTip": 3,
    "thumbTip": 4,
    
    # Index (5 joints)
    "indexMetacarpal": 5,
    "indexKnuckle": 6,
    "indexIntermediateBase": 7,
    "indexIntermediateTip": 8,
    "indexTip": 9,
    
    # Middle (5 joints)
    "middleMetacarpal": 10,
    "middleKnuckle": 11,
    "middleIntermediateBase": 12,
    "middleIntermediateTip": 13,
    "middleTip": 14,
    
    # Ring (5 joints)
    "ringMetacarpal": 15,
    "ringKnuckle": 16,
    "ringIntermediateBase": 17,
    "ringIntermediateTip": 18,
    "ringTip": 19,
    
    # Little (5 joints)
    "littleMetacarpal": 20,
    "littleKnuckle": 21,
    "littleIntermediateBase": 22,
    "littleIntermediateTip": 23,
    "littleTip": 24,
    
    # Forearm (2 joints, may not be present in older apps)
    "forearmWrist": 25,
    "forearmArm": 26,
}


class HandData(np.ndarray):
    """Hand tracking data as a numpy array with named joint access.
    
    This is a numpy ndarray subclass of shape (N, 4, 4) representing joint transforms
    in world frame. It can be used directly as a 27x4x4 array while also supporting
    attribute access to individual joints by name.
    
    Usage::
    
        tracking = TrackingData(streamer.get_latest())
        
        # Direct array access (27 x 4 x 4)
        joints = tracking.right  # Returns the full skeleton array
        joints.shape  # (27, 4, 4)
        
        # Indexing works as expected
        index_tip = tracking.right[9]  # 4x4 matrix
        
        # Named attribute access
        index_tip = tracking.right.indexTip  # Same as tracking.right[9]
        thumb_tip = tracking.right.thumbTip
        wrist = tracking.right.wrist  # Same as tracking.right[0]
        
        # Additional properties
        pinch = tracking.right.pinch_distance
        roll = tracking.right.wrist_roll
    
    Joint names (index):
        wrist(0), thumbKnuckle(1), thumbIntermediateBase(2), thumbIntermediateTip(3), thumbTip(4),
        indexMetacarpal(5), indexKnuckle(6), indexIntermediateBase(7), indexIntermediateTip(8), indexTip(9),
        middleMetacarpal(10), ..., littleTip(24), forearmWrist(25), forearmArm(26)
    """
    
    # These are stored in the array's metadata dict
    _metadata_attrs = ('pinch_distance', 'wrist_roll', 'has_forearm', '_wrist_mat')
    
    def __new__(cls, wrist: np.ndarray, fingers: np.ndarray, arm: np.ndarray,
                pinch_distance: float, wrist_roll: float):
        """Create HandData array from raw tracking values.
        
        Args:
            wrist: Wrist transform (1, 4, 4) or (4, 4)
            fingers: Finger joints in wrist-local frame (25, 4, 4)
            arm: Full arm skeleton in wrist-local frame (N, 4, 4) where N is 25 or 27
            pinch_distance: Distance between thumb and index tips
            wrist_roll: Axial wrist rotation in radians
        """
        # Ensure wrist is (4, 4)
        wrist_mat = wrist[0] if wrist.ndim == 3 else wrist
        
        # Compute world-frame skeleton (wrist @ local_joint for each joint)
        world_joints = wrist_mat @ arm  # Broadcast: (4,4) @ (N,4,4) -> (N,4,4)
        
        # Create the ndarray instance
        obj = np.asarray(world_joints).view(cls)
        
        # Store additional metadata
        obj.pinch_distance = pinch_distance
        obj.wrist_roll = wrist_roll
        obj.has_forearm = arm.shape[0] >= 27
        obj._wrist_mat = wrist_mat
        
        return obj
    
    def __array_finalize__(self, obj):
        """Called when the array is created or viewed."""
        if obj is None:
            return
        # Copy metadata from source object
        self.pinch_distance = getattr(obj, 'pinch_distance', 0.0)
        self.wrist_roll = getattr(obj, 'wrist_roll', 0.0)
        self.has_forearm = getattr(obj, 'has_forearm', False)
        self._wrist_mat = getattr(obj, '_wrist_mat', None)
    
    def __reduce__(self):
        """Support pickling by returning constructor info."""
        # Get the standard ndarray reduce tuple
        pickled_state = super().__reduce__()
        # Add our custom attributes
        new_state = pickled_state[2] + (
            self.pinch_distance, self.wrist_roll, self.has_forearm, self._wrist_mat
        )
        return (pickled_state[0], pickled_state[1], new_state)
    
    def __setstate__(self, state):
        """Restore from pickle."""
        # Extract our custom state
        self.pinch_distance = state[-4]
        self.wrist_roll = state[-3]
        self.has_forearm = state[-2]
        self._wrist_mat = state[-1]
        # Restore ndarray state
        super().__setstate__(state[:-4])
    
    def __getattr__(self, name: str) -> np.ndarray:
        """Access joint by name (e.g., hand.indexTip)."""
        # Don't intercept special numpy attributes
        if name.startswith('_') or name in ('T', 'dtype', 'shape', 'ndim', 'size', 'flat',
                                               'real', 'imag', 'data', 'strides', 'base',
                                               'flags', 'itemsize', 'nbytes', 'ctypes'):
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")
        
        if name in JOINT_NAMES:
            idx = JOINT_NAMES[name]
            if idx < self.shape[0]:
                return self[idx]
            else:
                raise AttributeError(f"Joint '{name}' not available (requires forearm tracking)")
        
        raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")
    
    def __repr__(self) -> str:
        n_joints = self.shape[0] if self.ndim >= 1 else 0
        return f"<HandData: {n_joints}x4x4 joints, has_forearm={self.has_forearm}>"


class TrackingData:
    """Wrapper for Vision Pro tracking data with both dict and attribute access.
    
    This class is fully backward compatible with the old dictionary API while
    providing convenient attribute access for the new API.
    
    Backward compatible dictionary access::
    
        data = streamer.get_latest()
        data["head"]           # (1, 4, 4) head pose
        data["right_wrist"]    # (1, 4, 4) wrist pose  
        data["right_fingers"]  # (25, 4, 4) finger joints
        data["right_arm"]      # (27, 4, 4) full arm skeleton
        data.keys()            # dict_keys([...])
    
    New attribute access::
    
        data.head             # (4, 4) head pose (squeezed)
        data.right            # (27, 4, 4) HandData array
        data.right.indexTip   # (4, 4) single joint
        data.right[9]         # Same as above
    
    Examples::
    
        # Old API (still works)
        pos = data["right_wrist"][0, :3, 3]
        
        # New API
        pos = data.right.wrist[:3, 3]
        index_tip = data.right.indexTip
    """
    
    # Keys supported for backward compatibility
    _DICT_KEYS = {
        "head", "left_wrist", "right_wrist", 
        "left_fingers", "right_fingers",
        "left_arm", "right_arm",
        "left_pinch_distance", "right_pinch_distance",
        "left_wrist_roll", "right_wrist_roll",
    }
    
    def __init__(self, data: Optional[Dict[str, Any]] = None):
        """Initialize TrackingData from a tracking dictionary.
        
        Args:
            data: Dictionary or TrackingData from VisionProStreamer.get_latest(), or None
        """
        # Handle TrackingData input (e.g., from from_streamer after get_latest returns TrackingData)
        if isinstance(data, TrackingData):
            data = data._raw
        
        self._raw = data
        
        if data is None:
            self._head = None
            self._left = None
            self._right = None
            return
        
        # Extract head (1, 4, 4) -> (4, 4)
        head = data.get("head")
        self._head = head[0] if head is not None and head.ndim == 3 else head
        
        # Create HandData for left hand
        self._left = HandData(
            wrist=data.get("left_wrist", np.eye(4)),
            fingers=data.get("left_fingers", np.eye(4)[np.newaxis].repeat(25, axis=0)),
            arm=data.get("left_arm", np.eye(4)[np.newaxis].repeat(25, axis=0)),
            pinch_distance=data.get("left_pinch_distance", 0.0),
            wrist_roll=data.get("left_wrist_roll", 0.0),
        )
        
        # Create HandData for right hand
        self._right = HandData(
            wrist=data.get("right_wrist", np.eye(4)),
            fingers=data.get("right_fingers", np.eye(4)[np.newaxis].repeat(25, axis=0)),
            arm=data.get("right_arm", np.eye(4)[np.newaxis].repeat(25, axis=0)),
            pinch_distance=data.get("right_pinch_distance", 0.0),
            wrist_roll=data.get("right_wrist_roll", 0.0),
        )
    
    @classmethod
    def from_streamer(cls, streamer: "VisionProStreamer") -> "TrackingData":
        """Create TrackingData from a VisionProStreamer's latest data."""
        # Get latest returns TrackingData now, so just return it directly
        return streamer.get_latest()
    
    # -------------------------------------------------------------------------
    # New attribute-style API
    # -------------------------------------------------------------------------
    
    @property
    def head(self) -> Optional[np.ndarray]:
        """Head pose in world frame, shape (4, 4)."""
        return self._head
    
    @property
    def left(self) -> Optional[HandData]:
        """Left hand tracking data as (N, 4, 4) HandData array."""
        return self._left
    
    @property
    def right(self) -> Optional[HandData]:
        """Right hand tracking data as (N, 4, 4) HandData array."""
        return self._right
    
    @property
    def raw(self) -> Optional[Dict[str, Any]]:
        """Original raw dictionary for full backward compatibility."""
        return self._raw
    
    # -------------------------------------------------------------------------
    # Backward compatible dict-style API
    # -------------------------------------------------------------------------
    
    def __getitem__(self, key: str) -> Any:
        """Dict-style access for backward compatibility."""
        if self._raw is None:
            raise KeyError(key)
        if key not in self._raw:
            raise KeyError(key)
        return self._raw[key]
    
    def __contains__(self, key: str) -> bool:
        """Support 'in' operator."""
        return self._raw is not None and key in self._raw
    
    def get(self, key: str, default: Any = None) -> Any:
        """Dict-style get() method."""
        if self._raw is None:
            return default
        return self._raw.get(key, default)
    
    def keys(self):
        """Return dict keys."""
        return self._raw.keys() if self._raw else {}.keys()
    
    def values(self):
        """Return dict values."""
        return self._raw.values() if self._raw else {}.values()
    
    def items(self):
        """Return dict items."""
        return self._raw.items() if self._raw else {}.items()
    
    def __iter__(self):
        """Iterate over keys."""
        return iter(self._raw) if self._raw else iter({})
    
    def __len__(self) -> int:
        """Number of keys in the raw dict."""
        return len(self._raw) if self._raw else 0
    
    # -------------------------------------------------------------------------
    # Utilities
    # -------------------------------------------------------------------------
    
    def __bool__(self) -> bool:
        """Returns True if tracking data is available."""
        return self._raw is not None
    
    def __repr__(self) -> str:
        if self._raw is None:
            return "<TrackingData: no data>"
        left_joints = self._left.shape[0] if self._left is not None else 0
        right_joints = self._right.shape[0] if self._right is not None else 0
        return f"<TrackingData: head + L({left_joints}j) + R({right_joints}j)>"


# =============================================================================
# Media Clock for A/V Synchronization
# =============================================================================

class MediaClock:
    """
    Shared media clock for audio/video synchronization in WebRTC streaming.
    
    This implements a common timing reference for both audio and video tracks,
    using the standard RTP clock rate approach. Both tracks share the same
    epoch and can calculate synchronized presentation timestamps.
    
    Key features:
    - Shared epoch across audio and video tracks
    - NTP-style timestamp for absolute time reference  
    - Methods for synchronized PTS calculation
    - Sub-millisecond precision using time.perf_counter()
    """
    
    # Standard clock rates for WebRTC
    VIDEO_CLOCK_RATE = 90000  # 90kHz for video (RTP standard)
    AUDIO_CLOCK_RATE = 48000  # Match sample rate for audio
    
    def __init__(self):
        self._lock = Lock()
        self._epoch: Optional[float] = None  # time.perf_counter() at start
        self._epoch_ntp: Optional[float] = None  # time.time() at start (for NTP)
        self._started = False
        self._video_frame_count = 0
        self._audio_sample_count = 0
        
    def start(self):
        """Start the media clock. Called when streaming begins."""
        with self._lock:
            if self._started:
                return
            self._epoch = time.perf_counter()
            self._epoch_ntp = time.time()
            self._started = True
            self._video_frame_count = 0
            self._audio_sample_count = 0
    
    def reset(self):
        """Reset the clock (for reconnection scenarios)."""
        with self._lock:
            self._epoch = time.perf_counter()
            self._epoch_ntp = time.time()
            self._video_frame_count = 0
            self._audio_sample_count = 0
    
    @property
    def is_started(self) -> bool:
        return self._started
    
    def elapsed_seconds(self) -> float:
        """Get elapsed time since clock start in seconds."""
        if not self._started:
            return 0.0
        return time.perf_counter() - self._epoch
    
    def elapsed_ms(self) -> float:
        """Get elapsed time since clock start in milliseconds."""
        return self.elapsed_seconds() * 1000.0
    
    def get_video_pts(self, frame_number: int, fps: float) -> int:
        """
        Calculate video PTS using frame number and FPS.
        Uses 90kHz clock rate (RTP standard for video).
        """
        return int(frame_number * self.VIDEO_CLOCK_RATE / fps)
    
    def get_audio_pts(self, sample_count: int) -> int:
        """
        Calculate audio PTS using sample count.
        Audio PTS is simply the sample count when using sample_rate as time_base.
        """
        return sample_count
    
    def get_sync_timestamp(self) -> Dict[str, Any]:
        """
        Get synchronized timestamp information for callbacks.
        
        This allows frame/audio callbacks to query the current media time
        and generate content that is properly synchronized.
        
        Returns:
            dict with:
                - elapsed_s: Seconds since stream start
                - elapsed_ms: Milliseconds since stream start
                - epoch_ntp: NTP timestamp at stream start
                - current_ntp: Current NTP timestamp
        """
        if not self._started:
            return {
                "elapsed_s": 0.0,
                "elapsed_ms": 0.0,
                "epoch_ntp": None,
                "current_ntp": time.time(),
            }
        
        elapsed = time.perf_counter() - self._epoch
        return {
            "elapsed_s": elapsed,
            "elapsed_ms": elapsed * 1000.0,
            "epoch_ntp": self._epoch_ntp,
            "current_ntp": time.time(),
        }
    
    def wait_until_video_time(self, frame_number: int, fps: float) -> float:
        """
        Calculate how long to wait before the next video frame.
        
        Returns:
            Sleep duration in seconds (can be negative if behind schedule).
        """
        if not self._started:
            return 0.0
        
        target_time = self._epoch + (frame_number / fps)
        current_time = time.perf_counter()
        return target_time - current_time
    
    def wait_until_audio_time(self, sample_count: int, sample_rate: int) -> float:
        """
        Calculate how long to wait before the next audio frame.
        
        Returns:
            Sleep duration in seconds (can be negative if behind schedule).
        """
        if not self._started:
            return 0.0
        
        target_time = self._epoch + (sample_count / sample_rate)
        current_time = time.perf_counter()
        return target_time - current_time


# =============================================================================
# Media Track Classes (for WebRTC streaming)
# =============================================================================

def _create_processed_audio_track_class(streamer_instance):
    """Factory function to create ProcessedAudioTrack class with streamer reference."""
    from aiortc import AudioStreamTrack
    from aiortc.contrib.media import MediaPlayer
    from av import AudioFrame
    
    class ProcessedAudioTrack(AudioStreamTrack):
        """Audio track that can process frames via callback or generate synthetic audio.
        
        Uses MediaClock for precise A/V synchronization with the video track.
        """
        kind = "audio"
        
        def __init__(self, audio_device, audio_fmt, callback, stereo=False, sample_rate=None):
            super().__init__()
            self.callback = callback
            self.use_microphone = audio_device is not None
            self.stereo = stereo
            self.sample_rate = sample_rate or 48000
            self._streamer = streamer_instance
            self._sample_count = 0  # Total samples sent (for PTS)
            self._drift_corrections = 0  # Track clock corrections
            
            if self.use_microphone:
                # Use physical microphone
                fmt = audio_fmt if audio_fmt else "avfoundation"
                self.player = MediaPlayer(audio_device, format=fmt, options={})
                self.audio_track = self.player.audio
                channels = "stereo" if stereo else "mono"
                self._streamer._log(f"[AUDIO] Initialized from device: {audio_device} ({channels})")
            else:
                # Generate frames programmatically
                self.player = None
                self.audio_track = None
                frame_duration_s = 0.02  # 20 ms frames for low latency
                self.samples_per_frame = max(1, int(self.sample_rate * frame_duration_s))
                channels = "stereo" if stereo else "mono"
                self._streamer._log(
                    "[AUDIO] Synthetic audio initialized "
                    f"({self.sample_rate}Hz, {self.samples_per_frame} samples/frame, {channels}, 20ms frames)"
                )
        
        async def recv(self):
            # Ensure MediaClock is started
            if not self._streamer._media_clock.is_started:
                self._streamer._media_clock.start()
            
            if self.use_microphone:
                frame = await self.audio_track.recv()
                
                # Apply user callback if registered
                if self.callback is not None:
                    try:
                        processed_frame = self.callback(frame)
                        return processed_frame
                    except Exception as e:
                        self._streamer._log(f"[AUDIO] Callback error: {e}", force=True)
                        return frame
                else:
                    return frame
            else:
                # Generate synthetic audio frame using MediaClock for sync
                clock = self._streamer._media_clock
                
                # Create blank audio frame (silence)
                layout = 'stereo' if self.stereo else 'mono'
                frame = AudioFrame(format='s16', layout=layout, samples=self.samples_per_frame)
                frame.sample_rate = self.sample_rate
                frame.pts = clock.get_audio_pts(self._sample_count)
                frame.time_base = fractions.Fraction(1, self.sample_rate)
                
                # Initialize with silence
                bytes_per_sample = 2  # s16 = 2 bytes per sample
                channels = 2 if self.stereo else 1
                for p in frame.planes:
                    p.update(bytes(self.samples_per_frame * bytes_per_sample * channels))
                
                # Log first few frames
                if self._sample_count < self.samples_per_frame * 3:
                    frame_num = self._sample_count // self.samples_per_frame
                    self._streamer._log(f"[AUDIO] Generated frame #{frame_num}: {self.samples_per_frame} samples, {frame.sample_rate}Hz, {layout}")
                
                # Use MediaClock for precise timing
                sleep_time = clock.wait_until_audio_time(
                    self._sample_count + self.samples_per_frame, 
                    self.sample_rate
                )
                
                # Adaptive timing: sleep if ahead, track drift if behind
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)
                elif sleep_time < -0.05:  # More than 50ms behind
                    self._drift_corrections += 1
                    if self._drift_corrections <= 3:
                        self._streamer._log(f"[AUDIO] Drift warning: {-sleep_time*1000:.0f}ms behind (correction #{self._drift_corrections})")
                
                # Apply callback if registered (callback should generate audio)
                if self.callback is not None:
                    try:
                        processed_frame = self.callback(frame)
                        self._sample_count += self.samples_per_frame
                        if self._sample_count == self.samples_per_frame * 10:
                            self._streamer._log(f"[AUDIO] Streaming normally (~20ms per frame, synced with MediaClock)")
                        return processed_frame
                    except Exception as e:
                        self._streamer._log(f"[AUDIO] Callback error: {e}", force=True)
                        if self._streamer.verbose:
                            traceback.print_exc()
                        self._sample_count += self.samples_per_frame
                        return frame
                else:
                    # No callback - return silence
                    self._sample_count += self.samples_per_frame
                    return frame
    
    return ProcessedAudioTrack


def _create_processed_video_track_class(streamer_instance):
    """Factory function to create ProcessedVideoTrack class with streamer reference."""
    from aiortc import VideoStreamTrack
    from aiortc.contrib.media import MediaPlayer
    from av import VideoFrame
    
    class ProcessedVideoTrack(VideoStreamTrack):
        """Video track that can process frames via callback or generate synthetic video.
        
        Uses MediaClock for precise A/V synchronization with the audio track.
        Uses 90kHz RTP clock rate for video timestamps (WebRTC standard).
        """
        
        def __init__(self, device, fmt, framerate, resolution, callback):
            super().__init__()
            self.callback = callback
            self.width, self.height = resolution
            self.use_camera = device is not None
            self.warmup_frames = 0  # Track warmup frames for encoder optimization
            self._streamer = streamer_instance
            self._drift_corrections = 0  # Track clock corrections
            
            if self.use_camera:
                # Use physical camera
                opts = {
                    "video_size": f"{self.width}x{self.height}", 
                    "framerate": str(framerate),
                    "fflags": "nobuffer",  # Minimize buffering
                    "flags": "low_delay"   # Low delay mode
                }
                self.player = MediaPlayer(device, format=fmt, options=opts)
                self.video_track = self.player.video
                self._streamer._log("[VIDEO] Camera initialized, warming up encoder...")
            else:
                # Generate frames programmatically
                self.player = None
                self.video_track = None
                self.fps = framerate
                self.frame_count = 0

        def set_resolution(self, resolution):
            width, height = resolution
            if width <= 0 or height <= 0:
                self._streamer._log(f"[VIDEO] Warning: Ignoring invalid resolution update: {width}x{height}", force=True)
                return
            self.width, self.height = width, height
            self._streamer._log(f"[VIDEO] ProcessedVideoTrack target resolution set to {width}x{height}")
            
        async def recv(self):
            # Ensure MediaClock is started
            if not self._streamer._media_clock.is_started:
                self._streamer._media_clock.start()
            
            clock = self._streamer._media_clock
            
            if self.use_camera:
                # Check if user provided a frame override
                if self._streamer.user_frame is not None:
                    # Use user-provided frame instead of camera
                    img = self._streamer.user_frame
                    # Get timing from camera track if available
                    try:
                        camera_frame = await self.video_track.recv()
                        pts = camera_frame.pts
                        time_base = camera_frame.time_base
                    except:
                        # Fallback timing using MediaClock
                        pts = clock.get_video_pts(self.warmup_frames, 30)
                        time_base = fractions.Fraction(1, MediaClock.VIDEO_CLOCK_RATE)
                else:
                    # Get frame from camera
                    frame = await self.video_track.recv()
                    img = frame.to_ndarray(format="bgr24")
                    pts = frame.pts
                    time_base = frame.time_base

                if img.shape[:2] != (self.height, self.width):
                    img = cv2.resize(img, (self.width, self.height))
                
                # Store frame for reference
                self.frame = img.copy()
                
                # Log encoder warmup progress
                if self.warmup_frames < 10:
                    self.warmup_frames += 1
                    if self.warmup_frames == 10:
                        self._streamer._log("[VIDEO] Encoder warmed up - optimal encoding should begin")
                
                # Apply user callback if registered
                if self.callback is not None:
                    try:
                        # Apply user's processing function
                        processed_img = self.callback(img)
                        
                        # Convert back to VideoFrame
                        new_frame = VideoFrame.from_ndarray(processed_img, format="bgr24")
                        new_frame.pts = pts
                        new_frame.time_base = time_base
                        return new_frame
                    except Exception as e:
                        if self._streamer.verbose:
                            traceback.print_exc()
                        self._streamer._log(f"Error in frame callback: {e}", force=True)
                        # Return original frame if processing fails
                        new_frame = VideoFrame.from_ndarray(img, format="bgr24")
                        new_frame.pts = pts
                        new_frame.time_base = time_base
                        return new_frame
                else:
                    # No callback, return original frame (identity)
                    new_frame = VideoFrame.from_ndarray(img, format="bgr24")
                    new_frame.pts = pts
                    new_frame.time_base = time_base
                    return new_frame
            else:
                # Generate frame programmatically using MediaClock for sync
                # Use MediaClock for precise timing
                wait_time = clock.wait_until_video_time(self.frame_count, self.fps)
                
                if wait_time > 0:
                    await asyncio.sleep(wait_time)
                elif wait_time < -0.05:
                    # Track drift but don't reset clock (MediaClock is shared)
                    self._drift_corrections += 1
                    if self._drift_corrections <= 3:
                        self._streamer._log(f"[VIDEO] Drift warning: {-wait_time*1000:.0f}ms behind (correction #{self._drift_corrections})")
                
                # Check if user provided a frame override
                if self._streamer.user_frame is not None:
                    # Use user-provided frame
                    self.frame = self._streamer.user_frame.copy()
                else:
                    # Create blank frame
                    self.frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                
                # Apply callback to populate the frame
                if self.callback is not None:
                    try:
                        processed_img = self.callback(self.frame)
                    except Exception as e:
                        if self._streamer.verbose:
                            traceback.print_exc()
                        self._streamer._log(f"Error in frame callback: {e}", force=True)
                        processed_img = self.frame
                else:
                    # No callback - just send blank frame
                    processed_img = self.frame
                
                # Convert to VideoFrame with 90kHz RTP clock rate
                new_frame = VideoFrame.from_ndarray(processed_img, format="bgr24")
                new_frame.pts = clock.get_video_pts(self.frame_count, self.fps)
                new_frame.time_base = fractions.Fraction(1, MediaClock.VIDEO_CLOCK_RATE)
                
                self.frame_count += 1
                return new_frame
    
    return ProcessedVideoTrack


def _is_room_code(value: str) -> bool:
    """Check if a string looks like a room code vs an IP address.
    
    Room codes are alphanumeric with dashes (e.g., "ABC-1234", "XYZ-5678").
    IP addresses are numeric with dots (e.g., "192.168.1.100").
    """
    if not value:
        return False
    # IP addresses contain only digits and dots
    if all(c.isdigit() or c == '.' for c in value):
        # Verify it looks like a valid IP (4 octets)
        parts = value.split('.')
        if len(parts) == 4 and all(p.isdigit() and 0 <= int(p) <= 255 for p in parts):
            return False  # It's an IP address
    # Anything else (contains letters, dashes, etc.) is treated as a room code
    return True


class VisionProStreamer:

    def __init__(self, ip=None, record=True, ht_backend="grpc", benchmark_quiet=False, verbose=False, origin="avp", signaling_url=None, relay_only=False):
        """Initialize the VisionProStreamer.

        Parameters
        ----------
        ip : str
            Connection identifier. The format determines the connection mode:
            - IP address (e.g., "192.168.1.100"): Same-network mode via gRPC.
            - Room code (e.g., "ABC-1234"): Cross-network mode via WebRTC signaling.
        record : bool, default True
            If True, every processed hand tracking sample is appended to an internal list retrievable via `get_recording()`.
        ht_backend : {"grpc","webrtc"}, default "grpc"
            Transport used for hand tracking. Use "webrtc" for lower latency after a WebRTC session is established.
            Note: In cross-network mode (room code), hand tracking always uses WebRTC.
        benchmark_quiet : bool, default False
            Suppress benchmark printouts (see latency measurements in `latency_test.py`).
        verbose : bool, default False
            If True, print detailed status messages. If False (default), only critical messages are shown.
        origin : {"avp", "sim"}, default "avp"
            Coordinate frame origin for hand tracking data.
            - "avp": Hand tracking in Vision Pro's native coordinate frame (default).
            - "sim": Hand tracking relative to the simulation's attach_to position.
              Useful when you want hand positions in the same frame as your MuJoCo scene.
        signaling_url : str, optional
            Custom signaling server URL for cross-network mode.
            Default: "wss://visionpro-signaling.<your-subdomain>.workers.dev/ws"

        Notes
        -----
        The connection mode is automatically inferred from the `ip` parameter:
        - If it looks like an IP address (e.g., "192.168.1.100"): Local/same-network mode.
        - If it looks like a room code (e.g., "ABC-1234"): Cross-network mode.
        
        Examples
        --------
        Same-network (IP address)::
        
            streamer = VisionProStreamer(ip="192.168.1.100")
        
        Cross-network (room code)::
        
            streamer = VisionProStreamer(ip="ABC-1234")
        """
        
        # Validate required parameter
        if not ip:
            raise ValueError("Must specify ip= parameter (IP address for local mode, or room code for cross-network mode)")
        
        # Determine connection mode based on ip format
        self._cross_network_mode = _is_room_code(ip)
        self._room_code = ip if self._cross_network_mode else None
        self._signaling_url = signaling_url or "wss://visionpro-signaling.parkyh9492.workers.dev/ws"
        self._signaling_ws = None
        self._signaling_connected = False
        self._peer_ready = False  # Track if VisionOS peer has joined

        # Vision Pro IP (only used in same-network mode)
        self.ip = None if self._cross_network_mode else ip
        self.record = record 
        
        # In cross-network mode, always use WebRTC for hand tracking
        if self._cross_network_mode:
            self.ht_backend = "webrtc"
        else:
            self.ht_backend = ht_backend.lower()
            
        self.verbose = verbose
        self.origin = origin.lower()
        if self.ht_backend not in {"grpc", "webrtc"}:
            raise ValueError(f"Unsupported ht_backend '{ht_backend}'. Expected 'grpc' or 'webrtc'.")
        if self.origin not in {"avp", "sim"}:
            raise ValueError(f"Unsupported origin '{origin}'. Expected 'avp' or 'sim'.")

        self.recording = [] 
        self.latest = None 
        self.axis_transform = YUP2ZUP
        self.webrtc_info = None  # Store WebRTC server info for HTTP endpoint
        self.info_server = None  # HTTP server for sharing WebRTC address
        self.frame_callback = None  # User-registered frame processing function
        self.audio_callback = None  # User-registered audio processing function
        self.camera = None  # Processed video track (accessible after start_streaming)
        self.user_frame = None  # User-provided frame to override camera input
        self.benchmark_quiet = benchmark_quiet  # Suppress benchmark print statements
        
        # Audio/Video synchronization state
        self._latest_cached = None  # Thread-local cache to reduce lock contention
        self._latest_cache_time = 0  # Timestamp of cached data
        self._av_sync_epoch = None  # Legacy: kept for backward compatibility
        self._media_clock = MediaClock()  # Shared media clock for A/V sync

        self._latest_lock = Lock()
        self._webrtc_hand_channel = None
        self._webrtc_hand_ready = False
        self._webrtc_sim_channel = None  # WebRTC data channel for sim pose streaming
        self._webrtc_sim_ready = False
        self._webrtc_connected = False  # WebRTC ICE connection established
        self._webrtc_connection_condition = Condition()  # For blocking wait
        self._webrtc_loop = None  # Event loop for WebRTC thread
        self._webrtc_thread = None # Thread object for WebRTC event loop
        self._benchmark_epoch = time.perf_counter()
        self._benchmark_condition = Condition()
        self._benchmark_events = {}
        
        # Video/Audio configuration (set by configure_video/configure_audio)
        self._video_config: Optional[Dict[str, Any]] = None
        self._audio_config: Optional[Dict[str, Any]] = None
        self._webrtc_port: int = 9999
        self._server_running: bool = False
        
        # MuJoCo simulation state
        self._sim_config: Optional[Dict[str, Any]] = None
        self._mujoco_model = None
        self._mujoco_data = None
        self._mujoco_bodies: Dict[str, int] = {}
        self._mujoco_channel = None
        self._mujoco_stub = None
        self._pose_stream_running = False
        self._pose_stream_thread = None
        self._pose_stream_lock = Lock()
        self._current_poses: Dict[str, Dict[str, Any]] = {}
        self._attach_to_mat = np.eye(4)
        self._session_id = f"avpstream_{int(time.time())}"
        
        # Isaac Lab simulation state
        self._isaac_stage = None
        self._isaac_bodies: Dict[str, str] = {}  # body_name -> prim_path
        self._isaac_config: Optional[Dict[str, Any]] = None
        
        # Sim benchmark state
        self._sim_benchmark_enabled = False
        self._sim_benchmark_seq = 0
        
        # Marker detection state
        self._detected_markers: Dict[int, Dict[str, Any]] = {}  # marker_id -> {"dict": int, "pose": np.ndarray}
        self._tracked_images: Dict[str, Dict[str, Any]] = {}  # image_id -> unified tracked image dict
        self._markers_lock = Lock()
        
        # Stylus tracking state
        self._stylus_data: Optional[Dict[str, Any]] = None  # Full stylus data dict
        self._stylus_lock = Lock()
        
        self._ice_servers = None  # Initialize ICE servers (populated in cross-network mode)
        self._relay_only = relay_only  # Force TURN relay only (for testing)
        
        # Register for cleanup on exit
        _active_streamers.append(self)

        # Start connection based on mode
        if self._cross_network_mode:
            self._log(f"[CROSS-NETWORK] Using room code: {self._room_code}", force=True)
            self._log(f"[CROSS-NETWORK] Signaling server: {self._signaling_url}", force=True)
            # Defer connection until stream() or start_webrtc() is called
            # This allows configure_video() / configure_audio() to be called first
        else:
            self._start_info_server()  # Start HTTP endpoint immediately
            self._start_hand_tracking()  # Start hand tracking stream
    
    def cleanup(self):
        """Clean up resources and notify VisionOS of disconnect.
        
        This method should be called before the Python process exits to ensure
        VisionOS receives a proper disconnect notification.
        """
        self._log("[CLEANUP] Cleaning up streamer resources...", force=True)
        
        # Send explicit leave message via signaling
        if self._cross_network_mode and self._signaling_ws and self._signaling_connected:
            try:
                self._send_signaling_message({"type": "leave"})
                self._log("[CLEANUP] Sent leave message to signaling server", force=True)
                # Give the message time to send
                time.sleep(0.1)
            except Exception as e:
                self._log(f"[CLEANUP] Error sending leave message: {e}", force=True)
        
        # Close WebSocket
        if self._signaling_ws:
            try:
                self._signaling_ws.close()
                self._log("[CLEANUP] Closed signaling WebSocket", force=True)
            except Exception as e:
                self._log(f"[CLEANUP] Error closing WebSocket: {e}", force=True)
        
        # Close WebRTC peer connection
        if hasattr(self, '_pc') and self._pc:
            try:
                if self._webrtc_loop:
                    asyncio.run_coroutine_threadsafe(self._pc.close(), self._webrtc_loop)
                self._log("[CLEANUP] Closed WebRTC peer connection", force=True)
            except Exception as e:
                self._log(f"[CLEANUP] Error closing WebRTC: {e}", force=True)
        
        # Remove from active streamers list
        if self in _active_streamers:
            _active_streamers.remove(self)
    
    def _log(self, message: str, force: bool = False):
        """Print a message if verbose mode is enabled or force is True.
        
        Args:
            message: The message to print
            force: If True, print regardless of verbose setting (for critical messages)
        """
        if self.verbose or force:
            print(message)

    def _process_hand_update(self, hand_update, source="grpc"):
        if getattr(hand_update.Head, "m00", 0.0) == 777.0:
            self._handle_benchmark_response(hand_update)
            return

        if source == "grpc" and self.ht_backend == "webrtc" and self._webrtc_hand_ready:
            # Prefer WebRTC data once channel is active.
            return

        try:
            # Base transforms in AVP coordinate frame
            left_wrist = self.axis_transform @ process_matrix(hand_update.left_hand.wristMatrix)
            right_wrist = self.axis_transform @ process_matrix(hand_update.right_hand.wristMatrix)
            head = rotate_head(self.axis_transform @ process_matrix(hand_update.Head))
            
            # If origin="sim", transform to simulation's attach_to frame
            if self.origin == "sim":
                inv_attach = np.linalg.inv(self._attach_to_mat)[np.newaxis, :, :]
                left_wrist = inv_attach @ left_wrist
                right_wrist = inv_attach @ right_wrist
                head = inv_attach @ head
            
            # Process skeleton joints (27 joints if new app, 25 if old app)
            left_fingers_full = process_matrices(hand_update.left_hand.skeleton.jointMatrices)
            right_fingers_full = process_matrices(hand_update.right_hand.skeleton.jointMatrices)
            
            # Check if we have the new 27-joint format or old 25-joint format
            # New ordering: [0-24] standard 25 joints, [25-26] forearm joints
            # This means left_arm[:25] == left_fingers for both formats
            has_forearm = left_fingers_full.shape[0] >= 27
            
            if has_forearm:
                # New 27-joint format: [wrist, thumb, index, ..., little, forearmWrist, forearmArm]
                # First 25 joints are the standard hand joints
                left_fingers_compat = left_fingers_full[:25]
                right_fingers_compat = right_fingers_full[:25]
            else:
                # Old 25-joint format: no forearm joints, use as-is
                left_fingers_compat = left_fingers_full
                right_fingers_compat = right_fingers_full
            
            transformations = {
                "left_wrist": left_wrist,
                "right_wrist": right_wrist,
                "left_fingers": left_fingers_compat,  # (25, 4, 4) always for backward compat
                "right_fingers": right_fingers_compat,  # (25, 4, 4) always for backward compat
                "left_arm": left_fingers_full,  # (27, 4, 4) if new app, (25, 4, 4) if old app
                "right_arm": right_fingers_full,  # (27, 4, 4) if new app, (25, 4, 4) if old app
                "head": head,
                "left_pinch_distance": get_pinch_distance(hand_update.left_hand.skeleton.jointMatrices),
                "right_pinch_distance": get_pinch_distance(hand_update.right_hand.skeleton.jointMatrices),
            }

            transformations["right_wrist_roll"] = get_wrist_roll(transformations["right_wrist"])
            transformations["left_wrist_roll"] = get_wrist_roll(transformations["left_wrist"])
            
            # Parse marker detection data from right hand skeleton
            # Format: After 27 joints, header matrix (m00=666.0, m01=count), then image pose matrices
            # Images can be ArUco markers (m30=0) or custom images (m30=1)
            right_joints = list(hand_update.right_hand.skeleton.jointMatrices)
            if len(right_joints) > 27:
                # Check for marker header
                header = right_joints[27]
                if abs(getattr(header, "m00", 0.0) - 666.0) < 0.1:
                    image_count = int(getattr(header, "m01", 0))
                    if image_count > 0 and len(right_joints) >= 28 + image_count:
                        new_markers = {}
                        new_tracked_images = {}
                        for i in range(image_count):
                            img_mat = right_joints[28 + i]
                            encoded = getattr(img_mat, "m33", 1.0)
                            image_type = getattr(img_mat, "m30", 0.0)  # 0=ArUco, 1=custom
                            
                            # Decode is_fixed flag from m32 (1.0 = fixed, 0.0 = not fixed)
                            is_fixed = getattr(img_mat, "m32", 0.0) > 0.5
                            # Decode is_tracked flag from m31 (1.0 = tracked, 0.0 = lost)
                            is_tracked = getattr(img_mat, "m31", 0.0) > 0.5
                            
                            # Apply YUP2ZUP transform (same as hand tracking)
                            pose = self.axis_transform @ process_matrix(img_mat)
                            # If origin="sim", also transform to simulation's attach_to frame
                            if self.origin == "sim":
                                inv_attach = np.linalg.inv(self._attach_to_mat)[np.newaxis, :, :]
                                pose = inv_attach @ pose
                            
                            if image_type < 0.5:  # ArUco marker
                                # Decode: m33 = 1000 + id + dict*100
                                if encoded > 900 and encoded < 2000:
                                    marker_id = int(encoded - 1000) % 100
                                    dict_type = int((encoded - 1000) // 100)
                                    new_markers[marker_id] = {
                                        "dict": dict_type,
                                        "pose": pose[0],  # (4,4) array
                                        "is_fixed": is_fixed,
                                        "is_tracked": is_tracked,
                                    }
                                    # Also add to tracked_images with unified format
                                    new_tracked_images[f"aruco_{dict_type}_{marker_id}"] = {
                                        "image_type": "aruco",
                                        "name": f"ArUco {dict_type}x{dict_type} #{marker_id}",
                                        "pose": pose[0],
                                        "is_fixed": is_fixed,
                                        "is_tracked": is_tracked,
                                        "aruco_id": marker_id,
                                        "aruco_dict": dict_type,
                                    }
                            else:  # Custom image
                                # Decode: m33 = 2000 + sequential_index
                                if encoded >= 2000:
                                    custom_index = int(encoded - 2000)
                                    image_id = f"custom_{custom_index}"
                                    new_tracked_images[image_id] = {
                                        "image_type": "custom",
                                        "name": f"Custom Image {custom_index}",
                                        "pose": pose[0],
                                        "is_fixed": is_fixed,
                                        "is_tracked": is_tracked,
                                    }
                        
                        with self._markers_lock:
                            self._detected_markers = new_markers
                            self._tracked_images = new_tracked_images
                    else:
                        with self._markers_lock:
                            self._detected_markers = {}
                            self._tracked_images = {}
            else:
                # No marker data in this update
                with self._markers_lock:
                    if self._detected_markers:
                        self._detected_markers = {}
                        self._tracked_images = {}
            
            # Parse stylus tracking data (after marker data)
            # Format: header matrix (m00=777.0, m01=count), then stylus pose matrices
            stylus_start_idx = 27  # Start after hand joints
            # Skip past marker data if present
            if len(right_joints) > 27:
                header = right_joints[27]
                if abs(getattr(header, "m00", 0.0) - 666.0) < 0.1:
                    marker_count = int(getattr(header, "m01", 0))
                    stylus_start_idx = 28 + marker_count  # Skip header + marker matrices
            
            # Check for stylus header
            if len(right_joints) > stylus_start_idx:
                stylus_header = right_joints[stylus_start_idx]
                if abs(getattr(stylus_header, "m00", 0.0) - 777.0) < 0.1:
                    stylus_count = int(getattr(stylus_header, "m01", 0))
                    if stylus_count > 0 and len(right_joints) > stylus_start_idx + 1:
                        stylus_mat = right_joints[stylus_start_idx + 1]
                        
                        # Extract button data from matrix fields
                        tip_pressure = getattr(stylus_mat, "m30", 0.0)
                        primary_pressure = getattr(stylus_mat, "m31", 0.0)
                        secondary_pressure = getattr(stylus_mat, "m32", 0.0)
                        flags_raw = getattr(stylus_mat, "m33", 1.0)
                        
                        # Decode button pressed states from flags
                        # Format: 1000 + (tip?1:0) + (primary?2:0) + (secondary?4:0) + timestamp_frac
                        if flags_raw >= 1000:
                            flags_int = int(flags_raw - 1000)
                            tip_pressed = bool(flags_int & 1)
                            primary_pressed = bool(flags_int & 2)
                            secondary_pressed = bool(flags_int & 4)
                        else:
                            tip_pressed = tip_pressure > 0.1
                            primary_pressed = primary_pressure > 0.1
                            secondary_pressed = secondary_pressure > 0.1
                        
                        # Apply YUP2ZUP transform (same as hand tracking)
                        pose = self.axis_transform @ process_matrix(stylus_mat)
                        # If origin="sim", also transform to simulation's attach_to frame
                        if self.origin == "sim":
                            inv_attach = np.linalg.inv(self._attach_to_mat)[np.newaxis, :, :]
                            pose = inv_attach @ pose
                        
                        stylus_data = {
                            "pose": pose[0],  # (4, 4) array
                            "tip_pressed": tip_pressed,
                            "tip_pressure": float(tip_pressure),
                            "primary_pressed": primary_pressed,
                            "primary_pressure": float(primary_pressure),
                            "secondary_pressed": secondary_pressed,
                            "secondary_pressure": float(secondary_pressure),
                        }
                        
                        with self._stylus_lock:
                            self._stylus_data = stylus_data
                    else:
                        with self._stylus_lock:
                            self._stylus_data = None
                else:
                    with self._stylus_lock:
                        if self._stylus_data is not None:
                            self._stylus_data = None
            else:
                with self._stylus_lock:
                    if self._stylus_data is not None:
                        self._stylus_data = None
        except Exception as exc:
            self._log(f"[HAND-TRACKING] Failed to process hand update from {source}: {exc}", force=True)
            return

        with self._latest_lock:
            if self.record:
                self.recording.append(transformations)
            self.latest = transformations
        
    def _handle_benchmark_response(self, hand_update):
        sequence_value = getattr(hand_update.Head, "m01", 0.0)
        if not math.isfinite(sequence_value):
            return

        sequence_id = int(round(sequence_value))
        sent_value = getattr(hand_update.Head, "m02", float("nan"))
        swift_value = getattr(hand_update.Head, "m03", float("nan"))

        sent_timestamp_ms = int(round(sent_value)) if math.isfinite(sent_value) else None
        swift_detected_ms = int(round(swift_value)) if math.isfinite(swift_value) else None
        python_receive_ms = int((time.perf_counter() - self._benchmark_epoch) * 1000)

        event = {
            "sequence_id": sequence_id,
            "sent_timestamp_ms": sent_timestamp_ms,
            "swift_detected_ms": swift_detected_ms,
            "python_receive_ms": python_receive_ms,
            "round_trip_ms": None if sent_timestamp_ms is None else python_receive_ms - sent_timestamp_ms,
            "source": "video"
        }

        # Use namespaced key to avoid collision with sim benchmark
        event_key = f"video:{sequence_id}"
        with self._benchmark_condition:
            self._benchmark_events[event_key] = event
            self._benchmark_condition.notify_all()

        if not self.benchmark_quiet:
            if event["round_trip_ms"] is not None:
                self._log(f"[BENCHMARK] seq={sequence_id} round-trip={event['round_trip_ms']} ms")
            else:
                self._log(f"[BENCHMARK] seq={sequence_id} received (missing sent timestamp)")

    def _handle_webrtc_hand_message(self, message):
        if isinstance(message, str):
            self._log("[WEBRTC] Warning: Received text message on hand data channel; ignoring")
            return

        if isinstance(message, memoryview):
            message = message.tobytes()

        update = handtracking_pb2.HandUpdate()
        try:
            update.ParseFromString(message)
        except Exception as exc:
            self._log(f"[WEBRTC] Error: Could not decode hand update from data channel: {exc}", force=True)
            return

        self._process_hand_update(update, source="webrtc")
        if not self._webrtc_hand_ready:
            self._log("[WEBRTC] Hand data flow established", force=True)
            self._webrtc_hand_ready = True

    def _register_webrtc_data_channel(self, channel):
        self._webrtc_hand_channel = channel

        @channel.on("open")
        def _on_open():
            self._log("[WEBRTC] Hand data channel opened", force=True)

        @channel.on("close")
        def _on_close():
            self._log("[WEBRTC] Hand data channel closed", force=True)
            self._webrtc_hand_ready = False
            self._webrtc_hand_channel = None

        @channel.on("message")
        def _on_message(message):
            self._handle_webrtc_hand_message(message)

    def _register_webrtc_sim_channel(self, channel):
        """Register the WebRTC data channel for simulation pose streaming."""
        self._webrtc_sim_channel = channel
        self._log(f"[WEBRTC] Registering sim-poses channel (readyState={channel.readyState})", force=True)

        @channel.on("open")
        def _on_open():
            self._log("[WEBRTC] Sim-poses data channel opened", force=True)
            self._webrtc_sim_ready = True
            
            # Start pose streaming with thread-based approach
            import threading
            
            def start_streaming_delayed():
                import time
                time.sleep(0.3)  # Small delay for channel stabilization
                
                # Check if we should wait for USDZ transfer to complete first
                wait_for_usdz = self._sim_config.get("wait_for_usdz_transfer", False) if self._sim_config else False
                if wait_for_usdz and self._cross_network_mode:
                    self._log("[WEBRTC] Waiting for USDZ transfer to complete before streaming poses...", force=True)
                    timeout = 300  # 5 minute timeout for large files
                    start_wait = time.time()
                    while not getattr(self, '_usdz_transfer_complete', False):
                        if time.time() - start_wait > timeout:
                            self._log("[WEBRTC] Warning: Timeout waiting for USDZ, starting pose streaming anyway", force=True)
                            break
                        time.sleep(0.5)
                    if getattr(self, '_usdz_transfer_complete', False):
                        self._log("[WEBRTC] USDZ transfer complete, starting pose streaming", force=True)
                
                self._log("[WEBRTC] Starting pose streaming...", force=True)
                if self._sim_config is not None and self._webrtc_sim_ready:
                    self._start_pose_streaming_webrtc()
            
            # Start in background thread to not block the event loop
            threading.Thread(target=start_streaming_delayed, daemon=True).start()

        @channel.on("close")
        def _on_close():
            self._log("[WEBRTC] Sim-poses data channel closed", force=True)
            self._webrtc_sim_ready = False
            self._webrtc_sim_channel = None

        @channel.on("message")
        def _on_message(message):
            # Handle benchmark echoes from VisionPro
            self._handle_sim_benchmark_message(message)

    def _handle_sim_benchmark_message(self, message):
        """Handle benchmark echo messages from the sim-poses channel."""
        try:
            if isinstance(message, str):
                data = json.loads(message)
            else:
                data = json.loads(message.decode('utf-8') if isinstance(message, bytes) else str(message))
            
            # Check for benchmark echo: {"b": {"s": sequence_id, "t": sent_timestamp_ms}}
            if "b" in data:
                bench = data["b"]
                sequence_id = bench.get("s")
                sent_timestamp_ms = bench.get("t")
                swift_detected_ms = bench.get("d")  # Optional: Swift's detection timestamp
                
                if sequence_id is not None and sent_timestamp_ms is not None:
                    python_receive_ms = int((time.perf_counter() - self._benchmark_epoch) * 1000)
                    
                    event = {
                        "sequence_id": sequence_id,
                        "sent_timestamp_ms": sent_timestamp_ms,
                        "swift_detected_ms": swift_detected_ms,
                        "python_receive_ms": python_receive_ms,
                        "round_trip_ms": python_receive_ms - sent_timestamp_ms,
                        "source": "sim-poses"
                    }
                    
                    # Use namespaced key to avoid collision with video benchmark
                    event_key = f"sim:{sequence_id}"
                    with self._benchmark_condition:
                        self._benchmark_events[event_key] = event
                        self._benchmark_condition.notify_all()
                    
                    if not self.benchmark_quiet:
                        self._log(f"[BENCHMARK] Sim seq={sequence_id} round-trip={event['round_trip_ms']} ms")
        except (json.JSONDecodeError, ValueError, TypeError):
            pass  # Not a benchmark message, ignore

    def _start_hand_tracking(self): 
        """Start the hand tracking gRPC stream in a background thread."""
        stream_thread = Thread(target = self.stream, daemon=True)
        stream_thread.start() 
        
        self._log(f"Hand tracking backend set to {self.ht_backend.upper()}")
        if self.ht_backend == "webrtc":
            self._log("Note: WebRTC hand tracking requires an active WebRTC session (start_streaming).")
        self._log('Waiting for hand tracking data...')
        retry_count = 0
        while self.latest is None: 
            time.sleep(0.1)
            retry_count += 1
            if retry_count > 100:  # 10 seconds timeout
                self._log('WARNING: No data received yet. Is the VisionOS app running?', force=True)
                retry_count = 0
        
        self._log(' == DATA IS FLOWING IN! ==', force=True)
        self._log('Ready to start streaming.') 

    def _ensure_cross_network_connected(self):
        """Ensure cross-network WebRTC connection is established (lazy initialization).
        
        Called automatically on first get_latest() in cross-network mode.
        This enables hand-tracking-only use cases without explicit start_webrtc() call.
        """
        if not self._cross_network_mode:
            return
        
        # Avoid re-entry
        if hasattr(self, '_cross_network_connecting') and self._cross_network_connecting:
            return
        self._cross_network_connecting = True
        
        try:
            # Step 1: Connect to signaling server if not connected
            if not self._signaling_connected:
                self._start_signaling_connection()
            
            # Step 2: Wait for VisionOS peer to be ready
            self._log('[CROSS-NETWORK] Waiting for VisionOS peer...', force=True)
            retry_count = 0
            while not self._peer_ready:
                time.sleep(0.1)
                retry_count += 1
                if retry_count > 100:  # 10 seconds
                    self._log('[CROSS-NETWORK] Still waiting for VisionOS to join the room...', force=True)
                    retry_count = 0
            
            # Step 3: Trigger WebRTC offer (hand-tracking only, no video/audio configured)
            self._trigger_webrtc_offer()
            
            # Step 4: Wait for hand tracking data to flow
            self._log('[CROSS-NETWORK] Waiting for hand tracking data...', force=True)
            retry_count = 0
            while not self._webrtc_hand_ready:
                time.sleep(0.1)
                retry_count += 1
                if retry_count > 100:  # 10 seconds
                    self._log('[CROSS-NETWORK] Still waiting for hand tracking channel...', force=True)
                    retry_count = 0
            
            self._log('[CROSS-NETWORK] Hand tracking connected!', force=True)
        finally:
            self._cross_network_connecting = False

    def _start_signaling_connection(self):
        """Start the cross-network signaling connection in a background thread.
        
        Note: This only establishes the signaling connection. The actual WebRTC
        offer creation happens later when serve()/start_webrtc() is called,
        ensuring video/audio configuration is complete before creating the offer.
        """
        signaling_thread = Thread(target=self._signaling_loop, daemon=True)
        signaling_thread.start()
        
        self._ice_servers = None  # Initialize ICE servers storage
        
        self._log('[CROSS-NETWORK] Connecting to signaling server...', force=True)
        
        # Wait for signaling connection
        retry_count = 0
        while not self._signaling_connected:
            time.sleep(0.1)
            retry_count += 1
            if retry_count > 100:  # 10 seconds timeout
                self._log('[CROSS-NETWORK] WARNING: Still waiting for signaling connection...', force=True)
                retry_count = 0
        
        self._log('[CROSS-NETWORK] Signaling connected.', force=True)
        self._log('[CROSS-NETWORK] Call configure_video()/configure_mujoco() then start_webrtc() to begin streaming.', force=True)


    def _signaling_loop(self):
        """Background thread that maintains the signaling server connection."""
        import websocket
        
        def on_open(ws):
            self._log('[SIGNALING] WebSocket connected', force=True)
            self._signaling_ws = ws
            self._signaling_connected = True
            
            # Join the room
            join_msg = json.dumps({
                "type": "join",
                "room": self._room_code,
                "role": "python"
            })
            ws.send(join_msg)
            self._log(f'[SIGNALING] Joining room: {self._room_code}')
            
            # Request ICE servers
            ice_req = json.dumps({"type": "request-ice"})
            ws.send(ice_req)
            self._log('[SIGNALING] Requested ICE servers')
        
        def on_message(ws, message):
            try:
                data = json.loads(message)
                msg_type = data.get("type")
                
                if msg_type == "joined":
                    peers_count = data.get("peersInRoom", 0)
                    self._log(f'[SIGNALING] Joined room with {peers_count} peer(s)', force=True)
                    
                    # If VisionOS is already in the room, mark it as ready
                    if peers_count > 1:
                        self._log('[SIGNALING] VisionOS already in room, ready for streaming', force=True)
                        self._peer_ready = True
                
                elif msg_type == "peer-joined":
                    peer_role = data.get("peerRole", "unknown")
                    self._log(f'[SIGNALING] Peer joined: {peer_role}', force=True)
                    # Mark peer as ready - offer will be created when serve() is called
                    self._peer_ready = True
                    self._log('[SIGNALING] Peer ready, waiting for start_webrtc() to initiate offer', force=True)
                
                elif msg_type == "peer-left":
                    self._log('[SIGNALING] Peer left the room', force=True)
                    self._webrtc_connected = False
                
                elif msg_type == "sdp":
                    sdp = data.get("sdp")
                    sdp_type = data.get("sdpType")
                    if sdp_type == "offer":
                        # We now create offers, so receiving one is unexpected
                        self._log('[SIGNALING] Warning: Received unexpected SDP offer (Python is offerer)', force=True)
                    elif sdp_type == "answer":
                        self._log('[SIGNALING] Received SDP answer from VisionOS', force=True)
                        self._handle_signaling_answer(sdp)
                
                elif msg_type == "ice":
                    candidate = data.get("candidate")
                    self._log(f'[SIGNALING] Received ICE candidate')
                    self._handle_signaling_ice_candidate(candidate)
                
                elif msg_type == "error":
                    self._log(f'[SIGNALING] Error: {data.get("message")}', force=True)
                
                elif msg_type == "ice-servers":
                    servers = data.get("servers")
                    self._log(f'[SIGNALING] Received {len(servers)} ICE servers')
                    # Debug: log the structure
                    for i, s in enumerate(servers):
                        self._log(f'[SIGNALING]   Server {i}: type={type(s).__name__}, value={s}')
                    self._ice_servers = servers
                
                elif msg_type == "pong":
                    pass  # Keepalive response
                    
            except json.JSONDecodeError:
                self._log(f'[SIGNALING] Invalid JSON message: {message}', force=True)
        
        def on_error(ws, error):
            self._log(f'[SIGNALING] WebSocket error: {error}', force=True)
            self._signaling_connected = False
        
        def on_close(ws, close_status_code, close_msg):
            self._log(f'[SIGNALING] WebSocket closed: {close_status_code} {close_msg}', force=True)
            self._signaling_connected = False
            self._signaling_ws = None
        
        # Connect to signaling server with retry
        while True:
            try:
                ws = websocket.WebSocketApp(
                    self._signaling_url,
                    on_open=on_open,
                    on_message=on_message,
                    on_error=on_error,
                    on_close=on_close
                )
                ws.run_forever(ping_interval=30, ping_timeout=10)
            except Exception as e:
                self._log(f'[SIGNALING] Connection error: {e}', force=True)
            
            # Retry after delay
            self._log('[SIGNALING] Reconnecting in 5 seconds...', force=True)
            time.sleep(5)

    async def _create_webrtc_offer_signaling(self):
        """Create WebRTC offer with video/audio/sim tracks for cross-network mode.
        
        Python creates the offer (offerer role) - mirrors local mode pattern.
        VisionOS will create the answer.
        """
        from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
        
        self._log('[WEBRTC] Creating WebRTC offer for cross-network mode...', force=True)
        
        # Create track factory classes with reference to this streamer
        ProcessedVideoTrack = _create_processed_video_track_class(self)
        ProcessedAudioTrack = _create_processed_audio_track_class(self)
        
        # Configure RTCPeerConnection with ICE servers (TURN for relay if needed)
        ice_servers = []
        if self._ice_servers:
            # _ice_servers can be:
            # 1. A list of server dicts: [{"urls": [...], "username": ..., "credential": ...}, ...]
            # 2. A single server dict: {"urls": [...], "username": ..., "credential": ...}
            
            servers_list = self._ice_servers
            # If it's a single dict (has 'urls' key), wrap it in a list
            if isinstance(servers_list, dict) and "urls" in servers_list:
                servers_list = [servers_list]
            
            self._log(f'[WEBRTC] Processing {len(servers_list)} ICE server(s) from signaling', force=True)
            
            for s in servers_list:
                if not isinstance(s, dict):
                    self._log(f'[WEBRTC] Warning: Unexpected ICE server format: {type(s)} - {s}', force=True)
                    continue
                    
                urls = s.get("urls", [])
                username = s.get("username")
                credential = s.get("credential")
                
                # urls can be a string or list
                if isinstance(urls, str):
                    urls = [urls]
                
                # Debug log credentials (truncated for security)
                if username:
                    self._log(f'[WEBRTC]   Credentials: username={username[:16]}..., credential={credential[:16] if credential else None}...', force=True)
                
                # Filter and categorize URLs
                stun_urls = []
                turn_urls = []
                for url in urls:
                    if not isinstance(url, str):
                        continue
                    # Skip port 53 URLs as they can cause timeouts
                    if ":53" in url:
                        continue
                    if url.startswith("stun:"):
                        stun_urls.append(url)
                    elif url.startswith("turn:") or url.startswith("turns:"):
                        turn_urls.append(url)
                
                # Add STUN server (no credentials needed)
                if stun_urls:
                    self._log(f'[WEBRTC]   Adding STUN: {stun_urls[0]}', force=True)
                    ice_servers.append(RTCIceServer(urls=stun_urls))
                
                # Add TURN server (with credentials)
                if turn_urls and username and credential:
                    self._log(f'[WEBRTC]   Adding TURN: {turn_urls[0][:60]}... ({len(turn_urls)} URLs)', force=True)
                    ice_servers.append(RTCIceServer(
                        urls=turn_urls,
                        username=username,
                        credential=credential
                    ))
        
        # Always add Google STUN as fallback
        ice_servers.append(RTCIceServer(urls=["stun:stun.l.google.com:19302"]))
        
        if len([s for s in ice_servers if any("turn:" in u or "turns:" in u for u in s.urls)]) == 0:
            self._log('[WEBRTC] Warning: No TURN servers available, using STUN only (may fail across NAT)', force=True)
        else:
            self._log(f'[WEBRTC] Configured {len(ice_servers)} ICE servers (including TURN)', force=True)
        
        # Apply relay-only mode if requested (forces TURN relay, no direct/STUN)
        if self._relay_only:
            self._log('[WEBRTC] ⚠️ RELAY-ONLY MODE: Forcing TURN relay (for testing)', force=True)
            from aioice.ice import Connection as AioiceConnection, TransportPolicy
            _original_init = AioiceConnection.__init__
            def _patched_init(self_conn, *args, **kwargs):
                kwargs['transport_policy'] = TransportPolicy.RELAY
                return _original_init(self_conn, *args, **kwargs)
            AioiceConnection.__init__ = _patched_init
        
        config = RTCConfiguration(iceServers=ice_servers)
        pc = RTCPeerConnection(configuration=config)
        self._pc = pc
        
        # Helper to log ICE state
        @pc.on("iceconnectionstatechange")
        async def on_ice_state_change():
            self._log(f"[WEBRTC] ICE state: {pc.iceConnectionState}", force=True)
            if pc.iceConnectionState == "connected" or pc.iceConnectionState == "completed":
                with self._webrtc_connection_condition:
                    self._webrtc_connected = True
                    self._webrtc_connection_condition.notify_all()
                # Log connection type (relay vs direct)
                try:
                    for transceiver in pc.getTransceivers():
                        transport = getattr(transceiver, '_transport', None)
                        if transport:
                            ice_transport = getattr(transport, '_transport', None)
                            if ice_transport and hasattr(ice_transport, '_connection'):
                                conn = ice_transport._connection
                                local = conn.local_candidate
                                remote = conn.remote_candidate
                                if local and remote:
                                    self._log(f"[WEBRTC] Connection path: local={local.type}({local.host}) <-> remote={remote.type}({remote.host})", force=True)
                except Exception as e:
                    pass  # Stats not available
            if pc.iceConnectionState in ("failed", "closed", "disconnected"):
                with self._webrtc_connection_condition:
                    self._webrtc_connected = False
        
        @pc.on("signalingstatechange")
        async def on_signaling_state_change():
            self._log(f"[WEBRTC] Signaling state: {pc.signalingState}", force=True)
        
        @pc.on("connectionstatechange")
        async def on_connection_state_change():
            self._log(f"[WEBRTC] Connection state: {pc.connectionState}", force=True)
        
        @pc.on("track")
        def on_track(track):
            self._log(f"[WEBRTC] Track received: {track.kind}")
        
        # Data Channels - Python creates them as offerer
        @pc.on("datachannel")
        def on_datachannel(channel):
            self._log(f"[WEBRTC] Data channel received: {channel.label}")
            if channel.label == "hand-tracking":
                self._register_webrtc_data_channel(channel)
            elif channel.label == "sim-poses":
                self._register_webrtc_sim_channel(channel)
        
        # Send local ICE candidates via signaling
        @pc.on("icecandidate")
        def on_icecandidate(candidate):
            if candidate:
                self._send_signaling_message({
                    "type": "ice",
                    "candidate": {
                        "candidate": candidate.candidate,
                        "sdpMid": candidate.sdpMid,
                        "sdpMLineIndex": candidate.sdpMLineIndex
                    }
                })
        
        # Create hand-tracking data channel
        hand_channel = pc.createDataChannel("hand-tracking", ordered=True)
        self._register_webrtc_data_channel(hand_channel)
        self._log("[WEBRTC] Created hand-tracking data channel")
        
        # Create sim-poses data channel if simulation is configured
        if self._sim_config is not None:
            sim_channel = pc.createDataChannel(
                "sim-poses",
                ordered=False,
                maxRetransmits=0,
            )
            self._register_webrtc_sim_channel(sim_channel)
            self._log("[WEBRTC] Created sim-poses data channel", force=True)
            
            # Create USDZ transfer channel for cross-network scene loading
            usdz_channel = pc.createDataChannel(
                "usdz-transfer",
                ordered=True,  # Must be ordered for file integrity
            )
            self._register_webrtc_usdz_channel(usdz_channel)
            self._log("[WEBRTC] Created usdz-transfer data channel", force=True)
        
        # Add video track if configured (mirrors local mode serve())
        if self._video_config is not None:
            video_cfg = self._video_config
            device = video_cfg.get("device")
            fmt = video_cfg.get("format")
            fps = video_cfg.get("fps", 30)
            width = video_cfg.get("width", 640)
            height = video_cfg.get("height", 480)
            size = video_cfg.get("size", "640x480")
            
            try:
                if device is None:
                    self._log("[VIDEO] Creating synthetic video stream for cross-network...", force=True)
                    if self.frame_callback is None:
                        self._log("[VIDEO] Warning: No frame callback registered. Stream will be blank.", force=True)
                else:
                    self._log(f"[VIDEO] Opening video device: {device}...", force=True)
                
                processed_track = ProcessedVideoTrack(
                    device, 
                    fmt, 
                    fps,
                    (width, height),
                    self.frame_callback
                )
                
                self.camera = processed_track
                
                if device is None:
                    self._log(f"[VIDEO] Synthetic video track created ({size} @ {fps}fps)", force=True)
                else:
                    self._log(f"[VIDEO] Camera track created for device: {device}", force=True)
                
                pc.addTrack(processed_track)
                self._log("[WEBRTC] Added video track to offer", force=True)
            except Exception as e:
                self._log(f"[VIDEO] Error creating video track: {e}", force=True)
                import traceback
                traceback.print_exc()
        
        # Add audio track if configured
        if self._audio_config is not None:
            audio_cfg = self._audio_config
            audio_device = audio_cfg.get("device")
            audio_format = audio_cfg.get("format")
            sample_rate = audio_cfg.get("sample_rate", 48000)
            stereo = audio_cfg.get("stereo", False)
            
            try:
                audio_track = ProcessedAudioTrack(
                    audio_device,
                    audio_format,
                    self.audio_callback,
                    stereo=stereo,
                    sample_rate=sample_rate
                )
                pc.addTrack(audio_track)
                self._log("[WEBRTC] Added audio track to offer", force=True)
            except Exception as e:
                self._log(f"[AUDIO] Error creating audio track: {e}", force=True)
        
        # Create Offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        
        # Gather config flags for visionOS
        video_cfg = self._video_config or {}
        audio_cfg = self._audio_config or {}
        stereo_video = video_cfg.get("stereo", False)
        stereo_audio = audio_cfg.get("stereo", False)
        video_enabled = self._video_config is not None
        audio_enabled = self._audio_config is not None
        sim_enabled = self._sim_config is not None
        
        # Debug: log the config being read
        self._log(f'[DEBUG] Video config at offer time: {self._video_config}', force=True)
        self._log(f'[DEBUG] stereo_video={stereo_video}, video_enabled={video_enabled}', force=True)
        
        # Send Offer via Signaling with config flags
        self._send_signaling_message({
            "type": "sdp",
            "sdp": pc.localDescription.sdp,
            "sdpType": "offer",
            "stereoVideo": stereo_video,
            "stereoAudio": stereo_audio,
            "videoEnabled": video_enabled,
            "audioEnabled": audio_enabled,
            "simEnabled": sim_enabled,
        })
        self._log(f'[SIGNALING] Sent SDP offer to VisionOS (stereoVideo={stereo_video}, stereoAudio={stereo_audio})', force=True)
        
        # Wait for and process the answer with timeout
        timeout_seconds = 30
        wait_interval = 0.1
        elapsed = 0.0
        last_log_time = 0.0
        
        while not hasattr(self, '_pending_answer') or self._pending_answer is None:
            await asyncio.sleep(wait_interval)
            elapsed += wait_interval
            
            # Log progress every 5 seconds
            if elapsed - last_log_time >= 5.0:
                self._log(f'[WEBRTC] Waiting for SDP answer from VisionOS... ({elapsed:.0f}s elapsed)', force=True)
                last_log_time = elapsed
            
            if elapsed >= timeout_seconds:
                self._log(f'[WEBRTC] Timeout waiting for SDP answer after {timeout_seconds}s', force=True)
                self._log('[WEBRTC] Hint: Make sure VisionOS app has "Cross-Network" mode enabled and room code matches', force=True)
                self._offer_in_progress = False
                return
        
        answer_sdp = self._pending_answer
        self._pending_answer = None
        
        self._log('[WEBRTC] Processing SDP answer from VisionOS...', force=True)
        answer = RTCSessionDescription(sdp=answer_sdp, type="answer")
        await pc.setRemoteDescription(answer)
        self._log('[WEBRTC] Remote description set successfully')
        
        # Process any pending ICE candidates
        if hasattr(self, '_pending_ice_candidates'):
            for candidate in self._pending_ice_candidates:
                await self._add_ice_candidate(candidate)
            self._pending_ice_candidates = []
        
        # Scene loading will be triggered when USDZ channel opens (see _register_webrtc_usdz_channel)


    async def _add_ice_candidate(self, candidate_dict):
        """Add an ICE candidate to the peer connection."""
        from aiortc import RTCIceCandidate
        from aiortc.sdp import candidate_from_sdp
        if not self._pc: return
        
        try:
            # Parse the candidate string from the dictionary
            cand_str = candidate_dict['candidate']
            sdp_mid = candidate_dict.get('sdpMid')
            sdp_mline_index = candidate_dict.get('sdpMLineIndex')
            
            # Use aiortc's parser to create the object from the candidate string
            # candidate_from_sdp returns a single RTCIceCandidate object
            candidate = candidate_from_sdp(cand_str)
            
            if candidate:
                candidate.sdpMid = sdp_mid
                candidate.sdpMLineIndex = sdp_mline_index
                await self._pc.addIceCandidate(candidate)
                self._log(f"[WEBRTC] Added ICE candidate: {candidate.type} {candidate.protocol} {candidate.ip}:{candidate.port}", force=True)
                
        except Exception as e:
            self._log(f"[WEBRTC] Error adding ICE candidate: {e}", force=True)

    def _trigger_webrtc_offer(self):
        """Trigger WebRTC offer creation in a background thread.
        
        Called when VisionOS peer joins or is already in the room.
        Python creates and sends the SDP offer.
        """
        # Avoid re-triggering if already connected or an offer is in progress
        if self._webrtc_connected:
            self._log('[WEBRTC] Already connected, skipping offer creation')
            return
        
        if hasattr(self, '_offer_in_progress') and self._offer_in_progress:
            self._log('[WEBRTC] Offer already in progress, skipping')
            return
        
        self._offer_in_progress = True
        
        # Wait for ICE servers before creating offer (needed for TURN relay)
        if self._ice_servers is None:
            self._log('[WEBRTC] Waiting for TURN servers from signaling...', force=True)
            wait_count = 0
            while self._ice_servers is None and wait_count < 50:  # 5 second timeout
                time.sleep(0.1)
                wait_count += 1
            if self._ice_servers is None:
                self._log('[WEBRTC] Warning: No TURN servers received, connection may fail across NAT', force=True)
            else:
                self._log(f'[WEBRTC] Got {len(self._ice_servers)} ICE servers (including TURN)', force=True)
        
        try:
            # Start a dedicated thread for the WebRTC loop if it doesn't exist
            if self._webrtc_thread is None or not self._webrtc_thread.is_alive():
                def run_webrtc_loop():
                    try:
                        loop = asyncio.new_event_loop()
                        asyncio.set_event_loop(loop)
                        self._webrtc_loop = loop
                        
                        # Custom exception handler for TURN channel bind errors
                        # These happen when TURN tries to bind to private IPs (which it can't relay to)
                        def handle_exception(loop, context):
                            exception = context.get('exception')
                            msg = context.get('message', '')
                            
                            # Handle known TURN channel bind 401 errors
                            if exception and 'TransactionFailed' in type(exception).__name__:
                                if '401' in str(exception):
                                    # Extract peer address from the task context if available
                                    # This happens when TURN server rejects channel binding to private IPs
                                    # (172.x.x.x, 192.168.x.x, 10.x.x.x) since it can't relay to them
                                    self._log(
                                        "[TURN] Channel bind rejected (401) - likely a private/host IP that TURN can't relay to. "
                                        "This is expected; connection will use public IP instead.",
                                        force=False  # Only show in verbose mode
                                    )
                                    return
                            # Log other exceptions normally
                            self._log(f"[ASYNCIO] Unhandled exception: {msg} - {exception}", force=True)
                        
                        loop.set_exception_handler(handle_exception)
                        loop.run_until_complete(self._create_webrtc_offer_signaling())
                        loop.run_forever()
                    except Exception as e:
                        self._log(f"[WEBRTC] Error in WebRTC loop: {e}", force=True)
                        import traceback
                        traceback.print_exc()
                    finally:
                        self._offer_in_progress = False
                
                self._webrtc_thread = Thread(target=run_webrtc_loop, daemon=True)
                self._webrtc_thread.start()
            else:
                # If loop exists, schedule it
                asyncio.run_coroutine_threadsafe(self._create_webrtc_offer_signaling(), self._webrtc_loop)
        except Exception as e:
            self._log(f"[SIGNALING] Error triggering WebRTC offer: {e}", force=True)
            self._offer_in_progress = False

    def _handle_signaling_answer(self, sdp: str):
        """Handle incoming SDP answer via signaling server."""
        if not hasattr(self, '_pending_answer'):
            self._pending_answer = None
        self._pending_answer = sdp
        self._log('[SIGNALING] Stored pending SDP answer')

    def _handle_signaling_ice_candidate(self, candidate: dict):
        """Handle incoming ICE candidate via signaling server."""
        if not hasattr(self, '_pending_ice_candidates'):
            self._pending_ice_candidates = []
        self._pending_ice_candidates.append(candidate)
        self._log(f'[SIGNALING] Stored ICE candidate (total: {len(self._pending_ice_candidates)})')

    def _send_signaling_message(self, message: dict):
        """Send a message to the signaling server."""
        if self._signaling_ws and self._signaling_connected:
            try:
                self._signaling_ws.send(json.dumps(message))
            except Exception as e:
                self._log(f'[SIGNALING] Failed to send message: {e}', force=True)
        else:
            self._log('[SIGNALING] Cannot send message: not connected', force=True)

    def stream(self): 

        # Create discovery request with our IP encoded
        request = handtracking_pb2.HandUpdate()
        local_ip = self.get_local_ip()
        ip_parts = local_ip.split('.')
        
        # Encode our IP in the request so VisionOS knows where we are
        request.Head.m00 = 888.0  # Discovery marker
        request.Head.m01 = float(ip_parts[0])
        request.Head.m02 = float(ip_parts[1])
        request.Head.m03 = float(ip_parts[2])
        request.Head.m10 = float(ip_parts[3])
        request.Head.m30 = float(LIBRARY_VERSION_CODE)  # Send library version for compatibility check
        
        # Retry connection with exponential backoff
        max_retries = 10
        retry_delay = 0.5
        
        for attempt in range(max_retries):
            try:
                self._log(f"Attempting to connect to {self.ip}:12345 (attempt {attempt + 1}/{max_retries})...")
                channel = grpc.insecure_channel(f"{self.ip}:12345")
                
                try:
                    # Wait for channel to be ready
                    self._log(f"  Waiting for channel to be ready (timeout: 5s)...")
                    grpc.channel_ready_future(channel).result(timeout=5)
                    self._log("[GRPC] Channel ready")
                except Exception as channel_error:
                    self._log(f"[GRPC] Warning: Channel not ready: {type(channel_error).__name__}: {channel_error}")
                    channel.close()
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        retry_delay *= 2
                    continue
                    
                self._log("  Creating stub and starting stream...")
                stub = handtracking_pb2_grpc.HandTrackingServiceStub(channel)
                
                # Add WebRTC info to metadata if available
                metadata = []
                if hasattr(self, '_webrtc_info_to_send'):
                    msg = self._webrtc_info_to_send
                    # Encode WebRTC info in metadata
                    webrtc_data = f"{msg.Head.m01}|{msg.Head.m02}|{msg.Head.m03}|{msg.Head.m10}|{msg.Head.m11}"
                    metadata.append(('webrtc-info', webrtc_data))
                    self._log(f"[GRPC] Adding WebRTC info to metadata: {webrtc_data}")
                
                responses = stub.StreamHandUpdates(request, metadata=metadata if metadata else None)
                self._log("[GRPC] Stream established, waiting for responses...")
                
                for response in responses:
                    self._process_hand_update(response, source="grpc")
                
                # If we get here, connection was successful
                channel.close()
                return
                    
            except grpc.RpcError as e:
                if attempt < max_retries - 1:
                    self._log(f"[GRPC] Connection failed: {e.code()}. Retrying in {retry_delay}s...")
                    time.sleep(retry_delay)
                    retry_delay *= 2  # Exponential backoff
                else:
                    self._log(f"[GRPC] Failed to connect after {max_retries} attempts", force=True)
                    self._log(f"Error: {e}", force=True)
                    self._log("\nMake sure:", force=True)
                    self._log("  1. VisionOS app is running", force=True)
                    self._log("  2. You pressed 'Start' in the app", force=True)
                    self._log(f"  3. Vision Pro IP ({self.ip}) is correct", force=True)
                    self._log("  4. Both devices are on the same network", force=True)
                    break
            except Exception as e:
                error_type = type(e).__name__
                error_msg = str(e) if str(e) else "(no error message)"
                self._log(f"[GRPC] Unexpected error: {error_type}: {error_msg}", force=True)
                if self.verbose:
                    import traceback
                    traceback.print_exc()
                if attempt < max_retries - 1:
                    self._log(f"  Retrying in {retry_delay}s...")
                    time.sleep(retry_delay)
                    retry_delay *= 2
                else:
                    break 

    def get_latest(self, use_cache=False, cache_ms=10) -> Optional[TrackingData]: 
        """Return the most recent tracking sample as a TrackingData object.

        The returned TrackingData is fully backward compatible with dict access:
        
        Dict-style access (backward compatible)::
        
            data["head"]           # (1, 4, 4) head pose
            data["right_wrist"]    # (1, 4, 4) wrist pose  
            data["right_fingers"]  # (25, 4, 4) finger joints
            data["right_arm"]      # (27, 4, 4) full arm skeleton
            data.keys()            # Works like a dict
        
        New attribute access::
        
            data.head             # (4, 4) head pose (squeezed from 1,4,4)
            data.right            # (27, 4, 4) HandData array
            data.right.indexTip   # (4, 4) single joint by name
            data.right[9]         # Same as indexTip by index
            data.right.pinch_distance  # Pinch distance
        
        Joint names for attribute access:
            wrist(0), thumbKnuckle(1), thumbIntermediateBase(2), thumbIntermediateTip(3), thumbTip(4),
            indexMetacarpal(5), indexKnuckle(6), indexIntermediateBase(7), indexIntermediateTip(8), indexTip(9),
            middleMetacarpal(10), ..., littleTip(24), forearmWrist(25), forearmArm(26)

        :param use_cache: If True, return cached data if recent enough (reduces lock contention)
        :param cache_ms: Cache validity in milliseconds (default: 10ms)
        :return: TrackingData object or None if not yet available.
        :rtype: TrackingData | None

        Example::
        
            streamer = VisionProStreamer(ip=avp_ip)
            data = streamer.get_latest()
            if data:
                # Old API (still works)
                left_pos = data["left_wrist"][0, :3, 3]
                
                # New API
                left_pos = data.left.wrist[:3, 3]
                index_tip = data.right.indexTip  # 4x4 matrix
                pinch = data.right.pinch_distance
        """
        # Lazy connection for cross-network mode
        if self._cross_network_mode and not self._webrtc_connected:
            self._ensure_cross_network_connected()
        
        raw_data = None
        if use_cache:
            current_time = time.perf_counter()
            if (self._latest_cached is not None and 
                (current_time - self._latest_cache_time) < (cache_ms / 1000.0)):
                raw_data = self._latest_cached
            else:
                # Cache expired, update it
                with self._latest_lock:
                    self._latest_cached = self.latest
                    self._latest_cache_time = current_time
                    raw_data = self._latest_cached
        else:
            with self._latest_lock:
                raw_data = self.latest
        
        if raw_data is None:
            return None
        return TrackingData(raw_data)
        
    def get_recording(self): 
        """Return a copy of all recorded tracking samples (requires `record=True`).

        Each element is a dict identical to `get_latest()`. Useful for offline analysis or saving to disk.

        Example
        -------
        .. code-block:: python

            streamer = VisionProStreamer(ip=avp_ip, record=True)
            # ... run for a while
            samples = streamer.get_recording()
            print(f"Collected {len(samples)} frames")
        """
        with self._latest_lock:
            return list(self.recording)
    
    def get_markers(self) -> Dict[int, Dict[str, Any]]:
        """Get currently detected ArUco markers.
        
        Returns a dict mapping marker_id to marker info. Each marker includes:
        - ``dict``: ArUco dictionary type (int, e.g., 0 = DICT_4X4_50)
        - ``pose``: 4x4 homogeneous transformation matrix (np.ndarray)
        - ``is_fixed``: Whether the marker pose is frozen (bool). When True,
          the pose won't update from tracking - useful for calibration.
        - ``is_tracked``: Whether ARKit is actively tracking this marker (bool).
          A marker can be fixed but not tracked (using saved pose), or
          tracked but not fixed (live updates).
        
        The pose is in the same coordinate frame as hand tracking data
        (affected by `set_origin()` setting).
        
        Marker detection must be enabled on the VisionOS app for this to return data.
        
        Returns:
            Dict mapping marker_id to {"dict": int, "pose": np.ndarray(4,4), 
                                        "is_fixed": bool, "is_tracked": bool}
            Empty dict if no markers detected or detection disabled.
        
        Example::
        
            markers = streamer.get_markers()
            if markers:
                for marker_id, info in markers.items():
                    pose = info["pose"]  # 4x4 transform
                    position = pose[:3, 3]  # XYZ position
                    is_fixed = info["is_fixed"]  # Whether pose is frozen
                    is_tracked = info["is_tracked"]  # Whether actively tracked
                    print(f"Marker {marker_id}: fixed={is_fixed}, tracked={is_tracked}")
        """
        with self._markers_lock:
            return dict(self._detected_markers)
    
    def get_tracked_images(self) -> Dict[str, Dict[str, Any]]:
        """Get all tracked images (ArUco markers + custom images).
        
        Returns a unified dict mapping image_id to image info. Each image includes:
        - ``image_type``: "aruco" or "custom"
        - ``name``: Display name (e.g., "ArUco 4x4 #5" or "Custom Image 0")
        - ``pose``: 4x4 homogeneous transformation matrix (np.ndarray)
        - ``is_fixed``: Whether the pose is frozen (bool)
        - ``is_tracked``: Whether ARKit is actively tracking (bool)
        - ``aruco_id``: Marker ID (only for ArUco markers)
        - ``aruco_dict``: Dictionary type (only for ArUco markers)
        
        Image IDs are formatted as:
        - ArUco markers: "aruco_{dict}_{id}" (e.g., "aruco_0_5")
        - Custom images: "custom_{index}" (e.g., "custom_0")
        
        The pose is in the same coordinate frame as hand tracking data
        (affected by `set_origin()` setting).
        
        Returns:
            Dict mapping image_id to tracked image info.
            Empty dict if no images detected or detection disabled.
        
        Example::
        
            images = streamer.get_tracked_images()
            for image_id, info in images.items():
                print(f"{info['name']}: type={info['image_type']}, tracked={info['is_tracked']}")
                position = info["pose"][:3, 3]
        """
        with self._markers_lock:
            return dict(self._tracked_images)
    
    def get_stylus(self) -> Optional[Dict[str, Any]]:
        """Get the current stylus (Apple Pencil Pro) state.
        
        Returns a dict with the tracked spatial stylus data including pose,
        button states, and pressure values. Stylus tracking must be enabled
        on the VisionOS app (Settings > Stylus Tracking) and requires visionOS 26.0+.
        
        The pose is in the same coordinate frame as hand tracking data
        (affected by `set_origin()` setting).
        
        Returns:
            Dict with stylus data if tracked, None if no stylus or tracking disabled.
            Keys:
            - ``pose``: (4, 4) homogeneous transformation matrix
            - ``tip_pressed``: bool - whether tip is pressed
            - ``tip_pressure``: float (0.0-1.0) - tip pressure
            - ``primary_pressed``: bool - whether primary button is pressed
            - ``primary_pressure``: float (0.0-1.0) - primary button pressure
            - ``secondary_pressed``: bool - whether secondary button is pressed
            - ``secondary_pressure``: float (0.0-1.0) - secondary button pressure
        
        Example::
        
            stylus = streamer.get_stylus()
            if stylus is not None:
                pose = stylus["pose"]  # 4x4 matrix
                position = pose[:3, 3]  # XYZ position
                if stylus["tip_pressed"]:
                    pressure = stylus["tip_pressure"]
                    print(f"Drawing at {position} with pressure {pressure:.2f}")
        """
        with self._stylus_lock:
            if self._stylus_data is None:
                return None
            # Return a copy with pose copied
            return {
                "pose": self._stylus_data["pose"].copy(),
                "tip_pressed": self._stylus_data["tip_pressed"],
                "tip_pressure": self._stylus_data["tip_pressure"],
                "primary_pressed": self._stylus_data["primary_pressed"],
                "primary_pressure": self._stylus_data["primary_pressure"],
                "secondary_pressed": self._stylus_data["secondary_pressed"],
                "secondary_pressure": self._stylus_data["secondary_pressure"],
            }
    
    def set_origin(self, origin: str):
        """Set the coordinate frame origin for hand tracking data.
        
        Args:
            origin: Either "avp" or "sim".
                - "avp": Hand tracking in Vision Pro's native coordinate frame.
                - "sim": Hand tracking relative to the simulation's attach_to position.
                  Useful when you want hand positions in the same frame as your MuJoCo scene.
        
        Example::
        
            streamer = VisionProStreamer(ip=avp_ip)
            streamer.configure_mujoco("scene.xml", model, data, relative_to=[0, 0, 0.8, 90])
            streamer.set_origin("sim")  # Now hand tracking is in simulation frame
            streamer.start_webrtc()
        """
        origin = origin.lower()
        if origin not in {"avp", "sim"}:
            raise ValueError(f"Unsupported origin '{origin}'. Expected 'avp' or 'sim'.")
        self.origin = origin
        self._log(f"[CONFIG] Origin set to '{origin}'")
    
    def get_sync_timestamp(self) -> Dict[str, Any]:
        """Get synchronized timestamp for use in audio/video callbacks.
        
        This method provides a common time reference that both audio and video
        callbacks can use to ensure synchronized content generation. Use this
        when you need to generate audio that corresponds to specific video frames.
        
        Returns:
            dict with:
                - elapsed_s: Seconds since streaming started
                - elapsed_ms: Milliseconds since streaming started  
                - epoch_ntp: NTP timestamp when streaming started
                - current_ntp: Current NTP timestamp
                - is_started: Whether streaming has begun
        
        Example (in audio callback)::
        
            def my_audio_callback(audio_frame):
                sync = streamer.get_sync_timestamp()
                elapsed_ms = sync["elapsed_ms"]
                # Generate audio based on the synchronized time
                return audio_frame
        
        Example (coordinating audio/video)::
        
            # Both callbacks see the same elapsed time
            def video_cb(frame):
                t = streamer.get_sync_timestamp()["elapsed_ms"]
                # Draw based on t
                return frame
                
            def audio_cb(frame):
                t = streamer.get_sync_timestamp()["elapsed_ms"]
                # Generate tone based on t
                return frame
        """
        info = self._media_clock.get_sync_timestamp()
        info["is_started"] = self._media_clock.is_started
        return info
    
    def get_local_ip(self):
        """Get the local IP address of this machine"""
        try:
            # Create a socket to find out our local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Use the Vision Pro IP to determine the correct local interface
            # This ensures we pick the network interface that can reach the Vision Pro
            target_ip = self.ip if self.ip else "8.8.8.8"
            s.connect((target_ip, 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"
    
    def register_frame_callback(self, callback):
        """
        Register a callback function to process video frames before streaming.
        
        Args:
            callback: A function that takes a numpy array (frame in BGR24 format) 
                     and returns a processed numpy array of the same shape.
                     Example: lambda frame: cv2.putText(frame, "Hello", (10,30), ...)
        
        Examples
        --------
        Camera overlay (``examples/04_process_frames.py``)::

            def add_text(frame):
                return cv2.putText(frame, "Hello VisionPro!", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                   (0,255,0), 2)
            streamer.register_frame_callback(add_text)
            streamer.start_streaming(device="0:none", format="avfoundation")

        Synthetic visualization (``examples/01_visualize_hand_callback.py``)::

            def hand_visualizer(blank):
                latest = streamer.get_latest()
                # draw wrists/fingers on blank
                return blank
            streamer.register_frame_callback(hand_visualizer)
            streamer.start_streaming(device=None, size="1280x720", fps=60)

        Stereo depth visualization (``examples/07_stereo_depth_visualization.py``)::

            streamer.register_frame_callback(create_stereo_depth_visualizer(streamer, disparity_scale=100))
            streamer.start_streaming(device=None, stereo_video=True, size="2000x1000")
        """
        self.frame_callback = callback
        self._log(f"[CONFIG] Frame callback registered: {callback.__name__ if hasattr(callback, '__name__') else 'anonymous function'}")
    
    def register_audio_callback(self, callback):
        """
        Register a callback function to process audio frames before streaming.
        
        Args:
            callback: A function that takes an AudioFrame and returns a processed AudioFrame.
                     The AudioFrame has properties: samples (numpy array), sample_rate, channels.
        
        Examples
        --------
        Microphone processing (simple gain)::

            def boost(frame):
                frame.samples = frame.samples * 1.2
                return frame
            streamer.register_audio_callback(boost)
            streamer.start_streaming(audio_device=":0")

        Pinch-triggered beeps (``examples/01_visualize_hand_with_audio_callback.py``)::

            streamer.register_audio_callback(beep_audio_generator(streamer))
            streamer.start_streaming(device=None, audio_device=None)

        Looping audio file (``examples/06_stream_audio_file.py``)::

            streamer.register_audio_callback(audio_file_streamer("music.mp3", stereo=True))
            streamer.start_streaming(device=None, stereo_audio=True)
        """
        self.audio_callback = callback
        self._log(f"[CONFIG] Audio callback registered: {callback.__name__ if hasattr(callback, '__name__') else 'anonymous function'}")
    
    def update_frame(self, user_frame):
        """
        Override the camera frame with a custom frame. The frame will still go through
        any registered callbacks before being sent to VisionPro.
        
        Args:
            user_frame: A numpy array in BGR24 format (H, W, 3) to send instead of camera input.
                       Should match the resolution specified in start_streaming().
        
        Examples
        --------
        Single override while camera running::

            frame = np.zeros((480,640,3), dtype=np.uint8); frame[:] = (0,255,0)
            streamer.update_frame(frame)

        Continuous manual generation (``examples/01_visualize_hand_direct.py``)::

            while True:
                frame = generate_hand_visualization(streamer, width=1280, height=720)
                streamer.update_frame(frame)
                time.sleep(1/60.)
        """
        self.user_frame = user_frame

    # ============================================================================
    # Modular Configuration API
    # ============================================================================
    
    def configure_video(
        self,
        device: Optional[str] = None,
        format: Optional[str] = None,
        size: str = "640x480",
        fps: int = 30,
        stereo: bool = False,
    ):
        """
        Configure video streaming parameters. Call this before serve().
        
        This stores video configuration. The actual video track is created
        per-WebRTC connection when serve() is called.
        
        Args:
            device: Video device identifier (default: None for synthetic frames).
                   Use "0:none" for macOS camera via avfoundation.
            format: Video format (default: "avfoundation" for macOS).
                   Ignored if device=None.
            size: Frame size as "WIDTHxHEIGHT" string (default: "640x480")
            fps: Frame rate (default: 30)
            stereo: If True, stream side-by-side stereo video (default: False)
        
        Example::
        
            streamer.configure_video(device=None, size="1280x720", fps=60)
            streamer.configure_audio()  # optional
            streamer.start_webrtc()
        """
        # Parse size
        width, height = map(int, size.split('x'))
        
        # Store configuration
        self._video_config = {
            "device": device,
            "format": format,
            "size": size,
            "fps": fps,
            "stereo": stereo,
            "width": width,
            "height": height,
        }
        
        self._log(f"[CONFIG] Video configured: {size} @ {fps}fps (device={device}, stereo={stereo})")
    
    def configure_audio(
        self,
        device: Optional[str] = None,
        format: Optional[str] = None,
        sample_rate: Optional[int] = None,
        stereo: bool = False,
    ):
        """
        Configure audio streaming parameters. Call this before serve().
        
        This stores audio configuration. The actual audio track is created
        per-WebRTC connection when serve() is called.
        
        Args:
            device: Audio device identifier (default: None for synthetic audio).
                   Use ":0" for default microphone on macOS via avfoundation.
            format: Audio input format (default: "avfoundation" for macOS).
            sample_rate: Sample rate in Hz (default: 48000)
            stereo: If True, stream stereo audio (2 channels), otherwise mono (default: False)
        
        Example::
        
            streamer.configure_video(device=None, size="1280x720")
            streamer.configure_audio(stereo=True)  # synthetic stereo audio
            streamer.start_webrtc()
        """
        effective_sample_rate = sample_rate or 48000
        
        # Store configuration
        self._audio_config = {
            "device": device,
            "format": format,
            "sample_rate": effective_sample_rate,
            "stereo": stereo,
        }
        
        self._log(f"[CONFIG] Audio configured: {effective_sample_rate}Hz (device={device}, stereo={stereo})")
    
    def configure_mujoco(
        self,
        xml_path: str,
        model,
        data,
        relative_to: Optional[List[float]] = None,
        grpc_port: int = 50051,
        force_reload: bool = False,
        streaming_hz: int = 120,
        wait_for_usdz_transfer: bool = False,
    ):
        """
        Configure MuJoCo simulation streaming. Call this before serve().
        
        This enables streaming MuJoCo simulation states to the VisionPro, displaying
        a 3D model that updates in real-time as the simulation runs.
        
        Args:
            xml_path: Path to the MuJoCo XML scene file
            model: MuJoCo model object (mujoco.MjModel)
            data: MuJoCo data object (mujoco.MjData)
            relative_to: 7-element array [x, y, z, qw, qx, qy, qz] specifying position 
                        and quaternion (wxyz order) in Z-up coordinates for scene placement.
                        Or 4-element array [x, y, z, yaw_degrees] for simpler positioning.
            grpc_port: gRPC port for MuJoCo AR communication (default: 50051)
            force_reload: If True, force re-conversion of XML to USDZ even if cached
            streaming_hz: Rate limit for pose streaming in Hz (default: 120)
            wait_for_usdz_transfer: If True, delay pose streaming until USDZ transfer
                        completes. Useful for large scenes where you want the 3D model
                        to be fully loaded before poses start animating. (default: False)
        
        Example::
        
            import mujoco
            model = mujoco.MjModel.from_xml_path("scene.xml")
            data = mujoco.MjData(model)
            
            streamer.configure_mujoco("scene.xml", model, data, relative_to=[0, 0, 1, 1, 0, 0, 0])
            streamer.start_webrtc()
            
            while True:
                mujoco.mj_step(model, data)
                streamer.update_sim()
        """
        
        # Process relative_to parameter
        attach_to = None
        if relative_to is not None:
            if len(relative_to) == 4:
                # [x, y, z, yaw_degrees] -> convert to [x, y, z, qw, qx, qy, qz]
                yaw_deg = relative_to[3]
                yaw_rad = np.radians(yaw_deg / 2)
                qw = np.cos(yaw_rad)
                qx = 0.0
                qy = 0.0
                qz = np.sin(yaw_rad)
                attach_to = [relative_to[0], relative_to[1], relative_to[2], qw, qx, qy, qz]
            elif len(relative_to) == 7:
                attach_to = list(relative_to)
            else:
                raise ValueError(f"relative_to must be 4 or 7 elements, got {len(relative_to)}")
            
            # Build transformation matrix
            self._attach_to_mat = np.eye(4)
            self._attach_to_mat[:3, :3] = R.from_quat(attach_to[3:], scalar_first=True).as_matrix()
            self._attach_to_mat[:3, 3] = attach_to[:3]
        
        # Register model and data
        self._mujoco_model = model
        self._mujoco_data = data
        
        # Build body name mapping and cache cleaned names for streaming
        self._mujoco_bodies = {}
        self._mujoco_clean_names = {}  # Cache: body_name -> cleaned name (no string ops per frame)
        for i in range(model.nbody):
            body_name = model.body(i).name
            if body_name:
                self._mujoco_bodies[body_name] = i
                # Pre-compute cleaned name to avoid string operations every frame
                clean_name = body_name.replace('/', '').replace('-', '') if body_name else body_name
                self._mujoco_clean_names[body_name] = clean_name
        
        self._sim_config = {
            "xml_path": xml_path,
            "attach_to": attach_to,
            "grpc_port": grpc_port,
            "force_reload": force_reload,
            "streaming_hz": streaming_hz,
            "wait_for_usdz_transfer": wait_for_usdz_transfer,
        }
        
        # Automatically switch to simulation-relative coordinates
        self.set_origin("sim")
        
        self._log(f"[CONFIG] Simulation configured: {xml_path} ({len(self._mujoco_bodies)} bodies)")
        if attach_to:
            self._log(f"  Position: [{attach_to[0]:.2f}, {attach_to[1]:.2f}, {attach_to[2]:.2f}]")
    
    # Backward compatibility alias
    configure_sim = configure_mujoco
    
    def configure_isaac(
        self,
        scene,
        relative_to: Optional[List[float]] = None,
        include_ground: bool = False,
        env_indices: Optional[List[int]] = None,
        grpc_port: int = 50051,
        stage = None,  # Optional, derived from scene if not provided
        streaming_hz: int = 120,
        force_reload: bool = False,
        wait_for_usdz_transfer: bool = False,
    ):
        """
        Configure Isaac Lab simulation streaming. Call this before start_webrtc().
        
        This enables streaming Isaac Lab simulation states to the VisionPro, displaying
        a 3D scene that updates in real-time as the simulation runs.
        
        Args:
            scene: Isaac Lab InteractiveScene object. Required for real-time pose updates
                   and USD stage access.
            relative_to: 4-element array [x, y, z, yaw_degrees] or 7-element array
                        [x, y, z, qw, qx, qy, qz] specifying position and orientation
                        in Z-up coordinates for scene placement.
            include_ground: Whether to include ground plane in USDZ export (default: False)
            env_indices: List of environment indices to include (default: None = all)
                        e.g., [0, 1, 2] will include env_0, env_1, env_2
            grpc_port: gRPC port for USDZ transfer (default: 50051)
            stage: (Deprecated) USD stage, automatically derived from scene.
            force_reload: If True, force re-export of USDZ even if cached (default: False)
            streaming_hz: Rate limit for pose streaming in Hz (default: 120)
            wait_for_usdz_transfer: If True, delay pose streaming until USDZ transfer
                        completes. Useful for large scenes where you want the 3D model
                        to be fully loaded before poses start animating. (default: False)
        
        Example::
        
            streamer.configure_isaac(
                scene=my_env.scene,
                relative_to=[0, 0, 0.8, 90],
                include_ground=False,
                env_indices=[0],
            )
            streamer.start_webrtc()
            
            while simulation_app.is_running():
                sim.step()
                my_env.scene.update()
                streamer.update_sim()
        """
        import numpy as np
        
        # Get USD stage from scene or use provided stage
        if stage is None:
            # Derive stage from Isaac Lab scene
            try:
                import omni.usd
                stage = omni.usd.get_context().get_stage()
                self._log("[ISAAC] Stage derived from omni.usd context")
            except Exception as e:
                raise ValueError(f"Could not get USD stage: {e}. Make sure Isaac Sim is initialized.")
        
        # Process relative_to parameter (same logic as configure_mujoco)
        attach_to = None
        if relative_to is not None:
            if len(relative_to) == 4:
                # [x, y, z, yaw_degrees] -> convert to [x, y, z, qw, qx, qy, qz]
                yaw_deg = relative_to[3]
                yaw_rad = np.radians(yaw_deg / 2)
                qw = np.cos(yaw_rad)
                qx = 0.0
                qy = 0.0
                qz = np.sin(yaw_rad)
                attach_to = [relative_to[0], relative_to[1], relative_to[2], qw, qx, qy, qz]
            elif len(relative_to) == 7:
                attach_to = list(relative_to)
            else:
                raise ValueError("relative_to must be [x, y, z, yaw_degrees] or [x, y, z, qw, qx, qy, qz]")
            
            # Build transformation matrix
            self._attach_to_mat = np.eye(4)
            self._attach_to_mat[:3, :3] = R.from_quat(attach_to[3:], scalar_first=True).as_matrix()
            self._attach_to_mat[:3, 3] = attach_to[:3]
        
        # Store the Isaac stage and scene
        self._isaac_stage = stage
        self._isaac_scene = scene
        
        # Extract body information from stage (for USDZ export naming)
        self._isaac_bodies = self._get_isaac_bodies(stage, env_indices)
        
        # Build articulation body mappings if scene provided
        self._isaac_articulations = {}
        self._isaac_static_assets = {}  # Static assets (AssetBaseCfg) - sent only first few frames
        self._isaac_static_frames_sent = 0  # Counter for static asset frames
        self._isaac_static_frames_limit = 999999  # DEBUG: Send static poses always (was 5)
        if scene is not None:
            self._isaac_articulations = self._get_isaac_articulations(scene)
            self._isaac_static_assets = self._get_isaac_static_assets(scene, env_indices)
            self._log(f"[ISAAC] Found {len(self._isaac_articulations)} articulations with bodies")
            self._log(f"[ISAAC] Found {len(self._isaac_static_assets)} static assets")
        
        # Store Isaac config (used by _load_and_send_scene)
        self._isaac_config = {
            "include_ground": include_ground,
            "env_indices": env_indices,
            "attach_to": attach_to,
            "grpc_port": grpc_port,
            "force_reload": force_reload,
        }
        
        # Also set _sim_config so the streaming infrastructure knows sim is enabled
        self._sim_config = {
            "type": "isaac",
            "attach_to": attach_to,
            "grpc_port": grpc_port,
            "streaming_hz": streaming_hz,
            "wait_for_usdz_transfer": wait_for_usdz_transfer,
        }
        
        # Automatically switch to simulation-relative coordinates
        self.set_origin("sim")
        
        self._log(f"[CONFIG] Isaac Lab configured: {len(self._isaac_bodies)} bodies, {len(self._isaac_articulations)} articulations")
        if attach_to:
            self._log(f"  Position: [{attach_to[0]:.2f}, {attach_to[1]:.2f}, {attach_to[2]:.2f}]")
    
    def _get_isaac_articulations(self, scene) -> Dict[str, Any]:
        """
        Get articulation objects from Isaac Lab scene for querying body poses.
        
        Returns dict mapping articulation name to (articulation, body_names) tuple.
        """
        articulations = {}
        
        # Isaac Lab InteractiveScene uses dictionary-style access: scene["Franka"]
        # Try multiple methods to get asset keys
        asset_keys = []
        
        # Method 1: scene.keys() (if it behaves like a dict)
        if hasattr(scene, 'keys') and callable(getattr(scene, 'keys')):
            try:
                asset_keys = list(scene.keys())
                self._log(f"[ISAAC] Found keys via scene.keys(): {asset_keys}")
            except Exception as e:
                self._log(f"[ISAAC] scene.keys() failed: {e}")
        
        # Method 2: scene.articulations dict
        if not asset_keys and hasattr(scene, 'articulations'):
            try:
                asset_keys = list(scene.articulations.keys())
                self._log(f"[ISAAC] Found keys via scene.articulations: {asset_keys}")
            except Exception as e:
                self._log(f"[ISAAC] scene.articulations failed: {e}")
        
        # Method 3: scene._all_assets (internal dict for all assets)
        if not asset_keys and hasattr(scene, '_all_assets'):
            try:
                asset_keys = list(scene._all_assets.keys())
                self._log(f"[ISAAC] Found keys via scene._all_assets: {asset_keys}")
            except Exception as e:
                self._log(f"[ISAAC] scene._all_assets failed: {e}")
        
        # Method 4: Known articulation names from config
        if not asset_keys:
            # Try common names
            for name in ['Franka', 'Robot', 'robot', 'arm']:
                try:
                    asset = scene[name]
                    if asset is not None:
                        asset_keys.append(name)
                except Exception:
                    pass
            if asset_keys:
                self._log(f"[ISAAC] Found keys via hardcoded probe: {asset_keys}")
        
        self._log(f"[ISAAC] Scanning scene for articulations, found keys: {asset_keys}")
        
        for name in asset_keys:
            try:
                # Use dictionary-style access for InteractiveScene
                asset = scene[name]
                
                # Check if it's an Articulation (has body_names)
                if hasattr(asset, 'body_names') and hasattr(asset, 'data'):
                    body_names = asset.body_names
                    if body_names:
                        # Extract the actual prim path prefix from the asset
                        # Isaac Lab articulations have cfg.prim_path like "{ENV_REGEX_NS}/G1"
                        # or _root_physx_view.prim_paths for actual paths like "/World/envs/env_0/G1"
                        prim_prefix = None
                        try:
                            # Try to get actual prim path from the articulation view
                            if hasattr(asset, '_root_physx_view') and asset._root_physx_view is not None:
                                prim_paths = asset._root_physx_view.prim_paths
                                if prim_paths and len(prim_paths) > 0:
                                    # Extract just the robot name from path like "/World/envs/env_0/G1"
                                    first_path = prim_paths[0]
                                    # Remove /World/envs/env_X/ prefix to get robot name
                                    parts = first_path.split('/')
                                    if len(parts) >= 5 and parts[1] == 'World' and parts[2] == 'envs':
                                        prim_prefix = parts[4]  # e.g., "G1"
                                        self._log(f"[ISAAC] Extracted prim name '{prim_prefix}' from physx view path")
                        except Exception as e:
                            self._log(f"[ISAAC] Could not extract prim path from physx view: {e}")
                        
                        # Fallback to scene key name if prim path not found
                        if prim_prefix is None:
                            prim_prefix = name
                        
                        # Also try to get body prim paths for the full USD hierarchy
                        body_prim_paths = {}
                        try:
                            # Method 1: Try _body_physx_view (may not exist in newer Isaac Lab)
                            if hasattr(asset, '_body_physx_view') and asset._body_physx_view is not None:
                                all_body_prim_paths = asset._body_physx_view.prim_paths
                                # Map each body name to its full prim path
                                for i, bname in enumerate(body_names):
                                    if i < len(all_body_prim_paths):
                                        full_path = all_body_prim_paths[i]
                                        if "/World/envs/" in full_path:
                                            relative_path = full_path.split("/World/envs/")[1]
                                            parts = relative_path.split("/", 1)
                                            if len(parts) > 1:
                                                body_prim_paths[bname] = parts[1]
                                self._log(f"[ISAAC] Extracted {len(body_prim_paths)} body prim paths from _body_physx_view")
                            
                            # Method 2: Fallback - traverse USD stage to find body prims
                            if not body_prim_paths and hasattr(self, '_isaac_stage') and self._isaac_stage is not None:
                                from pxr import Usd
                                stage = self._isaac_stage
                                root_path = f"/World/envs/env_0/{prim_prefix}"
                                root_prim = stage.GetPrimAtPath(root_path)
                                if root_prim and root_prim.IsValid():
                                    # Search for each body_name in the hierarchy
                                    for bname in body_names:
                                        # Traverse all descendants to find prim with matching name
                                        for prim in Usd.PrimRange(root_prim):
                                            if prim.GetName() == bname:
                                                full_path = str(prim.GetPath())
                                                if "/World/envs/" in full_path:
                                                    relative_path = full_path.split("/World/envs/")[1]
                                                    parts = relative_path.split("/", 1)
                                                    if len(parts) > 1:
                                                        body_prim_paths[bname] = parts[1]
                                                break
                                    self._log(f"[ISAAC] Extracted {len(body_prim_paths)} body prim paths from USD stage traversal")
                            
                            if body_prim_paths:
                                # Show a few examples
                                examples = list(body_prim_paths.items())[:5]
                                for bname, bpath in examples:
                                    self._log(f"  - {bname} → {bpath}")
                        except Exception as e:
                            self._log(f"[ISAAC] Could not extract body prim paths: {e}")
                        
                        articulations[name] = {
                            'asset': asset,
                            'body_names': list(body_names),
                            'type': 'articulation',
                            'prim_name': prim_prefix,  # Store the actual prim name
                            'body_prim_paths': body_prim_paths,  # Map body_name → full USD relative path
                        }
                        self._log(f"[ISAAC] Found articulation '{name}' (prim: '{prim_prefix}') with {len(body_names)} bodies: {list(body_names)[:5]}...")
                
                # Check if it's a RigidObject (has data.root_pos_w but no body_names)
                elif hasattr(asset, 'data') and hasattr(asset.data, 'root_pos_w'):
                    # RigidObject: extract prim name from physx view or asset path
                    prim_name = name.lower()
                    try:
                        if hasattr(asset, '_root_physx_view') and asset._root_physx_view is not None:
                            prim_paths = asset._root_physx_view.prim_paths
                            if prim_paths and len(prim_paths) > 0:
                                parts = prim_paths[0].split('/')
                                if len(parts) >= 5:
                                    prim_name = parts[4].lower()
                                    self._log(f"[ISAAC] Rigid object prim name: '{prim_name}'")
                    except Exception:
                        pass
                    
                    articulations[name] = {
                        'asset': asset,
                        'body_names': [prim_name],
                        'type': 'rigid_object',
                        'prim_name': prim_name,
                    }
                    self._log(f"[ISAAC] Found rigid object '{name}' (prim: '{prim_name}')")
                    
            except Exception as e:
                self._log(f"[ISAAC] Error accessing '{name}': {e}")
        
        return articulations
    
    def _get_isaac_static_assets(self, scene, env_indices: Optional[List[int]] = None) -> Dict[str, Dict[str, Any]]:
        """
        Get static assets (AssetBaseCfg) from Isaac Lab stage.
        
        Detects prims that exist in the USD stage but aren't tracked by articulations/rigid objects.
        These are static scene elements like tables, walls, etc.
        
        Returns:
            Dict mapping Swift-compatible name to pose info {xpos, xquat}
        """
        static_assets = {}
        
        if self._isaac_stage is None:
            return static_assets
        
        from pxr import Usd, UsdGeom
        
        # Get the prim names that are already tracked as articulations/rigid objects
        tracked_prim_names = set()
        if self._isaac_articulations:
            for art_info in self._isaac_articulations.values():
                prim_name = art_info.get('prim_name', '')
                if prim_name:
                    tracked_prim_names.add(prim_name)
        
        self._log(f"[ISAAC] Tracked articulation prim names: {tracked_prim_names}")
        
        # Determine which environments to scan
        env_idx = 0 if env_indices is None else env_indices[0]
        env_path = f"/World/envs/env_{env_idx}"
        
        env_prim = self._isaac_stage.GetPrimAtPath(env_path)
        if not env_prim or not env_prim.IsValid():
            self._log(f"[ISAAC] Environment prim not found at {env_path}")
            return static_assets
        
        # Scan immediate children of env_0 for static assets
        for child in env_prim.GetChildren():
            child_name = child.GetName()
            
            # Skip if already tracked as articulation/rigid object
            if child_name in tracked_prim_names:
                continue
            
            # Skip ground, light, and similar
            if child_name.lower() in ['defaultgroundplane', 'groundplane', 'light', 'dome_light']:
                continue
            
            # Skip materials, shaders, and looks
            if 'material' in child_name.lower() or 'shader' in child_name.lower() or 'looks' in child_name.lower():
                continue
            
            try:
                xformable = UsdGeom.Xformable(child)
                if not xformable:
                    continue
                
                # Get world transform
                xform_cache = UsdGeom.XformCache()
                world_transform = xform_cache.GetLocalToWorldTransform(child)
                translation = world_transform.ExtractTranslation()
                xpos = [translation[0], translation[1], translation[2]]
                
                rotation = world_transform.ExtractRotationQuat()
                w = rotation.GetReal()
                imaginary = rotation.GetImaginary()
                xquat = [w, imaginary[0], imaginary[1], imaginary[2]]
                
                # Create Swift-compatible name: env_0/Table
                swift_name = f"env_{env_idx}/{child_name}"
                
                static_assets[swift_name] = {
                    "xpos": xpos,
                    "xquat": xquat,
                    "prim_path": str(child.GetPath()),
                }
                self._log(f"[ISAAC] Static asset '{swift_name}': pos=[{xpos[0]:.3f}, {xpos[1]:.3f}, {xpos[2]:.3f}]", force=True)
                
            except Exception as e:
                self._log(f"[ISAAC] Error getting static asset '{child_name}': {e}")
        
        return static_assets
    
    def _get_isaac_bodies(self, stage, env_indices: Optional[List[int]] = None) -> Dict[str, str]:
        """
        Extract body prim paths from an Isaac Lab stage for pose streaming.
        
        Traverses the stage looking for prims with transforms (Xformable) that
        represent bodies we want to animate. Focuses on prims under /World/envs/.
        
        Returns:
            Dict mapping clean body name to prim path
        """
        from pxr import Usd, UsdGeom
        
        bodies = {}
        
        # Determine which env paths to scan
        env_paths = []
        if env_indices is not None:
            env_paths = [f"/World/envs/env_{i}" for i in env_indices]
        else:
            # Discover all env_* prims
            envs_prim = stage.GetPrimAtPath("/World/envs")
            if envs_prim.IsValid():
                for child in envs_prim.GetChildren():
                    if child.GetName().startswith("env_"):
                        env_paths.append(str(child.GetPath()))
        
        # Traverse each environment
        for env_path in env_paths:
            env_prim = stage.GetPrimAtPath(env_path)
            if not env_prim.IsValid():
                continue
            
            # Extract env index for naming
            env_name = env_prim.GetName()  # e.g., "env_0"
            
            # Recursively find all Xformable prims
            for prim in Usd.PrimRange(env_prim):
                if not prim.IsValid():
                    continue
                
                # Check if this prim is Xformable (has transform)
                xformable = UsdGeom.Xformable(prim)
                if not xformable:
                    continue
                
                # Get the prim path and name
                prim_path = str(prim.GetPath())
                prim_name = prim.GetName()
                
                # Skip certain prims
                if prim_name in ["defaultGroundPlane", "GroundPlane", "Light"]:
                    continue
                if "material" in prim_name.lower() or "shader" in prim_name.lower():
                    continue
                
                # Create a name that matches Swift's USDZ entity path indexing
                # Swift indexes paths like: env_0/Franka/fr3_link6
                # So we strip /World/envs/ prefix but keep the rest with / separators
                relative_path = prim_path.replace("/World/envs/", "")
                # Keep the / separators to match Swift's path format
                clean_name = relative_path.replace("-", "")
                
                bodies[clean_name] = prim_path
        
        self._log(f"[ISAAC] Found {len(bodies)} bodies to track")
        if self.verbose and bodies:
            for name in list(bodies.keys())[:5]:  # Show first 5
                self._log(f"  - {name}: {bodies[name]}")
            if len(bodies) > 5:
                self._log(f"  ... and {len(bodies) - 5} more")
        
        return bodies
    
    def update_sim(self):
        """
        Sync current simulation state to the VisionPro.
        
        Call this after each simulation step to update the 3D visualization.
        Works with both MuJoCo (configure_mujoco) and Isaac Lab (configure_isaac).
        
        Example (MuJoCo)::
        
            while True:
                mujoco.mj_step(model, data)
                streamer.update_sim()
        
        Example (Isaac Lab)::
        
            while simulation_app.is_running():
                sim.step()
                streamer.update_sim()
        """
        # Check which simulation backend is configured
        is_isaac = self._isaac_stage is not None
        is_mujoco = self._mujoco_model is not None and self._mujoco_data is not None
        
        if not is_isaac and not is_mujoco:
            self._log("[SIM] Warning: No simulation configured. Call configure_mujoco() or configure_isaac() first.", force=True)
            return
        
        if not self._pose_stream_running:
            # Pose streaming not started yet
            return
        
        # Get current poses based on backend
        if is_isaac:
            poses = self._get_isaac_poses()
            qpos = []  # Isaac doesn't have a simple qpos equivalent
            ctrl = []
        else:
            poses = self._get_mujoco_poses()
            qpos = self._mujoco_data.qpos.tolist()
            ctrl = self._mujoco_data.ctrl.tolist()
        
        timestamp = time.time()
        
        # Update the current poses that the streaming thread will send
        with self._pose_stream_lock:
            self._current_poses = {
                "poses": poses,
                "qpos": qpos,
                "ctrl": ctrl,
                "timestamp": timestamp,
            }
    
    def _get_mujoco_poses(self) -> Dict[str, Dict[str, Any]]:
        """Get current MuJoCo body poses as a dictionary.
        
        Returns dict with structure: {clean_body_name: {"xpos": [x,y,z], "xquat": [w,x,y,z]}}
        """
        body_dict = {}
        data = self._mujoco_data
        clean_names = self._mujoco_clean_names  # Use cached clean names
        
        for body_name, body_id in self._mujoco_bodies.items():
            if "world" in body_name.lower():
                continue  # Skip world body
            
            # Get pose data - .tolist() creates a copy (safe for threading)
            xpos = data.body(body_id).xpos.tolist()
            xquat = data.body(body_id).xquat.tolist()
            
            # Use pre-computed clean name (no string ops per frame)
            clean_name = clean_names.get(body_name, body_name)
            body_dict[clean_name] = {
                "xpos": xpos,
                "xquat": xquat,
            }
        
        return body_dict
    
    _isaac_pose_debug_counter = 0  # Class-level counter for debug
    
    def _get_isaac_poses(self) -> Dict[str, Dict[str, Any]]:
        """
        Get current Isaac Lab body poses as a dictionary.
        
        Uses articulation body data from the physics simulation (not USD stage).
        """
        body_dict = {}
        
        # Debug counter
        VisionProStreamer._isaac_pose_debug_counter += 1
        debug_this_frame = (VisionProStreamer._isaac_pose_debug_counter % 100 == 1)
        
        # Use articulation data if available (preferred - has live physics poses)
        if hasattr(self, '_isaac_articulations') and self._isaac_articulations:
            for art_name, art_info in self._isaac_articulations.items():
                asset = art_info['asset']
                body_names = art_info['body_names']
                asset_type = art_info.get('type', 'articulation')
                
                try:
                    # Only use first environment for now
                    env_idx = 0
                    
                    if asset_type == 'rigid_object':
                        # RigidObject: use root_pos_w and root_quat_w
                        pos = asset.data.root_pos_w[env_idx].cpu().numpy()
                        quat = asset.data.root_quat_w[env_idx].cpu().numpy()  # wxyz
                        
                        # Single-body: env_0/object
                        swift_name = f"env_{env_idx}/{body_names[0]}"
                        
                        body_dict[swift_name] = {
                            "xpos": pos.tolist(),
                            "xquat": quat.tolist(),
                        }
                    else:
                        # Articulation: use body_pos_w and body_quat_w
                        body_pos = asset.data.body_pos_w  # (num_envs, num_bodies, 3)
                        body_quat = asset.data.body_quat_w  # (num_envs, num_bodies, 4) wxyz
                        
                        # Check if this is a single-body asset (like a cube/object)
                        is_single_body = len(body_names) == 1
                        
                        for body_idx, body_name in enumerate(body_names):
                            # Create Swift-compatible path that matches USDZ entity hierarchy
                            if is_single_body:
                                # Single-body objects: env_0/object (matches USDZ directly)
                                swift_name = f"env_{env_idx}/{body_name}"
                            else:
                                # Multi-body articulations: try to use actual USD prim path
                                body_prim_paths = art_info.get('body_prim_paths', {})
                                if body_name in body_prim_paths:
                                    # Use the full USD hierarchy path: env_0/Robot/ee_link/palm_link
                                    swift_name = f"env_{env_idx}/{body_prim_paths[body_name]}"
                                else:
                                    # Fallback to old method: env_0/Robot/palm_link
                                    prim_name = art_info.get('prim_name', art_name)
                                    swift_name = f"env_{env_idx}/{prim_name}/{body_name}"
                            
                            # Get position and quaternion for this body
                            pos = body_pos[env_idx, body_idx].cpu().numpy()
                            quat = body_quat[env_idx, body_idx].cpu().numpy()  # wxyz
                            
                            body_dict[swift_name] = {
                                "xpos": pos.tolist(),
                                "xquat": quat.tolist(),  # Already wxyz format
                            }
                except Exception as e:
                    if debug_this_frame:
                        print(f"[DEBUG] Error getting poses for {art_name}: {e}")
                    continue
        
        # Fallback to USD stage if no articulation data (static poses)
        elif self._isaac_stage is not None:
            body_dict = self._get_isaac_poses_from_stage()
        
        # Include static assets for the first few frames only
        if hasattr(self, '_isaac_static_assets') and self._isaac_static_assets:
            if hasattr(self, '_isaac_static_frames_sent'):
                if self._isaac_static_frames_sent < getattr(self, '_isaac_static_frames_limit', 5):
                    # Include static asset poses
                    for swift_name, pose_info in self._isaac_static_assets.items():
                        body_dict[swift_name] = {
                            "xpos": pose_info["xpos"],
                            "xquat": pose_info["xquat"],
                        }
                    self._isaac_static_frames_sent += 1
                    if debug_this_frame:
                        print(f"[DEBUG] Including {len(self._isaac_static_assets)} static assets (frame {self._isaac_static_frames_sent}/{self._isaac_static_frames_limit})")
        
        # Debug: print poses every 100 frames
        if debug_this_frame and body_dict:
            print(f"[DEBUG] Isaac poses frame {VisionProStreamer._isaac_pose_debug_counter}: {len(body_dict)} bodies")
            # Show link7 specifically (the end effector link)
            for name, pose in body_dict.items():
                if "link7" in name.lower() or "link6" in name.lower() or "hand" in name.lower():
                    pos = pose["xpos"]
                    print(f"  {name}: pos=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
            # Also show target_pose and object
            for name, pose in body_dict.items():
                if "target" in name.lower() or "object" in name.lower():
                    pos = pose["xpos"]
                    quat = pose["xquat"]
                    print(f"  {name}: pos=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}], quat=[{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]")
        
        return body_dict
    
    def _get_isaac_poses_from_stage(self) -> Dict[str, Dict[str, Any]]:
        """Get poses from USD stage using PhysX runtime data (Isaac Lab only)."""
        body_dict = {}
        stage = self._isaac_stage
        
        if stage is None:
            return body_dict
        
        def prim_path_to_swift_name(prim_path: str) -> str:
            """Convert USD prim path to Swift entity name format.
            
            USD path: /World/envs/env_0/Franka/fr3_link6
            Swift expects: env_0/Franka/fr3_link6
            """
            # Strip /World/envs/ prefix to match USDZ hierarchy
            if prim_path.startswith("/World/envs/"):
                return prim_path[len("/World/envs/"):]
            elif prim_path.startswith("/World/"):
                return prim_path[len("/World/"):]
            return prim_path.lstrip("/")
        
        try:
            from pxr import UsdGeom, Usd
            
            # Use XformCache to get transforms from USD stage
            # Note: These won't update with physics simulation unless scene is provided
            xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
            
            for clean_name, prim_path in self._isaac_bodies.items():
                swift_name = prim_path_to_swift_name(prim_path)
                
                prim = stage.GetPrimAtPath(prim_path)
                if not prim.IsValid():
                    continue
                
                xformable = UsdGeom.Xformable(prim)
                if not xformable:
                    continue
                
                try:
                    xform_cache.Clear()
                    world_transform = xform_cache.GetLocalToWorldTransform(prim)
                    translation = world_transform.ExtractTranslation()
                    xpos = [translation[0], translation[1], translation[2]]
                    
                    rotation = world_transform.ExtractRotationQuat()
                    w = rotation.GetReal()
                    imaginary = rotation.GetImaginary()
                    xquat = [w, imaginary[0], imaginary[1], imaginary[2]]
                    
                    body_dict[swift_name] = {
                        "xpos": xpos,
                        "xquat": xquat,
                    }
                except Exception:
                    continue
        except Exception as e:
            self._log(f"[ISAAC] Error in _get_isaac_poses_from_stage: {e}")
        
        return body_dict
    
    def _start_pose_streaming(self):
        """Start pose streaming via WebRTC data channel."""
        if self._pose_stream_running:
            self._log("[SIM] Warning: Pose streaming already running")
            return
        
        if self._sim_config is None:
            self._log("[SIM] Warning: No simulation configured")
            return
        
        # If WebRTC channel is already ready, start streaming
        if self._webrtc_sim_ready:
            self._start_pose_streaming_webrtc()
        else:
            # Streaming will start when the WebRTC channel opens
            self._log("[SIM] Waiting for WebRTC sim-poses channel to open...", force=True)
    
    def _start_pose_streaming_webrtc(self):
        """Start the actual pose streaming loop via WebRTC.
        
        Uses thread-based approach for reliability.
        """
        if self._pose_stream_running:
            return
        
        self._log("[SIM] Starting pose streaming via WebRTC data channel...", force=True)
        self._pose_stream_running = True
        
        # Thread-based streaming
        self._pose_stream_thread = Thread(target=self._pose_streaming_loop_webrtc, daemon=True)
        self._pose_stream_thread.start()
    
    def _stop_pose_streaming(self):
        """Stop pose streaming."""
        if not self._pose_stream_running:
            return
        
        self._log("[SIM] Stopping pose streaming...")
        self._pose_stream_running = False
        if self._pose_stream_thread:
            self._pose_stream_thread.join(timeout=2.0)
    
    def _pose_streaming_loop_webrtc(self):
        """Background thread to stream poses via WebRTC data channel."""
        import struct
        import copy  # Pre-import for speed (was being imported inside loop!)
        
        # Pre-create struct objects for faster packing (avoid format string parsing each call)
        struct_timestamp = struct.Struct('<d')  # 8 bytes
        struct_body_count = struct.Struct('<H')  # 2 bytes
        struct_name_len = struct.Struct('<B')  # 1 byte
        struct_pose = struct.Struct('<7f')  # 28 bytes (x, y, z, qx, qy, qz, qw)
        struct_array_len = struct.Struct('<H')  # 2 bytes
        
        self._log("[SIM] Pose streaming via WebRTC started!", force=True)
        frame_count = 0
        name_bytes_cache = {}  # Cache encoded name bytes
        
        # Pre-allocate reusable bytearray (will grow if needed)
        buffer = bytearray(4096)
        
        try:
            while self._pose_stream_running:
                if not self._webrtc_sim_ready or self._webrtc_sim_channel is None:
                    time.sleep(0.1)
                    continue
                
                # CRITICAL: Must hold lock AND deep copy to avoid race with update_sim()
                with self._pose_stream_lock:
                    if self._current_poses:
                        current_data = copy.deepcopy(self._current_poses)
                    else:
                        current_data = None
                
                if current_data:
                    # Extract data components
                    if "poses" in current_data and isinstance(current_data["poses"], dict):
                        poses = current_data["poses"]
                        qpos = current_data.get("qpos", [])
                        ctrl = current_data.get("ctrl", [])
                        timestamp = current_data.get("timestamp", time.time())
                    else:
                        poses = current_data
                        qpos = []
                        ctrl = []
                        timestamp = time.time()
                    
                    # Build binary message using bytearray for efficiency
                    # Estimate size: 8 + 2 + bodies*(1+20+28) + 2 + qpos*4 + 2 + ctrl*4
                    body_count = len(poses)
                    estimated_size = 12 + body_count * 50 + len(qpos) * 4 + len(ctrl) * 4
                    
                    # Use list of bytes for simpler code (bytearray extend has overhead)
                    parts = []
                    parts.append(struct_timestamp.pack(timestamp))
                    
                    # Count valid bodies and pack
                    valid_count = 0
                    body_data = []
                    for body_name, pose_data in poses.items():
                        if not body_name:
                            continue
                        valid_count += 1
                        
                        xpos = pose_data["xpos"]
                        xquat = pose_data["xquat"]
                        
                        # Cache name bytes
                        if body_name not in name_bytes_cache:
                            name_bytes_cache[body_name] = body_name.encode('utf-8')[:255]
                        name_bytes = name_bytes_cache[body_name]
                        
                        body_data.append(struct_name_len.pack(len(name_bytes)))
                        body_data.append(name_bytes)
                        # Convert wxyz to xyzw for Swift
                        body_data.append(struct_pose.pack(
                            xpos[0], xpos[1], xpos[2],
                            xquat[1], xquat[2], xquat[3], xquat[0]
                        ))
                    
                    parts.append(struct_body_count.pack(valid_count))
                    parts.extend(body_data)
                    
                    # qpos - pack all at once if present
                    parts.append(struct_array_len.pack(len(qpos)))
                    if qpos:
                        parts.append(struct.pack(f'<{len(qpos)}f', *qpos))
                    
                    # ctrl - pack all at once if present
                    parts.append(struct_array_len.pack(len(ctrl)))
                    if ctrl:
                        parts.append(struct.pack(f'<{len(ctrl)}f', *ctrl))
                    
                    message = b''.join(parts)
                    
                    try:
                        if self._webrtc_loop is not None:
                            self._webrtc_loop.call_soon_threadsafe(
                                self._webrtc_sim_channel.send, message
                            )
                        else:
                            self._webrtc_sim_channel.send(message)
                        frame_count += 1
                        if frame_count == 1:
                            self._log(f"[SIM] First pose ({valid_count} bodies, {len(message)}B)", force=True)
                        elif frame_count % 500 == 0:  # Reduced logging frequency
                            self._log(f"[SIM] Streamed {frame_count} frames")
                    except Exception as e:
                        if frame_count < 5:
                            self._log(f"[SIM] Send failed: {e}", force=True)
                
                # Rate limit based on streaming_hz config
                hz = self._sim_config.get("streaming_hz", 120) if self._sim_config else 120
                time.sleep(1.0 / hz)
        
        except Exception as e:
            if self._pose_stream_running:
                self._log(f"[SIM] Streaming error: {e}", force=True)
            self._pose_stream_running = False
        
        self._log(f"[SIM] Streaming stopped ({frame_count} total)")
    
    async def _pose_streaming_loop_webrtc_async(self):
        """Async version of pose streaming that runs on the event loop.
        
        This is needed on Linux where cross-thread sends to aiortc data channels
        don't work reliably. By running on the event loop thread, sends work correctly.
        """
        import asyncio
        self._log("[SIM] Pose streaming started (async, binary)", force=True)
        frame_count = 0
        
        try:
            while self._pose_stream_running:
                if not self._webrtc_sim_ready or self._webrtc_sim_channel is None:
                    await asyncio.sleep(0.1)
                    continue
                
                with self._pose_stream_lock:
                    current_data = self._current_poses.copy()
                
                if current_data:
                    # Extract data components
                    if "poses" in current_data and isinstance(current_data["poses"], dict):
                        poses = current_data["poses"]
                        qpos = current_data.get("qpos", [])
                        ctrl = current_data.get("ctrl", [])
                        timestamp = current_data.get("timestamp", time.time())
                    else:
                        poses = current_data
                        qpos = []
                        ctrl = []
                        timestamp = time.time()

                    # Build binary message for poses (much smaller than JSON)
                    # Format:
                    #   [timestamp: 8 bytes double]
                    #   [body_count: 2 bytes uint16]
                    #   For each body:
                    #     [name_len: 1 byte uint8]
                    #     [name: N bytes UTF-8]
                    #     [x, y, z, qx, qy, qz, qw: 7 × 4 bytes float32]
                    #   [qpos_count: 2 bytes uint16]
                    #   [qpos values: N × 4 bytes float32]
                    #   [ctrl_count: 2 bytes uint16]
                    #   [ctrl values: N × 4 bytes float32]
                    
                    import struct
                    
                    # Start building binary message
                    parts = []
                    
                    # Timestamp (8 bytes, double)
                    parts.append(struct.pack('<d', timestamp))
                    
                    # Body count (2 bytes)
                    valid_bodies = [(name, pose) for name, pose in poses.items() if name]
                    parts.append(struct.pack('<H', len(valid_bodies)))
                    
                    # Bodies
                    for body_name, pose_data in valid_bodies:
                        xpos = pose_data["xpos"]
                        xquat = pose_data["xquat"]  # w, x, y, z (MuJoCo format)
                        
                        # Name (1 byte length + N bytes)
                        name_bytes = body_name.encode('utf-8')[:255]  # Max 255 chars
                        parts.append(struct.pack('<B', len(name_bytes)))
                        parts.append(name_bytes)
                        
                        # 7 floats: x, y, z, qx, qy, qz, qw
                        parts.append(struct.pack('<7f',
                            float(xpos[0]), float(xpos[1]), float(xpos[2]),
                            float(xquat[1]), float(xquat[2]), float(xquat[3]), float(xquat[0])  # Convert wxyz to xyzw
                        ))
                    
                    # qpos (2 bytes count + N floats)
                    parts.append(struct.pack('<H', len(qpos)))
                    if qpos:
                        parts.append(struct.pack(f'<{len(qpos)}f', *[float(x) for x in qpos]))
                    
                    # ctrl (2 bytes count + N floats)
                    parts.append(struct.pack('<H', len(ctrl)))
                    if ctrl:
                        parts.append(struct.pack(f'<{len(ctrl)}f', *[float(x) for x in ctrl]))
                    
                    message = b''.join(parts)
                    
                    # Send as single binary message
                    try:
                        self._webrtc_sim_channel.send(message)
                        frame_count += 1
                        if frame_count == 1:
                            self._log(f"[SIM] First sim pose sent ({len(valid_bodies)} bodies, {len(message)} bytes binary)", force=True)
                        elif frame_count % 500 == 0:
                            self._log(f"[SIM] Sent {frame_count} sim pose updates")
                    except Exception as e:
                        self._log(f"[SIM] Warning: Failed to send pose: {e}", force=True)
                
                # Use async sleep to yield control and not block the event loop
                await asyncio.sleep(0.008)  # ~120 Hz
        
        except asyncio.CancelledError:
            self._log("[SIM] Pose streaming cancelled")
        except Exception as e:
            if self._pose_stream_running:
                self._log(f"[SIM] Error: Pose streaming error: {e}", force=True)
            self._pose_stream_running = False
        
        self._log(f"[SIM] Pose streaming stopped (sent {frame_count} updates)")

    
    def _load_and_send_scene(self):
        """Load USDZ scene and send to VisionPro.
        
        Handles both MuJoCo and Isaac Lab scenes based on which configure_* was called.
        
        In local mode: Uses gRPC for fast, reliable transfer.
        In cross-network mode: Uses WebRTC data channel for NAT traversal.
        """
        if self._sim_config is None:
            return False
        
        attach_to = self._sim_config["attach_to"]
        
        # Cross-network mode: Use WebRTC data channel for USDZ transfer
        if self._cross_network_mode:
            return self._load_and_send_scene_webrtc(attach_to)
        
        # Local mode: Use gRPC
        grpc_port = self._sim_config["grpc_port"]
        
        # Check if this is an Isaac Lab scene
        is_isaac = self._sim_config.get("type") == "isaac"
        
        if is_isaac:
            # Isaac Lab: export stage to USDZ using our export utility
            return self._load_and_send_isaac_scene(attach_to, grpc_port)
        else:
            # MuJoCo: convert XML to USDZ
            return self._load_and_send_mujoco_scene(attach_to, grpc_port)
    
    def _load_and_send_scene_webrtc(self, attach_to) -> bool:
        """Send USDZ scene via WebRTC data channel for cross-network mode.
        
        Uses a dedicated data channel to transfer the USDZ file in chunks.
        This works across NAT/firewalls since WebRTC connection is already established.
        """
        import tempfile
        import hashlib
        import json
        import struct
        
        is_isaac = self._sim_config.get("type") == "isaac"
        
        # Get or generate the USDZ file path (using existing caching logic)
        if is_isaac:
            # Isaac Lab: export to cached USDZ
            if self._isaac_stage is None or self._isaac_config is None:
                self._log("[USDZ-WEBRTC] Error: No Isaac stage configured", force=True)
                return False
            
            cache_key = self._compute_isaac_cache_key()
            cache_dir = os.path.join(tempfile.gettempdir(), "isaac_usdz_cache")
            os.makedirs(cache_dir, exist_ok=True)
            usdz_path = os.path.join(cache_dir, f"isaac_scene_{cache_key}.usdz")
            
            force_reload = self._isaac_config.get("force_reload", False)
            if force_reload or not os.path.exists(usdz_path):
                self._log("[USDZ-WEBRTC] Exporting Isaac stage to USDZ...", force=True)
                try:
                    from avp_stream.utils.isaac_usdz_export import export_stage_to_usdz
                    include_ground = self._isaac_config.get("include_ground", False)
                    env_indices = self._isaac_config.get("env_indices")
                    usdz_path = export_stage_to_usdz(
                        stage=self._isaac_stage,
                        output_path=usdz_path,
                        include_ground=include_ground,
                        env_indices=env_indices,
                    )
                except Exception as e:
                    self._log(f"[USDZ-WEBRTC] Error exporting Isaac: {e}", force=True)
                    return False
        else:
            # MuJoCo: convert to cached USDZ
            xml_path = self._sim_config["xml_path"]
            force_reload = self._sim_config.get("force_reload", False)
            
            with open(xml_path, 'rb') as f:
                xml_content = f.read()
            xml_hash = hashlib.sha256(xml_content).hexdigest()[:16]
            
            cache_dir = os.path.join(tempfile.gettempdir(), "mujoco_usdz_cache")
            os.makedirs(cache_dir, exist_ok=True)
            usdz_path = os.path.join(cache_dir, f"mujoco_scene_{xml_hash}.usdz")
            
            if force_reload or not os.path.exists(usdz_path):
                self._log("[USDZ-WEBRTC] Converting MuJoCo XML to USDZ...", force=True)
                try:
                    usdz_path = self._convert_to_usdz(xml_path, output_path=usdz_path)
                    self._log(f"[USDZ-WEBRTC] Converted: {usdz_path}", force=True)
                except Exception as e:
                    self._log(f"[USDZ-WEBRTC] Local conversion failed: {e}, trying external...", force=True)
                    # Try external converter
                    try:
                        from avp_stream.mujoco_msg.upload_xml import convert_and_download
                        from pathlib import Path
                        usdz_path = convert_and_download(
                            server="http://mujoco-usd-convert.xyz",
                            scene_xml=Path(xml_path),
                            out_dir=Path(cache_dir)
                        )
                        self._log(f"[USDZ-WEBRTC] External conversion succeeded: {usdz_path}", force=True)
                    except Exception as e2:
                        self._log(f"[USDZ-WEBRTC] Error converting MuJoCo: {e2}", force=True)
                        return False
        
        # Read USDZ file
        if not os.path.exists(usdz_path):
            self._log(f"[USDZ-WEBRTC] Error: USDZ file not found: {usdz_path}", force=True)
            return False
        
        with open(usdz_path, 'rb') as f:
            usdz_data = f.read()
        
        filename = os.path.basename(usdz_path)
        total_size = len(usdz_data)
        chunk_size = 16 * 1024  # 16KB chunks
        total_chunks = (total_size + chunk_size - 1) // chunk_size
        
        self._log(f"[USDZ-WEBRTC] Sending {total_size/1024/1024:.2f} MB in {total_chunks} chunks", force=True)
        
        # Check if usdz-transfer channel exists and is open
        if not hasattr(self, '_webrtc_usdz_channel') or self._webrtc_usdz_channel is None:
            self._log("[USDZ-WEBRTC] Error: USDZ transfer channel not available", force=True)
            return False
        
        channel = self._webrtc_usdz_channel
        
        if channel.readyState != "open":
            self._log(f"[USDZ-WEBRTC] Error: Channel not open (state: {channel.readyState})", force=True)
            return False
        
        self._log("[USDZ-WEBRTC] Channel is open, starting transfer...", force=True)
        
        # Prepare attach_to metadata
        attach_position = None
        attach_rotation = None
        if attach_to:
            attach_position = list(attach_to[:3])  # [x, y, z]
            quat_wxyz = attach_to[3:]
            attach_rotation = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]  # wxyz -> xyzw
        
        # Compute cache key from USDZ file content
        cache_key = hashlib.sha256(usdz_data).hexdigest()[:32]
        
        # Send metadata message (JSON)
        metadata = {
            "type": "metadata",
            "filename": filename,
            "totalSize": total_size,
            "totalChunks": total_chunks,
            "attachPosition": attach_position,
            "attachRotation": attach_rotation,
            "cacheKey": cache_key,
            "forceReload": force_reload,
        }
        
        # Use call_soon_threadsafe since we're in a background thread
        def send_message(msg):
            if self._webrtc_loop is not None:
                self._webrtc_loop.call_soon_threadsafe(channel.send, msg)
            else:
                channel.send(msg)
        
        send_message(json.dumps(metadata))
        self._log(f"[USDZ-WEBRTC] Sent metadata for {filename} (cacheKey: {cache_key[:8]}...)", force=True)
        
        # Wait for cache response from visionOS before sending chunks
        # visionOS will respond with "cached" if it has the file, allowing us to skip transfer
        self._usdz_cached_hit = False
        self._usdz_transfer_complete = False
        import time
        
        # Wait up to 2 seconds for cache response (WebRTC round-trip can be slow)
        cache_check_timeout = 2.0
        cache_check_waited = 0
        while not self._usdz_cached_hit and cache_check_waited < cache_check_timeout:
            time.sleep(0.05)
            cache_check_waited += 0.05
            # Also check if transfer was marked complete (means cache hit was received)
            if self._usdz_transfer_complete:
                break
        
        if self._usdz_cached_hit:
            self._log("[USDZ-WEBRTC] visionOS has cached version, skipping transfer!", force=True)
            return True
        
        # Send data chunks (binary: 4-byte index + chunk data)
        for i in range(total_chunks):
            start = i * chunk_size
            end = min(start + chunk_size, total_size)
            chunk_data = usdz_data[start:end]
            
            # Pack chunk index as 4-byte big-endian + chunk data
            chunk_message = struct.pack('>I', i) + chunk_data
            send_message(chunk_message)
            
            if (i + 1) % 50 == 0 or i == total_chunks - 1:
                progress = (i + 1) / total_chunks * 100
                self._log(f"[USDZ-WEBRTC] Sent chunk {i+1}/{total_chunks} ({progress:.1f}%)", force=True)
        
        self._log("[USDZ-WEBRTC] All chunks sent, waiting for acknowledgment...", force=True)
        
        # Wait for acknowledgment (stored by channel message handler)
        self._usdz_transfer_complete = False
        max_wait = 30  # seconds
        waited = 0
        while not self._usdz_transfer_complete and waited < max_wait:
            import time
            time.sleep(0.1)
            waited += 0.1
        
        if self._usdz_transfer_complete:
            self._log("[USDZ-WEBRTC] Transfer completed successfully!", force=True)
            return True
        else:
            self._log("[USDZ-WEBRTC] Warning: No acknowledgment received (timeout)", force=True)
            return True  # Still return True as data was sent
    
    def _register_webrtc_usdz_channel(self, channel):
        """Register the USDZ transfer data channel and its handlers."""
        import json
        
        self._webrtc_usdz_channel = channel
        self._usdz_channel_ready = False
        
        @channel.on("open")
        def on_open():
            self._log("[USDZ-WEBRTC] Transfer channel opened", force=True)
            self._usdz_channel_ready = True
            
            # Trigger USDZ transfer now that channel is open
            if self._sim_config is not None and not getattr(self, '_usdz_sent', False):
                import threading
                
                def send_usdz_delayed():
                    import time
                    time.sleep(0.3)  # Small delay for channel stabilization
                    self._log("[USDZ-WEBRTC] Triggering USDZ transfer...", force=True)
                    if self._load_and_send_scene_webrtc(self._sim_config.get("attach_to")):
                        self._usdz_sent = True
                
                threading.Thread(target=send_usdz_delayed, daemon=True).start()
        
        @channel.on("message")
        def on_message(message):
            # Handle acknowledgment from VisionOS
            try:
                if isinstance(message, str):
                    data = json.loads(message)
                    if data.get("type") == "complete":
                        self._log(f"[USDZ-WEBRTC] VisionOS loaded scene: {data.get('path')}", force=True)
                        self._usdz_transfer_complete = True
                    elif data.get("type") == "cached":
                        self._log(f"[USDZ-WEBRTC] VisionOS using cached scene (key: {data.get('cacheKey', 'unknown')[:8]}...)", force=True)
                        self._usdz_cached_hit = True
                        self._usdz_transfer_complete = True
                    elif data.get("type") == "error":
                        self._log(f"[USDZ-WEBRTC] VisionOS error: {data.get('message')}", force=True)
            except Exception as e:
                self._log(f"[USDZ-WEBRTC] Error parsing message: {e}")
    
    def _compute_isaac_cache_key(self) -> str:
        """Compute a hash key for the current Isaac configuration for USDZ caching.
        
        Hashes the InteractiveSceneCfg dataclass to detect any structural scene changes.
        This includes robot types, asset paths, sensor configs, etc.
        """
        import hashlib
        import json
        
        cache_data = {
            "include_ground": self._isaac_config.get("include_ground", False),
            "env_indices": self._isaac_config.get("env_indices"),
        }
        
        # Try to hash the InteractiveSceneCfg for comprehensive change detection
        if self._isaac_scene is not None:
            try:
                import dataclasses
                if hasattr(self._isaac_scene, 'cfg') and dataclasses.is_dataclass(self._isaac_scene.cfg):
                    # Convert dataclass to dict for hashing
                    cfg_dict = dataclasses.asdict(self._isaac_scene.cfg)
                    cache_data["scene_cfg"] = cfg_dict
                    self._log("[ISAAC] Using InteractiveSceneCfg for cache key")
                else:
                    # Fallback: use scene class name and detected bodies/articulations
                    cache_data["scene_class"] = self._isaac_scene.__class__.__name__
                    cache_data["bodies"] = sorted(list(self._isaac_bodies.keys()))
                    cache_data["articulations"] = sorted(list(self._isaac_articulations.keys()))
                    self._log("[ISAAC] Using fallback (bodies/articulations) for cache key")
            except Exception as e:
                # Fallback if dataclasses.asdict fails (e.g., non-serializable fields)
                cache_data["scene_class"] = self._isaac_scene.__class__.__name__
                cache_data["bodies"] = sorted(list(self._isaac_bodies.keys()))
                cache_data["articulations"] = sorted(list(self._isaac_articulations.keys()))
                self._log(f"[ISAAC] Fallback cache key due to: {e}")
        
        # Create stable hash from dict
        def json_default(obj):
            """Handle non-serializable objects in dataclass."""
            if hasattr(obj, '__dict__'):
                return str(obj.__class__.__name__)
            return str(obj)
        
        cache_str = json.dumps(cache_data, sort_keys=True, default=json_default)
        return hashlib.sha256(cache_str.encode()).hexdigest()[:16]
    
    def _load_and_send_isaac_scene(self, attach_to, grpc_port) -> bool:
        """Export Isaac Lab stage to USDZ and send to VisionPro.
        
        Uses caching to skip export if InteractiveSceneCfg hasn't changed.
        """
        import tempfile
        
        if self._isaac_stage is None or self._isaac_config is None:
            self._log("[ISAAC] Error: No Isaac stage configured", force=True)
            return False
        
        include_ground = self._isaac_config.get("include_ground", False)
        env_indices = self._isaac_config.get("env_indices")
        force_reload = self._isaac_config.get("force_reload", False)
        
        # Check cache for existing USDZ (unless force_reload)
        cache_key = self._compute_isaac_cache_key()
        cache_dir = os.path.join(tempfile.gettempdir(), "isaac_usdz_cache")
        os.makedirs(cache_dir, exist_ok=True)
        cached_usdz_path = os.path.join(cache_dir, f"isaac_scene_{cache_key}.usdz")
        
        if not force_reload and os.path.exists(cached_usdz_path):
            self._log(f"[ISAAC] Cache hit! Using cached USDZ (key={cache_key[:8]}...)", force=True)
            return self._send_usdz_data(cached_usdz_path, attach_to, grpc_port)
        
        if force_reload:
            self._log(f"[ISAAC] Force reload requested, exporting new USDZ...", force=True)
        else:
            self._log(f"[ISAAC] Cache miss (key={cache_key[:8]}...), exporting new USDZ...", force=True)
        
        try:
            # Use our Isaac USDZ export utility
            from avp_stream.utils.isaac_usdz_export import export_stage_to_usdz
            
            self._log(f"[ISAAC] Exporting stage to USDZ...")
            usdz_path = export_stage_to_usdz(
                stage=self._isaac_stage,
                output_path=cached_usdz_path,
                include_ground=include_ground,
                env_indices=env_indices,
            )
            self._log(f"[ISAAC] USDZ exported and cached: {usdz_path}")
            
        except Exception as e:
            self._log(f"[ISAAC] Error: Failed to export USDZ: {e}", force=True)
            if self.verbose:
                import traceback
                traceback.print_exc()
            return False
        
        # Send USDZ to VisionPro
        return self._send_usdz_data(cached_usdz_path, attach_to, grpc_port)
    
    def _load_and_send_mujoco_scene(self, attach_to, grpc_port) -> bool:
        """Convert MuJoCo XML to USDZ and send to VisionPro.
        
        Uses content-based caching to skip conversion if XML hasn't changed.
        """
        import tempfile
        import hashlib
        
        xml_path = self._sim_config["xml_path"]
        force_reload = self._sim_config.get("force_reload", False)
        
        # Compute hash of XML content for cache key
        try:
            with open(xml_path, 'rb') as f:
                xml_content = f.read()
            xml_hash = hashlib.sha256(xml_content).hexdigest()[:16]
        except Exception as e:
            self._log(f"[USDZ] Warning: Could not read XML for hashing: {e}")
            xml_hash = None
        
        # Check cache for existing USDZ
        if xml_hash and not force_reload:
            cache_dir = os.path.join(tempfile.gettempdir(), "mujoco_usdz_cache")
            os.makedirs(cache_dir, exist_ok=True)
            cached_usdz_path = os.path.join(cache_dir, f"mujoco_scene_{xml_hash}.usdz")
            
            if os.path.exists(cached_usdz_path):
                self._log(f"[USDZ] Cache hit! Using cached USDZ (key={xml_hash[:8]}...)", force=True)
                return self._send_usdz_data(cached_usdz_path, attach_to, grpc_port)
            
            self._log(f"[USDZ] Cache miss (key={xml_hash[:8]}...), converting...", force=True)
            usdz_path = cached_usdz_path
        else:
            # Fallback to old behavior if hashing failed or force_reload
            usdz_path = xml_path.replace('.xml', '.usdz')
        
        # Convert XML to USDZ
        try:
            usdz_path = self._convert_to_usdz(xml_path, output_path=usdz_path)
            self._log(f"[USDZ] Converted and cached: {usdz_path}")
        except Exception as e:
            self._log(f"[USDZ] Warning: Local conversion failed: {e}")
            # Try external converter
            try:
                from avp_stream.mujoco_msg.upload_xml import convert_and_download
                usdz_path = convert_and_download(
                    server="http://mujoco-usd-convert.xyz",
                    scene_xml=Path(xml_path),
                    out_dir=Path(xml_path).parent
                )
            except Exception as e2:
                self._log(f"[USDZ] Error: External conversion also failed: {e2}", force=True)
                return False
        
        # Send USDZ to VisionPro
        return self._send_usdz_data(usdz_path, attach_to, grpc_port)
    
    def _convert_to_usdz(self, xml_path: str, output_path: Optional[str] = None) -> str:
        """Convert MuJoCo XML to USDZ file (requires mujoco_usd_converter).
        
        Args:
            xml_path: Path to the MuJoCo XML file
            output_path: Optional path for USDZ output. If None, saves next to XML.
        """
        try:
            import mujoco_usd_converter
            import usdex.core
            from pxr import Usd, UsdUtils
        except ImportError:
            raise ImportError("USDZ conversion requires mujoco_usd_converter. Try: pip install mujoco-usd-converter")
        
        converter = mujoco_usd_converter.Converter()
        usd_output_path = xml_path.replace('.xml', '_usd')
        usdz_output_path = output_path if output_path else xml_path.replace('.xml', '.usdz')
        
        asset = converter.convert(xml_path, usd_output_path)
        stage = Usd.Stage.Open(asset.path)
        usdex.core.saveStage(stage, comment="modified after conversion")
        
        UsdUtils.CreateNewUsdzPackage(asset.path, usdz_output_path)
        self._log(f"[USDZ] File created: {usdz_output_path}")
        
        return usdz_output_path
    
    def _send_usdz_data(self, usdz_path: str, attach_to: Optional[List[float]], grpc_port: int) -> bool:
        """Send USDZ file data to VisionPro via gRPC."""
        
        try:
            # Connect to gRPC server
            options = [
                ('grpc.max_send_message_length', 100 * 1024 * 1024),  # 100MB
                ('grpc.max_receive_message_length', 100 * 1024 * 1024),  # 100MB
                ('grpc.keepalive_time_ms', 30000),  # 30 seconds
                ('grpc.keepalive_timeout_ms', 5000),  # 5 seconds
                ('grpc.keepalive_permit_without_calls', True),
                ('grpc.http2.max_pings_without_data', 0),
                ('grpc.http2.min_time_between_pings_ms', 10000),
                ('grpc.http2.min_ping_interval_without_data_ms', 300000),
            ]
            
            with grpc.insecure_channel(f"{self.ip}:{grpc_port}", options=options) as channel:
                grpc.channel_ready_future(channel).result(timeout=10)
                stub = mujoco_ar_pb2_grpc.MuJoCoARServiceStub(channel)
                
                # Read USDZ file
                with open(usdz_path, 'rb') as f:
                    usdz_data = f.read()
                
                usdz_filename = os.path.basename(usdz_path)
                file_size_mb = len(usdz_data) / (1024 * 1024)
                self._log(f"[USDZ] File size: {file_size_mb:.2f} MB")
                
                # Prepare attach_to parameters
                attach_position = None
                attach_rotation = None
                if attach_to:
                    position = attach_to[:3]
                    quat_wxyz = attach_to[3:]
                    # Convert wxyz -> xyzw for protobuf
                    attach_position = mujoco_ar_pb2.Vector3(x=position[0], y=position[1], z=position[2])
                    attach_rotation = mujoco_ar_pb2.Quaternion(
                        x=quat_wxyz[1], y=quat_wxyz[2], z=quat_wxyz[3], w=quat_wxyz[0]
                    )
                
                # Use chunked transfer for files larger than 2MB (more conservative)
                if file_size_mb > 2.0:
                    self._log("[USDZ] Using chunked transfer for large file...")
                    return self._send_usdz_chunked(stub, usdz_data, usdz_filename, attach_position, attach_rotation)
                else:
                    self._log(f"[USDZ] Sending data as single message...")
                    return self._send_usdz_single(stub, usdz_data, usdz_filename, attach_position, attach_rotation, file_size_mb)
                    
        except grpc.RpcError as e:
            self._log(f"[USDZ] Error: gRPC error sending USDZ: {e}", force=True)
            return False
        except Exception as e:
            self._log(f"[USDZ] Error: {e}", force=True)
            if self.verbose:
                import traceback
                traceback.print_exc()
            return False
    
    def _send_usdz_single(self, stub, usdz_data: bytes, filename: str, 
                          attach_position, attach_rotation, file_size_mb: float) -> bool:
        """Send USDZ as a single message (for small files)."""
        request = mujoco_ar_pb2.UsdzDataRequest(
            usdz_data=usdz_data,
            filename=filename,
            session_id=self._session_id,
        )
        
        if attach_position and attach_rotation:
            request.attach_to_position.CopyFrom(attach_position)
            request.attach_to_rotation.CopyFrom(attach_rotation)
        
        timeout = max(60.0, file_size_mb * 2)
        response = stub.SendUsdzData(request, timeout=timeout)
        
        if response.success:
            self._log(f"[USDZ] Sent successfully: {response.local_file_path}")
            return True
        else:
            self._log(f"[USDZ] Error: Failed to send: {response.message}", force=True)
            return False
    
    def _send_usdz_chunked(self, stub, usdz_data: bytes, filename: str,
                           attach_position, attach_rotation) -> bool:
        """Send USDZ file in chunks via gRPC streaming."""
        chunk_size = 1024 * 1024  # 1MB chunks
        total_size = len(usdz_data)
        total_chunks = (total_size + chunk_size - 1) // chunk_size
        
        self._log(f"[USDZ] Sending {total_size} bytes in {total_chunks} chunks of {chunk_size} bytes each")
        
        def chunk_generator():
            for i in range(total_chunks):
                start = i * chunk_size
                end = min(start + chunk_size, total_size)
                chunk_data = usdz_data[start:end]
                
                chunk_request = mujoco_ar_pb2.UsdzChunkRequest(
                    chunk_data=chunk_data,
                    filename=filename,
                    session_id=self._session_id,
                    chunk_index=i,
                    total_chunks=total_chunks,
                    total_size=total_size,
                    is_last_chunk=(i == total_chunks - 1)
                )
                
                # Add attach_to information only to the first chunk
                if i == 0 and attach_position is not None and attach_rotation is not None:
                    chunk_request.attach_to_position.CopyFrom(attach_position)
                    chunk_request.attach_to_rotation.CopyFrom(attach_rotation)
                
                self._log(f"[USDZ] Sending chunk {i+1}/{total_chunks} ({len(chunk_data)} bytes)")
                yield chunk_request
        
        try:
            response = stub.SendUsdzDataChunked(chunk_generator(), timeout=120.0)
            
            if response.success:
                self._log(f"[USDZ] Chunked transfer successful: {response.local_file_path}")
                return True
            else:
                self._log(f"[USDZ] Error: Failed to send chunked: {response.message}", force=True)
                return False
        except grpc.RpcError as e:
            self._log(f"[USDZ] Error: gRPC error in chunked transfer: {e}", force=True)
            return False
    
    def serve(self, port: int = 9999):
        """
        Start the WebRTC streaming server using configured video/audio/simulation.
        
        This is the core method that starts the WebRTC server. It uses the
        configuration set by configure_video(), configure_audio(), and configure_mujoco().
        
        Note: Consider using start_webrtc() instead for clarity.
        
        Args:
            port: WebRTC server port (default: 9999)
        
        Example (video only)::
        
            streamer.configure_video(device=None, size="1280x720")
            streamer.start_webrtc()
        
        Example (simulation only)::
        
            streamer.configure_mujoco("scene.xml", model, data)
            streamer.start_webrtc()
        
        Example (video + simulation)::
        
            streamer.configure_video(device=None, size="1280x720")
            streamer.configure_mujoco("scene.xml", model, data)
            streamer.start_webrtc()
        """
        self._webrtc_port = port
        
        # Cross-network mode: trigger WebRTC offer creation
        # (Video/audio/sim is now configured, so offer will include tracks)
        if self._cross_network_mode:
            self._log("[CROSS-NETWORK] Starting WebRTC streaming...", force=True)
            
            # Check if anything is configured
            if self._video_config is None and self._audio_config is None and self._sim_config is None:
                self._log("[CONFIG] Warning: No video/audio/simulation configured. Call configure_*() first.", force=True)
                return
            
            # Trigger offer creation (will include video/audio/sim tracks)
            self._log("[CROSS-NETWORK] Creating WebRTC offer with configured tracks...", force=True)
            self._trigger_webrtc_offer()
            
            # Wait for WebRTC connection if not already connected
            if not self._webrtc_connected:
                self._log("[CROSS-NETWORK] Waiting for WebRTC connection...", force=True)
                retry_count = 0
                while not self._webrtc_connected:
                    time.sleep(0.1)
                    retry_count += 1
                    if retry_count > 100:  # 10 seconds
                        self._log("[CROSS-NETWORK] Still waiting for WebRTC connection...", force=True)
                        retry_count = 0
            
            self._log("[CROSS-NETWORK] WebRTC signaling flow started.", force=True)
            return
        
        # Local mode: start TCP server and wait for VisionOS connection
        
        # Start simulation streaming if configured
        if self._sim_config is not None:
            self._log("[SIM] Starting MuJoCo simulation streaming...")
            if self._load_and_send_scene():
                self._start_pose_streaming()
        
        # Check if anything is configured
        if self._video_config is None and self._audio_config is None and self._sim_config is None:
            self._log("[CONFIG] Warning: No video/audio/simulation configured. Call configure_*() first.", force=True)
            return
        
        # Extract configuration
        video_cfg = self._video_config or {}
        audio_cfg = self._audio_config or {}
        
        device = video_cfg.get("device")
        fmt = video_cfg.get("format")
        size = video_cfg.get("size", "640x480")
        fps = video_cfg.get("fps", 30)
        width = video_cfg.get("width", 640)
        height = video_cfg.get("height", 480)
        stereo_video = video_cfg.get("stereo", False)
        
        audio_device = audio_cfg.get("device")
        audio_format = audio_cfg.get("format")
        audio_sample_rate = audio_cfg.get("sample_rate", 48000)
        stereo_audio = audio_cfg.get("stereo", False)
        
        # Get local IP
        local_ip = self.get_local_ip()
        self._log(f"[WEBRTC] Starting server on {local_ip}:{port}", force=True)
        
        # Determine what is enabled based on whether configure_* was called
        video_enabled = self._video_config is not None
        audio_enabled = self._audio_config is not None
        sim_enabled = self._sim_config is not None
        
        # Store WebRTC info for the HTTP endpoint (backup method)
        self.webrtc_info = {
            "host": local_ip,
            "port": port,
            "status": "ready",
            "stereo_video": stereo_video,
            "stereo_audio": stereo_audio,
            "audio_enabled": audio_enabled,
            "size": size,
            "audio_sample_rate": audio_sample_rate,
        }
        
        # Send WebRTC server info via gRPC
        self._log(f"[WEBRTC] Sending server info via gRPC...", force=True)
        self._send_webrtc_info_via_grpc(
            local_ip, port, 
            stereo_video=stereo_video, 
            stereo_audio=stereo_audio, 
            audio_enabled=audio_enabled,
            video_enabled=video_enabled,
            sim_enabled=sim_enabled,
        )
        
        # Create track factory classes with reference to this streamer
        ProcessedVideoTrack = _create_processed_video_track_class(self)
        ProcessedAudioTrack = _create_processed_audio_track_class(self)
        
        # Store reference for closure access
        streamer_instance = self
        
        # Create a new event loop for the WebRTC server in a separate thread
        def start_async_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            # Store loop reference for pose streaming thread
            streamer_instance._webrtc_loop = loop
            
            async def run_peer_custom(reader, writer):
                from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
                
                client_addr = writer.get_extra_info('peername')
                streamer_instance._log(f"[WEBRTC] VisionOS client connected from {client_addr}", force=True)
                
                # Configure RTCPeerConnection for low latency
                # Configure RTCPeerConnection for low latency
                ice_servers = []
                if streamer_instance._ice_servers:
                    streamer_instance._log(f"[WEBRTC] Using {len(streamer_instance._ice_servers)} custom ICE servers from signaling")
                    for s in streamer_instance._ice_servers:
                        ice_servers.append(RTCIceServer(
                            urls=s.get("urls"),
                            username=s.get("username"),
                            credential=s.get("credential")
                        ))
                else:
                    streamer_instance._log("[WEBRTC] Warning: No custom ICE servers, falling back to Google STUN")
                    ice_servers = [RTCIceServer(urls=["stun:stun.l.google.com:19302"])]

                config = RTCConfiguration(iceServers=ice_servers)
                pc = RTCPeerConnection(configuration=config)
                
                @pc.on("iceconnectionstatechange")
                async def on_ice_state_change():
                    streamer_instance._log(f"[WEBRTC] ICE state ({client_addr}): {pc.iceConnectionState}", force=True)
                    if pc.iceConnectionState == "connected":
                        streamer_instance._log(f"[WEBRTC] Connection established ({client_addr}) - video should flow now", force=True)
                        with streamer_instance._webrtc_connection_condition:
                            streamer_instance._webrtc_connected = True
                            streamer_instance._webrtc_connection_condition.notify_all()
                    if pc.iceConnectionState in ("failed", "closed", "disconnected"):
                        with streamer_instance._webrtc_connection_condition:
                            streamer_instance._webrtc_connected = False
                        if pc.iceConnectionState in ("failed", "closed"):
                            await pc.close()
                            writer.close()
                            await writer.wait_closed()
                
                @pc.on("track")
                def on_track(track):
                    streamer_instance._log(f"[WEBRTC] Track received: {track.kind}")
                
                # Create video track if video is configured
                if streamer_instance._video_config is not None:
                    try:
                        if device is None:
                            streamer_instance._log("Creating synthetic video stream (no camera)...")
                            if streamer_instance.frame_callback is None:
                                streamer_instance._log("[VIDEO] Warning: No frame callback registered. Stream will be blank.", force=True)
                                streamer_instance._log("   Use register_frame_callback() to generate frames.", force=True)
                        else:
                            streamer_instance._log(f"Opening video device: {device}...")
                        
                        processed_track = ProcessedVideoTrack(
                            device, 
                            fmt, 
                            fps,
                            (width, height),
                            streamer_instance.frame_callback
                        )
                        
                        # Store the track for external access
                        streamer_instance.camera = processed_track
                        
                        if device is None:
                            streamer_instance._log(f"[VIDEO] Synthetic video track created ({size} @ {fps}fps)")
                        else:
                            streamer_instance._log(f"[VIDEO] Camera track created for device: {device}")
                        
                        if streamer_instance.frame_callback:
                            streamer_instance._log("[VIDEO] Frame processing enabled")
                        else:
                            streamer_instance._log("[VIDEO] Frame processing disabled (identity)")
                        
                        pc.addTrack(processed_track)
                    except Exception as e:
                        streamer_instance._log(f"Error creating video track: {e}", force=True)
                        if streamer_instance.verbose:
                            traceback.print_exc()
                else:
                    streamer_instance._log("[WEBRTC] No video configured, using data channels only")
                
                # Create hand tracking data channel if using WebRTC backend
                if streamer_instance.ht_backend == "webrtc":
                    hand_channel = pc.createDataChannel(
                        "hand-tracking",
                        ordered=False,
                        maxRetransmits=0,
                    )
                    streamer_instance._register_webrtc_data_channel(hand_channel)

                    @pc.on("datachannel")
                    def _on_remote_datachannel(channel):
                        if channel.label == "hand-tracking":
                            streamer_instance._log("[WEBRTC] Remote hand data channel detected")
                            streamer_instance._register_webrtc_data_channel(channel)
                        elif channel.label == "sim-poses":
                            streamer_instance._log("[WEBRTC] Remote sim-poses data channel detected")
                            streamer_instance._register_webrtc_sim_channel(channel)
                
                # Create sim-poses data channel if simulation is configured
                if streamer_instance._sim_config is not None:
                    # Use unreliable mode for lowest latency (dropped frames are replaced by next update)
                    # Chunking keeps messages small enough to avoid MTU fragmentation issues on Linux
                    sim_channel = pc.createDataChannel(
                        "sim-poses",
                        ordered=False,  # Must be ordered to avoid body parts at different timesteps
                        maxRetransmits=0,  # Still unreliable for low latency
                    )
                    streamer_instance._register_webrtc_sim_channel(sim_channel)
                    streamer_instance._log("[WEBRTC] Created sim-poses data channel")

                # Create audio track if audio is configured (configure_audio was called)
                if streamer_instance._audio_config is not None:
                    try:
                        processed_audio_track = ProcessedAudioTrack(
                            audio_device, 
                            audio_format, 
                            streamer_instance.audio_callback,
                            stereo=stereo_audio,
                            sample_rate=audio_sample_rate,
                        )
                        pc.addTrack(processed_audio_track)
                        channels = "stereo" if stereo_audio else "mono"
                        if audio_device:
                            streamer_instance._log(f"[AUDIO] Track added from device: {audio_device} ({channels})")
                        else:
                            streamer_instance._log(f"[AUDIO] Synthetic track added ({channels}, callback-generated)")
                    except Exception as e:
                        streamer_instance._log(f"[AUDIO] Warning: Could not create audio track: {e}", force=True)
                        streamer_instance._log(f"    Continuing with video only...")
                        if streamer_instance.verbose:
                            traceback.print_exc()
                
                # Create offer
                offer = await pc.createOffer()
                
                # Force high bitrate by modifying SDP
                sdp = offer.sdp
                if "m=video" in sdp:
                    sdp = re.sub(r"(m=video.*\r\n)", r"\1b=AS:15000\r\n", sdp)
                
                offer = RTCSessionDescription(sdp=sdp, type=offer.type)
                await pc.setLocalDescription(offer)
                
                # Wait briefly for ICE candidates
                start_time = asyncio.get_event_loop().time()
                max_wait_time = 0.5
                
                while asyncio.get_event_loop().time() - start_time < max_wait_time:
                    if pc.iceGatheringState == "complete":
                        streamer_instance._log(f"ICE gathering complete in {asyncio.get_event_loop().time() - start_time:.2f}s")
                        break
                    elif pc.iceGatheringState == "gathering":
                        await asyncio.sleep(0.1)
                        streamer_instance._log(f"ICE gathering in progress, proceeding after {asyncio.get_event_loop().time() - start_time:.2f}s")
                        break
                    await asyncio.sleep(0.05)
                
                if pc.iceGatheringState == "new":
                    streamer_instance._log("Warning: ICE gathering hasn't started yet, sending offer anyway")
                
                # Send offer
                offer_payload = {
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type,
                }
                writer.write((json.dumps(offer_payload) + "\n").encode("utf-8"))
                await writer.drain()
                streamer_instance._log(f"Sent offer to VisionOS (ICE state: {pc.iceGatheringState})")
                
                # Wait for answer
                line = await reader.readline()
                if not line:
                    streamer_instance._log("VisionOS disconnected before sending answer")
                    await pc.close()
                    return
                
                answer_payload = json.loads(line.decode("utf-8"))
                answer = RTCSessionDescription(
                    sdp=answer_payload["sdp"],
                    type=answer_payload["type"],
                )
                
                await pc.setRemoteDescription(answer)
                streamer_instance._log("[WEBRTC] Connection established!", force=True)
                
                # Start stats monitoring
                async def monitor_stats():
                    while True:
                        try:
                            await asyncio.sleep(5)
                            if pc.connectionState in ["closed", "failed"]:
                                break
                                
                            stats = await pc.getStats()
                            active_pair = None
                            
                            # Find the active candidate pair
                            for report in stats.values():
                                if report.type == "transport":
                                    selected_pair_id = getattr(report, "selectedCandidatePairId", None)
                                    if selected_pair_id:
                                        active_pair = stats.get(selected_pair_id)
                                        break
                            
                            if active_pair:
                                local_candidate = stats.get(active_pair.localCandidateId)
                                remote_candidate = stats.get(active_pair.remoteCandidateId)
                                
                                if local_candidate and remote_candidate:
                                    local_type = getattr(local_candidate, "candidateType", "unknown")
                                    remote_type = getattr(remote_candidate, "candidateType", "unknown")
                                    protocol = getattr(local_candidate, "protocol", "unknown").upper()
                                    
                                    # Determine simplified connection type
                                    # relay = TURN
                                    # srflx = STUN (Server Reflexive)
                                    # prflx = STUN (Peer Reflexive)
                                    # host = Local/Direct
                                    
                                    conn_type = "UNKNOWN"
                                    if local_type == "relay" or remote_type == "relay":
                                        conn_type = "TURN (Relay)"
                                    elif local_type in ["srflx", "prflx"] or remote_type in ["srflx", "prflx"]:
                                        conn_type = "STUN (P2P)"
                                    elif local_type == "host" and remote_type == "host":
                                        conn_type = "Direct (Host)"
                                    
                                    streamer_instance._log(
                                        f"[STATS] Connected via {conn_type} "
                                        f"({local_type} <-> {remote_type} over {protocol})"
                                    )
                        except Exception as e:
                            streamer_instance._log(f"[STATS] Error monitoring stats: {e}")
                            # Don't crash the loop
                            await asyncio.sleep(5)

                asyncio.create_task(monitor_stats())

                # Keep connection alive
                try:
                    while True:
                        await asyncio.sleep(1)
                except asyncio.CancelledError:
                    pass
                finally:
                    await pc.close()
                    writer.close()
                    await writer.wait_closed()
            
            async def run_server():
                server = await asyncio.start_server(
                    run_peer_custom, "0.0.0.0", port,
                    start_serving=True, reuse_address=True, reuse_port=True
                )
                streamer_instance._log(f"[WEBRTC] Server listening on 0.0.0.0:{port}", force=True)
                async with server:
                    await server.serve_forever()
            
            loop.run_until_complete(run_server())
        
        webrtc_thread = Thread(target=start_async_server, daemon=True)
        webrtc_thread.start()
        
        # Wait a bit for server to start
        time.sleep(0.5)
        
        self._log(f"[WEBRTC] Server ready at {local_ip}:{port}", force=True)
        self._log(f"[WEBRTC] VisionOS can query http://{local_ip}:8888/webrtc_info to get connection details", force=True)
        
        self._server_running = True

        # Wait for WebRTC sim-poses channel to be ready
        if self._sim_config is None:
            return
        self._log("[WEBRTC] Waiting for connection...", force=True)
        if not self.wait_for_sim_channel(timeout=30):
            self._log("[WEBRTC] Error: Failed to establish sim-poses channel", force=True)
            return
        self._log("[WEBRTC] Sim-poses channel ready!", force=True)

    def start_webrtc(self, port: int = 9999, blocking: bool = False, timeout: float = 30.0) -> bool:
        """
        Start WebRTC streaming to Vision Pro.
        
        This starts the outbound media streaming (video/audio/simulation) to Vision Pro
        using the configuration set by configure_video(), configure_audio(), and configure_mujoco().
        
        Note: Hand tracking data flows automatically via gRPC when VisionProStreamer is
        instantiated. This method starts the *outbound* streaming for visual/audio feedback.
        
        Args:
            port: WebRTC server port (default: 9999)
            blocking: If True, block until VisionOS connects and starts receiving frames (default: False)
            timeout: Maximum time to wait for connection when blocking=True (default: 30.0 seconds)
        
        Returns:
            bool: True if connection was established (always True if blocking=False), 
                  False if timeout occurred while waiting for connection.
        
        Example::
        
            streamer = VisionProStreamer(ip=avp_ip)  # Hand tracking starts automatically
            
            streamer.configure_video(device=None, size="1280x720")
            streamer.start_webrtc()  # Start video streaming to Vision Pro (non-blocking)
            
            # Or wait for VisionOS to connect:
            streamer.configure_video(device=None, size="1280x720")
            if streamer.start_webrtc(blocking=True, timeout=60.0):
                print("VisionOS connected and receiving frames!")
            else:
                print("Timeout waiting for VisionOS connection")
        """
        if self._cross_network_mode:
            # In cross-network mode, start signaling if not already active
            if not self._signaling_connected and not (self._signaling_ws and self._signaling_ws.keep_running):
                self._start_signaling_connection()
            
            # Call serve() to trigger WebRTC offer creation with configured tracks
            self.serve(port=port)
            
            if blocking:
                return self.wait_for_connection(timeout=timeout)
            return True
        else:
            self.serve(port=port)
            
            if blocking:
                return self.wait_for_connection(timeout=timeout)
            return True

    def wait_for_sim_channel(self, timeout: float = 30.0) -> bool:
        """
        Wait for the WebRTC sim-poses data channel to open.
        
        Args:
            timeout: Maximum time to wait in seconds (default: 30)
        
        Returns:
            True if channel opened successfully, False on timeout
        
        Example::
        
            streamer.configure_mujoco("scene.xml", model, data)
            streamer.start_webrtc()
            if streamer.wait_for_sim_channel():
                # Channel is ready, start simulation
                while True:
                    mujoco.mj_step(model, data)
                    streamer.update_sim()
        """
        if self._sim_config is None:
            self._log("[SIM] Warning: No simulation configured")
            return False
        
        start_time = time.time()
        
        # Wait for sim channel to open
        while not self._webrtc_sim_ready:
            if time.time() - start_time > timeout:
                self._log(f"[WEBRTC] Warning: Timeout waiting for sim-poses channel (waited {timeout}s)", force=True)
                return False
            time.sleep(0.1)
            
        # In cross-network mode, also wait for USDZ transfer to complete
        if self._cross_network_mode:
            self._log("[USDZ-WEBRTC] Waiting for USDZ transfer to complete...", force=True)
            # Use remaining time from timeout
            elapsed = time.time() - start_time
            remaining = max(0.1, timeout - elapsed)
            wait_usdz_start = time.time()
            
            while not getattr(self, '_usdz_transfer_complete', False):
                if time.time() - wait_usdz_start > remaining:
                    self._log(f"[USDZ-WEBRTC] Warning: Timeout waiting for USDZ transfer (waited {remaining:.1f}s)", force=True)
                    return False
                time.sleep(0.1)
            
            self._log("[USDZ-WEBRTC] Transfer complete and confirmed!", force=True)
        
        return True

    def wait_for_connection(self, timeout: float = 30.0) -> bool:
        """
        Wait for the WebRTC connection to be established with VisionOS.
        
        This method blocks until VisionOS connects and the ICE connection state
        becomes "connected", meaning video/audio frames are flowing.
        
        Args:
            timeout: Maximum time to wait in seconds (default: 30)
        
        Returns:
            True if connection established successfully, False on timeout
        
        Example::
        
            streamer.configure_video(device=None, size="1280x720")
            streamer.start_webrtc()  # Non-blocking
            
            # Later, wait for connection
            if streamer.wait_for_connection(timeout=60):
                print("VisionOS is now receiving frames!")
            else:
                print("Connection timed out")
        """
        self._log(f"[WEBRTC] Waiting for VisionOS to connect (timeout={timeout}s)...", force=True)
        
        deadline = time.time() + timeout
        with self._webrtc_connection_condition:
            while not self._webrtc_connected:
                remaining = deadline - time.time()
                if remaining <= 0:
                    self._log(f"[WEBRTC] Timeout waiting for connection (waited {timeout}s)", force=True)
                    return False
                self._webrtc_connection_condition.wait(timeout=remaining)
        
        self._log("[WEBRTC] VisionOS connected - frames are flowing!", force=True)
        return True

    def is_connected(self) -> bool:
        """
        Check if WebRTC connection to VisionOS is currently active.
        
        Returns:
            True if connected and streaming, False otherwise
        
        Example::
        
            if streamer.is_connected():
                print("Streaming to VisionOS")
            else:
                print("Not connected")
        """
        return self._webrtc_connected

    def reset_benchmark_epoch(self, epoch=None):
        if epoch is None:
            epoch = time.perf_counter()
        with self._benchmark_condition:
            self._benchmark_epoch = epoch
            self._benchmark_events.clear()
        self._sim_benchmark_seq = 0  # Also reset sim benchmark sequence
        return self._benchmark_epoch

    def enable_sim_benchmark(self, enabled: bool = True):
        """Enable or disable simulation benchmark mode.
        
        When enabled, each pose update includes a sequence ID and timestamp
        that Swift can echo back to measure round-trip latency.
        
        Args:
            enabled: Whether to enable benchmark mode
            
        Example::
        
            streamer.enable_sim_benchmark(True)
            streamer.reset_benchmark_epoch()
            
            # After some updates, check for echoes
            event = streamer.wait_for_benchmark_event(sequence_id, timeout=1.0)
            if event:
                print(f"Round-trip: {event['round_trip_ms']} ms")
        """
        self._sim_benchmark_enabled = enabled
        if enabled:
            self._sim_benchmark_seq = 0
            self._log("[BENCHMARK] Sim benchmark mode enabled")
        else:
            self._log("[BENCHMARK] Sim benchmark mode disabled")

    def update_stream_resolution(self, size):
        """Request the video track to emit frames at a new resolution.

        Example
        -------
        .. code-block:: python

            streamer.update_stream_resolution("1024x768")
        """
        if self.camera is None:
            self._log("[VIDEO] Warning: Cannot update stream resolution before starting video streaming", force=True)
            return

        try:
            width, height = map(int, size.lower().split("x", 1))
        except ValueError:
            self._log(f"[VIDEO] Warning: Invalid resolution '{size}' (expected WIDTHxHEIGHT)", force=True)
            return

        if hasattr(self.camera, "set_resolution"):
            self.camera.set_resolution((width, height))

        if self.webrtc_info is not None:
            self.webrtc_info["size"] = size

        self._log(f"[VIDEO] Updated stream resolution to {width}x{height}")

    def wait_for_benchmark_event(self, sequence_id, timeout=2.0, source="video"):
        """Wait for a benchmark event with the given sequence ID.
        
        Args:
            sequence_id: The sequence ID to wait for
            timeout: Maximum time to wait in seconds
            source: Either "video" or "sim" to specify which benchmark type
            
        Returns:
            The benchmark event dict, or None if timeout
        """
        event_key = f"{source}:{sequence_id}"
        deadline = time.perf_counter() + timeout
        with self._benchmark_condition:
            while True:
                event = self._benchmark_events.pop(event_key, None)
                if event is not None:
                    return event
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    # Drop any stale data for this sequence if it arrives late.
                    self._benchmark_events.pop(event_key, None)
                    return None
                self._benchmark_condition.wait(timeout=remaining)
    
    def _start_info_server(self, port=8888):
        """
        Start a simple HTTP server that provides WebRTC connection info.
        VisionOS can query this to discover where the WebRTC server is running.
        """
        streamer_instance = self
        
        class ReusableHTTPServer(HTTPServer):
            allow_reuse_address = True

        class InfoHandler(BaseHTTPRequestHandler):
            def do_GET(handler_self):
                client_ip = handler_self.client_address[0]
                self._log(f"[HTTP] Request from {client_ip}: {handler_self.path}")
                
                if handler_self.path == '/webrtc_info':
                    if streamer_instance.webrtc_info:
                        self._log(f"[HTTP] Sending WebRTC info to {client_ip}: {streamer_instance.webrtc_info}")
                        handler_self.send_response(200)
                        handler_self.send_header('Content-type', 'application/json')
                        handler_self.end_headers()
                        handler_self.wfile.write(json.dumps(streamer_instance.webrtc_info).encode())
                    else:
                        self._log(f"[HTTP] Warning: WebRTC not started yet, sending 404 to {client_ip}")
                        handler_self.send_response(404)
                        handler_self.send_header('Content-type', 'application/json')
                        handler_self.end_headers()
                        handler_self.wfile.write(json.dumps({"error": "WebRTC not started"}).encode())
                else:
                    self._log(f"[HTTP] Error: Unknown path from {client_ip}: {handler_self.path}")
                    handler_self.send_response(404)
                    handler_self.end_headers()
            
            def log_message(self, format, *args):
                # Custom logging
                pass
        
        def run_http_server():
            try:
                server = ReusableHTTPServer(('0.0.0.0', port), InfoHandler)
                self.info_server = server
                # print(f"Info server started on port {port}")
                server.serve_forever()
            except Exception as e:
                # print(f"Could not start info server: {e}")
                pass 
        
        info_thread = Thread(target=run_http_server, daemon=True)
        info_thread.start()
    
    def start_streaming(
        self,
        # Video configuration
        device=None,
        format=None,
        size="640x480",
        fps=30,
        stereo_video=False,
        # Audio configuration
        audio_device=None,
        audio_format=None,
        audio_sample_rate=None,
        stereo_audio=False,
        # Network configuration
        port=9999,
    ):
        """
        Convenience method that configures video/audio and starts the WebRTC server.
        
        This is equivalent to calling configure_video(), configure_audio(), and serve()
        with the appropriate parameters.
        
        Args:
            device: Video device identifier (default: None for synthetic frames).
                   Set to None to generate frames programmatically without a camera.
            format: Video format (default: "avfoundation" for macOS). 
                   Ignored if device=None.
            fps: Frame rate (default: 30)
            size: Frame size as "WIDTHxHEIGHT" string (default: "640x480")
            port: Port to run WebRTC server on (default: 9999)
            stereo_video: If True, stream side-by-side stereo video (default: False)
            stereo_audio: If True, stream stereo audio (2 channels), otherwise mono (default: False)
            audio_device: Audio device identifier (default: None, no audio).
                         Example: ":0" for default microphone on macOS.
            audio_format: Audio input format (default: None, auto-detect).
                         Use "avfoundation" for macOS audio input.
            audio_sample_rate: Optional sample rate (Hz) for synthetic audio frames.
                                Defaults to 48000 when not specified.
        
        Note:
            - Use register_frame_callback() to add custom frame processing before calling this
            - Use register_audio_callback() to add custom audio processing before calling this
            - If device=None, the callback MUST generate frames (not just modify them)
            - Without device, callback receives a blank frame to populate
        
        Example::
        
            # Simple usage (equivalent to configure_video + serve)
            streamer.start_streaming(device="0:none", format="avfoundation")
            
            # With callbacks
            streamer.register_frame_callback(my_frame_processor)
            streamer.start_streaming(device=None, size="1280x720", fps=60)
        """
        # Always configure video (start_streaming implies video streaming)
        self.configure_video(
            device=device,
            format=format,
            size=size,
            fps=fps,
            stereo=stereo_video,
        )
        
        # Configure audio if audio_device is specified or audio_callback is registered
        if audio_device is not None or self.audio_callback is not None:
            self.configure_audio(
                device=audio_device,
                format=audio_format,
                sample_rate=audio_sample_rate,
                stereo=stereo_audio,
            )
        
        if self._cross_network_mode:
            # If not already connected (e.g. via start_webrtc), connect now
            if not self._signaling_connected and not (self._signaling_ws and self._signaling_ws.keep_running):
                self._start_signaling_connection()
            
            self._log('[CROSS-NETWORK] Signaling started. WebRTC handshake will proceed automatically.', force=True)
            return
        else:
            # Start the video streaming server (WebRTC)
            asyncio.run(self.start_async_server(self._webrtc_port))
    
    def _send_webrtc_info_via_grpc(self, host, port, stereo_video=False, stereo_audio=False, audio_enabled=False, video_enabled=False, sim_enabled=False):
        """Send WebRTC server info to VisionOS by opening a new gRPC connection."""
        try:
            ip_parts = host.split('.')
            
            self._log(f"[GRPC] Opening new connection to send WebRTC info (video={video_enabled}, audio={audio_enabled}, sim={sim_enabled})...")
            
            # Create a new connection just to send WebRTC server info
            with grpc.insecure_channel(f"{self.ip}:12345") as channel:
                try:
                    grpc.channel_ready_future(channel).result(timeout=3)
                    
                    # Create WebRTC info message with marker 999.0
                    webrtc_msg = handtracking_pb2.HandUpdate()
                    webrtc_msg.Head.m00 = 999.0  # WebRTC info marker
                    webrtc_msg.Head.m01 = float(ip_parts[0])
                    webrtc_msg.Head.m02 = float(ip_parts[1])
                    webrtc_msg.Head.m03 = float(ip_parts[2])
                    webrtc_msg.Head.m10 = float(ip_parts[3])
                    webrtc_msg.Head.m11 = float(port)
                    webrtc_msg.Head.m12 = 1.0 if stereo_video else 0.0  # Stereo video flag
                    webrtc_msg.Head.m13 = 1.0 if stereo_audio else 0.0  # Stereo audio flag
                    webrtc_msg.Head.m20 = 1.0 if audio_enabled else 0.0  # Audio enabled flag
                    webrtc_msg.Head.m21 = 1.0 if video_enabled else 0.0  # Video enabled flag
                    webrtc_msg.Head.m22 = 1.0 if sim_enabled else 0.0    # Sim enabled flag
                    webrtc_msg.Head.m30 = float(LIBRARY_VERSION_CODE)    # Library version for compatibility check
                    
                    self._webrtc_info_to_send = webrtc_msg

                    stub = handtracking_pb2_grpc.HandTrackingServiceStub(channel)
                    # Send the message and immediately close
                    responses = stub.StreamHandUpdates(webrtc_msg)
                    # Just start the stream but don't consume responses
                    next(responses)  # Get first response to ensure message was sent
                    
                    self._log(f"[GRPC] WebRTC info sent: {host}:{port} (video={video_enabled}, audio={audio_enabled}, sim={sim_enabled})")
                except Exception as e:
                    self._log(f"[GRPC] Warning: Error sending WebRTC info: {e}")
                    
        except Exception as e:
            self._log(f"[GRPC] Error in _send_webrtc_info_via_grpc: {e}", force=True)
    

if __name__ == "__main__": 

    streamer = VisionProStreamer(ip = '10.29.230.57')
    while True: 

        latest = streamer.get_latest()
        print(latest) 