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
from copy import deepcopy
from typing import Optional, Tuple, List, Dict, Any, Callable, TYPE_CHECKING
from pathlib import Path
from aiortc import VideoStreamTrack, AudioStreamTrack
from av import VideoFrame, AudioFrame
from avp_stream.mujoco_msg import mujoco_ar_pb2, mujoco_ar_pb2_grpc
from scipy.spatial.transform import Rotation as R


YUP2ZUP = np.array([[[1, 0, 0, 0], 
                    [0, 0, -1, 0], 
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]]], dtype = np.float64)


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


class VisionProStreamer:

    def __init__(self, ip, record=True, ht_backend="grpc", benchmark_quiet=False, verbose=False, origin="avp"):
        """Initialize the VisionProStreamer.

        Parameters
        ----------
        ip : str
            Vision Pro IP address (shown inside the Tracking Streamer app – pick the wired one when using `setup-avp-wired`).
        record : bool, default True
            If True, every processed hand tracking sample is appended to an internal list retrievable via `get_recording()`.
        ht_backend : {"grpc","webrtc"}, default "grpc"
            Transport used for hand tracking. Use "webrtc" for lower latency after a WebRTC session is established.
        benchmark_quiet : bool, default False
            Suppress benchmark printouts (see latency measurements in `latency_test.py`).
        verbose : bool, default False
            If True, print detailed status messages. If False (default), only critical messages are shown.
        origin : {"avp", "sim"}, default "avp"
            Coordinate frame origin for hand tracking data.
            - "avp": Hand tracking in Vision Pro's native coordinate frame (default).
            - "sim": Hand tracking relative to the simulation's attach_to position.
              Useful when you want hand positions in the same frame as your MuJoCo scene.

        Notes
        -----
        Instantiates background threads immediately: (1) gRPC hand tracking stream and (2) HTTP info server for WebRTC discovery.
        """

        # Vision Pro IP 
        self.ip = ip
        self.record = record 
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
        self._webrtc_loop = None  # Event loop for WebRTC thread
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
        
        # Sim benchmark state
        self._sim_benchmark_enabled = False
        self._sim_benchmark_seq = 0

        self._start_info_server()  # Start HTTP endpoint immediately
        self._start_hand_tracking()  # Start hand tracking stream
    
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
            
            transformations = {
                "left_wrist": left_wrist,
                "right_wrist": right_wrist,
                "left_fingers": process_matrices(hand_update.left_hand.skeleton.jointMatrices),
                "right_fingers": process_matrices(hand_update.right_hand.skeleton.jointMatrices),
                "head": head,
                "left_pinch_distance": get_pinch_distance(hand_update.left_hand.skeleton.jointMatrices),
                "right_pinch_distance": get_pinch_distance(hand_update.right_hand.skeleton.jointMatrices),
            }

            transformations["right_wrist_roll"] = get_wrist_roll(transformations["right_wrist"])
            transformations["left_wrist_roll"] = get_wrist_roll(transformations["left_wrist"])
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

        @channel.on("open")
        def _on_open():
            self._log("[WEBRTC] Sim-poses data channel opened", force=True)
            self._webrtc_sim_ready = True
            # Start pose streaming once channel is ready
            if self._sim_config is not None:
                self._start_pose_streaming_webrtc()

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

    def get_latest(self, use_cache=False, cache_ms=10): 
        """Return the most recent tracking sample.

        Keys in the returned dict:
          - head (1,4,4 ndarray): Head pose (Z-up)
          - left_wrist / right_wrist (1,4,4 ndarray): Wrist poses
          - left_fingers / right_fingers (25,4,4 ndarray): Finger joints in wrist frame
          - left_pinch_distance / right_pinch_distance (float): Thumb–index pinch distance (m)
          - left_wrist_roll / right_wrist_roll (float): Axial wrist rotation (rad)

        :param use_cache: If True, return cached data if recent enough (reduces lock contention)
        :param cache_ms: Cache validity in milliseconds (default: 10ms)
        :return: Tracking dictionary or None if not yet available.
        :rtype: dict | None

        Example::
            streamer = VisionProStreamer(ip=avp_ip)
            sample = streamer.get_latest()
            if sample:
                left_pos = sample["left_wrist"][0, :3, 3]
        """
        if use_cache:
            current_time = time.perf_counter()
            if (self._latest_cached is not None and 
                (current_time - self._latest_cache_time) < (cache_ms / 1000.0)):
                return self._latest_cached
            
            # Cache expired, update it
            with self._latest_lock:
                self._latest_cached = self.latest
                self._latest_cache_time = current_time
                return self._latest_cached
        else:
            with self._latest_lock:
                return self.latest
        
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
    
    def set_origin(self, origin: str):
        """Set the coordinate frame origin for hand tracking data.
        
        Args:
            origin: Either "avp" or "sim".
                - "avp": Hand tracking in Vision Pro's native coordinate frame.
                - "sim": Hand tracking relative to the simulation's attach_to position.
                  Useful when you want hand positions in the same frame as your MuJoCo scene.
        
        Example::
        
            streamer = VisionProStreamer(ip=avp_ip)
            streamer.configure_sim("scene.xml", model, data, relative_to=[0, 0, 0.8, 90])
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
    
    def configure_sim(
        self,
        xml_path: str,
        model,
        data,
        relative_to: Optional[List[float]] = None,
        grpc_port: int = 50051,
        force_reload: bool = False,
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
        
        Example::
        
            import mujoco
            model = mujoco.MjModel.from_xml_path("scene.xml")
            data = mujoco.MjData(model)
            
            streamer.configure_sim("scene.xml", model, data, relative_to=[0, 0, 1, 1, 0, 0, 0])
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
        
        # Build body name mapping
        self._mujoco_bodies = {}
        for i in range(model.nbody):
            body_name = model.body(i).name
            if body_name:
                self._mujoco_bodies[body_name] = i
        
        self._sim_config = {
            "xml_path": xml_path,
            "attach_to": attach_to,
            "grpc_port": grpc_port,
            "force_reload": force_reload,
        }
        
        # Automatically switch to simulation-relative coordinates
        self.set_origin("sim")
        
        self._log(f"[CONFIG] Simulation configured: {xml_path} ({len(self._mujoco_bodies)} bodies)")
        if attach_to:
            self._log(f"  Position: [{attach_to[0]:.2f}, {attach_to[1]:.2f}, {attach_to[2]:.2f}]")
    
    def update_sim(self):
        """
        Sync current MuJoCo simulation state to the VisionPro.
        
        Call this after each simulation step to update the 3D visualization.
        The poses are streamed via gRPC to the MuJoCo AR viewer on VisionPro.
        
        Example::
        
            while True:
                mujoco.mj_step(model, data)
                streamer.update_sim()
        """
        if self._mujoco_model is None or self._mujoco_data is None:
            self._log("[SIM] Warning: No simulation configured. Call configure_sim() first.", force=True)
            return
        
        if not self._pose_stream_running:
            # Pose streaming not started yet
            return
        
        # Get current poses
        poses = self._get_mujoco_poses()
        
        # Update the current poses that the streaming thread will send
        with self._pose_stream_lock:
            self._current_poses = poses
    
    def _get_mujoco_poses(self) -> Dict[str, Dict[str, Any]]:
        """Get current MuJoCo body poses as a dictionary."""
        body_dict = {}
        model = self._mujoco_model
        data = self._mujoco_data
        
        for body_name, body_id in self._mujoco_bodies.items():
            if "world" in body_name.lower():
                continue  # Skip world body
            
            xpos = deepcopy(data.body(body_id).xpos.tolist())
            xquat = deepcopy(data.body(body_id).xquat.tolist())
            
            # Clean body name for Swift compatibility
            clean_name = body_name.replace('/', '').replace('-', '') if body_name else body_name
            body_dict[clean_name] = {
                "xpos": xpos,
                "xquat": xquat,
            }
        
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
        """Start the actual pose streaming loop via WebRTC."""
        if self._pose_stream_running:
            return
        
        self._log("[SIM] Starting pose streaming via WebRTC data channel...", force=True)
        self._pose_stream_running = True
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
        self._log("[SIM] Pose streaming via WebRTC started!", force=True)
        frame_count = 0
        
        try:
            while self._pose_stream_running:
                if not self._webrtc_sim_ready or self._webrtc_sim_channel is None:
                    time.sleep(0.1)
                    continue
                
                with self._pose_stream_lock:
                    current_poses = self._current_poses.copy()
                
                if current_poses:
                    # Build compact JSON message for poses
                    # Format: {"t": timestamp, "p": {"body_name": [x,y,z,qx,qy,qz,qw], ...}}
                    # With benchmark: {"t": ..., "p": {...}, "b": {"s": seq, "t": sent_ms}}
                    pose_dict = {}
                    for body_name, pose_data in current_poses.items():
                        if not body_name:
                            continue
                        # Pack position and quaternion into a list
                        # [x, y, z, qx, qy, qz, qw] - 7 floats per body
                        xpos = pose_data["xpos"]
                        xquat = pose_data["xquat"]  # MuJoCo: w,x,y,z
                        pose_dict[body_name] = [
                            round(xpos[0], 5), round(xpos[1], 5), round(xpos[2], 5),
                            round(xquat[1], 5), round(xquat[2], 5), round(xquat[3], 5), round(xquat[0], 5)  # x,y,z,w
                        ]
                    
                    if pose_dict:
                        msg_data = {"t": time.time(), "p": pose_dict}
                        
                        # Add benchmark data if enabled
                        if self._sim_benchmark_enabled:
                            sent_ms = int((time.perf_counter() - self._benchmark_epoch) * 1000)
                            msg_data["b"] = {
                                "s": self._sim_benchmark_seq,
                                "t": sent_ms
                            }
                            self._sim_benchmark_seq += 1
                        
                        message = json.dumps(msg_data)
                        try:
                            # Send using the WebRTC event loop's thread-safe call
                            if self._webrtc_loop is not None:
                                self._webrtc_loop.call_soon_threadsafe(
                                    self._webrtc_sim_channel.send, message
                                )
                            else:
                                self._webrtc_sim_channel.send(message)
                            frame_count += 1
                            if frame_count == 1:
                                self._log(f"[SIM] First sim pose sent via WebRTC ({len(pose_dict)} bodies)")
                            elif frame_count % 500 == 0:
                                self._log(f"[SIM] Sent {frame_count} sim pose updates")
                        except Exception as e:
                            self._log(f"[SIM] Warning: Failed to send pose via WebRTC: {e}")
                
                time.sleep(0.008)  # ~120 Hz (faster than gRPC was)
        
        except Exception as e:
            if self._pose_stream_running:
                self._log(f"[SIM] Error: Pose streaming error: {e}", force=True)
            self._pose_stream_running = False
        
        self._log(f"[SIM] Pose streaming stopped (sent {frame_count} updates)")
    
    def _load_and_send_scene(self):
        """Load USDZ scene and send to VisionPro via gRPC."""
        if self._sim_config is None:
            return False
        
        xml_path = self._sim_config["xml_path"]
        attach_to = self._sim_config["attach_to"]
        grpc_port = self._sim_config["grpc_port"]
        force_reload = self._sim_config.get("force_reload", False)
        
        # Check if USDZ already exists
        usdz_path = xml_path.replace('.xml', '.usdz')
        if not os.path.exists(usdz_path) or force_reload:
            try:
                usdz_path = self._convert_to_usdz(xml_path)
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
    
    def _convert_to_usdz(self, xml_path: str) -> str:
        """Convert MuJoCo XML to USDZ file (requires mujoco_usd_converter)."""
        try:
            import mujoco_usd_converter
            import usdex.core
            from pxr import Usd, UsdUtils
        except ImportError:
            raise ImportError("USDZ conversion requires mujoco_usd_converter. Try: pip install mujoco-usd-converter")
        
        converter = mujoco_usd_converter.Converter()
        usd_output_path = xml_path.replace('.xml', '_usd')
        usdz_output_path = xml_path.replace('.xml', '.usdz')
        
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
        configuration set by configure_video(), configure_audio(), and configure_sim().
        
        Note: Consider using start_webrtc() instead for clarity.
        
        Args:
            port: WebRTC server port (default: 9999)
        
        Example (video only)::
        
            streamer.configure_video(device=None, size="1280x720")
            streamer.start_webrtc()
        
        Example (simulation only)::
        
            streamer.configure_sim("scene.xml", model, data)
            streamer.start_webrtc()
        
        Example (video + simulation)::
        
            streamer.configure_video(device=None, size="1280x720")
            streamer.configure_sim("scene.xml", model, data)
            streamer.start_webrtc()
        """
        self._webrtc_port = port
        
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
                config = RTCConfiguration(
                    iceServers=[RTCIceServer(urls=["stun:stun.l.google.com:19302"])]
                )
                pc = RTCPeerConnection(configuration=config)
                
                @pc.on("iceconnectionstatechange")
                async def on_ice_state_change():
                    streamer_instance._log(f"[WEBRTC] ICE state ({client_addr}): {pc.iceConnectionState}", force=True)
                    if pc.iceConnectionState == "connected":
                        streamer_instance._log(f"[WEBRTC] Connection established ({client_addr}) - video should flow now", force=True)
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
                    sim_channel = pc.createDataChannel(
                        "sim-poses",
                        ordered=False,
                        maxRetransmits=0,
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

    def start_webrtc(self, port: int = 9999):
        """
        Start WebRTC streaming to Vision Pro.
        
        This starts the outbound media streaming (video/audio/simulation) to Vision Pro
        using the configuration set by configure_video(), configure_audio(), and configure_sim().
        
        Note: Hand tracking data flows automatically via gRPC when VisionProStreamer is
        instantiated. This method starts the *outbound* streaming for visual/audio feedback.
        
        Args:
            port: WebRTC server port (default: 9999)
        
        Example::
        
            streamer = VisionProStreamer(ip=avp_ip)  # Hand tracking starts automatically
            
            streamer.configure_video(device=None, size="1280x720")
            streamer.start_webrtc()  # Start video streaming to Vision Pro
        """
        self.serve(port=port)

    def wait_for_sim_channel(self, timeout: float = 30.0) -> bool:
        """
        Wait for the WebRTC sim-poses data channel to open.
        
        Args:
            timeout: Maximum time to wait in seconds (default: 30)
        
        Returns:
            True if channel opened successfully, False on timeout
        
        Example::
        
            streamer.configure_sim("scene.xml", model, data)
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
        while not self._webrtc_sim_ready:
            if time.time() - start_time > timeout:
                self._log(f"[WEBRTC] Warning: Timeout waiting for sim-poses channel (waited {timeout}s)", force=True)
                return False
            time.sleep(0.1)
        
        return True

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
        
        # Start the WebRTC server
        self.serve(port=port)
    
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