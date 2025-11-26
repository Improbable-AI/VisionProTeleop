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
import struct
try:
    import DracoPy
except ImportError:
    DracoPy = None

from aiortc import VideoStreamTrack, AudioStreamTrack, RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
from aiortc.contrib.media import MediaPlayer
from av import VideoFrame, AudioFrame
import logging


YUP2ZUP = np.array([[[1, 0, 0, 0], 
                    [0, 0, -1, 0], 
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]]], dtype = np.float64)


class VisionProStreamer:

    def __init__(self, ip, record=True, ht_backend="grpc", benchmark_quiet=False):
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

        Notes
        -----
        Instantiates background threads immediately: (1) gRPC hand tracking stream and (2) HTTP info server for WebRTC discovery.
        """

        # Vision Pro IP 
        self.ip = ip
        self.record = record 
        self.ht_backend = ht_backend.lower()
        if self.ht_backend not in {"grpc", "webrtc"}:
            raise ValueError(f"Unsupported ht_backend '{ht_backend}'. Expected 'grpc' or 'webrtc'.")

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
        self._av_sync_epoch = None  # Shared A/V timing reference

        self._latest_lock = Lock()
        self._webrtc_hand_channel = None
        self._webrtc_hand_ready = False
        self._benchmark_epoch = time.perf_counter()
        self._benchmark_condition = Condition()
        self._benchmark_events = {}

        self._point_cloud_enabled = False
        self._point_cloud_channel = None
        
        # Configuration state
        self._video_config = None
        self._audio_config = None

        self._start_info_server()  # Start HTTP endpoint immediately
        self._start_hand_tracking()  # Start hand tracking stream

    def _process_hand_update(self, hand_update, source="grpc"):
        if getattr(hand_update.Head, "m00", 0.0) == 777.0:
            self._handle_benchmark_response(hand_update)
            return

        if source == "grpc" and self.ht_backend == "webrtc" and self._webrtc_hand_ready:
            # Prefer WebRTC data once channel is active.
            return

        try:
            transformations = {
                "left_wrist": self.axis_transform @ process_matrix(hand_update.left_hand.wristMatrix),
                "right_wrist": self.axis_transform @ process_matrix(hand_update.right_hand.wristMatrix),
                "left_fingers": process_matrices(hand_update.left_hand.skeleton.jointMatrices),
                "right_fingers": process_matrices(hand_update.right_hand.skeleton.jointMatrices),
                "head": rotate_head(self.axis_transform @ process_matrix(hand_update.Head)),
                "left_pinch_distance": get_pinch_distance(hand_update.left_hand.skeleton.jointMatrices),
                "right_pinch_distance": get_pinch_distance(hand_update.right_hand.skeleton.jointMatrices),
            }

            transformations["right_wrist_roll"] = get_wrist_roll(transformations["right_wrist"])
            transformations["left_wrist_roll"] = get_wrist_roll(transformations["left_wrist"])
        except Exception as exc:
            print(f"⚠️  Failed to process hand update from {source}: {exc}")
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
        }

        with self._benchmark_condition:
            self._benchmark_events[sequence_id] = event
            self._benchmark_condition.notify_all()

        if not self.benchmark_quiet:
            if event["round_trip_ms"] is not None:
                print(f"🧪 Benchmark seq={sequence_id} round-trip={event['round_trip_ms']} ms")
            else:
                print(f"🧪 Benchmark seq={sequence_id} received (missing sent timestamp)")

    def _handle_webrtc_hand_message(self, message):
        if isinstance(message, str):
            print("⚠️  Received text message on hand data channel; ignoring")
            return

        if isinstance(message, memoryview):
            message = message.tobytes()

        update = handtracking_pb2.HandUpdate()
        try:
            update.ParseFromString(message)
        except Exception as exc:
            print(f"⚠️  Could not decode hand update from WebRTC data channel: {exc}")
            return

        self._process_hand_update(update, source="webrtc")
        if not self._webrtc_hand_ready:
            print("🟢 WebRTC hand data flow established")
            self._webrtc_hand_ready = True

    def _register_webrtc_data_channel(self, channel):
        self._webrtc_hand_channel = channel

        @channel.on("open")
        def _on_open():
            print("🟢 WebRTC hand data channel opened")

        @channel.on("close")
        def _on_close():
            print("🔴 WebRTC hand data channel closed")
            self._webrtc_hand_ready = False
            self._webrtc_hand_channel = None

        @channel.on("message")
        def _on_message(message):
            self._handle_webrtc_hand_message(message)

    def _start_hand_tracking(self): 
        """Start the hand tracking gRPC stream in a background thread."""
        stream_thread = Thread(target = self.stream, daemon=True)
        stream_thread.start() 
        
        print(f"Hand tracking backend set to {self.ht_backend.upper()}")
        if self.ht_backend == "webrtc":
            print("Note: WebRTC hand tracking requires an active WebRTC session (start_streaming).")
        print('Waiting for hand tracking data...')
        retry_count = 0
        while self.latest is None: 
            time.sleep(0.1)
            retry_count += 1
            if retry_count > 100:  # 10 seconds timeout
                print('WARNING: No data received yet. Is the VisionOS app running?')
                retry_count = 0
        
        print(' == DATA IS FLOWING IN! ==')
        print('Ready to start streaming.') 


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
                print(f"Attempting to connect to {self.ip}:12345 (attempt {attempt + 1}/{max_retries})...")
                channel = grpc.insecure_channel(f"{self.ip}:12345")
                
                try:
                    # Wait for channel to be ready
                    print(f"  Waiting for channel to be ready (timeout: 5s)...")
                    grpc.channel_ready_future(channel).result(timeout=5)
                    print("✓ gRPC channel ready")
                except Exception as channel_error:
                    print(f"⚠️  Channel not ready: {type(channel_error).__name__}: {channel_error}")
                    channel.close()
                    if attempt < max_retries - 1:
                        time.sleep(retry_delay)
                        retry_delay *= 2
                    continue
                    
                print("  Creating stub and starting stream...")
                stub = handtracking_pb2_grpc.HandTrackingServiceStub(channel)
                
                # Add WebRTC info to metadata if available
                metadata = []
                if hasattr(self, '_webrtc_info_to_send'):
                    msg = self._webrtc_info_to_send
                    # Encode WebRTC info in metadata
                    webrtc_data = f"{msg.Head.m01}|{msg.Head.m02}|{msg.Head.m03}|{msg.Head.m10}|{msg.Head.m11}"
                    metadata.append(('webrtc-info', webrtc_data))
                    print(f"📤 Adding WebRTC info to gRPC metadata: {webrtc_data}")
                
                responses = stub.StreamHandUpdates(request, metadata=metadata if metadata else None)
                print("✓ Stream established, waiting for responses...")
                
                for response in responses:
                    self._process_hand_update(response, source="grpc")
                
                # If we get here, connection was successful
                channel.close()
                return
                    
            except grpc.RpcError as e:
                if attempt < max_retries - 1:
                    print(f"⚠️  Connection failed: {e.code()}. Retrying in {retry_delay}s...")
                    time.sleep(retry_delay)
                    retry_delay *= 2  # Exponential backoff
                else:
                    print(f"❌ Failed to connect after {max_retries} attempts")
                    print(f"Error: {e}")
                    print("\nMake sure:")
                    print("  1. VisionOS app is running")
                    print("  2. You pressed 'Start' in the app")
                    print(f"  3. Vision Pro IP ({self.ip}) is correct")
                    print("  4. Both devices are on the same network")
                    break
            except Exception as e:
                error_type = type(e).__name__
                error_msg = str(e) if str(e) else "(no error message)"
                print(f"❌ An unexpected error occurred: {error_type}: {error_msg}")
                import traceback
                traceback.print_exc()
                if attempt < max_retries - 1:
                    print(f"  Retrying in {retry_delay}s...")
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
            streamer.start_streaming(device="0:none", format="avfoundation")
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
        print(f"✓ Frame callback registered: {callback.__name__ if hasattr(callback, '__name__') else 'anonymous function'}")
    
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
        print(f"✓ Audio callback registered: {callback.__name__ if hasattr(callback, '__name__') else 'anonymous function'}")
    
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

    def reset_benchmark_epoch(self, epoch=None):
        if epoch is None:
            epoch = time.perf_counter()
        with self._benchmark_condition:
            self._benchmark_epoch = epoch
            self._benchmark_events.clear()
        return self._benchmark_epoch

    def start_pointcloud_streaming(self):
        """
        Enable point cloud streaming. 
        
        This must be called before start_streaming() to ensure the data channel is created.
        Once streaming starts, use update_pc() to send data.
        """
        if DracoPy is None:
            print("❌ DracoPy not installed. Cannot enable point cloud streaming.")
            print("   Run: pip install DracoPy")
            return
            
        self._point_cloud_enabled = True
        print("☁️ Point cloud streaming enabled (will start with WebRTC)")

    # Alias for new API
    def enable_point_cloud(self):
        """Enable point cloud streaming (alias for start_pointcloud_streaming)."""
        self.start_pointcloud_streaming()


    def update_pc(self, points, colors=None, transform=None, quantize=False, use_draco=False):
        """
        Send a new point cloud frame.
        points: (N, 3) numpy array of float32
        colors: (N, 3) numpy array of uint8 (optional)
        transform: (4, 4) numpy array of float32 (optional)
        quantize: bool, if True, convert points to float16 (half precision)
        use_draco: bool, if True, use Draco compression (overrides quantize)
        """
        if self._point_cloud_channel is None or self._point_cloud_channel.readyState != "open":
            # Rate limit warnings?
            return

        try:
            # 1. Send Transform if provided (Header 0x01)
            if transform is not None:
                # Ensure 4x4 float32
                tf_flat = transform.astype(np.float32).flatten()
                if len(tf_flat) != 16:
                    print("⚠️ Invalid transform shape")
                else:
                    header = bytes([0x01])
                    payload = header + tf_flat.tobytes()
                    
                    if hasattr(self, '_webrtc_loop') and self._webrtc_loop is not None:
                        self._webrtc_loop.call_soon_threadsafe(self._point_cloud_channel.send, payload)
                    else:
                        self._point_cloud_channel.send(payload) # Fallback (might fail)


            # 2. Encode and Send Points
            if points is not None and len(points) > 0:
                
                if use_draco and DracoPy is not None:
                    # Draco Encoding
                    encoded_bytes = DracoPy.encode(
                        points, 
                        colors=colors,
                        compression_level=5, 
                        quantization_bits=10
                    )
                    # Header 0x08: Draco Data
                    header = bytes([0x08])
                    
                elif quantize:
                    # Float16 Quantization
                    points_bytes = points.astype(np.float16).tobytes()
                    
                    if colors is not None:
                        # Header 0x06: Points (Float16) + Colors
                        header = bytes([0x06])
                        colors_bytes = colors.astype(np.uint8).tobytes()
                        encoded_bytes = points_bytes + colors_bytes
                    else:
                        # Header 0x05: Points (Float16) only
                        header = bytes([0x05])
                        encoded_bytes = points_bytes
                else:
                    # Float32 (Standard)
                    points_bytes = points.astype(np.float32).tobytes()
                    
                    if colors is not None:
                        # Header 0x03: Points + Colors
                        header = bytes([0x03])
                        colors_bytes = colors.astype(np.uint8).tobytes()
                        encoded_bytes = points_bytes + colors_bytes
                    else:
                        # Header 0x02: Points only
                        header = bytes([0x02])
                        encoded_bytes = points_bytes
                
                payload = header + encoded_bytes

                
                # Use call_soon_threadsafe to send from the main thread to the asyncio loop
                if hasattr(self, '_webrtc_loop') and self._webrtc_loop is not None:
                    self._webrtc_loop.call_soon_threadsafe(self._point_cloud_channel.send, payload)
                    # print("DEBUG: Sent point cloud payload via threadsafe call")
                else:
                    print("⚠️ WebRTC loop not available, cannot send point cloud")
                
        except Exception as e:
            print(f"⚠️ Error sending point cloud: {e}")

            import traceback
            traceback.print_exc()

    def send_benchmark_pc(self, points, colors, sequence_id, timestamp, quantize=False, use_draco=False):
        """
        Send a benchmark point cloud with header 0x04 (Float32), 0x07 (Float16), or 0x09 (Draco).
        Payload: [Header] + [SeqID (4)] + [Timestamp (8)] + [Point Cloud Data]
        Returns: encoding_time_ms (float)
        """
        if not self._point_cloud_enabled or self._point_cloud_channel is None:
            return 0.0

        try:
            encoding_start = time.perf_counter()
            
            # Encode points/colors
            if use_draco and DracoPy is not None:
                pc_payload = DracoPy.encode(
                    points, 
                    colors=colors,
                    compression_level=5, 
                    quantization_bits=10
                )
                header = bytes([0x09]) # Benchmark Draco
            elif quantize:
                points_bytes = points.astype(np.float16).tobytes()
                header = bytes([0x07]) # Benchmark Float16
                
                if colors is not None:
                    colors_bytes = colors.astype(np.uint8).tobytes()
                    pc_payload = points_bytes + colors_bytes
                else:
                    pc_payload = points_bytes
            else:
                points_bytes = points.astype(np.float32).tobytes()
                header = bytes([0x04]) # Benchmark Float32
                
                if colors is not None:
                    colors_bytes = colors.astype(np.uint8).tobytes()
                    pc_payload = points_bytes + colors_bytes
                else:
                    pc_payload = points_bytes
            
            encoding_end = time.perf_counter()
            encoding_time_ms = (encoding_end - encoding_start) * 1000.0
                
            # SeqID (4 bytes, little endian)
            seq_bytes = struct.pack('<I', sequence_id)
            # Timestamp (8 bytes, little endian, double)
            ts_bytes = struct.pack('<d', float(timestamp))
            
            payload = header + seq_bytes + ts_bytes + pc_payload
            
            if hasattr(self, '_webrtc_loop') and self._webrtc_loop is not None:
                self._webrtc_loop.call_soon_threadsafe(self._point_cloud_channel.send, payload)
                # print(f"DEBUG: Sent benchmark PC payload ({len(payload)} bytes)")
            else:
                self._point_cloud_channel.send(payload)
                # print(f"DEBUG: Sent benchmark PC payload ({len(payload)} bytes)")
                
            return encoding_time_ms
                
        except Exception as e:
            print(f"⚠️ Error sending benchmark PC: {e}")
            return 0.0


    def update_stream_resolution(self, size):
        """Request the video track to emit frames at a new resolution.

        Example
        -------
        .. code-block:: python

            streamer.update_stream_resolution("1024x768")
        """
        if self.camera is None:
            print("⚠️  Cannot update stream resolution before starting video streaming")
            return

        try:
            width, height = map(int, size.lower().split("x", 1))
        except ValueError:
            print(f"⚠️  Invalid resolution '{size}' (expected WIDTHxHEIGHT)")
            return

        if hasattr(self.camera, "set_resolution"):
            self.camera.set_resolution((width, height))

        if self.webrtc_info is not None:
            self.webrtc_info["size"] = size

        print(f"🔄 Updated WebRTC stream resolution to {width}x{height}")

    def wait_for_benchmark_event(self, sequence_id, timeout=2.0):
        deadline = time.perf_counter() + timeout
        with self._benchmark_condition:
            while True:
                event = self._benchmark_events.pop(sequence_id, None)
                if event is not None:
                    return event
                remaining = deadline - time.perf_counter()
                if remaining <= 0:
                    # Drop any stale data for this sequence if it arrives late.
                    self._benchmark_events.pop(sequence_id, None)
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
            def do_GET(self):
                client_ip = self.client_address[0]
                print(f"📥 [HTTP] Request from {client_ip}: {self.path}")
                
                if self.path == '/webrtc_info':
                    if streamer_instance.webrtc_info:
                        print(f"✅ [HTTP] Sending WebRTC info to {client_ip}: {streamer_instance.webrtc_info}")
                        self.send_response(200)
                        self.send_header('Content-type', 'application/json')
                        self.end_headers()
                        self.wfile.write(json.dumps(streamer_instance.webrtc_info).encode())
                    else:
                        print(f"⚠️ [HTTP] WebRTC not started yet, sending 404 to {client_ip}")
                        self.send_response(404)
                        self.send_header('Content-type', 'application/json')
                        self.end_headers()
                        self.wfile.write(json.dumps({"error": "WebRTC not started"}).encode())
                else:
                    print(f"❌ [HTTP] Unknown path from {client_ip}: {self.path}")
                    self.send_response(404)
                    self.end_headers()
            
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
    
    def configure_video(self, device=None, format=None, size="640x480", fps=30, stereo_video=False):
        """
        Configure video streaming settings.
        
        Args:
            device: Video device identifier (default: None).
            format: Video format (default: None).
            size: Frame size "WIDTHxHEIGHT" (default: "640x480").
            fps: Frame rate (default: 30).
            stereo_video: Enable side-by-side stereo (default: False).
        """
        self._video_config = {
            "device": device,
            "format": format,
            "size": size,
            "fps": fps,
            "stereo_video": stereo_video
        }
        print(f"📹 Video configured: {size} @ {fps}fps (device={device})")

    def configure_audio(self, device=None, format=None, sample_rate=None, stereo_audio=False):
        """
        Configure audio streaming settings.
        
        Args:
            device: Audio device identifier (default: None).
            format: Audio format (default: None).
            sample_rate: Sample rate in Hz (default: 48000).
            stereo_audio: Enable stereo audio (default: False).
        """
        self._audio_config = {
            "device": device,
            "format": format,
            "sample_rate": sample_rate or 48000,
            "stereo_audio": stereo_audio
        }
        print(f"🎤 Audio configured: {sample_rate}Hz (device={device})")

    def serve(self, port=9999):
        """
        Start the WebRTC signaling server and wait for connections.
        Uses the configurations set by configure_video/audio/point_cloud.
        """
        # Get local IP
        local_ip = self.get_local_ip()
        print(f"Starting WebRTC server on {local_ip}:{port}")
        
        # Prepare WebRTC info
        video_conf = self._video_config or {}
        audio_conf = self._audio_config or {}
        
        stereo_video = video_conf.get("stereo_video", False)
        stereo_audio = audio_conf.get("stereo_audio", False)
        audio_enabled = self._audio_config is not None or self.audio_callback is not None
        size = video_conf.get("size", "640x480")
        audio_sample_rate = audio_conf.get("sample_rate", 48000)
        
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
        print(f"📤 Sending WebRTC server info via gRPC...")
        self._send_webrtc_info_via_grpc(local_ip, port, stereo_video, stereo_audio, audio_enabled)
        
        # Create a new event loop for the WebRTC server in a separate thread
        def start_async_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._webrtc_loop = loop
            
            async def run_peer_custom(reader, writer):
                client_addr = writer.get_extra_info('peername')
                print(f"🔗 VisionOS client connected from {client_addr}")
                
                try:
                    config = RTCConfiguration(
                        iceServers=[RTCIceServer(urls=["stun:stun.l.google.com:19302"])]
                    )
                    pc = RTCPeerConnection(configuration=config)
                    
                    @pc.on("iceconnectionstatechange")
                    async def on_ice_state_change():
                        print(f"ICE state ({client_addr}): {pc.iceConnectionState}")
                        if pc.iceConnectionState == "connected":
                            print(f"🎥 WebRTC connection established ({client_addr})")
                        if pc.iceConnectionState in ("failed", "closed"):
                            await pc.close()
                            writer.close()
                            await writer.wait_closed()
                    
                    print(f"DEBUG: Adding tracks for {client_addr}...")
                    # Add Video Track
                    if self._video_config:
                        try:
                            width, height = map(int, self._video_config["size"].split('x'))
                            processed_track = ProcessedVideoTrack(
                                self, # Pass streamer instance
                                self._video_config["device"],
                                self._video_config["format"],
                                self._video_config["fps"],
                                (width, height),
                                self.frame_callback
                            )
                            self.camera = processed_track
                            pc.addTrack(processed_track)
                            print("✓ Video track added")
                        except Exception as e:
                            print(f"❌ Error adding video track: {e}")
                            import traceback
                            traceback.print_exc()

                    # Add Audio Track
                    if self._audio_config or self.audio_callback:
                        try:
                            # Use config if available, else defaults
                            dev = self._audio_config.get("device") if self._audio_config else None
                            fmt = self._audio_config.get("format") if self._audio_config else None
                            sr = self._audio_config.get("sample_rate", 48000) if self._audio_config else 48000
                            stereo = self._audio_config.get("stereo_audio", False) if self._audio_config else False
                            
                            processed_audio = ProcessedAudioTrack(
                                self, # Pass streamer instance
                                dev,
                                fmt,
                                self.audio_callback,
                                stereo=stereo,
                                sample_rate=sr
                            )
                            pc.addTrack(processed_audio)
                            print("✓ Audio track added")
                        except Exception as e:
                            print(f"❌ Error adding audio track: {e}")

                    # Add Data Channels
                    if self.ht_backend == "webrtc":
                        hand_channel = pc.createDataChannel("hand-tracking", ordered=False, maxRetransmits=0)
                        self._register_webrtc_data_channel(hand_channel)
                        
                        @pc.on("datachannel")
                        def _on_remote_datachannel(channel):
                            if channel.label == "hand-tracking":
                                self._register_webrtc_data_channel(channel)

                    if self._point_cloud_enabled:
                        print("DEBUG: Creating pointCloud data channel...")
                        pc_channel = pc.createDataChannel("pointCloud", ordered=False, maxRetransmits=0)
                        self._point_cloud_channel = pc_channel
                        print(f"☁️ Created pointCloud data channel (Unreliable)")
                        
                        @pc_channel.on("message")
                        def on_pc_message(message):
                            if isinstance(message, bytes):
                                # Benchmark Echo: [SeqID (4)] + [Timestamp (8)] + [ProcessingTime (8)]
                                # Total 20 bytes
                                try:
                                    if len(message) >= 20:
                                        seq_id, timestamp, proc_time = struct.unpack("<Idd", message[:20])
                                        now_ms = (time.perf_counter() - self._benchmark_epoch) * 1000
                                        rtt = now_ms - timestamp
                                        
                                        # Store event
                                        with self._benchmark_condition:
                                            self._benchmark_events[seq_id] = {
                                                "rtt": rtt,
                                                "proc_time": proc_time * 1000, # Convert to ms
                                                "recv_ts": time.time() * 1000
                                            }
                                            self._benchmark_condition.notify_all()
                                        
                                        if not self.benchmark_quiet:
                                            print(f"⏱️ Benchmark Echo: Seq={seq_id}, RTT={rtt:.2f}ms, Proc={proc_time*1000:.2f}ms")
                                    elif len(message) >= 12:
                                        # Legacy support (just in case)
                                        seq_id, timestamp = struct.unpack("<Id", message[:12])
                                        now_ms = (time.perf_counter() - self._benchmark_epoch) * 1000
                                        rtt = now_ms - timestamp
                                        
                                        with self._benchmark_condition:
                                            self._benchmark_events[seq_id] = {
                                                "rtt": rtt,
                                                "proc_time": 0.0,
                                                "recv_ts": time.time() * 1000
                                            }
                                            self._benchmark_condition.notify_all()
                                        
                                        if not self.benchmark_quiet:
                                            print(f"⏱️ Benchmark Echo: Seq={seq_id}, RTT={rtt:.2f}ms")
                                except Exception as e:
                                    print(f"⚠️ Error parsing PC benchmark response: {e}")

                    # Offer/Answer exchange
                    print("DEBUG: Creating offer...")
                    offer = await pc.createOffer()
                    sdp = offer.sdp
                    if "m=video" in sdp:
                        import re
                        sdp = re.sub(r"(m=video.*\r\n)", r"\1b=AS:15000\r\n", sdp)
                    
                    print("DEBUG: Setting local description...")
                    offer = RTCSessionDescription(sdp=sdp, type=offer.type)
                    await pc.setLocalDescription(offer)
                    
                    # Wait for ICE
                    print("DEBUG: Waiting for ICE gathering...")
                    start_time = asyncio.get_event_loop().time()
                    while asyncio.get_event_loop().time() - start_time < 0.5:
                        if pc.iceGatheringState == "complete": break
                        if pc.iceGatheringState == "gathering": 
                            await asyncio.sleep(0.1); break
                        await asyncio.sleep(0.05)
                    print(f"DEBUG: ICE gathering state: {pc.iceGatheringState}")
                    
                    # Send Offer
                    print("DEBUG: Sending offer to client...")
                    offer_payload = {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
                    writer.write((json.dumps(offer_payload) + "\n").encode("utf-8"))
                    await writer.drain()
                    print("DEBUG: Offer sent.")
                except Exception as e:
                    print(f"❌ Error in run_peer_custom: {e}")
                    import traceback
                    traceback.print_exc()
                    return
                
                # Receive Answer
                line = await reader.readline()
                if not line:
                    await pc.close()
                    return
                
                answer_payload = json.loads(line.decode("utf-8"))
                answer = RTCSessionDescription(sdp=answer_payload["sdp"], type=answer_payload["type"])
                await pc.setRemoteDescription(answer)
                
                # Keep alive
                try:
                    while True: await asyncio.sleep(1)
                except asyncio.CancelledError: pass
                finally:
                    await pc.close()
                    writer.close()
                    await writer.wait_closed()

            async def run_server():
                server = await asyncio.start_server(run_peer_custom, "0.0.0.0", port, start_serving=True, reuse_address=True, reuse_port=True)
                print(f"WebRTC server listening on 0.0.0.0:{port}")
                async with server:
                    await server.serve_forever()
            
            loop.run_until_complete(run_server())
        
        webrtc_thread = Thread(target=start_async_server, daemon=True)
        webrtc_thread.start()
        time.sleep(0.5)
        print(f"✓ WebRTC server ready. VisionOS can connect.")

    def start_streaming(
        self,
        device=None,
        format=None,
        size="640x480",
        fps=30,
        stereo_video=False,
        audio_device=None,
        audio_format=None,
        audio_sample_rate=None,
        stereo_audio=False,
        port=9999,
    ):
        """
        Legacy wrapper for starting streaming. 
        Configures video/audio and starts the server.
        """
        self.configure_video(device, format, size, fps, stereo_video)
        if audio_device or self.audio_callback or stereo_audio:
            self.configure_audio(audio_device, audio_format, audio_sample_rate, stereo_audio)
        
        self.serve(port)
    
    def _send_webrtc_info_via_grpc(self, host, port, stereo_video=False, stereo_audio=False, audio_enabled=False):
        """Send WebRTC server info to VisionOS by opening a new gRPC connection."""
        try:
            ip_parts = host.split('.')
            
            print(f"📤 Opening new gRPC connection to send WebRTC info (stereo_video={stereo_video}, stereo_audio={stereo_audio}, audio_enabled={audio_enabled})...")
            
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
                    
                    self._webrtc_info_to_send = webrtc_msg

                    stub = handtracking_pb2_grpc.HandTrackingServiceStub(channel)
                    # Send the message and immediately close
                    responses = stub.StreamHandUpdates(webrtc_msg)
                    # Just start the stream but don't consume responses
                    next(responses)  # Get first response to ensure message was sent
                    
                    print(f"✓ WebRTC info sent via gRPC: {host}:{port} (stereo_video={stereo_video}, stereo_audio={stereo_audio}, audio_enabled={audio_enabled})")
                except Exception as e:
                    print(f"⚠️  Error sending WebRTC info via gRPC: {e}")
                    
        except Exception as e:
            print(f"❌ Error in _send_webrtc_info_via_grpc: {e}")
    

# Define custom audio track for audio processing
class ProcessedAudioTrack(AudioStreamTrack):
    kind = "audio"
    _track_counter = 0  # Class-level counter for unique track IDs
    
    def __init__(self, streamer, audio_device, audio_fmt, callback, stereo=False, sample_rate=None):
        super().__init__()
        self.streamer = streamer
        self.callback = callback
        self.use_microphone = audio_device is not None
        self.stereo = stereo
        self.sample_rate = sample_rate or 48000
        
        if self.use_microphone:
            # Use physical microphone
            fmt = audio_fmt if audio_fmt else "avfoundation"
            self.player = MediaPlayer(audio_device, format=fmt, options={})
            self.audio_track = self.player.audio
            channels = "stereo" if stereo else "mono"
            print(f"🎤 Audio initialized from device: {audio_device} ({channels})")
        else:
            # Generate frames programmatically
            self.player = None
            self.audio_track = None
            frame_duration_s = 0.02  # 20 ms
            self.samples_per_frame = max(1, int(self.sample_rate * frame_duration_s))
            self.frame_count = 0
            self.start_time = None
            channels = "stereo" if stereo else "mono"
            print(
                "🎤 Synthetic audio initialized "
                f"({self.sample_rate}Hz, {self.samples_per_frame} samples/frame, {channels}, 20ms frames)"
            )
    
    async def recv(self):
        if self.use_microphone:
            frame = await self.audio_track.recv()
            
            # Apply user callback if registered
            if self.callback is not None:
                try:
                    processed_frame = self.callback(frame)
                    return processed_frame
                except Exception as e:
                    print(f"⚠️  Audio callback error: {e}")
                    return frame
            else:
                return frame
        else:
            # Generate synthetic audio frame
            from av import AudioFrame
            import numpy as np
            
            # Synchronize with video timing if available
            if self.streamer._av_sync_epoch is None:
                self.streamer._av_sync_epoch = asyncio.get_event_loop().time()
            
            if self.start_time is None:
                self.start_time = self.streamer._av_sync_epoch
            
            # Create blank audio frame (silence)
            layout = 'stereo' if self.stereo else 'mono'
            frame = AudioFrame(format='s16', layout=layout, samples=self.samples_per_frame)
            frame.sample_rate = self.sample_rate
            frame.pts = self.frame_count * self.samples_per_frame
            frame.time_base = fractions.Fraction(1, self.sample_rate)
            
            # Initialize with silence
            bytes_per_sample = 2  # s16 = 2 bytes per sample
            channels = 2 if self.stereo else 1
            for p in frame.planes:
                p.update(bytes(self.samples_per_frame * bytes_per_sample * channels))
            
            # Precise timing control with jitter compensation
            expected_time = self.start_time + (self.frame_count * self.samples_per_frame / self.sample_rate)
            current_time = asyncio.get_event_loop().time()
            sleep_time = expected_time - current_time
            
            # Adaptive timing: sleep if ahead, reset if too far behind
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            elif sleep_time < -0.05:  # More than 50ms behind - reset to avoid drift
                self.start_time = current_time - (self.frame_count * self.samples_per_frame / self.sample_rate)
            
            # Apply callback if registered (callback should generate audio)
            if self.callback is not None:
                try:
                    processed_frame = self.callback(frame)
                    self.frame_count += 1
                    return processed_frame
                except Exception as e:
                    print(f"⚠️  Audio callback error: {e}")
                    import traceback
                    traceback.print_exc()
                    self.frame_count += 1
                    return frame
            else:
                # No callback - return silence
                self.frame_count += 1
                return frame

# Define custom video track for frame processing
class ProcessedVideoTrack(VideoStreamTrack):
    def __init__(self, streamer, device, fmt, framerate, resolution, callback):
        super().__init__()
        self.streamer = streamer
        self.callback = callback
        self.width, self.height = resolution
        self.use_camera = device is not None
        self.warmup_frames = 0  # Track warmup frames for encoder optimization
        
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
            print("🎥 Camera initialized, warming up encoder...")
        else:
            # Generate frames programmatically
            self.player = None
            self.video_track = None
            self.fps = framerate
            self.frame_count = 0
            self.start_time = None

    def set_resolution(self, resolution):
        width, height = resolution
        if width <= 0 or height <= 0:
            print(f"⚠️  Ignoring invalid resolution update: {width}x{height}")
            return
        self.width, self.height = width, height
        print(f"🔧 ProcessedVideoTrack target resolution set to {width}x{height}")
        
    async def recv(self):
        if self.use_camera:
            # Check if user provided a frame override
            if self.streamer.user_frame is not None:
                # Use user-provided frame instead of camera
                img = self.streamer.user_frame
                # Get timing from camera track if available
                try:
                    camera_frame = await self.video_track.recv()
                    pts = camera_frame.pts
                    time_base = camera_frame.time_base
                except:
                    # Fallback timing if camera fails
                    pts = None
                    time_base = None
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
                    print("✅ Encoder warmed up - optimal encoding should begin")
            
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
                    import traceback
                    traceback.print_exc()
                    print(f"Error in frame callback: {e}")
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
            # Generate frame programmatically
            # Synchronize with audio timing if available
            if self.streamer._av_sync_epoch is None:
                self.streamer._av_sync_epoch = asyncio.get_event_loop().time()
            
            if self.start_time is None:
                self.start_time = self.streamer._av_sync_epoch

            target_time = self.start_time + (self.frame_count / self.fps)
            current_time = asyncio.get_event_loop().time()
            wait_time = target_time - current_time
            
            if wait_time > 0:
                await asyncio.sleep(wait_time)
            elif wait_time < -0.05:
                # Reset clock if we are behind by more than 50ms to prevent buffer bloat
                if wait_time < -0.1:
                    print(f"⚠️  Video lag {-wait_time*1000:.0f}ms - resetting clock")
                self.start_time = current_time - (self.frame_count / self.fps)
            
            # Check if user provided a frame override
            if self.streamer.user_frame is not None:
                # Use user-provided frame
                self.frame = self.streamer.user_frame.copy()
            else:
                # Create blank frame
                self.frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            
            # Apply callback to populate the frame
            if self.callback is not None:
                try:
                    processed_img = self.callback(self.frame)
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    print(f"Error in frame callback: {e}")
                    processed_img = self.frame
            else:
                # No callback - just send blank frame
                processed_img = self.frame
            
            # Convert to VideoFrame
            new_frame = VideoFrame.from_ndarray(processed_img, format="bgr24")
            new_frame.pts = self.frame_count
            new_frame.time_base = fractions.Fraction(1, self.fps)
            
            self.frame_count += 1
            return new_frame

if __name__ == "__main__": 

    streamer = VisionProStreamer(ip = '10.29.230.57')
    while True: 

        latest = streamer.get_latest()
        print(latest)