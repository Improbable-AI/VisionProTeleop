import grpc
from avp_stream.grpc_msg import * 
from threading import Thread
from avp_stream.utils.grpc_utils import * 
import time 
import numpy as np
import asyncio
import socket
from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import fractions 


YUP2ZUP = np.array([[[1, 0, 0, 0], 
                    [0, 0, -1, 0], 
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]]], dtype = np.float64)


class VisionProStreamer:

    def __init__(self, ip, record = True): 

        # Vision Pro IP 
        self.ip = ip
        self.record = record 
        self.recording = [] 
        self.latest = None 
        self.axis_transform = YUP2ZUP
        self.webrtc_info = None  # Store WebRTC server info for HTTP endpoint
        self.info_server = None  # HTTP server for sharing WebRTC address
        self.frame_callback = None  # User-registered frame processing function
        self.audio_callback = None  # User-registered audio processing function
        self.camera = None  # Processed video track (accessible after start_video_streaming)
        self.user_frame = None  # User-provided frame to override camera input
        self.start_streaming()
        self._start_info_server()  # Start HTTP endpoint immediately

    def start_streaming(self): 

        stream_thread = Thread(target = self.stream)
        stream_thread.start() 
        
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
                    transformations = {
                        "left_wrist": self.axis_transform @  process_matrix(response.left_hand.wristMatrix),
                        "right_wrist": self.axis_transform @  process_matrix(response.right_hand.wristMatrix),
                        "left_fingers":   process_matrices(response.left_hand.skeleton.jointMatrices),
                        "right_fingers":  process_matrices(response.right_hand.skeleton.jointMatrices),
                        "head": rotate_head(self.axis_transform @  process_matrix(response.Head)) , 
                        "left_pinch_distance": get_pinch_distance(response.left_hand.skeleton.jointMatrices),
                        "right_pinch_distance": get_pinch_distance(response.right_hand.skeleton.jointMatrices),
                        # "rgb": response.rgb, # TODO: should figure out how to get the rgb image from vision pro 
                    }
                    transformations["right_wrist_roll"] = get_wrist_roll(transformations["right_wrist"])
                    transformations["left_wrist_roll"] = get_wrist_roll(transformations["left_wrist"])
                    if self.record: 
                        self.recording.append(transformations)
                    self.latest = transformations
                
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

    def get_latest(self): 
        return self.latest
        
    def get_recording(self): 
        return self.recording
    
    def get_local_ip(self):
        """Get the local IP address of this machine"""
        try:
            # Create a socket to find out our local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
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
        
        Example:
            def my_processor(frame):
                import cv2
                return cv2.putText(frame, "Hello VisionPro!", (50, 50), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            streamer.register_frame_callback(my_processor)
            streamer.start_video_streaming()
        """
        self.frame_callback = callback
        print(f"✓ Frame callback registered: {callback.__name__ if hasattr(callback, '__name__') else 'anonymous function'}")
    
    def register_audio_callback(self, callback):
        """
        Register a callback function to process audio frames before streaming.
        
        Args:
            callback: A function that takes an AudioFrame and returns a processed AudioFrame.
                     The AudioFrame has properties: samples (numpy array), sample_rate, channels.
        
        Example:
            def my_audio_processor(frame):
                # Apply volume boost
                frame.samples = frame.samples * 1.5
                return frame
            
            streamer.register_audio_callback(my_audio_processor)
            streamer.start_video_streaming(audio_device=":0")
        """
        self.audio_callback = callback
        print(f"✓ Audio callback registered: {callback.__name__ if hasattr(callback, '__name__') else 'anonymous function'}")
    
    def update_frame(self, user_frame):
        """
        Override the camera frame with a custom frame. The frame will still go through
        any registered callbacks before being sent to VisionPro.
        
        Args:
            user_frame: A numpy array in BGR24 format (H, W, 3) to send instead of camera input.
                       Should match the resolution specified in start_video_streaming().
        
        Example:
            import numpy as np
            
            # Create a custom frame
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            frame[:, :] = [0, 255, 0]  # Green frame
            
            streamer.update_frame(frame)
        """
        self.user_frame = user_frame
    
    def _start_info_server(self, port=8888):
        """
        Start a simple HTTP server that provides WebRTC connection info.
        VisionOS can query this to discover where the WebRTC server is running.
        """
        streamer_instance = self
        
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
                server = HTTPServer(('0.0.0.0', port), InfoHandler)
                self.info_server = server
                print(f"Info server started on port {port}")
                server.serve_forever()
            except Exception as e:
                print(f"Could not start info server: {e}")
        
        info_thread = Thread(target=run_http_server, daemon=True)
        info_thread.start()
    
    def start_video_streaming(self, device="0:none", format="avfoundation", 
                             fps=30, size="640x480", 
                             port=9999, stereo_video=False, stereo_audio=False,
                             audio_device=None, audio_format=None):
        """
        Start WebRTC video streaming server with optional audio.
        VisionOS will discover the server address by querying the info HTTP endpoint.
        
        Args:
            device: Video device identifier (default: "0:none" for macOS).
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
        
        Note:
            - Use register_frame_callback() to add custom frame processing
            - Use register_audio_callback() to add custom audio processing
            - If device=None, the callback MUST generate frames (not just modify them)
            - Without device, callback receives a blank frame to populate
        """
        from aiortc import VideoStreamTrack
        from aiortc.contrib.media import MediaPlayer
        from av import VideoFrame
        
        # Get local IP
        local_ip = self.get_local_ip()
        
        print(f"Starting WebRTC server on {local_ip}:{port}")
        
        # Determine if audio will be present
        audio_enabled = audio_device is not None or self.audio_callback is not None
        
        # Store WebRTC info for the HTTP endpoint (backup method)
        self.webrtc_info = {
            "host": local_ip,
            "port": port,
            "status": "ready",
            "stereo_video": stereo_video,
            "stereo_audio": stereo_audio,
            "audio_enabled": audio_enabled
        }
        
        # Send WebRTC server info via gRPC
        print(f"📤 Sending WebRTC server info via gRPC (stereo_video={stereo_video}, stereo_audio={stereo_audio}, audio_enabled={audio_enabled})...")
        self._send_webrtc_info_via_grpc(local_ip, port, stereo_video, stereo_audio, audio_enabled)
        
        # Parse size
        width, height = map(int, size.split('x'))
        
        # Store reference to streamer instance for use in ProcessedVideoTrack
        streamer_instance = self
        
        # Define custom audio track for audio processing
        class ProcessedAudioTrack(VideoStreamTrack):
            kind = "audio"
            
            def __init__(self, audio_device, audio_fmt, callback, stereo=False):
                super().__init__()
                self.callback = callback
                self.use_microphone = audio_device is not None
                self.stereo = stereo
                
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
                    self.sample_rate = 48000
                    self.samples_per_frame = 960  # 20ms at 48kHz (reduces CPU load)
                    self.frame_count = 0
                    self.start_time = None
                    channels = "stereo" if stereo else "mono"
                    print(f"🎤 Synthetic audio initialized (48kHz, {self.samples_per_frame} samples/frame, {channels}, 20ms frames)")
            
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
                    
                    # Log first few frames
                    if self.frame_count < 3:
                        print(f"🎤 Generated audio frame #{self.frame_count}: {self.samples_per_frame} samples, {frame.sample_rate}Hz, {layout}")
                    
                    # Throttle audio generation to match expected timing (reduce CPU)
                    if self.start_time is None:
                        self.start_time = asyncio.get_event_loop().time()
                    
                    expected_time = self.start_time + (self.frame_count * self.samples_per_frame / self.sample_rate)
                    current_time = asyncio.get_event_loop().time()
                    sleep_time = expected_time - current_time
                    
                    if sleep_time > 0:
                        await asyncio.sleep(sleep_time)
                    
                    # Apply callback if registered (callback should generate audio)
                    if self.callback is not None:
                        try:
                            processed_frame = self.callback(frame)
                            self.frame_count += 1
                            if self.frame_count == 10:
                                print(f"🎤 Audio streaming normally (generated {self.frame_count} frames, 20ms per frame)")
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
            def __init__(self, device, fmt, framerate, resolution, callback):
                super().__init__()
                self.callback = callback
                self.width, self.height = resolution
                self.use_camera = device is not None
                self.warmup_frames = 0  # Track warmup frames for encoder optimization
                
                if self.use_camera:
                    # Use physical camera
                    opts = {"video_size": f"{self.width}x{self.height}", "framerate": str(framerate)}
                    self.player = MediaPlayer(device, format=fmt, options=opts)
                    self.video_track = self.player.video
                    print("🎥 Camera initialized, warming up encoder...")
                else:
                    # Generate frames programmatically
                    self.player = None
                    self.video_track = None
                    self.fps = framerate
                    self.frame_count = 0
                    self.start_time = time.time()
                
            async def recv(self):
                if self.use_camera:
                    # Check if user provided a frame override
                    if streamer_instance.user_frame is not None:
                        # Use user-provided frame instead of camera
                        img = streamer_instance.user_frame
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
                    # Wait to maintain target FPS
                    target_time = self.start_time + (self.frame_count / self.fps)
                    current_time = time.time()
                    wait_time = target_time - current_time
                    if wait_time > 0:
                        await asyncio.sleep(wait_time)
                    
                    # Check if user provided a frame override
                    if streamer_instance.user_frame is not None:
                        # Use user-provided frame
                        self.frame = streamer_instance.user_frame.copy()
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
        
        # Create a new event loop for the WebRTC server in a separate thread
        def start_async_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            # Modify run_peer to use custom parameters
            async def run_peer_custom(reader, writer):
                from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
                import logging
                
                client_addr = writer.get_extra_info('peername')
                print(f"🔗 VisionOS client connected from {client_addr}")
                
                # Configure RTCPeerConnection for low latency
                config = RTCConfiguration(
                    iceServers=[RTCIceServer(urls=["stun:stun.l.google.com:19302"])]
                )
                pc = RTCPeerConnection(configuration=config)
                
                @pc.on("iceconnectionstatechange")
                async def on_ice_state_change():
                    print(f"ICE state: {pc.iceConnectionState}")
                    if pc.iceConnectionState == "connected":
                        print("🎥 WebRTC connection established - video should flow now")
                    if pc.iceConnectionState in ("failed", "closed"):
                        await pc.close()
                        writer.close()
                        await writer.wait_closed()
                
                @pc.on("track")
                def on_track(track):
                    print(f"📹 Track received: {track.kind}")
                
                # Create video track with or without camera
                try:
                    if device is None:
                        print("Creating synthetic video stream (no camera)...")
                        if self.frame_callback is None:
                            print("⚠️  WARNING: No frame callback registered. Stream will be blank.")
                            print("   Use register_frame_callback() to generate frames.")
                    else:
                        print(f"Opening video device: {device}...")
                    
                    processed_track = ProcessedVideoTrack(
                        device, 
                        format, 
                        fps,
                        (width, height),
                        self.frame_callback  # Pass the registered callback
                    )
                    
                    # Store the track as instance attribute for external access
                    self.camera = processed_track
                    
                    if device is None:
                        print(f"✓ Synthetic video track created ({size} @ {fps}fps)")
                    else:
                        print(f"✓ Camera track created for device: {device}")
                    
                    if self.frame_callback:
                        print("✓ Frame processing enabled")
                    else:
                        print("✓ Frame processing disabled (identity)")
                except Exception as e:
                    print(f"Error creating ProcessedVideoTrack: {e}")
                    raise
                
                pc.addTrack(processed_track)
                
                # Add audio track if audio device is specified OR if audio callback is registered
                if audio_device is not None or self.audio_callback is not None:
                    try:
                        processed_audio_track = ProcessedAudioTrack(
                            audio_device, 
                            audio_format, 
                            self.audio_callback,
                            stereo=stereo_audio
                        )
                        pc.addTrack(processed_audio_track)
                        channels = "stereo" if stereo_audio else "mono"
                        if audio_device:
                            print(f"🎤 Audio track added from device: {audio_device} ({channels})")
                        else:
                            print(f"🎤 Synthetic audio track added ({channels}, callback-generated)")
                    except Exception as e:
                        print(f"⚠️  Could not create audio track: {e}")
                        print(f"    Continuing with video only...")
                        import traceback
                        traceback.print_exc()
                
                # Create offer
                offer = await pc.createOffer()
                await pc.setLocalDescription(offer)
                
                # Wait briefly for at least one ICE candidate (improves connection speed)
                # Modern WebRTC uses trickle ICE, so we don't need complete gathering
                start_time = asyncio.get_event_loop().time()
                max_wait_time = 0.5  # 500ms max
                
                while asyncio.get_event_loop().time() - start_time < max_wait_time:
                    if pc.iceGatheringState == "complete":
                        print(f"ICE gathering complete in {asyncio.get_event_loop().time() - start_time:.2f}s")
                        break
                    elif pc.iceGatheringState == "gathering":
                        # At least one candidate should be available, that's good enough
                        await asyncio.sleep(0.1)
                        print(f"ICE gathering in progress, proceeding after {asyncio.get_event_loop().time() - start_time:.2f}s")
                        break
                    await asyncio.sleep(0.05)
                
                if pc.iceGatheringState == "new":
                    print("Warning: ICE gathering hasn't started yet, sending offer anyway")
                
                # Send offer (ICE candidates will be trickled separately if needed)
                offer_payload = {
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type,
                }
                writer.write((json.dumps(offer_payload) + "\n").encode("utf-8"))
                await writer.drain()
                print(f"Sent offer to VisionOS (ICE state: {pc.iceGatheringState})")
                
                # Wait for answer
                line = await reader.readline()
                if not line:
                    print("VisionOS disconnected before sending answer")
                    await pc.close()
                    return
                
                answer_payload = json.loads(line.decode("utf-8"))
                answer = RTCSessionDescription(
                    sdp=answer_payload["sdp"],
                    type=answer_payload["type"],
                )
                
                await pc.setRemoteDescription(answer)
                print("WebRTC connection established!")
                
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
                server = await asyncio.start_server(run_peer_custom, "0.0.0.0", port)
                print(f"WebRTC server listening on 0.0.0.0:{port}")
                async with server:
                    await server.serve_forever()
            
            loop.run_until_complete(run_server())
        
        webrtc_thread = Thread(target=start_async_server, daemon=True)
        webrtc_thread.start()
        
        # Wait a bit for server to start
        time.sleep(0.5)
        
        print(f"✓ WebRTC server ready at {local_ip}:{port}")
        print(f"✓ VisionOS can query http://{local_ip}:8888/webrtc_info to get connection details")
    
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
    

if __name__ == "__main__": 

    streamer = VisionProStreamer(ip = '10.29.230.57')
    while True: 

        latest = streamer.get_latest()
        print(latest)