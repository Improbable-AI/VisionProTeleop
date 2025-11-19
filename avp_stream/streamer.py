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
    
    def start_video_streaming(self, video_device="0:none", format="avfoundation", 
                             options={"video_size": "640x480", "framerate": "30"}, 
                             port=9999):
        """
        Start WebRTC video streaming server.
        VisionOS will discover the server address by querying the info HTTP endpoint.
        
        Args:
            video_device: Video device identifier (default: "0:none" for macOS)
            format: Video format (default: "avfoundation" for macOS)
            options: Video options dict (default: 640x480 @ 30fps)
            port: Port to run WebRTC server on (default: 9999)
        """
        from avp_stream.visual.server import run_peer
        
        # Get local IP
        local_ip = self.get_local_ip()
        
        print(f"Starting WebRTC server on {local_ip}:{port}")
        
        # Store WebRTC info for the HTTP endpoint (backup method)
        self.webrtc_info = {
            "host": local_ip,
            "port": port,
            "status": "ready"
        }
        
        # Send WebRTC server info via gRPC
        print(f"📤 Sending WebRTC server info via gRPC...")
        self._send_webrtc_info_via_grpc(local_ip, port)
        
        # Create a new event loop for the WebRTC server in a separate thread
        def start_async_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            # Modify run_peer to use custom parameters
            async def run_peer_custom(reader, writer):
                from aiortc import RTCPeerConnection, RTCSessionDescription
                from aiortc.contrib.media import MediaPlayer
                import logging
                
                pc = RTCPeerConnection()
                
                @pc.on("iceconnectionstatechange")
                async def on_ice_state_change():
                    print(f"ICE state: {pc.iceConnectionState}")
                    if pc.iceConnectionState in ("failed", "closed"):
                        await pc.close()
                        writer.close()
                        await writer.wait_closed()
                
                # Open video device with provided parameters
                try:
                    print("Opening video device...")
                    player = MediaPlayer(
                        video_device,
                        format=format,
                        options=options,
                    )
                    print(f"MediaPlayer created for device: {video_device}")
                except Exception as e:
                    print(f"Error creating MediaPlayer: {e}")
                    raise
                
                if player.video:
                    pc.addTrack(player.video)
                
                # Create offer
                offer = await pc.createOffer()
                await pc.setLocalDescription(offer)
                
                # Wait for ICE gathering
                while pc.iceGatheringState != "complete":
                    await asyncio.sleep(0.1)
                
                # Send offer
                offer_payload = {
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type,
                }
                writer.write((json.dumps(offer_payload) + "\n").encode("utf-8"))
                await writer.drain()
                print("Sent offer to VisionOS")
                
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
    
    def _send_webrtc_info_via_grpc(self, host, port):
        """Send WebRTC server info to VisionOS by opening a new gRPC connection."""
        try:
            ip_parts = host.split('.')
            
            print(f"📤 Opening new gRPC connection to send WebRTC info...")
            
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
                    
                    stub = handtracking_pb2_grpc.HandTrackingServiceStub(channel)
                    # Send the message and immediately close
                    responses = stub.StreamHandUpdates(webrtc_msg)
                    # Just start the stream but don't consume responses
                    next(responses)  # Get first response to ensure message was sent
                    
                    print(f"✓ WebRTC info sent via gRPC: {host}:{port}")
                except Exception as e:
                    print(f"⚠️  Error sending WebRTC info via gRPC: {e}")
                    
        except Exception as e:
            print(f"❌ Error in _send_webrtc_info_via_grpc: {e}")
    

if __name__ == "__main__": 

    streamer = VisionProStreamer(ip = '10.29.230.57')
    while True: 

        latest = streamer.get_latest()
        print(latest)