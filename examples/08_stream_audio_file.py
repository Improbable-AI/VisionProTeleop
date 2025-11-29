"""Stream an audio file (MP3/WAV) in a loop while displaying hand tracking visualization.

Requires: pydub (pip install pydub), ffmpeg (brew install ffmpeg)
"""

from avp_stream.streamer import VisionProStreamer
import cv2
import numpy as np
import time
import array

def hand_tracking_visualizer(streamer):
    """Visualize hand tracking data in 3D space"""
    
    def generate_frame(blank_frame):
        h, w = blank_frame.shape[:2]
        
        # Dark background
        blank_frame[:] = [20, 20, 20]
        
        # Get latest hand tracking data
        latest = streamer.get_latest()
        if latest is None:
            cv2.putText(blank_frame, "Waiting for hand data...", (w//2 - 200, h//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            return blank_frame
        
        # Draw virtual 3D space
        center_x, center_y = w // 2, h // 2
        
        # Draw coordinate axes
        cv2.line(blank_frame, (center_x, center_y), (center_x + 100, center_y), (0, 0, 255), 2)
        cv2.line(blank_frame, (center_x, center_y), (center_x, center_y - 100), (0, 255, 0), 2)
        cv2.putText(blank_frame, "X", (center_x + 110, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(blank_frame, "Y", (center_x, center_y - 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Project and draw hand positions
        left_wrist = latest.get("left_wrist")
        right_wrist = latest.get("right_wrist")
        left_fingers = latest.get("left_fingers")
        right_fingers = latest.get("right_fingers")
        
        # Draw left hand
        if left_wrist is not None and left_fingers is not None:
            left_fingers_world = left_wrist @ left_fingers
            lx, ly = int(center_x + float(left_wrist[0, 0, 3]) * 400), int(center_y - float(left_wrist[0, 1, 3]) * 400)
            left_pinch = latest.get("left_pinch_distance", 0)
            color = (0, 255, 0) if left_pinch < 0.02 else (100, 100, 255)
            cv2.circle(blank_frame, (lx, ly), 15, color, -1)
            cv2.putText(blank_frame, "L", (lx - 8, ly + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            for i in range(left_fingers_world.shape[0]):
                fx = int(center_x + float(left_fingers_world[i, 0, 3]) * 400)
                fy = int(center_y - float(left_fingers_world[i, 1, 3]) * 400)
                cv2.circle(blank_frame, (fx, fy), 3, (150, 150, 255), -1)
        
        # Draw right hand
        if right_wrist is not None and right_fingers is not None:
            right_fingers_world = right_wrist @ right_fingers
            rx, ry = int(center_x + float(right_wrist[0, 0, 3]) * 400), int(center_y - float(right_wrist[0, 1, 3]) * 400)
            right_pinch = latest.get("right_pinch_distance", 0)
            color = (0, 255, 0) if right_pinch < 0.02 else (255, 100, 100)
            cv2.circle(blank_frame, (rx, ry), 15, color, -1)
            cv2.putText(blank_frame, "R", (rx - 8, ry + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            for i in range(right_fingers_world.shape[0]):
                fx = int(center_x + float(right_fingers_world[i, 0, 3]) * 400)
                fy = int(center_y - float(right_fingers_world[i, 1, 3]) * 400)
                cv2.circle(blank_frame, (fx, fy), 3, (255, 150, 150), -1)
        
        # Display pinch info
        left_pinch = latest.get("left_pinch_distance", 0)
        right_pinch = latest.get("right_pinch_distance", 0)
        cv2.putText(blank_frame, f"Left Pinch: {left_pinch:.3f}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(blank_frame, f"Right Pinch: {right_pinch:.3f}", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Display audio status
        cv2.putText(blank_frame, "🎵 Audio: Streaming", (20, h - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return blank_frame
    
    return generate_frame


def audio_file_streamer(audio_file_path, stereo=False):
    """Stream audio file in a loop"""
    from pydub import AudioSegment
    
    print(f"🎵 Loading audio file: {audio_file_path}")
    
    # Load audio file (supports MP3, WAV, etc.)
    audio = AudioSegment.from_file(audio_file_path)
    
    # Convert to 48kHz with specified channel count
    channels = 2 if stereo else 1
    audio = audio.set_frame_rate(48000).set_channels(channels)
    
    channel_mode = "stereo" if stereo else "mono"
    print(f"✓ Audio loaded ({channel_mode}):")
    print(f"  - Duration: {len(audio) / 1000:.2f} seconds")
    print(f"  - Sample rate: {audio.frame_rate} Hz")
    print(f"  - Channels: {audio.channels}")
    print(f"  - Sample width: {audio.sample_width} bytes")
    
    # Get raw audio data as int16 samples
    # For stereo, this array is interleaved: [L, R, L, R, L, R, ...]
    audio_samples = np.array(audio.get_array_of_samples(), dtype=np.int16)
    total_samples = len(audio_samples)
    
    # Track position in audio file (in terms of individual samples, not frames)
    current_position = 0
    
    def generate_audio(audio_frame):
        nonlocal current_position
        
        # Number of samples per frame PER CHANNEL
        num_samples_per_channel = audio_frame.samples
        
        # Total samples needed (for stereo: num_samples * 2, for mono: num_samples * 1)
        num_samples_needed = num_samples_per_channel * channels
        
        # Extract samples for this frame
        end_position = current_position + num_samples_needed
        
        # Handle looping
        if end_position >= total_samples:
            # Wrap around to beginning
            samples_before_end = total_samples - current_position
            samples_after_wrap = num_samples_needed - samples_before_end
            
            # Combine end + beginning
            frame_data = np.concatenate([
                audio_samples[current_position:],
                audio_samples[:samples_after_wrap]
            ])
            
            current_position = samples_after_wrap
        else:
            # Normal case: extract from current position
            frame_data = audio_samples[current_position:end_position]
            current_position = end_position
        
        # Convert numpy array to bytes
        audio_array = array.array('h', frame_data.tolist())
        audio_bytes = audio_array.tobytes()
        
        # Update the audio frame's plane data
        for plane in audio_frame.planes:
            plane.update(audio_bytes)
        
        return audio_frame
    
    return generate_audio


if __name__ == "__main__":
    import argparse 
    import os
    
    parser = argparse.ArgumentParser(description="Stream audio file with hand tracking visualization")
    parser.add_argument("--ip", type=str, required=True, help="Vision Pro IP address")
    parser.add_argument("--audio", type=str, required=True, help="Path to audio file (MP3, WAV, etc.)")
    parser.add_argument("--stereo-audio", action="store_true", help="Stream audio in stereo (2 channels) instead of mono")
    args = parser.parse_args()
    
    # Check if audio file exists
    if not os.path.exists(args.audio):
        print(f"❌ Error: Audio file not found: {args.audio}")
        exit(1)
    
    # Create streamer
    streamer = VisionProStreamer(ip=args.ip)
    
    # Register video callback for hand visualization
    streamer.register_frame_callback(hand_tracking_visualizer(streamer))
    
    # Register audio callback for file streaming
    streamer.register_audio_callback(audio_file_streamer(args.audio, stereo=args.stereo_audio))
    
    # Configure video streaming
    streamer.configure_video(
        device=None,           # No camera - synthetic video
        format=None,           
        fps=30,                # 30fps video
        size="1280x720",       # HD resolution
        stereo=False,          # Mono video
    )
    
    # Configure audio streaming
    streamer.configure_audio(
        device=None,           # No microphone - use audio file
        format=None,
        stereo=args.stereo_audio,  # Stereo audio if flag is set
    )
    
    streamer.start_webrtc(port=9999)
    
    print("=" * 60)
    print("Streaming hand tracking visualization with audio file")
    print("=" * 60)
    print()
    print(f"📹 Video: Hand tracking visualization at 30fps")
    audio_mode = "stereo" if args.stereo_audio else "mono"
    print(f"🎵 Audio: Looping {args.audio} ({audio_mode})")
    print()
    print("Make sure Vision Pro app is running and connected!")
    print("Press Ctrl+C to stop")
    print()
    
    try:
        while True:
            time.sleep(1/30.)
    except KeyboardInterrupt:
        print("\n\nStopping...")
        print("Done!")
