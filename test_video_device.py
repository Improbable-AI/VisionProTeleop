#!/usr/bin/env python3
"""
Simple script to test video device configurations before using with VisionProStreamer.
Helps you find the right video_device, format, and options for your system.

Usage:
    python test_video_device.py           # Quick test (receive one frame)
    python test_video_device.py --live    # Live view (shows frames until Ctrl+C)
    python test_video_device.py --save    # Save 10 seconds of video
"""

import asyncio
import sys
import argparse
import signal
from aiortc.contrib.media import MediaPlayer
import cv2
import numpy as np
import av


async def test_video_device(device, format="avfoundation", size="1280x720", fps="30", mode="test"):
    """
    Test if a video device can be opened with the given parameters.
    
    Args:
        device: Device identifier (e.g., "0:none", "0", "/dev/video0")
        format_name: Format string (e.g., "avfoundation", "v4l2", "dshow")
        size: Video resolution as "WIDTHxHEIGHT" (e.g., "1280x720")
        fps: Frames per second (e.g., "30")
        mode: "test" (single frame), "live" (continuous display), or "save" (record video)
    
    Returns:
        bool: True if device opens successfully, False otherwise
    """
    
    print(f"\n{'='*60}")
    print(f"Testing: {device}")
    print(f"Format: {format}")
    print(f"Size: {size}")
    print(f"FPS: {fps}")
    print(f"Mode: {mode}")
    print(f"{'='*60}")
    
    player = None
    video_writer = None
    stop_flag = False
    
    def signal_handler(sig, frame):
        nonlocal stop_flag
        print("\n🛑 Ctrl+C detected, stopping...")
        stop_flag = True
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        player = MediaPlayer(device, format=format, options={"video_size": size, "framerate": str(fps)})
        
        if player.video:
            print("✓ SUCCESS! Video device opened successfully.")
            print(f"  Video track: {player.video}")
            
            if mode == "test":
                # Quick test - receive one frame
                try:
                    frame = await asyncio.wait_for(player.video.recv(), timeout=2.0)
                    print(f"✓ Received video frame: {frame.width}x{frame.height}")
                    return True
                except asyncio.TimeoutError:
                    print("⚠️  Device opened but no frame received within 2 seconds")
                    return False
                except Exception as e:
                    print(f"⚠️  Error receiving frame: {e}")
                    return False
                    
            elif mode == "live":
                # Live display
                print("📹 Starting live view (press Ctrl+C to stop)...")
                frame_count = 0
                
                while not stop_flag:
                    try:
                        frame = await asyncio.wait_for(player.video.recv(), timeout=0.1)
                        
                        # Convert frame to numpy array for OpenCV
                        img = frame.to_ndarray(format="bgr24")
                        
                        # Display frame
                        cv2.imshow("Video Device Test - Press Q to quit", img)
                        
                        # Check for 'q' key or window close
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                            
                        frame_count += 1
                        if frame_count % 30 == 0:
                            print(f"  Frames received: {frame_count}")
                            
                    except asyncio.TimeoutError:
                        continue
                    except Exception as e:
                        print(f"⚠️  Error: {e}")
                        break
                
                cv2.destroyAllWindows()
                print(f"✓ Received {frame_count} frames total")
                return True
                
            elif mode == "save":
                # Save video for 10 seconds
                print("💾 Recording 10 seconds of video...")
                output_file = "test_video_output.mp4"
                frame_count = 0
                start_time = asyncio.get_event_loop().time()
                duration = 10.0
                
                # Get first frame to initialize video writer
                frame = await asyncio.wait_for(player.video.recv(), timeout=2.0)
                height, width = frame.height, frame.width
                
                # Initialize video writer
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                fps = int(options.get("framerate", "30"))
                video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))
                
                print(f"  Recording to: {output_file}")
                print(f"  Resolution: {width}x{height} @ {fps}fps")
                
                # Write first frame
                img = frame.to_ndarray(format="bgr24")
                video_writer.write(img)
                frame_count += 1
                
                # Continue recording
                while asyncio.get_event_loop().time() - start_time < duration and not stop_flag:
                    try:
                        frame = await asyncio.wait_for(player.video.recv(), timeout=0.1)
                        img = frame.to_ndarray(format="bgr24")
                        video_writer.write(img)
                        frame_count += 1
                        
                        if frame_count % 30 == 0:
                            elapsed = asyncio.get_event_loop().time() - start_time
                            print(f"  Recording... {elapsed:.1f}s / {duration}s ({frame_count} frames)")
                            
                    except asyncio.TimeoutError:
                        continue
                    except Exception as e:
                        print(f"⚠️  Error: {e}")
                        break
                
                print(f"✓ Saved {frame_count} frames to {output_file}")
                return True
        else:
            print("❌ FAILED: No video track available")
            return False
            
    except Exception as e:
        print(f"❌ FAILED: {type(e).__name__}: {e}")
        return False
    finally:
        # Clean up
        if video_writer is not None:
            video_writer.release()
            
        if player is not None:
            print("🧹 Cleaning up player...")            
            if player.video:
                player.video.stop()
            if player.audio:
                player.audio.stop()
            await asyncio.sleep(0.5)
            print("✓ Cleanup complete")


async def main():
    """Run tests for common video device configurations."""
    
    parser = argparse.ArgumentParser(
        description="Test video device configurations for VisionProStreamer"
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Show live video feed (press Ctrl+C or 'q' to stop)"
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save 10 seconds of video to test_video_output.mp4"
    )
    
    args = parser.parse_args()
    
    # Determine mode
    if args.live:
        mode = "live"
    elif args.save:
        mode = "save"
    else:
        mode = "test"
    
    await test_video_device(
        device="0:none", 
        format="avfoundation", 
        size="1280x720",
        fps=30,
        mode=mode
    )
    

if __name__ == "__main__":
    asyncio.run(main())
        
