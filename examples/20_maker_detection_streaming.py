#!/usr/bin/env python3
"""
Marker Detection Demo
=====================

This example demonstrates how to receive ArUco marker positions from the Vision Pro
using the VisionProStreamer's get_markers() API.

Requirements:
- Vision Pro running the Tracking Streamer app
- Marker detection enabled in Settings > Marker Detection
- Physical ArUco markers (default: DICT_4X4_50, IDs 0-19)

Usage:
    python marker_detection_demo.py --ip <VISION_PRO_IP>

The script will print detected marker positions in real-time.

New in 2.51.0:
- Each marker now includes 'is_fixed' and 'is_tracked' fields:
  - is_fixed: Whether the pose is frozen (won't update from tracking)
  - is_tracked: Whether ARKit is actively seeing and tracking the marker
- Use the Fix/Delete buttons on the Vision Pro UI to control each marker
"""

import argparse
import time
import numpy as np
from avp_stream import VisionProStreamer


def get_status_icon(is_fixed: bool, is_tracked: bool) -> str:
    """Get visual status indicator for marker state."""
    if is_fixed and is_tracked:
        return "📌🟢"  # Fixed + Tracked (frozen pose, but still visible)
    elif is_fixed and not is_tracked:
        return "📌⚪"  # Fixed + Not tracked (frozen pose, marker out of view)
    elif not is_fixed and is_tracked:
        return "🔴🟢"  # Live + Tracked (actively updating)
    else:
        return "🔴⚪"  # Live + Not tracked (waiting for detection)


def main():
    parser = argparse.ArgumentParser(description="ArUco Marker Detection Demo")
    parser.add_argument("--ip", type=str, required=True, help="Vision Pro IP address")
    args = parser.parse_args()

    print(f"[INFO] Connecting to Vision Pro at {args.ip}...")
    streamer = VisionProStreamer(ip=args.ip, verbose=True)
    
    print("[INFO] Connected! Waiting for marker detection data...")
    print("[INFO] Make sure 'Marker Detection' is enabled in the Vision Pro Settings panel.")
    print("[INFO] Use the 'Fix' button on Vision Pro to freeze a marker's pose.")
    print()
    print("Status Legend: 📌=Fixed, 🔴=Live, 🟢=Tracked, ⚪=Not tracked")
    print("-" * 75)
    
    last_print_time = 0
    print_interval = 0.5  # Print every 0.5 seconds
    
    try:
        while True:
            markers = streamer.get_markers()
            current_time = time.time()
            
            if markers and current_time - last_print_time >= print_interval:
                last_print_time = current_time
                
                # Count states
                n_fixed = sum(1 for m in markers.values() if m.get("is_fixed", False))
                n_tracked = sum(1 for m in markers.values() if m.get("is_tracked", False))
                
                print(f"\n[{time.strftime('%H:%M:%S')}] {len(markers)} marker(s) "
                      f"| {n_tracked} tracked | {n_fixed} fixed")
                
                for marker_id, info in sorted(markers.items()):
                    pose = info["pose"]  # 4x4 homogeneous transform
                    position = pose[:3, 3]  # XYZ translation
                    is_fixed = info.get("is_fixed", False)
                    is_tracked = info.get("is_tracked", False)
                    
                    # Visual indicator for state
                    status = get_status_icon(is_fixed, is_tracked)
                    fixed_str = "FIXED" if is_fixed else "live "
                    tracked_str = "TRACKED" if is_tracked else "lost   "
                    
                    print(f"  {status} ID {marker_id}: [{position[0]:6.3f}, {position[1]:6.3f}, {position[2]:6.3f}] m  "
                          f"({fixed_str}, {tracked_str})")
            
            elif not markers:
                if current_time - last_print_time >= 2.0:
                    last_print_time = current_time
                    print(f"[{time.strftime('%H:%M:%S')}] No markers detected. Point camera at ArUco markers.")
            
            time.sleep(0.05)  # 20 Hz polling
            
    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")


if __name__ == "__main__":
    main()
