#!/usr/bin/env python3
"""
Example: Browsing and Downloading Public Recordings

This example demonstrates how to use the avp_stream.datasets module to:
1. List all publicly shared recordings from CloudKit
2. View recording metadata
3. Download recordings to a local directory

Setup
-----
Before running this script, you need to set your CloudKit API token:

    export CLOUDKIT_API_TOKEN="your-api-token"

To get an API token:
1. Go to https://icloud.developer.apple.com/dashboard/
2. Select container: iCloud.com.younghyopark.VisionProTeleop
3. Navigate to "API Access" → "Server-to-Server Keys"
4. Create a new key with Read-only, Public Database permissions
5. Copy the generated token

Usage
-----
    python examples/12_public_datasets.py

"""

from avp_stream.datasets import (
    list_public_recordings,
    get_recording,
    download_recording,
)


def main():
    print("=" * 60)
    print("VisionProTeleop Public Recordings Browser")
    print("=" * 60)
    print()
    
    # List all public recordings
    print("Fetching public recordings...")
    try:
        recordings = list_public_recordings()
    except RuntimeError as e:
        print(f"❌ Error: {e}")
        print()
        print("Make sure you have set your CloudKit API token:")
        print("  export CLOUDKIT_API_TOKEN='your-token-here'")
        return
    
    if not recordings:
        print("No public recordings found.")
        print("Share a recording from the Tracking Viewer app to see it here!")
        return
    
    print(f"Found {len(recordings)} public recording(s):\n")
    
    # Display recording information
    for i, rec in enumerate(recordings, 1):
        print(f"[{i}] {rec.title}")
        print(f"    Provider: {rec.provider}")
        if rec.recording_type:
            print(f"    Type: {rec.recording_type}")
        if rec.duration:
            print(f"    Duration: {rec.duration:.1f}s")
        if rec.frame_count:
            fps_str = f" ({rec.average_fps:.1f} FPS)" if rec.average_fps else ""
            print(f"    Frames: {rec.frame_count}{fps_str}")
        # Show capabilities
        caps = []
        if rec.has_video:
            caps.append(f"video ({rec.video_source})" if rec.video_source else "video")
        if rec.has_left_hand or rec.has_right_hand:
            hands = []
            if rec.has_left_hand:
                hands.append("L")
            if rec.has_right_hand:
                hands.append("R")
            caps.append(f"hands ({'+'.join(hands)})")
        if rec.has_simulation_data:
            caps.append("sim")
        if rec.has_usdz:
            caps.append("usdz")
        if caps:
            print(f"    Data: {', '.join(caps)}")
        print(f"    Created: {rec.created_at.strftime('%Y-%m-%d %H:%M')}")
        if rec.description:
            print(f"    Description: {rec.description}")
        print(f"    URL: {rec.cloud_url}")
        print()
    
    # Interactive download prompt
    print("-" * 60)
    choice = input("Enter number to download (or 'q' to quit): ").strip()
    
    if choice.lower() == 'q':
        print("Goodbye!")
        return
    
    try:
        idx = int(choice) - 1
        if 0 <= idx < len(recordings):
            recording = recordings[idx]
            print()
            download_path = download_recording(recording, dest_dir="./downloads")
            print(f"\n📁 Saved to: {download_path}")
        else:
            print("Invalid selection.")
    except ValueError:
        print("Invalid input. Please enter a number.")


if __name__ == "__main__":
    main()
