#!/usr/bin/env python3
"""
Example script demonstrating the new TrackingData API.

This shows how to load tracking data from a JSONL file using load_jsonl()
and use both the new attribute-style API and the old dictionary-style API.
"""

from avp_stream import load_jsonl

# Load tracking data from JSONL
tracking_file = "downloads/recording_20251208_192758_F883/recording_20251208_192758_F883/tracking.jsonl"

print(f"Loading: {tracking_file}")
frames = load_jsonl(tracking_file)
print(f"Loaded {len(frames)} frames\n")

# Get the first frame
frame = frames[0]
print(f"Frame type: {type(frame).__name__}")
print(f"Frame repr: {frame}\n")

# ============================================================================
# New API: Attribute access
# ============================================================================
print("=" * 60)
print("NEW API: Attribute Access")
print("=" * 60)

# Head pose (4x4 matrix)
head_pos = frame.head[:3, 3]
print(f"Head position: {head_pos}")

# Right hand - direct array access (27 x 4 x 4)
print(f"\nRight hand shape: {frame.right.shape}")
print(f"Right hand type: {type(frame.right).__name__}")

# Access joints by name
index_tip = frame.right.indexTip  # 4x4 matrix
thumb_tip = frame.right.thumbTip
wrist = frame.right.wrist

print(f"\nRight wrist position: {wrist[:3, 3]}")
print(f"Right index tip position: {index_tip[:3, 3]}")
print(f"Right thumb tip position: {thumb_tip[:3, 3]}")

# Access by index (same as by name)
print(f"\nRight joint[9] (indexTip): {frame.right[9][:3, 3]}")

# Additional properties
print(f"\nPinch distance: {frame.right.pinch_distance:.4f}m")
print(f"Wrist roll: {frame.right.wrist_roll:.4f} rad")
print(f"Has forearm: {frame.right.has_forearm}")

# ============================================================================
# Old API: Dictionary access (backward compatible)
# ============================================================================
print("\n" + "=" * 60)
print("OLD API: Dictionary Access (Backward Compatible)")
print("=" * 60)

# Access like a dictionary
head = frame["head"]  # (1, 4, 4)
right_wrist = frame["right_wrist"]  # (1, 4, 4)
right_fingers = frame["right_fingers"]  # (25, 4, 4)
right_arm = frame["right_arm"]  # (27, 4, 4)

print(f"head shape: {head.shape}")
print(f"right_wrist shape: {right_wrist.shape}")
print(f"right_fingers shape: {right_fingers.shape}")
print(f"right_arm shape: {right_arm.shape}")

# Dict methods work
print(f"\nKeys: {list(frame.keys())}")
print(f"'head' in frame: {'head' in frame}")

# ============================================================================
# Loop through frames
# ============================================================================
print("\n" + "=" * 60)
print("Iterating through frames")
print("=" * 60)

for i, frame in enumerate(frames[:5]):  # First 5 frames
    pos = frame.right.wrist[:3, 3]
    print(f"Frame {i}: right wrist = [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")

print(f"... and {len(frames) - 5} more frames")
