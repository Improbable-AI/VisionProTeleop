#!/usr/bin/env python3
"""
Logitech Muse Stylus Streaming Demo
====================================

This example demonstrates how to receive Logitech Muse stylus tracking data
from the Vision Pro using the VisionProStreamer's get_stylus() API.

The Logitech Muse stylus provides:
- TIP: Pressure-sensitive tip (0-100%)
- PRI: Binary button (on/off)
- SEC: Pressure-sensitive secondary button (0-100%)

Requirements:
- Vision Pro running visionOS 26.0+
- Tracking Streamer app with Stylus Tracking enabled
- Logitech Muse stylus paired via Bluetooth

Usage:
    python 22_logitech_muse_stylus.py --ip <VISION_PRO_IP>

The script will print:
- Stylus position (XYZ in meters)
- Orientation (as Euler angles)
- Button states (TIP pressure, PRI on/off, SEC pressure)
"""

import argparse
import time
import numpy as np
from avp_stream import VisionProStreamer


def rotation_matrix_to_euler(R: np.ndarray) -> tuple:
    """Convert 3x3 rotation matrix to Euler angles (roll, pitch, yaw) in degrees."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def format_button(name: str, pressed: bool, pressure: float) -> str:
    """Format button state with visual indicator."""
    if pressed:
        bar_len = int(pressure * 10)
        bar = "█" * bar_len + "░" * (10 - bar_len)
        return f"{name}: [{bar}] {pressure*100:5.1f}%"
    else:
        return f"{name}: [----------]   --%"


def main():
    parser = argparse.ArgumentParser(description="Logitech Muse Stylus Streaming Demo")
    parser.add_argument("--ip", type=str, required=True, help="Vision Pro IP address")
    args = parser.parse_args()

    print(f"[INFO] Connecting to Vision Pro at {args.ip}...")
    streamer = VisionProStreamer(ip=args.ip, verbose=True)
    
    print("[INFO] Connected! Waiting for stylus data...")
    print("[INFO] Make sure 'Stylus Tracking' is enabled in the Vision Pro Settings panel.")
    print("[INFO] Ensure your spatial stylus is paired and nearby.")
    print()
    print("=" * 80)
    print("LOGITECH MUSE STYLUS STREAMING")
    print("=" * 80)
    
    last_print_time = 0
    print_interval = 0.1  # Print every 100ms (10 Hz display)
    
    # Track button state changes for event detection
    prev_tip = False
    prev_primary = False
    prev_secondary = False
    
    try:
        while True:
            stylus = streamer.get_stylus()
            current_time = time.time()
            
            if stylus and current_time - last_print_time >= print_interval:
                last_print_time = current_time
                
                # Extract pose data
                pose = stylus["pose"]  # 4x4 homogeneous transform
                position = pose[:3, 3]  # XYZ translation (meters)
                rotation = pose[:3, :3]  # 3x3 rotation matrix
                roll, pitch, yaw = rotation_matrix_to_euler(rotation)
                
                # Extract button data
                tip_pressed = stylus["tip_pressed"]
                tip_pressure = stylus["tip_pressure"]
                primary_pressed = stylus["primary_pressed"]
                primary_pressure = stylus["primary_pressure"]
                secondary_pressed = stylus["secondary_pressed"]
                secondary_pressure = stylus["secondary_pressure"]
                
                # Detect button events
                events = []
                if tip_pressed and not prev_tip:
                    events.append("🖊️ TIP DOWN")
                elif not tip_pressed and prev_tip:
                    events.append("🖊️ TIP UP")
                if primary_pressed and not prev_primary:
                    events.append("🔵 PRIMARY PRESSED")
                elif not primary_pressed and prev_primary:
                    events.append("🔵 PRIMARY RELEASED")
                if secondary_pressed and not prev_secondary:
                    events.append("🟡 SECONDARY PRESSED")
                elif not secondary_pressed and prev_secondary:
                    events.append("🟡 SECONDARY RELEASED")
                
                prev_tip = tip_pressed
                prev_primary = primary_pressed
                prev_secondary = secondary_pressed
                
                # Clear previous output and print new state
                print("\033[2J\033[H", end="")  # Clear screen
                print("=" * 80)
                print(f"LOGITECH MUSE STYLUS  |  {time.strftime('%H:%M:%S')}")
                print("=" * 80)
                print()
                
                # Position
                print("📍 POSITION (meters)")
                print(f"   X: {position[0]:+8.4f}  Y: {position[1]:+8.4f}  Z: {position[2]:+8.4f}")
                print()
                
                # Orientation
                print("🔄 ORIENTATION (degrees)")
                print(f"   Roll: {roll:+7.1f}°  Pitch: {pitch:+7.1f}°  Yaw: {yaw:+7.1f}°")
                print()
                
                # Buttons
                any_pressed = tip_pressed or primary_pressed or secondary_pressed
                status = "🟢 ACTIVE" if any_pressed else "⚪ IDLE"
                print(f"🎮 BUTTONS  [{status}]")
                print(f"   {format_button('TIP      ', tip_pressed, tip_pressure)}")
                print(f"   {format_button('PRIMARY  ', primary_pressed, primary_pressure)}")
                print(f"   {format_button('SECONDARY', secondary_pressed, secondary_pressure)}")
                print()
                
                # Events
                if events:
                    print("⚡ EVENTS")
                    for event in events:
                        print(f"   {event}")
                    print()
                
                print("-" * 80)
                print("Press Ctrl+C to exit")
                
            elif stylus is None:
                if current_time - last_print_time >= 2.0:
                    last_print_time = current_time
                    print(f"[{time.strftime('%H:%M:%S')}] No stylus detected.")
                    print("  → Enable 'Stylus Tracking' in Settings panel")
                    print("  → Make sure stylus is paired and nearby")
                    print()
            
            time.sleep(0.02)  # 50 Hz polling
            
    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")


if __name__ == "__main__":
    main()
