#!/usr/bin/env python3
"""
Interactive 3D Visualizer for VisionProTeleop hand tracking data using Viser.

Features:
- Interactive 3D visualization with hand joint spheres
- Timeline scrubbing with playback controls
- Head pose and camera frame visualization  
- Video playback synchronized with tracking data
- Follow-head camera mode

Usage:
    python viser_tracking_visualizer.py /path/to/recording_directory
    
Or with a specific JSONL file:
    python viser_tracking_visualizer.py /path/to/tracking.jsonl

Requirements:
    pip install viser numpy
    pip install opencv-python  # Optional, for video playback
"""

import argparse
import json
import time
import threading
from pathlib import Path
from typing import Optional
from dataclasses import dataclass

import numpy as np

try:
    import viser
    import viser.transforms as tf
except ImportError:
    print("Please install viser: pip install viser")
    exit(1)

# Plotly analytics removed for performance

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("Warning: opencv-python not installed. Video playback will be disabled.")


# ============================================================================
# Hand Skeleton Definition
# ============================================================================

# Hand skeleton connections (joint name pairs)
HAND_CONNECTIONS = [
    # Thumb
    ("wrist", "thumbCMC"),
    ("thumbCMC", "thumbMP"),
    ("thumbMP", "thumbIP"),
    ("thumbIP", "thumbTip"),
    # Index
    ("wrist", "indexMCP"),
    ("indexMCP", "indexPIP"),
    ("indexPIP", "indexDIP"),
    ("indexDIP", "indexTip"),
    # Middle
    ("wrist", "middleMCP"),
    ("middleMCP", "middlePIP"),
    ("middlePIP", "middleDIP"),
    ("middleDIP", "middleTip"),
    # Ring
    ("wrist", "ringMCP"),
    ("ringMCP", "ringPIP"),
    ("ringPIP", "ringDIP"),
    ("ringDIP", "ringTip"),
    # Little
    ("wrist", "littleMCP"),
    ("littleMCP", "littlePIP"),
    ("littlePIP", "littleDIP"),
    ("littleDIP", "littleTip"),
    # Palm connections
    ("indexMCP", "middleMCP"),
    ("middleMCP", "ringMCP"),
    ("ringMCP", "littleMCP"),
]

# Finger tips for pinch detection
FINGERTIPS = ["thumbTip", "indexTip", "middleTip", "ringTip", "littleTip"]

# Total joints per hand
JOINTS_PER_HAND = 25

# Joint names in order - matches ARKit HandSkeleton joint order from RecordingManager.swift
# 25 joints total per hand:
#   0: wrist
#   1-4: thumb (knuckle, intermediateBase, intermediateTip, tip)
#   5-9: index (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
#   10-14: middle (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
#   15-19: ring (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
#   20-24: little (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
JOINT_NAMES = [
    "wrist",
    # Thumb (4 joints)
    "thumbKnuckle", "thumbIntermediateBase", "thumbIntermediateTip", "thumbTip",
    # Index (5 joints)
    "indexMetacarpal", "indexKnuckle", "indexIntermediateBase", "indexIntermediateTip", "indexTip",
    # Middle (5 joints)
    "middleMetacarpal", "middleKnuckle", "middleIntermediateBase", "middleIntermediateTip", "middleTip",
    # Ring (5 joints)
    "ringMetacarpal", "ringKnuckle", "ringIntermediateBase", "ringIntermediateTip", "ringTip",
    # Little (5 joints)
    "littleMetacarpal", "littleKnuckle", "littleIntermediateBase", "littleIntermediateTip", "littleTip",
]

# Colors for visualization
LEFT_HAND_COLOR = (0.4, 0.6, 1.0)   # Blue
RIGHT_HAND_COLOR = (1.0, 0.5, 0.4)  # Red/Orange
HEAD_COLOR = (0.2, 0.8, 0.4)        # Green
PINCH_COLOR = (0.2, 1.0, 0.2)       # Bright green


# ============================================================================
# Data Loading
# ============================================================================

def load_tracking_data(path: Path) -> tuple[list[dict], dict | None]:
    """Load tracking data and metadata from a file or directory."""
    
    # If path is a directory, look for tracking.jsonl
    if path.is_dir():
        tracking_file = path / "tracking.jsonl"
        metadata_file = path / "metadata.json"
    else:
        tracking_file = path
        metadata_file = path.parent / "metadata.json"
    
    if not tracking_file.exists():
        raise FileNotFoundError(f"Tracking file not found: {tracking_file}")
    
    # Load tracking frames
    frames = []
    print(f"Loading tracking data from: {tracking_file}")
    with open(tracking_file, "r") as f:
        for i, line in enumerate(f):
            line = line.strip()
            if line:
                try:
                    frames.append(json.loads(line))
                except json.JSONDecodeError as e:
                    print(f"Warning: Failed to parse line {i+1}: {e}")
            if i % 1000 == 0 and i > 0:
                print(f"  Loaded {i} frames...")
    
    print(f"Loaded {len(frames)} frames total")
    
    # Load metadata if available
    metadata = None
    if metadata_file.exists():
        with open(metadata_file, "r") as f:
            metadata = json.load(f)
        print(f"Loaded metadata: duration={metadata.get('duration', 'N/A'):.1f}s")
    
    return frames, metadata


# ============================================================================
# Matrix Utilities  
# ============================================================================

def matrix_from_array(arr: list[float]) -> np.ndarray:
    """Convert a 16-element column-major array to a 4x4 matrix."""
    return np.array(arr, dtype=np.float64).reshape(4, 4, order='F')


def get_position(matrix: np.ndarray) -> np.ndarray:
    """Extract translation (position) from a 4x4 transformation matrix."""
    return matrix[:3, 3].copy()


def yup_to_zup(pos: np.ndarray) -> np.ndarray:
    """Convert Y-up (ARKit) to Z-up coordinate system.
    
    ARKit: X-right, Y-up, Z-backward
    Z-up:  X-right, Y-forward, Z-up
    
    Transformation: (x, y, z) -> (x, -z, y)
    """
    return np.array([pos[0], -pos[2], pos[1]])


def get_rotation_matrix(matrix: np.ndarray) -> np.ndarray:
    """Extract rotation matrix from a 4x4 transformation matrix."""
    return matrix[:3, :3].copy()


def rotation_to_wxyz(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to wxyz quaternion."""
    # Using scipy's method manually
    trace = np.trace(R)
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    
    return np.array([w, x, y, z])


def extract_hand_positions(hand_data: dict, convert_coords: bool = True) -> dict[str, np.ndarray]:
    """Extract 3D positions for all joints from hand data.
    
    Joint matrices are relative to the wrist, so we compute:
    global_joint = wrist @ joint_local
    
    Args:
        hand_data: Dictionary with joint matrices
        convert_coords: If True, convert from Y-up to Z-up
    """
    positions = {}
    
    # Get wrist matrix first (this is in global frame)
    if "wrist" not in hand_data or hand_data["wrist"] is None:
        return positions
    
    wrist_matrix = matrix_from_array(hand_data["wrist"])
    pos = get_position(wrist_matrix)
    positions["wrist"] = yup_to_zup(pos) if convert_coords else pos
    
    # All other joints are relative to wrist
    for joint_name in JOINT_NAMES:
        if joint_name == "wrist":
            continue
        if joint_name in hand_data and hand_data[joint_name] is not None:
            joint_local = matrix_from_array(hand_data[joint_name])
            joint_global = wrist_matrix @ joint_local
            pos = get_position(joint_global)
            positions[joint_name] = yup_to_zup(pos) if convert_coords else pos
    
    return positions


def compute_pinch_distance(positions: dict[str, np.ndarray]) -> float:
    """Compute distance between thumb tip and index tip."""
    if "thumbTip" in positions and "indexTip" in positions:
        return float(np.linalg.norm(positions["thumbTip"] - positions["indexTip"]))
    return float('inf')


# ============================================================================
# Tracking Visualizer Class
# ============================================================================

@dataclass
class PlaybackState:
    """Playback state for the visualizer."""
    is_playing: bool = False
    playback_speed: float = 1.0
    loop: bool = True
    current_frame: int = 0


class TrackingVisualizer:
    """Interactive 3D visualizer using Viser."""
    
    def __init__(self, frames: list[dict], metadata: dict | None = None, port: int = 8080,
                 video_path: Path | None = None):
        self.frames = frames
        self.metadata = metadata
        self.port = port
        self.video_path = video_path
        
        # Video capture
        self.video_cap = None
        self.video_frame_count = 0
        self.video_fps = 30.0
        if video_path and CV2_AVAILABLE and video_path.exists():
            self.video_cap = cv2.VideoCapture(str(video_path))
            self.video_frame_count = int(self.video_cap.get(cv2.CAP_PROP_FRAME_COUNT))
            self.video_fps = self.video_cap.get(cv2.CAP_PROP_FPS) or 30.0
            print(f"Loaded video: {video_path.name} ({self.video_frame_count} frames, {self.video_fps:.1f} fps)")
        
        # Parse extrinsic calibration
        self.extrinsic = None
        if metadata:
            ext_data = metadata.get("extrinsicCalibration")
            if ext_data:
                if isinstance(ext_data, str):
                    self.extrinsic = json.loads(ext_data)
                else:
                    self.extrinsic = ext_data
        
        # Playback state
        self.state = PlaybackState()
        self.state.current_frame = 0
        
        # Pre-compute some analytics
        self._precompute_analytics()
        
        # Initialize server
        self.server: Optional[viser.ViserServer] = None
        self._scene_handles = {}  # Store scene node handles for updates
        self._gui_handles = {}
        
    def _precompute_analytics(self):
        """Pre-compute analytics like velocities, distances."""
        n = len(self.frames)
        
        self.timestamps = np.zeros(n)
        self.left_wrist_pos = np.zeros((n, 3))
        self.right_wrist_pos = np.zeros((n, 3))
        self.head_pos = np.zeros((n, 3))
        self.left_pinch = np.zeros(n)
        self.right_pinch = np.zeros(n)
        self.has_left = np.zeros(n, dtype=bool)
        self.has_right = np.zeros(n, dtype=bool)
        self.has_head = np.zeros(n, dtype=bool)
        
        for i, frame in enumerate(self.frames):
            self.timestamps[i] = frame.get("timestamp", i / 60.0)
            
            # Head
            if frame.get("headMatrix"):
                head_mat = matrix_from_array(frame["headMatrix"])
                self.head_pos[i] = get_position(head_mat)
                self.has_head[i] = True
            
            # Left hand (don't convert coords for analytics - keep original)
            if frame.get("leftHand"):
                left_pos = extract_hand_positions(frame["leftHand"], convert_coords=False)
                if "wrist" in left_pos:
                    self.left_wrist_pos[i] = left_pos["wrist"]
                    self.has_left[i] = True
                    self.left_pinch[i] = compute_pinch_distance(left_pos)
            
            # Right hand (don't convert coords for analytics - keep original)
            if frame.get("rightHand"):
                right_pos = extract_hand_positions(frame["rightHand"], convert_coords=False)
                if "wrist" in right_pos:
                    self.right_wrist_pos[i] = right_pos["wrist"]
                    self.has_right[i] = True
                    self.right_pinch[i] = compute_pinch_distance(right_pos)
        
        # Compute velocities
        dt = np.diff(self.timestamps, prepend=self.timestamps[0])
        dt[dt == 0] = 1/60.0  # Avoid division by zero
        
        self.left_velocity = np.linalg.norm(np.diff(self.left_wrist_pos, axis=0, prepend=self.left_wrist_pos[:1]), axis=1) / dt
        self.right_velocity = np.linalg.norm(np.diff(self.right_wrist_pos, axis=0, prepend=self.right_wrist_pos[:1]), axis=1) / dt
        
        # Smooth velocities
        kernel_size = 5
        kernel = np.ones(kernel_size) / kernel_size
        self.left_velocity = np.convolve(self.left_velocity, kernel, mode='same')
        self.right_velocity = np.convolve(self.right_velocity, kernel, mode='same')
    
    def _setup_gui(self):
        """Set up GUI controls."""
        server = self.server
        
        # Header
        server.gui.add_markdown("## 🎯 VisionPro Tracking Visualizer")
        server.gui.add_markdown(f"**Frames:** {len(self.frames)} | **Duration:** {self.timestamps[-1]:.1f}s")
        
        # Playback controls folder
        with server.gui.add_folder("▶️ Playback", expand_by_default=True):
            # Timeline slider
            self._gui_handles["frame_slider"] = server.gui.add_slider(
                "Frame",
                min=0,
                max=len(self.frames) - 1,
                step=1,
                initial_value=0,
            )
            
            # Play/Pause button
            self._gui_handles["play_btn"] = server.gui.add_button(
                "▶ Play", icon=viser.Icon.PLAYER_PLAY
            )
            
            # Speed control
            self._gui_handles["speed"] = server.gui.add_slider(
                "Speed",
                min=0.1,
                max=3.0,
                step=0.1,
                initial_value=1.0,
            )
            
            # Loop checkbox
            self._gui_handles["loop"] = server.gui.add_checkbox(
                "Loop",
                initial_value=True,
            )
            
            # Step buttons
            with server.gui.add_folder("Step Controls"):
                self._gui_handles["prev_btn"] = server.gui.add_button(
                    "◀ Prev", icon=viser.Icon.ARROW_LEFT
                )
                self._gui_handles["next_btn"] = server.gui.add_button(
                    "Next ▶", icon=viser.Icon.ARROW_RIGHT
                )
                self._gui_handles["step_size"] = server.gui.add_slider(
                    "Step Size",
                    min=1,
                    max=100,
                    step=1,
                    initial_value=1,
                )
        
        # Visualization options
        with server.gui.add_folder("👁️ Visualization"):
            self._gui_handles["show_left"] = server.gui.add_checkbox(
                "Show Left Hand", initial_value=True
            )
            self._gui_handles["show_right"] = server.gui.add_checkbox(
                "Show Right Hand", initial_value=True
            )
            self._gui_handles["show_head"] = server.gui.add_checkbox(
                "Show Head Frame", initial_value=True
            )
            self._gui_handles["follow_head"] = server.gui.add_checkbox(
                "Follow Head (Camera)", initial_value=False
            )
            self._gui_handles["follow_distance"] = server.gui.add_slider(
                "Follow Distance",
                min=0.1,
                max=1.0,
                step=0.05,
                initial_value=0.3,
            )
            self._gui_handles["show_trajectory"] = server.gui.add_checkbox(
                "Show Trajectory", initial_value=False
            )
            self._gui_handles["trajectory_length"] = server.gui.add_slider(
                "Trajectory Length",
                min=10,
                max=500,
                step=10,
                initial_value=100,
            )
            self._gui_handles["joint_size"] = server.gui.add_slider(
                "Joint Size",
                min=0.005,
                max=0.05,
                step=0.002,
                initial_value=0.015,
            )
        
        # Video display options (if video is loaded)
        if self.video_cap is not None:
            with server.gui.add_folder("🎬 Video"):
                self._gui_handles["show_video"] = server.gui.add_checkbox(
                    "Show Video", initial_value=True
                )
                self._gui_handles["video_scale"] = server.gui.add_slider(
                    "Video Scale",
                    min=0.2,
                    max=2.0,
                    step=0.05,
                    initial_value=0.5,
                )
        
        # Setup callbacks
        self._setup_callbacks()
    
    def _setup_callbacks(self):
        """Set up GUI callbacks."""
        
        @self._gui_handles["frame_slider"].on_update
        def _(_):
            # Only update from slider when not playing (manual scrubbing)
            if not self.state.is_playing:
                new_frame = int(self._gui_handles["frame_slider"].value)
                if new_frame != self.state.current_frame:
                    self.state.current_frame = new_frame
                    self._update_scene()
        
        @self._gui_handles["play_btn"].on_click
        def _(_):
            self.state.is_playing = not self.state.is_playing
            if self.state.is_playing:
                self._gui_handles["play_btn"].label = "⏸ Pause"
            else:
                self._gui_handles["play_btn"].label = "▶ Play"
        
        @self._gui_handles["prev_btn"].on_click
        def _(_):
            step = int(self._gui_handles["step_size"].value)
            self.state.current_frame = max(0, self.state.current_frame - step)
            self._gui_handles["frame_slider"].value = self.state.current_frame
            self._update_scene()
        
        @self._gui_handles["next_btn"].on_click
        def _(_):
            step = int(self._gui_handles["step_size"].value)
            self.state.current_frame = min(len(self.frames) - 1, self.state.current_frame + step)
            self._gui_handles["frame_slider"].value = self.state.current_frame
            self._update_scene()
        
        # Visualization toggles
        for key in ["show_left", "show_right", "show_head",
                    "show_trajectory", "trajectory_length", "joint_size"]:
            @self._gui_handles[key].on_update
            def _(_):
                self._update_scene()
        
        # Speed control callback
        @self._gui_handles["speed"].on_update
        def _(_):
            self.state.playback_speed = self._gui_handles["speed"].value
        
        # Loop checkbox callback
        @self._gui_handles["loop"].on_update
        def _(_):
            self.state.loop = self._gui_handles["loop"].value
    
    def _draw_hand(self, hand_key: str, positions: dict[str, np.ndarray], 
                   color: tuple[float, float, float], pinch_dist: float):
        """Draw or update hand joints in the scene."""
        server = self.server
        base_path = f"/hands/{hand_key}"
        joint_size = self._gui_handles["joint_size"].value
        
        # Check if pinching
        is_pinching = pinch_dist < 0.02
        
        # Draw/update joints as spheres
        for joint_name, pos in positions.items():
            joint_path = f"{base_path}/joints/{joint_name}"
            
            # Highlight fingertips
            if joint_name in FINGERTIPS:
                size = joint_size * 1.3
                joint_color = PINCH_COLOR if is_pinching and joint_name in ["thumbTip", "indexTip"] else color
            else:
                size = joint_size
                joint_color = color
            
            # Check if we already have a handle for this joint
            if joint_path in self._scene_handles:
                # Update existing sphere position
                handle = self._scene_handles[joint_path]
                handle.position = tuple(pos)
            else:
                # Create new sphere and store handle
                handle = server.scene.add_icosphere(
                    joint_path,
                    radius=size,
                    position=tuple(pos),
                    color=joint_color,
                )
                self._scene_handles[joint_path] = handle
    
    def _draw_trajectory(self, hand_key: str, color: tuple[float, float, float]):
        """Draw hand trajectory."""
        server = self.server
        traj_length = int(self._gui_handles["trajectory_length"].value)
        
        start = max(0, self.state.current_frame - traj_length)
        end = self.state.current_frame + 1
        
        if hand_key == "left":
            positions = self.left_wrist_pos[start:end]
            valid = self.has_left[start:end]
        else:
            positions = self.right_wrist_pos[start:end]
            valid = self.has_right[start:end]
        
        # Filter valid positions and convert to Z-up
        valid_pos = np.array([yup_to_zup(p) for p in positions[valid]])
        
        if len(valid_pos) > 1:
            # Create line segments
            points = np.zeros((len(valid_pos) - 1, 2, 3), dtype=np.float32)
            for i in range(len(valid_pos) - 1):
                points[i, 0] = valid_pos[i]
                points[i, 1] = valid_pos[i + 1]
            
            # Fade colors based on recency
            colors = np.zeros((len(valid_pos) - 1, 2, 3), dtype=np.uint8)
            for i in range(len(valid_pos) - 1):
                alpha = (i + 1) / len(valid_pos)
                c = tuple(int(c * 255 * alpha) for c in color)
                colors[i, 0] = c
                colors[i, 1] = c
            
            server.scene.add_line_segments(
                f"/trajectory/{hand_key}",
                points=points,
                colors=colors,
                line_width=1.5,
            )
    
    def _draw_head_frame(self, head_matrix: np.ndarray):
        """Draw or update head coordinate frame."""
        server = self.server
        
        # Get position and convert from Y-up to Z-up
        pos = get_position(head_matrix)
        pos_zup = yup_to_zup(pos)
        
        # For rotation: we need to apply the Y-up to Z-up rotation
        # R_zup = R_coord_change @ R_original
        # The coordinate change rotation is 90 degrees around X axis
        rot = get_rotation_matrix(head_matrix)
        
        # Rotation matrix for Y-up to Z-up (90 deg around X)
        # This swaps Y and Z axes
        R_yup_to_zup = np.array([
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0]
        ], dtype=np.float64)
        
        rot_zup = R_yup_to_zup @ rot
        wxyz = rotation_to_wxyz(rot_zup)
        
        # Check if we already have a handle for the head frame
        if "/head" in self._scene_handles:
            handle = self._scene_handles["/head"]
            handle.position = tuple(pos_zup)
            handle.wxyz = tuple(wxyz)
        else:
            handle = server.scene.add_frame(
                "/head",
                wxyz=tuple(wxyz),
                position=tuple(pos_zup),
                axes_length=0.1,
                axes_radius=0.003,
            )
            self._scene_handles["/head"] = handle
    
    def _update_video_frame(self, frame_idx: int, head_matrix_data: list[float] | None = None):
        """Update video frame display synchronized with tracking data.
        
        Video is placed in front of head at -0.3 along head's Z axis,
        perpendicular to head's Z axis (facing the user).
        """
        if self.video_cap is None or head_matrix_data is None:
            return
        
        if not self._gui_handles.get("show_video", type('obj', (object,), {'value': True})).value:
            # Hide video if disabled
            self.server.scene.remove_by_name("/video")
            return
        
        # Map tracking frame index to video frame index
        if self.video_frame_count > 0:
            video_frame_idx = int(frame_idx * self.video_frame_count / len(self.frames))
            video_frame_idx = min(video_frame_idx, self.video_frame_count - 1)
        else:
            video_frame_idx = frame_idx
        
        # Seek to the frame
        self.video_cap.set(cv2.CAP_PROP_POS_FRAMES, video_frame_idx)
        ret, frame = self.video_cap.read()
        
        if not ret:
            return
        
        # Convert BGR to RGB and flip vertically (image coords vs 3D scene coords)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_rgb = cv2.flip(frame_rgb, 0)  # Flip vertically
        
        # Get scale from GUI
        scale = self._gui_handles.get("video_scale",
                                      type('obj', (object,), {'value': 2.0})).value
        
        # Calculate aspect ratio
        h, w = frame_rgb.shape[:2]
        aspect = w / h
        
        # Get head pose
        head_matrix = matrix_from_array(head_matrix_data)
        head_pos = get_position(head_matrix)
        head_pos_zup = yup_to_zup(head_pos)
        
        # Get head rotation and convert to Z-up
        rot = get_rotation_matrix(head_matrix)
        R_yup_to_zup = np.array([
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0]
        ], dtype=np.float64)
        rot_zup = R_yup_to_zup @ rot
        
        # Head's Z axis in world coordinates (column 2)
        head_z = rot_zup[:, 2]
        
        # Position video at -0.6 along head's Z axis (in front of head)
        # In ARKit, -Z is forward, so after conversion, we use -head_z
        video_pos = head_pos_zup - head_z * 0.6
        
        # Orientation: video should be perpendicular to head's Z axis, facing the head
        # We need to rotate 180 degrees around Y so the video faces back toward the head
        # The video's normal should point toward +head_z (back toward head)
        wxyz = rotation_to_wxyz(rot_zup)
        
        self.server.scene.add_image(
            "/video",
            frame_rgb,
            render_width=scale * aspect,
            render_height=scale,
            format="jpeg",
            wxyz=tuple(wxyz),
            position=tuple(video_pos),
        )

    def _update_camera_to_follow_head(self, head_matrix_data: list[float]):
        """Position camera behind the head, looking at the head position.
        
        Camera is placed at head_position + distance along the head's +Z axis
        (behind the head in ARKit coordinates), then looking at the head.
        """
        head_matrix = matrix_from_array(head_matrix_data)
        
        # Get head position in Z-up coordinates
        head_pos = get_position(head_matrix)
        head_pos_zup = yup_to_zup(head_pos)
        
        # Get the rotation matrix and convert to Z-up
        rot = get_rotation_matrix(head_matrix)
        R_yup_to_zup = np.array([
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0]
        ], dtype=np.float64)
        rot_zup = R_yup_to_zup @ rot
        
        # In ARKit, the head's forward direction is -Z, so +Z is backward (behind the head)
        # After Y-up to Z-up conversion, the original Z-axis becomes the Y-axis
        # So the head's +Z axis (behind) is the original Z column, which after transform is:
        # Original Z-column -> after R_yup_to_zup becomes the new Y direction
        # Actually, let's get the Z-axis of the head directly from the rotated matrix
        # Column 2 (Z-axis) of rot_zup gives us where the head's Z-axis points in world coords
        head_z_axis = rot_zup[:, 2]  # Z column - the head's local +Z in world coordinates
        
        # Camera distance behind head (along head's +Z axis)
        distance = self._gui_handles["follow_distance"].value
        
        # Position camera behind the head (along +Z axis of the head)
        camera_pos = head_pos_zup + head_z_axis * distance
        
        # Look at the head position
        look_at = head_pos_zup
        
        # The up vector is world Z
        up = np.array([0.0, 0.0, 1.0])
        
        # Set camera for all connected clients
        for client in self.server.get_clients().values():
            client.camera.position = tuple(camera_pos)
            client.camera.look_at = tuple(look_at)
            client.camera.up_direction = tuple(up)

    def _update_scene(self):
        """Update the 3D scene for the current frame."""
        if self.server is None:
            return
        
        idx = self.state.current_frame
        frame = self.frames[idx]
        
        # Track visibility for hands
        left_visible = False
        right_visible = False
        head_visible = False
        
        # Draw left hand
        if self._gui_handles["show_left"].value and frame.get("leftHand"):
            left_positions = extract_hand_positions(frame["leftHand"])
            if left_positions:
                self._draw_hand("left", left_positions, LEFT_HAND_COLOR, self.left_pinch[idx])
                left_visible = True
                
                if self._gui_handles["show_trajectory"].value:
                    self._draw_trajectory("left", LEFT_HAND_COLOR)
        
        # Draw right hand
        if self._gui_handles["show_right"].value and frame.get("rightHand"):
            right_positions = extract_hand_positions(frame["rightHand"])
            if right_positions:
                self._draw_hand("right", right_positions, RIGHT_HAND_COLOR, self.right_pinch[idx])
                right_visible = True
                
                if self._gui_handles["show_trajectory"].value:
                    self._draw_trajectory("right", RIGHT_HAND_COLOR)
        
        # Draw head frame
        if self._gui_handles["show_head"].value and frame.get("headMatrix"):
            head_matrix = matrix_from_array(frame["headMatrix"])
            self._draw_head_frame(head_matrix)
            head_visible = True
        
        # Follow head with camera if enabled
        if self._gui_handles["follow_head"].value and frame.get("headMatrix"):
            self._update_camera_to_follow_head(frame["headMatrix"])
        
        # Update video frame (synchronized with tracking)
        if self.video_cap is not None:
            self._update_video_frame(idx, frame.get("headMatrix"))
        
        # Set visibility for elements - only do this when NOT playing to reduce overhead
        if not self.state.is_playing:
            for joint_name in JOINT_NAMES:
                left_path = f"/hands/left/joints/{joint_name}"
                right_path = f"/hands/right/joints/{joint_name}"
                
                if left_path in self._scene_handles:
                    self._scene_handles[left_path].visible = left_visible
                if right_path in self._scene_handles:
                    self._scene_handles[right_path].visible = right_visible
            
            if "/head" in self._scene_handles:
                self._scene_handles["/head"].visible = head_visible
            
            # Hide trajectories if not showing
            if not self._gui_handles["show_trajectory"].value:
                self.server.scene.remove_by_name("/trajectory/left")
                self.server.scene.remove_by_name("/trajectory/right")
        

    
    def _playback_loop(self):
        """Background thread for playback."""
        last_time = time.time()
        last_slider_update = 0
        
        while True:
            time.sleep(0.008)  # ~120Hz loop for smooth playback
            
            if not self.state.is_playing:
                last_time = time.time()
                continue
            
            current_time = time.time()
            elapsed = current_time - last_time
            
            # Use fixed frame rate for smoother playback (30 fps base)
            target_dt = (1.0 / 30.0) / self.state.playback_speed
            
            if elapsed >= target_dt:
                last_time = current_time
                
                # Advance frame
                next_frame = self.state.current_frame + 1
                
                if next_frame >= len(self.frames):
                    if self._gui_handles["loop"].value:
                        next_frame = 0
                    else:
                        self.state.is_playing = False
                        self._gui_handles["play_btn"].label = "▶ Play"
                        continue
                
                self.state.current_frame = next_frame
                
                # Update scene directly during playback
                self._update_scene()
                
                # Update slider less frequently (every 10 frames) to reduce network chatter
                if next_frame - last_slider_update >= 10 or next_frame == 0:
                    self._gui_handles["frame_slider"].value = next_frame
                    last_slider_update = next_frame
    
    def run(self):
        """Start the visualizer server."""
        self.server = viser.ViserServer(port=self.port)
        
        # Set up GUI
        self._setup_gui()
        
        # Add checkerboard ground plane with alternating colored patches
        cell_size = 2.5  # 2.5m cells (10x larger)
        grid_extent = 15.0  # +/- 15m (10x larger)
        color1 = (140, 140, 140)  # Gray
        color2 = (180, 180, 180)  # Light gray
        
        # Create each cell as a separate mesh (allows different colors)
        cell_idx = 0
        for i, x in enumerate(np.arange(-grid_extent, grid_extent, cell_size)):
            for j, y in enumerate(np.arange(-grid_extent, grid_extent, cell_size)):
                # Four corners of this cell
                vertices = np.array([
                    [x, y, -0.001],
                    [x + cell_size, y, -0.001],
                    [x + cell_size, y + cell_size, -0.001],
                    [x, y + cell_size, -0.001],
                ], dtype=np.float32)
                faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)
                
                # Checkerboard pattern
                color = color1 if (i + j) % 2 == 0 else color2
                
                self.server.scene.add_mesh_simple(
                    f"/ground/cell_{cell_idx}",
                    vertices=vertices,
                    faces=faces,
                    color=color,
                )
                cell_idx += 1
        
        # Add grid lines on top for reference
        self.server.scene.add_grid(
            "/grid",
            width=30.0,
            height=30.0,
            plane="xy",
            position=(0, 0, 0),
            cell_color=(120, 120, 120),     # Gray grid lines
            section_color=(100, 100, 100),  # Darker section lines
            cell_size=2.5,                   # Match cell size
            section_size=5.0,                # 5m sections
        )
        
        # Set up direction (Z-up for visualization, we convert from Y-up ARKit)
        self.server.scene.set_up_direction("+z")
        
        # Initial scene
        self._update_scene()
        
        # Start playback thread
        playback_thread = threading.Thread(target=self._playback_loop, daemon=True)
        playback_thread.start()
        
        print(f"\n{'='*60}")
        print(f"🎯 VisionPro Tracking Visualizer is running!")
        print(f"   Open in browser: http://localhost:{self.port}")
        print(f"{'='*60}")
        print("\nControls:")
        print("  - Use the timeline slider to scrub through frames")
        print("  - Click Play to animate the recording")
        print("  - Toggle visibility options in Visualization folder")
        print("  - View velocity and pinch graphs in Analytics")
        print("\nPress Ctrl+C to stop")
        
        try:
            while True:
                # Update speed from GUI
                self.state.playback_speed = self._gui_handles["speed"].value
                self.state.loop = self._gui_handles["loop"].value
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nShutting down...")


# ============================================================================
# Main Entry Point
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Interactive 3D visualizer for VisionProTeleop hand tracking data using Viser."
    )
    parser.add_argument(
        "path",
        type=str,
        help="Path to the recording directory or tracking.jsonl file"
    )
    parser.add_argument(
        "--port", "-p",
        type=int,
        default=8080,
        help="Port for the Viser server (default: 8080)"
    )
    parser.add_argument(
        "--video", "-v",
        type=str,
        default=None,
        help="Path to video file (mp4) to display synchronized with tracking"
    )
    
    args = parser.parse_args()
    
    path = Path(args.path)
    if not path.exists():
        print(f"Error: Path not found: {path}")
        return 1
    
    # Look for video file
    video_path = None
    if args.video:
        video_path = Path(args.video)
        if not video_path.exists():
            print(f"Warning: Video file not found: {video_path}")
            video_path = None
    else:
        # Auto-detect video in same directory
        if path.is_dir():
            for ext in ['.mp4', '.MP4', '.mov', '.MOV']:
                candidate = path / f"video{ext}"
                if candidate.exists():
                    video_path = candidate
                    break
        else:
            # Look in parent directory of tracking file
            parent = path.parent
            for ext in ['.mp4', '.MP4', '.mov', '.MOV']:
                candidate = parent / f"video{ext}"
                if candidate.exists():
                    video_path = candidate
                    break
    
    try:
        frames, metadata = load_tracking_data(path)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return 1
    
    if len(frames) == 0:
        print("Error: No frames found in tracking data")
        return 1
    
    visualizer = TrackingVisualizer(frames, metadata, port=args.port, video_path=video_path)
    visualizer.run()
    
    return 0


if __name__ == "__main__":
    exit(main())
