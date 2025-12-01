#!/usr/bin/env python3
"""
Visualize hand tracking keypoints and head frame from VisionProTeleop recordings.

Usage:
    python visualize_tracking.py /path/to/recording_directory

The recording directory should contain:
    - tracking.jsonl: Hand and head tracking data
    - metadata.json: Recording metadata (optional, for extrinsic calibration)
"""

import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
from pathlib import Path


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

# Joint names in order
JOINT_NAMES = [
    "wrist",
    "thumbCMC", "thumbMP", "thumbIP", "thumbTip",
    "indexMCP", "indexPIP", "indexDIP", "indexTip",
    "middleMCP", "middlePIP", "middleDIP", "middleTip",
    "ringMCP", "ringPIP", "ringDIP", "ringTip",
    "littleMCP", "littlePIP", "littleDIP", "littleTip",
]


def load_tracking_data(directory: Path) -> tuple[list[dict], dict | None]:
    """Load tracking data and metadata from a recording directory."""
    tracking_file = directory / "tracking.jsonl"
    metadata_file = directory / "metadata.json"
    
    if not tracking_file.exists():
        raise FileNotFoundError(f"Tracking file not found: {tracking_file}")
    
    # Load tracking frames
    frames = []
    with open(tracking_file, "r") as f:
        for line in f:
            line = line.strip()
            if line:
                frames.append(json.loads(line))
    
    # Load metadata if available
    metadata = None
    if metadata_file.exists():
        with open(metadata_file, "r") as f:
            metadata = json.load(f)
    
    return frames, metadata


def matrix_from_array(arr: list[float]) -> np.ndarray:
    """Convert a 16-element column-major array to a 4x4 matrix."""
    return np.array(arr, dtype=np.float32).reshape(4, 4, order='F')


def get_position(matrix: np.ndarray) -> np.ndarray:
    """Extract translation (position) from a 4x4 transformation matrix."""
    return matrix[:3, 3]


def swap_yz(point: np.ndarray) -> np.ndarray:
    """No coordinate swap - display in native ARKit coordinates (Y-up)."""
    return point


def get_axes(matrix: np.ndarray, scale: float = 0.05) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Extract X, Y, Z axes from a 4x4 transformation matrix."""
    origin = matrix[:3, 3]
    x_axis = origin + matrix[:3, 0] * scale
    y_axis = origin + matrix[:3, 1] * scale
    z_axis = origin + matrix[:3, 2] * scale
    return x_axis, y_axis, z_axis


def extract_hand_positions(hand_data: dict) -> dict[str, np.ndarray]:
    """Extract 3D positions for all joints from hand data.
    
    Joint matrices are relative to the wrist, so we compute:
    global_joint = wrist @ joint_local
    """
    positions = {}
    
    # Get wrist matrix first (this is in global frame)
    if "wrist" not in hand_data or hand_data["wrist"] is None:
        return positions
    
    wrist_matrix = matrix_from_array(hand_data["wrist"])
    positions["wrist"] = get_position(wrist_matrix)
    
    # All other joints are relative to wrist
    for joint_name in JOINT_NAMES:
        if joint_name == "wrist":
            continue
        if joint_name in hand_data and hand_data[joint_name] is not None:
            joint_local = matrix_from_array(hand_data[joint_name])
            joint_global = wrist_matrix @ joint_local
            positions[joint_name] = get_position(joint_global)
    
    return positions


def draw_coordinate_frame(ax, matrix: np.ndarray, label: str, scale: float = 0.1):
    """Draw a coordinate frame (XYZ axes) at the given transform."""
    origin = swap_yz(get_position(matrix))
    x_end = swap_yz(get_position(matrix) + matrix[:3, 0] * scale)
    y_end = swap_yz(get_position(matrix) + matrix[:3, 1] * scale)
    z_end = swap_yz(get_position(matrix) + matrix[:3, 2] * scale)
    
    # Draw axes (X=red, Y=green, Z=blue)
    ax.plot([origin[0], x_end[0]], [origin[1], x_end[1]], [origin[2], x_end[2]], 
            'r-', linewidth=2, label=f'{label} X' if label else None)
    ax.plot([origin[0], y_end[0]], [origin[1], y_end[1]], [origin[2], y_end[2]], 
            'g-', linewidth=2, label=f'{label} Y' if label else None)
    ax.plot([origin[0], z_end[0]], [origin[1], z_end[1]], [origin[2], z_end[2]], 
            'b-', linewidth=2, label=f'{label} Z' if label else None)
    
    # Label
    if label:
        ax.text(origin[0], origin[1], origin[2] + scale * 1.2, label, fontsize=10, fontweight='bold')


def draw_hand(ax, hand_positions: dict[str, np.ndarray], color: str, label: str):
    """Draw a hand skeleton."""
    if not hand_positions:
        return
    
    # Swap Y/Z for all positions
    swapped_positions = {k: swap_yz(v) for k, v in hand_positions.items()}
    
    # Draw joints
    positions = np.array(list(swapped_positions.values()))
    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
               c=color, s=20, alpha=0.8, label=label)
    
    # Draw connections
    for joint1, joint2 in HAND_CONNECTIONS:
        if joint1 in swapped_positions and joint2 in swapped_positions:
            p1 = swapped_positions[joint1]
            p2 = swapped_positions[joint2]
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 
                    color=color, linewidth=1.5, alpha=0.7)


def draw_camera_frame(ax, head_matrix: np.ndarray, extrinsic: dict | None, scale: float = 0.08):
    """Draw camera frame if extrinsic calibration is available.
    
    The extrinsic calibration stores T_head^camera (transforms points from head to camera).
    To visualize the camera's POSE in the head frame, we need the inverse: T_camera^head.
    
    The camera's Z-axis in ARKit convention points BACKWARD, so the camera looks in -Z direction.
    """
    if extrinsic is None:
        return
    
    def draw_camera(T_head_camera: np.ndarray, label: str, cam_scale: float):
        """Draw a single camera with its coordinate frame and viewing direction."""
        # T_head^camera transforms points from head to camera
        # To get camera pose in head frame, we need the inverse: T_camera^head
        T_camera_in_head = np.linalg.inv(T_head_camera)
        
        # Now transform from head frame to world frame
        T_world_camera = head_matrix @ T_camera_in_head
        
        # Draw coordinate frame
        draw_coordinate_frame(ax, T_world_camera, label, cam_scale)
        
        # Draw viewing direction (camera looks in -Z direction in ARKit convention)
        origin = get_position(T_world_camera)
        # -Z direction is the viewing direction
        look_dir = -T_world_camera[:3, 2]  # Negative Z axis
        look_end = origin + look_dir * cam_scale * 2
        
        # Draw a dashed cyan line for viewing direction
        ax.plot([origin[0], look_end[0]], [origin[1], look_end[1]], [origin[2], look_end[2]], 
                'c--', linewidth=1.5, alpha=0.7)
    
    # Get head-to-camera transform
    left_h2c = extrinsic.get("leftHeadToCamera")
    if left_h2c:
        T_head_camera = matrix_from_array(left_h2c)
        draw_camera(T_head_camera, "L-Cam", scale)
    
    # Right camera for stereo
    right_h2c = extrinsic.get("rightHeadToCamera")
    if right_h2c:
        T_head_camera_r = matrix_from_array(right_h2c)
        draw_camera(T_head_camera_r, "R-Cam", scale * 0.8)


def print_calibration_info(extrinsic: dict):
    """Print detailed calibration information."""
    print("\n=== Extrinsic Calibration Info ===")
    
    # Show if baseline constraint was used
    if extrinsic.get("stereoBaselineMeters"):
        print(f"Stereo baseline constraint: {extrinsic['stereoBaselineMeters']*100:.1f} cm (enforced)")
    
    left_h2c = extrinsic.get("leftHeadToCamera")
    left_cam_pos = None
    if left_h2c:
        T_head_camera = matrix_from_array(left_h2c)
        # Camera pose in head frame = inverse of T_head^camera
        T_camera_in_head = np.linalg.inv(T_head_camera)
        left_cam_pos = get_position(T_camera_in_head)
        print(f"\nLeft Camera pose in head frame:")
        print(f"  Position: x={left_cam_pos[0]*100:.2f}cm, y={left_cam_pos[1]*100:.2f}cm, z={left_cam_pos[2]*100:.2f}cm")
        print(f"  (Negative Z = in front of head)")
    
    right_h2c = extrinsic.get("rightHeadToCamera")


def print_intrinsic_info(intrinsic: dict):
    """Print intrinsic calibration information."""
    print("\n=== Intrinsic Calibration Info ===")
    
    device_name = intrinsic.get("deviceName", "Unknown")
    is_stereo = intrinsic.get("isStereo", False)
    print(f"Device: {device_name} ({'Stereo' if is_stereo else 'Mono'})")
    
    left = intrinsic.get("leftIntrinsics")
    if left:
        print(f"\nLeft Camera Intrinsics:")
        print(f"  Resolution: {left.get('imageWidth', 'N/A')}x{left.get('imageHeight', 'N/A')}")
        print(f"  Focal length: fx={left.get('fx', 0):.2f}, fy={left.get('fy', 0):.2f}")
        print(f"  Principal point: cx={left.get('cx', 0):.2f}, cy={left.get('cy', 0):.2f}")
        dist = left.get('distortionCoeffs', [])
        if dist:
            print(f"  Distortion: [{', '.join(f'{d:.4f}' for d in dist[:5])}]")
        print(f"  Reproj error: {left.get('reprojectionError', 0):.4f} px")
    
    right = intrinsic.get("rightIntrinsics")
    if right:
        print(f"\nRight Camera Intrinsics:")
        print(f"  Resolution: {right.get('imageWidth', 'N/A')}x{right.get('imageHeight', 'N/A')}")
        print(f"  Focal length: fx={right.get('fx', 0):.2f}, fy={right.get('fy', 0):.2f}")
        print(f"  Principal point: cx={right.get('cx', 0):.2f}, cy={right.get('cy', 0):.2f}")
        dist = right.get('distortionCoeffs', [])
        if dist:
            print(f"  Distortion: [{', '.join(f'{d:.4f}' for d in dist[:5])}]")
        print(f"  Reproj error: {right.get('reprojectionError', 0):.4f} px")
    
    print("=" * 35)


def print_extrinsic_info(extrinsic: dict):
    if right_h2c:
        T_head_camera_r = matrix_from_array(right_h2c)
        T_camera_in_head_r = np.linalg.inv(T_head_camera_r)
        right_cam_pos = get_position(T_camera_in_head_r)
        print(f"\nRight Camera pose in head frame:")
        print(f"  Position: x={right_cam_pos[0]*100:.2f}cm, y={right_cam_pos[1]*100:.2f}cm, z={right_cam_pos[2]*100:.2f}cm")
        
        if left_cam_pos is not None:
            baseline = np.linalg.norm(right_cam_pos - left_cam_pos)
            x_baseline = right_cam_pos[0] - left_cam_pos[0]
            midpoint_x = (left_cam_pos[0] + right_cam_pos[0]) / 2
            print(f"\nStereo baseline: {baseline*100:.2f} cm (X separation: {x_baseline*100:.2f} cm)")
            print(f"Midpoint X: {midpoint_x*100:.2f} cm (ideally ~0)")
    
    print("=" * 35)


class TrackingVisualizer:
    """Interactive visualizer for tracking data."""
    
    def __init__(self, frames: list[dict], metadata: dict | None = None):
        self.frames = frames
        self.metadata = metadata
        
        # Parse extrinsic calibration (may be stored as JSON string)
        self.extrinsic = None
        if metadata:
            ext_data = metadata.get("extrinsicCalibration")
            if ext_data:
                if isinstance(ext_data, str):
                    # Parse JSON string
                    self.extrinsic = json.loads(ext_data)
                else:
                    self.extrinsic = ext_data
                
                # Print calibration details
                print_calibration_info(self.extrinsic)
        
        self.current_frame = 0
        self.azim = -60  # Horizontal rotation angle (around Y axis)
        
        # Set up figure
        self.fig = plt.figure(figsize=(14, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Add slider for frame selection
        slider_ax = self.fig.add_axes([0.2, 0.02, 0.6, 0.03])
        self.slider = Slider(slider_ax, 'Frame', 0, len(frames) - 1, 
                            valinit=0, valstep=1)
        self.slider.on_changed(self.update_frame)
        
        # Track mouse for rotation
        self.last_mouse_x = None
        self.is_dragging = False
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        
        # Initial draw
        self.update_frame(0)
        
        # Set up keyboard navigation
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
    
    def on_mouse_press(self, event):
        """Handle mouse button press."""
        if event.inaxes == self.ax and event.button == 1:
            self.is_dragging = True
            self.last_mouse_x = event.x
    
    def on_mouse_release(self, event):
        """Handle mouse button release."""
        if event.button == 1:
            self.is_dragging = False
            self.last_mouse_x = None
    
    def on_mouse_move(self, event):
        """Handle mouse movement - rotate only around Y axis."""
        if self.is_dragging and self.last_mouse_x is not None and event.x is not None:
            dx = event.x - self.last_mouse_x
            self.azim += dx * 0.5  # Sensitivity
            self.last_mouse_x = event.x
            self.ax.view_init(elev=25, azim=self.azim)  # Fixed elevation (Y-up view)
            self.fig.canvas.draw_idle()
    
    def update_frame(self, frame_idx):
        """Update the visualization for a given frame."""
        self.current_frame = int(frame_idx)
        frame = self.frames[self.current_frame]
        
        self.ax.clear()
        
        # Draw head frame
        if frame.get("headMatrix"):
            head_matrix = matrix_from_array(frame["headMatrix"])
            draw_coordinate_frame(self.ax, head_matrix, "Head", scale=0.1)
            
            # Draw camera if extrinsic calibration available
            draw_camera_frame(self.ax, head_matrix, self.extrinsic)
        else:
            head_matrix = None
        
        # Set axis labels (native ARKit coordinates: X-right, Y-up, Z-backward)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        
        # Auto-scale based on data with equal aspect ratio
        self._set_axis_limits(frame)
        
        # Set viewing angle
        self.ax.view_init(elev=20, azim=self.azim)
        
        # Title with frame info
        timestamp = frame.get("timestamp", 0)
        self.ax.set_title(f'Frame {self.current_frame + 1}/{len(self.frames)} | Time: {timestamp:.2f}s | '
                         f'Head: {"✓" if frame.get("headMatrix") else "✗"}')
        
        self.ax.legend(loc='upper left', fontsize=8)
        self.fig.canvas.draw_idle()
    
    def _set_axis_limits(self, frame):
        """Set axis limits centered on head with fixed range to see cameras."""
        if not frame.get("headMatrix"):
            # Default view
            self.ax.set_xlim([-0.5, 0.5])
            self.ax.set_ylim([-0.5, 0.5])
            self.ax.set_zlim([-0.5, 0.5])
            self.ax.set_box_aspect([1, 1, 1])
            return
        
        head_pos = get_position(matrix_from_array(frame["headMatrix"]))
        
        # Fixed range to show head + cameras (about 30cm in each direction)
        view_range = 0.3
        
        # Set limits centered on head
        self.ax.set_xlim([head_pos[0] - view_range, head_pos[0] + view_range])
        self.ax.set_ylim([head_pos[1] - view_range, head_pos[1] + view_range])
        self.ax.set_zlim([head_pos[2] - view_range, head_pos[2] + view_range])
        
        # Force equal aspect ratio
        self.ax.set_box_aspect([1, 1, 1])
    
    def on_key(self, event):
        """Handle keyboard events."""
        if event.key == 'right' or event.key == 'n':
            new_frame = min(self.current_frame + 1, len(self.frames) - 1)
            self.slider.set_val(new_frame)
        elif event.key == 'left' or event.key == 'p':
            new_frame = max(self.current_frame - 1, 0)
            self.slider.set_val(new_frame)
        elif event.key == 'home':
            self.slider.set_val(0)
        elif event.key == 'end':
            self.slider.set_val(len(self.frames) - 1)
        elif event.key == ' ':
            # Toggle play (simple step forward)
            new_frame = (self.current_frame + 1) % len(self.frames)
            self.slider.set_val(new_frame)
    
    def show(self):
        """Display the visualization."""
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Visualize hand tracking and head pose from VisionProTeleop recordings."
    )
    parser.add_argument(
        "directory",
        type=str,
        help="Path to the recording directory containing tracking.jsonl and metadata.json"
    )
    parser.add_argument(
        "--frame", "-f",
        type=int,
        default=0,
        help="Starting frame index (default: 0)"
    )
    
    args = parser.parse_args()
    
    directory = Path(args.directory)
    if not directory.exists():
        print(f"Error: Directory not found: {directory}")
        return 1
    
    print(f"Loading tracking data from: {directory}")
    
    try:
        frames, metadata = load_tracking_data(directory)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return 1
    
    print(f"Loaded {len(frames)} frames")
    
    if metadata:
        print(f"Metadata: duration={metadata.get('duration', 'N/A'):.1f}s, "
              f"fps={metadata.get('averageFPS', 'N/A'):.1f}")
        if metadata.get("extrinsicCalibration"):
            print("Extrinsic calibration: Available ✓")
        else:
            print("Extrinsic calibration: Not available")
    
    print("\nControls:")
    print("  Left/Right arrows or P/N: Previous/Next frame")
    print("  Home/End: First/Last frame")
    print("  Space: Step forward")
    print("  Drag: Rotate view")
    print("  Scroll: Zoom")
    
    visualizer = TrackingVisualizer(frames, metadata)
    if args.frame > 0:
        visualizer.slider.set_val(min(args.frame, len(frames) - 1))
    visualizer.show()
    
    return 0


if __name__ == "__main__":
    exit(main())
