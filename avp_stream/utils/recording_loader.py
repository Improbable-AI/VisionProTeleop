"""
Simple JSONL loader for VisionPro tracking recordings.

Example:
    from avp_stream import load_jsonl
    
    frames = load_jsonl("path/to/tracking.jsonl")
    for frame in frames:
        print(frame.right.indexTip)
"""

import json
import numpy as np
from pathlib import Path
from typing import Dict, List, Any, Union


# YUP2ZUP transform
YUP2ZUP = np.array([[[1, 0, 0, 0], 
                    [0, 0, -1, 0], 
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]]], dtype=np.float64)


# Joint name to index mapping
JOINT_MAPPING = {
    'wrist': 0,
    'thumbKnuckle': 1, 'thumbIntermediateBase': 2, 'thumbIntermediateTip': 3, 'thumbTip': 4,
    'indexMetacarpal': 5, 'indexKnuckle': 6, 'indexIntermediateBase': 7, 'indexIntermediateTip': 8, 'indexTip': 9,
    'middleMetacarpal': 10, 'middleKnuckle': 11, 'middleIntermediateBase': 12, 'middleIntermediateTip': 13, 'middleTip': 14,
    'ringMetacarpal': 15, 'ringKnuckle': 16, 'ringIntermediateBase': 17, 'ringIntermediateTip': 18, 'ringTip': 19,
    'littleMetacarpal': 20, 'littleKnuckle': 21, 'littleIntermediateBase': 22, 'littleIntermediateTip': 23, 'littleTip': 24,
    'forearmWrist': 25, 'forearmArm': 26,
}


def _matrix_from_flat(flat: List[float]) -> np.ndarray:
    """Convert 16-element column-major list to 4x4 matrix."""
    if flat is None or len(flat) != 16:
        return np.eye(4, dtype=np.float64)
    return np.array(flat, dtype=np.float64).reshape(4, 4).T


def _parse_hand(hand_data: Dict[str, Any]) -> np.ndarray:
    """Parse hand data to (27, 4, 4) array."""
    joints = np.zeros((27, 4, 4), dtype=np.float64)
    for i in range(27):
        joints[i] = np.eye(4)
    if hand_data is None:
        return joints
    for field, idx in JOINT_MAPPING.items():
        if field in hand_data and hand_data[field] is not None:
            joints[idx] = _matrix_from_flat(hand_data[field])
    return joints


def _parse_frame(frame_data: Dict[str, Any]) -> Dict[str, Any]:
    """Parse a raw JSONL frame to get_latest() format dict."""
    # Head: Apply YUP2ZUP then rotate -90 degrees around X (same as rotate_head in grpc_utils)
    head = _matrix_from_flat(frame_data.get('headMatrix')).reshape(1, 4, 4)
    head = YUP2ZUP @ head
    # Rotate -90 degrees around X axis (matching rotate_head(R, degrees=-90))
    theta = np.radians(-90)
    rot_x = np.array([[[1, 0, 0, 0],
                       [0, np.cos(theta), -np.sin(theta), 0],
                       [0, np.sin(theta), np.cos(theta), 0],
                       [0, 0, 0, 1]]], dtype=np.float64)
    head = head @ rot_x
    
    # Hands
    left_joints = _parse_hand(frame_data.get('leftHand'))
    right_joints = _parse_hand(frame_data.get('rightHand'))
    
    left_wrist = YUP2ZUP @ left_joints[0:1].copy()
    right_wrist = YUP2ZUP @ right_joints[0:1].copy()
    
    # Pinch distance (same as get_pinch_distance in grpc_utils)
    def pinch(wrist, joints):
        thumb = wrist[0] @ joints[4]
        index = wrist[0] @ joints[9]
        return float(np.linalg.norm(thumb[:3, 3] - index[:3, 3]))
    
    # Wrist roll (same as get_wrist_roll in grpc_utils)
    def wrist_roll(mat):
        R = mat[0, :3, :3]
        theta_z = np.arctan2(R[1, 0], R[0, 0])
        Rz = np.array([
            [np.cos(-theta_z), -np.sin(-theta_z), 0],
            [np.sin(-theta_z), np.cos(-theta_z), 0],
            [0, 0, 1]
        ])
        R_after_z = Rz @ R
        theta_y = np.arctan2(R_after_z[0, 2], R_after_z[0, 0])
        Ry = np.array([
            [np.cos(-theta_y), 0, np.sin(-theta_y)],
            [0, 1, 0],
            [-np.sin(-theta_y), 0, np.cos(-theta_y)]
        ])
        R_after_y = Ry @ R_after_z
        theta_x = np.arctan2(R_after_y[1, 2], R_after_y[1, 1])
        return float(theta_x)
    
    return {
        'head': head,
        'left_wrist': left_wrist,
        'right_wrist': right_wrist,
        'left_fingers': left_joints[:25],
        'right_fingers': right_joints[:25],
        'left_arm': left_joints,
        'right_arm': right_joints,
        'left_pinch_distance': pinch(left_wrist, left_joints),
        'right_pinch_distance': pinch(right_wrist, right_joints),
        'left_wrist_roll': wrist_roll(left_wrist),
        'right_wrist_roll': wrist_roll(right_wrist),
        'timestamp': frame_data.get('timestamp', 0.0),
    }


def load_jsonl(jsonl_file: Union[str, Path]) -> List["TrackingData"]:
    """Load a tracking.jsonl file and return a list of TrackingData objects.
    
    Args:
        jsonl_file: Path to tracking.jsonl file
    
    Returns:
        List of TrackingData objects with both dict and attribute access
    
    Example::
    
        from avp_stream import load_jsonl
        
        frames = load_jsonl("downloads/my_recording/tracking.jsonl")
        
        for frame in frames:
            # Attribute access (world frame)
            head_pos = frame.head[:3, 3]
            index_tip = frame.right.indexTip
            
            # Dict access (wrist-relative)
            head_pos = frame["head"][0, :3, 3]
    """
    from avp_stream.streamer import TrackingData
    
    path = Path(jsonl_file)
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")
    
    frames = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    raw = json.loads(line)
                    frames.append(TrackingData(_parse_frame(raw)))
                except json.JSONDecodeError:
                    continue
    
    return frames
