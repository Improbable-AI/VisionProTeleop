#!/usr/bin/env python3
"""
Web-Based Camera Calibration
=============================

Flask-based web UI for camera calibration with MJPEG video streaming.
Provides a browser-based interface for:
1. Camera selection with live preview
2. Intrinsic calibration (ChArUco)
3. Intrinsic verification (ArUco distance check)
4. Extrinsic calibration (with Vision Pro)

Usage:
    python -m avp_stream.calibration_server
    # Then open http://localhost:5050 in your browser

Requirements:
    pip install flask
"""

import cv2
import numpy as np
import json
import time
import threading
import webbrowser
from typing import Dict, List, Optional, Tuple, Any
from pathlib import Path
from queue import Queue
from flask import Flask, Response, render_template_string, jsonify, request

# Import calibration logic from run_calibration
from avp_stream.run_calibration import (
    discover_cameras,
    IntrinsicCalibrator,
    split_stereo_image,
    CalibrationResult,
    ExtrinsicCalibrator,
    CHARUCO_ROWS,
    CHARUCO_COLS,
    CHARUCO_SQUARE_SIZE,
    CHARUCO_MARKER_SIZE,
    VERIFICATION_TAG_SIZE,
    VERIFICATION_MARGIN,
    VERIFICATION_TAG_IDS,
    EXTRINSIC_MARKER_SIZE_PRINT,
    EXTRINSIC_MARKER_SIZE_IPHONE,
    EXTRINSIC_MARKER_IDS,
)
from avp_stream.verification_engine import VerificationEngine

app = Flask(__name__)

CHECKPOINT_FILE = Path("calibration_checkpoint.json")


def scale_camera_matrix(camera_matrix: np.ndarray, 
                        orig_size: Tuple[int, int], 
                        new_size: Tuple[int, int]) -> np.ndarray:
    """Scale camera matrix from one resolution to another.
    
    Args:
        camera_matrix: 3x3 camera intrinsic matrix (fx, fy, cx, cy)
        orig_size: (width, height) of original calibration
        new_size: (width, height) of current frame
    
    Returns:
        Scaled camera matrix
    """
    if orig_size == new_size:
        return camera_matrix
    
    scale_x = new_size[0] / orig_size[0]
    scale_y = new_size[1] / orig_size[1]
    
    scaled = camera_matrix.copy()
    scaled[0, 0] *= scale_x  # fx
    scaled[1, 1] *= scale_y  # fy
    scaled[0, 2] *= scale_x  # cx
    scaled[1, 2] *= scale_y  # cy
    
    return scaled

class CalibrationState:
    def __init__(self):
        self.phase: str = "print"  # print, camera_selection, intrinsic, verification, extrinsic, complete
        self.calibration_mode: str = "print"  # "print" or "iphone"
        self.selected_camera: Optional[int] = None
        self.is_stereo: bool = False
        self.cameras: List[Dict] = []
        
        # Frame handling
        self.cap: Optional[cv2.VideoCapture] = None
        self.current_frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()
        
        # Calibration objects
        self.intrinsic_calibrator: Optional[IntrinsicCalibrator] = None
        self.intrinsic_result: Optional[Dict[str, CalibrationResult]] = None
        self.verification_engines = {"left": None, "right": None}
        self.verification_status = {"left": {}, "right": {}}
        
        # Extrinsic calibration - separate for left/right in stereo mode
        self.avp_ip: Optional[str] = None
        self.avp_connected: bool = False
        self.avp_streamer = None  # VisionProStreamer instance
        self.extrinsic_logs: List[str] = []
        
        # Separate calibrators, results, and debug for left/right
        self.extrinsic_calibrators = {"left": None, "right": None}
        self.extrinsic_results = {"left": None, "right": None}
        self.extrinsic_losses = {"left": None, "right": None}
        self.debug_projections = {"left": None, "right": None}
        
        self.extrinsic_samples = 0
        self.extrinsic_step = 0  # 0, 1, 2 for markers 0, 2, 3
        self.samples_per_step = {
            "left": {0: 0, 1: 0, 2: 0},
            "right": {0: 0, 1: 0, 2: 0}
        }
        self.TARGET_SAMPLES_PER_STEP = 30  # 30 samples per marker
        
        # Capture logic state - separate for left/right
        self.last_capture_time = {"left": 0, "right": 0}
        self.last_head_pos = {"left": None, "right": None}
        self.MIN_CAPTURE_INTERVAL = 0.3  # seconds
        self.MIN_HEAD_MOVEMENT = 0.02  # meters (2cm)

        
        # Trajectory data for 3D visualization
        self.raw_samples: List[Dict] = []
        self.raw_data_dir = Path('calibration_raw_data')
        self.raw_data_dir.mkdir(parents=True, exist_ok=True)

        
        # Streaming thread
        self.streaming: bool = False
        self.stream_thread: Optional[threading.Thread] = None
        
        # Status
        self.detection_status: Dict = {}
    
    def add_log(self, msg: str):
        """Add a log message for extrinsic calibration."""
        import time as t
        timestamp = t.strftime("%H:%M:%S")
        self.extrinsic_logs.append(f"[{timestamp}] {msg}")
        if len(self.extrinsic_logs) > 100:
            self.extrinsic_logs = self.extrinsic_logs[-100:]
    
    def send_calibration_status(self, marker_detected: bool = False, step_status: int = 0):
        """Send calibration status to VisionOS via gRPC.
        
        Args:
            marker_detected: Whether the target marker is currently detected
            step_status: 0=collecting, 1=calibrating, 2=complete
        """
        if not self.avp_ip:
            return
        
        try:
            from avp_stream.grpc_msg import handtracking_pb2, handtracking_pb2_grpc
            import grpc
            
            # Determine target marker for current step
            marker_ids = [0, 2, 3]
            target_marker = marker_ids[self.extrinsic_step] if self.extrinsic_step < 3 else 0
            
            # Calculate samples collected for current step (combine left+right for stereo)
            samples = 0
            if self.is_stereo:
                left = self.samples_per_step.get("left", {}).get(self.extrinsic_step, 0)
                right = self.samples_per_step.get("right", {}).get(self.extrinsic_step, 0)
                samples = min(left, right)  # Use minimum since both need samples
            else:
                samples = self.samples_per_step.get("left", {}).get(self.extrinsic_step, 0)
            
            # Calculate progress percentage
            progress = (samples / self.TARGET_SAMPLES_PER_STEP) * 100 if self.TARGET_SAMPLES_PER_STEP > 0 else 0
            
            with grpc.insecure_channel(f"{self.avp_ip}:12345") as channel:
                try:
                    grpc.channel_ready_future(channel).result(timeout=1)
                    
                    # Create calibration status message with marker 778.0
                    msg = handtracking_pb2.HandUpdate()
                    msg.Head.m00 = 778.0  # Calibration mode signal
                    msg.Head.m01 = float(self.extrinsic_step)  # Current step (0, 1, 2)
                    msg.Head.m02 = float(samples)  # Samples collected
                    msg.Head.m03 = float(self.TARGET_SAMPLES_PER_STEP)  # Samples needed
                    msg.Head.m10 = float(target_marker)  # Target marker ID
                    msg.Head.m11 = 1.0 if marker_detected else 0.0  # Marker detected flag
                    msg.Head.m12 = float(progress)  # Progress percentage (0-100)
                    msg.Head.m13 = float(step_status)  # Step status
                    
                    stub = handtracking_pb2_grpc.HandTrackingServiceStub(channel)
                    responses = stub.StreamHandUpdates(msg)
                    next(responses)  # Get first response to ensure message was sent
                    
                except Exception as e:
                    # Don't spam logs on connection errors
                    pass
                    
        except Exception as e:
            # Silently ignore errors - calibration status is optional
            pass
    
    def save_checkpoint(self):
        """Save current calibration state to disk."""
        data = {
            "phase": self.phase,
            "selected_camera": self.selected_camera,
            "is_stereo": self.is_stereo,
            "calibration_mode": self.calibration_mode,
            "avp_ip": self.avp_ip,
        }
        
        # Save intrinsic results if available
        if self.intrinsic_result:
            data["intrinsic"] = {}
            for name, result in self.intrinsic_result.items():
                if result.camera_matrix is not None:
                    data["intrinsic"][name] = {
                        "camera_matrix": result.camera_matrix.tolist(),
                        "distortion": result.distortion.tolist(),
                        "image_size": result.image_size,
                        "reprojection_error": result.reprojection_error,
                    }
        
        with open(CHECKPOINT_FILE, 'w') as f:
            json.dump(data, f, indent=2)
        
        self.add_log(f"Checkpoint saved: {self.phase}")
    
    def load_checkpoint(self) -> bool:
        """Load calibration state from disk. Returns True if checkpoint exists."""
        if not CHECKPOINT_FILE.exists():
            return False
        
        try:
            with open(CHECKPOINT_FILE, 'r') as f:
                data = json.load(f)
            
            self.phase = data.get("phase", "print")
            self.selected_camera = data.get("selected_camera")
            self.is_stereo = data.get("is_stereo", False)
            self.calibration_mode = data.get("calibration_mode", "print")
            self.avp_ip = data.get("avp_ip")
            
            # Restore intrinsic results
            if "intrinsic" in data:
                self.intrinsic_result = {}
                for name, vals in data["intrinsic"].items():
                    self.intrinsic_result[name] = CalibrationResult(
                        camera_matrix=np.array(vals["camera_matrix"]),
                        distortion=np.array(vals["distortion"]),
                        image_size=tuple(vals["image_size"]) if vals["image_size"] else None,
                        reprojection_error=vals["reprojection_error"],
                    )
            
            return True
        except Exception as e:
            print(f"Error loading checkpoint: {e}")
            return False
    
    def clear_checkpoint(self):
        """Delete checkpoint file."""
        if CHECKPOINT_FILE.exists():
            CHECKPOINT_FILE.unlink()

state = CalibrationState()

# HTML Template - Minimalistic white design with vertical layout
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Camera Calibration</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #fafafa;
            color: #333;
            min-height: 100vh;
        }
        .container { max-width: 1000px; margin: 0 auto; padding: 16px; }
        
        h1 {
            text-align: center;
            font-size: 1.5em;
            font-weight: 600;
            margin-bottom: 4px;
            color: #111;
        }
        .subtitle {
            text-align: center;
            color: #888;
            font-size: 0.9em;
            margin-bottom: 16px;
        }
        
        .phase-indicator {
            display: flex;
            justify-content: center;
            gap: 6px;
            margin-bottom: 20px;
        }
        .phase-step {
            padding: 6px 14px;
            background: #eee;
            border-radius: 4px;
            font-size: 0.8em;
            color: #666;
        }
        .phase-step.active { background: #333; color: white; }
        .phase-step.completed { background: #4CAF50; color: white; }
        
        .video-container {
            background: #111;
            border-radius: 8px;
            overflow: hidden;
            margin-bottom: 16px;
        }
        .video-container img {
            width: 100%;
            display: block;
        }
        
        .camera-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 12px;
            margin-bottom: 20px;
        }
        .camera-card {
            background: white;
            border-radius: 8px;
            overflow: hidden;
            cursor: pointer;
            border: 2px solid #eee;
            transition: border-color 0.2s;
        }
        .camera-card:hover { border-color: #333; }
        .camera-card.selected { border-color: #4CAF50; }
        .camera-card img {
            width: 100%;
            display: block;
        }
        .camera-info {
            padding: 10px 12px;
            border-top: 1px solid #eee;
        }
        .camera-info h3 { font-size: 0.9em; font-weight: 600; }
        .camera-info p { color: #666; font-size: 0.8em; margin-top: 2px; }
        
        .info-panel {
            background: white;
            border-radius: 8px;
            padding: 16px;
            border: 1px solid #eee;
            margin-bottom: 16px;
        }
        .info-panel h2 {
            font-size: 1em;
            font-weight: 600;
            margin-bottom: 12px;
            padding-bottom: 8px;
            border-bottom: 1px solid #eee;
        }
        
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 16px;
        }
        .stat-group h3 {
            font-size: 0.75em;
            color: #888;
            text-transform: uppercase;
            letter-spacing: 0.5px;
            margin-bottom: 8px;
        }
        .stat-row {
            display: flex;
            justify-content: space-between;
            padding: 4px 0;
            font-size: 0.85em;
        }
        .stat-value { font-weight: 600; font-family: monospace; }
        .stat-value.good { color: #4CAF50; }
        .stat-value.warning { color: #FF9800; }
        .stat-value.bad { color: #f44336; }
        
        .progress-section {
            margin-top: 12px;
            padding-top: 12px;
            border-top: 1px solid #eee;
        }
        .progress-label {
            display: flex;
            justify-content: space-between;
            font-size: 0.85em;
            margin-bottom: 6px;
        }
        .progress-bar {
            height: 8px;
            background: #e0e0e0;
            border-radius: 4px;
            overflow: hidden;
        }
        .progress-fill {
            height: 100%;
            background: #4CAF50;
            transition: width 0.3s;
        }
        
        .btn-row {
            display: flex;
            gap: 8px;
        }
        .btn {
            flex: 1;
            padding: 12px;
            border: none;
            border-radius: 6px;
            font-size: 0.9em;
            font-weight: 500;
            cursor: pointer;
        }
        .btn-primary { background: #333; color: white; }
        .btn-primary:hover { opacity: 0.85; }
        .btn-secondary { background: #eee; color: #333; }
        .btn-secondary:hover { background: #ddd; }
        .btn:disabled { opacity: 0.4; cursor: not-allowed; }
        
        .toggle-group {
            display: flex;
            gap: 8px;
            margin-bottom: 12px;
        }
        .toggle-btn {
            flex: 1;
            padding: 10px;
            border: 1px solid #ddd;
            background: white;
            color: #333;
            border-radius: 6px;
            cursor: pointer;
            font-size: 0.85em;
        }
        .toggle-btn.active {
            border-color: #333;
            background: #333;
            color: white;
        }
        
        .instructions {
            background: #f8f9fa;
            padding: 12px;
            border-radius: 6px;
            font-size: 0.85em;
            color: #555;
            margin-bottom: 12px;
        }
        .instructions strong { color: #333; }
        
        .center-box {
            max-width: 400px;
            margin: 20px auto;
            background: white;
            padding: 20px;
            border-radius: 8px;
            border: 1px solid #eee;
        }
        
        /* Extrinsic Overlay */
        .video-overlay {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.75);
            display: flex;
            align-items: center;
            justify-content: center;
            border-radius: 8px;
        }
        .video-overlay-content {
            text-align: center;
            color: white;
            padding: 24px;
        }
        .video-overlay input {
            padding: 10px 14px;
            border: 1px solid #555;
            border-radius: 6px;
            background: #333;
            color: white;
            font-size: 1em;
            width: 200px;
            margin: 8px 0;
        }
        .video-overlay input::placeholder { color: #888; }
        
        .log-area {
            background: #1e1e1e;
            color: #ccc;
            font-family: monospace;
            font-size: 0.75em;
            padding: 10px;
            border-radius: 6px;
            height: 120px;
            overflow-y: auto;
            margin-top: 12px;
            text-align: left;
        }
        .log-area .log-line { padding: 2px 0; }
        .log-area .log-line.success { color: #4CAF50; }
        .log-area .log-line.error { color: #f44336; }
        
        /* Extrinsic Progress Panel */
        .extrinsic-progress-panel {
            width: 100%;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            border-radius: 12px;
            margin-top: 12px;
            padding: 20px;
            box-sizing: border-box;
        }
        .extrinsic-stage-info {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 16px;
        }
        .stage-label {
            font-size: 1.2em;
            font-weight: 600;
            color: #4CAF50;
        }
        .stage-marker {
            font-size: 1.1em;
            color: #90CAF9;
            font-family: monospace;
        }
        .extrinsic-samples-info {
            background: rgba(255,255,255,0.05);
            border-radius: 8px;
            padding: 16px;
            margin-bottom: 16px;
        }
        .samples-count {
            font-size: 1.4em;
            color: #fff;
            margin-bottom: 10px;
            text-align: center;
        }
        .samples-count span:first-child {
            font-weight: 700;
            color: #4CAF50;
        }
        .samples-progress-bar {
            height: 20px;
            background: rgba(255,255,255,0.1);
            border-radius: 10px;
            overflow: hidden;
        }
        .samples-progress-fill {
            height: 100%;
            background: linear-gradient(90deg, #4CAF50, #8BC34A);
            border-radius: 10px;
            transition: width 0.3s ease;
        }
        .extrinsic-overall-progress {
            text-align: center;
        }
        .overall-label {
            color: #888;
            font-size: 0.9em;
            margin-bottom: 10px;
        }
        .overall-stages {
            display: flex;
            justify-content: center;
            align-items: center;
            gap: 8px;
        }
        .stage-dot {
            width: 40px;
            height: 40px;
            border-radius: 50%;
            background: rgba(255,255,255,0.1);
            color: #666;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 0.85em;
            font-weight: 600;
            transition: all 0.3s;
        }
        .stage-dot.active {
            background: #4CAF50;
            color: #fff;
            box-shadow: 0 0 12px rgba(76, 175, 80, 0.5);
        }
        .stage-dot.complete {
            background: #2E7D32;
            color: #fff;
        }
        .stage-connector {
            width: 30px;
            height: 3px;
            background: rgba(255,255,255,0.1);
        }
    </style>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
</head>
<body>
    <div class="container">
        <h1>Camera Calibration</h1>
        <p class="subtitle">Calibrate your camera for accurate tracking</p>
        
        <div class="phase-indicator">
            <div class="phase-step" id="phase-0">0. Print</div>
            <div class="phase-step" id="phase-1">1. Camera</div>
            <div class="phase-step" id="phase-2">2. Intrinsic</div>
            <div class="phase-step" id="phase-3">3. Verify</div>
            <div class="phase-step" id="phase-4">4. Extrinsic</div>
        </div>
        
        <!-- Print Calibration Sheets Phase -->
        <div id="print-phase">
            <div style="max-width: 800px; margin: 0 auto;">
                <!-- Mode Selector -->
                <div style="background: white; border: 1px solid #eee; border-radius: 8px; padding: 16px; margin-bottom: 16px;">
                    <h2 style="font-size: 1.1em; margin-bottom: 8px;">Calibration Mode</h2>
                    <p style="color: #666; font-size: 0.85em; margin-bottom: 12px;">
                        Choose how you'll display calibration patterns:
                    </p>
                    <div style="display: flex; gap: 12px;">
                        <button class="mode-btn active" id="mode-print-btn" onclick="setCalibrationMode('print')" 
                                style="flex: 1; padding: 16px; border: 2px solid #4CAF50; border-radius: 8px; background: #e8f5e9; cursor: pointer;">
                            <div style="font-size: 2em;">📄</div>
                            <div style="font-weight: bold; margin: 8px 0;">Print Mode</div>
                            <div style="font-size: 0.8em; color: #666;">ChArUco 7×5, 30mm</div>
                        </button>
                        <button class="mode-btn" id="mode-iphone-btn" onclick="setCalibrationMode('iphone')"
                                style="flex: 1; padding: 16px; border: 2px solid #ddd; border-radius: 8px; background: white; cursor: pointer;">
                            <div style="font-size: 2em;">📱</div>
                            <div style="font-weight: bold; margin: 8px 0;">iPhone Mode</div>
                            <div style="font-size: 0.8em; color: #666;">Checkerboard 6×12, 10mm</div>
                        </button>
                    </div>
                </div>
                
                <!-- Print instructions (shown in print mode) -->
                <div id="print-instructions" style="background: white; border: 1px solid #eee; border-radius: 8px; padding: 16px; margin-bottom: 16px;">
                    <h2 style="font-size: 1.1em; margin-bottom: 8px;">Print Calibration Sheets</h2>
                    <p style="color: #666; font-size: 0.85em; margin-bottom: 12px;">
                        Print the ChArUco board (intrinsic) and ArUco markers (extrinsic) before starting calibration.
                    </p>
                    <div class="btn-row">
                        <button class="btn btn-primary" onclick="printPDF()">Print PDF</button>
                        <button class="btn btn-secondary" onclick="skipToCameraSelection()">Continue →</button>
                    </div>
                </div>
                
                <!-- iPhone instructions (shown in iPhone mode) -->
                <div id="iphone-instructions" style="display: none; background: white; border: 1px solid #eee; border-radius: 8px; padding: 16px; margin-bottom: 16px;">
                    <h2 style="font-size: 1.1em; margin-bottom: 8px;">Prepare iPhone Patterns</h2>
                    <p style="color: #666; font-size: 0.85em; margin-bottom: 12px;">
                        Open the calibration pattern app on your iPhone. You'll need:
                    </p>
                    <ul style="color: #666; font-size: 0.85em; margin-bottom: 12px; padding-left: 20px;">
                        <li><strong>Intrinsic:</strong> 6×12 checkerboard (10mm squares)</li>
                        <li><strong>Verification:</strong> 1×3 ArUco (IDs 0,1,2 @ 40mm, 6mm gap)</li>
                        <li><strong>Extrinsic:</strong> Single ArUco markers (IDs 0,2,3 @ 55mm)</li>
                    </ul>
                    <div class="btn-row">
                        <button class="btn btn-secondary" onclick="skipToCameraSelection()">Continue →</button>
                    </div>
                </div>
                
                <iframe id="pdf-frame" src="/calibration_sheets.pdf" style="width: 100%; height: 500px; border: 1px solid #ddd; border-radius: 8px;"></iframe>
            </div>
        </div>

        
        <!-- Camera Selection Phase -->
        <div id="camera-selection-phase" style="display: none;">
            <div class="camera-grid" id="camera-grid">
                <p style="text-align: center; padding: 40px; color: #666;">Loading cameras...</p>
            </div>
            
            <div class="center-box">
                <div class="toggle-group">
                    <button class="toggle-btn active" id="mono-btn" onclick="setStereo(false)">Mono</button>
                    <button class="toggle-btn" id="stereo-btn" onclick="setStereo(true)">Stereo</button>
                </div>
                <button class="btn btn-primary" id="start-btn" onclick="startCalibration()" disabled style="width:100%">
                    Start Calibration
                </button>
            </div>
        </div>
        
        <!-- Calibration Phase -->
        <div id="calibration-phase" style="display: none;">
            <div class="video-container" style="position: relative;">
                <img id="video-feed" src="/video_feed" alt="Camera Feed">
                
                <!-- Extrinsic Overlay -->
                <div class="video-overlay" id="extrinsic-overlay" style="display: none;">
                    <div class="video-overlay-content">
                        <h2 style="margin-bottom: 16px;">Connect to Vision Pro</h2>
                        <p style="font-size: 0.9em; margin-bottom: 16px; opacity: 0.8;">
                            Wear your Vision Pro, launch <strong>Tracking Streamer</strong>,<br>
                            then enter the IP address shown on your device.
                        </p>
                        <input type="text" id="avp-ip-input" placeholder="e.g. 192.168.1.100">
                        <br>
                        <button class="btn btn-primary" onclick="connectAVP()" id="connect-avp-btn" style="margin-top: 8px;">
                            Connect to Vision Pro
                        </button>
                    </div>
                </div>
            </div>
            
            <!-- Log Area (for extrinsic phase) -->
            <div class="log-area" id="extrinsic-logs" style="display: none;">
                <div class="log-line">Waiting to connect to Vision Pro...</div>
            </div>
            
            <!-- Extrinsic Progress Panel (replaces 3D viewer) -->
            <div class="extrinsic-progress-panel" id="extrinsic-progress-panel" style="display: none;">
                <div class="extrinsic-stage-info" id="extrinsic-stage-info">
                    <span class="stage-label">Stage 1/3</span>
                    <span class="stage-marker">Marker ID: 0</span>
                </div>
                
                <div class="extrinsic-samples-info" id="extrinsic-samples-mono">
                    <div class="samples-count">
                        <span id="extrinsic-sample-count">0</span> / <span id="extrinsic-sample-target">30</span> samples
                    </div>
                    <div class="samples-progress-bar">
                        <div class="samples-progress-fill" id="extrinsic-progress-fill" style="width: 0%;"></div>
                    </div>
                </div>
                
                <!-- Stereo mode: dual progress bars -->
                <div class="extrinsic-samples-info" id="extrinsic-samples-stereo" style="display: none;">
                    <div style="display: flex; gap: 20px;">
                        <div style="flex: 1;">
                            <div class="samples-count" style="font-size: 1em;">Left: <span id="extrinsic-left-count">0</span>/30</div>
                            <div class="samples-progress-bar">
                                <div class="samples-progress-fill" id="extrinsic-left-fill" style="width: 0%;"></div>
                            </div>
                        </div>
                        <div style="flex: 1;">
                            <div class="samples-count" style="font-size: 1em;">Right: <span id="extrinsic-right-count">0</span>/30</div>
                            <div class="samples-progress-bar">
                                <div class="samples-progress-fill" id="extrinsic-right-fill" style="width: 0%;"></div>
                            </div>
                        </div>
                    </div>
                </div>
                
                <div class="extrinsic-overall-progress">
                    <div class="overall-label">Overall Progress</div>
                    <div class="overall-stages">
                        <div class="stage-dot" id="stage-dot-0">M0</div>
                        <div class="stage-connector"></div>
                        <div class="stage-dot" id="stage-dot-1">M2</div>
                        <div class="stage-connector"></div>
                        <div class="stage-dot" id="stage-dot-2">M3</div>
                    </div>
                </div>
            </div>
            
            <div class="info-panel">
                <h2 id="phase-title">Intrinsic Calibration</h2>
                
                <div class="instructions" id="instructions">
                    <strong>Instructions:</strong> Hold ChArUco board in front of camera. Move slowly to capture different angles. Samples are captured automatically.
                </div>
                
                <div class="stats-grid" id="stats-grid">
                    <div class="stat-group" id="left-stats">
                        <h3>Left Camera</h3>
                        <div class="stat-row intrinsic-stat"><span>Samples</span><span class="stat-value" id="left-samples">0</span></div>
                        <div class="stat-row intrinsic-stat"><span>fx</span><span class="stat-value" id="left-fx">--</span></div>
                        <div class="stat-row intrinsic-stat"><span>fy</span><span class="stat-value" id="left-fy">--</span></div>
                        <div class="stat-row intrinsic-stat"><span>cx</span><span class="stat-value" id="left-cx">--</span></div>
                        <div class="stat-row intrinsic-stat"><span>cy</span><span class="stat-value" id="left-cy">--</span></div>
                        <div class="stat-row intrinsic-stat"><span>Error</span><span class="stat-value" id="left-error">--</span></div>
                        
                        <div class="stat-row verif-stat" style="display:none"><span>Detected</span><span class="stat-value" id="left-verif-detected">No</span></div>
                        <div class="stat-row verif-stat" style="display:none"><span>Mean Err</span><span class="stat-value" id="left-verif-mean">--</span></div>
                        <div class="stat-row verif-stat" style="display:none"><span>Max Err</span><span class="stat-value" id="left-verif-max">--</span></div>
                        <div class="verif-stat" style="display:none; font-size: 0.85em; line-height: 1.6; margin-top: 8px;" id="left-verif-details">Detecting...</div>
                    </div>
                    <div class="stat-group" id="right-stats" style="display: none;">
                        <h3>Right Camera</h3>
                        <div class="stat-row intrinsic-stat"><span>Samples</span><span class="stat-value" id="right-samples">0</span></div>
                        <div class="stat-row intrinsic-stat"><span>fx</span><span class="stat-value" id="right-fx">--</span></div>
                        <div class="stat-row intrinsic-stat"><span>fy</span><span class="stat-value" id="right-fy">--</span></div>
                        <div class="stat-row intrinsic-stat"><span>cx</span><span class="stat-value" id="right-cx">--</span></div>
                        <div class="stat-row intrinsic-stat"><span>cy</span><span class="stat-value" id="right-cy">--</span></div>
                        <div class="stat-row intrinsic-stat"><span>Error</span><span class="stat-value" id="right-error">--</span></div>
                        
                        <div class="stat-row verif-stat" style="display:none"><span>Detected</span><span class="stat-value" id="right-verif-detected">No</span></div>
                        <div class="stat-row verif-stat" style="display:none"><span>Mean Err</span><span class="stat-value" id="right-verif-mean">--</span></div>
                        <div class="stat-row verif-stat" style="display:none"><span>Max Err</span><span class="stat-value" id="right-verif-max">--</span></div>
                        <div class="verif-stat" style="display:none; font-size: 0.85em; line-height: 1.6; margin-top: 8px;" id="right-verif-details">Detecting...</div>
                    </div>
                </div>
                
                <div class="progress-section" id="stability-section">
                    <div id="left-stability">
                        <div class="progress-label">
                            <span>Left Stability</span>
                            <span id="left-stability-text">0%</span>
                        </div>
                        <div class="progress-bar">
                            <div class="progress-fill" id="left-progress-fill" style="width: 0%"></div>
                        </div>
                    </div>
                    <div id="right-stability" style="display: none; margin-top: 8px;">
                        <div class="progress-label">
                            <span>Right Stability</span>
                            <span id="right-stability-text">0%</span>
                        </div>
                        <div class="progress-bar">
                            <div class="progress-fill" id="right-progress-fill" style="width: 0%"></div>
                        </div>
                    </div>
                </div>
                
                <div class="btn-row" style="margin-top: 16px;">
                    <button class="btn btn-secondary" id="prev-btn" onclick="prevPhase()" style="display: none;">Recalibrate</button>
                    <button class="btn btn-primary" id="next-btn" onclick="nextPhase()" disabled>Continue</button>
                    <button class="btn btn-secondary" id="reset-btn" onclick="resetPhase()">Reset</button>
                </div>
            </div>
        </div>
        
        <!-- Complete Phase -->
        <div id="complete-phase" style="display: none;">
            <div class="center-box" style="text-align: center;">
                <h2 style="margin-bottom: 12px;">Calibration Complete</h2>
                <p style="color: #666; margin-bottom: 16px; font-size: 0.9em;">
                    Your camera has been calibrated successfully.
                </p>
                <button class="btn btn-primary" onclick="downloadResults()" style="width: 100%">
                    Download JSON
                </button>
            </div>
        </div>
    </div>
    
    <script>
        let selectedCamera = null;
        let isStereo = false;
        let currentPhase = 'print';
        let statusInterval = null;
        
        document.addEventListener('DOMContentLoaded', async () => {
            loadCameras();
            updatePhaseIndicator();
            
            // Check for existing checkpoint
            try {
                const res = await fetch('/api/has_checkpoint');
                const data = await res.json();
                if (data.exists) {
                    if (confirm('A previous calibration session was found. Resume from where you left off?')) {
                        const loadRes = await fetch('/api/load_checkpoint', {method: 'POST'});
                        const loadData = await loadRes.json();
                        if (loadData.success) {
                            currentPhase = loadData.phase;
                            
                            // Restore stereo mode
                            if (loadData.is_stereo !== undefined) {
                                isStereo = loadData.is_stereo;
                                document.getElementById('mono-btn').classList.toggle('active', !isStereo);
                                document.getElementById('stereo-btn').classList.toggle('active', isStereo);
                            }
                            
                            // Restore calibration mode
                            if (loadData.calibration_mode) {
                                calibrationMode = loadData.calibration_mode;
                                setCalibrationMode(calibrationMode);
                            }
                            
                            updatePhaseIndicator();
                            
                            // Update stereo UI elements
                            if (isStereo) {
                                document.getElementById('right-stats').style.display = 'block';
                            }
                            
                            // Show appropriate phase
                            document.querySelectorAll('[id$="-phase"]').forEach(el => el.style.display = 'none');
                            if (currentPhase === 'print') {
                                document.getElementById('print-phase').style.display = 'block';
                            } else if (currentPhase === 'camera_selection') {
                                document.getElementById('camera-selection-phase').style.display = 'block';
                            } else if (currentPhase === 'complete') {
                                document.getElementById('complete-phase').style.display = 'block';
                            } else {
                                document.getElementById('calibration-phase').style.display = 'block';
                                // Immediate status update to reflect current state
                                updateStatus();
                                statusInterval = setInterval(updateStatus, 300);
                            }
                            return;
                        }
                    } else {
                        // User chose to start fresh
                        await fetch('/api/clear_checkpoint', {method: 'POST'});
                    }
                }
            } catch (e) {
                console.error('Checkpoint check failed:', e);
            }
        });
        
        let calibrationMode = 'print';  // 'print' or 'iphone'
        
        async function setCalibrationMode(mode) {
            calibrationMode = mode;
            
            // Update button styles
            document.getElementById('mode-print-btn').style.border = mode === 'print' ? '2px solid #4CAF50' : '2px solid #ddd';
            document.getElementById('mode-print-btn').style.background = mode === 'print' ? '#e8f5e9' : 'white';
            document.getElementById('mode-iphone-btn').style.border = mode === 'iphone' ? '2px solid #4CAF50' : '2px solid #ddd';
            document.getElementById('mode-iphone-btn').style.background = mode === 'iphone' ? '#e8f5e9' : 'white';
            
            // Show/hide instructions
            document.getElementById('print-instructions').style.display = mode === 'print' ? 'block' : 'none';
            document.getElementById('iphone-instructions').style.display = mode === 'iphone' ? 'block' : 'none';
            document.getElementById('pdf-frame').style.display = mode === 'print' ? 'block' : 'none';
            
            // Send to server
            await fetch('/api/set_mode', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({mode: mode})
            });
        }
        
        function skipToCameraSelection() {
            document.getElementById('print-phase').style.display = 'none';
            document.getElementById('camera-selection-phase').style.display = 'block';
            currentPhase = 'camera_selection';
            updatePhaseIndicator();
        }
        
        function printPDF() {
            const iframe = document.getElementById('pdf-frame');
            iframe.contentWindow.print();
        }
        
        async function loadCameras() {
            const response = await fetch('/api/cameras');
            const cameras = await response.json();
            
            const grid = document.getElementById('camera-grid');
            if (cameras.length === 0) {
                grid.innerHTML = '<p style="text-align: center; padding: 40px; color: #666;">No cameras found.</p>';
                return;
            }
            
            grid.innerHTML = cameras.map((cam) => `
                <div class="camera-card" id="cam-${cam.index}" onclick="selectCamera(${cam.index})">
                    <img src="/camera_stream/${cam.index}" alt="Camera ${cam.index}">
                    <div class="camera-info">
                        <h3>Camera ${cam.index}</h3>
                        <p>${cam.width}x${cam.height}${cam.width/cam.height > 1.8 ? ' • Stereo' : ''}</p>
                    </div>
                </div>
            `).join('');
        }
        
        function selectCamera(index) {
            document.querySelectorAll('.camera-card').forEach(c => c.classList.remove('selected'));
            document.getElementById(`cam-${index}`).classList.add('selected');
            selectedCamera = index;
            document.getElementById('start-btn').disabled = false;
            
            fetch('/api/cameras').then(r => r.json()).then(cameras => {
                const cam = cameras.find(c => c.index === index);
                if (cam && cam.width / cam.height > 1.8) setStereo(true);
            });
        }
        
        function setStereo(stereo) {
            isStereo = stereo;
            document.getElementById('mono-btn').classList.toggle('active', !stereo);
            document.getElementById('stereo-btn').classList.toggle('active', stereo);
        }
        
        async function startCalibration() {
            if (selectedCamera === null) return;
            
            await fetch('/api/select_camera', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({camera: selectedCamera, stereo: isStereo})
            });
            
            // Show/hide stereo elements
            document.getElementById('right-stats').style.display = isStereo ? 'block' : 'none';
            document.getElementById('right-stability').style.display = isStereo ? 'block' : 'none';
            
            document.getElementById('camera-selection-phase').style.display = 'none';
            document.getElementById('calibration-phase').style.display = 'block';
            currentPhase = 'intrinsic';
            updatePhaseIndicator();
            
            statusInterval = setInterval(updateStatus, 300);
        }
        
        async function updateStatus() {
            const response = await fetch('/api/status');
            const s = await response.json();
            
            currentPhase = s.phase;
            updatePhaseIndicator();
            
            if (s.phase === 'complete') {
                clearInterval(statusInterval);
                document.getElementById('calibration-phase').style.display = 'none';
                document.getElementById('complete-phase').style.display = 'block';
                return;
            }
            
            // Update left camera stats
            if (s.left) {
                document.getElementById('left-samples').textContent = s.left.samples || 0;
                updateStat('left-fx', s.left.fx);
                updateStat('left-fy', s.left.fy);
                updateStat('left-cx', s.left.cx);
                updateStat('left-cy', s.left.cy);
                updateError('left-error', s.left.error);
                
                const leftStab = s.left.stability || 0;
                document.getElementById('left-stability-text').textContent = leftStab.toFixed(0) + '%';
                document.getElementById('left-progress-fill').style.width = leftStab + '%';
            }
            
            // Update right camera stats (if stereo)
            if (s.right && isStereo) {
                document.getElementById('right-samples').textContent = s.right.samples || 0;
                updateStat('right-fx', s.right.fx);
                updateStat('right-fy', s.right.fy);
                updateStat('right-cx', s.right.cx);
                updateStat('right-cy', s.right.cy);
                updateError('right-error', s.right.error);
                
                const rightStab = s.right.stability || 0;
                document.getElementById('right-stability-text').textContent = rightStab.toFixed(0) + '%';
                document.getElementById('right-progress-fill').style.width = rightStab + '%';
            }
            
            // Verification stats
            if (currentPhase === 'verification') {
                document.querySelectorAll('.intrinsic-stat').forEach(el => el.style.display = 'none');
                document.querySelectorAll('.verif-stat').forEach(el => el.style.display = 'flex');
                
                // Build detailed measurements HTML
                function formatMeasurements(camStats) {
                    if (!camStats || !camStats.measurements) return 'Detecting...';
                    return camStats.measurements.map(m => {
                        const color = m.error < 2 ? '#4CAF50' : m.error < 5 ? '#FF9800' : '#f44336';
                        return `<span style="color:${color}">${m.pair}: ${m.measured}mm (GT:${m.expected}mm)</span>`;
                    }).join('<br>');
                }
                
                if (s.left) {
                    document.getElementById('left-verif-detected').textContent = s.left.detected ? "Yes" : "No";
                    document.getElementById('left-verif-mean').textContent = s.left.mean_error?.toFixed(1) || '-';
                    document.getElementById('left-verif-max').textContent = s.left.max_error?.toFixed(1) || '-';
                    document.getElementById('left-verif-details').innerHTML = formatMeasurements(s.left);
                }
                
                if (s.right && isStereo) {
                    document.getElementById('right-verif-detected').textContent = s.right.detected ? "Yes" : "No";
                    document.getElementById('right-verif-mean').textContent = s.right.mean_error?.toFixed(1) || '-';
                    document.getElementById('right-verif-max').textContent = s.right.max_error?.toFixed(1) || '-';
                    document.getElementById('right-verif-details').innerHTML = formatMeasurements(s.right);
                }
            } else {
                document.querySelectorAll('.intrinsic-stat').forEach(el => el.style.display = 'flex');
                document.querySelectorAll('.verif-stat').forEach(el => el.style.display = 'none');
            }
            
            // Enable continue when both cameras are ready
            if (currentPhase === 'verification') {
                 // In verification, user decides when to proceed
                 document.getElementById('next-btn').disabled = false;
                 document.getElementById('stability-section').style.display = 'none';
                 document.getElementById('prev-btn').style.display = 'inline-block';
                 document.getElementById('reset-btn').style.display = 'none';
            } else {
                const leftReady = (s.left?.stability || 0) >= 50;
                const rightReady = !isStereo || (s.right?.stability || 0) >= 50;
                document.getElementById('next-btn').disabled = !(leftReady && rightReady);
                document.getElementById('stability-section').style.display = 'block';
                document.getElementById('prev-btn').style.display = 'none';
                document.getElementById('reset-btn').style.display = 'inline-block';
            }
            
            updatePhaseUI(s);
        }
        
        function updateStat(id, val) {
            const el = document.getElementById(id);
            el.textContent = val != null ? val.toFixed(1) : '--';
        }
        
        function updateError(id, val) {
            const el = document.getElementById(id);
            if (val != null && val < 999) {
                el.textContent = val.toFixed(3);
                el.className = 'stat-value ' + (val < 0.5 ? 'good' : val < 1.0 ? 'warning' : 'bad');
            } else {
                el.textContent = '--';
                el.className = 'stat-value';
            }
        }
        
        function updatePhaseUI(status) {
            const title = document.getElementById('phase-title');
            const instr = document.getElementById('instructions');
            const overlay = document.getElementById('extrinsic-overlay');
            const logs = document.getElementById('extrinsic-logs');
            const statsGrid = document.getElementById('stats-grid');
            
            if (status.phase === 'intrinsic') {
                title.textContent = 'Intrinsic Calibration';
                instr.innerHTML = '<strong>Instructions:</strong> Hold ChArUco board in front of camera. Move slowly to capture different angles and distances.';
                overlay.style.display = 'none';
                logs.style.display = 'none';
                statsGrid.style.display = 'grid';
            } else if (status.phase === 'verification') {
                title.textContent = 'Verification';
                instr.innerHTML = '<strong>Instructions:</strong> Hold 2x2 ArUco verification grid. Green distances = good calibration.';
                overlay.style.display = 'none';
                logs.style.display = 'none';
                statsGrid.style.display = 'grid';
            } else if (status.phase === 'extrinsic') {
                title.textContent = 'Extrinsic Calibration';
                instr.innerHTML = '<strong>Instructions:</strong> Connect to Vision Pro, then look at the marker while moving your head.';
                logs.style.display = 'block';
                statsGrid.style.display = 'none';
                
                // Show overlay if not connected
                if (!status.avp_connected) {
                    overlay.style.display = 'flex';
                } else {
                    overlay.style.display = 'none';
                }
                
                // Update extrinsic progress panel
                if (status.extrinsic_step !== undefined) {
                    const step = status.extrinsic_step;
                    const samplesMap = status.extrinsic_samples || {};
                    const count = samplesMap[step] || 0;
                    const targetId = [0, 2, 3][step];
                    const targetTotal = 30;
                    
                    // Update stage info
                    document.querySelector('.stage-label').textContent = step < 3 ? `Stage ${step + 1}/3` : 'Complete!';
                    document.querySelector('.stage-marker').textContent = step < 3 ? `Marker ID: ${targetId}` : 'All Markers Done';
                    
                    // Switch between mono/stereo progress bars
                    if (isStereo) {
                        document.getElementById('extrinsic-samples-mono').style.display = 'none';
                        document.getElementById('extrinsic-samples-stereo').style.display = 'block';
                        const leftCount = status.extrinsic_samples?.left?.[step] || 0;
                        const rightCount = status.extrinsic_samples?.right?.[step] || 0;
                        document.getElementById('extrinsic-left-count').textContent = leftCount;
                        document.getElementById('extrinsic-right-count').textContent = rightCount;
                        document.getElementById('extrinsic-left-fill').style.width = Math.min(100, (leftCount / targetTotal) * 100) + '%';
                        document.getElementById('extrinsic-right-fill').style.width = Math.min(100, (rightCount / targetTotal) * 100) + '%';
                    } else {
                        document.getElementById('extrinsic-samples-mono').style.display = 'block';
                        document.getElementById('extrinsic-samples-stereo').style.display = 'none';
                        document.getElementById('extrinsic-sample-count').textContent = count;
                        document.getElementById('extrinsic-progress-fill').style.width = Math.min(100, (count / targetTotal) * 100) + '%';
                    }
                    
                    // Update stage dots
                    for (let i = 0; i < 3; i++) {
                        const dot = document.getElementById(`stage-dot-${i}`);
                        dot.classList.remove('active', 'complete');
                        if (i < step) dot.classList.add('complete');
                        else if (i === step && step < 3) dot.classList.add('active');
                        else if (step >= 3) dot.classList.add('complete');
                    }
                    
                    // Show extrinsic results and debug info (now supports stereo)
                    const debugProjs = status.debug_projections || {};
                    const extResults = status.extrinsic_results || {};
                    if (Object.keys(debugProjs).length > 0 || Object.keys(extResults).length > 0) {
                        let html = '';
                        
                        // Debug Projections (Real-time T_b_h @ T_c_m)
                        const camNames = isStereo ? ['left', 'right'] : ['left'];
                        for (const cam of camNames) {
                            if (debugProjs[cam]) {
                                const dp = debugProjs[cam];
                                const label = isStereo ? cam.toUpperCase() : '';
                                html += `<div style="margin-top: 16px; padding: 12px; background: rgba(33,150,243,0.1); border-radius: 8px;">`;
                                html += `<div style="font-weight: 600; color: #2196F3; margin-bottom: 8px;">${label} Debug: Marker in Body Frame</div>`;
                                html += `<div style="font-family: monospace; color: #eee; font-size: 0.9em;">`;
                                html += `X: ${dp[0].toFixed(4).padStart(8)}<br>`;
                                html += `Y: ${dp[1].toFixed(4).padStart(8)}<br>`;
                                html += `Z: ${dp[2].toFixed(4).padStart(8)}`;
                                html += `</div></div>`;
                            }
                        }

                        // Extrinsic Optimization Results
                        for (const cam of camNames) {
                            if (extResults[cam]) {
                                const res = extResults[cam];
                                const label = isStereo ? cam.toUpperCase() + ' ' : '';
                                html += `<div style="margin-top: 12px; padding: 12px; background: rgba(76,175,80,0.1); border-radius: 8px;">`;
                                html += `<div style="font-weight: 600; color: #4CAF50; margin-bottom: 8px;">${label}Extrinsic Result (T_head_camera)</div>`;
                                html += `<div style="font-size: 0.8em; color: #888;">Loss: ${res.loss?.toFixed(4) || '--'} mm</div>`;
                                if (res.matrix) {
                                    html += `<pre style="font-size: 0.7em; color: #aaa; margin-top: 8px; overflow-x: auto;">`;
                                    for (let row = 0; row < 4; row++) {
                                        html += '[';
                                        for (let col = 0; col < 4; col++) {
                                            html += res.matrix[row * 4 + col].toFixed(4).padStart(9);
                                            if (col < 3) html += ', ';
                                        }
                                        html += ']\\n';
                                    }
                                    html += '</pre>';
                                }
                                html += '</div>';
                            }
                        }
                        
                        document.getElementById('extrinsic-stage-info').insertAdjacentHTML('afterend', 
                            document.getElementById('extrinsic-result-display') ? '' : '<div id="extrinsic-result-display"></div>');
                        const resultDiv = document.getElementById('extrinsic-result-display');
                        if (resultDiv) resultDiv.innerHTML = html;
                    }
                }
                
                // Update logs
                updateExtrinsicLogs();
                
                // Show progress panel for extrinsic phase
                document.getElementById('extrinsic-progress-panel').style.display = 'block';
            } else if (status.phase === 'results') {
                // Results phase - show final summary
                document.getElementById('extrinsic-progress-panel').style.display = 'none';
                showResultsPage(status);
            } else {
                document.getElementById('extrinsic-progress-panel').style.display = 'none';
            }
        }
        
        function showResultsPage(status) {
            // Create or update results overlay
            let resultsDiv = document.getElementById('results-overlay');
            if (!resultsDiv) {
                resultsDiv = document.createElement('div');
                resultsDiv.id = 'results-overlay';
                resultsDiv.style.cssText = 'position: absolute; top: 0; left: 0; right: 0; bottom: 0; background: rgba(0,0,0,0.95); z-index: 100; overflow-y: auto; padding: 40px;';
                document.querySelector('.video-container').appendChild(resultsDiv);
            }
            
            let html = '<div style="max-width: 900px; margin: 0 auto;">';
            html += '<h2 style="color: #4CAF50; margin-bottom: 30px;">✓ Calibration Complete</h2>';
            
            const camNames = status.is_stereo ? ['left', 'right'] : ['left'];
            
            // Intrinsic Results
            html += '<h3 style="color: #2196F3; margin-top: 30px;">Intrinsic Calibration</h3>';
            for (const cam of camNames) {
                const intr = status.intrinsic_results?.[cam];
                if (intr) {
                    const label = status.is_stereo ? cam.toUpperCase() : '';
                    html += `<div style="background: rgba(33,150,243,0.1); padding: 16px; border-radius: 8px; margin: 10px 0;">`;
                    html += `<div style="font-weight: 600; color: #2196F3;">${label} Camera</div>`;
                    html += `<div style="font-family: monospace; color: #ddd; margin-top: 8px;">`;
                    html += `fx: ${intr.fx?.toFixed(2)} | fy: ${intr.fy?.toFixed(2)}<br>`;
                    html += `cx: ${intr.cx?.toFixed(2)} | cy: ${intr.cy?.toFixed(2)}<br>`;
                    html += `Error: ${intr.error?.toFixed(4)} px`;
                    html += `</div></div>`;
                }
            }
            
            // Extrinsic Results
            html += '<h3 style="color: #4CAF50; margin-top: 30px;">Extrinsic Calibration (T_head_camera)</h3>';
            for (const cam of camNames) {
                const ext = status.extrinsic_results?.[cam];
                if (ext) {
                    const label = status.is_stereo ? cam.toUpperCase() : '';
                    html += `<div style="background: rgba(76,175,80,0.1); padding: 16px; border-radius: 8px; margin: 10px 0;">`;
                    html += `<div style="font-weight: 600; color: #4CAF50;">${label} Camera</div>`;
                    html += `<div style="font-size: 0.8em; color: #888;">Loss: ${ext.loss?.toFixed(4) || '--'} mm</div>`;
                    if (ext.matrix) {
                        html += `<pre style="font-size: 0.65em; color: #aaa; margin-top: 8px;">`;
                        for (let row = 0; row < 4; row++) {
                            html += '[';
                            for (let col = 0; col < 4; col++) {
                                html += ext.matrix[row * 4 + col].toFixed(4).padStart(9);
                                if (col < 3) html += ', ';
                            }
                            html += ']\\n';
                        }
                        html += '</pre>';
                    }
                    html += '</div>';
                }
            }
            
            html += '<div style="margin-top: 40px; text-align: center;">';
            html += '<button onclick="saveCalibration()" style="padding: 15px 40px; font-size: 1.1em; background: #4CAF50; color: white; border: none; border-radius: 8px; cursor: pointer; margin-right: 20px;">Save Calibration</button>';
            html += '<button onclick="restartCalibration()" style="padding: 15px 40px; font-size: 1.1em; background: #666; color: white; border: none; border-radius: 8px; cursor: pointer;">Restart</button>';
            html += '</div>';
            
            html += '</div>';
            resultsDiv.innerHTML = html;
        }
        
        async function saveCalibration() {
            // TODO: Implement save to file/VisionPro
            alert('Calibration saved! (To be implemented: save to file or send to VisionPro)');
        }
        
        async function restartCalibration() {
            if (confirm('Restart calibration from the beginning?')) {
                await fetch('/api/restart', {method: 'POST'});
                location.reload();
            }
        }
        
        async function connectAVP() {
            const ip = document.getElementById('avp-ip-input').value.trim();
            if (!ip) {
                alert('Please enter an IP address');
                return;
            }
            
            const btn = document.getElementById('connect-avp-btn');
            btn.disabled = true;
            btn.textContent = 'Connecting...';
            
            try {
                const response = await fetch('/api/connect_avp', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({ip: ip})
                });
                const data = await response.json();
                if (!data.success) {
                    btn.textContent = 'Connect to Vision Pro';
                    btn.disabled = false;
                }
            } catch (e) {
                btn.textContent = 'Connect to Vision Pro';
                btn.disabled = false;
            }
        }
        
        async function updateExtrinsicLogs() {
            try {
                const response = await fetch('/api/extrinsic_logs');
                const data = await response.json();
                const logsDiv = document.getElementById('extrinsic-logs');
                logsDiv.innerHTML = data.logs.map(line => {
                    let cls = '';
                    if (line.includes('SUCCESS') || line.includes('Connected')) cls = 'success';
                    if (line.includes('ERROR') || line.includes('Failed')) cls = 'error';
                    return `<div class="log-line ${cls}">${line}</div>`;
                }).join('');
                logsDiv.scrollTop = logsDiv.scrollHeight;
            } catch (e) {}
        }
        
        // --- Three.js Logic ---
        let scene, camera, renderer, headLine, markerPoints = {};
        
        function init3DViewer() {
            const container = document.getElementById('trajectory-viewer');
            
            // Scene
            scene = new THREE.Scene();
            
            // Camera (looking down-ish)
            camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.01, 10);
            camera.position.set(0, 0.5, 0.5);
            camera.lookAt(0, 0, 0);
            
            // Renderer
            renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
            renderer.setSize(container.clientWidth, container.clientHeight);
            container.appendChild(renderer.domElement);
            
            // Axes Helper (RGB = XYZ)
            const axesHelper = new THREE.AxesHelper(0.2);
            scene.add(axesHelper);
            
            // Grid Helper
            const gridHelper = new THREE.GridHelper(2, 20, 0x444444, 0x222222);
            scene.add(gridHelper);
            
            // Head Trajectory Line
            const geometry = new THREE.BufferGeometry();
            const material = new THREE.LineBasicMaterial({ color: 0x00ffff });
            headLine = new THREE.Line(geometry, material);
            scene.add(headLine);
            
            // Head Sphere (Current Position)
            const headGeo = new THREE.SphereGeometry(0.05, 16, 16);
            const headMat = new THREE.MeshBasicMaterial({ color: 0x00ffff });
            headMesh = new THREE.Mesh(headGeo, headMat);
            scene.add(headMesh);
            
            // Markers container
            markerPoints = {}; // id -> mesh
            
            // Resize handler
            window.addEventListener('resize', () => {
                if (container.clientWidth > 0 && container.clientHeight > 0) {
                    camera.aspect = container.clientWidth / container.clientHeight;
                    camera.updateProjectionMatrix();
                    renderer.setSize(container.clientWidth, container.clientHeight);
                }
            });
            
            // Start animation loop
            animate();
            
            // Start data polling
            setInterval(updateTrajectoryData, 100);
        }
        
        function animate() {
            requestAnimationFrame(animate);
            renderer.render(scene, camera);
        }
        
        async function updateTrajectoryData() {
            if (document.getElementById('trajectory-viewer').style.display === 'none') return;
            
            try {
                const res = await fetch('/api/trajectory_data');
                const data = await res.json();
                
                // Update Head Trajectory
                if (data.head_trajectory && data.head_trajectory.length > 0) {
                    const points = data.head_trajectory.map(p => new THREE.Vector3(p[0], p[1], p[2]));
                    headLine.geometry.setFromPoints(points);
                    
                    // Update stats
                    const last = data.head_trajectory[data.head_trajectory.length - 1];
                    const count = Object.keys(data.markers).length;
                    document.getElementById('viewer-stats').textContent = 
                        `Head: [${last[0].toFixed(2)}, ${last[1].toFixed(2)}, ${last[2].toFixed(2)}] | Markers: ${count}`;
                    
                    // Update Head Sphere
                    if (headMesh) {
                        headMesh.position.set(last[0], last[1], last[2]);
                    }
                    
                    // Simple auto-follow
                    // Move camera to chase the head
                    const targetPos = new THREE.Vector3(last[0], last[1] + 1.0, last[2] + 1.0);
                    camera.position.lerp(targetPos, 0.05); // Smooth follow
                    camera.lookAt(last[0], last[1], last[2]);
                }
                
                // Update Markers
                if (data.markers) {
                    for (const [id, pos] of Object.entries(data.markers)) {
                        if (!markerPoints[id]) {
                            // Create new marker mesh
                            const geometry = new THREE.SphereGeometry(0.02, 16, 16);
                            const material = new THREE.MeshBasicMaterial({ color: 0xff00ff }); // Magenta
                            const sphere = new THREE.Mesh(geometry, material);
                            scene.add(sphere);
                            markerPoints[id] = sphere;
                            
                            // Add label (detectable ID) - simplistic for now
                        }
                        markerPoints[id].position.set(pos[0], pos[1], pos[2]);
                    }
                }
            } catch (e) {}
        }
        
        function updatePhaseIndicator() {
            const phases = ['print', 'camera_selection', 'intrinsic', 'verification', 'extrinsic'];
            const currentIdx = phases.indexOf(currentPhase);
            for (let i = 0; i <= 4; i++) {
                const el = document.getElementById(`phase-${i}`);
                el.classList.remove('active', 'completed');
                if (i === currentIdx) el.classList.add('active');
                else if (i < currentIdx) el.classList.add('completed');
            }
        }
        
        async function nextPhase() { await fetch('/api/next_phase', {method: 'POST'}); }
        async function prevPhase() { await fetch('/api/prev_phase', {method: 'POST'}); }
        async function resetPhase() { await fetch('/api/reset_phase', {method: 'POST'}); }
        function downloadResults() { window.location.href = '/api/download_calibration'; }
    </script>

</body>
</html>
'''




# Routes
@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/calibration_sheets.pdf')
def serve_calibration_pdf():
    """Serve the calibration sheets PDF."""
    pdf_path = Path(__file__).parent.parent / "calibration_sheets.pdf"
    if pdf_path.exists():
        return Response(
            open(pdf_path, 'rb').read(),
            mimetype='application/pdf',
            headers={'Content-Disposition': 'inline; filename=calibration_sheets.pdf'}
        )
    else:
        return "PDF not found. Run 'python -m avp_stream.prepare_calibration' first.", 404

@app.route('/api/cameras')
def get_cameras():
    if not state.cameras:
        state.cameras = discover_cameras()
    return jsonify(state.cameras)

@app.route('/camera_preview/<int:camera_id>')
def camera_preview(camera_id):
    """Return a single frame as JPEG for camera preview grid."""
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        # Return placeholder
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, "No Signal", (220, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ret, img = cap.read()
        cap.release()
        if not ret:
            img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return Response(jpeg.tobytes(), mimetype='image/jpeg')


# Store camera stream threads
camera_streams = {}
camera_stream_locks = {}

def generate_camera_stream(camera_id):
    """Generator for individual camera MJPEG streaming."""
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        while True:
            img = np.zeros((360, 480, 3), dtype=np.uint8)
            cv2.putText(img, "No Signal", (160, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.5)
            return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            
            # Resize for thumbnail
            frame = cv2.resize(frame, (480, 270))
            
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            
            time.sleep(0.1)  # ~10fps for preview
    finally:
        cap.release()


@app.route('/camera_stream/<int:camera_id>')
def camera_stream(camera_id):
    """MJPEG stream for individual camera preview."""
    return Response(generate_camera_stream(camera_id),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/select_camera', methods=['POST'])
def select_camera():
    data = request.json
    state.selected_camera = data['camera']
    state.is_stereo = data['stereo']
    
    # Start streaming
    start_video_capture()
    state.phase = 'intrinsic'
    
    # Initialize intrinsic calibrator based on mode
    if state.calibration_mode == "iphone":
        # iPhone mode: plain checkerboard
        from avp_stream.run_calibration import (
            IPHONE_CHECKER_COLS, IPHONE_CHECKER_ROWS, IPHONE_CHECKER_SIZE
        )
        state.intrinsic_calibrator = IntrinsicCalibrator(
            state.selected_camera, state.is_stereo,
            mode="checkerboard",
            rows=IPHONE_CHECKER_ROWS,
            cols=IPHONE_CHECKER_COLS,
            square_size=IPHONE_CHECKER_SIZE
        )
    else:
        # Print mode: ChArUco
        state.intrinsic_calibrator = IntrinsicCalibrator(
            state.selected_camera, state.is_stereo
        )
    
    return jsonify({"status": "ok"})

def start_video_capture():
    """Start video capture in background thread."""
    if state.streaming:
        return
    
    state.cap = cv2.VideoCapture(state.selected_camera)
    if not state.cap.isOpened():
        return
    
    state.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    state.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    state.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    state.streaming = True
    state.stream_thread = threading.Thread(target=capture_loop, daemon=True)
    state.stream_thread.start()

def capture_loop():
    """Background thread for video capture and processing."""
    while state.streaming:
        ret, frame = state.cap.read()
        if not ret:
            break
            
        vis_frame = frame.copy()
        
        if state.phase == 'intrinsic' and state.intrinsic_calibrator:
            vis_frame, status = state.intrinsic_calibrator.process_frame(frame)
            state.detection_status = status
            
        elif state.phase == 'verification':
            # Verification logic
            if state.is_stereo:
                left_frame, right_frame = split_stereo_image(frame)
                
                # Process left
                if state.verification_engines["left"]:
                    vis_l, stats_l = state.verification_engines["left"].process_frame(left_frame)
                    state.verification_status["left"] = stats_l
                else:
                    vis_l = left_frame
                
                # Process right
                if state.verification_engines["right"]:
                    vis_r, stats_r = state.verification_engines["right"].process_frame(right_frame)
                    state.verification_status["right"] = stats_r
                else:
                    vis_r = right_frame
                
                # Combine for display
                vis_frame = np.hstack([vis_l, vis_r])
                
            else:
                if state.verification_engines["left"]:
                    vis_frame, stats = state.verification_engines["left"].process_frame(frame)
                    state.verification_status["left"] = stats
        elif state.phase == 'extrinsic':
            # Extrinsic calibration with marker detection
            vis_frame = frame.copy()
            
            # --- Visualization ---
            # For stereo mode, process each half with its own calibrator
            if state.is_stereo:
                left_calib = state.extrinsic_calibrators.get("left")
                right_calib = state.extrinsic_calibrators.get("right")
                
                if left_calib and right_calib:
                    try:
                        h, w = frame.shape[:2]
                        half_w = w // 2
                        
                        # Split frame into left and right halves
                        left_frame = frame[:, :half_w].copy()
                        right_frame = frame[:, half_w:].copy()
                        
                        # Process left half
                        gray_left = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
                        corners_l, ids_l, _ = left_calib.detector.detectMarkers(gray_left)
                        
                        if ids_l is not None:
                            cv2.aruco.drawDetectedMarkers(left_frame, corners_l, ids_l)
                            
                            # Draw axis for current target
                            if state.extrinsic_step < 3:
                                current_target = [0, 2, 3][state.extrinsic_step]
                                for i, mk_id in enumerate(ids_l):
                                    if mk_id[0] == current_target:
                                        half_size = left_calib.marker_size / 2
                                        obj_pts = np.array([[-half_size, half_size, 0], [half_size, half_size, 0], 
                                                           [half_size, -half_size, 0], [-half_size, -half_size, 0]], dtype=np.float32)
                                        succ, r, t = cv2.solvePnP(obj_pts, corners_l[i].reshape(-1, 2), 
                                                                   left_calib.camera_matrix, left_calib.dist_coeffs)
                                        if succ:
                                            cv2.drawFrameAxes(left_frame, left_calib.camera_matrix, left_calib.dist_coeffs, r, t, 0.05)
                        
                        # Process right half
                        gray_right = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
                        corners_r, ids_r, _ = right_calib.detector.detectMarkers(gray_right)
                        
                        if ids_r is not None:
                            cv2.aruco.drawDetectedMarkers(right_frame, corners_r, ids_r)
                            
                            # Draw axis for current target
                            if state.extrinsic_step < 3:
                                current_target = [0, 2, 3][state.extrinsic_step]
                                for i, mk_id in enumerate(ids_r):
                                    if mk_id[0] == current_target:
                                        half_size = right_calib.marker_size / 2
                                        obj_pts = np.array([[-half_size, half_size, 0], [half_size, half_size, 0], 
                                                           [half_size, -half_size, 0], [-half_size, -half_size, 0]], dtype=np.float32)
                                        succ, r, t = cv2.solvePnP(obj_pts, corners_r[i].reshape(-1, 2), 
                                                                   right_calib.camera_matrix, right_calib.dist_coeffs)
                                        if succ:
                                            cv2.drawFrameAxes(right_frame, right_calib.camera_matrix, right_calib.dist_coeffs, r, t, 0.05)
                        
                        # Combine back into vis_frame
                        vis_frame = np.hstack([left_frame, right_frame])
                    except Exception as e:
                        pass
            else:
                # Mono mode: use left calibrator for entire frame
                calib = state.extrinsic_calibrators.get("left")
                if calib:
                    try:
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        corners, ids, _ = calib.detector.detectMarkers(gray)
                        
                        if ids is not None:
                            cv2.aruco.drawDetectedMarkers(vis_frame, corners, ids)
                            
                            # Draw axis for current target if active
                            if state.extrinsic_step < 3:
                                current_target = [0, 2, 3][state.extrinsic_step]
                                for i, mk_id in enumerate(ids):
                                    if mk_id[0] == current_target:
                                        half_size = calib.marker_size / 2
                                        obj_pts = np.array([[-half_size, half_size, 0], [half_size, half_size, 0], 
                                                           [half_size, -half_size, 0], [-half_size, -half_size, 0]], dtype=np.float32)
                                        succ, r, t = cv2.solvePnP(obj_pts, corners[i].reshape(-1, 2), 
                                                                   calib.camera_matrix, calib.dist_coeffs)
                                        if succ:
                                            cv2.drawFrameAxes(vis_frame, calib.camera_matrix, calib.dist_coeffs, r, t, 0.05)
                    except Exception as e:
                        pass

            # Draw persistent instructions on video
            if state.extrinsic_step < 3:
                marker_ids = [0, 2, 3]
                target = marker_ids[state.extrinsic_step]
                cv2.putText(vis_frame, f"Step {state.extrinsic_step+1}/3: Look at Marker {target}", 
                           (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                
                # Show sample counts per camera
                total = state.TARGET_SAMPLES_PER_STEP
                if state.is_stereo:
                    l_count = state.samples_per_step["left"][state.extrinsic_step]
                    r_count = state.samples_per_step["right"][state.extrinsic_step]
                    cv2.putText(vis_frame, f"L: {l_count}/{total}  R: {r_count}/{total}", 
                               (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                else:
                    progress = state.samples_per_step["left"][state.extrinsic_step]
                    cv2.putText(vis_frame, f"Samples: {progress}/{total}", 
                               (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            else:
                cv2.putText(vis_frame, "Calibration Complete!", 
                           (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            if state.avp_connected and state.extrinsic_calibrators["left"] and state.avp_streamer and state.extrinsic_step < 3:
                # Try to collect samples for each camera
                marker_ids = [0, 2, 3]
                current_marker = marker_ids[state.extrinsic_step]
                
                try:
                    # Get data from Vision Pro
                    latest = state.avp_streamer.get_latest()
                    markers = state.avp_streamer.get_markers()
                    
                    # Record trajectory data for visualization (always, if connected)
                    if latest and "head" in latest:
                        try:
                            T_head = latest["head"][0]
                            pos = T_head[:3, 3].tolist()
                            state.head_trajectory.append(pos)
                            
                            if len(state.head_trajectory) > 500:
                                state.head_trajectory.pop(0)
                                
                            if markers:
                                for mid, mdata in markers.items():
                                    if "pose" in mdata:
                                        mpos = mdata["pose"][:3, 3].tolist()
                                        state.marker_positions[mid] = mpos
                        except Exception:
                            pass

                    # Send calibration status to VisionOS
                    marker_detected = latest and "head" in latest and current_marker in markers
                    state.send_calibration_status(marker_detected=marker_detected, step_status=0)

                    if latest and "head" in latest and current_marker in markers:
                        T_b_h = latest["head"][0]  # Head pose
                        T_b_m = markers[current_marker]["pose"]  # Marker pose from ARKit
                        current_time = time.time()
                        head_pos = T_b_h[:3, 3]
                        
                        # Determine which cameras to process
                        cam_names = ["left", "right"] if state.is_stereo else ["left"]
                        
                        # Split frame if stereo
                        if state.is_stereo:
                            left_frame, right_frame = split_stereo_image(frame)
                            frames = {"left": left_frame, "right": right_frame}
                        else:
                            frames = {"left": frame}
                        
                        for cam_name in cam_names:
                            cam_frame = frames[cam_name]
                            calibrator = state.extrinsic_calibrators[cam_name]
                            
                            # Per-camera capture logic
                            is_first = (state.samples_per_step[cam_name][state.extrinsic_step] == 0)
                            time_since_last = current_time - state.last_capture_time[cam_name]
                            
                            should_capture = False
                            if time_since_last >= state.MIN_CAPTURE_INTERVAL:
                                if is_first:
                                    should_capture = True
                                elif state.last_head_pos[cam_name] is not None:
                                    dist = np.linalg.norm(head_pos - state.last_head_pos[cam_name])
                                    if dist >= state.MIN_HEAD_MOVEMENT:
                                        should_capture = True
                            
                            # Collect sample (detection runs regardless, save only if should_capture)
                            sample = calibrator.collect_sample(
                                cam_frame, current_marker, T_b_h, T_b_m, save=should_capture
                            )
                            
                            if sample:
                                # Update debug projection for this camera
                                debug_pos = (sample.T_b_h @ sample.T_c_m)[:3, 3]
                                state.debug_projections[cam_name] = debug_pos.tolist()
                            
                            if should_capture and sample:
                                state.last_capture_time[cam_name] = current_time
                                state.last_head_pos[cam_name] = head_pos.copy()
                                state.samples_per_step[cam_name][state.extrinsic_step] += 1
                                
                                # Update running result from background thread
                                if calibrator.latest_result is not None:
                                    state.extrinsic_results[cam_name] = calibrator.latest_result
                                    state.extrinsic_losses[cam_name] = calibrator.last_loss
                        
                        # Log progress (use left camera count as reference for mono, minimum for stereo)
                        if state.is_stereo:
                            left_count = state.samples_per_step["left"][state.extrinsic_step]
                            right_count = state.samples_per_step["right"][state.extrinsic_step]
                            min_count = min(left_count, right_count)
                            if min_count > 0 and min_count % 5 == 0:
                                state.add_log(f"Step {state.extrinsic_step+1}: L={left_count} R={right_count}/{state.TARGET_SAMPLES_PER_STEP}")
                        else:
                            count = state.samples_per_step["left"][state.extrinsic_step]
                            if count > 0 and count % 5 == 0:
                                state.add_log(f"Step {state.extrinsic_step+1}: {count}/{state.TARGET_SAMPLES_PER_STEP}")
                        
                        # Check if step complete (both cameras must reach target for stereo)
                        if state.is_stereo:
                            step_complete = all(
                                state.samples_per_step[cam][state.extrinsic_step] >= state.TARGET_SAMPLES_PER_STEP
                                for cam in ["left", "right"]
                            )
                        else:
                            step_complete = state.samples_per_step["left"][state.extrinsic_step] >= state.TARGET_SAMPLES_PER_STEP
                        
                        if step_complete:
                            # Send step complete status
                            state.send_calibration_status(marker_detected=True, step_status=2)
                            
                            state.extrinsic_step += 1
                            if state.extrinsic_step < 3:
                                next_m = marker_ids[state.extrinsic_step]
                                state.add_log(f"SUCCESS: Marker {current_marker} done! Move to Marker {next_m}")
                            else:
                                # All steps complete - compute final results for all cameras
                                state.add_log("ALL STEPS COMPLETE. Computing final T_h_c...")
                                
                                # Signal calibration is computing
                                state.send_calibration_status(marker_detected=True, step_status=1)
                                
                                for cam_name in cam_names:
                                    calibrator = state.extrinsic_calibrators[cam_name]
                                    result = calibrator.compute_T_h_c()
                                    if result is not None:
                                        state.extrinsic_results[cam_name] = result
                                        state.extrinsic_losses[cam_name] = calibrator.last_loss
                                        state.add_log(f"SUCCESS: {cam_name.upper()} T_h_c computed! Loss: {state.extrinsic_losses[cam_name]:.2f}mm")
                                        
                                        # Save samples for debugging
                                        try:
                                            samples_dir = Path(__file__).parent.parent / "calibration"
                                            samples_dir.mkdir(exist_ok=True)
                                            samples_file = samples_dir / f"extrinsic_samples_{cam_name}.json"
                                            calibrator.save_samples(str(samples_file))
                                            state.add_log(f"Saved {cam_name} samples to {samples_file}")
                                        except Exception as save_err:
                                            state.add_log(f"Warning: Could not save samples: {save_err}")
                                    else:
                                        state.add_log(f"FAILED: Could not compute {cam_name.upper()} T_h_c")
                                
                                # Transition to results phase
                                state.phase = "results"
                                state.add_log("Calibration complete! Showing results...")
                                
                except Exception as e:
                    pass  # Silent fail for continuous collection
                
                # Legacy visualization removed to avoid duplicate detection and clutter
        
        with state.frame_lock:
            state.current_frame = vis_frame
        
        time.sleep(0.01)  # ~100fps max


@app.route('/api/has_checkpoint')
def has_checkpoint():
    """Check if a checkpoint exists."""
    return jsonify({"exists": CHECKPOINT_FILE.exists()})

@app.route('/api/load_checkpoint', methods=['POST'])
def api_load_checkpoint():
    """Load checkpoint and resume calibration."""
    if state.load_checkpoint():
        # Re-initialize camera if needed
        if state.selected_camera is not None and state.phase not in ['print', 'camera_selection']:
            start_video_capture()
            
            # Re-initialize intrinsic calibrator if in intrinsic phase
            if state.phase == 'intrinsic':
                if state.calibration_mode == "iphone":
                    from avp_stream.run_calibration import (
                        IPHONE_CHECKER_COLS, IPHONE_CHECKER_ROWS, IPHONE_CHECKER_SIZE
                    )
                    state.intrinsic_calibrator = IntrinsicCalibrator(
                        state.selected_camera, state.is_stereo,
                        mode="checkerboard",
                        rows=IPHONE_CHECKER_ROWS,
                        cols=IPHONE_CHECKER_COLS,
                        square_size=IPHONE_CHECKER_SIZE
                    )
                else:
                    state.intrinsic_calibrator = IntrinsicCalibrator(
                        state.selected_camera, state.is_stereo
                    )
            
            # Re-initialize verification engines if in verification phase
            if state.phase == 'verification' and state.intrinsic_result:
                if "left" in state.intrinsic_result:
                    res = state.intrinsic_result["left"]
                    if res.camera_matrix is not None:
                        state.verification_engines["left"] = VerificationEngine(res.camera_matrix, res.distortion, state.calibration_mode)
                if state.is_stereo and "right" in state.intrinsic_result:
                    res = state.intrinsic_result["right"]
                    if res.camera_matrix is not None:
                        state.verification_engines["right"] = VerificationEngine(res.camera_matrix, res.distortion, state.calibration_mode)
        
        return jsonify({
            "success": True, 
            "phase": state.phase,
            "is_stereo": state.is_stereo,
            "calibration_mode": state.calibration_mode
        })
    return jsonify({"success": False})

@app.route('/api/clear_checkpoint', methods=['POST'])
def api_clear_checkpoint():
    """Clear checkpoint and start fresh."""
    state.clear_checkpoint()
    return jsonify({"success": True})


def generate_frames():
    """Generator for MJPEG streaming."""
    while True:
        with state.frame_lock:
            if state.current_frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "No Camera", (250, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            else:
                frame = state.current_frame.copy()
        
        # Resize for streaming
        h, w = frame.shape[:2]
        max_width = 1280
        if w > max_width:
            scale = max_width / w
            frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
        
        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        
        time.sleep(0.033)  # ~30fps

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def get_status():
    """Get current calibration status with full camera data."""
    status = {
        "phase": state.phase,
        "is_stereo": state.is_stereo,
        "calibration_mode": state.calibration_mode,
        "stability": 0,
        "left": None,
        "right": None,
    }
    
    if state.phase == 'intrinsic' and state.intrinsic_calibrator:
        results = state.intrinsic_calibrator.get_results()
        
        # Get data for each camera
        for name in ["left", "right"]:
            result = results.get(name)
            n_samples = len(state.intrinsic_calibrator.collections[name]["imgpoints"])
            
            cam_data = {
                "samples": n_samples,
                "fx": None,
                "fy": None,
                "cx": None,
                "cy": None,
                "error": 999,
            }
            
            if result and result.camera_matrix is not None:
                K = result.camera_matrix
                cam_data["fx"] = float(K[0, 0])
                cam_data["fy"] = float(K[1, 1])
                cam_data["cx"] = float(K[0, 2])
                cam_data["cy"] = float(K[1, 2])
                cam_data["error"] = float(result.reprojection_error)
            
            # Add per-camera stability
            conv = state.detection_status.get("convergence", {}).get(name, {})
            cam_data["stability"] = conv.get("stability", 0)
            
            status[name] = cam_data
    
    elif state.phase == 'verification':
        # Add verification stats
        for name in ["left", "right"]:
            stats = state.verification_status.get(name, {})
            status[name] = stats
    
    elif state.phase == 'extrinsic':
        status["avp_connected"] = state.avp_connected
        status["avp_ip"] = state.avp_ip
        status["extrinsic_step"] = state.extrinsic_step
        status["extrinsic_samples"] = state.samples_per_step
        status["debug_projections"] = state.debug_projections
        
        # Add extrinsic results for each camera (live from background thread)
        extrinsic_results = {}
        for cam_name in ["left", "right"]:
            calibrator = state.extrinsic_calibrators.get(cam_name)
            if calibrator is not None:
                latest = getattr(calibrator, 'latest_result', None)
                loss = getattr(calibrator, 'last_loss', None)
                if latest is not None:
                    extrinsic_results[cam_name] = {
                        "matrix": latest.flatten().tolist(),
                        "loss": loss
                    }
            # Fallback to stored final result
            elif state.extrinsic_results.get(cam_name) is not None:
                extrinsic_results[cam_name] = {
                    "matrix": state.extrinsic_results[cam_name].flatten().tolist(),
                    "loss": state.extrinsic_losses.get(cam_name)
                }
        status["extrinsic_results"] = extrinsic_results
    
    elif state.phase == 'results':
        # Return comprehensive results
        status["intrinsic_results"] = {}
        status["extrinsic_results"] = {}
        
        # Intrinsic results
        if state.intrinsic_result:
            for cam_name in ["left", "right"]:
                if cam_name in state.intrinsic_result:
                    res = state.intrinsic_result[cam_name]
                    if res.camera_matrix is not None:
                        K = res.camera_matrix
                        D = res.distortion
                        status["intrinsic_results"][cam_name] = {
                            "fx": float(K[0, 0]),
                            "fy": float(K[1, 1]),
                            "cx": float(K[0, 2]),
                            "cy": float(K[1, 2]),
                            "distortion": D.flatten().tolist() if D is not None else None,
                            "error": float(res.reprojection_error),
                            "image_size": res.image_size,
                        }
        
        # Extrinsic results
        for cam_name in ["left", "right"]:
            if state.extrinsic_results.get(cam_name) is not None:
                status["extrinsic_results"][cam_name] = {
                    "matrix": state.extrinsic_results[cam_name].flatten().tolist(),
                    "loss": state.extrinsic_losses.get(cam_name)
                }
    
    return jsonify(status)


@app.route('/api/set_mode', methods=['POST'])
def set_mode():
    """Set calibration mode (print or iphone)."""
    data = request.json
    mode = data.get('mode', 'print')
    if mode in ['print', 'iphone']:
        state.calibration_mode = mode
        state.add_log(f"Mode set to: {mode}")
    return jsonify({"mode": state.calibration_mode})


@app.route('/api/next_phase', methods=['POST'])
def next_phase():
    """Move to next calibration phase."""
    if state.phase == 'intrinsic':
        state.intrinsic_result = state.intrinsic_calibrator.get_results()
        
        # Initialize verification engines
        if state.intrinsic_result:
            if "left" in state.intrinsic_result:
                res = state.intrinsic_result["left"]
                if res.camera_matrix is not None:
                    state.verification_engines["left"] = VerificationEngine(res.camera_matrix, res.distortion, state.calibration_mode)
            
            if state.is_stereo and "right" in state.intrinsic_result:
                res = state.intrinsic_result["right"]
                if res.camera_matrix is not None:
                    state.verification_engines["right"] = VerificationEngine(res.camera_matrix, res.distortion, state.calibration_mode)
        
        state.phase = 'verification'
        state.save_checkpoint()
    elif state.phase == 'verification':
        state.phase = 'extrinsic'
        state.save_checkpoint()
    elif state.phase == 'extrinsic':
        state.phase = 'complete'
        state.save_checkpoint()
    
    return jsonify({"phase": state.phase})

@app.route('/api/prev_phase', methods=['POST'])
def prev_phase():
    """Move to previous calibration phase."""
    if state.phase == 'verification':
        state.phase = 'intrinsic'
        # Optional: Reset verification state if needed
    elif state.phase == 'extrinsic':
        state.phase = 'verification'
    
    return jsonify({"phase": state.phase})

@app.route('/api/reset_phase', methods=['POST'])
def reset_phase():
    """Reset current calibration phase."""
    if state.phase == 'intrinsic' and state.intrinsic_calibrator:
        state.intrinsic_calibrator.reset()
    
    return jsonify({"status": "ok"})

@app.route('/api/connect_avp', methods=['POST'])
def connect_avp():
    """Connect to Vision Pro."""
    from avp_stream import VisionProStreamer
    
    data = request.json
    ip = data.get('ip', '')
    
    state.avp_ip = ip
    state.add_log(f"Connecting to Vision Pro at {ip}...")
    
    try:
        # Import here to avoid circular imports
        streamer = VisionProStreamer(ip=ip, verbose=True)
        state.avp_streamer = streamer  # Store reference!
        state.avp_connected = True
        state.add_log(f"SUCCESS: Connected to Vision Pro!")
        state.add_log(f"Hold marker in view of both camera and Vision Pro.")
        
        # Initialize extrinsic calibrators for left (and right if stereo)
        if state.intrinsic_result and "left" in state.intrinsic_result:
            from avp_stream.run_calibration import ExtrinsicCalibrator
            
            cam_names = ["left", "right"] if state.is_stereo else ["left"]
            for cam_name in cam_names:
                if cam_name in state.intrinsic_result:
                    res = state.intrinsic_result[cam_name]
                    if res.camera_matrix is not None:
                        # Scale matrix if resolution changed
                        mtx = res.camera_matrix
                        if state.cap and state.cap.isOpened():
                            current_w = int(state.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            current_h = int(state.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            if state.is_stereo:
                                current_w = current_w // 2
                                
                            if res.image_size and (res.image_size[0] != current_w or res.image_size[1] != current_h):
                                state.add_log(f"Scaling {cam_name} matrix: {res.image_size} -> {(current_w, current_h)}")
                                mtx = scale_camera_matrix(mtx, res.image_size, (current_w, current_h))

                        # Determine marker size based on mode
                        marker_size = EXTRINSIC_MARKER_SIZE_PRINT if state.calibration_mode == "print" else EXTRINSIC_MARKER_SIZE_IPHONE
                        
                        state.extrinsic_calibrators[cam_name] = ExtrinsicCalibrator(
                            mtx, res.distortion, marker_size
                        )
                        state.add_log(f"Extrinsic calibrator for {cam_name.upper()} initialized (marker={int(marker_size*1000)}mm).")
            
            state.add_log(f"Collecting samples... (0/{state.TARGET_SAMPLES_PER_STEP})")
        
        return jsonify({"success": True})
    except Exception as e:
        state.add_log(f"ERROR: Failed to connect - {e}")
        state.avp_connected = False
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/trajectory_data')
def get_trajectory_data():
    """Get 3D trajectory data for visualization."""
    return jsonify({
        "head_trajectory": state.head_trajectory,
        "markers": state.marker_positions
    })

@app.route('/api/extrinsic_logs')
def get_extrinsic_logs():
    """Get extrinsic calibration logs."""
    return jsonify({"logs": state.extrinsic_logs})

@app.route('/api/download_calibration')
def download_calibration():
    """Download calibration results as JSON."""
    result = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "stereo": state.is_stereo,
    }
    
    if state.intrinsic_result:
        for name in ["left", "right"] if state.is_stereo else ["left"]:
            r = state.intrinsic_result.get(name)
            if r and r.camera_matrix is not None:
                result[f"intrinsic_{name}"] = {
                    "camera_matrix": r.camera_matrix.tolist(),
                    "distortion": r.distortion.tolist(),
                    "image_size": list(r.image_size) if r.image_size else None,
                    "reprojection_error": r.reprojection_error,
                    "n_samples": r.n_samples,
                }
    
    if state.extrinsic_result is not None:
        result["extrinsic"] = {
            "T_head_camera": state.extrinsic_result.tolist(),
        }
    
    return Response(
        json.dumps(result, indent=2),
        mimetype='application/json',
        headers={"Content-Disposition": "attachment; filename=camera_calibration.json"}
    )

def main():
    """Start the calibration web server."""
    print("=" * 60)
    print("CAMERA CALIBRATION WEB UI")
    print("=" * 60)
    print()
    print("Starting web server...")
    print("Open your browser to: http://localhost:5050")
    print()
    print("Press Ctrl+C to stop the server.")
    print()
    
    # Open browser
    webbrowser.open('http://localhost:5050')
    
    # Start Flask
    app.run(host='0.0.0.0', port=5050, debug=False, threaded=True)

if __name__ == '__main__':
    main()
