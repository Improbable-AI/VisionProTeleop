#!/usr/bin/env python3
"""
Debug script for iterating over extrinsic calibration samples.

Usage:
    python scripts/debug_extrinsic_samples.py calibration/extrinsic_samples_left.json

This allows you to:
- Load saved samples from a previous calibration run
- Visualize the data
- Re-run the optimization with different parameters
- Analyze outliers
"""

import argparse
import json
import numpy as np
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from avp_stream.run_calibration import ExtrinsicCalibrator, ExtrinsicSample


def load_samples_raw(filepath: str) -> dict:
    """Load raw sample data from JSON."""
    with open(filepath, 'r') as f:
        return json.load(f)


def analyze_samples(data: dict):
    """Analyze the loaded samples."""
    print("\n" + "=" * 60)
    print("SAMPLE ANALYSIS")
    print("=" * 60)
    
    print(f"\nCamera Matrix:")
    cam_matrix = np.array(data["camera_matrix"])
    print(f"  fx: {cam_matrix[0, 0]:.2f}")
    print(f"  fy: {cam_matrix[1, 1]:.2f}")
    print(f"  cx: {cam_matrix[0, 2]:.2f}")
    print(f"  cy: {cam_matrix[1, 2]:.2f}")
    
    print(f"\nMarker size: {data['marker_size'] * 1000:.1f}mm")
    print(f"Total samples: {data['num_samples']}")
    
    samples = data["samples"]
    
    # Analyze by marker ID
    marker_counts = {}
    for s in samples:
        mid = s["marker_id"]
        marker_counts[mid] = marker_counts.get(mid, 0) + 1
    
    print(f"\nSamples per marker:")
    for mid in sorted(marker_counts.keys()):
        print(f"  Marker {mid}: {marker_counts[mid]} samples")
    
    # Analyze marker positions
    print(f"\nMarker positions in camera frame (T_c_m):")
    for mid in sorted(marker_counts.keys()):
        positions = []
        for s in samples:
            if s["marker_id"] == mid:
                T_c_m = np.array(s["T_c_m"])
                positions.append(T_c_m[:3, 3])
        
        positions = np.array(positions)
        mean_pos = positions.mean(axis=0)
        std_pos = positions.std(axis=0)
        print(f"  Marker {mid}:")
        print(f"    Mean: x={mean_pos[0]*1000:.1f}mm, y={mean_pos[1]*1000:.1f}mm, z={mean_pos[2]*1000:.1f}mm")
        print(f"    Std:  x={std_pos[0]*1000:.1f}mm, y={std_pos[1]*1000:.1f}mm, z={std_pos[2]*1000:.1f}mm")


def run_optimization(data: dict, verbose: bool = True):
    """Re-run optimization on loaded samples."""
    print("\n" + "=" * 60)
    print("RE-RUNNING OPTIMIZATION")
    print("=" * 60)
    
    # Create calibrator with saved parameters
    cam_matrix = np.array(data["camera_matrix"])
    dist_coeffs = np.array(data["dist_coeffs"])
    marker_size = data["marker_size"]
    
    calibrator = ExtrinsicCalibrator(
        camera_matrix=cam_matrix,
        dist_coeffs=dist_coeffs,
        marker_size=marker_size,
    )
    
    # Load samples
    samples = []
    for s in data["samples"]:
        sample = ExtrinsicSample(
            T_b_h=np.array(s["T_b_h"]),
            T_c_m=np.array(s["T_c_m"]),
            T_b_m=np.array(s["T_b_m"]),
            marker_id=s["marker_id"],
            timestamp=s["timestamp"],
        )
        samples.append(sample)
    
    with calibrator._samples_lock:
        calibrator.samples = samples
    
    print(f"\nLoaded {len(samples)} samples")
    
    # Compute result
    result = calibrator.compute_T_h_c()
    
    if result is not None:
        print(f"\nFinal T_h_c:")
        print(result)
        print(f"\nTranslation: {result[:3, 3] * 1000} mm")
        print(f"Final loss: {calibrator.last_loss:.2f} mm")
    else:
        print("\nFailed to compute T_h_c")
    
    calibrator.stop()
    return result


def compute_per_sample_errors(data: dict, T_h_c: np.ndarray):
    """Compute and display per-sample errors."""
    print("\n" + "=" * 60)
    print("PER-SAMPLE ERRORS")
    print("=" * 60)
    
    samples = data["samples"]
    errors = []
    
    for i, s in enumerate(samples):
        T_b_h = np.array(s["T_b_h"])
        T_c_m = np.array(s["T_c_m"])
        T_b_m = np.array(s["T_b_m"])
        
        # Expected position (from ARKit)
        T_h_b = np.linalg.inv(T_b_h)
        p_h_expected = (T_h_b @ np.append(T_b_m[:3, 3], 1))[:3]
        
        # Predicted position (from camera)
        p_h_predicted = (T_h_c @ np.append(T_c_m[:3, 3], 1))[:3]
        
        error = np.linalg.norm(p_h_expected - p_h_predicted) * 1000  # mm
        errors.append({
            "index": i,
            "marker_id": s["marker_id"],
            "error_mm": error,
            "timestamp": s["timestamp"],
        })
    
    # Sort by error (descending)
    errors_sorted = sorted(errors, key=lambda x: x["error_mm"], reverse=True)
    
    print(f"\nTop 10 highest error samples:")
    for e in errors_sorted[:10]:
        print(f"  Sample {e['index']:3d} (Marker {e['marker_id']}): {e['error_mm']:.2f} mm")
    
    errors_np = np.array([e["error_mm"] for e in errors])
    print(f"\nError statistics:")
    print(f"  Mean:   {np.mean(errors_np):.2f} mm")
    print(f"  Median: {np.median(errors_np):.2f} mm")
    print(f"  Std:    {np.std(errors_np):.2f} mm")
    print(f"  Min:    {np.min(errors_np):.2f} mm")
    print(f"  Max:    {np.max(errors_np):.2f} mm")
    
    return errors


def main():
    parser = argparse.ArgumentParser(description="Debug extrinsic calibration samples")
    parser.add_argument("filepath", help="Path to extrinsic samples JSON file")
    parser.add_argument("--analyze-only", action="store_true", help="Only analyze, don't re-run optimization")
    parser.add_argument("--errors", action="store_true", help="Show per-sample errors")
    args = parser.parse_args()
    
    # Load data
    print(f"Loading samples from: {args.filepath}")
    data = load_samples_raw(args.filepath)
    
    # Analyze
    analyze_samples(data)
    
    if not args.analyze_only:
        # Re-run optimization
        result = run_optimization(data)
        
        if args.errors and result is not None:
            compute_per_sample_errors(data, result)


if __name__ == "__main__":
    main()
