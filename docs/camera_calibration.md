# Camera Calibration Guide

This guide explains how to calibrate external UVC cameras for use with VisionProTeleop. There are two types of calibration:

1. **Intrinsic Calibration** - Determines the camera's internal parameters (focal length, principal point, distortion)
2. **Extrinsic Calibration** - Determines the camera's position and orientation relative to the Vision Pro head

## Prerequisites

- Apple Vision Pro with the VisionProTeleop app installed
- External UVC camera (mono or stereo)
- Printed calibration patterns:
  - Checkerboard pattern for intrinsic calibration
  - ArUco markers for extrinsic calibration

## Generating Calibration Patterns

### ArUco Markers

Generate ArUco markers for extrinsic calibration:

```bash
python utils/generate_aruco_markers.py
```

This creates markers with IDs 0-3 using the `DICT_4X4_50` dictionary. Print them at the specified size (default: 110mm including border).

**Important**: Print multiple markers and place them at **different 3D locations** (not on the same flat surface). This is critical for accurate rotation estimation.

---

## Intrinsic Calibration

Intrinsic calibration determines the camera's internal optical parameters. This must be done **before** extrinsic calibration.

### Parameters Determined

- **Focal length** (fx, fy): How the camera maps 3D points to 2D pixels
- **Principal point** (cx, cy): The optical center of the image
- **Distortion coefficients**: Lens distortion correction (k1, k2, p1, p2, k3)

### Procedure

1. Open the VisionProTeleop app
2. Go to **Settings** → **Camera Calibration** → **Intrinsic Calibration**
3. Select your camera from the list
4. For stereo cameras, enable "Stereo Mode"
5. Hold a checkerboard pattern and move it around the camera's field of view
6. Capture images from various angles and distances
7. The app will automatically detect corners and compute calibration

### Tips for Good Intrinsic Calibration

- Cover the entire field of view
- Include images at various distances
- Tilt the checkerboard at different angles
- Ensure good lighting without glare
- For stereo, make sure both cameras see the pattern

---

## Extrinsic Calibration

Extrinsic calibration determines the rigid transform between the Vision Pro's head frame and your external camera. This is essential when the camera is mounted to the headset.

### The Transform: T_head^camera

The calibration computes `T_head^camera`, which transforms points **from** the head frame **to** the camera frame:

```
P_camera = T_head^camera × P_head
```

To get the camera's position in the head frame (for visualization), use the inverse:

```
T_camera_in_head = inverse(T_head^camera)
```

### Coordinate Conventions

**ARKit/visionOS Head Frame** (right-handed):
- **X**: Right (wearer's right hand side)
- **Y**: Up
- **Z**: Backward (toward the back of the head)

**OpenCV Camera Frame** (right-handed):
- **X**: Right (in image)
- **Y**: Down (in image)
- **Z**: Forward (optical axis, into the scene)

For a camera mounted on the front of the headset looking forward, there's a 180° rotation around the X-axis between these conventions.

### Procedure

1. Complete intrinsic calibration first
2. Go to **Settings** → **Camera Calibration** → **Extrinsic Calibration**
3. Select your camera
4. Configure marker settings:
   - **Marker Size**: Physical size of printed markers (default: 110mm)
   - **Stereo Baseline**: For stereo cameras, set the known baseline if available
5. Click **Calibrate**

### During Calibration

1. **Place markers** at ≥3 different 3D locations (not coplanar!)
2. **Look at each marker** with the Vision Pro while the camera sees it
3. Move your head between markers to collect diverse samples
4. The status will show:
   - Number of samples collected (L/R for stereo)
   - Number of unique markers seen
   - Progress toward completion

### Why Multiple Markers at Different Locations?

ARKit can only track **one image at a time**. The app remembers marker positions once seen, allowing you to look at different markers throughout calibration.

Using markers at different 3D locations (varying depth, not just on a flat wall) is **critical** for accurate rotation estimation. Coplanar points can lead to degenerate solutions.

---

## Stereo Calibration

For stereo cameras (side-by-side format), the app calibrates both cameras independently and can optionally enforce constraints.

### Stereo Baseline Constraint

If you know your camera's baseline (distance between left and right optical centers), you can enforce it:

1. Set **Stereo Baseline** in the marker settings (e.g., 65mm)
2. The calibration will:
   - Average the rotations (cameras look in the same direction)
   - Average the Y and Z positions
   - Force X positions to `-baseline/2` and `+baseline/2`
   - Center the stereo rig at `x=0` in the head frame

This is useful when:
- The baseline is precisely known from manufacturer specs
- Independent calibration gives inconsistent results
- You want cameras perfectly symmetric around head center

### Disabling Baseline Constraint

Click "Disable" if you want fully independent left/right camera calibration.

---

## Algorithm Details

### Kabsch Algorithm

The extrinsic calibration uses the **Kabsch algorithm** to find the optimal rigid transform (rotation + translation) that minimizes the error between:
- Marker positions in the head frame (from ARKit)
- Marker positions in the camera frame (from OpenCV ArUco detection)

The algorithm:
1. Collects paired 3D points from multiple observations
2. Centers both point clouds at their centroids
3. Computes the covariance matrix
4. Uses SVD to find the optimal rotation
5. Computes the translation

### Coordinate Conversion

Since ARKit and OpenCV use different conventions:

1. **Before Kabsch**: OpenCV camera points are converted to ARKit convention
2. **After Kabsch**: The result is post-multiplied by Rx(180°) to account for the camera frame convention difference

---

## Visualization

### Swift 3D View

After calibration, tap "Show 3D View" to see the camera positions relative to the head. The view shows:
- Head coordinate frame (origin)
- Left camera position and orientation
- Right camera position (for stereo)
- Camera viewing directions (cyan lines)

### Python Visualization

Use the Python tool to visualize calibration from recordings:

```bash
python utils/visualize_tracking.py /path/to/recording_directory
```

This displays:
- Head coordinate frame
- Camera positions and orientations
- Calibration details (positions, baseline, etc.)

---

## Troubleshooting

### "Need at least 3 unique markers"

Place markers at different 3D locations. Markers on a flat wall don't provide enough depth variation for accurate calibration.

### Camera axes pointing wrong direction

This usually indicates a coordinate convention mismatch. The calibration automatically handles ARKit↔OpenCV conversion. If issues persist:
1. Verify marker size is correct
2. Ensure intrinsic calibration is accurate
3. Try recalibrating with more diverse marker positions

### Stereo baseline too small/large

- Check that the correct baseline is set
- Verify both cameras are detecting markers independently
- Try enforcing the baseline constraint if known

### High reprojection error

- Use more samples (increase min samples)
- Ensure markers are at different depths
- Check for motion blur in camera images
- Verify marker size matches physical printout

---

## Technical Reference

### Data Structures

```swift
struct ExtrinsicCalibrationData {
    let leftHeadToCamera: [Double]    // 4x4 matrix, column-major
    let rightHeadToCamera: [Double]?  // For stereo cameras
    let leftReprojectionError: Double
    let stereoBaselineMeters: Float?  // If constraint was used
    // ...
}
```

### Transform Usage

To transform a point from head frame to camera frame:

```python
import numpy as np

# Load T_head^camera (4x4 matrix)
T_head_to_camera = np.array(calibration['leftHeadToCamera']).reshape(4, 4, order='F')

# Transform point
p_head = np.array([x, y, z, 1])
p_camera = T_head_to_camera @ p_head
```

To get camera position in head frame:

```python
T_camera_in_head = np.linalg.inv(T_head_to_camera)
camera_position = T_camera_in_head[:3, 3]
```

### Export Format

The calibration can be exported as JSON with row-major 4x4 matrices:

```json
{
  "description": "Extrinsic calibration from Vision Pro head frame to camera frame",
  "convention": "T_head_to_camera transforms points from head frame to camera frame",
  "left_head_to_camera": [[...], [...], [...], [...]],
  "stereo_baseline_meters": 0.065
}
```
