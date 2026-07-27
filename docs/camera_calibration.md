# Camera Calibration Guide

This guide covers calibrating a UVC camera rigidly mounted on Vision Pro via the Developer Strap (see Use Case 3: Egocentric Video Dataset Recording). Calibration has two parts: intrinsic, the camera's own optics, and extrinsic, where the camera sits relative to the Vision Pro coordinate frame that ARKit reports. Both are needed to align recorded video with hand and head tracking data.

Contributor note: this guide documents the calibration math and general workflow. The exact Tracking Manager app tap-through steps and screenshots should be filled in by the maintainers, since those are internal to the app UI.

## Why two separate calibrations

Intrinsic calibration answers what a 3D point looks like in pixels for this specific lens, independent of where the camera physically is. Extrinsic calibration answers where the camera is relative to a known reference frame, independent of the lens. Both are needed because the pipeline takes a hand or head pose from ARKit, expressed in the Vision Pro frame, and has to project it into a video frame captured by a camera that sits at a different physical location than any of Vision Pro's built in cameras.

## Intrinsic calibration

Goal: recover the camera matrix K, meaning the focal lengths fx and fy and principal point cx and cy, plus the lens distortion coefficients.

Step 1. Print a calibration target, either a checkerboard or a ChArUco board. ChArUco is more robust to partial occlusion and motion blur, which matters for a headset mounted camera.
Step 2. Capture 20 to 30 frames of the target at varied distances, angles, and positions across the frame, including the corners of the field of view. Corner coverage constrains the distortion model far better than center frame shots alone.
Step 3. Run standard corner detection and calibration, for example OpenCV's calibrateCamera function, or calibrateCameraCharuco for a ChArUco board.
Step 4. Check reprojection error. Under roughly 0.5 pixels average is a reasonable bar for this use case. If it is high, the usual cause is motion blur in the capture frames or an insufficient spread of target poses, not a bad solver.
Step 5. If the mounted lens has noticeable barrel or fisheye distortion, use the fisheye specific model, OpenCV's cv2.fisheye module, rather than the standard pinhole model. Forcing a wide field of view lens through the pinhole model biases the recovered focal length and leaves systematic error at the edges of the frame.

## Extrinsic calibration, the camera to headset transform

Goal: recover the fixed rigid transform T, rotation plus translation, from the UVC camera's optical frame to the Vision Pro's own tracking frame. Because the Developer Strap mount is rigid, this transform is constant and only needs to be solved once per physical mount.

This is a hand eye calibration problem, classically formulated as A X equals X B, where A_i are relative poses of the calibration target as seen by the UVC camera between two capture instants, B_i are the corresponding relative poses of the Vision Pro's own frame from ARKit between the same two instants, and X is the unknown, constant camera to headset transform being solved for.

Step 1. With the camera mounted and Vision Pro running, move the rig in front of a calibration target while capturing UVC camera frames and ARKit head pose, synchronized by timestamp.
Step 2. Detect the target in each UVC frame and solve PnP to get the target's pose relative to the camera at each timestamp.
Step 3. Pair up consecutive, or all pairwise, timestamps to build the A_i and B_i relative pose sets described above.
Step 4. Solve for X using a standard hand eye method, either Tsai-Lenz or Park-Martin. OpenCV exposes both through cv2.calibrateHandEye.
Step 5. Sanity check by reprojecting a known 3D point, for example a corner of the calibration target, through the recovered T into the UVC frame, and confirm it lands where the target actually is, across several held out frames the solve did not see.

## Practical notes specific to this rig

Recalibrate extrinsics if the physical mount is removed and reattached. The whole point of solving a rigid transform is that it assumes the mount does not move, and reattachment does not guarantee the same position within sub-millimeter tolerance.

Rolling shutter on the UVC camera combined with fast head motion during capture will bias the hand eye solve. Capture calibration data with slow, deliberate motion, even though normal recording will not have that constraint.

If intrinsics drift between sessions, for example with autofocus lenses, re-run intrinsic calibration. Extrinsics should still hold as long as the mount itself has not moved.

## References

Tsai, R. Y., and Lenz, R. K. (1989). A new technique for fully autonomous and efficient 3D robotics hand and eye calibration. IEEE Transactions on Robotics and Automation.

Park, F. C., and Martin, B. J. (1994). Robot sensor calibration: solving A X equals X B on the Euclidean group. IEEE Transactions on Robotics and Automation.

OpenCV documentation: Camera Calibration tutorial (docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) and calibrateHandEye reference (docs.opencv.org/4.x/d9/d0c/group__calib3d.html).
