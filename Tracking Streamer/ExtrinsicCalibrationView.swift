import SwiftUI
import AVFoundation

/// View for camera extrinsic calibration (head-to-camera transform)
struct ExtrinsicCalibrationView: View {
    @StateObject private var calibrationManager = ExtrinsicCalibrationManager.shared
    @StateObject private var intrinsicCalibrationManager = CameraCalibrationManager.shared
    @StateObject private var uvcCameraManager = UVCCameraManager.shared
    
    // Dismiss callback (for overlay mode)
    var onDismiss: (() -> Void)? = nil
    
    // Calibration state
    @State private var isCalibrating = false
    @State private var showingDeleteConfirmation = false
    @State private var calibrationToDelete: String? = nil
    @State private var showingProceedDialog = false
    @State private var isProcessing = false
    
    // Camera selection
    @State private var selectedDeviceId: String? = nil
    @State private var isStereoMode: Bool = false
    
    var body: some View {
        NavigationStack {
            VStack(spacing: 0) {
                if isCalibrating {
                    calibrationActiveView
                } else {
                    calibrationMenuView
                }
            }
            .navigationTitle("Extrinsic Calibration")
            .navigationBarTitleDisplayMode(.inline)
            .onAppear {
                dlog("📐 [ExtrinsicCalibrationView] View appeared!")
                setupFrameCallback()
            }
            .onDisappear {
                uvcCameraManager.onPixelBufferReceived = nil
            }
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    Button("Close") {
                        if isCalibrating {
                            calibrationManager.cancelCalibration()
                        }
                        onDismiss?()
                    }
                }
            }
        }
        .alert("Delete Calibration", isPresented: $showingDeleteConfirmation) {
            Button("Delete", role: .destructive) {
                if let deviceId = calibrationToDelete {
                    calibrationManager.deleteCalibration(for: deviceId)
                }
            }
            Button("Cancel", role: .cancel) {}
        } message: {
            Text("Are you sure you want to delete this calibration? This action cannot be undone.")
        }
    }
    
    // MARK: - Menu View
    
    private var calibrationMenuView: some View {
        ScrollView {
            VStack(spacing: 24) {
                // Info header
                infoHeaderSection
                
                // Available cameras
                availableCamerasSection
                
                // Saved calibrations
                savedCalibrationsSection
                
                // Marker config
                markerConfigSection
            }
            .padding()
        }
    }
    
    private var infoHeaderSection: some View {
        VStack(spacing: 12) {
            Image(systemName: "arrow.triangle.swap")
                .font(.system(size: 48))
                .foregroundColor(.purple)
            
            Text("Extrinsic Calibration")
                .font(.title2)
                .fontWeight(.bold)
            
            Text("Compute the rigid transform between Vision Pro's head and your external camera. Required when the camera is mounted to the headset.")
                .font(.subheadline)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
                .padding(.horizontal)
            
            // Requirements
            VStack(alignment: .leading, spacing: 8) {
                Label("Requires printed ArUco markers", systemImage: "qrcode")
                Label("Requires intrinsic calibration first", systemImage: "camera.aperture")
                Label("Move head while viewing markers", systemImage: "move.3d")
            }
            .font(.caption)
            .foregroundColor(.secondary)
            .padding()
            .background(Color.secondary.opacity(0.1))
            .cornerRadius(12)
        }
    }
    
    private var availableCamerasSection: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Available Cameras")
                .font(.headline)
            
            if uvcCameraManager.availableDevices.isEmpty {
                HStack {
                    Image(systemName: "video.slash")
                        .foregroundColor(.secondary)
                    Text("No cameras connected")
                        .foregroundColor(.secondary)
                }
                .frame(maxWidth: .infinity)
                .padding()
                .background(Color.secondary.opacity(0.1))
                .cornerRadius(12)
            } else {
                ForEach(uvcCameraManager.availableDevices, id: \.id) { device in
                    cameraDeviceRow(device)
                }
            }
        }
    }
    
    private func cameraDeviceRow(_ device: UVCDevice) -> some View {
        let hasIntrinsic = intrinsicCalibrationManager.hasCalibration(for: device.id)
        let hasExtrinsic = calibrationManager.hasCalibration(for: device.id)
        let intrinsicCalibration = intrinsicCalibrationManager.allCalibrations[device.id]
        let isStereo = intrinsicCalibration?.isStereo ?? false
        
        return VStack(alignment: .leading, spacing: 8) {
            HStack {
                VStack(alignment: .leading, spacing: 4) {
                    Text(device.name)
                        .font(.subheadline)
                        .fontWeight(.medium)
                    
                    HStack(spacing: 8) {
                        // Intrinsic status
                        HStack(spacing: 4) {
                            Image(systemName: hasIntrinsic ? "checkmark.circle.fill" : "xmark.circle.fill")
                                .font(.caption2)
                            Text("Intrinsic")
                                .font(.caption2)
                        }
                        .foregroundColor(hasIntrinsic ? .green : .red)
                        
                        // Extrinsic status
                        HStack(spacing: 4) {
                            Image(systemName: hasExtrinsic ? "checkmark.circle.fill" : "circle")
                                .font(.caption2)
                            Text("Extrinsic")
                                .font(.caption2)
                        }
                        .foregroundColor(hasExtrinsic ? .green : .orange)
                        
                        // Stereo badge
                        if isStereo {
                            Text("Stereo")
                                .font(.caption2)
                                .padding(.horizontal, 6)
                                .padding(.vertical, 2)
                                .background(Color.blue.opacity(0.2))
                                .foregroundColor(.blue)
                                .cornerRadius(4)
                        }
                    }
                }
                
                Spacer()
                
                Button(action: {
                    if hasIntrinsic {
                        selectedDeviceId = device.id
                        isStereoMode = isStereo
                        startCalibration(for: device)
                    }
                }) {
                    Text(hasExtrinsic ? "Recalibrate" : "Calibrate")
                        .font(.subheadline)
                        .fontWeight(.medium)
                        .foregroundColor(.white)
                        .padding(.horizontal, 16)
                        .padding(.vertical, 8)
                        .background(hasIntrinsic ? Color.purple : Color.gray)
                        .cornerRadius(8)
                }
                .disabled(!hasIntrinsic)
            }
            
            if !hasIntrinsic {
                Text("⚠️ Intrinsic calibration required first")
                    .font(.caption)
                    .foregroundColor(.orange)
            }
        }
        .padding()
        .background(Color.secondary.opacity(0.1))
        .cornerRadius(12)
    }
    
    private var savedCalibrationsSection: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Saved Calibrations")
                .font(.headline)
            
            if calibrationManager.allCalibrations.isEmpty {
                Text("No extrinsic calibrations saved")
                    .font(.subheadline)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(Color.secondary.opacity(0.1))
                    .cornerRadius(12)
            } else {
                ForEach(Array(calibrationManager.allCalibrations.keys.sorted()), id: \.self) { deviceId in
                    if let calibration = calibrationManager.allCalibrations[deviceId] {
                        savedCalibrationRow(calibration)
                    }
                }
            }
        }
    }
    
    private func savedCalibrationRow(_ calibration: ExtrinsicCalibrationData) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                VStack(alignment: .leading, spacing: 4) {
                    Text(calibration.cameraDeviceName)
                        .font(.subheadline)
                        .fontWeight(.medium)
                    
                    HStack(spacing: 8) {
                        Text(calibration.isStereo ? "Stereo" : "Mono")
                            .font(.caption2)
                            .padding(.horizontal, 6)
                            .padding(.vertical, 2)
                            .background(calibration.isStereo ? Color.blue.opacity(0.2) : Color.gray.opacity(0.2))
                            .foregroundColor(calibration.isStereo ? .blue : .gray)
                            .cornerRadius(4)
                        
                        Text(calibration.calibrationDate, style: .date)
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                }
                
                Spacer()
                
                Button(action: {
                    calibrationToDelete = calibration.cameraDeviceId
                    showingDeleteConfirmation = true
                }) {
                    Image(systemName: "trash")
                        .foregroundColor(.red)
                }
            }
            
            // Show transform details
            VStack(alignment: .leading, spacing: 4) {
                let t = calibration.leftHeadToCameraMatrix
                Text("T_head^camera translation: (\(String(format: "%.3f", t.columns.3.x)), \(String(format: "%.3f", t.columns.3.y)), \(String(format: "%.3f", t.columns.3.z))) m")
                    .font(.system(size: 10, design: .monospaced))
                    .foregroundColor(.purple)
                
                Text("Reproj error: \(String(format: "%.4f m", calibration.leftReprojectionError))")
                    .font(.caption)
                    .foregroundColor(calibration.leftReprojectionError < 0.01 ? .green : .orange)
            }
        }
        .padding()
        .background(Color.secondary.opacity(0.1))
        .cornerRadius(12)
    }
    
    private var markerConfigSection: some View {
        VStack(alignment: .leading, spacing: 12) {
            Text("Marker Settings")
                .font(.headline)
            
            VStack(spacing: 16) {
                // Marker size
                HStack {
                    Text("Marker Size")
                    Spacer()
                    Text("\(Int(calibrationManager.markerSizeMeters * 1000)) mm")
                        .foregroundColor(.secondary)
                        .monospacedDigit()
                }
                
                Slider(
                    value: Binding(
                        get: { calibrationManager.markerSizeMeters * 1000 },
                        set: { calibrationManager.markerSizeMeters = $0 / 1000 }
                    ),
                    in: 50...200,
                    step: 10
                )
                .tint(.purple)
                
                // Stereo baseline (for stereo cameras)
                VStack(spacing: 8) {
                    HStack {
                        Text("Stereo Baseline")
                        Spacer()
                        if let baseline = calibrationManager.knownStereoBaseline {
                            Text("\(Int(baseline * 1000)) mm")
                                .foregroundColor(.blue)
                                .monospacedDigit()
                        } else {
                            Text("Not set")
                                .foregroundColor(.secondary)
                        }
                    }
                    
                    Slider(
                        value: Binding(
                            get: { (calibrationManager.knownStereoBaseline ?? 0.065) * 1000 },
                            set: { calibrationManager.knownStereoBaseline = $0 / 1000 }
                        ),
                        in: 40...120,
                        step: 1
                    )
                    .tint(.blue)
                    
                    HStack {
                        Button(action: {
                            calibrationManager.knownStereoBaseline = nil
                        }) {
                            Text("Disable")
                                .font(.caption)
                                .padding(.horizontal, 8)
                                .padding(.vertical, 4)
                                .background(calibrationManager.knownStereoBaseline == nil ? Color.gray.opacity(0.3) : Color.gray.opacity(0.1))
                                .cornerRadius(4)
                        }
                        
                        Button(action: {
                            calibrationManager.knownStereoBaseline = 0.065  // 65mm default
                        }) {
                            Text("65mm")
                                .font(.caption)
                                .padding(.horizontal, 8)
                                .padding(.vertical, 4)
                                .background(calibrationManager.knownStereoBaseline == 0.065 ? Color.blue.opacity(0.3) : Color.blue.opacity(0.1))
                                .cornerRadius(4)
                        }
                        
                        Spacer()
                        
                        Text("For stereo cameras")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                }
                .padding()
                .background(Color.blue.opacity(0.05))
                .cornerRadius(8)
                
                // Sequential marker workflow info
                VStack(alignment: .leading, spacing: 8) {
                    Text("Sequential Workflow")
                        .font(.subheadline)
                        .fontWeight(.medium)
                    
                    HStack {
                        Text("Marker IDs")
                        Spacer()
                        Text(calibrationManager.markerIds.map { String($0) }.joined(separator: " → "))
                            .foregroundColor(.purple)
                    }
                    
                    HStack {
                        Text("Samples per position")
                        Spacer()
                        Text("\(calibrationManager.samplesPerPosition)")
                            .foregroundColor(.purple)
                    }
                    
                    HStack {
                        Text("Min positions needed")
                        Spacer()
                        Text("\(calibrationManager.minUniqueMarkers)")
                            .foregroundColor(.orange)
                    }
                }
                .font(.caption)
                .padding()
                .background(Color.purple.opacity(0.05))
                .cornerRadius(8)
            }
            .padding()
            .background(Color.secondary.opacity(0.1))
            .cornerRadius(12)
            
            // Instructions - updated for sequential workflow
            VStack(alignment: .leading, spacing: 8) {
                Text("📱 Calibration Workflow")
                    .font(.subheadline)
                    .fontWeight(.medium)
                
                VStack(alignment: .leading, spacing: 4) {
                    Text("1. Open iPhone Calibration tab → 'Display ArUco Marker'")
                    Text("2. iPhone shows marker ID 0 (55mm)")
                    Text("3. Look CLOSELY at marker until ARKit registers correct ID")
                    Text("4. Once registered, move away and view from different angles")
                    Text("5. After 20 samples → swipe iPhone to marker ID 2")
                    Text("6. MOVE iPhone to different location, repeat steps 3-4")
                    Text("7. Repeat for marker ID 3 at third location")
                    Text("8. Tap 'Finish' when all 3 markers completed")
                }
                
                Text("⚠️ Markers: 0, 2, 3 only (ID 1 removed)")
                    .font(.caption2)
                    .foregroundColor(.orange)
                    .padding(.top, 4)
            }
            .font(.caption)
            .foregroundColor(.secondary)
            .padding()
            .background(Color.secondary.opacity(0.1))
            .cornerRadius(12)
        }
    }
    
    // MARK: - Active Calibration View
    
    private var calibrationActiveView: some View {
        VStack(spacing: 20) {
            // Status header
            VStack(spacing: 8) {
                if calibrationManager.isCalibrating {
                    // Current marker indicator - prominent display
                    VStack(spacing: 12) {
                        Text("📱 Display on iPhone:")
                            .font(.headline)
                            .foregroundColor(.secondary)
                        
                        HStack(spacing: 20) {
                            ForEach(calibrationManager.markerIds, id: \.self) { markerId in
                                let isCurrent = markerId == calibrationManager.currentMarkerId
                                let samplesForMarker = calibrationManager.samples.filter { $0.markerId == markerId }.count
                                let isDone = samplesForMarker >= calibrationManager.samplesPerPosition
                                
                                VStack(spacing: 4) {
                                    Text("ID \(markerId)")
                                        .font(.system(size: isCurrent ? 24 : 16, weight: isCurrent ? .bold : .medium, design: .rounded))
                                        .foregroundColor(isCurrent ? .purple : (isDone ? .green : .secondary))
                                    
                                    if isDone {
                                        Image(systemName: "checkmark.circle.fill")
                                            .foregroundColor(.green)
                                    } else if isCurrent {
                                        Text("\(samplesForMarker)/\(calibrationManager.samplesPerPosition)")
                                            .font(.caption2)
                                            .foregroundColor(.purple)
                                    }
                                }
                                .frame(width: 60)
                                .padding(.vertical, 8)
                                .background(isCurrent ? Color.purple.opacity(0.2) : Color.clear)
                                .cornerRadius(8)
                            }
                        }
                        
                        Text(calibrationManager.statusMessage)
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                            .multilineTextAlignment(.center)
                            .padding(.top, 4)
                    }
                    .padding()
                    .background(Color.secondary.opacity(0.1))
                    .cornerRadius(16)
                    
                } else if let error = calibrationManager.lastError {
                    Image(systemName: "exclamationmark.triangle.fill")
                        .font(.largeTitle)
                        .foregroundColor(.red)
                    
                    Text("Error")
                        .font(.headline)
                    
                    Text(error)
                        .font(.subheadline)
                        .foregroundColor(.red)
                        .multilineTextAlignment(.center)
                }
            }
            .padding()
            
            // Overall progress
            VStack(spacing: 8) {
                ProgressView(value: calibrationManager.calibrationProgress)
                    .progressViewStyle(.linear)
                    .tint(.purple)
                
                let uniqueMarkers = Set(calibrationManager.samples.map { $0.markerId }).count
                
                if isStereoMode {
                    HStack {
                        Text("Left: \(calibrationManager.leftSamplesCollected)")
                        Spacer()
                        Text("Right: \(calibrationManager.rightSamplesCollected)")
                        Spacer()
                        Text("Markers: \(uniqueMarkers)/\(calibrationManager.minUniqueMarkers)")
                    }
                    .font(.caption)
                    .foregroundColor(.secondary)
                } else {
                    Text("\(calibrationManager.samplesCollected) samples • \(uniqueMarkers) markers")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            .padding(.horizontal)
            
            // Detection status
            VStack(alignment: .leading, spacing: 8) {
                // Target marker
                HStack {
                    Text("Target Marker")
                        .font(.caption)
                        .foregroundColor(.secondary)
                    Spacer()
                    Text("ID \(calibrationManager.currentMarkerId)")
                        .font(.caption)
                        .fontWeight(.bold)
                        .foregroundColor(.purple)
                }
                
                // ARKit tracking status
                HStack {
                    Text("ARKit Sees")
                        .font(.caption)
                        .foregroundColor(.secondary)
                    Spacer()
                    if calibrationManager.arkitTrackedMarkers.isEmpty && calibrationManager.rememberedMarkerPositions.isEmpty {
                        Text("No markers detected")
                            .font(.caption)
                            .foregroundColor(.orange)
                    } else {
                        let active = calibrationManager.arkitTrackedMarkers.keys.sorted()
                        let hasTarget = calibrationManager.rememberedMarkerPositions[calibrationManager.currentMarkerId] != nil || calibrationManager.arkitTrackedMarkers[calibrationManager.currentMarkerId] != nil
                        HStack(spacing: 4) {
                            if hasTarget {
                                Image(systemName: "checkmark.circle.fill")
                                    .foregroundColor(.green)
                                    .font(.caption)
                            }
                            Text(active.isEmpty ? "remembered" : active.map { String($0) }.joined(separator: ", "))
                                .font(.caption)
                                .foregroundColor(hasTarget ? .green : .blue)
                        }
                    }
                }
                
                // Camera detection status
                HStack {
                    Text("Camera Sees")
                        .font(.caption)
                        .foregroundColor(.secondary)
                    Spacer()
                    if calibrationManager.cameraDetectedMarkers.isEmpty {
                        Text("No markers detected")
                            .font(.caption)
                            .foregroundColor(.orange)
                    } else {
                        let detected = calibrationManager.cameraDetectedMarkers.keys.sorted()
                        let hasTarget = calibrationManager.cameraDetectedMarkers[calibrationManager.currentMarkerId] != nil
                        HStack(spacing: 4) {
                            if hasTarget {
                                Image(systemName: "checkmark.circle.fill")
                                    .foregroundColor(.green)
                                    .font(.caption)
                            }
                            Text(detected.map { String($0) }.joined(separator: ", "))
                                .font(.caption)
                                .foregroundColor(hasTarget ? .green : .purple)
                        }
                    }
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding()
            .background(Color.secondary.opacity(0.1))
            .cornerRadius(12)
            .padding(.horizontal)
            
            // Dynamic user guidance
            VStack(spacing: 8) {
                let arkitHasTarget = calibrationManager.rememberedMarkerPositions[calibrationManager.currentMarkerId] != nil || calibrationManager.arkitTrackedMarkers[calibrationManager.currentMarkerId] != nil
                let cameraHasTarget = calibrationManager.cameraDetectedMarkers[calibrationManager.currentMarkerId] != nil
                let samplesForCurrent = calibrationManager.samplesForCurrentMarker
                
                if !arkitHasTarget {
                    // Step 1: ARKit needs to register the marker
                    HStack(spacing: 8) {
                        Image(systemName: "eye.fill")
                            .font(.title3)
                            .foregroundColor(.orange)
                        VStack(alignment: .leading, spacing: 2) {
                            Text("👀 Look closely at marker ID \(calibrationManager.currentMarkerId)")
                                .font(.subheadline)
                                .fontWeight(.semibold)
                            Text("Move closer until ARKit registers the marker")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)
                    .padding()
                    .background(Color.orange.opacity(0.15))
                    .cornerRadius(10)
                } else if samplesForCurrent < calibrationManager.samplesPerPosition {
                    // Step 2: Collecting samples
                    HStack(spacing: 8) {
                        Image(systemName: "move.3d")
                            .font(.title3)
                            .foregroundColor(.blue)
                        VStack(alignment: .leading, spacing: 2) {
                            Text("📐 Move away & view from different angles")
                                .font(.subheadline)
                                .fontWeight(.semibold)
                            Text("Collecting: \(samplesForCurrent)/\(calibrationManager.samplesPerPosition) samples")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)
                    .padding()
                    .background(Color.blue.opacity(0.15))
                    .cornerRadius(10)
                } else if calibrationManager.canAdvanceToNextMarker {
                    // Step 3: Ready to advance
                    HStack(spacing: 8) {
                        Image(systemName: "arrow.right.circle.fill")
                            .font(.title3)
                            .foregroundColor(.green)
                        VStack(alignment: .leading, spacing: 2) {
                            Text("✅ Position complete!")
                                .font(.subheadline)
                                .fontWeight(.semibold)
                            Text("Swipe iPhone to next marker & move to new location")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)
                    .padding()
                    .background(Color.green.opacity(0.15))
                    .cornerRadius(10)
                } else {
                    // Final step: Ready to finish
                    HStack(spacing: 8) {
                        Image(systemName: "checkmark.circle.fill")
                            .font(.title3)
                            .foregroundColor(.green)
                        VStack(alignment: .leading, spacing: 2) {
                            Text("🎉 All markers complete!")
                                .font(.subheadline)
                                .fontWeight(.semibold)
                            Text("Tap 'Finish Calibration' below")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)
                    .padding()
                    .background(Color.green.opacity(0.15))
                    .cornerRadius(10)
                }
            }
            .padding(.horizontal)
            
            Spacer()
            
            // Action buttons
            VStack(spacing: 12) {
                // Proceed button (when current position is done, waiting for user to advance)
                if calibrationManager.hasEnoughSamplesForCurrentMarker && !showingProceedDialog {
                    if calibrationManager.canAdvanceToNextMarker {
                        Button(action: {
                            showingProceedDialog = true
                        }) {
                            HStack {
                                Image(systemName: "arrow.right.circle.fill")
                                Text("Proceed to Next Marker")
                            }
                            .font(.headline)
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity)
                            .padding()
                            .background(Color.green)
                            .cornerRadius(12)
                        }
                    } else {
                        // All markers done - automatically finish
                        Button(action: {
                            Task {
                                await autoFinishCalibration()
                            }
                        }) {
                            HStack {
                                if isProcessing {
                                    ProgressView()
                                        .progressViewStyle(CircularProgressViewStyle(tint: .white))
                                } else {
                                    Image(systemName: "checkmark.circle.fill")
                                }
                                Text(isProcessing ? "Optimizing..." : "Complete!")
                            }
                            .font(.headline)
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity)
                            .padding()
                            .background(Color.green)
                            .cornerRadius(12)
                        }
                        .disabled(isProcessing)
                    }
                }
                
                if !isProcessing {
                    Button(action: {
                        calibrationManager.cancelCalibration()
                        isCalibrating = false
                    }) {
                        Text("Cancel")
                            .font(.headline)
                            .foregroundColor(.red)
                            .frame(maxWidth: .infinity)
                            .padding()
                            .background(Color.red.opacity(0.1))
                            .cornerRadius(12)
                    }
                }
            }
            .padding()
            .alert("Move to Next Position", isPresented: $showingProceedDialog) {
                Button("Cancel", role: .cancel) {}
                Button("Ready - Proceed") {
                    // Advance to next marker and reset ARKit tracking
                    calibrationManager.advanceToNextMarker()
                    // Clear remembered positions to force re-registration
                    calibrationManager.rememberedMarkerPositions.removeAll()
                }
            } message: {
                Text("Swipe iPhone to show marker ID \(calibrationManager.markerIds[calibrationManager.currentMarkerIndex + 1])\n\nMove iPhone to a DIFFERENT location\n\nThen press 'Ready' to start collecting 20 more samples")
            }
        }
    }
    
    private var canFinish: Bool {
        // Need minimum unique markers and total samples
        let uniqueMarkers = Set(calibrationManager.samples.map { $0.markerId }).count
        
        if isStereoMode {
            return uniqueMarkers >= calibrationManager.minUniqueMarkers &&
                   calibrationManager.leftSamplesCollected >= calibrationManager.minSamples &&
                   calibrationManager.rightSamplesCollected >= calibrationManager.minSamples
        } else {
            return uniqueMarkers >= calibrationManager.minUniqueMarkers &&
                   calibrationManager.samplesCollected >= calibrationManager.minSamples
        }
    }
    
    // MARK: - Frame Processing
    
    private func setupFrameCallback() {
        dlog("📐 [ExtrinsicCalibrationView] Setting up frame callback...")
        
        uvcCameraManager.onPixelBufferReceived = { [self] pixelBuffer in
            guard calibrationManager.isCalibrating else { return }
            
            // Get intrinsics for the selected device
            guard let deviceId = selectedDeviceId,
                  let intrinsicData = intrinsicCalibrationManager.allCalibrations[deviceId] else {
                return
            }
            
            let leftIntrinsics = intrinsicData.leftIntrinsics
            let rightIntrinsics = intrinsicData.rightIntrinsics
            
            // Process frame
            calibrationManager.processCameraFrame(
                pixelBuffer,
                intrinsics: leftIntrinsics,
                rightIntrinsics: rightIntrinsics
            )
        }
    }
    
    // MARK: - Calibration Actions
    
    private func startCalibration(for device: UVCDevice) {
        dlog("📐 [ExtrinsicCalibrationView] ========== START CALIBRATION ==========")
        dlog("📐 [ExtrinsicCalibrationView] Device: \(device.name)")
        dlog("📐 [ExtrinsicCalibrationView] Stereo mode: \(isStereoMode)")
        
        // Select device and start capture if needed
        if uvcCameraManager.selectedDevice?.id != device.id {
            uvcCameraManager.selectDevice(device)
        }
        
        if !uvcCameraManager.isCapturing {
            uvcCameraManager.startCapture()
        }
        
        // Start calibration asynchronously
        Task {
            await calibrationManager.startCalibration(
                deviceId: device.id,
                deviceName: device.name,
                isStereo: isStereoMode
            )
        }
        
        isCalibrating = true
        setupFrameCallback()
        
        dlog("📐 [ExtrinsicCalibrationView] ========== CALIBRATION STARTED ==========")
    }
    
    private func finishCalibration() {
        guard let deviceId = selectedDeviceId,
              let device = uvcCameraManager.availableDevices.first(where: { $0.id == deviceId }) else {
            return
        }
        
        let result = calibrationManager.finishCalibration(
            deviceId: deviceId,
            deviceName: device.name,
            isStereo: isStereoMode
        )
        
        isCalibrating = false
        
        if result != nil {
            dlog("✅ [ExtrinsicCalibrationView] Calibration complete, dismissing...")
            onDismiss?()
        }
    }
    
    private func autoFinishCalibration() async {
        guard let deviceId = selectedDeviceId,
              let device = uvcCameraManager.availableDevices.first(where: { $0.id == deviceId }) else {
            return
        }
        
        isProcessing = true
        
        // Small delay to show processing state
        try? await Task.sleep(nanoseconds: 500_000_000) // 0.5 seconds
        
        let result = calibrationManager.finishCalibration(
            deviceId: deviceId,
            deviceName: device.name,
            isStereo: isStereoMode
        )
        
        isProcessing = false
        isCalibrating = false
        
        if result != nil {
            dlog("✅ [ExtrinsicCalibrationView] Auto-finish complete, dismissing...")
            // Dismiss after showing success briefly
            try? await Task.sleep(nanoseconds: 500_000_000)
            onDismiss?()
        }
    }
}
