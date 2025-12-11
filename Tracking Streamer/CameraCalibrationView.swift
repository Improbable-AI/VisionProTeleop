import SwiftUI
import AVFoundation

/// View for camera intrinsic calibration
struct CameraCalibrationView: View {
    @StateObject private var calibrationManager = CameraCalibrationManager.shared
    @StateObject private var uvcCameraManager = UVCCameraManager.shared
    
    // Dismiss callback (for overlay mode)
    var onDismiss: (() -> Void)? = nil
    
    // Calibration state
    @State private var isCalibrating = false
    @State private var showingDeleteConfirmation = false
    @State private var calibrationToDelete: String? = nil
    @State private var showingExportSheet = false
    @State private var exportData: Data? = nil
    
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
            .navigationTitle("Camera Calibration")
            .navigationBarTitleDisplayMode(.inline)
            .onAppear {
                dlog("📱 [CalibrationView] View appeared!")
                dlog("📱 [CalibrationView] Available devices: \(uvcCameraManager.availableDevices.map { $0.name })")
                dlog("📱 [CalibrationView] isCalibrating: \(isCalibrating)")
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
                
                // Checkerboard config
                checkerboardConfigSection
            }
            .padding()
        }
    }
    
    private var infoHeaderSection: some View {
        VStack(spacing: 12) {
            Image(systemName: "camera.viewfinder")
                .font(.system(size: 48))
                .foregroundColor(.blue)
            
            Text("Intrinsic Calibration")
                .font(.title2)
                .fontWeight(.bold)
            
            Text("Calibrate your camera's intrinsic parameters (focal length, principal point, distortion) for accurate pose estimation.")
                .font(.subheadline)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
                .padding(.horizontal)
        }
        .padding()
        .background(Color.blue.opacity(0.1))
        .cornerRadius(16)
    }
    
    private var availableCamerasSection: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Image(systemName: "video.fill")
                    .foregroundColor(.green)
                Text("Available Cameras")
                    .font(.headline)
            }
            
            if uvcCameraManager.availableDevices.isEmpty {
                HStack {
                    Image(systemName: "exclamationmark.triangle.fill")
                        .foregroundColor(.orange)
                    Text("No USB cameras detected")
                        .foregroundColor(.secondary)
                }
                .padding()
                .frame(maxWidth: .infinity)
                .background(Color.orange.opacity(0.1))
                .cornerRadius(12)
            } else {
                ForEach(uvcCameraManager.availableDevices) { device in
                    cameraRow(device: device)
                }
            }
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(16)
    }
    
    private func cameraRow(device: UVCDevice) -> some View {
        let hasCalibration = calibrationManager.hasCalibration(for: device.id)
        
        return VStack(spacing: 8) {
            HStack {
                VStack(alignment: .leading, spacing: 4) {
                    Text(device.name)
                        .font(.body)
                        .fontWeight(.medium)
                    
                    if hasCalibration {
                        HStack(spacing: 4) {
                            Image(systemName: "checkmark.circle.fill")
                                .foregroundColor(.green)
                                .font(.caption)
                            Text("Calibrated")
                                .font(.caption)
                                .foregroundColor(.green)
                        }
                    } else {
                        HStack(spacing: 4) {
                            Image(systemName: "exclamationmark.circle.fill")
                                .foregroundColor(.orange)
                                .font(.caption)
                            Text("Not calibrated")
                                .font(.caption)
                                .foregroundColor(.orange)
                        }
                    }
                }
                
                Spacer()
                
                Button {
                    dlog("🔘 [CalibrationView] Calibrate button tapped for device: \(device.name)")
                    startCalibrationFor(device: device)
                } label: {
                    Text(hasCalibration ? "Recalibrate" : "Calibrate")
                        .font(.subheadline)
                        .fontWeight(.medium)
                        .padding(.horizontal, 16)
                        .padding(.vertical, 8)
                        .background(Color.blue)
                        .foregroundColor(.white)
                        .cornerRadius(8)
                }
                .buttonStyle(.plain)
            }
            
            // Stereo mode toggle
            Toggle(isOn: Binding(
                get: { selectedDeviceId == device.id && isStereoMode },
                set: { newValue in
                    selectedDeviceId = device.id
                    isStereoMode = newValue
                }
            )) {
                Text("Stereo Camera (side-by-side)")
                    .font(.caption)
            }
            .toggleStyle(SwitchToggleStyle(tint: .blue))
        }
        .padding()
        .background(Color(.tertiarySystemBackground))
        .cornerRadius(12)
    }
    
    private var savedCalibrationsSection: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Image(systemName: "doc.fill")
                    .foregroundColor(.purple)
                Text("Saved Calibrations")
                    .font(.headline)
            }
            
            if calibrationManager.allCalibrations.isEmpty {
                Text("No calibrations saved yet")
                    .foregroundColor(.secondary)
                    .padding()
                    .frame(maxWidth: .infinity)
            } else {
                ForEach(Array(calibrationManager.allCalibrations.values).sorted(by: { $0.calibrationDate > $1.calibrationDate }), id: \.deviceId) { calibration in
                    savedCalibrationRow(calibration: calibration)
                }
            }
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(16)
    }
    
    private func savedCalibrationRow(calibration: CameraCalibrationData) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                VStack(alignment: .leading, spacing: 4) {
                    Text(calibration.deviceName)
                        .font(.body)
                        .fontWeight(.medium)
                    
                    HStack(spacing: 8) {
                        Text(calibration.isStereo ? "Stereo" : "Mono")
                            .font(.caption)
                            .padding(.horizontal, 6)
                            .padding(.vertical, 2)
                            .background(calibration.isStereo ? Color.blue.opacity(0.2) : Color.gray.opacity(0.2))
                            .cornerRadius(4)
                        
                        Text("\(calibration.sampleCount) samples")
                            .font(.caption)
                            .foregroundColor(.secondary)
                        
                        Text(calibration.calibrationDate, style: .date)
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                }
                
                Spacer()
                
                HStack(spacing: 8) {
                    Button {
                        showingDeleteConfirmation = true
                        calibrationToDelete = calibration.deviceId
                    } label: {
                        Image(systemName: "trash")
                            .foregroundColor(.red)
                    }
                    .buttonStyle(.plain)
                }
            }
            
            // Calibration details
            VStack(alignment: .leading, spacing: 4) {
                let fx = calibration.leftIntrinsics.fx
                let fy = calibration.leftIntrinsics.fy
                let cx = calibration.leftIntrinsics.cx
                let cy = calibration.leftIntrinsics.cy
                let error = calibration.leftIntrinsics.reprojectionError
                
                Text("fx: \(String(format: "%.1f", fx)), fy: \(String(format: "%.1f", fy))")
                    .font(.caption2)
                    .foregroundColor(.secondary)
                Text("cx: \(String(format: "%.1f", cx)), cy: \(String(format: "%.1f", cy))")
                    .font(.caption2)
                    .foregroundColor(.secondary)
                Text("Reproj. Error: \(String(format: "%.4f", error))")
                    .font(.caption2)
                    .foregroundColor(error < 0.5 ? .green : (error < 1.0 ? .orange : .red))
            }
        }
        .padding()
        .background(Color(.tertiarySystemBackground))
        .cornerRadius(12)
    }
    
    private var checkerboardConfigSection: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Image(systemName: "checkerboard.rectangle")
                    .foregroundColor(.orange)
                Text("Checkerboard Settings")
                    .font(.headline)
            }
            
            VStack(spacing: 16) {
                HStack {
                    Text("Inner Corners X")
                        .font(.subheadline)
                    Spacer()
                    Stepper(value: Binding(
                        get: { calibrationManager.checkerboardConfig.innerCornersX },
                        set: { calibrationManager.checkerboardConfig = CheckerboardConfig(
                            innerCornersX: $0,
                            innerCornersY: calibrationManager.checkerboardConfig.innerCornersY,
                            squareSize: calibrationManager.checkerboardConfig.squareSize
                        )}
                    ), in: 4...15) {
                        Text("\(calibrationManager.checkerboardConfig.innerCornersX)")
                            .fontWeight(.medium)
                            .frame(width: 30)
                    }
                }
                
                HStack {
                    Text("Inner Corners Y")
                        .font(.subheadline)
                    Spacer()
                    Stepper(value: Binding(
                        get: { calibrationManager.checkerboardConfig.innerCornersY },
                        set: { calibrationManager.checkerboardConfig = CheckerboardConfig(
                            innerCornersX: calibrationManager.checkerboardConfig.innerCornersX,
                            innerCornersY: $0,
                            squareSize: calibrationManager.checkerboardConfig.squareSize
                        )}
                    ), in: 4...15) {
                        Text("\(calibrationManager.checkerboardConfig.innerCornersY)")
                            .fontWeight(.medium)
                            .frame(width: 30)
                    }
                }
                
                HStack {
                    Text("Square Size (mm)")
                        .font(.subheadline)
                    Spacer()
                    TextField("24", value: Binding(
                        get: { calibrationManager.checkerboardConfig.squareSize * 1000 },
                        set: { calibrationManager.checkerboardConfig = CheckerboardConfig(
                            innerCornersX: calibrationManager.checkerboardConfig.innerCornersX,
                            innerCornersY: calibrationManager.checkerboardConfig.innerCornersY,
                            squareSize: Float($0) / 1000
                        )}
                    ), format: .number)
                    .keyboardType(.decimalPad)
                    .frame(width: 60)
                    .multilineTextAlignment(.trailing)
                    .textFieldStyle(.roundedBorder)
                }
            }
            
            Text("Default: 9×6 inner corners with 24mm squares")
                .font(.caption)
                .foregroundColor(.secondary)
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(16)
    }
    
    // MARK: - Active Calibration View
    
    private var calibrationActiveView: some View {
        VStack(spacing: 20) {
            // Progress header
            VStack(spacing: 8) {
                HStack {
                    Text("Calibrating: \(selectedDeviceId ?? "Unknown")")
                        .font(.headline)
                    Spacer()
                    Text("\(calibrationManager.samplesCollected)/\(calibrationManager.minSamples)")
                        .font(.headline)
                        .foregroundColor(.blue)
                }
                
                ProgressView(value: calibrationManager.calibrationProgress)
                    .progressViewStyle(LinearProgressViewStyle(tint: .blue))
                
                Text(calibrationManager.statusMessage)
                    .font(.subheadline)
                    .foregroundColor(.secondary)
            }
            .padding()
            .background(Color(.secondarySystemBackground))
            .cornerRadius(12)
            
            // Camera preview would go here
            // In VisionOS, we'd need to use RealityKit or a different approach
            // For now, show instructions
            calibrationInstructionsView
            
            Spacer()
            
            // Action buttons
            HStack(spacing: 16) {
                Button {
                    calibrationManager.cancelCalibration()
                    isCalibrating = false
                } label: {
                    Text("Cancel")
                        .fontWeight(.medium)
                        .frame(maxWidth: .infinity)
                        .padding()
                        .background(Color.gray.opacity(0.2))
                        .foregroundColor(.primary)
                        .cornerRadius(12)
                }
                .buttonStyle(.plain)
                
                if calibrationManager.samplesCollected >= calibrationManager.minSamples {
                    Button {
                        finishCalibration()
                    } label: {
                        Text("Finish")
                            .fontWeight(.medium)
                            .frame(maxWidth: .infinity)
                            .padding()
                            .background(Color.green)
                            .foregroundColor(.white)
                            .cornerRadius(12)
                    }
                    .buttonStyle(.plain)
                }
            }
        }
        .padding()
    }
    
    private var calibrationInstructionsView: some View {
        VStack(spacing: 16) {
            Image(systemName: "checkerboard.rectangle")
                .font(.system(size: 64))
                .foregroundColor(.orange)
            
            Text("Move the checkerboard around")
                .font(.title3)
                .fontWeight(.medium)
            
            VStack(alignment: .leading, spacing: 8) {
                instructionRow(number: 1, text: "Hold the checkerboard in view of the camera")
                instructionRow(number: 2, text: "Move it to different positions and angles")
                instructionRow(number: 3, text: "Cover the entire field of view")
                instructionRow(number: 4, text: "Samples are captured automatically")
            }
            .padding()
            
            if let error = calibrationManager.lastError {
                Text(error)
                    .font(.caption)
                    .foregroundColor(.red)
                    .padding()
            }
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(16)
    }
    
    private func instructionRow(number: Int, text: String) -> some View {
        HStack(alignment: .top, spacing: 12) {
            Text("\(number)")
                .font(.caption)
                .fontWeight(.bold)
                .foregroundColor(.white)
                .frame(width: 20, height: 20)
                .background(Color.blue)
                .clipShape(Circle())
            
            Text(text)
                .font(.subheadline)
        }
    }
    
    // MARK: - Actions
    
    private func setupFrameCallback() {
        dlog("🔧 [CalibrationView] Setting up frame callback...")
        dlog("🔧 [CalibrationView] uvcCameraManager.isCapturing: \(uvcCameraManager.isCapturing)")
        dlog("🔧 [CalibrationView] uvcCameraManager.selectedDevice: \(uvcCameraManager.selectedDevice?.name ?? "nil")")
        
        uvcCameraManager.onPixelBufferReceived = { [weak calibrationManager] pixelBuffer in
            dlog("📹 [CalibrationView] Received pixel buffer: \(CVPixelBufferGetWidth(pixelBuffer))x\(CVPixelBufferGetHeight(pixelBuffer))")
            
            guard let manager = calibrationManager else {
                dlog("❌ [CalibrationView] calibrationManager is nil!")
                return
            }
            
            guard manager.isCalibrating else {
                dlog("⏸️ [CalibrationView] Not calibrating, skipping frame")
                return
            }
            
            dlog("🔄 [CalibrationView] Processing frame for calibration (stereo: \(self.isStereoMode))")
            
            Task { @MainActor in
                // Process frame based on stereo/mono mode
                if self.isStereoMode {
                    let detection = manager.processStereoFrame(pixelBuffer)
                    dlog("🔍 [CalibrationView] Stereo detection: left=\(detection?.foundLeft ?? false), right=\(detection?.foundRight ?? false)")
                } else {
                    let detection = manager.processMonoFrame(pixelBuffer)
                    dlog("🔍 [CalibrationView] Mono detection: found=\(detection?.foundLeft ?? false)")
                }
            }
        }
        
        dlog("✅ [CalibrationView] Frame callback set up")
    }
    
    private func startCalibrationFor(device: UVCDevice) {
        dlog("🚀 [CalibrationView] ========== START CALIBRATION ==========")
        dlog("🚀 [CalibrationView] Device: \(device.name) (id: \(device.id))")
        dlog("🚀 [CalibrationView] isStereoMode: \(isStereoMode)")
        dlog("🚀 [CalibrationView] Current state - isCapturing: \(uvcCameraManager.isCapturing)")
        dlog("🚀 [CalibrationView] Current state - selectedDevice: \(uvcCameraManager.selectedDevice?.name ?? "nil")")
        
        selectedDeviceId = device.id
        
        // Select and start the camera if not already capturing
        if uvcCameraManager.selectedDevice?.id != device.id {
            dlog("🔄 [CalibrationView] Selecting device...")
            uvcCameraManager.selectDevice(device)
        } else {
            dlog("✅ [CalibrationView] Device already selected")
        }
        
        if !uvcCameraManager.isCapturing {
            dlog("▶️ [CalibrationView] Starting capture...")
            uvcCameraManager.startCapture()
        } else {
            dlog("✅ [CalibrationView] Already capturing")
        }
        
        dlog("🎯 [CalibrationView] Calling calibrationManager.startCalibration...")
        
        // Start calibration
        calibrationManager.startCalibration(
            deviceId: device.id,
            deviceName: device.name,
            isStereo: isStereoMode
        )
        
        dlog("📊 [CalibrationView] After startCalibration - isCalibrating: \(calibrationManager.isCalibrating)")
        
        isCalibrating = true
        dlog("✅ [CalibrationView] Local isCalibrating set to true")
        
        // Re-setup frame callback to ensure it's active
        dlog("🔧 [CalibrationView] Re-setting up frame callback...")
        setupFrameCallback()
        
        dlog("🚀 [CalibrationView] ========== START CALIBRATION COMPLETE ==========")
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
        
        // If calibration was successful, dismiss the view
        if result != nil {
            dlog("✅ [CalibrationView] Calibration complete, dismissing...")
            onDismiss?()
        }
    }
}

// MARK: - Calibration Status Badge

/// Small badge showing calibration status for a camera
struct CalibrationStatusBadge: View {
    let deviceId: String
    @StateObject private var calibrationManager = CameraCalibrationManager.shared
    
    var body: some View {
        let hasCalibration = calibrationManager.hasCalibration(for: deviceId)
        
        HStack(spacing: 4) {
            Image(systemName: hasCalibration ? "checkmark.circle.fill" : "exclamationmark.circle.fill")
                .font(.system(size: 10))
            Text(hasCalibration ? "Calibrated" : "Not Calibrated")
                .font(.caption2)
        }
        .foregroundColor(hasCalibration ? .green : .orange)
        .padding(.horizontal, 6)
        .padding(.vertical, 2)
        .background((hasCalibration ? Color.green : Color.orange).opacity(0.2))
        .cornerRadius(4)
    }
}

// MARK: - Calibration Warning Banner

/// Banner shown when camera is not calibrated
struct CalibrationWarningBanner: View {
    let deviceId: String
    let deviceName: String
    @Binding var showCalibrationSheet: Bool
    @StateObject private var calibrationManager = CameraCalibrationManager.shared
    
    var body: some View {
        if !calibrationManager.hasCalibration(for: deviceId) {
            HStack(spacing: 12) {
                Image(systemName: "exclamationmark.triangle.fill")
                    .foregroundColor(.orange)
                
                VStack(alignment: .leading, spacing: 2) {
                    Text("Camera Not Calibrated")
                        .font(.subheadline)
                        .fontWeight(.medium)
                    Text("Intrinsic calibration recommended for accurate recordings")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                
                Spacer()
                
                Button {
                    showCalibrationSheet = true
                } label: {
                    Text("Calibrate")
                        .font(.caption)
                        .fontWeight(.medium)
                        .padding(.horizontal, 12)
                        .padding(.vertical, 6)
                        .background(Color.orange)
                        .foregroundColor(.white)
                        .cornerRadius(6)
                }
                .buttonStyle(.plain)
            }
            .padding()
            .background(Color.orange.opacity(0.1))
            .cornerRadius(12)
        }
    }
}

#Preview {
    CameraCalibrationView()
}
