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
                print("📐 [ExtrinsicCalibrationView] View appeared!")
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
                
                // Marker IDs info
                HStack {
                    Text("Marker IDs")
                    Spacer()
                    Text(calibrationManager.markerIds.map { String($0) }.joined(separator: ", "))
                        .foregroundColor(.secondary)
                }
                
                // Min samples
                HStack {
                    Text("Min Samples")
                    Spacer()
                    Text("\(calibrationManager.minSamples)")
                        .foregroundColor(.secondary)
                }
            }
            .padding()
            .background(Color.secondary.opacity(0.1))
            .cornerRadius(12)
            
            // Instructions
            VStack(alignment: .leading, spacing: 8) {
                Text("Instructions")
                    .font(.subheadline)
                    .fontWeight(.medium)
                
                Text("1. Print ArUco markers using: python utils/generate_aruco_markers.py")
                Text("2. Place markers in your environment (flat surface, good lighting)")
                Text("3. Point camera at markers while moving your head")
                Text("4. Collect samples from different viewpoints")
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
                    ProgressView()
                        .scaleEffect(1.5)
                    
                    Text("Calibrating...")
                        .font(.headline)
                    
                    Text(calibrationManager.statusMessage)
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                        .multilineTextAlignment(.center)
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
            
            // Progress
            VStack(spacing: 8) {
                ProgressView(value: calibrationManager.calibrationProgress)
                    .progressViewStyle(.linear)
                    .tint(.purple)
                
                if isStereoMode {
                    HStack {
                        Text("Left: \(calibrationManager.leftSamplesCollected)/\(calibrationManager.minSamples)")
                        Spacer()
                        Text("Right: \(calibrationManager.rightSamplesCollected)/\(calibrationManager.minSamples)")
                    }
                    .font(.caption)
                    .foregroundColor(.secondary)
                } else {
                    Text("\(calibrationManager.samplesCollected) / \(calibrationManager.minSamples) samples")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            .padding(.horizontal)
            
            // Marker detection status
            VStack(alignment: .leading, spacing: 8) {
                Text("ARKit Tracked Markers")
                    .font(.caption)
                    .foregroundColor(.secondary)
                
                if calibrationManager.arkitTrackedMarkers.isEmpty {
                    Text("None detected - point at ArUco markers")
                        .font(.caption)
                        .foregroundColor(.orange)
                } else {
                    Text("IDs: \(calibrationManager.arkitTrackedMarkers.keys.sorted().map { String($0) }.joined(separator: ", "))")
                        .font(.caption)
                        .foregroundColor(.green)
                }
                
                Text("Camera Detected Markers")
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .padding(.top, 4)
                
                if calibrationManager.cameraDetectedMarkers.isEmpty {
                    Text("None detected")
                        .font(.caption)
                        .foregroundColor(.orange)
                } else {
                    Text("IDs: \(calibrationManager.cameraDetectedMarkers.keys.sorted().map { String($0) }.joined(separator: ", "))")
                        .font(.caption)
                        .foregroundColor(.green)
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding()
            .background(Color.secondary.opacity(0.1))
            .cornerRadius(12)
            .padding(.horizontal)
            
            Spacer()
            
            // Action buttons
            HStack(spacing: 16) {
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
                
                Button(action: {
                    finishCalibration()
                }) {
                    Text("Finish")
                        .font(.headline)
                        .foregroundColor(.white)
                        .frame(maxWidth: .infinity)
                        .padding()
                        .background(canFinish ? Color.purple : Color.gray)
                        .cornerRadius(12)
                }
                .disabled(!canFinish)
            }
            .padding()
        }
    }
    
    private var canFinish: Bool {
        if isStereoMode {
            return calibrationManager.leftSamplesCollected >= calibrationManager.minSamples &&
                   calibrationManager.rightSamplesCollected >= calibrationManager.minSamples
        } else {
            return calibrationManager.samplesCollected >= calibrationManager.minSamples
        }
    }
    
    // MARK: - Frame Processing
    
    private func setupFrameCallback() {
        print("📐 [ExtrinsicCalibrationView] Setting up frame callback...")
        
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
        print("📐 [ExtrinsicCalibrationView] ========== START CALIBRATION ==========")
        print("📐 [ExtrinsicCalibrationView] Device: \(device.name)")
        print("📐 [ExtrinsicCalibrationView] Stereo mode: \(isStereoMode)")
        
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
        
        print("📐 [ExtrinsicCalibrationView] ========== CALIBRATION STARTED ==========")
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
            print("✅ [ExtrinsicCalibrationView] Calibration complete, dismissing...")
            onDismiss?()
        }
    }
}
