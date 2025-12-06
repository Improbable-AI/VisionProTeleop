import SwiftUI

// MARK: - Clean Extrinsic Calibration View

struct ExtrinsicCalibrationViewNew: View {
    @StateObject private var calibrationManager = ExtrinsicCalibrationManager.shared
    @StateObject private var cameraCalibrationManager = CameraCalibrationManager.shared
    @StateObject private var uvcCameraManager = UVCCameraManager.shared
    @StateObject private var multipeerManager = MultipeerCalibrationManager.shared
    
    var onDismiss: (() -> Void)? = nil
    
    @State private var isCalibrating = false
    @State private var selectedDeviceId: String? = nil
    @State private var isStereoMode: Bool = false
    @State private var showingProceedDialog = false
    @State private var isProcessing = false
    @State private var showingDeleteConfirmation = false
    @State private var calibrationToDelete: String? = nil
    @State private var autoCollectWhenStill = true
    @State private var autoAdvanceOnStill = true  // Auto-proceed to next marker when phone moves and becomes still
    @State private var wasPhoneMoving = false     // Track if phone was previously moving
    
    var body: some View {
        NavigationStack {
            Group {
                if isCalibrating {
                    calibrationInProgressView
                } else {
                    calibrationSetupView
                }
            }
            .navigationTitle("Extrinsic Calibration")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button(action: { onDismiss?() }) {
                        Image(systemName: "xmark.circle.fill")
                            .foregroundColor(.secondary)
                    }
                }
            }
            .onAppear {
                setupFrameCallback()
                setupMultipeerCallbacks()
                multipeerManager.startBrowsing()
            }
            .onDisappear {
                uvcCameraManager.onPixelBufferReceived = nil
                multipeerManager.stopBrowsing()
            }
        }
    }
    
    // MARK: - Setup View
    
    private var calibrationSetupView: some View {
        ScrollView {
            VStack(spacing: 20) {
                // iPhone Connection Status
                iPhoneConnectionCard
                
                // Instructions
                instructionsCard
                
                // Camera Selection
                cameraSelectionCard
                
                // Camera Mode (Mono/Stereo)
                if selectedDeviceId != nil {
                    cameraModeCard
                }
                
                // Stereo Baseline (if stereo mode)
                if isStereoMode {
                    stereoBaselineCard
                }
                
                // Saved Calibrations
                savedCalibrationsCard
                
                // Start Button
                if selectedDeviceId != nil {
                    startButton
                }
            }
            .padding()
        }
        .background(Color(.systemGroupedBackground))
    }
    
    private var iPhoneConnectionCard: some View {
        VStack(alignment: .leading, spacing: 12) {
            Label("iPhone Connection", systemImage: "iphone.radiowaves.left.and.right")
                .font(.headline)
            
            HStack {
                // Connection status indicator
                Circle()
                    .fill(multipeerManager.isConnected ? Color.green : Color.orange)
                    .frame(width: 12, height: 12)
                
                VStack(alignment: .leading, spacing: 2) {
                    Text(multipeerManager.connectionStatus)
                        .font(.subheadline)
                        .fontWeight(.medium)
                    
                    if multipeerManager.isConnected {
                        Text("Auto marker switching enabled")
                            .font(.caption)
                            .foregroundColor(.green)
                    } else {
                        Text("Open iPhone app → Calibration → Display ArUco")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                }
                
                Spacer()
                
                if multipeerManager.isConnected {
                    Image(systemName: "checkmark.circle.fill")
                        .foregroundColor(.green)
                        .font(.title2)
                }
            }
            .padding()
            .background(multipeerManager.isConnected ? Color.green.opacity(0.1) : Color.orange.opacity(0.1))
            .cornerRadius(8)
            
            // Auto-collection toggle
            if multipeerManager.isConnected {
                Toggle(isOn: $autoCollectWhenStill) {
                    HStack {
                        Image(systemName: "hand.raised.fill")
                        Text("Auto-collect when iPhone is still")
                            .font(.subheadline)
                    }
                }
                .tint(.purple)
                
                Toggle(isOn: $autoAdvanceOnStill) {
                    HStack {
                        Image(systemName: "arrow.right.circle.fill")
                        Text("Auto-proceed when iPhone moves & settles")
                            .font(.subheadline)
                    }
                }
                .tint(.green)
            }
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
    
    private var instructionsCard: some View {
        VStack(alignment: .leading, spacing: 12) {
            Label("Calibration Steps", systemImage: "list.number")
                .font(.headline)
            
            VStack(alignment: .leading, spacing: 8) {
                instructionRow(number: 1, text: "Open iPhone → Calibration → Display ArUco Marker")
                instructionRow(number: 2, text: "iPhone shows 55mm marker (IDs: 0, 2, 3)")
                instructionRow(number: 3, text: "Look CLOSELY until ARKit registers marker")
                instructionRow(number: 4, text: "Move away, collect 20 samples from angles")
                instructionRow(number: 5, text: "Tap 'Proceed' → Move iPhone & swipe to next marker")
                instructionRow(number: 6, text: "Repeat for all 3 markers (60 samples total)")
                instructionRow(number: 7, text: "Auto-completes when done")
            }
            .font(.caption)
            .foregroundColor(.secondary)
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
    
    private func instructionRow(number: Int, text: String) -> some View {
        HStack(alignment: .top, spacing: 8) {
            Text("\(number).")
                .fontWeight(.semibold)
                .frame(width: 20, alignment: .leading)
            Text(text)
        }
    }
    
    private var cameraSelectionCard: some View {
        VStack(alignment: .leading, spacing: 12) {
            Label("Camera Setup", systemImage: "camera")
                .font(.headline)
            
            if uvcCameraManager.availableDevices.isEmpty {
                Text("No UVC cameras detected")
                    .font(.subheadline)
                    .foregroundColor(.orange)
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(Color.orange.opacity(0.1))
                    .cornerRadius(8)
            } else {
                ForEach(uvcCameraManager.availableDevices, id: \.id) { device in
                    cameraDeviceRow(device)
                }
            }
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
    
    private func cameraDeviceRow(_ device: UVCDevice) -> some View {
        let hasIntrinsic = cameraCalibrationManager.allCalibrations[device.id] != nil
        let isSelected = selectedDeviceId == device.id
        
        return Button(action: {
            selectedDeviceId = device.id
            isStereoMode = device.name.lowercased().contains("stereo")
        }) {
            HStack {
                VStack(alignment: .leading, spacing: 4) {
                    Text(device.name)
                        .font(.subheadline)
                        .fontWeight(.medium)
                        .foregroundColor(.primary)
                    
                    HStack(spacing: 8) {
                        if hasIntrinsic {
                            Label("Intrinsic ✓", systemImage: "checkmark.circle.fill")
                                .font(.caption2)
                                .foregroundColor(.green)
                        } else {
                            Label("No Intrinsic", systemImage: "exclamationmark.triangle.fill")
                                .font(.caption2)
                                .foregroundColor(.orange)
                        }
                        
                        if device.name.lowercased().contains("stereo") {
                            Label("Stereo", systemImage: "eye.fill")
                                .font(.caption2)
                                .foregroundColor(.blue)
                        }
                    }
                }
                
                Spacer()
                
                Image(systemName: isSelected ? "checkmark.circle.fill" : "circle")
                    .foregroundColor(isSelected ? .green : .gray)
            }
            .padding()
            .background(isSelected ? Color.green.opacity(0.1) : Color(.tertiarySystemBackground))
            .cornerRadius(8)
        }
        .disabled(!hasIntrinsic)
    }
    
    private var cameraModeCard: some View {
        VStack(alignment: .leading, spacing: 12) {
            Label("Camera Mode", systemImage: "camera.metering.multispot")
                .font(.headline)
            
            HStack(spacing: 12) {
                // Mono button
                Button(action: { isStereoMode = false }) {
                    HStack {
                        Image(systemName: "camera")
                        Text("Mono")
                    }
                    .font(.subheadline)
                    .fontWeight(.medium)
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(!isStereoMode ? Color.purple.opacity(0.2) : Color(.tertiarySystemBackground))
                    .foregroundColor(!isStereoMode ? .purple : .secondary)
                    .cornerRadius(8)
                }
                
                // Stereo button
                Button(action: { isStereoMode = true }) {
                    HStack {
                        Image(systemName: "camera.fill")
                        Text("Stereo")
                    }
                    .font(.subheadline)
                    .fontWeight(.medium)
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(isStereoMode ? Color.blue.opacity(0.2) : Color(.tertiarySystemBackground))
                    .foregroundColor(isStereoMode ? .blue : .secondary)
                    .cornerRadius(8)
                }
            }
            
            Text(isStereoMode ? "Calibrates both left and right cameras" : "Calibrates single camera")
                .font(.caption)
                .foregroundColor(.secondary)
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
    
    private var stereoBaselineCard: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Label("Stereo Baseline", systemImage: "ruler")
                    .font(.headline)
                
                Spacer()
                
                if let baseline = calibrationManager.knownStereoBaseline {
                    Text("\(Int(baseline * 1000)) mm")
                        .font(.subheadline)
                        .fontWeight(.medium)
                        .foregroundColor(.blue)
                        .monospacedDigit()
                } else {
                    Text("Auto")
                        .font(.subheadline)
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
            
            HStack(spacing: 8) {
                Button(action: { calibrationManager.knownStereoBaseline = nil }) {
                    Text("Auto")
                        .font(.caption)
                        .fontWeight(.medium)
                        .padding(.horizontal, 12)
                        .padding(.vertical, 6)
                        .background(calibrationManager.knownStereoBaseline == nil ? Color.gray.opacity(0.3) : Color(.tertiarySystemBackground))
                        .cornerRadius(6)
                }
                
                Button(action: { calibrationManager.knownStereoBaseline = 0.065 }) {
                    Text("65mm")
                        .font(.caption)
                        .fontWeight(.medium)
                        .padding(.horizontal, 12)
                        .padding(.vertical, 6)
                        .background(calibrationManager.knownStereoBaseline == 0.065 ? Color.blue.opacity(0.3) : Color(.tertiarySystemBackground))
                        .cornerRadius(6)
                }
                
                Spacer()
                
                Text("Known baseline enforces scale")
                    .font(.caption2)
                    .foregroundColor(.secondary)
            }
        }
        .padding()
        .background(Color.blue.opacity(0.05))
        .cornerRadius(12)
    }
    
    private var savedCalibrationsCard: some View {
        VStack(alignment: .leading, spacing: 12) {
            Label("Saved Calibrations", systemImage: "checkmark.seal")
                .font(.headline)
            
            if calibrationManager.allCalibrations.isEmpty {
                Text("No calibrations saved yet")
                    .font(.subheadline)
                    .foregroundColor(.secondary)
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(Color(.tertiarySystemBackground))
                    .cornerRadius(8)
            } else {
                ForEach(Array(calibrationManager.allCalibrations.keys.sorted()), id: \.self) { deviceId in
                    if let calibration = calibrationManager.allCalibrations[deviceId] {
                        savedCalibrationRow(calibration)
                    }
                }
            }
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
    
    private func savedCalibrationRow(_ calibration: ExtrinsicCalibrationData) -> some View {
        HStack {
            VStack(alignment: .leading, spacing: 4) {
                Text(calibration.cameraDeviceName)
                    .font(.subheadline)
                    .fontWeight(.medium)
                
                HStack(spacing: 8) {
                    Text(calibration.calibrationDate, style: .date)
                        .font(.caption2)
                        .foregroundColor(.secondary)
                    
                    Text(String(format: "%.1fmm error", calibration.leftReprojectionError * 1000))
                        .font(.caption2)
                        .foregroundColor(calibration.leftReprojectionError < 0.01 ? .green : .orange)
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
        .padding()
        .background(Color(.tertiarySystemBackground))
        .cornerRadius(8)
        .alert("Delete Calibration?", isPresented: $showingDeleteConfirmation) {
            Button("Cancel", role: .cancel) {}
            Button("Delete", role: .destructive) {
                if let deviceId = calibrationToDelete {
                    calibrationManager.deleteCalibration(for: deviceId)
                }
            }
        }
    }
    
    private var startButton: some View {
        Button(action: startCalibration) {
            HStack {
                Image(systemName: "play.circle.fill")
                Text("Start Calibration")
            }
            .font(.headline)
            .foregroundColor(.white)
            .frame(maxWidth: .infinity)
            .padding()
            .background(Color.purple)
            .cornerRadius(12)
        }
    }
    
    // MARK: - Calibration In Progress View
    
    private var calibrationInProgressView: some View {
        VStack(spacing: 20) {
            // Current marker indicator
            currentMarkerDisplay
            
            // Progress bar
            progressDisplay
            
            // iPhone motion status (if connected)
            if multipeerManager.isConnected {
                iPhoneMotionStatusDisplay
            }
            
            // Detection status
            detectionStatusDisplay
            
            // User guidance
            userGuidanceDisplay
            
            Spacer()
            
            // Action buttons
            actionButtons
        }
        .padding()
        .background(Color(.systemGroupedBackground))
    }
    
    private var iPhoneMotionStatusDisplay: some View {
        VStack(spacing: 10) {
            HStack {
                Image(systemName: multipeerManager.isPhoneStationary ? "hand.raised.fill" : "figure.walk")
                    .foregroundColor(multipeerManager.isPhoneStationary ? .green : .orange)
                    .frame(width: 24)
                
                Text("iPhone Status")
                    .font(.subheadline)
                
                Spacer()
                
                HStack(spacing: 4) {
                    Image(systemName: multipeerManager.isPhoneStationary ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
                    Text(multipeerManager.isPhoneStationary ? "Stationary - Collecting" : "Moving - Paused")
                        .font(.caption)
                }
                .foregroundColor(multipeerManager.isPhoneStationary ? .green : .orange)
            }
            
            // Motion magnitude bar
            GeometryReader { geometry in
                ZStack(alignment: .leading) {
                    RoundedRectangle(cornerRadius: 4)
                        .fill(Color.gray.opacity(0.3))
                        .frame(height: 8)
                    
                    RoundedRectangle(cornerRadius: 4)
                        .fill(motionColor(for: multipeerManager.phoneMotionMagnitude))
                        .frame(width: geometry.size.width * CGFloat(min(1.0, multipeerManager.phoneMotionMagnitude)), height: 8)
                }
            }
            .frame(height: 8)
            
            HStack {
                Text("Still")
                    .font(.caption2)
                    .foregroundColor(.secondary)
                Spacer()
                Text("Moving")
                    .font(.caption2)
                    .foregroundColor(.secondary)
            }
        }
        .padding()
        .background(multipeerManager.isPhoneStationary ? Color.green.opacity(0.1) : Color.orange.opacity(0.1))
        .cornerRadius(12)
    }
    
    private func motionColor(for magnitude: Double) -> Color {
        if magnitude < 0.3 {
            return .green
        } else if magnitude < 0.6 {
            return .yellow
        } else {
            return .orange
        }
    }
    
    private var currentMarkerDisplay: some View {
        VStack(spacing: 12) {
            Text("Current Marker")
                .font(.caption)
                .foregroundColor(.secondary)
            
            HStack(spacing: 20) {
                ForEach(calibrationManager.markerIds, id: \.self) { markerId in
                    let isCurrent = markerId == calibrationManager.currentMarkerId
                    let leftSamples = calibrationManager.samples.filter { 
                        $0.markerId == markerId && ($0.cameraSide == .left || $0.cameraSide == .mono)
                    }.count
                    let isComplete = leftSamples >= 20
                    
                    VStack(spacing: 6) {
                        ZStack {
                            Circle()
                                .fill(isCurrent ? Color.purple : (isComplete ? Color.green : Color.gray).opacity(0.2))
                                .frame(width: 60, height: 60)
                            
                            if isComplete {
                                Image(systemName: "checkmark")
                                    .font(.title2)
                                    .foregroundColor(.white)
                            } else {
                                Text("\(markerId)")
                                    .font(.system(size: 28, weight: .bold, design: .rounded))
                                    .foregroundColor(isCurrent ? .white : .gray)
                            }
                        }
                        
                        Text(isCurrent ? "\(leftSamples)/20" : (isComplete ? "Done" : "—"))
                            .font(.caption2)
                            .foregroundColor(isCurrent ? .purple : .secondary)
                    }
                }
            }
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(16)
    }
    
    private var progressDisplay: some View {
        VStack(spacing: 8) {
            HStack {
                Text("Total Progress")
                    .font(.caption)
                    .foregroundColor(.secondary)
                Spacer()
                Text("\(calibrationManager.samplesCollected)/60 samples")
                    .font(.caption)
                    .fontWeight(.medium)
                    .monospacedDigit()
            }
            
            ProgressView(value: Double(calibrationManager.samplesCollected), total: 60.0)
                .tint(.purple)
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
    
    private var detectionStatusDisplay: some View {
        VStack(spacing: 10) {
            detectionRow(
                icon: "eye.fill",
                label: "ARKit",
                isDetected: calibrationManager.arkitTrackedMarkers[calibrationManager.currentMarkerId] != nil ||
                           calibrationManager.rememberedMarkerPositions[calibrationManager.currentMarkerId] != nil
            )
            
            detectionRow(
                icon: "camera.fill",
                label: "Camera",
                isDetected: calibrationManager.cameraDetectedMarkers[calibrationManager.currentMarkerId] != nil
            )
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
    
    private func detectionRow(icon: String, label: String, isDetected: Bool) -> some View {
        HStack {
            Image(systemName: icon)
                .frame(width: 24)
                .foregroundColor(isDetected ? .green : .gray)
            
            Text(label)
                .font(.subheadline)
            
            Spacer()
            
            HStack(spacing: 4) {
                Image(systemName: isDetected ? "checkmark.circle.fill" : "circle")
                Text(isDetected ? "Tracking ID \(calibrationManager.currentMarkerId)" : "Not detected")
                    .font(.caption)
            }
            .foregroundColor(isDetected ? .green : .secondary)
        }
    }
    
    private var userGuidanceDisplay: some View {
        let arkitHasMarker = calibrationManager.rememberedMarkerPositions[calibrationManager.currentMarkerId] != nil || 
                            calibrationManager.arkitTrackedMarkers[calibrationManager.currentMarkerId] != nil
        let leftSamples = calibrationManager.samples.filter { 
            $0.markerId == calibrationManager.currentMarkerId && ($0.cameraSide == .left || $0.cameraSide == .mono)
        }.count
        
        let (icon, color, title, subtitle) = getGuidanceInfo(arkitHasMarker: arkitHasMarker, samples: leftSamples)
        
        return HStack(spacing: 12) {
            Image(systemName: icon)
                .font(.title2)
                .foregroundColor(color)
                .frame(width: 40)
            
            VStack(alignment: .leading, spacing: 2) {
                Text(title)
                    .font(.subheadline)
                    .fontWeight(.semibold)
                Text(subtitle)
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            
            Spacer()
        }
        .padding()
        .background(color.opacity(0.1))
        .cornerRadius(12)
    }
    
    private func getGuidanceInfo(arkitHasMarker: Bool, samples: Int) -> (String, Color, String, String) {
        if !arkitHasMarker {
            return ("eye.fill", .orange, "Look at marker ID \(calibrationManager.currentMarkerId)", "Move closer until ARKit registers it")
        } else if samples < 20 {
            return ("move.3d", .blue, "Collecting samples \(samples)/20", "Move around and view from different angles")
        } else if calibrationManager.canAdvanceToNextMarker {
            return ("arrow.right.circle.fill", .green, "Position complete!", "Ready to move to next marker")
        } else {
            return ("checkmark.circle.fill", .green, "All markers complete!", "Ready to finish calibration")
        }
    }
    
    private var nextMarkerId: Int {
        let nextIndex = calibrationManager.currentMarkerIndex + 1
        if nextIndex < calibrationManager.markerIds.count {
            return calibrationManager.markerIds[nextIndex]
        }
        return calibrationManager.currentMarkerId
    }
    
    private var actionButtons: some View {
        VStack(spacing: 12) {
            let leftSamples = calibrationManager.samples.filter { 
                $0.markerId == calibrationManager.currentMarkerId && ($0.cameraSide == .left || $0.cameraSide == .mono)
            }.count
            
            if leftSamples >= 20 {
                if calibrationManager.canAdvanceToNextMarker {
                    // Proceed to next marker - directly advance without dialog for simplicity
                    Button(action: {
                        print("📐 [UI] Proceed button tapped - advancing to next marker")
                        calibrationManager.advanceToNextMarker()
                        calibrationManager.rememberedMarkerPositions.removeAll()
                        
                        // Notify iPhone to switch marker if connected
                        if multipeerManager.isConnected {
                            multipeerManager.switchToMarker(calibrationManager.currentMarkerId)
                        }
                    }) {
                        HStack {
                            Image(systemName: "arrow.right.circle.fill")
                            Text("Proceed to Marker \(nextMarkerId)")
                        }
                        .font(.headline)
                        .foregroundColor(.white)
                        .frame(maxWidth: .infinity)
                        .padding()
                        .background(Color.green)
                        .cornerRadius(12)
                    }
                } else {
                    // All done - auto finish
                    Button(action: { Task { await autoFinishCalibration() } }) {
                        HStack {
                            if isProcessing {
                                ProgressView()
                                    .progressViewStyle(CircularProgressViewStyle(tint: .white))
                            } else {
                                Image(systemName: "checkmark.circle.fill")
                            }
                            Text(isProcessing ? "Optimizing..." : "Complete Calibration")
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
                    
                    // Notify iPhone that calibration is cancelled
                    if multipeerManager.isConnected {
                        multipeerManager.calibrationComplete()
                    }
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
    }
    
    // MARK: - Actions
    
    private func startCalibration() {
        guard let deviceId = selectedDeviceId else { return }
        isCalibrating = true
        
        Task {
            await calibrationManager.startCalibration(
                deviceId: deviceId,
                deviceName: uvcCameraManager.availableDevices.first(where: { $0.id == deviceId })?.name ?? "Unknown",
                isStereo: isStereoMode
            )
            
            // Notify iPhone to start calibration if connected
            if multipeerManager.isConnected {
                multipeerManager.startCalibration(markerIds: calibrationManager.markerIds)
            }
        }
    }
    
    private func autoFinishCalibration() async {
        guard let deviceId = selectedDeviceId,
              let device = uvcCameraManager.availableDevices.first(where: { $0.id == deviceId }) else {
            return
        }
        
        isProcessing = true
        try? await Task.sleep(nanoseconds: 300_000_000)
        
        let result = calibrationManager.finishCalibration(
            deviceId: deviceId,
            deviceName: device.name,
            isStereo: isStereoMode
        )
        
        // Notify iPhone that calibration is complete
        if multipeerManager.isConnected {
            multipeerManager.calibrationComplete()
        }
        
        isProcessing = false
        isCalibrating = false
        
        if result != nil {
            try? await Task.sleep(nanoseconds: 500_000_000)
            onDismiss?()
        }
    }
    
    private func setupMultipeerCallbacks() {
        // Listen for iPhone motion status to control sample collection
        multipeerManager.onStatusReceived = { [self] status in
            Task { @MainActor in
                // If auto-collection is enabled, update the pause state based on phone motion
                if autoCollectWhenStill && isCalibrating {
                    calibrationManager.isCollectionPaused = !status.isPhoneStationary
                }
                
                // Auto-advance logic: when phone transitions from moving → still
                if autoAdvanceOnStill && isCalibrating {
                    let isNowStationary = status.isPhoneStationary
                    
                    // Detect transition: was moving, now still
                    if wasPhoneMoving && isNowStationary {
                        // Check if current marker has enough samples
                        let leftSamples = calibrationManager.samples.filter { 
                            $0.markerId == calibrationManager.currentMarkerId && ($0.cameraSide == .left || $0.cameraSide == .mono)
                        }.count
                        
                        if leftSamples >= 20 {
                            if calibrationManager.canAdvanceToNextMarker {
                                // Auto-advance to next marker
                                print("📐 [Auto-Advance] Phone settled after moving - advancing to next marker")
                                calibrationManager.advanceToNextMarker()
                                calibrationManager.rememberedMarkerPositions.removeAll()
                                
                                // Notify iPhone to switch marker
                                multipeerManager.switchToMarker(calibrationManager.currentMarkerId)
                            } else {
                                // All markers complete - auto finish
                                print("📐 [Auto-Advance] Phone settled after moving - all markers complete, finishing calibration")
                                Task {
                                    await autoFinishCalibration()
                                }
                            }
                        }
                    }
                    
                    // Update tracking state
                    wasPhoneMoving = !isNowStationary
                }
            }
        }
        
        // Handle connection changes
        multipeerManager.onConnectionChanged = { [self] isConnected in
            Task { @MainActor in
                if isConnected && isCalibrating {
                    // Notify iPhone about current calibration state
                    multipeerManager.startCalibration(markerIds: calibrationManager.markerIds)
                    multipeerManager.switchToMarker(calibrationManager.currentMarkerId)
                }
            }
        }
    }
    
    private func setupFrameCallback() {
        uvcCameraManager.onPixelBufferReceived = { [self] pixelBuffer in
            guard calibrationManager.isCalibrating,
                  let deviceId = selectedDeviceId,
                  let intrinsicData = cameraCalibrationManager.allCalibrations[deviceId] else {
                return
            }
            
            calibrationManager.processCameraFrame(
                pixelBuffer,
                intrinsics: intrinsicData.leftIntrinsics,
                rightIntrinsics: intrinsicData.rightIntrinsics
            )
        }
    }
}
