//
//  CalibrationCoordinatorView.swift
//  Tracking Viewer
//
//  Coordinates calibration display based on Vision Pro commands.
//  This view automatically shows the correct pattern (checkerboard or ArUco)
//  when commanded by the Vision Pro during the calibration wizard.
//

import SwiftUI

/// A coordinator view that listens for Vision Pro commands and displays the appropriate calibration pattern
struct CalibrationCoordinatorView: View {
    @StateObject private var multipeerManager = MultipeerCalibrationManager_iOS.shared
    @StateObject private var displayManager = CalibrationDisplayManager.shared
    @Environment(\.dismiss) private var dismiss
    
    var body: some View {
        ZStack {
            switch multipeerManager.displayMode {
            case .none:
                // Waiting for Vision Pro to send command
                waitingView
                
            case .checkerboard:
                // Show checkerboard pattern with progress
                CheckerboardCalibrationView(
                    progress: multipeerManager.intrinsicProgress,
                    onDismiss: { dismiss() }
                )
                
            case .charuco:
                // Show ChArUco pattern with progress
                CharucoCalibrationView(
                    progress: multipeerManager.intrinsicProgress,
                    onDismiss: { dismiss() }
                )
                
            case .aruco(let markerId):
                // Show ArUco marker with progress
                ArucoCalibrationView(
                    markerId: markerId,
                    markerDetected: multipeerManager.markerDetectedByARKit,
                    progress: multipeerManager.extrinsicProgress,
                    readyForNext: multipeerManager.readyForNextPosition,
                    onDismiss: { dismiss() }
                )
                
            case .verificationBoard(let cols, let rows, let tagSizeMM, let marginMM, let tagIds):
                // Show verification board for intrinsic verification
                VerificationBoardCalibrationView(
                    cols: cols,
                    rows: rows,
                    tagSizeMM: tagSizeMM,
                    marginMM: marginMM,
                    tagIds: tagIds,
                    onDismiss: { dismiss() }
                )
            }
        }
        .statusBarHidden(true)
        .onAppear {
            // Start advertising if not already
            multipeerManager.startAdvertising()
            multipeerManager.startStatusUpdates()
        }
        .onDisappear {
            multipeerManager.stopStatusUpdates()
        }
        .onChange(of: multipeerManager.displayMode) { oldValue, newValue in
            // Haptic feedback on mode change
            if oldValue != newValue {
                let generator = UIImpactFeedbackGenerator(style: .medium)
                generator.impactOccurred()
            }
        }
        .onChange(of: multipeerManager.isInCalibrationMode) { _, isCalibrating in
            // Auto-dismiss when calibration mode ends (triggered by Vision Pro sending hideDisplay)
            if !isCalibrating {
                dismiss()
            }
        }
    }
    
    // MARK: - Waiting View
    
    private var waitingView: some View {
        ZStack {
            Color.black.ignoresSafeArea()
            
            VStack(spacing: 32) {
                // Connection status
                VStack(spacing: 16) {
                    if multipeerManager.isConnected {
                        Image(systemName: "visionpro")
                            .font(.system(size: 64))
                            .foregroundColor(.green)
                        
                        Text("Connected to Vision Pro")
                            .font(.title2)
                            .fontWeight(.semibold)
                            .foregroundColor(.white)
                        
                        Text("Waiting for calibration to start...")
                            .font(.body)
                            .foregroundColor(.white.opacity(0.7))
                        
                        ProgressView()
                            .progressViewStyle(CircularProgressViewStyle(tint: .white))
                            .scaleEffect(1.2)
                            .padding(.top, 8)
                    } else {
                        // Not connected animation
                        ZStack {
                            Circle()
                                .stroke(Color.blue.opacity(0.3), lineWidth: 4)
                                .frame(width: 120, height: 120)
                            
                            Circle()
                                .stroke(Color.blue.opacity(0.5), lineWidth: 4)
                                .frame(width: 120, height: 120)
                                .scaleEffect(1.0)
                                .opacity(1.0)
                                .animation(.easeOut(duration: 1.5).repeatForever(autoreverses: false), value: multipeerManager.isConnected)
                            
                            Image(systemName: "visionpro")
                                .font(.system(size: 48))
                                .foregroundColor(.blue)
                        }
                        
                        Text("Waiting for Vision Pro")
                            .font(.title2)
                            .fontWeight(.semibold)
                            .foregroundColor(.white)
                        
                        Text(multipeerManager.connectionStatus)
                            .font(.body)
                            .foregroundColor(.white.opacity(0.7))
                    }
                }
                
                // Debug / Manual Triggers
                if !multipeerManager.isConnected {
                    VStack(spacing: 12) {
                        Text("Manual Debug")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.5))
                        
                        HStack(spacing: 20) {
                            Button {
                                multipeerManager.displayMode = .charuco
                                multipeerManager.isInCalibrationMode = true
                            } label: {
                                VStack {
                                    Image(systemName: "square.grid.4x3.fill")
                                        .font(.system(size: 24))
                                    Text("ChArUco")
                                        .font(.caption)
                                }
                                .foregroundColor(.white)
                                .frame(width: 80, height: 60)
                                .background(Color.white.opacity(0.1))
                                .cornerRadius(12)
                            }
                            
                            Button {
                                multipeerManager.displayMode = .checkerboard
                                multipeerManager.isInCalibrationMode = true
                            } label: {
                                VStack {
                                    Image(systemName: "checkerboard.rectangle")
                                        .font(.system(size: 24))
                                    Text("Checker")
                                        .font(.caption)
                                }
                                .foregroundColor(.white)
                                .frame(width: 80, height: 60)
                                .background(Color.white.opacity(0.1))
                                .cornerRadius(12)
                            }
                        }
                    }
                }

                
                // Instructions
                VStack(alignment: .leading, spacing: 12) {
                    instructionRow(number: "1", text: "Open Camera Calibration on Vision Pro")
                    instructionRow(number: "2", text: "Tap 'Calibrate Camera' to start")
                    instructionRow(number: "3", text: "Follow the on-screen instructions")
                }
                .padding(24)
                .background(Color.white.opacity(0.1))
                .cornerRadius(16)
                
                // Close button
                Button(action: { dismiss() }) {
                    Text("Close")
                        .font(.headline)
                        .foregroundColor(.white)
                        .frame(width: 200)
                        .padding(.vertical, 14)
                        .background(Color.white.opacity(0.2))
                        .cornerRadius(12)
                }
            }
            .padding(32)
        }
    }
    
    private func instructionRow(number: String, text: String) -> some View {
        HStack(spacing: 12) {
            ZStack {
                Circle()
                    .fill(Color.blue)
                    .frame(width: 28, height: 28)
                Text(number)
                    .font(.system(size: 14, weight: .bold))
                    .foregroundColor(.white)
            }
            
            Text(text)
                .font(.body)
                .foregroundColor(.white.opacity(0.9))
        }
    }
}

// MARK: - Checkerboard Calibration View

/// Displays the checkerboard pattern with intrinsic calibration progress
struct CheckerboardCalibrationView: View {
    let progress: (samples: Int, total: Int)
    let onDismiss: () -> Void
    
    @StateObject private var manager = CalibrationDisplayManager.shared
    @StateObject private var multipeerManager = MultipeerCalibrationManager_iOS.shared
    
    @State private var showControls = true
    @State private var hideControlsTimer: Timer?
    
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                // White background
                Color.white.ignoresSafeArea()
                
                // Checkerboard pattern (rotated 90° for portrait)
                CheckerboardPatternView(
                    innerCornersX: manager.checkerboardConfig.innerCornersX,
                    innerCornersY: manager.checkerboardConfig.innerCornersY,
                    squareSizePoints: manager.mmToPoints(Double(manager.checkerboardConfig.squareSizeMM))
                )
                .rotationEffect(.degrees(90))
                .frame(maxWidth: geometry.size.width, maxHeight: geometry.size.height)
                .clipped()
                
                // Controls overlay
                if showControls {
                    controlsOverlay
                }
            }
            .onTapGesture {
                withAnimation(.easeInOut(duration: 0.2)) {
                    showControls.toggle()
                }
                if showControls {
                    resetHideTimer()
                }
            }
        }
        .onAppear {
            resetHideTimer()
        }
        .onDisappear {
            hideControlsTimer?.invalidate()
        }
    }
    
    private var controlsOverlay: some View {
        VStack {
            // Top gradient + close button
            ZStack(alignment: .top) {
                LinearGradient(
                    colors: [Color.black.opacity(0.5), Color.clear],
                    startPoint: .top,
                    endPoint: .bottom
                )
                .frame(height: 140)
                
                HStack {
                    Button(action: onDismiss) {
                        Image(systemName: "xmark")
                            .font(.system(size: 16, weight: .bold))
                            .foregroundColor(.white)
                            .frame(width: 36, height: 36)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    
                    Spacer()
                    
                    // Connection indicator
                    HStack(spacing: 6) {
                        Image(systemName: "visionpro")
                            .font(.system(size: 12))
                        Text("Connected")
                            .font(.system(size: 12, weight: .medium))
                    }
                    .foregroundColor(.white)
                    .padding(.horizontal, 12)
                    .padding(.vertical, 6)
                    .background(Color.green.opacity(0.8))
                    .clipShape(Capsule())
                    
                    Spacer()
                    
                    Color.clear.frame(width: 36, height: 36)
                }
                .padding(.horizontal, 20)
                .padding(.top, 16)
            }
            
            Spacer()
            
            // Bottom gradient + progress
            ZStack(alignment: .bottom) {
                LinearGradient(
                    colors: [Color.clear, Color.black.opacity(0.6)],
                    startPoint: .top,
                    endPoint: .bottom
                )
                .frame(height: 200)
                
                VStack(spacing: 16) {
                    // Progress ring
                    ZStack {
                        Circle()
                            .stroke(Color.white.opacity(0.3), lineWidth: 6)
                            .frame(width: 80, height: 80)
                        
                        Circle()
                            .trim(from: 0, to: CGFloat(progress.samples) / CGFloat(progress.total))
                            .stroke(Color.cyan, style: StrokeStyle(lineWidth: 6, lineCap: .round))
                            .frame(width: 80, height: 80)
                            .rotationEffect(.degrees(-90))
                        
                        VStack(spacing: 0) {
                            Text("\(progress.samples)")
                                .font(.system(size: 24, weight: .bold, design: .rounded))
                                .foregroundColor(.white)
                            Text("/\(progress.total)")
                                .font(.system(size: 12))
                                .foregroundColor(.white.opacity(0.7))
                        }
                    }
                    
                    Text("Intrinsic Calibration")
                        .font(.headline)
                        .foregroundColor(.white)
                    
                    Text("Move around to capture different angles")
                        .font(.subheadline)
                        .foregroundColor(.white.opacity(0.8))
                    
                    // Motion status
                    HStack(spacing: 8) {
                        Circle()
                            .fill(multipeerManager.isPhoneStationary ? Color.green : Color.orange)
                            .frame(width: 8, height: 8)
                        Text(multipeerManager.isPhoneStationary ? "Steady" : "Moving")
                            .font(.caption)
                            .foregroundColor(multipeerManager.isPhoneStationary ? .green : .orange)
                    }
                    .padding(.horizontal, 12)
                    .padding(.vertical, 6)
                    .background(Color.white.opacity(0.1))
                    .clipShape(Capsule())
                }
                .padding(.bottom, 50)
            }
        }
        .ignoresSafeArea()
    }
    
    private func resetHideTimer() {
        hideControlsTimer?.invalidate()
        hideControlsTimer = Timer.scheduledTimer(withTimeInterval: 4.0, repeats: false) { _ in
            withAnimation(.easeInOut(duration: 0.2)) {
                showControls = false
            }
        }
    }
}

// MARK: - ArUco Calibration View

/// Displays the ArUco marker with extrinsic calibration progress
struct ArucoCalibrationView: View {
    let markerId: Int
    let markerDetected: Bool
    let progress: (marker: Int, samples: Int, total: Int)
    let readyForNext: Bool
    let onDismiss: () -> Void
    
    @StateObject private var manager = CalibrationDisplayManager.shared
    @StateObject private var multipeerManager = MultipeerCalibrationManager_iOS.shared
    
    @State private var showControls = true
    @State private var hideControlsTimer: Timer?
    
    private var shouldHideMarker: Bool {
        !multipeerManager.isPhoneStationary && multipeerManager.motionDetector.isRunning
    }
    
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                // Background
                (shouldHideMarker ? Color.black : Color.white)
                    .ignoresSafeArea()
                    .animation(.easeInOut(duration: 0.3), value: shouldHideMarker)
                
                // Marker or motion warning
                if shouldHideMarker {
                    motionWarningView
                } else {
                    // ArUco marker
                    if let markerImage = manager.generateArucoMarker(
                        id: markerId,
                        sizePoints: manager.markerSizePoints
                    ) {
                        Image(uiImage: markerImage)
                            .interpolation(.none)
                            .resizable()
                            .frame(width: manager.markerSizePoints, height: manager.markerSizePoints)
                    }
                }
                
                // Controls overlay
                if showControls {
                    controlsOverlay
                }
            }
            .onTapGesture {
                withAnimation(.easeInOut(duration: 0.2)) {
                    showControls.toggle()
                }
                if showControls {
                    resetHideTimer()
                }
            }
        }
        .onAppear {
            manager.setMarkerId(markerId)
            resetHideTimer()
        }
        .onDisappear {
            hideControlsTimer?.invalidate()
        }
        .onChange(of: markerId) { _, newId in
            manager.setMarkerId(newId)
        }
    }
    
    private var motionWarningView: some View {
        VStack(spacing: 24) {
            Image(systemName: "iphone.gen3.radiowaves.left.and.right")
                .font(.system(size: 64))
                .foregroundColor(.orange)
                .symbolEffect(.pulse, options: .repeating)
            
            Text("Hold iPhone Still")
                .font(.title2)
                .fontWeight(.semibold)
                .foregroundColor(.white)
            
            // Motion indicator
            HStack(spacing: 4) {
                ForEach(0..<5, id: \.self) { index in
                    RoundedRectangle(cornerRadius: 2)
                        .fill(index < Int(multipeerManager.motionMagnitude * 5) + 1 ? Color.orange : Color.white.opacity(0.3))
                        .frame(width: 16, height: 8)
                }
            }
        }
    }
    
    private var controlsOverlay: some View {
        VStack {
            // Top bar
            ZStack(alignment: .top) {
                LinearGradient(
                    colors: [Color.black.opacity(0.5), Color.clear],
                    startPoint: .top,
                    endPoint: .bottom
                )
                .frame(height: 140)
                
                HStack {
                    Button(action: onDismiss) {
                        Image(systemName: "xmark")
                            .font(.system(size: 16, weight: .bold))
                            .foregroundColor(.white)
                            .frame(width: 36, height: 36)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                    
                    Spacer()
                    
                    // Marker ID badge
                    VStack(spacing: 2) {
                        Text("Marker")
                            .font(.system(size: 10, weight: .medium))
                            .foregroundColor(.white.opacity(0.7))
                        Text("\(markerId)")
                            .font(.system(size: 24, weight: .bold, design: .rounded))
                            .foregroundColor(.white)
                    }
                    .padding(.horizontal, 16)
                    .padding(.vertical, 8)
                    .background(Color.black.opacity(0.5))
                    .clipShape(Capsule())
                    
                    Spacer()
                    
                    Color.clear.frame(width: 36, height: 36)
                }
                .padding(.horizontal, 20)
                .padding(.top, 16)
            }
            
            Spacer()
            
            // Bottom bar with progress
            ZStack(alignment: .bottom) {
                LinearGradient(
                    colors: [Color.clear, Color.black.opacity(0.6)],
                    startPoint: .top,
                    endPoint: .bottom
                )
                .frame(height: 250)
                
                VStack(spacing: 16) {
                    // ARKit detection status
                    HStack(spacing: 8) {
                        Image(systemName: markerDetected ? "eye.fill" : "eye.slash.fill")
                            .font(.system(size: 14))
                        Text(markerDetected ? "ARKit Tracking" : "Looking for marker...")
                            .font(.system(size: 13, weight: .medium))
                    }
                    .foregroundColor(markerDetected ? .green : .orange)
                    .padding(.horizontal, 14)
                    .padding(.vertical, 8)
                    .background((markerDetected ? Color.green : Color.orange).opacity(0.2))
                    .clipShape(Capsule())
                    
                    // Progress ring
                    ZStack {
                        Circle()
                            .stroke(Color.white.opacity(0.3), lineWidth: 6)
                            .frame(width: 80, height: 80)
                        
                        Circle()
                            .trim(from: 0, to: CGFloat(progress.samples) / CGFloat(progress.total))
                            .stroke(Color.purple, style: StrokeStyle(lineWidth: 6, lineCap: .round))
                            .frame(width: 80, height: 80)
                            .rotationEffect(.degrees(-90))
                        
                        VStack(spacing: 0) {
                            Text("\(progress.samples)")
                                .font(.system(size: 24, weight: .bold, design: .rounded))
                                .foregroundColor(.white)
                            Text("/\(progress.total)")
                                .font(.system(size: 12))
                                .foregroundColor(.white.opacity(0.7))
                        }
                    }
                    
                    // Ready for next position indicator
                    if readyForNext {
                        VStack(spacing: 8) {
                            Image(systemName: "checkmark.circle.fill")
                                .font(.system(size: 32))
                                .foregroundColor(.green)
                            
                            Text("Position Complete!")
                                .font(.headline)
                                .foregroundColor(.white)
                            
                            Text("Move iPhone to a new location")
                                .font(.subheadline)
                                .foregroundColor(.white.opacity(0.8))
                        }
                        .padding(16)
                        .background(Color.green.opacity(0.2))
                        .cornerRadius(16)
                    } else {
                        Text("Extrinsic Calibration")
                            .font(.headline)
                            .foregroundColor(.white)
                        
                        Text(markerDetected ? "Keep marker visible to both devices" : "Point Vision Pro at this marker")
                            .font(.subheadline)
                            .foregroundColor(.white.opacity(0.8))
                    }
                    
                    // Motion status
                    HStack(spacing: 8) {
                        Circle()
                            .fill(multipeerManager.isPhoneStationary ? Color.green : Color.orange)
                            .frame(width: 8, height: 8)
                        Text(multipeerManager.isPhoneStationary ? "Steady - Collecting" : "Moving - Paused")
                            .font(.caption)
                            .foregroundColor(multipeerManager.isPhoneStationary ? .green : .orange)
                    }
                    .padding(.horizontal, 12)
                    .padding(.vertical, 6)
                    .background(Color.white.opacity(0.1))
                    .clipShape(Capsule())
                }
                .padding(.bottom, 50)
            }
        }
        .ignoresSafeArea()
    }
    
    private func resetHideTimer() {
        hideControlsTimer?.invalidate()
        hideControlsTimer = Timer.scheduledTimer(withTimeInterval: 4.0, repeats: false) { _ in
            withAnimation(.easeInOut(duration: 0.2)) {
                showControls = false
            }
        }
    }
}

// MARK: - Preview

// MARK: - ChArUco Calibration View

/// Displays the ChArUco pattern with intrinsic calibration progress
struct CharucoCalibrationView: View {
    let progress: (samples: Int, total: Int)
    let onDismiss: () -> Void
    
    @StateObject private var manager = CalibrationDisplayManager.shared
    @StateObject private var multipeerManager = MultipeerCalibrationManager_iOS.shared
    
    @State private var showControls = true
    @State private var showConfig = false
    @State private var hideControlsTimer: Timer?
    @State private var generatedImage: UIImage?
    
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                // White background
                Color.white.ignoresSafeArea()
                
                // ChArUco pattern - displayed at exact physical size
                if let image = generatedImage {
                    Image(uiImage: image)
                        .interpolation(.none)
                        .frame(width: image.size.width, height: image.size.height)
                }
                
                // Screen overflow warning
                if !manager.charucoFitsOnScreen {
                    VStack {
                        HStack {
                            Image(systemName: "exclamationmark.triangle.fill")
                            Text("Pattern exceeds screen size")
                        }
                        .font(.caption.bold())
                        .foregroundColor(.black)
                        .padding(.horizontal, 12)
                        .padding(.vertical, 8)
                        .background(Color.yellow)
                        .cornerRadius(8)
                        .padding(.top, 60)
                        
                        Spacer()
                    }
                }
                
                // Controls overlay
                if showControls {
                    controlsOverlay
                }
                
                // Config sheet
                if showConfig {
                    configSheet
                }
            }
            .onTapGesture {
                if !showConfig {
                    withAnimation(.easeInOut(duration: 0.2)) {
                        showControls.toggle()
                    }
                    if showControls {
                        resetHideTimer()
                    }
                }
            }
            .onAppear {
                regenerateBoard()
            }
            .onChange(of: manager.charucoConfig) { _, _ in
                regenerateBoard()
            }
        }
        .onAppear {
            resetHideTimer()
        }
        .onDisappear {
            hideControlsTimer?.invalidate()
        }
    }
    
    private func regenerateBoard() {
        generatedImage = manager.generateCharucoBoard(sizePoints: manager.charucoSizePoints)
    }
    
    private var configSheet: some View {
        VStack(spacing: 0) {
            Spacer()
            
            VStack(spacing: 16) {
                // Header
                HStack {
                    Text("ChArUco Configuration")
                        .font(.headline)
                    Spacer()
                    Button(action: { showConfig = false }) {
                        Image(systemName: "xmark.circle.fill")
                            .font(.title2)
                            .foregroundColor(.gray)
                    }
                }
                .padding(.horizontal)
                .padding(.top, 16)
                
                Divider()
                
                VStack(alignment: .leading, spacing: 16) {
                    // Grid configuration
                    HStack {
                        Text("Grid:")
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                        Spacer()
                        
                        Stepper("\(manager.charucoConfig.squaresX)×\(manager.charucoConfig.squaresY)", 
                                value: Binding(
                                    get: { manager.charucoConfig.squaresX },
                                    set: { manager.charucoConfig.squaresX = $0 }
                                ), in: 2...10)
                        .fixedSize()
                    }
                    
                    HStack {
                        Text("Rows:")
                        Spacer()
                        Stepper("\(manager.charucoConfig.squaresY)", 
                                value: Binding(
                                    get: { manager.charucoConfig.squaresY },
                                    set: { manager.charucoConfig.squaresY = $0 }
                                ), in: 2...10)
                        .fixedSize()
                    }
                    .font(.subheadline)
                    .foregroundColor(.secondary)
                    
                    // Square size
                    VStack(alignment: .leading, spacing: 4) {
                        HStack {
                            Text("Square Size:")
                            Spacer()
                            Text("\(Int(manager.charucoConfig.squareSizeMM))mm")
                                .foregroundColor(.primary)
                                .monospacedDigit()
                        }
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                        
                        Slider(
                            value: Binding(
                                get: { Double(manager.charucoConfig.squareSizeMM) },
                                set: { manager.charucoConfig.squareSizeMM = Float($0) }
                            ),
                            in: 10...40,
                            step: 1
                        )
                    }
                    
                    // Marker size
                    VStack(alignment: .leading, spacing: 4) {
                        HStack {
                            Text("Marker Size:")
                            Spacer()
                            Text("\(Int(manager.charucoConfig.markerSizeMM))mm")
                                .foregroundColor(.primary)
                                .monospacedDigit()
                        }
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                        
                        Slider(
                            value: Binding(
                                get: { Double(manager.charucoConfig.markerSizeMM) },
                                set: { manager.charucoConfig.markerSizeMM = Float($0) }
                            ),
                            in: 5...max(5, Double(manager.charucoConfig.squareSizeMM) - 2),
                            step: 1
                        )
                    }
                    
                    // Size info
                    HStack {
                        Text("Board Size:")
                        Spacer()
                        let boardW = manager.charucoConfig.patternWidthMM
                        let boardH = manager.charucoConfig.patternHeightMM
                        Text("\(Int(boardW))×\(Int(boardH))mm")
                            .foregroundColor(manager.charucoFitsOnScreen ? .green : .red)
                            .fontWeight(.medium)
                    }
                    .font(.subheadline)
                    
                    if !manager.charucoFitsOnScreen {
                        Text("⚠️ Pattern too large for screen (\(Int(manager.displaySpec.screenWidthMM))×\(Int(manager.displaySpec.screenHeightMM))mm)")
                            .font(.caption)
                            .foregroundColor(.red)
                    }
                }
                .padding(.horizontal)
                
                Divider()
                
                // Done button
                Button(action: { showConfig = false }) {
                    Text("Done")
                        .fontWeight(.semibold)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 12)
                }
                .buttonStyle(.borderedProminent)
                .padding(.horizontal)
                .padding(.bottom, 16)
            }
            .background(Color(.systemBackground))
            .clipShape(RoundedRectangle(cornerRadius: 20))
        }
        .background(Color.black.opacity(0.3).ignoresSafeArea())
    }
    
    private var controlsOverlay: some View {
        VStack {
            // Top gradient + close button
            ZStack(alignment: .top) {
                LinearGradient(
                    colors: [Color.black.opacity(0.5), Color.clear],
                    startPoint: .top,
                    endPoint: .bottom
                )
                .frame(height: 140)
                
                HStack {
                    Button(action: onDismiss) {
                        Image(systemName: "xmark")
                        .font(.system(size: 16, weight: .bold))
                        .foregroundColor(.white)
                        .frame(width: 36, height: 36)
                        .background(Color.black.opacity(0.5))
                        .clipShape(Circle())
                    }
                    
                    Spacer()
                    
                    // Connection indicator
                    HStack(spacing: 6) {
                        Image(systemName: "visionpro")
                            .font(.system(size: 12))
                        Text("Connected")
                            .font(.system(size: 12, weight: .medium))
                    }
                    .foregroundColor(.white)
                    .padding(.horizontal, 12)
                    .padding(.vertical, 6)
                    .background(Color.green.opacity(0.8))
                    .clipShape(Capsule())
                    
                    Spacer()
                    
                    // Config button
                    Button(action: { showConfig = true }) {
                        Image(systemName: "gearshape.fill")
                            .font(.system(size: 16, weight: .bold))
                            .foregroundColor(.white)
                            .frame(width: 36, height: 36)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Circle())
                    }
                }
                .padding(.horizontal, 20)
                .padding(.top, 16)
            }
            
            Spacer()
            
            // Bottom gradient + progress
            ZStack(alignment: .bottom) {
                LinearGradient(
                    colors: [Color.clear, Color.black.opacity(0.6)],
                    startPoint: .top,
                    endPoint: .bottom
                )
                .frame(height: 200)
                
                VStack(spacing: 16) {
                    // Progress ring (Purple for ChArUco)
                    ZStack {
                        Circle()
                            .stroke(Color.white.opacity(0.3), lineWidth: 6)
                            .frame(width: 80, height: 80)
                        
                        Circle()
                            .trim(from: 0, to: CGFloat(progress.samples) / CGFloat(progress.total))
                            .stroke(Color.purple, style: StrokeStyle(lineWidth: 6, lineCap: .round))
                            .frame(width: 80, height: 80)
                            .rotationEffect(.degrees(-90))
                        
                        VStack(spacing: 0) {
                            Text("\(progress.samples)")
                                .font(.system(size: 24, weight: .bold, design: .rounded))
                                .foregroundColor(.white)
                            Text("/\(progress.total)")
                                .font(.system(size: 12))
                                .foregroundColor(.white.opacity(0.7))
                        }
                    }
                    
                    Text("ChArUco Calibration")
                        .font(.headline)
                        .foregroundColor(.white)
                    
                    // Config summary
                    Text("\(manager.charucoConfig.squaresX)×\(manager.charucoConfig.squaresY) • \(Int(manager.charucoConfig.squareSizeMM))mm squares")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.7))
                    
                    // Motion status
                    HStack(spacing: 8) {
                        Circle()
                            .fill(multipeerManager.isPhoneStationary ? Color.green : Color.orange)
                            .frame(width: 8, height: 8)
                        Text(multipeerManager.isPhoneStationary ? "Steady" : "Moving")
                            .font(.caption)
                            .foregroundColor(multipeerManager.isPhoneStationary ? .green : .orange)
                    }
                    .padding(.horizontal, 12)
                    .padding(.vertical, 6)
                    .background(Color.white.opacity(0.1))
                    .clipShape(Capsule())
                }
                .padding(.bottom, 50)
            }
        }
        .ignoresSafeArea()
    }
    
    private func resetHideTimer() {
        hideControlsTimer?.invalidate()
        hideControlsTimer = Timer.scheduledTimer(withTimeInterval: 4.0, repeats: false) { _ in
            withAnimation(.easeInOut(duration: 0.2)) {
                showControls = false
            }
        }
    }
}

// MARK: - Verification Board Calibration View

/// View shown when Vision Pro commands verification board display
/// Displays a grid of ArUco tags for intrinsic calibration verification
struct VerificationBoardCalibrationView: View {
    let cols: Int
    let rows: Int
    let tagSizeMM: Float
    let marginMM: Float
    let tagIds: [Int]
    let onDismiss: () -> Void
    
    @StateObject private var manager = CalibrationDisplayManager.shared
    @State private var showControls = true
    @State private var hideControlsTimer: Timer?
    @State private var brightness: CGFloat = UIScreen.main.brightness
    
    // Computed sizes based on screen PPI
    private var tagSizePoints: CGFloat {
        let ppi = manager.displaySpec.ppi
        return CGFloat(tagSizeMM) / 25.4 * ppi
    }
    
    private var marginPoints: CGFloat {
        let ppi = manager.displaySpec.ppi
        return CGFloat(marginMM) / 25.4 * ppi
    }
    
    private var effectiveTagIds: [Int] {
        if tagIds.count >= cols * rows {
            return Array(tagIds.prefix(cols * rows))
        } else {
            return Array(0..<(cols * rows))
        }
    }
    
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                // White background
                Color.white
                    .ignoresSafeArea()
                
                // ArUco verification board pattern
                ArucoVerificationBoardPatternView(
                    columns: cols,
                    rows: rows,
                    tagSizePoints: tagSizePoints,
                    marginPoints: marginPoints,
                    tagIds: effectiveTagIds,
                    generateMarker: { id, size in
                        manager.generateArucoMarker(id: id, sizePoints: size)
                    }
                )
                .frame(maxWidth: geometry.size.width, maxHeight: geometry.size.height)
                
                // Controls overlay
                if showControls {
                    VStack {
                        // Top gradient
                        LinearGradient(
                            colors: [Color.black.opacity(0.6), Color.clear],
                            startPoint: .top,
                            endPoint: .bottom
                        )
                        .frame(height: 120)
                        .allowsHitTesting(false)
                        
                        Spacer()
                        
                        // Bottom gradient
                        LinearGradient(
                            colors: [Color.clear, Color.black.opacity(0.6)],
                            startPoint: .top,
                            endPoint: .bottom
                        )
                        .frame(height: 120)
                        .allowsHitTesting(false)
                    }
                    .ignoresSafeArea()
                    
                    VStack {
                        // Top bar with close button and info
                        HStack(alignment: .top) {
                            Button(action: onDismiss) {
                                Image(systemName: "xmark.circle.fill")
                                    .font(.system(size: 32))
                                    .foregroundStyle(.white.opacity(0.9), .white.opacity(0.2))
                            }
                            
                            Spacer()
                            
                            VStack(alignment: .trailing, spacing: 4) {
                                Text("Intrinsic Verification")
                                    .font(.system(size: 17, weight: .semibold))
                                    .foregroundColor(.white)
                                
                                Text("\(cols)×\(rows) ArUco Tags • \(Int(tagSizeMM))mm")
                                    .font(.system(size: 13))
                                    .foregroundColor(.white.opacity(0.7))
                            }
                        }
                        .padding(.horizontal, 20)
                        .padding(.top, 16)
                        
                        Spacer()
                        
                        // Bottom info
                        VStack(spacing: 8) {
                            Text("Verifying Calibration")
                                .font(.headline)
                                .foregroundColor(.white)
                            
                            Text("Vision Pro is measuring 3D distances")
                                .font(.caption)
                                .foregroundColor(.white.opacity(0.7))
                        }
                        .padding(.bottom, 50)
                    }
                }
            }
            .ignoresSafeArea()
            .onTapGesture {
                withAnimation(.easeInOut(duration: 0.2)) {
                    showControls.toggle()
                }
                if showControls {
                    resetHideTimer()
                }
            }
            .onAppear {
                brightness = UIScreen.main.brightness
                UIScreen.main.brightness = 1.0
                resetHideTimer()
            }
            .onDisappear {
                UIScreen.main.brightness = brightness
                hideControlsTimer?.invalidate()
            }
        }
    }
    
    private func resetHideTimer() {
        hideControlsTimer?.invalidate()
        hideControlsTimer = Timer.scheduledTimer(withTimeInterval: 4.0, repeats: false) { _ in
            withAnimation(.easeInOut(duration: 0.2)) {
                showControls = false
            }
        }
    }
}

#Preview {
    CalibrationCoordinatorView()
}
