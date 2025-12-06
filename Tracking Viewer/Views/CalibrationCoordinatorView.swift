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
                
            case .aruco(let markerId):
                // Show ArUco marker with progress
                ArucoCalibrationView(
                    markerId: markerId,
                    markerDetected: multipeerManager.markerDetectedByARKit,
                    progress: multipeerManager.extrinsicProgress,
                    readyForNext: multipeerManager.readyForNextPosition,
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

#Preview {
    CalibrationCoordinatorView()
}
