//
//  CameraCalibrationWizardView.swift
//  Tracking Streamer
//
//  A unified wizard for camera calibration that guides users through
//  both intrinsic and extrinsic calibration in a seamless flow.
//

import SwiftUI
import simd
import AVFoundation

// MARK: - Calibration Wizard State

/// Represents the current step in the calibration wizard
enum CalibrationWizardStep: Equatable {
    case setup                     // Initial setup questions
    case waitingForPhone           // Waiting for iPhone to connect via MPC
    case intrinsicCalibration      // Collecting checkerboard samples
    case intrinsicProcessing       // Computing intrinsic calibration
    case intrinsicComplete         // Intrinsic done, ready for extrinsic
    case extrinsicCalibration      // Collecting ArUco samples for each marker
    case extrinsicProcessing       // Computing extrinsic calibration
    case complete                  // All done!
    case error(String)            // Something went wrong
}

/// Represents the extrinsic calibration sub-step
enum ExtrinsicSubStep: Equatable {
    case waitingForARKit           // Waiting for ARKit to detect the marker
    case collectingSamples         // ARKit detected, collecting samples
    case readyForNextMarker        // Done with this marker, prompt to move
}

// MARK: - Calibration Wizard View

/// Main calibration wizard view that handles the entire calibration flow
struct CameraCalibrationWizardView: View {
    let onDismiss: () -> Void
    
    @StateObject private var calibrationManager = CameraCalibrationManager.shared
    @StateObject private var extrinsicManager = ExtrinsicCalibrationManager.shared
    @StateObject private var uvcManager = UVCCameraManager.shared
    @StateObject private var multipeerManager = MultipeerCalibrationManager.shared
    
    @State private var currentStep: CalibrationWizardStep = .setup
    @State private var extrinsicSubStep: ExtrinsicSubStep = .waitingForARKit
    
    // Setup options
    @State private var isStereoCamera: Bool = false
    @State private var knowsBaseline: Bool = false
    @State private var baselineMM: Float = 65.0
    @State private var enforceBaseline: Bool = false
    
    // Progress tracking
    @State private var intrinsicSamples: Int = 0
    @State private var extrinsicSamplesForCurrentMarker: Int = 0
    @State private var currentMarkerIndex: Int = 0
    @State private var totalMarkers: Int = 3
    
    // Animation states
    @State private var pulseAnimation: Bool = false
    @State private var showSuccessCheckmark: Bool = false
    
    // Timer for periodic updates
    @State private var updateTimer: Timer?
    
    private let targetIntrinsicSamples = 30
    private let targetExtrinsicSamplesPerMarker = 30
    
    var body: some View {
        VStack(spacing: 0) {
            // Header
            headerView
            
            Divider()
            
            // Main content
            ScrollView {
                VStack(spacing: 24) {
                    switch currentStep {
                    case .setup:
                        setupView
                    case .waitingForPhone:
                        waitingForPhoneView
                    case .intrinsicCalibration:
                        intrinsicCalibrationView
                    case .intrinsicProcessing:
                        processingView(title: "Computing Intrinsics", subtitle: "Optimizing camera parameters...")
                    case .intrinsicComplete:
                        intrinsicCompleteView
                    case .extrinsicCalibration:
                        extrinsicCalibrationView
                    case .extrinsicProcessing:
                        processingView(title: "Computing Extrinsics", subtitle: "Calculating head-to-camera transform...")
                    case .complete:
                        completeView
                    case .error(let message):
                        errorView(message: message)
                    }
                }
                .padding(24)
            }
            
            // Footer with action buttons
            footerView
        }
        .onAppear {
            startUpdateTimer()
            // Start MPC browsing
            multipeerManager.startBrowsing()
        }
        .onDisappear {
            updateTimer?.invalidate()
            multipeerManager.stopBrowsing()
            // Clean up frame callback
            uvcManager.onPixelBufferReceived = nil
            // Tell iPhone to hide calibration display
            multipeerManager.sendCommand(.hideDisplay)
            // Clean up if calibration was in progress
            if calibrationManager.isCalibrating {
                calibrationManager.cancelCalibration()
            }
            if extrinsicManager.isCalibrating {
                // Cancel extrinsic calibration if needed
            }
        }
    }
    
    // MARK: - Header View
    
    private var headerView: some View {
        HStack {
            VStack(alignment: .leading, spacing: 4) {
                Text("Camera Calibration")
                    .font(.title2)
                    .fontWeight(.bold)
                
                if let device = uvcManager.selectedDevice {
                    Text(device.name)
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                }
            }
            
            Spacer()
            
            // Progress indicator
            HStack(spacing: 16) {
                stepIndicator(step: 1, label: "Setup", isActive: currentStep == .setup, isComplete: currentStep != .setup)
                stepIndicator(step: 2, label: "Intrinsic", isActive: isIntrinsicStep, isComplete: isIntrinsicComplete)
                stepIndicator(step: 3, label: "Extrinsic", isActive: isExtrinsicStep, isComplete: currentStep == .complete)
            }
            
            Spacer()
            
            Button(action: onDismiss) {
                Image(systemName: "xmark")
                    .font(.system(size: 14, weight: .bold))
                    .foregroundColor(.secondary)
                    .frame(width: 32, height: 32)
                    .background(Color.secondary.opacity(0.2))
                    .clipShape(Circle())
            }
            .buttonStyle(.plain)
        }
        .padding(20)
        .background(Color.black.opacity(0.3))
    }
    
    private var isIntrinsicStep: Bool {
        switch currentStep {
        case .waitingForPhone, .intrinsicCalibration, .intrinsicProcessing, .intrinsicComplete:
            return true
        default:
            return false
        }
    }
    
    private var isIntrinsicComplete: Bool {
        switch currentStep {
        case .intrinsicComplete, .extrinsicCalibration, .extrinsicProcessing, .complete:
            return true
        default:
            return false
        }
    }
    
    private var isExtrinsicStep: Bool {
        switch currentStep {
        case .extrinsicCalibration, .extrinsicProcessing, .complete:
            return true
        default:
            return false
        }
    }
    
    private func stepIndicator(step: Int, label: String, isActive: Bool, isComplete: Bool) -> some View {
        VStack(spacing: 4) {
            ZStack {
                Circle()
                    .fill(isComplete ? Color.green : (isActive ? Color.blue : Color.secondary.opacity(0.3)))
                    .frame(width: 28, height: 28)
                
                if isComplete {
                    Image(systemName: "checkmark")
                        .font(.system(size: 12, weight: .bold))
                        .foregroundColor(.white)
                } else {
                    Text("\(step)")
                        .font(.system(size: 12, weight: .bold))
                        .foregroundColor(isActive ? .white : .secondary)
                }
            }
            
            Text(label)
                .font(.caption2)
                .foregroundColor(isActive ? .white : .secondary)
        }
    }
    
    // MARK: - Setup View
    
    private var setupView: some View {
        VStack(spacing: 24) {
            // Camera type selection
            VStack(alignment: .leading, spacing: 12) {
                Text("Camera Type")
                    .font(.headline)
                
                HStack(spacing: 16) {
                    cameraTypeButton(title: "Mono", subtitle: "Single lens", icon: "camera.fill", isSelected: !isStereoCamera) {
                        isStereoCamera = false
                    }
                    
                    cameraTypeButton(title: "Stereo", subtitle: "Dual lens (side-by-side)", icon: "camera.fill.badge.ellipsis", isSelected: isStereoCamera) {
                        isStereoCamera = true
                    }
                }
            }
            
            // Stereo baseline (only for stereo)
            if isStereoCamera {
                VStack(alignment: .leading, spacing: 12) {
                    Text("Stereo Baseline")
                        .font(.headline)
                    
                    Toggle("I know the camera baseline", isOn: $knowsBaseline)
                        .toggleStyle(SwitchToggleStyle(tint: .blue))
                    
                    if knowsBaseline {
                        HStack {
                            Text("Baseline:")
                            TextField("mm", value: $baselineMM, format: .number)
                                .textFieldStyle(.roundedBorder)
                                .frame(width: 80)
                            Text("mm")
                        }
                        
                        Toggle("Enforce baseline during extrinsic calibration", isOn: $enforceBaseline)
                            .toggleStyle(SwitchToggleStyle(tint: .blue))
                            .font(.subheadline)
                    }
                }
                .padding(16)
                .background(Color.white.opacity(0.1))
                .cornerRadius(12)
            }
            
            // Info card
            infoCard(
                icon: "info.circle.fill",
                title: "What you'll need",
                points: [
                    "iPhone with the Tracking Viewer app installed",
                    "Camera connected and streaming",
                    "Good lighting conditions"
                ],
                color: .blue
            )
        }
    }
    
    private func cameraTypeButton(title: String, subtitle: String, icon: String, isSelected: Bool, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            VStack(spacing: 8) {
                Image(systemName: icon)
                    .font(.system(size: 32))
                    .foregroundColor(isSelected ? .white : .secondary)
                
                VStack(spacing: 2) {
                    Text(title)
                        .font(.headline)
                        .foregroundColor(isSelected ? .white : .primary)
                    
                    Text(subtitle)
                        .font(.caption)
                        .foregroundColor(isSelected ? .white.opacity(0.8) : .secondary)
                }
            }
            .frame(maxWidth: .infinity)
            .padding(.vertical, 20)
            .background(isSelected ? Color.blue : Color.white.opacity(0.1))
            .cornerRadius(16)
            .overlay(
                RoundedRectangle(cornerRadius: 16)
                    .stroke(isSelected ? Color.blue : Color.clear, lineWidth: 2)
            )
        }
        .buttonStyle(.plain)
    }
    
    // MARK: - Waiting for Phone View
    
    private var waitingForPhoneView: some View {
        VStack(spacing: 32) {
            // Animated phone icon
            ZStack {
                Circle()
                    .stroke(Color.blue.opacity(0.2), lineWidth: 4)
                    .frame(width: 120, height: 120)
                
                Circle()
                    .stroke(Color.blue.opacity(pulseAnimation ? 0.0 : 0.5), lineWidth: 4)
                    .frame(width: 120, height: 120)
                    .scaleEffect(pulseAnimation ? 1.5 : 1.0)
                    .animation(.easeOut(duration: 1.5).repeatForever(autoreverses: false), value: pulseAnimation)
                
                Image(systemName: "iphone")
                    .font(.system(size: 48))
                    .foregroundColor(.blue)
            }
            .onAppear { pulseAnimation = true }
            
            VStack(spacing: 12) {
                Text("Waiting for iPhone")
                    .font(.title2)
                    .fontWeight(.semibold)
                
                Text("Open the Tracking Viewer app on your iPhone")
                    .font(.body)
                    .foregroundColor(.secondary)
                    .multilineTextAlignment(.center)
            }
            
            // Connection status
            HStack(spacing: 8) {
                if multipeerManager.isConnected {
                    Image(systemName: "checkmark.circle.fill")
                        .foregroundColor(.green)
                    Text("Connected to \(multipeerManager.connectedDeviceName ?? "iPhone")")
                        .foregroundColor(.green)
                } else {
                    ProgressView()
                        .scaleEffect(0.8)
                    Text(multipeerManager.connectionStatus)
                        .foregroundColor(.secondary)
                }
            }
            .font(.subheadline)
            .padding(.horizontal, 16)
            .padding(.vertical, 10)
            .background(Color.white.opacity(0.1))
            .cornerRadius(20)
            
            // Instructions
            infoCard(
                icon: "1.circle.fill",
                title: "Instructions",
                points: [
                    "Open Tracking Viewer on iPhone",
                    "Go to the Calibration tab",
                    "The app will connect automatically"
                ],
                color: .orange
            )
        }
        .onAppear {
            // When this view appears, start watching for connection
            checkPhoneConnection()
        }
        .onChange(of: multipeerManager.isConnected) { _, connected in
            if connected {
                // iPhone connected! Send command to show checkerboard
                multipeerManager.sendCommand(.showCheckerboard)
                
                // Start intrinsic calibration
                startIntrinsicCalibration()
            }
        }
    }
    
    // MARK: - Intrinsic Calibration View
    
    private var intrinsicCalibrationView: some View {
        VStack(spacing: 24) {
            // Big progress indicator
            ZStack {
                Circle()
                    .stroke(Color.white.opacity(0.2), lineWidth: 12)
                    .frame(width: 160, height: 160)
                
                Circle()
                    .trim(from: 0, to: CGFloat(intrinsicSamples) / CGFloat(targetIntrinsicSamples))
                    .stroke(Color.cyan, style: StrokeStyle(lineWidth: 12, lineCap: .round))
                    .frame(width: 160, height: 160)
                    .rotationEffect(.degrees(-90))
                    .animation(.spring(response: 0.3), value: intrinsicSamples)
                
                VStack(spacing: 4) {
                    Text("\(intrinsicSamples)")
                        .font(.system(size: 48, weight: .bold, design: .rounded))
                    Text("/ \(targetIntrinsicSamples)")
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                }
            }
            
            VStack(spacing: 8) {
                Text("Intrinsic Calibration")
                    .font(.title2)
                    .fontWeight(.semibold)
                
                Text("Move the camera around to capture the checkerboard pattern from different angles")
                    .font(.body)
                    .foregroundColor(.secondary)
                    .multilineTextAlignment(.center)
            }
            
            // Status from calibration manager
            HStack(spacing: 8) {
                Image(systemName: calibrationManager.samplesCollected > 0 ? "checkmark.circle.fill" : "circle")
                    .foregroundColor(calibrationManager.samplesCollected > 0 ? .green : .secondary)
                Text(calibrationManager.statusMessage.isEmpty ? "Looking for checkerboard..." : calibrationManager.statusMessage)
                    .foregroundColor(.secondary)
            }
            .font(.subheadline)
            .padding(.horizontal, 16)
            .padding(.vertical, 10)
            .background(Color.white.opacity(0.1))
            .cornerRadius(20)
            
            // Tips
            infoCard(
                icon: "lightbulb.fill",
                title: "Tips for good calibration",
                points: [
                    "Move slowly and steadily",
                    "Capture from different angles",
                    "Keep the pattern fully visible",
                    "Avoid motion blur"
                ],
                color: .yellow
            )
        }
    }
    
    // MARK: - Intrinsic Complete View
    
    private var intrinsicCompleteView: some View {
        VStack(spacing: 24) {
            // Success animation
            ZStack {
                Circle()
                    .fill(Color.green.opacity(0.2))
                    .frame(width: 120, height: 120)
                
                Image(systemName: "checkmark")
                    .font(.system(size: 48, weight: .bold))
                    .foregroundColor(.green)
                    .scaleEffect(showSuccessCheckmark ? 1.0 : 0.5)
                    .opacity(showSuccessCheckmark ? 1.0 : 0.0)
                    .animation(.spring(response: 0.4, dampingFraction: 0.6), value: showSuccessCheckmark)
            }
            .onAppear { showSuccessCheckmark = true }
            
            VStack(spacing: 8) {
                Text("Intrinsic Calibration Complete!")
                    .font(.title2)
                    .fontWeight(.semibold)
                
                // Show reprojection error
                if let calibration = calibrationManager.currentCalibration {
                    Text("Reprojection Error: \(String(format: "%.4f", calibration.leftIntrinsics.reprojectionError)) px")
                        .font(.subheadline)
                        .foregroundColor(calibration.leftIntrinsics.reprojectionError < 0.5 ? .green : .orange)
                }
            }
            
            // Next steps
            infoCard(
                icon: "arrow.right.circle.fill",
                title: "Next: Extrinsic Calibration",
                points: [
                    "The iPhone will now show ArUco markers",
                    "Look at the marker with Vision Pro",
                    "Point the camera at the marker",
                    "Repeat for 3 different marker positions"
                ],
                color: .purple
            )
        }
    }
    
    // MARK: - Extrinsic Calibration View
    
    private var extrinsicCalibrationView: some View {
        VStack(spacing: 24) {
            // Marker progress indicators
            HStack(spacing: 20) {
                ForEach(0..<totalMarkers, id: \.self) { index in
                    markerProgressView(index: index)
                }
            }
            
            // Current marker status
            switch extrinsicSubStep {
            case .waitingForARKit:
                waitingForARKitView
            case .collectingSamples:
                collectingSamplesView
            case .readyForNextMarker:
                readyForNextMarkerView
            }
        }
    }
    
    private func markerProgressView(index: Int) -> some View {
        VStack(spacing: 8) {
            ZStack {
                Circle()
                    .stroke(Color.white.opacity(0.2), lineWidth: 4)
                    .frame(width: 60, height: 60)
                
                if index < currentMarkerIndex {
                    // Completed
                    Circle()
                        .fill(Color.green)
                        .frame(width: 60, height: 60)
                    Image(systemName: "checkmark")
                        .font(.system(size: 24, weight: .bold))
                        .foregroundColor(.white)
                } else if index == currentMarkerIndex {
                    // Current
                    Circle()
                        .trim(from: 0, to: CGFloat(extrinsicSamplesForCurrentMarker) / CGFloat(targetExtrinsicSamplesPerMarker))
                        .stroke(Color.purple, style: StrokeStyle(lineWidth: 4, lineCap: .round))
                        .frame(width: 60, height: 60)
                        .rotationEffect(.degrees(-90))
                    
                    Text("\(extrinsicSamplesForCurrentMarker)")
                        .font(.system(size: 18, weight: .bold, design: .rounded))
                } else {
                    // Not started
                    Text("\(index + 1)")
                        .font(.system(size: 18, weight: .bold))
                        .foregroundColor(.secondary)
                }
            }
            
            Text("Position \(index + 1)")
                .font(.caption)
                .foregroundColor(index == currentMarkerIndex ? .white : .secondary)
        }
    }
    
    private var waitingForARKitView: some View {
        VStack(spacing: 20) {
            // Animated look icon
            Image(systemName: "eye.fill")
                .font(.system(size: 48))
                .foregroundColor(.purple)
                .symbolEffect(.pulse, options: .repeating)
            
            VStack(spacing: 8) {
                Text("Look at the Marker")
                    .font(.title3)
                    .fontWeight(.semibold)
                
                Text("Move closer to the iPhone so ARKit can detect the ArUco marker")
                    .font(.body)
                    .foregroundColor(.secondary)
                    .multilineTextAlignment(.center)
            }
            
            // Status
            HStack(spacing: 8) {
                ProgressView()
                    .scaleEffect(0.8)
                Text("Waiting for ARKit detection...")
                    .foregroundColor(.secondary)
            }
            .font(.subheadline)
            .padding(.horizontal, 16)
            .padding(.vertical, 10)
            .background(Color.white.opacity(0.1))
            .cornerRadius(20)
        }
    }
    
    private var collectingSamplesView: some View {
        VStack(spacing: 20) {
            // Big progress
            ZStack {
                Circle()
                    .stroke(Color.white.opacity(0.2), lineWidth: 8)
                    .frame(width: 120, height: 120)
                
                Circle()
                    .trim(from: 0, to: CGFloat(extrinsicSamplesForCurrentMarker) / CGFloat(targetExtrinsicSamplesPerMarker))
                    .stroke(Color.purple, style: StrokeStyle(lineWidth: 8, lineCap: .round))
                    .frame(width: 120, height: 120)
                    .rotationEffect(.degrees(-90))
                    .animation(.spring(response: 0.3), value: extrinsicSamplesForCurrentMarker)
                
                VStack(spacing: 2) {
                    Text("\(extrinsicSamplesForCurrentMarker)")
                        .font(.system(size: 36, weight: .bold, design: .rounded))
                    Text("/ \(targetExtrinsicSamplesPerMarker)")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            
            VStack(spacing: 8) {
                Text("Collecting Samples")
                    .font(.title3)
                    .fontWeight(.semibold)
                
                Text("Move your head slightly while keeping the marker visible to both ARKit and the camera")
                    .font(.body)
                    .foregroundColor(.secondary)
                    .multilineTextAlignment(.center)
            }
            
            // Status
            if !multipeerManager.isPhoneStationary {
                HStack(spacing: 8) {
                    Image(systemName: "exclamationmark.triangle.fill")
                        .foregroundColor(.orange)
                    Text("iPhone is moving - collection paused")
                        .foregroundColor(.orange)
                }
                .font(.subheadline)
                .padding(.horizontal, 16)
                .padding(.vertical, 10)
                .background(Color.orange.opacity(0.2))
                .cornerRadius(20)
            }
        }
    }
    
    private var readyForNextMarkerView: some View {
        VStack(spacing: 20) {
            // Success for this marker
            ZStack {
                Circle()
                    .fill(Color.green.opacity(0.2))
                    .frame(width: 100, height: 100)
                
                Image(systemName: "checkmark")
                    .font(.system(size: 40, weight: .bold))
                    .foregroundColor(.green)
            }
            
            VStack(spacing: 8) {
                Text("Position \(currentMarkerIndex + 1) Complete!")
                    .font(.title3)
                    .fontWeight(.semibold)
                
                if currentMarkerIndex < totalMarkers - 1 {
                    Text("Move the iPhone to a **new physical location** and hold it still")
                        .font(.body)
                        .foregroundColor(.secondary)
                        .multilineTextAlignment(.center)
                }
            }
            
            // Big move icon
            if currentMarkerIndex < totalMarkers - 1 {
                HStack(spacing: 16) {
                    Image(systemName: "iphone")
                        .font(.system(size: 32))
                    Image(systemName: "arrow.right")
                        .font(.system(size: 24))
                    Image(systemName: "location.fill")
                        .font(.system(size: 32))
                }
                .foregroundColor(.blue)
                .padding(20)
                .background(Color.blue.opacity(0.2))
                .cornerRadius(16)
            }
        }
    }
    
    // MARK: - Processing View
    
    private func processingView(title: String, subtitle: String) -> some View {
        VStack(spacing: 24) {
            ProgressView()
                .scaleEffect(1.5)
            
            VStack(spacing: 8) {
                Text(title)
                    .font(.title2)
                    .fontWeight(.semibold)
                
                Text(subtitle)
                    .font(.body)
                    .foregroundColor(.secondary)
            }
        }
        .padding(40)
    }
    
    // MARK: - Complete View
    
    private var completeView: some View {
        VStack(spacing: 24) {
            // Big success
            ZStack {
                Circle()
                    .fill(Color.green.opacity(0.2))
                    .frame(width: 140, height: 140)
                
                Circle()
                    .fill(Color.green)
                    .frame(width: 100, height: 100)
                
                Image(systemName: "checkmark")
                    .font(.system(size: 48, weight: .bold))
                    .foregroundColor(.white)
            }
            
            VStack(spacing: 8) {
                Text("Calibration Complete!")
                    .font(.title)
                    .fontWeight(.bold)
                
                Text("Your camera is now calibrated and ready for use")
                    .font(.body)
                    .foregroundColor(.secondary)
            }
            
            // Results summary
            if let intrinsic = calibrationManager.currentCalibration,
               let extrinsic = extrinsicManager.currentCalibration {
                VStack(spacing: 12) {
                    resultRow(label: "Intrinsic Error", value: String(format: "%.4f px", intrinsic.leftIntrinsics.reprojectionError), isGood: intrinsic.leftIntrinsics.reprojectionError < 0.5)
                    resultRow(label: "Extrinsic Error", value: String(format: "%.4f m", extrinsic.leftReprojectionError), isGood: extrinsic.leftReprojectionError < 0.01)
                    resultRow(label: "Samples Used", value: "\(intrinsic.sampleCount) + \(extrinsic.sampleCount)", isGood: true)
                }
                .padding(16)
                .background(Color.white.opacity(0.1))
                .cornerRadius(12)
            }
        }
    }
    
    private func resultRow(label: String, value: String, isGood: Bool) -> some View {
        HStack {
            Text(label)
                .foregroundColor(.secondary)
            Spacer()
            HStack(spacing: 4) {
                Image(systemName: isGood ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
                    .foregroundColor(isGood ? .green : .orange)
                Text(value)
                    .fontWeight(.medium)
                    .foregroundColor(isGood ? .green : .orange)
            }
        }
        .font(.subheadline)
    }
    
    // MARK: - Error View
    
    private func errorView(message: String) -> some View {
        VStack(spacing: 24) {
            ZStack {
                Circle()
                    .fill(Color.red.opacity(0.2))
                    .frame(width: 100, height: 100)
                
                Image(systemName: "exclamationmark.triangle.fill")
                    .font(.system(size: 40))
                    .foregroundColor(.red)
            }
            
            VStack(spacing: 8) {
                Text("Calibration Error")
                    .font(.title2)
                    .fontWeight(.semibold)
                
                Text(message)
                    .font(.body)
                    .foregroundColor(.secondary)
                    .multilineTextAlignment(.center)
            }
        }
    }
    
    // MARK: - Footer View
    
    private var footerView: some View {
        HStack(spacing: 16) {
            // Cancel button (always visible except on complete)
            if currentStep != .complete {
                Button(action: {
                    cancelCalibration()
                    onDismiss()
                }) {
                    Text("Cancel")
                        .font(.headline)
                        .foregroundColor(.secondary)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(Color.secondary.opacity(0.2))
                        .cornerRadius(12)
                }
                .buttonStyle(.plain)
            }
            
            // Primary action button
            Button(action: primaryAction) {
                HStack(spacing: 8) {
                    if currentStep == .complete {
                        Image(systemName: "checkmark")
                    }
                    Text(primaryButtonTitle)
                }
                .font(.headline)
                .foregroundColor(.white)
                .frame(maxWidth: .infinity)
                .padding(.vertical, 14)
                .background(primaryButtonEnabled ? Color.blue : Color.gray)
                .cornerRadius(12)
            }
            .buttonStyle(.plain)
            .disabled(!primaryButtonEnabled)
        }
        .padding(20)
        .background(Color.black.opacity(0.3))
    }
    
    private var primaryButtonTitle: String {
        switch currentStep {
        case .setup:
            return "Start Calibration"
        case .waitingForPhone:
            return "Waiting..."
        case .intrinsicCalibration:
            return intrinsicSamples >= targetIntrinsicSamples ? "Finish Intrinsic" : "Collecting..."
        case .intrinsicProcessing:
            return "Processing..."
        case .intrinsicComplete:
            return "Continue to Extrinsic"
        case .extrinsicCalibration:
            if extrinsicSubStep == .readyForNextMarker {
                return currentMarkerIndex < totalMarkers - 1 ? "Next Position" : "Finish Calibration"
            }
            return "Collecting..."
        case .extrinsicProcessing:
            return "Processing..."
        case .complete:
            return "Done"
        case .error:
            return "Retry"
        }
    }
    
    private var primaryButtonEnabled: Bool {
        switch currentStep {
        case .setup:
            return uvcManager.selectedDevice != nil
        case .waitingForPhone:
            return false
        case .intrinsicCalibration:
            return intrinsicSamples >= targetIntrinsicSamples
        case .intrinsicProcessing:
            return false
        case .intrinsicComplete:
            return true
        case .extrinsicCalibration:
            if extrinsicSubStep == .readyForNextMarker {
                return true
            }
            return false
        case .extrinsicProcessing:
            return false
        case .complete:
            return true
        case .error:
            return true
        }
    }
    
    private func primaryAction() {
        switch currentStep {
        case .setup:
            currentStep = .waitingForPhone
        case .intrinsicCalibration:
            finishIntrinsicCalibration()
        case .intrinsicComplete:
            startExtrinsicCalibration()
        case .extrinsicCalibration:
            if extrinsicSubStep == .readyForNextMarker {
                if currentMarkerIndex < totalMarkers - 1 {
                    advanceToNextMarker()
                } else {
                    finishExtrinsicCalibration()
                }
            }
        case .complete:
            onDismiss()
        case .error:
            currentStep = .setup
        default:
            break
        }
    }
    
    // MARK: - Helper Views
    
    private func infoCard(icon: String, title: String, points: [String], color: Color) -> some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack(spacing: 8) {
                Image(systemName: icon)
                    .foregroundColor(color)
                Text(title)
                    .font(.headline)
            }
            
            VStack(alignment: .leading, spacing: 8) {
                ForEach(points, id: \.self) { point in
                    HStack(alignment: .top, spacing: 8) {
                        Image(systemName: "circle.fill")
                            .font(.system(size: 6))
                            .foregroundColor(.secondary)
                            .padding(.top, 6)
                        Text(point)
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                    }
                }
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(16)
        .background(Color.white.opacity(0.1))
        .cornerRadius(12)
    }
    
    // MARK: - Actions
    
    private func startUpdateTimer() {
        updateTimer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { _ in
            Task { @MainActor in
                updateProgress()
            }
        }
    }
    
    private func updateProgress() {
        // Update intrinsic samples
        intrinsicSamples = calibrationManager.samplesCollected
        
        // Update extrinsic samples
        extrinsicSamplesForCurrentMarker = extrinsicManager.samplesForCurrentMarker
        currentMarkerIndex = extrinsicManager.currentMarkerIndex
        
        // Check for step transitions - auto-advance when intrinsic is complete
        if currentStep == .intrinsicCalibration && intrinsicSamples >= targetIntrinsicSamples {
            // Auto-finish intrinsic calibration
            finishIntrinsicCalibration()
        }
        
        if currentStep == .extrinsicCalibration {
            // Check ARKit detection FOR THE CURRENT MARKER specifically
            let currentMarkerId = extrinsicManager.currentMarkerId
            let hasARKitDetectionForCurrentMarker = extrinsicManager.rememberedMarkerPositions[currentMarkerId] != nil || 
                                                     extrinsicManager.arkitTrackedMarkers[currentMarkerId] != nil
            let hasCameraDetectionForCurrentMarker = extrinsicManager.cameraDetectedMarkers[currentMarkerId] != nil
            
            if extrinsicSubStep == .waitingForARKit && hasARKitDetectionForCurrentMarker && hasCameraDetectionForCurrentMarker {
                // ARKit detected the current marker! Start collecting
                extrinsicSubStep = .collectingSamples
                // Play sound
                playDetectionSound()
            }
            
            if extrinsicSubStep == .collectingSamples && extrinsicSamplesForCurrentMarker >= targetExtrinsicSamplesPerMarker {
                // Done with this marker
                extrinsicSubStep = .readyForNextMarker
                // Play sound
                playCompletionSound()
            }
        }
    }
    
    private func checkPhoneConnection() {
        // If already connected, proceed
        if multipeerManager.isConnected {
            multipeerManager.sendCommand(.showCheckerboard)
            startIntrinsicCalibration()
        }
    }
    
    private func setupIntrinsicFrameCallback() {
        dlog("🔧 [CalibrationWizard] Setting up INTRINSIC frame callback...")
        
        uvcManager.onPixelBufferReceived = { [weak calibrationManager] pixelBuffer in
            guard let manager = calibrationManager else { return }
            guard manager.isCalibrating else { return }
            
            Task { @MainActor in
                // Process frame based on stereo/mono mode
                if self.isStereoCamera {
                    _ = manager.processStereoFrame(pixelBuffer)
                } else {
                    _ = manager.processMonoFrame(pixelBuffer)
                }
            }
        }
        
        dlog("✅ [CalibrationWizard] INTRINSIC frame callback set up")
    }
    
    private func setupExtrinsicFrameCallback() {
        dlog("🔧 [CalibrationWizard] Setting up EXTRINSIC frame callback...")
        
        guard let device = uvcManager.selectedDevice,
              let intrinsicData = calibrationManager.allCalibrations[device.id] else {
            dlog("❌ [CalibrationWizard] No intrinsic calibration available for extrinsic!")
            return
        }
        
        uvcManager.onPixelBufferReceived = { [weak extrinsicManager] pixelBuffer in
            guard let manager = extrinsicManager else { return }
            guard manager.isCalibrating else { return }
            
            manager.processCameraFrame(
                pixelBuffer,
                intrinsics: intrinsicData.leftIntrinsics,
                rightIntrinsics: intrinsicData.rightIntrinsics
            )
        }
        
        dlog("✅ [CalibrationWizard] EXTRINSIC frame callback set up")
    }
    
    private func startIntrinsicCalibration() {
        guard let device = uvcManager.selectedDevice else { return }
        
        // Ensure camera is capturing
        if !uvcManager.isCapturing {
            uvcManager.startCapture()
        }
        
        // Set up frame callback for processing
        setupIntrinsicFrameCallback()
        
        calibrationManager.startCalibration(
            deviceId: device.id,
            deviceName: device.name,
            isStereo: isStereoCamera
        )
        
        currentStep = .intrinsicCalibration
    }
    
    private func finishIntrinsicCalibration() {
        guard let device = uvcManager.selectedDevice else { return }
        
        currentStep = .intrinsicProcessing
        
        // Compute calibration in background (using async version to not block UI)
        Task {
            let result = await calibrationManager.finishCalibrationAsync(
                deviceId: device.id,
                deviceName: device.name,
                isStereo: isStereoCamera
            )
            
            await MainActor.run {
                if result != nil {
                    currentStep = .intrinsicComplete
                    showSuccessCheckmark = false // Reset for animation
                } else {
                    currentStep = .error(calibrationManager.lastError ?? "Unknown error")
                }
            }
        }
    }
    
    private func startExtrinsicCalibration() {
        guard let device = uvcManager.selectedDevice else { return }
        
        // Configure baseline if stereo
        if isStereoCamera && knowsBaseline && enforceBaseline {
            extrinsicManager.knownStereoBaseline = baselineMM / 1000.0
        } else {
            extrinsicManager.knownStereoBaseline = nil
        }
        
        // Tell iPhone to show ArUco marker
        multipeerManager.sendCommand(.showAruco(markerId: extrinsicManager.markerIds.first ?? 0))
        
        // Set up frame callback for ArUco detection
        setupExtrinsicFrameCallback()
        
        // Start extrinsic calibration
        Task {
            await extrinsicManager.startCalibration(
                deviceId: device.id,
                deviceName: device.name,
                isStereo: isStereoCamera
            )
            
            await MainActor.run {
                currentStep = .extrinsicCalibration
                extrinsicSubStep = .waitingForARKit
                totalMarkers = extrinsicManager.markerIds.count
            }
        }
    }
    
    private func advanceToNextMarker() {
        extrinsicManager.advanceToNextMarker()
        
        // Tell iPhone to show next marker
        let nextMarkerId = extrinsicManager.currentMarkerId
        multipeerManager.sendCommand(.showAruco(markerId: nextMarkerId))
        
        extrinsicSubStep = .waitingForARKit
        extrinsicSamplesForCurrentMarker = 0
    }
    
    private func finishExtrinsicCalibration() {
        guard let device = uvcManager.selectedDevice else { return }
        
        currentStep = .extrinsicProcessing
        
        // Compute calibration
        Task {
            let result = extrinsicManager.finishCalibration(
                deviceId: device.id,
                deviceName: device.name,
                isStereo: isStereoCamera
            )
            
            await MainActor.run {
                if result != nil {
                    // Tell iPhone calibration is complete
                    multipeerManager.sendCommand(.calibrationComplete)
                    currentStep = .complete
                } else {
                    currentStep = .error(extrinsicManager.lastError ?? "Unknown error")
                }
            }
        }
    }
    
    private func cancelCalibration() {
        calibrationManager.cancelCalibration()
        multipeerManager.sendCommand(.calibrationComplete)
    }
    
    private func playDetectionSound() {
        // System sound for ARKit detection
        AudioServicesPlaySystemSound(1057) // Short tick
    }
    
    private func playCompletionSound() {
        // System sound for marker completion
        AudioServicesPlaySystemSound(1025) // Success sound
    }
}

// MARK: - Preview

#Preview {
    CameraCalibrationWizardView(onDismiss: {})
        .frame(width: 500, height: 700)
        .background(.regularMaterial)
}
