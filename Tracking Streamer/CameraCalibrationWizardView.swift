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
import UIKit

// MARK: - ShareSheet Helper

/// UIActivityViewController wrapper for SwiftUI
struct ShareSheet: UIViewControllerRepresentable {
    let activityItems: [Any]
    
    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: activityItems, applicationActivities: nil)
    }
    
    func updateUIViewController(_ uiViewController: UIActivityViewController, context: Context) {}
}

// MARK: - Calibration Wizard State

/// Represents the current step in the calibration wizard
enum CalibrationWizardStep: Equatable {
    case setup                     // Initial setup questions
    case waitingForPhone           // Waiting for iPhone to connect via MPC
    case intrinsicCalibration      // Collecting checkerboard samples
    case intrinsicProcessing       // Computing intrinsic calibration
    case intrinsicComplete         // Intrinsic done, ready for verification
    case intrinsicVerification     // Verifying intrinsics with ArUco board distance measurement
    case extrinsicCalibration      // Collecting ArUco samples for each marker
    case extrinsicProcessing       // Computing extrinsic calibration
    case verification              // Verifying calibration in AR
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
    var initialStep: CalibrationWizardStep = .setup
    
    @StateObject private var calibrationManager = CameraCalibrationManager.shared
    @StateObject private var extrinsicManager = ExtrinsicCalibrationManager.shared
    @StateObject private var uvcManager = UVCCameraManager.shared
    @StateObject private var multipeerManager = MultipeerCalibrationManager.shared
    @StateObject private var debugRecorder = CalibrationDebugRecorder.shared
    

    
    @State private var currentStep: CalibrationWizardStep = .setup
    @State private var extrinsicSubStep: ExtrinsicSubStep = .waitingForARKit
    
    // Setup options
    @State private var isStereoCamera: Bool = false
    @State private var knowsBaseline: Bool = false
    @State private var baselineMM: Float = 65.0
    @State private var enforceBaseline: Bool = false
    
    // Intrinsic Configuration
    @State private var intrinsicPattern: CalibrationMode = .charuco
    
    // Progress tracking
    @State private var intrinsicSamples: Int = 0
    @State private var leftIntrinsicSamples: Int = 0
    @State private var rightIntrinsicSamples: Int = 0
    @State private var extrinsicSamplesForCurrentMarker: Int = 0
    @State private var currentMarkerIndex: Int = 0
    @State private var totalMarkers: Int = 3
    
    // Verification state
    @State private var hasVerifiedLeft: Bool = false
    @State private var hasVerifiedRight: Bool = false
    
    // Intrinsic verification state
    @State private var verificationPassed: Bool = false
    @State private var verificationAverageError: Double = 0.0
    @State private var verificationMaxError: Double = 0.0
    @State private var verificationDistances: [(pair: String, measured: Double, expected: Double, error: Double)] = []
    @State private var detectedTagCount: Int = 0
    
    // Animation states
    @State private var pulseAnimation: Bool = false
    @State private var showSuccessCheckmark: Bool = false
    
    // Timer for periodic updates
    @State private var updateTimer: Timer?
    
    // Debug recording
    @State private var enableDebugRecording: Bool = false
    @State private var showShareSheet: Bool = false
    @State private var shareURL: URL?
    
    private let targetIntrinsicSamples = 30
    private let targetExtrinsicSamplesPerMarker = 30
    
    // Verification board configuration (3x1 grid, 40mm tags, 6mm margin)
    private let verificationCols = 1
    private let verificationRows = 3
    private let verificationTagSizeMM: Float = 40.0
    private let verificationMarginMM: Float = 6.0
    private var verificationTagIds: [Int] { Array(0..<(verificationCols * verificationRows)) }
    
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
                    case .intrinsicVerification:
                        intrinsicVerificationView
                    case .extrinsicCalibration:
                        extrinsicCalibrationView
                    case .extrinsicProcessing:
                        processingView(title: "Computing Extrinsics", subtitle: "Calculating head-to-camera transform...")
                    case .verification:
                        verificationView
                    case .complete:
                        completeView
                    case .error(let message):
                        errorView(message: message)
                    }
                }
                .padding(24)
            }
            
            // Footer with action buttons
            if currentStep != .verification {
                footerView
            }
        }
        .onAppear {
            if initialStep == .verification {
                verifyExistingCalibration()
            } else {
                startUpdateTimer()
            }
            // Start MPC browsing
            multipeerManager.startBrowsing()
            // Mark wizard as active so status view minimizes
            DataManager.shared.isCalibrationWizardActive = true
        }
        .onDisappear {
            updateTimer?.invalidate()
            multipeerManager.stopBrowsing()
            // Clean up frame callback
            uvcManager.onPixelBufferReceived = nil
            // Tell iPhone to hide calibration display
            multipeerManager.sendCommand(.hideDisplay)
            // Mark wizard as inactive
            DataManager.shared.isCalibrationWizardActive = false
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
                stepIndicator(step: 3, label: "Verify", isActive: currentStep == .intrinsicVerification, isComplete: isVerificationComplete)
                stepIndicator(step: 4, label: "Extrinsic", isActive: isExtrinsicStep, isComplete: currentStep == .complete)
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
        case .intrinsicComplete, .intrinsicVerification, .extrinsicCalibration, .extrinsicProcessing, .complete:
            return true
        default:
            return false
        }
    }
    
    private var isVerificationComplete: Bool {
        switch currentStep {
        case .extrinsicCalibration, .extrinsicProcessing, .complete:
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
            
            // Intrinsic Pattern selection
            VStack(alignment: .leading, spacing: 12) {
                Text("Calibration Pattern")
                    .font(.headline)
                
                Picker("Pattern", selection: $intrinsicPattern) {
                    Text("ChArUco Board").tag(CalibrationMode.charuco)
                    Text("Checkerboard").tag(CalibrationMode.checkerboard)
                }
                .pickerStyle(.segmented)
                
                Text(intrinsicPattern == .charuco ? "ChArUco provides better accuracy and detection even with partial occlusion." : "Standard checkerboard pattern.")
                    .font(.caption)
                    .foregroundColor(.secondary)
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
            
            // Debug recording toggle
            VStack(alignment: .leading, spacing: 8) {
                Toggle("Record Debug Data", isOn: $enableDebugRecording)
                    .toggleStyle(SwitchToggleStyle(tint: .orange))
                
                if enableDebugRecording {
                    Text("Saves camera frames and pose matrices for Python analysis")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            .padding(12)
            .background(Color.orange.opacity(0.1))
            .cornerRadius(10)
            
            // Allow verifying existing calibration if available
            if let currentCal = extrinsicManager.currentCalibration {
                Button(action: verifyExistingCalibration) {
                    HStack {
                        Image(systemName: "checkmark.circle.fill")
                        Text("Verify Existing Calibration")
                            .fontWeight(.medium)
                    }
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 14)
                    .background(Color.green.opacity(0.15))
                    .cornerRadius(12)
                    .overlay(
                        RoundedRectangle(cornerRadius: 12)
                            .stroke(Color.green.opacity(0.5), lineWidth: 1)
                    )
                }
                .buttonStyle(.plain)
            }
        }
    }
    
    private func verifyExistingCalibration() {
        guard let calibration = extrinsicManager.currentCalibration else { return }
        guard let device = uvcManager.selectedDevice,
              let intrinsicData = calibrationManager.allCalibrations[device.id] else {
            dlog("❌ [CalibrationWizard] No intrinsic calibration for verification!")
            return
        }
        
        // Setup internal state for verification
        extrinsicManager.proposedCalibration = calibration
        extrinsicManager.isStereoCalibration = calibration.isStereo
        extrinsicManager.verificationSide = .left
        
        // Setup UVC frame callback for verification (CRITICAL!)
        let extrinsicManagerRef = extrinsicManager
        uvcManager.onPixelBufferReceived = { [weak extrinsicManagerRef] pixelBuffer in
            guard let manager = extrinsicManagerRef else { return }
            guard manager.isCalibrating else { return }
            
            manager.processCameraFrame(
                pixelBuffer,
                intrinsics: intrinsicData.leftIntrinsics,
                rightIntrinsics: intrinsicData.rightIntrinsics
            )
        }
        
        // Ensure camera is capturing
        if !uvcManager.isCapturing {
            uvcManager.startCapture()
        }
        
        // Start AR session and transition
        Task {
            await extrinsicManager.startVerificationSession(isStereo: calibration.isStereo)
            await MainActor.run {
                currentStep = .verification
            }
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
                // iPhone connected! Send command to show chosen pattern
                if intrinsicPattern == .charuco {
                    multipeerManager.sendCommand(.showCharuco)
                } else {
                    multipeerManager.sendCommand(.showCheckerboard)
                }
                
                // Start intrinsic calibration
                startIntrinsicCalibration()
            }
        }
    }
    
    // MARK: - Intrinsic Calibration View
    
    private var intrinsicCalibrationView: some View {
        VStack(spacing: 20) {
            // UVC camera preview with visualization overlay
            ZStack(alignment: .bottom) {
                if let frame = calibrationManager.previewWithVisualization ?? calibrationManager.previewFrame {
                    Image(uiImage: frame)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .frame(maxHeight: 200)
                        .cornerRadius(12)
                        .overlay(
                            RoundedRectangle(cornerRadius: 12)
                                .stroke(calibrationManager.detectionFeedback.isDetected ? Color.green.opacity(0.7) : Color.cyan.opacity(0.5), lineWidth: 2)
                        )
                }
                
                // Guidance text overlay
                let feedback = calibrationManager.detectionFeedback
                HStack(spacing: 6) {
                    Image(systemName: feedback.isDetected ? (feedback.isTooFar ? "exclamationmark.triangle.fill" : "checkmark.circle.fill") : "eye.fill")
                        .foregroundColor(feedback.isDetected ? (feedback.isTooFar ? .orange : .green) : .white)
                    Text(feedback.guidanceMessage)
                        .fontWeight(.medium)
                        .foregroundColor(feedback.isDetected ? (feedback.isTooFar ? .orange : .green) : .white)
                }
                .font(.subheadline)
                .padding(.horizontal, 12)
                .padding(.vertical, 6)
                .background(Color.black.opacity(0.7))
                .cornerRadius(8)
                .padding(.bottom, 8)
            }
            
            // Progress indicator(s) - showing stability % with linear bars
            if isStereoCamera {
                // Stereo: show left and right stability with horizontal bars
                VStack(spacing: 12) {
                    // Left camera stability bar
                    HStack(spacing: 8) {
                        Text("L")
                            .font(.system(size: 14, weight: .bold, design: .rounded))
                            .foregroundColor(calibrationManager.leftReady ? .green : .cyan)
                            .frame(width: 20)
                        
                        GeometryReader { geo in
                            ZStack(alignment: .leading) {
                                // Background
                                RoundedRectangle(cornerRadius: 4)
                                    .fill(Color.white.opacity(0.2))
                                
                                // Progress fill
                                RoundedRectangle(cornerRadius: 4)
                                    .fill(calibrationManager.leftReady ? Color.green : Color.cyan)
                                    .frame(width: geo.size.width * CGFloat(calibrationManager.leftStability) / 100.0)
                                    .animation(.spring(response: 0.3), value: calibrationManager.leftStability)
                            }
                        }
                        .frame(height: 8)
                        
                        Text("\(Int(calibrationManager.leftStability))%")
                            .font(.system(size: 12, weight: .semibold, design: .rounded))
                            .foregroundColor(calibrationManager.leftReady ? .green : .white)
                            .frame(width: 40, alignment: .trailing)
                        
                        Text("n=\(leftIntrinsicSamples)")
                            .font(.system(size: 10, design: .rounded))
                            .foregroundColor(.secondary)
                            .frame(width: 45, alignment: .trailing)
                    }
                    
                    // Right camera stability bar
                    HStack(spacing: 8) {
                        Text("R")
                            .font(.system(size: 14, weight: .bold, design: .rounded))
                            .foregroundColor(calibrationManager.rightReady ? .green : .purple)
                            .frame(width: 20)
                        
                        GeometryReader { geo in
                            ZStack(alignment: .leading) {
                                // Background
                                RoundedRectangle(cornerRadius: 4)
                                    .fill(Color.white.opacity(0.2))
                                
                                // Progress fill
                                RoundedRectangle(cornerRadius: 4)
                                    .fill(calibrationManager.rightReady ? Color.green : Color.purple)
                                    .frame(width: geo.size.width * CGFloat(calibrationManager.rightStability) / 100.0)
                                    .animation(.spring(response: 0.3), value: calibrationManager.rightStability)
                            }
                        }
                        .frame(height: 8)
                        
                        Text("\(Int(calibrationManager.rightStability))%")
                            .font(.system(size: 12, weight: .semibold, design: .rounded))
                            .foregroundColor(calibrationManager.rightReady ? .green : .white)
                            .frame(width: 40, alignment: .trailing)
                        
                        Text("n=\(rightIntrinsicSamples)")
                            .font(.system(size: 10, design: .rounded))
                            .foregroundColor(.secondary)
                            .frame(width: 45, alignment: .trailing)
                    }
                }
                .padding(.horizontal, 20)
            } else {
                // Mono: single stability bar
                VStack(spacing: 4) {
                    HStack(spacing: 8) {
                        GeometryReader { geo in
                            ZStack(alignment: .leading) {
                                // Background
                                RoundedRectangle(cornerRadius: 6)
                                    .fill(Color.white.opacity(0.2))
                                
                                // Progress fill
                                RoundedRectangle(cornerRadius: 6)
                                    .fill(calibrationManager.leftReady ? Color.green : Color.cyan)
                                    .frame(width: geo.size.width * CGFloat(calibrationManager.leftStability) / 100.0)
                                    .animation(.spring(response: 0.3), value: calibrationManager.leftStability)
                            }
                        }
                        .frame(height: 12)
                        
                        Text("\(Int(calibrationManager.leftStability))%")
                            .font(.system(size: 16, weight: .bold, design: .rounded))
                            .foregroundColor(calibrationManager.leftReady ? .green : .white)
                            .frame(width: 50, alignment: .trailing)
                    }
                    
                    Text("n=\(intrinsicSamples) samples")
                        .font(.system(size: 11, design: .rounded))
                        .foregroundColor(.secondary)
                }
                .padding(.horizontal, 30)
            }
            
            VStack(spacing: 8) {
                Text("Intrinsic Calibration")
                    .font(.title3)
                    .fontWeight(.semibold)
                
                Text(isStereoCamera ? "Collect checkerboard samples from each lens independently" : "Move the camera to capture the checkerboard from different angles")
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
            
            // Live intrinsic parameters display
            if isStereoCamera {
                // Stereo: show left and right intrinsics
                VStack(spacing: 8) {
                    liveIntrinsicsRow(label: "L", intrinsics: calibrationManager.liveLeftIntrinsics, stability: calibrationManager.leftStability, ready: calibrationManager.leftReady)
                    liveIntrinsicsRow(label: "R", intrinsics: calibrationManager.liveRightIntrinsics, stability: calibrationManager.rightStability, ready: calibrationManager.rightReady)
                    
                    // Update timestamp
                    if let updateTime = calibrationManager.lastIntrinsicsUpdateTime {
                        let secondsAgo = Int(-updateTime.timeIntervalSinceNow)
                        Text("updated \(secondsAgo)s ago • \(calibrationManager.lastIntrinsicsSampleCount) samples")
                            .font(.caption2)
                            .foregroundColor(.gray)
                    }
                }
                .padding(12)
                .background(Color.black.opacity(0.3))
                .cornerRadius(12)
            } else if let intr = calibrationManager.liveLeftIntrinsics {
                // Mono: show single intrinsics
                VStack(spacing: 4) {
                    liveIntrinsicsRow(label: "", intrinsics: intr, stability: calibrationManager.leftStability, ready: calibrationManager.leftReady)
                    
                    // Update timestamp
                    if let updateTime = calibrationManager.lastIntrinsicsUpdateTime {
                        let secondsAgo = Int(-updateTime.timeIntervalSinceNow)
                        Text("updated \(secondsAgo)s ago • \(calibrationManager.lastIntrinsicsSampleCount) samples")
                            .font(.caption2)
                            .foregroundColor(.gray)
                    }
                }
                .padding(12)
                .background(Color.black.opacity(0.3))
                .cornerRadius(12)
            }
            
            // Stability/Ready indicator
            if calibrationManager.leftStability > 0 || calibrationManager.rightStability > 0 {
                HStack(spacing: 12) {
                    if isStereoCamera {
                        stabilityIndicator(label: "L", stability: calibrationManager.leftStability, ready: calibrationManager.leftReady)
                        stabilityIndicator(label: "R", stability: calibrationManager.rightStability, ready: calibrationManager.rightReady)
                    } else {
                        stabilityIndicator(label: "", stability: calibrationManager.leftStability, ready: calibrationManager.leftReady)
                    }
                    
                    if (isStereoCamera && calibrationManager.leftReady && calibrationManager.rightReady) ||
                       (!isStereoCamera && calibrationManager.leftReady) {
                        Text("READY")
                            .font(.caption.bold())
                            .foregroundColor(.green)
                            .padding(.horizontal, 8)
                            .padding(.vertical, 4)
                            .background(Color.green.opacity(0.2))
                            .cornerRadius(8)
                    }
                }
            }
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
                
                // Show reprojection error and calibration values
                if let calibration = calibrationManager.currentCalibration {
                    if calibration.isStereo, let rightIntr = calibration.rightIntrinsics {
                        // Stereo: show both left and right
                        HStack(spacing: 16) {
                            calibrationResultCard(label: "Left", intrinsics: calibration.leftIntrinsics)
                            calibrationResultCard(label: "Right", intrinsics: rightIntr)
                        }
                    } else {
                        // Mono: show single result
                        calibrationResultCard(label: "", intrinsics: calibration.leftIntrinsics)
                    }
                }
            }
            
            // Next steps
            infoCard(
                icon: "checkmark.seal.fill",
                title: "Next: Verify Intrinsics",
                points: [
                    "iPhone will show ArUco verification board",
                    "Point camera at the board",
                    "Verify 3D distance measurements match expected values",
                    "Green = good calibration, Red = needs recalibration"
                ],
                color: .purple
            )
        }
    }
    
    // MARK: - Intrinsic Verification View
    
    private var intrinsicVerificationView: some View {
        VStack(spacing: 20) {
            // UVC camera preview with ArUco visualization overlay
            ZStack(alignment: .bottom) {
                if let frame = verificationPreviewFrame {
                    Image(uiImage: frame)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .frame(maxHeight: 300)
                        .cornerRadius(12)
                        .overlay(
                            RoundedRectangle(cornerRadius: 12)
                                .stroke(Color.white.opacity(0.2), lineWidth: 1)
                        )
                } else {
                    RoundedRectangle(cornerRadius: 12)
                        .fill(Color.black)
                        .frame(height: 200)
                        .overlay(
                            ProgressView()
                                .scaleEffect(1.2)
                        )
                }
                
                // Detection status
                HStack(spacing: 6) {
                    Image(systemName: detectedTagCount >= verificationRows * verificationCols ? "checkmark.circle.fill" : "eye.fill")
                        .foregroundColor(detectedTagCount >= verificationRows * verificationCols ? .green : .white)
                    Text("Detected \(detectedTagCount)/\(verificationRows * verificationCols) tags")
                        .font(.caption)
                        .foregroundColor(.white)
                }
                .padding(.horizontal, 12)
                .padding(.vertical, 6)
                .background(Color.black.opacity(0.7))
                .cornerRadius(8)
                .padding(.bottom, 8)
            }
            
            // Verification status
            VStack(spacing: 8) {
                Text("Intrinsic Verification")
                    .font(.title3)
                    .fontWeight(.semibold)
                
                Text("Measuring 3D distances between ArUco tags")
                    .font(.body)
                    .foregroundColor(.secondary)
                    .multilineTextAlignment(.center)
            }
            
            // Distance measurements table
            if !verificationDistances.isEmpty {
                VStack(spacing: 8) {
                    ForEach(verificationDistances, id: \.pair) { item in
                        HStack {
                            Text(item.pair)
                                .font(.system(size: 13, design: .monospaced))
                                .foregroundColor(.secondary)
                            
                            Spacer()
                            
                            Text("\(String(format: "%.1f", item.measured))mm")
                                .font(.system(size: 13, weight: .medium, design: .monospaced))
                            
                            Text("(\(String(format: "%+.1f", item.error))mm)")
                                .font(.system(size: 12, design: .monospaced))
                                .foregroundColor(abs(item.error) < 2.0 ? .green : (abs(item.error) < 5.0 ? .orange : .red))
                        }
                    }
                }
                .padding(12)
                .background(Color.black.opacity(0.3))
                .cornerRadius(10)
            }
            
            // Overall result
            if verificationDistances.count > 0 {
                HStack(spacing: 12) {
                    ZStack {
                        Circle()
                            .fill(verificationPassed ? Color.green.opacity(0.2) : Color.orange.opacity(0.2))
                            .frame(width: 48, height: 48)
                        
                        Image(systemName: verificationPassed ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
                            .font(.system(size: 24))
                            .foregroundColor(verificationPassed ? .green : .orange)
                    }
                    
                    VStack(alignment: .leading, spacing: 2) {
                        Text(verificationPassed ? "Calibration Verified" : "High Error Detected")
                            .font(.headline)
                            .foregroundColor(verificationPassed ? .green : .orange)
                        
                        Text("Avg error: \(String(format: "%.2f", verificationAverageError))mm, Max: \(String(format: "%.2f", verificationMaxError))mm")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                }
                .padding(12)
                .background(Color.white.opacity(0.1))
                .cornerRadius(12)
            }
        }
    }
    
    @State private var verificationPreviewFrame: UIImage? = nil
    
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
            // UVC camera preview with ArUco visualization overlay
            ZStack(alignment: .bottom) {
                if let frame = extrinsicManager.previewWithVisualization ?? extrinsicManager.previewFrame {
                    Image(uiImage: frame)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .frame(maxHeight: 200)
                        .cornerRadius(12)
                        .overlay(
                            RoundedRectangle(cornerRadius: 12)
                                .stroke(extrinsicManager.arucoDetectionFeedback.isDetected ? Color.green.opacity(0.7) : Color.purple.opacity(0.5), lineWidth: 2)
                        )
                }
                
                // Guidance text overlay for ArUco detection
                let feedback = extrinsicManager.arucoDetectionFeedback
                HStack(spacing: 6) {
                    Image(systemName: feedback.isDetected ? (feedback.isTooFar ? "exclamationmark.triangle.fill" : "checkmark.circle.fill") : "eye.fill")
                        .foregroundColor(feedback.isDetected ? (feedback.isTooFar ? .orange : .green) : .white)
                    Text(feedback.guidanceMessage)
                        .fontWeight(.medium)
                        .foregroundColor(feedback.isDetected ? (feedback.isTooFar ? .orange : .green) : .white)
                }
                .font(.subheadline)
                .padding(.horizontal, 12)
                .padding(.vertical, 6)
                .background(Color.black.opacity(0.7))
                .cornerRadius(8)
                .padding(.bottom, 8)
            }
            
            // Check if both ARKit and camera have detected the marker
            let currentMarkerId = extrinsicManager.currentMarkerId
            let hasARKitDetection = extrinsicManager.rememberedMarkerPositions[currentMarkerId] != nil || 
                                    extrinsicManager.arkitTrackedMarkers[currentMarkerId] != nil
            let hasCameraDetection = extrinsicManager.cameraDetectedMarkers[currentMarkerId] != nil
            let bothDetected = hasARKitDetection && hasCameraDetection
            
            if !bothDetected {
                // Still waiting for detection
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
                
                // Detection status
                HStack(spacing: 16) {
                    HStack(spacing: 4) {
                        Image(systemName: hasARKitDetection ? "checkmark.circle.fill" : "circle")
                            .foregroundColor(hasARKitDetection ? .green : .secondary)
                        Text("ARKit")
                            .font(.caption)
                    }
                    HStack(spacing: 4) {
                        Image(systemName: hasCameraDetection ? "checkmark.circle.fill" : "circle")
                            .foregroundColor(hasCameraDetection ? .green : .secondary)
                        Text("Camera")
                            .font(.caption)
                    }
                }
                .padding(.horizontal, 16)
                .padding(.vertical, 10)
                .background(Color.white.opacity(0.1))
                .cornerRadius(20)
            } else {
                // Both detected - check freeze state
                let isFrozen = extrinsicManager.isMarkerVisualizationFrozen
                
                if isFrozen {
                    // Ready to collect samples
                    Image(systemName: "checkmark.circle.fill")
                        .font(.system(size: 48))
                        .foregroundColor(.green)
                    
                    VStack(spacing: 8) {
                        Text("Position Locked!")
                            .font(.title3)
                            .fontWeight(.semibold)
                            .foregroundColor(.green)
                        
                        Text("Collecting samples will begin automatically...")
                            .font(.body)
                            .foregroundColor(.secondary)
                            .multilineTextAlignment(.center)
                    }
                } else {
                    // Detected but not frozen - prompt user to freeze
                    Image(systemName: "snowflake")
                        .font(.system(size: 48))
                        .foregroundColor(.blue)
                    
                    VStack(spacing: 8) {
                        Text("Marker Detected!")
                            .font(.title3)
                            .fontWeight(.semibold)
                            .foregroundColor(.green)
                        
                        Text("Toggle **Freeze** on the marker visualization to lock the position and start sample collection")
                            .font(.body)
                            .foregroundColor(.secondary)
                            .multilineTextAlignment(.center)
                    }
                    
                    // Status badge
                    HStack(spacing: 8) {
                        Image(systemName: "exclamationmark.circle")
                            .foregroundColor(.orange)
                        Text("Enable Freeze toggle on marker label")
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
    }
    
    private var collectingSamplesView: some View {
        VStack(spacing: 20) {
            // UVC camera preview with ArUco visualization overlay
            ZStack(alignment: .bottom) {
                if let frame = extrinsicManager.previewWithVisualization ?? extrinsicManager.previewFrame {
                    Image(uiImage: frame)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .frame(maxHeight: 180)
                        .cornerRadius(12)
                        .overlay(
                            RoundedRectangle(cornerRadius: 12)
                                .stroke(extrinsicManager.arucoDetectionFeedback.isDetected ? Color.green.opacity(0.7) : Color.purple.opacity(0.5), lineWidth: 2)
                        )
                }
                
                // Guidance text overlay for ArUco detection
                let feedback = extrinsicManager.arucoDetectionFeedback
                if feedback.isTooFar {
                    HStack(spacing: 6) {
                        Image(systemName: "exclamationmark.triangle.fill")
                            .foregroundColor(.orange)
                        Text(feedback.guidanceMessage)
                            .fontWeight(.medium)
                            .foregroundColor(.orange)
                    }
                    .font(.subheadline)
                    .padding(.horizontal, 12)
                    .padding(.vertical, 6)
                    .background(Color.black.opacity(0.7))
                    .cornerRadius(8)
                    .padding(.bottom, 8)
                }
            }
            
            // DEBUG: Real-time pose values (placed here for visibility)
            VStack(alignment: .leading, spacing: 2) {
                Text("DEBUG - Transform Chain (Identity H2C)")
                    .font(.caption2.bold())
                    .foregroundColor(.orange)
                
                let hp = extrinsicManager.debugHeadPos
                let cp = extrinsicManager.debugCamMarkerPos
                let wp = extrinsicManager.debugWorldMarkerPos
                let ap = extrinsicManager.debugARKitMarkerPos
                
                HStack(spacing: 8) {
                    Text("Head: (\(String(format: "%.2f", hp.x)), \(String(format: "%.2f", hp.y)), \(String(format: "%.2f", hp.z)))")
                    Text("Cam: (\(String(format: "%.2f", cp.x)), \(String(format: "%.2f", cp.y)), \(String(format: "%.2f", cp.z)))")
                }
                .font(.system(size: 9, design: .monospaced))
                
                Text("WORLD: (\(String(format: "%.3f", wp.x)), \(String(format: "%.3f", wp.y)), \(String(format: "%.3f", wp.z)))")
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                    .foregroundColor(.green)
                
                Text("ARKit: (\(String(format: "%.3f", ap.x)), \(String(format: "%.3f", ap.y)), \(String(format: "%.3f", ap.z)))")
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                    .foregroundColor(.cyan)
                
                Text("Intrinsics: \(extrinsicManager.debugIntrinsics)")
                    .font(.system(size: 8, design: .monospaced))
                    .foregroundColor(.yellow)
            }
            .padding(6)
            .background(Color.black.opacity(0.7))
            .cornerRadius(6)
            
            // Progress indicator
            ZStack {
                Circle()
                    .stroke(Color.white.opacity(0.2), lineWidth: 8)
                    .frame(width: 100, height: 100)
                
                Circle()
                    .trim(from: 0, to: CGFloat(extrinsicSamplesForCurrentMarker) / CGFloat(targetExtrinsicSamplesPerMarker))
                    .stroke(Color.purple, style: StrokeStyle(lineWidth: 8, lineCap: .round))
                    .frame(width: 100, height: 100)
                    .rotationEffect(.degrees(-90))
                    .animation(.spring(response: 0.3), value: extrinsicSamplesForCurrentMarker)
                
                VStack(spacing: 2) {
                    Text("\(extrinsicSamplesForCurrentMarker)")
                        .font(.system(size: 28, weight: .bold, design: .rounded))
                    Text("/ \(targetExtrinsicSamplesPerMarker)")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            
            VStack(spacing: 8) {
                Text("Collecting Samples")
                    .font(.title3)
                    .fontWeight(.semibold)
                
                Text("Move your head slightly while keeping the marker visible")
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
            
            // Export debug data button (if recording was enabled)
            if debugRecorder.extrinsicSampleCount > 0 {
                Button {
                    // End the session and share the folder directly
                    debugRecorder.endSession()
                    if let folderURL = debugRecorder.getSessionURL() {
                        shareURL = folderURL
                        showShareSheet = true
                    }
                } label: {
                    HStack(spacing: 8) {
                        Image(systemName: "square.and.arrow.up")
                        Text("Export Debug Data (\(debugRecorder.extrinsicSampleCount) samples)")
                    }
                    .foregroundColor(.orange)
                    .padding(.horizontal, 16)
                    .padding(.vertical, 10)
                    .background(Color.orange.opacity(0.15))
                    .cornerRadius(8)
                }
                .buttonStyle(.plain)
                .sheet(isPresented: $showShareSheet) {
                    if let url = shareURL {
                        ShareSheet(activityItems: [url])
                    }
                }
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
            return "Verify Intrinsics"
        case .intrinsicVerification:
            return verificationPassed ? "Continue to Extrinsic" : "Skip Verification"
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
        case .verification:
            return ""
        }
    }
    
    private var primaryButtonEnabled: Bool {
        switch currentStep {
        case .setup:
            return uvcManager.selectedDevice != nil
        case .waitingForPhone:
            return false
        case .intrinsicCalibration:
            if isStereoCamera {
                // Stereo: require both sides to be ready (stability > 90%)
                return calibrationManager.leftReady && calibrationManager.rightReady
            } else {
                return calibrationManager.leftReady
            }
        case .intrinsicProcessing:
            return false
        case .intrinsicComplete:
            return true
        case .intrinsicVerification:
            return true  // Can always skip or continue
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
        case .verification:
            return false
        }
    }
    
    private func primaryAction() {
        switch currentStep {
        case .setup:
            currentStep = .waitingForPhone
        case .intrinsicCalibration:
            finishIntrinsicCalibration()
        case .intrinsicComplete:
            startIntrinsicVerification()
        case .intrinsicVerification:
            // Skip or continue to extrinsic
            stopIntrinsicVerification()
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
        // Update intrinsic samples (including left/right for stereo)
        intrinsicSamples = calibrationManager.samplesCollected
        leftIntrinsicSamples = calibrationManager.leftSamplesCollected
        rightIntrinsicSamples = calibrationManager.rightSamplesCollected
        
        // Update extrinsic samples
        extrinsicSamplesForCurrentMarker = extrinsicManager.samplesForCurrentMarker
        currentMarkerIndex = extrinsicManager.currentMarkerIndex
        
        // NOTE: Auto-advance removed - user now decides when to finish based on stability %
        // The "Next" button is enabled when stability > 90%
        if currentStep == .extrinsicCalibration {
            // Check marker detection for current marker
            let currentMarkerId = extrinsicManager.currentMarkerId
            let hasARKitDetection = extrinsicManager.rememberedMarkerPositions[currentMarkerId] != nil || 
                                    extrinsicManager.arkitTrackedMarkers[currentMarkerId] != nil
            let hasCameraDetection = extrinsicManager.cameraDetectedMarkers[currentMarkerId] != nil
            let bothDetected = hasARKitDetection && hasCameraDetection
            
            // Only transition to collecting samples when marker is detected AND freeze toggle is ON
            let isFrozen = extrinsicManager.isMarkerVisualizationFrozen
            if extrinsicSubStep == .waitingForARKit && bothDetected && isFrozen {
                // User has frozen the position - start collecting samples
                extrinsicSubStep = .collectingSamples
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
            if intrinsicPattern == .charuco {
                multipeerManager.sendCommand(.showCharuco)
            } else {
                multipeerManager.sendCommand(.showCheckerboard)
            }
            startIntrinsicCalibration()
        }
    }
    
    private func setupIntrinsicFrameCallback() {
        dlog("🔧 [CalibrationWizard] Setting up INTRINSIC frame callback...")
        
        let debugRecorderRef = debugRecorder
        
        uvcManager.onPixelBufferReceived = { [weak calibrationManager] pixelBuffer in
            guard let manager = calibrationManager else { return }
            guard manager.isCalibrating else { return }
            
            // Run detection on background thread to avoid blocking video
            Task.detached(priority: .userInitiated) {
                // Process frame based on stereo/mono mode
                // Detection runs on background thread
                if await MainActor.run(body: { self.isStereoCamera }) {
                    _ = await MainActor.run { manager.processStereoFrame(pixelBuffer) }
                } else {
                    _ = await MainActor.run { manager.processMonoFrame(pixelBuffer) }
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
        
        // Capture weak references for the closure
        let extrinsicManagerRef = extrinsicManager
        
        uvcManager.onPixelBufferReceived = { [weak extrinsicManagerRef] pixelBuffer in
            guard let manager = extrinsicManagerRef else { return }
            guard manager.isCalibrating else { return }
            
            // Process for marker detection (preview frame is generated inside processCameraFrame)
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
        
        // Update manager mode
        calibrationManager.calibrationMode = intrinsicPattern
        
        // Start debug recording if enabled
        if enableDebugRecording {
            debugRecorder.startSession(
                deviceId: device.id,
                deviceName: device.name,
                isStereo: isStereoCamera,
                markerSizeMeters: extrinsicManager.markerSizeMeters
            )
        }
        
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
                if let calibration = result {
                    currentStep = .intrinsicComplete
                    showSuccessCheckmark = false // Reset for animation
                    
                    // Save intrinsic result to debug folder
                    if debugRecorder.isRecording {
                        debugRecorder.saveIntrinsicCalibration(
                            leftIntrinsics: calibration.leftIntrinsics,
                            rightIntrinsics: calibration.rightIntrinsics,
                            imageWidth: calibration.leftIntrinsics.imageWidth,
                            imageHeight: calibration.leftIntrinsics.imageHeight
                        )
                    }
                } else {
                    currentStep = .error(calibrationManager.lastError ?? "Unknown error")
                }
            }
        }
    }
    
    // MARK: - Intrinsic Verification Functions
    
    private func startIntrinsicVerification() {
        guard let device = uvcManager.selectedDevice,
              let intrinsicData = calibrationManager.allCalibrations[device.id] else {
            dlog("❌ [CalibrationWizard] No intrinsic calibration for verification!")
            // Skip verification if no calibration
            startExtrinsicCalibration()
            return
        }
        
        // Reset verification state
        verificationPassed = false
        verificationAverageError = 0.0
        verificationMaxError = 0.0
        verificationDistances = []
        detectedTagCount = 0
        verificationPreviewFrame = nil
        
        // Tell iPhone to show verification board
        multipeerManager.sendCommand(.showVerificationBoard(
            cols: verificationCols,
            rows: verificationRows,
            tagSizeMM: verificationTagSizeMM,
            marginMM: verificationMarginMM,
            tagIds: verificationTagIds
        ))
        
        // Set up frame callback for ArUco detection and distance measurement
        setupVerificationFrameCallback(intrinsicData: intrinsicData)
        
        // Ensure camera is capturing
        if !uvcManager.isCapturing {
            uvcManager.startCapture()
        }
        
        currentStep = .intrinsicVerification
    }
    
    private func stopIntrinsicVerification() {
        // Clean up frame callback (will be replaced by extrinsic callback)
        uvcManager.onPixelBufferReceived = nil
        
        // Tell iPhone to hide the verification board
        multipeerManager.sendCommand(.hideDisplay)
    }
    
    private func setupVerificationFrameCallback(intrinsicData: CameraCalibrationData) {
        dlog("🔧 [CalibrationWizard] Setting up VERIFICATION frame callback...")
        
        // Create ArUco detector
        let arucoDetector = OpenCVArucoDetector(dictionary: .dict4X4_50)
        
        // Expected center-to-center distance
        let expectedCenterDistM = Double(verificationTagSizeMM + verificationMarginMM) / 1000.0
        
        // Get camera intrinsics
        let leftIntrinsics = intrinsicData.leftIntrinsics
        let cameraMatrix: [NSNumber] = [
            NSNumber(value: leftIntrinsics.fx), 0, NSNumber(value: leftIntrinsics.cx),
            0, NSNumber(value: leftIntrinsics.fy), NSNumber(value: leftIntrinsics.cy),
            0, 0, 1
        ]
        let distCoeffs: [NSNumber] = leftIntrinsics.distortionCoeffs.map { NSNumber(value: $0) }
        let markerLength = Float(verificationTagSizeMM) / 1000.0
        
        uvcManager.onPixelBufferReceived = { [weak arucoDetector] pixelBuffer in
            guard let detector = arucoDetector else { return }
            
            // Detect markers with pose
            guard let detections = detector.detectMarkers(
                in: pixelBuffer,
                cameraMatrix: cameraMatrix,
                distCoeffs: distCoeffs,
                markerLength: markerLength
            ) else { return }
            
            // Process on main thread
            Task { @MainActor in
                self.processVerificationDetections(
                    detections: detections,
                    pixelBuffer: pixelBuffer,
                    detector: detector,
                    cameraMatrix: cameraMatrix,
                    distCoeffs: distCoeffs,
                    expectedCenterDistM: expectedCenterDistM
                )
            }
        }
        
        dlog("✅ [CalibrationWizard] VERIFICATION frame callback set up")
    }
    
    private func processVerificationDetections(
        detections: [ArucoDetectionResult],
        pixelBuffer: CVPixelBuffer,
        detector: OpenCVArucoDetector,
        cameraMatrix: [NSNumber],
        distCoeffs: [NSNumber],
        expectedCenterDistM: Double
    ) {
        // Update detected tag count
        let validTagIds = Set(verificationTagIds)
        let relevantDetections = detections.filter { validTagIds.contains(Int($0.markerId)) }
        detectedTagCount = relevantDetections.count
        
        // Generate visualization
        if let vizData = detector.visualizeMarkers(
            pixelBuffer: pixelBuffer,
            detections: detections,
            drawAxes: true,
            cameraMatrix: cameraMatrix,
            distCoeffs: distCoeffs,
            axisLength: Float(verificationTagSizeMM) / 2000.0
        ), let uiImage = UIImage(data: vizData) {
            verificationPreviewFrame = uiImage
        }
        
        // Compute distances if we have at least 2 relevant tags
        guard relevantDetections.count >= 2 else { return }
        
        // Build tag center positions (from tvec)
        var tagCenters: [Int: simd_float3] = [:]
        for detection in relevantDetections {
            if detection.poseValid, let tvec = detection.tvec, tvec.count >= 3 {
                tagCenters[Int(detection.markerId)] = simd_float3(
                    tvec[0].floatValue,
                    tvec[1].floatValue,
                    tvec[2].floatValue
                )
            }
        }
        
        // Compute pairwise distances
        var distances: [(pair: String, measured: Double, expected: Double, error: Double)] = []
        let tagList = Array(tagCenters.keys).sorted()
        
        for i in 0..<tagList.count {
            for j in (i+1)..<tagList.count {
                let id1 = tagList[i]
                let id2 = tagList[j]
                
                guard let pos1 = tagCenters[id1], let pos2 = tagCenters[id2] else { continue }
                
                let measured = Double(simd_distance(pos1, pos2))
                
                // Compute expected distance based on grid positions
                let idx1 = verificationTagIds.firstIndex(of: id1) ?? 0
                let idx2 = verificationTagIds.firstIndex(of: id2) ?? 0
                let col1 = idx1 % verificationCols
                let row1 = idx1 / verificationCols
                let col2 = idx2 % verificationCols
                let row2 = idx2 / verificationCols
                
                let dx = Double(abs(col1 - col2))
                let dy = Double(abs(row1 - row2))
                let expected = expectedCenterDistM * sqrt(dx*dx + dy*dy)
                
                let error = (measured - expected) * 1000.0  // Convert to mm
                
                distances.append((
                    pair: "\(id1)-\(id2)",
                    measured: measured * 1000.0,
                    expected: expected * 1000.0,
                    error: error
                ))
            }
        }
        
        // Update state
        verificationDistances = distances
        
        if !distances.isEmpty {
            let errors = distances.map { abs($0.error) }
            verificationAverageError = errors.reduce(0, +) / Double(errors.count)
            verificationMaxError = errors.max() ?? 0.0
            
            // Pass if average error < 3mm and max error < 5mm
            verificationPassed = verificationAverageError < 3.0 && verificationMaxError < 5.0
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
        
        // Start debug recording if enabled and not already recording (may have started during intrinsic)
        if enableDebugRecording && !debugRecorder.isRecording {
            debugRecorder.startSession(
                deviceId: device.id,
                deviceName: device.name,
                isStereo: isStereoCamera,
                markerSizeMeters: extrinsicManager.markerSizeMeters
            )
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
        
        // Reset freeze toggle - user must freeze again for new marker position
        extrinsicManager.isMarkerVisualizationFrozen = false
    }
    
    private func finishExtrinsicCalibration() {
        guard let device = uvcManager.selectedDevice else { return }
        
        currentStep = .extrinsicProcessing
        
        // Compute calibration
        Task {
            // Use computeCalibration to verify first
            let result = extrinsicManager.computeCalibration(
                deviceId: device.id,
                deviceName: device.name,
                isStereo: isStereoCamera
            )
            
            await MainActor.run {
                if let result = result {
                    // Set proposal and move to verification
                    extrinsicManager.proposedCalibration = result
                    
                    // Reset verification tracking
                    hasVerifiedLeft = false
                    hasVerifiedRight = false
                    // Auto-mark as verified if not stereo
                    if !isStereoCamera {
                        hasVerifiedLeft = true
                        hasVerifiedRight = true // N/A
                        extrinsicManager.verificationSide = .left
                    } else {
                        // Default to left
                        extrinsicManager.verificationSide = .left
                        hasVerifiedLeft = true 
                    }
                    
                    withAnimation {
                        currentStep = .verification
                    }
                } else {
                    currentStep = .error(extrinsicManager.lastError ?? "Calibration computation failed")
                }
            }
        }
    }
    
    // MARK: - Verification View
    
    private var verificationView: some View {
        VStack(spacing: 24) {
            Image(systemName: "checkmark.magnifyingglass")
                .font(.system(size: 48))
                .foregroundColor(.blue)
            
            Text("Verify Calibration")
                .font(.title2)
                .fontWeight(.bold)
            
            Text("Look at the marker in AR. A green overlay should appear securely attached to the physical marker. Moving your head should not cause the overlay to drift.")
                .multilineTextAlignment(.center)
                .foregroundColor(.secondary)
            
            // Camera Preview and Controls
            VStack(spacing: 16) {
                // Preview
                if let frame = extrinsicManager.previewFrame {
                    Image(uiImage: frame)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .frame(height: 200)
                        .cornerRadius(12)
                        .overlay(
                            RoundedRectangle(cornerRadius: 12)
                                .stroke(Color.white.opacity(0.2), lineWidth: 1)
                        )
                } else {
                    ZStack {
                        Color.black.opacity(0.8)
                        ProgressView()
                    }
                    .frame(height: 200)
                    .cornerRadius(12)
                }
                
                // Stereo Toggle
                if isStereoCamera {
                    HStack(spacing: 20) {
                        Text("Verify Eye:")
                            .fontWeight(.medium)
                        
                        Picker("Eye", selection: $extrinsicManager.verificationSide) {
                            Text("Left").tag(CameraSide.left)
                            Text("Right").tag(CameraSide.right)
                        }
                        .pickerStyle(.segmented)
                        .frame(width: 150)
                        .onChange(of: extrinsicManager.verificationSide) { newSide in
                            if newSide == .left { hasVerifiedLeft = true }
                            if newSide == .right { hasVerifiedRight = true }
                        }
                    }
                    .padding(.horizontal)
                    .background(Color.black.opacity(0.3))
                    .cornerRadius(8)
                }
                
                // DEBUG: Real-time pose values
                VStack(alignment: .leading, spacing: 4) {
                    Text("DEBUG - Transform Chain (Identity H2C)")
                        .font(.caption.bold())
                        .foregroundColor(.orange)
                    
                    let hp = extrinsicManager.debugHeadPos
                    let cp = extrinsicManager.debugCamMarkerPos
                    let wp = extrinsicManager.debugWorldMarkerPos
                    
                    Text("Head: (\(String(format: "%.3f", hp.x)), \(String(format: "%.3f", hp.y)), \(String(format: "%.3f", hp.z)))")
                        .font(.system(size: 10, design: .monospaced))
                    Text("CamMarker: (\(String(format: "%.3f", cp.x)), \(String(format: "%.3f", cp.y)), \(String(format: "%.3f", cp.z)))")
                        .font(.system(size: 10, design: .monospaced))
                    Text("WorldMarker: (\(String(format: "%.3f", wp.x)), \(String(format: "%.3f", wp.y)), \(String(format: "%.3f", wp.z)))")
                        .font(.system(size: 10, design: .monospaced))
                        .foregroundColor(simd_length(wp) > 0.01 ? .green : .gray)
                }
                .padding(8)
                .background(Color.black.opacity(0.5))
                .cornerRadius(8)
            }
            
            Spacer()
            
            // Buttons
            HStack(spacing: 20) {
                Button(action: {
                    // Recalibrate
                    extrinsicManager.discardProposedCalibration()
                    // Restart extrinsic flow (reset to first marker)
                    currentMarkerIndex = 0
                    extrinsicSamplesForCurrentMarker = 0
                    extrinsicSubStep = .waitingForARKit
                    // Clear all samples in manager?
                    extrinsicManager.discardProposedCalibration()
                    // Restart extrinsic flow
                    cancelCalibration() // Stop current just in case (though we are paused)
                    startExtrinsicCalibration()
                    withAnimation {
                        currentStep = .extrinsicCalibration
                    }
                }) {
                    HStack {
                        Image(systemName: "arrow.counterclockwise")
                        Text("Recalibrate")
                    }
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(Color.red.opacity(0.2))
                    .foregroundColor(.red)
                    .cornerRadius(12)
                }
                
                Button(action: {
                    extrinsicManager.saveProposedCalibration()
                    // Tell iPhone calibration is complete
                    multipeerManager.sendCommand(.calibrationComplete)
                    withAnimation {
                        showSuccessCheckmark = true
                        currentStep = .complete
                    }
                }) {
                    HStack {
                        Image(systemName: "square.and.arrow.down")
                        Text("Save Calibration")
                    }
                    .frame(maxWidth: .infinity)
                    .padding()
                    .background(canSave ? Color.blue : Color.gray.opacity(0.3))
                    .foregroundColor(canSave ? .white : .gray)
                    .cornerRadius(12)
                }
                .disabled(!canSave)
            }
        }
    }
    
    private var canSave: Bool {
        if isStereoCamera {
            return hasVerifiedLeft && hasVerifiedRight
        } else {
            return true
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
    
    // MARK: - Live Intrinsics Helpers
    
    /// Row showing live intrinsic parameters for one camera
    @ViewBuilder
    private func liveIntrinsicsRow(label: String, intrinsics: CameraIntrinsics?, stability: Float, ready: Bool) -> some View {
        if let intr = intrinsics {
            VStack(alignment: .leading, spacing: 2) {
                // First line: fx, fy, cx, cy, error
                HStack(spacing: 4) {
                    if !label.isEmpty {
                        Text("\(label):")
                            .font(.system(size: 11, weight: .bold, design: .monospaced))
                            .foregroundColor(.white)
                    }
                    Text("fx=\(String(format: "%.1f", intr.fx)) fy=\(String(format: "%.1f", intr.fy))")
                        .font(.system(size: 10, design: .monospaced))
                        .foregroundColor(.cyan)
                    Text("cx=\(String(format: "%.1f", intr.cx)) cy=\(String(format: "%.1f", intr.cy))")
                        .font(.system(size: 10, design: .monospaced))
                        .foregroundColor(.orange)
                    Text("err=\(String(format: "%.3f", intr.reprojectionError))")
                        .font(.system(size: 10, weight: .semibold, design: .monospaced))
                        .foregroundColor(reprojectionErrorColor(intr.reprojectionError))
                }
                
                // Second line: distortion coefficients
                if intr.distortionCoeffs.count >= 5 {
                    HStack(spacing: 4) {
                        if !label.isEmpty {
                            Text("   ")
                                .font(.system(size: 11, design: .monospaced))
                        }
                        Text("k1=\(String(format: "%.4f", intr.distortionCoeffs[0])) k2=\(String(format: "%.4f", intr.distortionCoeffs[1])) k3=\(String(format: "%.4f", intr.distortionCoeffs[4]))")
                            .font(.system(size: 9, design: .monospaced))
                            .foregroundColor(.gray)
                    }
                }
            }
        } else {
            HStack {
                if !label.isEmpty {
                    Text("\(label):")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .foregroundColor(.white)
                }
                Text("collecting...")
                    .font(.system(size: 10, design: .monospaced))
                    .foregroundColor(.gray)
            }
        }
    }
    
    /// Color for reprojection error (green < 0.5, yellow < 1.0, orange < 2.0, red >= 2.0)
    private func reprojectionErrorColor(_ error: Double) -> Color {
        if error < 0.5 { return .green }
        if error < 1.0 { return .yellow }
        if error < 2.0 { return .orange }
        return .red
    }
    
    /// Stability indicator showing percentage
    @ViewBuilder
    private func stabilityIndicator(label: String, stability: Float, ready: Bool) -> some View {
        HStack(spacing: 4) {
            if !label.isEmpty {
                Text("\(label):")
                    .font(.caption2.bold())
                    .foregroundColor(.white)
            }
            Text("Stability: \(Int(stability))%")
                .font(.caption2)
                .foregroundColor(ready ? .green : (stability > 50 ? .yellow : .gray))
            
            if ready {
                Image(systemName: "checkmark.circle.fill")
                    .font(.caption2)
                    .foregroundColor(.green)
            }
        }
    }
    
    /// Card showing calibration result for one camera
    @ViewBuilder
    private func calibrationResultCard(label: String, intrinsics: CameraIntrinsics) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            if !label.isEmpty {
                Text(label)
                    .font(.caption.bold())
                    .foregroundColor(.white)
            }
            
            Text("err: \(String(format: "%.4f", intrinsics.reprojectionError)) px")
                .font(.system(size: 11, weight: .semibold, design: .monospaced))
                .foregroundColor(intrinsics.reprojectionError < 0.5 ? .green : .orange)
            
            VStack(alignment: .leading, spacing: 1) {
                Text("fx=\(String(format: "%.1f", intrinsics.fx))")
                Text("fy=\(String(format: "%.1f", intrinsics.fy))")
                Text("cx=\(String(format: "%.1f", intrinsics.cx))")
                Text("cy=\(String(format: "%.1f", intrinsics.cy))")
            }
            .font(.system(size: 10, design: .monospaced))
            .foregroundColor(.cyan)
            
            if intrinsics.distortionCoeffs.count >= 5 {
                VStack(alignment: .leading, spacing: 1) {
                    Text("k1=\(String(format: "%.4f", intrinsics.distortionCoeffs[0]))")
                    Text("k2=\(String(format: "%.4f", intrinsics.distortionCoeffs[1]))")
                    Text("k3=\(String(format: "%.4f", intrinsics.distortionCoeffs[4]))")
                }
                .font(.system(size: 9, design: .monospaced))
                .foregroundColor(.gray)
            }
        }
        .padding(10)
        .background(Color.black.opacity(0.3))
        .cornerRadius(8)
    }
}

// MARK: - Preview

#Preview {
    CameraCalibrationWizardView(onDismiss: {})
        .frame(width: 500, height: 700)
        .background(.regularMaterial)
}
