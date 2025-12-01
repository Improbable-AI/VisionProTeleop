import SwiftUI
import RealityKit
import Network
import AVFoundation
import GRPCCore
import GRPCNIOTransportHTTP2
import GRPCProtobuf

/// Protocol for MuJoCo manager to allow StatusOverlay to display status
@MainActor
protocol MuJoCoManager: ObservableObject {
    var ipAddress: String { get }
    var connectionStatus: String { get }
    var grpcPort: Int { get }
    var isServerRunning: Bool { get }
    var simEnabled: Bool { get }  // True if simulation data has been received (USDZ loaded or poses streaming)
    var poseStreamingViaWebRTC: Bool { get }
    var bodyCount: Int { get }
    var updateFrequency: Double { get }  // Hz
}

/// Network information manager for displaying connection status
class NetworkInfoManager: ObservableObject {
    @Published var ipAddresses: [(name: String, address: String)] = []
    @Published var pythonClientIP: String? = nil
    @Published var webrtcServerInfo: (host: String, port: Int)? = nil
    
    init() {
        print("🔵 [StatusView] NetworkInfoManager init called")
        updateNetworkInfo()
        
        // Periodically update status
        Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
            self?.updateNetworkInfo()
        }
    }
    
    func updateNetworkInfo() {
        ipAddresses = getIPAddresses()
        pythonClientIP = DataManager.shared.pythonClientIP
        webrtcServerInfo = DataManager.shared.webrtcServerInfo
    }
}

/// Enum to track which settings panel is currently expanded
enum ExpandedPanel: Equatable {
    case none
    case videoSource
    case viewControls
    case recording
    case statusPosition
    case calibration
    case extrinsicCalibration
}

/// A floating status display that shows network connection info and follows the user's head
struct StatusOverlay: View {
    @Binding var hasFrames: Bool
    let showVideoStatus: Bool  // Whether to show WebRTC and frame status
    @Binding var isMinimized: Bool
    @Binding var showViewControls: Bool
    @Binding var previewZDistance: Float?
    @Binding var previewActive: Bool
    @Binding var userInteracted: Bool
    @Binding var videoMinimized: Bool
    @Binding var videoFixed: Bool
    @Binding var previewStatusPosition: (x: Float, y: Float)?
    @Binding var previewStatusActive: Bool
    var mujocoManager: (any MuJoCoManager)?  // Optional MuJoCo manager for combined streaming
    @ObservedObject var dataManager = DataManager.shared
    @StateObject private var uvcCameraManager = UVCCameraManager.shared
    @StateObject private var recordingManager = RecordingManager.shared
    @State private var ipAddresses: [(name: String, address: String)] = []
    @State private var pythonConnected: Bool = false
    @State private var pythonIP: String = "Not connected"
    @State private var webrtcConnected: Bool = false
    @State private var hidePreviewTask: Task<Void, Never>?
    @State private var expandedPanel: ExpandedPanel = .none
    @State private var showLocalExitConfirmation: Bool = false
    @State private var mujocoStatusUpdateTrigger: Bool = false  // Trigger for MuJoCo status updates
    @State private var showCalibrationSheet: Bool = false
    @State private var showExtrinsicCalibrationSheet: Bool = false
    @StateObject private var calibrationManager = CameraCalibrationManager.shared
    @StateObject private var extrinsicCalibrationManager = ExtrinsicCalibrationManager.shared
    
    init(hasFrames: Binding<Bool> = .constant(false), showVideoStatus: Bool = true, isMinimized: Binding<Bool> = .constant(false), showViewControls: Binding<Bool> = .constant(false), previewZDistance: Binding<Float?> = .constant(nil), previewActive: Binding<Bool> = .constant(false), userInteracted: Binding<Bool> = .constant(false), videoMinimized: Binding<Bool> = .constant(false), videoFixed: Binding<Bool> = .constant(false), previewStatusPosition: Binding<(x: Float, y: Float)?> = .constant(nil), previewStatusActive: Binding<Bool> = .constant(false), mujocoManager: (any MuJoCoManager)? = nil) {
        self._hasFrames = hasFrames
        self.showVideoStatus = showVideoStatus
        self._isMinimized = isMinimized
        self._showViewControls = showViewControls
        self._previewZDistance = previewZDistance
        self._previewActive = previewActive
        self._userInteracted = userInteracted
        self._videoMinimized = videoMinimized
        self._videoFixed = videoFixed
        self._previewStatusPosition = previewStatusPosition
        self._previewStatusActive = previewStatusActive
        self.mujocoManager = mujocoManager
        print("🟢 [StatusView] StatusOverlay init called, hasFrames: \(hasFrames.wrappedValue), showVideoStatus: \(showVideoStatus), mujocoEnabled: \(mujocoManager != nil)")
    }
    
    var body: some View {
        ZStack {
            Group {
                if isMinimized {
                    minimizedView
                } else {
                    expandedView
                }
            }
            .animation(.spring(response: 0.45, dampingFraction: 0.85), value: isMinimized)
            
            // Calibration overlay
            if showCalibrationSheet {
                CameraCalibrationView(onDismiss: {
                    showCalibrationSheet = false
                })
                .frame(width: 500, height: 700)
                .background(.regularMaterial)
                .cornerRadius(20)
                .shadow(radius: 20)
            }
            
            // Extrinsic calibration overlay
            if showExtrinsicCalibrationSheet {
                ExtrinsicCalibrationView(onDismiss: {
                    showExtrinsicCalibrationSheet = false
                })
                .frame(width: 500, height: 700)
                .background(.regularMaterial)
                .cornerRadius(20)
                .shadow(radius: 20)
            }
        }
        .onAppear {
            print("🔴 [StatusView] StatusOverlay onAppear called")
            ipAddresses = getIPAddresses()
            print("🔴 [StatusView] IP Addresses: \(ipAddresses)")
            
            // Update status periodically
            Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { _ in
                let wasPythonConnected = pythonConnected
                let wasWebrtcConnected = webrtcConnected
                
                if let pythonClientIP = DataManager.shared.pythonClientIP {
                    pythonConnected = true
                    pythonIP = pythonClientIP
                } else {
                    pythonConnected = false
                    pythonIP = "Not connected"
                }
                
                webrtcConnected = DataManager.shared.webrtcServerInfo != nil
                
                // Toggle trigger to force MuJoCo status update
                mujocoStatusUpdateTrigger.toggle()
                
                // Detect disconnection and maximize status view
                if (wasPythonConnected && !pythonConnected) || (wasWebrtcConnected && !webrtcConnected) {
                    print("🔌 [StatusView] Connection lost - maximizing status view")
                    withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                        isMinimized = false
                        userInteracted = false  // Reset so it can auto-minimize again on next connection
                        hasFrames = false  // Clear frames flag
                    }
                }
                
                // Detect connection and minimize status view
                if (!wasPythonConnected && pythonConnected) {
                     // Only minimize on Python connection if NOT in video mode (hand tracking only)
                     // In video mode, we wait for frames to arrive (handled in ImmersiveView)
                     if !showVideoStatus {
                         print("🔌 [StatusView] Connection established - minimizing status view")
                         if !userInteracted {
                             withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                                 isMinimized = true
                             }
                         }
                     }
                }
            }
        }
    }
    
    private var minimizedView: some View {
        VStack(spacing: 12) {
            // Recording timer (show above buttons when recording)
            if recordingManager.isRecording {
                HStack(spacing: 8) {
                    Circle()
                        .fill(Color.red)
                        .frame(width: 12, height: 12)
                        .overlay(
                            Circle()
                                .stroke(Color.red.opacity(0.5), lineWidth: 2)
                                .scaleEffect(1.5)
                                .opacity(0.8)
                        )
                    Text("REC")
                        .font(.system(size: 14, weight: .bold))
                        .foregroundColor(.red)
                    Text(recordingManager.formatDuration(recordingManager.recordingDuration))
                        .font(.system(size: 16, weight: .bold, design: .monospaced))
                        .foregroundColor(.white)
                    Text("•")
                        .foregroundColor(.white.opacity(0.5))
                    Text("\(recordingManager.frameCount) frames")
                        .font(.system(size: 12))
                        .foregroundColor(.white.opacity(0.7))
                }
                .padding(.horizontal, 16)
                .padding(.vertical, 8)
                .background(Color.black.opacity(0.8))
                .cornerRadius(20)
            }
            
            HStack(spacing: 16) {
                if showLocalExitConfirmation {
                    // Confirmation mode
                    Text("Exit?")
                        .font(.headline)
                        .foregroundColor(.white)
                    
                    Button {
                        print("❌ [StatusView] Exiting app now")
                        exit(0)
                    } label: {
                        ZStack {
                            Circle()
                                .fill(Color.red)
                                .frame(width: 60, height: 60)
                            Image(systemName: "checkmark")
                                .font(.system(size: 24, weight: .bold))
                                .foregroundColor(.white)
                        }
                    }
                    .buttonStyle(.plain)
                    
                    Button {
                        withAnimation {
                            showLocalExitConfirmation = false
                        }
                    } label: {
                        ZStack {
                            Circle()
                                .fill(Color.gray.opacity(0.5))
                                .frame(width: 60, height: 60)
                            Image(systemName: "xmark")
                                .font(.system(size: 24, weight: .bold))
                                .foregroundColor(.white)
                        }
                    }
                    .buttonStyle(.plain)
                } else {
                    // Normal mode
                    // Expand button
                    Button {
                        withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                            isMinimized = false
                            userInteracted = true  // Mark that user has interacted
                        }
                    } label: {
                        ZStack {
                            Circle()
                                .fill(Color.white.opacity(0.3))
                                .frame(width: 60, height: 60)
                            Image(systemName: "arrow.up.left.and.arrow.down.right")
                                .font(.system(size: 24, weight: .bold))
                                .foregroundColor(.white)
                        }
                    }
                    .buttonStyle(.plain)
                    
                    // Recording button
                    Button {
                        withAnimation(.spring(response: 0.3, dampingFraction: 0.7)) {
                            if recordingManager.isRecording {
                                recordingManager.stopRecordingManually()
                            } else {
                                recordingManager.startRecording()
                            }
                        }
                    } label: {
                        ZStack {
                            Circle()
                                .fill(recordingManager.isRecording ? Color.red : Color.red.opacity(0.8))
                                .frame(width: 60, height: 60)
                            if recordingManager.isRecording {
                                // Stop icon (square)
                                RoundedRectangle(cornerRadius: 4)
                                    .fill(Color.white)
                                    .frame(width: 22, height: 22)
                            } else {
                                // Record icon (circle)
                                Circle()
                                    .fill(Color.white)
                                    .frame(width: 24, height: 24)
                            }
                        }
                    }
                    .buttonStyle(.plain)
                    
                    // Video minimize/maximize button (only show if video streaming mode is enabled)
                    if showVideoStatus {
                        Button {
                            withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                                videoMinimized.toggle()
                            }
                        } label: {
                            ZStack {
                                Circle()
                                    .fill(Color.blue.opacity(0.8))
                                    .frame(width: 60, height: 60)
                                Image(systemName: videoMinimized ? "video.fill" : "video.slash.fill")
                                    .font(.system(size: 24, weight: .bold))
                                    .foregroundColor(.white)
                            }
                        }
                        .buttonStyle(.plain)

                        // Toggle world-fixed mode for the video panel
                        Button {
                            videoFixed.toggle()
                        } label: {
                            ZStack {
                                Circle()
                                    .fill(videoFixed ? Color.orange.opacity(0.8) : Color.white.opacity(0.3))
                                    .frame(width: 60, height: 60)
                                Image(systemName: videoFixed ? "lock.fill" : "lock.open.fill")
                                    .font(.system(size: 24, weight: .bold))
                                    .foregroundColor(.white)
                            }
                        }
                        .buttonStyle(.plain)
                    }
                    
                    // Exit button
                    Button {
                        print("🔴 [StatusView] Exit button tapped (minimized)")
                        withAnimation {
                            showLocalExitConfirmation = true
                        }
                    } label: {
                        ZStack {
                            Circle()
                                .fill(Color.red)
                                .frame(width: 60, height: 60)
                            Text("✕")
                                .font(.system(size: 27, weight: .bold))
                                .foregroundColor(.white)
                        }
                    }
                    .buttonStyle(.plain)
                }
            }
        }
        .padding(30)
        .background(Color.black.opacity(0.6))
        .cornerRadius(36)
        .fixedSize()
    }

    private var headerSection: some View {
        HStack {
            // Minimize button
            Button {
                withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                    isMinimized = true
                    userInteracted = true  // Mark that user has interacted
                }
            } label: {
                ZStack {
                    Circle()
                        .fill(Color.white.opacity(0.3))
                        .frame(width: 60, height: 60)
                    Image(systemName: "arrow.down.right.and.arrow.up.left")
                        .font(.system(size: 24, weight: .bold))
                        .foregroundColor(.white)
                }
            }
            .buttonStyle(.plain)
            
            Spacer()
            
            Text("VisionProTeleop")
                .font(.title3)
                .fontWeight(.bold)
                .foregroundColor(.white)
            
            Spacer()
            
            Button {
                print("🔴 [StatusView] Exit button tapped (expanded)")
                withAnimation {
                    showLocalExitConfirmation = true
                }
            } label: {
                ZStack {
                    Circle()
                        .fill(Color.red)
                        .frame(width: 60, height: 60)
                    Text("✕")
                        .font(.system(size: 27, weight: .bold))
                        .foregroundColor(.white)
                }
            }
            .buttonStyle(.plain)
            .contentShape(Circle())
        }
    }

    private var networkInfoSection: some View {
        Group {
            VStack(alignment: .leading, spacing: 6) {
                ForEach(ipAddresses, id: \.address) { ip in
                    HStack {
                        Circle()
                            .fill(Color.green)
                            .frame(width: 12, height: 12)
                        Text("\(ip.name):")
                            .foregroundColor(.white.opacity(0.8))
                        Text(ip.address)
                            .foregroundColor(.white)
                            .fontWeight(.medium)
                    }
                    .font(.body)
                }
            }
            
            HStack {
                Circle()
                    .fill(pythonConnected ? Color.green : Color.red)
                    .frame(width: 12, height: 12)
                Text("Python Client:")
                    .foregroundColor(.white.opacity(0.8))
                Text(pythonIP)
                    .foregroundColor(.white)
                    .fontWeight(.medium)
            }
            .font(.body)
            
            // Only show WebRTC status if video streaming is enabled
            if showVideoStatus {
                HStack {
                    Circle()
                        .fill(webrtcConnected ? Color.green : Color.orange)
                        .frame(width: 12, height: 12)
                    Text("WebRTC:")
                        .foregroundColor(.white.opacity(0.8))
                    Text(webrtcConnected ? "Connected" : "Waiting...")
                        .foregroundColor(.white)
                        .fontWeight(.medium)
                }
                .font(.body)
            }
        }
    }

    private var streamDetailsSection: some View {
        HStack(spacing: 0) {
            // Video column
            VStack(spacing: 6) {
                // Title badge - show source type
                let isUVCMode = dataManager.videoSource == .uvcCamera
                let videoActive = isUVCMode ? uvcCameraManager.isCapturing : dataManager.videoEnabled
                
                Text(isUVCMode ? "USB Cam" : "Video")
                    .font(.caption2)
                    .fontWeight(.semibold)
                    .foregroundColor(videoActive ? .blue : .gray)
                    .padding(.horizontal, 8)
                    .padding(.vertical, 3)
                    .background((videoActive ? Color.blue : Color.gray).opacity(0.2))
                    .cornerRadius(8)
                
                if isUVCMode {
                    // UVC camera info
                    if uvcCameraManager.isCapturing {
                        Text("Mono")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.9))
                        
                        if uvcCameraManager.frameWidth > 0 {
                            Text("\(uvcCameraManager.frameWidth)×\(uvcCameraManager.frameHeight)")
                                .font(.caption2)
                                .foregroundColor(.white.opacity(0.7))
                                .monospacedDigit()
                        }
                        if uvcCameraManager.fps > 0 {
                            Text("\(uvcCameraManager.fps) fps")
                                .font(.caption2)
                                .foregroundColor(.white.opacity(0.7))
                                .monospacedDigit()
                        }
                    } else if uvcCameraManager.selectedDevice != nil {
                        Text("Starting...")
                            .font(.caption)
                            .foregroundColor(.yellow)
                    } else {
                        Text("No Device")
                            .font(.caption)
                            .foregroundColor(.orange)
                    }
                } else if dataManager.videoEnabled {
                    // Network video info
                    if dataManager.stereoEnabled {
                        Text("Stereo")
                            .font(.caption)
                            .foregroundColor(.cyan)
                            .fontWeight(.medium)
                    } else {
                        Text("Mono")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.9))
                    }
                    
                    // Stats
                    if !dataManager.videoResolution.isEmpty && dataManager.videoResolution != "Waiting..." {
                        Text(dataManager.videoResolution)
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.7))
                            .monospacedDigit()
                    }
                    if dataManager.videoFPS > 0 {
                        Text("\(dataManager.videoFPS) fps")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.7))
                            .monospacedDigit()
                    }
                } else {
                    Text("N/A")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                }
            }
            .frame(maxWidth: .infinity)
            
            // Vertical divider
            Rectangle()
                .fill(Color.white.opacity(0.2))
                .frame(width: 1, height: 40)
            
            // Audio column
            VStack(spacing: 6) {
                // Title badge
                Text("Audio")
                    .font(.caption2)
                    .fontWeight(.semibold)
                    .foregroundColor(dataManager.audioEnabled ? .green : .gray)
                    .padding(.horizontal, 8)
                    .padding(.vertical, 3)
                    .background((dataManager.audioEnabled ? Color.green : Color.gray).opacity(0.2))
                    .cornerRadius(8)
                
                // Status
                if dataManager.audioEnabled {
                    if dataManager.stereoAudioEnabled {
                        Text("Stereo")
                            .font(.caption)
                            .foregroundColor(.green)
                            .fontWeight(.medium)
                    } else {
                        Text("Mono")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.9))
                    }
                    
                    // Stats
                    if dataManager.audioSampleRate > 0 {
                        Text("\(dataManager.audioSampleRate) Hz")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.7))
                            .monospacedDigit()
                    }
                } else {
                    Text("N/A")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                }
            }
            .frame(maxWidth: .infinity)
            
            // Vertical divider
            Rectangle()
                .fill(Color.white.opacity(0.2))
                .frame(width: 1, height: 40)
            
            // Sim column
            VStack(spacing: 6) {
                let _ = mujocoStatusUpdateTrigger  // Force refresh
                
                // Determine if simulation is active (has received data)
                let simActive = mujocoManager?.simEnabled == true || (mujocoManager?.bodyCount ?? 0) > 0
                
                // Title badge
                Text("Sim")
                    .font(.caption2)
                    .fontWeight(.semibold)
                    .foregroundColor(simActive ? .orange : .gray)
                    .padding(.horizontal, 8)
                    .padding(.vertical, 3)
                    .background((simActive ? Color.orange : Color.gray).opacity(0.2))
                    .cornerRadius(8)
                
                if let mujoco = mujocoManager, simActive {
                    // Status
                    if mujoco.poseStreamingViaWebRTC {
                        Text("WebRTC")
                            .font(.caption)
                            .foregroundColor(.green)
                            .fontWeight(.medium)
                    } else {
                        Text("gRPC")
                            .font(.caption)
                            .foregroundColor(.orange)
                            .fontWeight(.medium)
                    }
                    
                    // Body count
                    if mujoco.bodyCount > 0 {
                        Text("\(mujoco.bodyCount) bodies")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.7))
                    }
                    
                    // Update frequency
                    if mujoco.updateFrequency > 0 {
                        Text(String(format: "%.0f Hz", mujoco.updateFrequency))
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.7))
                            .monospacedDigit()
                    }
                } else {
                    Text("N/A")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                }
            }
            .frame(maxWidth: .infinity)
        }
        .padding(.vertical, 8)
    }

    private var expandedView: some View {
        HStack(alignment: .top, spacing: 0) {
            // Left column - Main status panel
            leftColumnView
            
            // Right column - Expanded settings panel (if any)
            if expandedPanel != .none {
                rightColumnView
            }
        }
        .animation(.spring(response: 0.35, dampingFraction: 0.85), value: expandedPanel)
        .overlay(
            Group {
                if showLocalExitConfirmation {
                    ZStack {
                        Color.black.opacity(0.9)
                        
                        VStack(spacing: 20) {
                            Text("Are you sure you want to exit?")
                                .font(.headline)
                                .foregroundColor(.white)
                            
                            HStack(spacing: 20) {
                                Button {
                                    withAnimation {
                                        showLocalExitConfirmation = false
                                    }
                                } label: {
                                    Text("Cancel")
                                        .fontWeight(.medium)
                                        .foregroundColor(.white)
                                        .padding(.horizontal, 20)
                                        .padding(.vertical, 10)
                                        .background(Color.gray.opacity(0.5))
                                        .cornerRadius(8)
                                }
                                .buttonStyle(.plain)
                                
                                Button {
                                    print("❌ [StatusView] Exiting app now")
                                    exit(0)
                                } label: {
                                    Text("Exit")
                                        .fontWeight(.bold)
                                        .foregroundColor(.white)
                                        .padding(.horizontal, 20)
                                        .padding(.vertical, 10)
                                        .background(Color.red)
                                        .cornerRadius(8)
                                }
                                .buttonStyle(.plain)
                            }
                        }
                    }
                    .cornerRadius(16)
                }
            }
        )
    }
    
    private var leftColumnView: some View {
        VStack(alignment: .leading, spacing: 15) {
            headerSection
            
            Divider()
                .background(Color.white.opacity(0.3))
            
            networkInfoSection
            
            // Show detailed track information when connected (either WebRTC or UVC camera)
            let showStreamDetails = showVideoStatus && (webrtcConnected || (dataManager.videoSource == .uvcCamera && uvcCameraManager.isCapturing))
            if showStreamDetails {
                Divider()
                    .background(Color.white.opacity(0.2))
                
                streamDetailsSection
            }
            
            // Show waiting message when no frames are available (only for video mode)
            // Don't show waiting for UVC if a camera is capturing
            let isUVCActive = dataManager.videoSource == .uvcCamera && uvcCameraManager.isCapturing
            if showVideoStatus && !hasFrames && !isUVCActive {
                Divider()
                    .background(Color.white.opacity(0.3))
                
                HStack {
                    ProgressView()
                        .progressViewStyle(CircularProgressViewStyle(tint: .white))
                        .scaleEffect(1.0)
                    Text(dataManager.videoSource == .uvcCamera ?
                         (uvcCameraManager.selectedDevice == nil ? "No USB camera detected" : "Waiting for camera...") :
                         dataManager.connectionStatus)
                        .foregroundColor(.white.opacity(0.9))
                        .font(.body)
                        .fontWeight(.medium)
                        .lineLimit(2)
                }
            }
            
            // Menu items section
            if showVideoStatus {
                Divider()
                    .background(Color.white.opacity(0.3))
                
                // Video Source menu item
                menuItem(
                    icon: dataManager.videoSource.icon,
                    title: "Video Source",
                    subtitle: dataManager.videoSource.rawValue,
                    isExpanded: expandedPanel == .videoSource,
                    accentColor: .blue
                ) {
                    withAnimation(.spring(response: 0.35, dampingFraction: 0.85)) {
                        expandedPanel = expandedPanel == .videoSource ? .none : .videoSource
                    }
                }
                
                // Modify Video View menu item
                menuItem(
                    icon: "slider.horizontal.3",
                    title: "Modify Video View",
                    subtitle: "\(String(format: "%.1f", -dataManager.videoPlaneZDistance))m",
                    isExpanded: expandedPanel == .viewControls,
                    accentColor: .green
                ) {
                    withAnimation(.spring(response: 0.35, dampingFraction: 0.85)) {
                        if expandedPanel == .viewControls {
                            expandedPanel = .none
                            previewZDistance = nil
                            previewActive = false
                            hidePreviewTask?.cancel()
                        } else {
                            expandedPanel = .viewControls
                        }
                    }
                }
            }
            
            Divider()
                .background(Color.white.opacity(0.3))
            
            // Recording menu item
            menuItem(
                icon: recordingManager.isRecording ? "record.circle.fill" : "record.circle",
                title: "Recording",
                subtitle: recordingManager.isRecording ? recordingManager.formatDuration(recordingManager.recordingDuration) : recordingManager.storageLocation.rawValue,
                isExpanded: expandedPanel == .recording,
                accentColor: .red,
                iconColor: recordingManager.isRecording ? .red : nil
            ) {
                withAnimation(.spring(response: 0.35, dampingFraction: 0.85)) {
                    expandedPanel = expandedPanel == .recording ? .none : .recording
                }
            }
            
            // Camera Calibration menu item (show when UVC camera is active)
            if dataManager.videoSource == .uvcCamera {
                let hasCalibration = uvcCameraManager.selectedDevice.map { calibrationManager.hasCalibration(for: $0.id) } ?? false
                menuItem(
                    icon: "camera.viewfinder",
                    title: "Intrinsic Calibration",
                    subtitle: hasCalibration ? "Calibrated" : "Not Calibrated",
                    isExpanded: expandedPanel == .calibration,
                    accentColor: .cyan,
                    iconColor: hasCalibration ? .green : .orange
                ) {
                    withAnimation(.spring(response: 0.35, dampingFraction: 0.85)) {
                        expandedPanel = expandedPanel == .calibration ? .none : .calibration
                    }
                }
                
                // Extrinsic Calibration menu item (head-to-camera transform)
                let hasExtrinsicCalibration = uvcCameraManager.selectedDevice.map { extrinsicCalibrationManager.hasCalibration(for: $0.id) } ?? false
                menuItem(
                    icon: "arrow.triangle.swap",
                    title: "Extrinsic Calibration",
                    subtitle: hasExtrinsicCalibration ? "Calibrated" : "Not Calibrated",
                    isExpanded: expandedPanel == .extrinsicCalibration,
                    accentColor: .purple,
                    iconColor: hasExtrinsicCalibration ? .green : .orange
                ) {
                    withAnimation(.spring(response: 0.35, dampingFraction: 0.85)) {
                        expandedPanel = expandedPanel == .extrinsicCalibration ? .none : .extrinsicCalibration
                    }
                }
            }
            
            Divider()
                .background(Color.white.opacity(0.3))
            
            // Controller Position menu item
            menuItem(
                icon: "move.3d",
                title: "Controller Position",
                subtitle: nil,
                isExpanded: expandedPanel == .statusPosition,
                accentColor: .purple
            ) {
                withAnimation(.spring(response: 0.35, dampingFraction: 0.85)) {
                    if expandedPanel == .statusPosition {
                        expandedPanel = .none
                        previewStatusPosition = nil
                        previewStatusActive = false
                        hidePreviewTask?.cancel()
                    } else {
                        expandedPanel = .statusPosition
                    }
                }
            }
        }
        .padding(24)
        .frame(width: 352)
        .background(Color.black.opacity(0.7))
        .cornerRadius(16)
    }
    
    private func menuItem(icon: String, title: String, subtitle: String?, isExpanded: Bool, accentColor: Color, iconColor: Color? = nil, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            HStack {
                Image(systemName: icon)
                    .font(.system(size: 17, weight: .medium))
                    .foregroundColor(iconColor ?? (isExpanded ? accentColor : .white.opacity(0.9)))
                    .frame(width: 24)
                Text(title)
                    .font(.body)
                    .fontWeight(.medium)
                Spacer()
                if let subtitle = subtitle {
                    Text(subtitle)
                        .font(.caption)
                        .foregroundColor(isExpanded ? accentColor : .white.opacity(0.6))
                }
                Image(systemName: isExpanded ? "chevron.right" : "chevron.right")
                    .font(.system(size: 14, weight: .bold))
                    .foregroundColor(isExpanded ? accentColor : .white.opacity(0.5))
            }
            .foregroundColor(isExpanded ? accentColor : .white.opacity(0.9))
            .padding(.vertical, 8)
            .padding(.horizontal, 4)
            .background(isExpanded ? accentColor.opacity(0.15) : Color.clear)
            .cornerRadius(8)
        }
        .buttonStyle(.plain)
    }
    
    private var rightColumnView: some View {
        VStack(alignment: .leading, spacing: 8) {
            // Panel header
            HStack {
                Text(panelTitle)
                    .font(.body)
                    .fontWeight(.semibold)
                    .foregroundColor(.white)
                Spacer()
                Button {
                    withAnimation(.spring(response: 0.35, dampingFraction: 0.85)) {
                        expandedPanel = .none
                        previewZDistance = nil
                        previewActive = false
                        previewStatusPosition = nil
                        previewStatusActive = false
                        hidePreviewTask?.cancel()
                    }
                } label: {
                    ZStack {
                        Circle()
                            .fill(Color.white.opacity(0.2))
                            .frame(width: 44, height: 44)
                        Image(systemName: "chevron.left")
                            .font(.system(size: 18, weight: .bold))
                            .foregroundColor(.white)
                    }
                }
                .buttonStyle(.plain)
            }
            
            Divider()
                .background(Color.white.opacity(0.3))
            
            // Panel content
            switch expandedPanel {
            case .videoSource:
                videoSourcePanelContent
            case .viewControls:
                viewControlsPanelContent
            case .recording:
                recordingPanelContent
            case .statusPosition:
                statusPositionPanelContent
            case .calibration:
                calibrationPanelContent
            case .extrinsicCalibration:
                extrinsicCalibrationPanelContent
            case .none:
                EmptyView()
            }
        }
        .padding(24)
        .frame(width: 300)
        .background(Color.black.opacity(0.7))
        .cornerRadius(16)
        .padding(.leading, 8)
    }
    
    private var panelTitle: String {
        switch expandedPanel {
        case .videoSource: return "Video Source"
        case .viewControls: return "Video View"
        case .recording: return "Recording"
        case .statusPosition: return "Controller Position"
        case .calibration: return "Camera Calibration"
        case .extrinsicCalibration: return "Extrinsic Calibration"
        case .none: return ""
        }
    }
    
    // MARK: - Right Panel Content Views
    
    private var videoSourcePanelContent: some View {
        VStack(alignment: .leading, spacing: 8) {
            // Network Stream option
            Button {
                withAnimation {
                    dataManager.videoSource = .network
                }
            } label: {
                HStack {
                    Image(systemName: "wifi")
                        .font(.system(size: 14))
                        .frame(width: 20)
                    VStack(alignment: .leading, spacing: 2) {
                        Text("Network Stream")
                            .font(.caption)
                            .fontWeight(.medium)
                        Text("WebRTC from Python")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.5))
                    }
                    Spacer()
                    if dataManager.videoSource == .network {
                        Image(systemName: "checkmark.circle.fill")
                            .font(.system(size: 14))
                            .foregroundColor(.green)
                    }
                }
                .padding(.horizontal, 10)
                .padding(.vertical, 8)
                .background(dataManager.videoSource == .network ? Color.blue.opacity(0.3) : Color.white.opacity(0.1))
                .cornerRadius(8)
                .foregroundColor(.white)
            }
            .buttonStyle(.plain)
            
            // USB Camera option
            Button {
                Task {
                    let granted = await uvcCameraManager.requestCameraAccess()
                    if granted {
                        await MainActor.run {
                            withAnimation {
                                dataManager.videoSource = .uvcCamera
                            }
                        }
                    }
                }
            } label: {
                HStack {
                    Image(systemName: "cable.connector")
                        .font(.system(size: 14))
                        .frame(width: 20)
                    VStack(alignment: .leading, spacing: 2) {
                        Text("USB Camera")
                            .font(.caption)
                            .fontWeight(.medium)
                        Text("UVC via Developer Strap")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.5))
                    }
                    Spacer()
                    if dataManager.videoSource == .uvcCamera {
                        Image(systemName: "checkmark.circle.fill")
                            .font(.system(size: 14))
                            .foregroundColor(.green)
                    }
                }
                .padding(.horizontal, 10)
                .padding(.vertical, 8)
                .background(dataManager.videoSource == .uvcCamera ? Color.blue.opacity(0.3) : Color.white.opacity(0.1))
                .cornerRadius(8)
                .foregroundColor(.white)
            }
            .buttonStyle(.plain)
            
            // Available cameras section (show only when USB selected)
            if dataManager.videoSource == .uvcCamera && !uvcCameraManager.availableDevices.isEmpty {
                Divider()
                    .background(Color.white.opacity(0.3))
                
                Text("Available Cameras")
                    .font(.caption2)
                    .foregroundColor(.white.opacity(0.5))
                
                ForEach(uvcCameraManager.availableDevices.prefix(2)) { device in
                    Button {
                        uvcCameraManager.selectDevice(device)
                        Task {
                            try? await Task.sleep(nanoseconds: 200_000_000)
                            await MainActor.run { uvcCameraManager.startCapture() }
                        }
                    } label: {
                        HStack {
                            Image(systemName: "video.fill")
                                .font(.system(size: 12))
                                .frame(width: 16)
                            Text(device.name)
                                .font(.caption)
                                .lineLimit(1)
                            Spacer()
                            if uvcCameraManager.selectedDevice?.id == device.id && uvcCameraManager.isCapturing {
                                Circle()
                                    .fill(Color.green)
                                    .frame(width: 6, height: 6)
                            }
                        }
                        .padding(.horizontal, 10)
                        .padding(.vertical, 8)
                        .background(uvcCameraManager.selectedDevice?.id == device.id ? Color.green.opacity(0.2) : Color.white.opacity(0.1))
                        .cornerRadius(8)
                        .foregroundColor(.white.opacity(0.9))
                    }
                    .buttonStyle(.plain)
                }
            }
        }
    }
    
    private var viewControlsPanelContent: some View {
        VStack(alignment: .leading, spacing: 10) {
            // Distance control
            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    Text("Distance")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.7))
                    Spacer()
                    Text("\(String(format: "%.1f", -dataManager.videoPlaneZDistance))m")
                        .font(.caption)
                        .foregroundColor(.white)
                        .monospacedDigit()
                }
                
                Slider(value: Binding(
                    get: { -dataManager.videoPlaneZDistance },
                    set: { positiveValue in
                        let negativeValue = -positiveValue
                        dataManager.videoPlaneZDistance = negativeValue
                        previewZDistance = negativeValue
                        hidePreviewTask?.cancel()
                        hidePreviewTask = Task { @MainActor in
                            try? await Task.sleep(nanoseconds: 3_000_000_000)
                            if !Task.isCancelled { previewZDistance = nil }
                        }
                    }
                ), in: 2.0...20.0, step: 0.5)
                .tint(.blue)
            }
            
            // Height control
            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    Text("Height")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.7))
                    Spacer()
                    Text("\(String(format: "%.2f", dataManager.videoPlaneYPosition))m")
                        .font(.caption)
                        .foregroundColor(.white)
                        .monospacedDigit()
                }
                
                Slider(value: Binding(
                    get: { dataManager.videoPlaneYPosition },
                    set: { newValue in
                        dataManager.videoPlaneYPosition = newValue
                        previewActive = true
                        hidePreviewTask?.cancel()
                        hidePreviewTask = Task { @MainActor in
                            try? await Task.sleep(nanoseconds: 3_000_000_000)
                            if !Task.isCancelled { previewActive = false }
                        }
                    }
                ), in: -2.0...2.0, step: 0.1)
                .tint(.green)
            }
            
            // Lock to world toggle
            HStack {
                Text("Lock To World")
                    .font(.caption)
                    .foregroundColor(.white.opacity(0.8))
                Spacer()
                Toggle("", isOn: $videoFixed)
                    .labelsHidden()
                    .tint(.orange)
            }
            
            // Action buttons
            HStack(spacing: 8) {
                Button {
                    withAnimation { videoMinimized.toggle() }
                } label: {
                    HStack(spacing: 4) {
                        Image(systemName: videoMinimized ? "eye.fill" : "eye.slash.fill")
                            .font(.system(size: 10, weight: .bold))
                        Text(videoMinimized ? "Show" : "Hide")
                            .font(.caption2)
                    }
                    .foregroundColor(.blue)
                    .padding(.horizontal, 10)
                    .padding(.vertical, 6)
                    .background(Color.white.opacity(0.15))
                    .cornerRadius(6)
                }
                .buttonStyle(.plain)
                
                Button {
                    dataManager.videoPlaneZDistance = -10.0
                    dataManager.videoPlaneYPosition = 0.0
                    dataManager.videoPlaneAutoPerpendicular = false
                    previewZDistance = -10.0
                    previewActive = true
                    hidePreviewTask?.cancel()
                    hidePreviewTask = Task { @MainActor in
                        try? await Task.sleep(nanoseconds: 3_000_000_000)
                        if !Task.isCancelled {
                            previewZDistance = nil
                            previewActive = false
                        }
                    }
                } label: {
                    Text("Reset")
                        .font(.caption2)
                        .foregroundColor(.blue)
                        .padding(.horizontal, 10)
                        .padding(.vertical, 6)
                        .background(Color.white.opacity(0.15))
                        .cornerRadius(6)
                }
                .buttonStyle(.plain)
            }
        }
    }
    
    private var recordingPanelContent: some View {
        VStack(alignment: .leading, spacing: 8) {
            if recordingManager.isRecording {
                // Active recording display
                HStack {
                    Circle()
                        .fill(Color.red)
                        .frame(width: 10, height: 10)
                    Text(recordingManager.formatDuration(recordingManager.recordingDuration))
                        .font(.caption)
                        .fontWeight(.bold)
                        .foregroundColor(.red)
                        .monospacedDigit()
                    Text("• \(recordingManager.frameCount) frames")
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.6))
                    if recordingManager.isAutoRecording {
                        Text("(auto)")
                            .font(.caption2)
                            .foregroundColor(.orange.opacity(0.8))
                    }
                    Spacer()
                }
                
                Button { recordingManager.stopRecordingManually() } label: {
                    HStack {
                        Image(systemName: "stop.fill")
                            .font(.system(size: 12, weight: .bold))
                        Text("Stop Recording")
                            .font(.caption)
                            .fontWeight(.semibold)
                    }
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 10)
                    .background(Color.red)
                    .cornerRadius(8)
                }
                .buttonStyle(.plain)
            } else if recordingManager.isSaving {
                HStack {
                    ProgressView()
                        .progressViewStyle(CircularProgressViewStyle(tint: .white))
                        .scaleEffect(0.6)
                    Text("Saving...")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.8))
                }
            } else {
                // Auto-recording toggle
                HStack {
                    Toggle(isOn: $recordingManager.autoRecordingEnabled) {
                        HStack(spacing: 6) {
                            Image(systemName: recordingManager.autoRecordingEnabled ? "record.circle.fill" : "record.circle")
                                .font(.system(size: 14))
                                .foregroundColor(recordingManager.autoRecordingEnabled ? .orange : .white.opacity(0.6))
                            VStack(alignment: .leading, spacing: 1) {
                                Text("Auto-Record")
                                    .font(.caption)
                                    .fontWeight(.medium)
                                Text("Start when video frames arrive")
                                    .font(.caption2)
                                    .foregroundColor(.white.opacity(0.5))
                            }
                        }
                    }
                    .toggleStyle(SwitchToggleStyle(tint: .orange))
                }
                .padding(.horizontal, 10)
                .padding(.vertical, 8)
                .background(recordingManager.autoRecordingEnabled ? Color.orange.opacity(0.15) : Color.white.opacity(0.1))
                .cornerRadius(8)
                
                Divider()
                    .background(Color.white.opacity(0.2))
                    .padding(.vertical, 4)
                
                // Save location options
                ForEach(RecordingStorageLocation.allCases, id: \.self) { location in
                    Button { recordingManager.storageLocation = location } label: {
                        HStack {
                            Image(systemName: location.icon)
                                .font(.system(size: 14))
                                .frame(width: 20)
                            VStack(alignment: .leading, spacing: 2) {
                                Text(location.rawValue)
                                    .font(.caption)
                                    .fontWeight(.medium)
                                Text(storageDescription(for: location))
                                    .font(.caption2)
                                    .foregroundColor(.white.opacity(0.5))
                            }
                            Spacer()
                            if recordingManager.storageLocation == location {
                                Image(systemName: "checkmark.circle.fill")
                                    .font(.system(size: 14))
                                    .foregroundColor(.green)
                            }
                        }
                        .padding(.horizontal, 10)
                        .padding(.vertical, 8)
                        .background(recordingManager.storageLocation == location ? Color.green.opacity(0.2) : Color.white.opacity(0.1))
                        .cornerRadius(8)
                        .foregroundColor(.white)
                    }
                    .buttonStyle(.plain)
                }
                
                Button { recordingManager.startRecording() } label: {
                    HStack {
                        Image(systemName: "record.circle")
                            .font(.system(size: 14, weight: .bold))
                        Text("Start Recording")
                            .font(.caption)
                            .fontWeight(.semibold)
                    }
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 10)
                    .background(Color.red.opacity(0.8))
                    .cornerRadius(8)
                }
                .buttonStyle(.plain)
                
                // Calibration warning for UVC camera
                if dataManager.videoSource == .uvcCamera {
                    let hasCalibration = uvcCameraManager.selectedDevice.map { calibrationManager.hasCalibration(for: $0.id) } ?? false
                    if !hasCalibration {
                        HStack(spacing: 6) {
                            Image(systemName: "exclamationmark.triangle.fill")
                                .font(.caption)
                                .foregroundColor(.orange)
                            Text("Camera not calibrated. Intrinsics won't be saved.")
                                .font(.caption2)
                                .foregroundColor(.orange.opacity(0.8))
                        }
                        .padding(8)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .background(Color.orange.opacity(0.1))
                        .cornerRadius(6)
                    }
                }
            }
        }
    }
    
    private func storageDescription(for location: RecordingStorageLocation) -> String {
        switch location {
        case .local:
            return "App-only storage"
        case .iCloudDrive:
            return "Syncs across devices"
        case .documentsFolder:
            return "Accessible via Files app"
        }
    }
    
    private var statusPositionPanelContent: some View {
        VStack(alignment: .leading, spacing: 10) {
            // X position control
            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    Text("X (Left-Right)")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.7))
                    Spacer()
                    Text("\(String(format: "%.2f", dataManager.statusMinimizedXPosition))m")
                        .font(.caption)
                        .foregroundColor(.white)
                        .monospacedDigit()
                }
                
                Slider(value: Binding(
                    get: { dataManager.statusMinimizedXPosition },
                    set: { newValue in
                        dataManager.statusMinimizedXPosition = newValue
                        previewStatusPosition = (x: newValue, y: dataManager.statusMinimizedYPosition)
                        previewStatusActive = true
                        hidePreviewTask?.cancel()
                        hidePreviewTask = Task { @MainActor in
                            try? await Task.sleep(nanoseconds: 3_000_000_000)
                            if !Task.isCancelled {
                                previewStatusPosition = nil
                                previewStatusActive = false
                            }
                        }
                    }
                ), in: -0.5...0.5, step: 0.05)
                .tint(.purple)
            }
            
            // Y position control
            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    Text("Y (Up-Down)")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.7))
                    Spacer()
                    Text("\(String(format: "%.2f", dataManager.statusMinimizedYPosition))m")
                        .font(.caption)
                        .foregroundColor(.white)
                        .monospacedDigit()
                }
                
                Slider(value: Binding(
                    get: { dataManager.statusMinimizedYPosition },
                    set: { newValue in
                        dataManager.statusMinimizedYPosition = newValue
                        previewStatusPosition = (x: dataManager.statusMinimizedXPosition, y: newValue)
                        previewStatusActive = true
                        hidePreviewTask?.cancel()
                        hidePreviewTask = Task { @MainActor in
                            try? await Task.sleep(nanoseconds: 3_000_000_000)
                            if !Task.isCancelled {
                                previewStatusPosition = nil
                                previewStatusActive = false
                            }
                        }
                    }
                ), in: -0.5...0.5, step: 0.05)
                .tint(.purple)
            }
            
            Button {
                dataManager.statusMinimizedXPosition = 0.0
                dataManager.statusMinimizedYPosition = -0.3
                previewStatusPosition = (x: 0.0, y: -0.3)
                previewStatusActive = true
                hidePreviewTask?.cancel()
                hidePreviewTask = Task { @MainActor in
                    try? await Task.sleep(nanoseconds: 3_000_000_000)
                    if !Task.isCancelled {
                        previewStatusPosition = nil
                        previewStatusActive = false
                    }
                }
            } label: {
                Text("Reset")
                    .font(.caption2)
                    .foregroundColor(.purple)
                    .padding(.horizontal, 10)
                    .padding(.vertical, 6)
                    .frame(maxWidth: .infinity)
                    .background(Color.white.opacity(0.15))
                    .cornerRadius(6)
            }
            .buttonStyle(.plain)
        }
    }
    
    private var calibrationPanelContent: some View {
        VStack(alignment: .leading, spacing: 12) {
            // Camera status
            if let device = uvcCameraManager.selectedDevice {
                let hasCalibration = calibrationManager.hasCalibration(for: device.id)
                
                VStack(alignment: .leading, spacing: 4) {
                    Text(device.name)
                        .font(.caption)
                        .fontWeight(.medium)
                        .foregroundColor(.white)
                    
                    HStack(spacing: 4) {
                        Image(systemName: hasCalibration ? "checkmark.circle.fill" : "exclamationmark.circle.fill")
                            .font(.caption)
                            .foregroundColor(hasCalibration ? .green : .orange)
                        Text(hasCalibration ? "Calibrated" : "Not Calibrated")
                            .font(.caption)
                            .foregroundColor(hasCalibration ? .green : .orange)
                    }
                }
                .padding(.horizontal, 10)
                .padding(.vertical, 8)
                .frame(maxWidth: .infinity, alignment: .leading)
                .background(hasCalibration ? Color.green.opacity(0.15) : Color.orange.opacity(0.15))
                .cornerRadius(8)
                
                // Show calibration details if available
                if hasCalibration, let calibration = calibrationManager.allCalibrations[device.id] {
                    VStack(alignment: .leading, spacing: 6) {
                        // Left camera intrinsics (or mono)
                        Text(calibration.isStereo ? "Left Intrinsic Matrix K_L" : "Intrinsic Matrix K")
                            .font(.caption2)
                            .fontWeight(.medium)
                            .foregroundColor(.white.opacity(0.7))
                        
                        let fx = calibration.leftIntrinsics.fx
                        let fy = calibration.leftIntrinsics.fy
                        let cx = calibration.leftIntrinsics.cx
                        let cy = calibration.leftIntrinsics.cy
                        
                        // Display as 3x3 matrix
                        VStack(alignment: .leading, spacing: 2) {
                            Text("⎡ \(String(format: "%7.1f", fx))   \(String(format: "%7.1f", 0.0))   \(String(format: "%7.1f", cx)) ⎤")
                                .font(.system(size: 9, design: .monospaced))
                                .foregroundColor(.cyan)
                            Text("⎢ \(String(format: "%7.1f", 0.0))   \(String(format: "%7.1f", fy))   \(String(format: "%7.1f", cy)) ⎥")
                                .font(.system(size: 9, design: .monospaced))
                                .foregroundColor(.cyan)
                            Text("⎣ \(String(format: "%7.1f", 0.0))   \(String(format: "%7.1f", 0.0))   \(String(format: "%7.1f", 1.0)) ⎦")
                                .font(.system(size: 9, design: .monospaced))
                                .foregroundColor(.cyan)
                        }
                        
                        Text("Reproj: \(String(format: "%.4f px", calibration.leftIntrinsics.reprojectionError))")
                            .font(.caption2)
                            .foregroundColor(calibration.leftIntrinsics.reprojectionError < 0.5 ? .green : (calibration.leftIntrinsics.reprojectionError < 1.0 ? .orange : .red))
                        
                        // Right camera intrinsics (stereo only)
                        if calibration.isStereo, let rightIntrinsics = calibration.rightIntrinsics {
                            Text("Right Intrinsic Matrix K_R")
                                .font(.caption2)
                                .fontWeight(.medium)
                                .foregroundColor(.white.opacity(0.7))
                                .padding(.top, 4)
                            
                            let rfx = rightIntrinsics.fx
                            let rfy = rightIntrinsics.fy
                            let rcx = rightIntrinsics.cx
                            let rcy = rightIntrinsics.cy
                            
                            VStack(alignment: .leading, spacing: 2) {
                                Text("⎡ \(String(format: "%7.1f", rfx))   \(String(format: "%7.1f", 0.0))   \(String(format: "%7.1f", rcx)) ⎤")
                                    .font(.system(size: 9, design: .monospaced))
                                    .foregroundColor(.orange)
                                Text("⎢ \(String(format: "%7.1f", 0.0))   \(String(format: "%7.1f", rfy))   \(String(format: "%7.1f", rcy)) ⎥")
                                    .font(.system(size: 9, design: .monospaced))
                                    .foregroundColor(.orange)
                                Text("⎣ \(String(format: "%7.1f", 0.0))   \(String(format: "%7.1f", 0.0))   \(String(format: "%7.1f", 1.0)) ⎦")
                                    .font(.system(size: 9, design: .monospaced))
                                    .foregroundColor(.orange)
                            }
                            
                            Text("Reproj: \(String(format: "%.4f px", rightIntrinsics.reprojectionError))")
                                .font(.caption2)
                                .foregroundColor(rightIntrinsics.reprojectionError < 0.5 ? .green : (rightIntrinsics.reprojectionError < 1.0 ? .orange : .red))
                            
                            // Stereo extrinsics
                            if let stereo = calibration.stereoExtrinsics {
                                Text("Stereo Reproj: \(String(format: "%.4f px", stereo.stereoReprojectionError))")
                                    .font(.caption2)
                                    .foregroundColor(stereo.stereoReprojectionError < 1.0 ? .green : (stereo.stereoReprojectionError < 2.0 ? .orange : .red))
                                    .padding(.top, 2)
                            }
                        }
                        
                        // Image size and metadata
                        HStack {
                            Text(calibration.isStereo ? "Stereo" : "Mono")
                                .font(.caption2)
                                .padding(.horizontal, 4)
                                .padding(.vertical, 1)
                                .background(calibration.isStereo ? Color.blue.opacity(0.3) : Color.gray.opacity(0.3))
                                .cornerRadius(3)
                            Text("\(calibration.leftIntrinsics.imageWidth)×\(calibration.leftIntrinsics.imageHeight)")
                                .font(.caption2)
                                .foregroundColor(.white.opacity(0.5))
                            Text("•")
                                .foregroundColor(.white.opacity(0.3))
                            Text("\(calibration.sampleCount) samples")
                                .font(.caption2)
                                .foregroundColor(.white.opacity(0.5))
                        }
                        .padding(.top, 4)
                        
                        Text(calibration.calibrationDate, style: .date)
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.4))
                    }
                    .padding(.horizontal, 10)
                    .padding(.vertical, 8)
                    .frame(maxWidth: .infinity, alignment: .leading)
                    .background(Color.white.opacity(0.1))
                    .cornerRadius(8)
                }
                
                // Why calibration matters
                if !hasCalibration {
                    VStack(alignment: .leading, spacing: 4) {
                        Text("Why Calibrate?")
                            .font(.caption2)
                            .fontWeight(.medium)
                            .foregroundColor(.white.opacity(0.8))
                        Text("Intrinsic calibration enables accurate pose estimation and is stored with recordings for later analysis.")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.5))
                    }
                    .padding(.horizontal, 10)
                    .padding(.vertical, 8)
                    .background(Color.white.opacity(0.05))
                    .cornerRadius(8)
                }
                
                // Calibrate button
                Button {
                    print("🔘 [StatusView] Start Calibration button tapped!")
                    print("🔘 [StatusView] showCalibrationSheet was: \(showCalibrationSheet)")
                    showCalibrationSheet = true
                    print("🔘 [StatusView] showCalibrationSheet now: \(showCalibrationSheet)")
                } label: {
                    HStack {
                        Image(systemName: "camera.viewfinder")
                            .font(.system(size: 12, weight: .bold))
                        Text(hasCalibration ? "Recalibrate" : "Start Calibration")
                            .font(.caption)
                            .fontWeight(.semibold)
                    }
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 10)
                    .background(Color.cyan)
                    .cornerRadius(8)
                }
                .buttonStyle(.plain)
                
            } else {
                // No camera selected
                VStack(spacing: 8) {
                    Image(systemName: "video.slash")
                        .font(.title2)
                        .foregroundColor(.white.opacity(0.5))
                    Text("No camera selected")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                }
                .frame(maxWidth: .infinity)
                .padding()
            }
        }
    }
    
    private var extrinsicCalibrationPanelContent: some View {
        VStack(alignment: .leading, spacing: 12) {
            // Camera status
            if let device = uvcCameraManager.selectedDevice {
                let hasIntrinsic = calibrationManager.hasCalibration(for: device.id)
                let hasExtrinsic = extrinsicCalibrationManager.hasCalibration(for: device.id)
                
                // Status header
                VStack(alignment: .leading, spacing: 4) {
                    Text(device.name)
                        .font(.caption)
                        .fontWeight(.medium)
                        .foregroundColor(.white)
                    
                    HStack(spacing: 4) {
                        Image(systemName: hasExtrinsic ? "checkmark.circle.fill" : "exclamationmark.circle.fill")
                            .font(.caption)
                            .foregroundColor(hasExtrinsic ? .green : .orange)
                        Text(hasExtrinsic ? "Extrinsic Calibrated" : "Not Calibrated")
                            .font(.caption)
                            .foregroundColor(hasExtrinsic ? .green : .orange)
                    }
                }
                .padding(.horizontal, 10)
                .padding(.vertical, 8)
                .frame(maxWidth: .infinity, alignment: .leading)
                .background(hasExtrinsic ? Color.green.opacity(0.15) : Color.orange.opacity(0.15))
                .cornerRadius(8)
                
                // Show calibration details if available
                if hasExtrinsic, let calibration = extrinsicCalibrationManager.allCalibrations[device.id] {
                    VStack(alignment: .leading, spacing: 6) {
                        // Mono/Stereo badge
                        HStack {
                            Text(calibration.isStereo ? "Stereo" : "Mono")
                                .font(.caption2)
                                .fontWeight(.semibold)
                                .foregroundColor(.white)
                                .padding(.horizontal, 6)
                                .padding(.vertical, 2)
                                .background(calibration.isStereo ? Color.blue.opacity(0.5) : Color.gray.opacity(0.5))
                                .cornerRadius(4)
                            Spacer()
                        }
                        
                        // Left/Mono camera transform
                        Text(calibration.isStereo ? "T_head^left_camera (Head to Left)" : "T_head^camera (Head to Camera)")
                            .font(.caption2)
                            .fontWeight(.medium)
                            .foregroundColor(.white.opacity(0.7))
                        
                        // Display translation
                        let t = calibration.leftHeadToCameraMatrix
                        let tx = t.columns.3.x
                        let ty = t.columns.3.y
                        let tz = t.columns.3.z
                        
                        Text("Translation: (\(String(format: "%.3f", tx)), \(String(format: "%.3f", ty)), \(String(format: "%.3f", tz))) m")
                            .font(.system(size: 9, design: .monospaced))
                            .foregroundColor(.purple)
                        
                        // Left reprojection error
                        Text("Reproj: \(String(format: "%.4f m", calibration.leftReprojectionError)) (\(calibration.leftSampleCount) samples)")
                            .font(.caption2)
                            .foregroundColor(calibration.leftReprojectionError < 0.01 ? .green : (calibration.leftReprojectionError < 0.05 ? .orange : .red))
                        
                        // Right camera transform (stereo only)
                        if calibration.isStereo, let rightMatrix = calibration.rightHeadToCameraMatrix {
                            Divider()
                                .background(Color.white.opacity(0.2))
                            
                            Text("T_head^right_camera (Head to Right)")
                                .font(.caption2)
                                .fontWeight(.medium)
                                .foregroundColor(.white.opacity(0.7))
                            
                            let rtx = rightMatrix.columns.3.x
                            let rty = rightMatrix.columns.3.y
                            let rtz = rightMatrix.columns.3.z
                            
                            Text("Translation: (\(String(format: "%.3f", rtx)), \(String(format: "%.3f", rty)), \(String(format: "%.3f", rtz))) m")
                                .font(.system(size: 9, design: .monospaced))
                                .foregroundColor(.cyan)
                            
                            if let rightError = calibration.rightReprojectionError, let rightCount = calibration.rightSampleCount {
                                Text("Reproj: \(String(format: "%.4f m", rightError)) (\(rightCount) samples)")
                                    .font(.caption2)
                                    .foregroundColor(rightError < 0.01 ? .green : (rightError < 0.05 ? .orange : .red))
                            }
                        }
                        
                        // Metadata
                        HStack {
                            Text("\(calibration.markerCount) markers")
                                .font(.caption2)
                                .foregroundColor(.white.opacity(0.5))
                        }
                        .padding(.top, 2)
                        
                        Text(calibration.calibrationDate, style: .date)
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.4))
                    }
                    .padding(.horizontal, 10)
                    .padding(.vertical, 8)
                    .frame(maxWidth: .infinity, alignment: .leading)
                    .background(Color.white.opacity(0.1))
                    .cornerRadius(8)
                }
                
                // Prerequisites check
                if !hasIntrinsic {
                    VStack(alignment: .leading, spacing: 4) {
                        HStack(spacing: 6) {
                            Image(systemName: "exclamationmark.triangle.fill")
                                .font(.caption)
                                .foregroundColor(.orange)
                            Text("Intrinsic Calibration Required")
                                .font(.caption2)
                                .fontWeight(.medium)
                                .foregroundColor(.orange)
                        }
                        Text("Please complete intrinsic calibration first for accurate extrinsic calibration.")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.5))
                    }
                    .padding(.horizontal, 10)
                    .padding(.vertical, 8)
                    .background(Color.orange.opacity(0.1))
                    .cornerRadius(8)
                }
                
                // What is extrinsic calibration
                if !hasExtrinsic && hasIntrinsic {
                    VStack(alignment: .leading, spacing: 4) {
                        Text("What is Extrinsic Calibration?")
                            .font(.caption2)
                            .fontWeight(.medium)
                            .foregroundColor(.white.opacity(0.8))
                        Text("Determines the rigid transform between Vision Pro's head and your external camera. Required for teleoperation when camera is mounted to the headset.")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.5))
                    }
                    .padding(.horizontal, 10)
                    .padding(.vertical, 8)
                    .background(Color.white.opacity(0.05))
                    .cornerRadius(8)
                    
                    // Instructions
                    VStack(alignment: .leading, spacing: 4) {
                        Text("How to Calibrate:")
                            .font(.caption2)
                            .fontWeight(.medium)
                            .foregroundColor(.white.opacity(0.8))
                        Text("1. Print ArUco markers (see utils/generate_aruco_markers.py)")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.5))
                        Text("2. Place markers in your environment")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.5))
                        Text("3. Point camera at markers while moving head")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.5))
                    }
                    .padding(.horizontal, 10)
                    .padding(.vertical, 8)
                    .background(Color.white.opacity(0.05))
                    .cornerRadius(8)
                }
                
                // Settings
                VStack(alignment: .leading, spacing: 8) {
                    Text("Marker Settings")
                        .font(.caption2)
                        .fontWeight(.medium)
                        .foregroundColor(.white.opacity(0.7))
                    
                    HStack {
                        Text("Marker Size")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.8))
                        Spacer()
                        Text("\(Int(extrinsicCalibrationManager.markerSizeMeters * 1000)) mm")
                            .font(.caption)
                            .foregroundColor(.white)
                            .monospacedDigit()
                    }
                    
                    Slider(value: Binding(
                        get: { extrinsicCalibrationManager.markerSizeMeters * 1000 },
                        set: { extrinsicCalibrationManager.markerSizeMeters = $0 / 1000 }
                    ), in: 50...200, step: 10)
                    .tint(.purple)
                }
                .padding(.horizontal, 10)
                .padding(.vertical, 8)
                .background(Color.white.opacity(0.05))
                .cornerRadius(8)
                
                // Calibrate button
                Button {
                    if hasIntrinsic {
                        showExtrinsicCalibrationSheet = true
                    }
                } label: {
                    HStack {
                        Image(systemName: "arrow.triangle.swap")
                            .font(.system(size: 12, weight: .bold))
                        Text(hasExtrinsic ? "Recalibrate" : "Start Calibration")
                            .font(.caption)
                            .fontWeight(.semibold)
                    }
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 10)
                    .background(hasIntrinsic ? Color.purple : Color.gray)
                    .cornerRadius(8)
                }
                .buttonStyle(.plain)
                .disabled(!hasIntrinsic)
                
                // Delete button if calibration exists
                if hasExtrinsic {
                    Button {
                        extrinsicCalibrationManager.deleteCalibration(for: device.id)
                    } label: {
                        HStack {
                            Image(systemName: "trash")
                                .font(.system(size: 12, weight: .bold))
                            Text("Delete Calibration")
                                .font(.caption)
                                .fontWeight(.semibold)
                        }
                        .foregroundColor(.red)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 10)
                        .background(Color.red.opacity(0.15))
                        .cornerRadius(8)
                    }
                    .buttonStyle(.plain)
                }
                
            } else {
                // No camera selected
                VStack(spacing: 8) {
                    Image(systemName: "video.slash")
                        .font(.title2)
                        .foregroundColor(.white.opacity(0.5))
                    Text("No camera selected")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                }
                .frame(maxWidth: .infinity)
                .padding()
            }
        }
    }
}

/// Preview view that looks exactly like the minimized status but with 50% opacity
struct StatusPreviewView: View {
    let showVideoStatus: Bool
    let videoFixed: Bool
    
    var body: some View {
        HStack(spacing: 16) {
            // Expand button (non-functional in preview)
            ZStack {
                Circle()
                    .fill(Color.white.opacity(0.3))
                    .frame(width: 60, height: 60)
                Image(systemName: "arrow.up.left.and.arrow.down.right")
                    .font(.system(size: 24, weight: .bold))
                    .foregroundColor(.white)
            }
            
            // Recording button (non-functional in preview)
            ZStack {
                Circle()
                    .fill(Color.red.opacity(0.8))
                    .frame(width: 60, height: 60)
                Circle()
                    .fill(Color.white)
                    .frame(width: 24, height: 24)
            }
            
            // Video minimize/maximize button (only show if video streaming mode is enabled)
            if showVideoStatus {
                ZStack {
                    Circle()
                        .fill(Color.blue.opacity(0.8))
                        .frame(width: 60, height: 60)
                    Image(systemName: "video.fill")
                        .font(.system(size: 24, weight: .bold))
                        .foregroundColor(.white)
                }
            
                ZStack {
                    Circle()
                        .fill(videoFixed ? Color.orange.opacity(0.8) : Color.white.opacity(0.3))
                        .frame(width: 60, height: 60)
                    Image(systemName: videoFixed ? "lock.fill" : "lock.open.fill")
                        .font(.system(size: 24, weight: .bold))
                        .foregroundColor(.white)
                }
            }
            
            // Close button (non-functional in preview)
            ZStack {
                Circle()
                    .fill(Color.red)
                    .frame(width: 60, height: 60)
                Text("✕")
                    .font(.system(size: 27, weight: .bold))
                    .foregroundColor(.white)
            }
        }
        .padding(30)
        .background(Color.black.opacity(0.6))
        .cornerRadius(36)
        .fixedSize()
        .opacity(0.5)  // 50% transparent
    }
}

/// Creates a floating status entity that follows the head
func createStatusEntity() -> Entity {
    let statusEntity = Entity()
    statusEntity.name = "statusDisplay"
    return statusEntity
}
