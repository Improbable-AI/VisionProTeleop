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
    case cameraCalibration  // Unified camera calibration
    case cloudStorageDebug
    case visualizations  // Hand/head visualization toggles
    case positionLayout  // Combined video view + controller position
    case personaCapture  // Spatial Persona front camera preview
}

/// App mode: Teleop (network-based teleoperation) vs Egorecord (local UVC recording)
enum AppMode: String, CaseIterable {
    case teleop = "teleop"
    case egorecord = "egorecord"
    
    var displayName: String {
        switch self {
        case .teleop: return "Teleoperation"
        case .egorecord: return "EgoRecord"
        }
    }
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
    @ObservedObject private var uvcCameraManager = UVCCameraManager.shared
    @ObservedObject private var recordingManager = RecordingManager.shared
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
    @ObservedObject private var calibrationManager = CameraCalibrationManager.shared
    @ObservedObject private var extrinsicCalibrationManager = ExtrinsicCalibrationManager.shared
    @ObservedObject private var cloudStorageSettings = CloudStorageSettings.shared
    
    // App mode persistence
    @AppStorage("appMode") private var appMode: AppMode = .teleop
    // Remember the video source used in teleop mode so we can restore it when switching back from egorecord
    @AppStorage("teleopVideoSource") private var teleopVideoSource: String = VideoSource.network.rawValue
    
    // Flashing animation for warnings
    @State private var flashingOpacity: Double = 1.0
    
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
        print("🟡 [StatusView] StatusOverlay body called")
        return ZStack {
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
                ExtrinsicCalibrationViewNew(onDismiss: {
                    showExtrinsicCalibrationSheet = false
                })
                .frame(width: 500, height: 700)
                .background(.regularMaterial)
                .cornerRadius(20)
                .shadow(radius: 20)
            }
            
            // Camera Calibration Wizard overlay
            if showCalibrationWizard {
                CameraCalibrationWizardView(onDismiss: {
                    showCalibrationWizard = false
                })
                .frame(width: 520, height: 720)
                .background(.regularMaterial)
                .cornerRadius(20)
                .shadow(radius: 20)
            }
        }
        .onAppear {
            print("🔴 [StatusView] StatusOverlay onAppear called")
            ipAddresses = getIPAddresses()
            print("🔴 [StatusView] IP Addresses: \(ipAddresses)")
            
            // Flashing animation for warnings (gentle pulse 1.5s cycle)
            Timer.scheduledTimer(withTimeInterval: 1.5, repeats: true) { _ in
                withAnimation(.easeInOut(duration: 0.75)) {
                    flashingOpacity = flashingOpacity == 1.0 ? 0.4 : 1.0
                }
            }
            
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
                // But don't maximize if we're currently uploading to cloud - wait for upload to complete
                if (wasPythonConnected && !pythonConnected) || (wasWebrtcConnected && !webrtcConnected) {
                    if recordingManager.isUploadingToCloud {
                        print("🔌 [StatusView] Connection lost but upload in progress - staying minimized")
                        // Just reset flags, don't maximize yet
                        userInteracted = false
                        hasFrames = false
                    } else {
                        print("🔌 [StatusView] Connection lost - maximizing status view")
                        withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                            isMinimized = false
                            userInteracted = false  // Reset so it can auto-minimize again on next connection
                            hasFrames = false  // Clear frames flag
                        }
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
                
                // Check if upload just finished and we should maximize (deferred maximize after upload)
                // If no Python connected, not uploading, and still minimized without user interaction
                if !pythonConnected && !recordingManager.isUploadingToCloud && isMinimized && !userInteracted {
                    // Additional check: only maximize if we were waiting for upload to finish
                    // This is triggered periodically, so we need to detect the transition
                    // We rely on the fact that hasFrames was set to false when connection was lost
                    if !hasFrames {
                        print("☁️ [StatusView] Upload completed and no connection - maximizing status view")
                        withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                            isMinimized = false
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
            
            // Cloud upload progress indicator (show above buttons when uploading)
            if recordingManager.isUploadingToCloud {
                VStack(spacing: 6) {
                    // Destination header
                    HStack(spacing: 6) {
                        Image(systemName: recordingManager.cloudProvider.icon)
                            .font(.system(size: 16, weight: .semibold))
                            .foregroundColor(recordingManager.cloudProvider.color)
                        
                        Text("Uploading to \(recordingManager.cloudProvider.displayName)")
                            .font(.system(size: 14, weight: .semibold))
                            .foregroundColor(.white)
                        
                        // Spinning indicator
                        ProgressView()
                            .progressViewStyle(CircularProgressViewStyle(tint: recordingManager.cloudProvider.color))
                            .scaleEffect(0.7)
                    }
                    
                    // Current file being uploaded
                    if !recordingManager.cloudUploadCurrentFileName.isEmpty {
                        Text(recordingManager.cloudUploadCurrentFileName)
                            .font(.system(size: 11, design: .monospaced))
                            .foregroundColor(.white.opacity(0.7))
                            .lineLimit(1)
                            .truncationMode(.middle)
                    }
                    
                    let progress = recordingManager.cloudUploadTotalFiles > 0 
                        ? Double(recordingManager.cloudUploadCurrentFile) / Double(recordingManager.cloudUploadTotalFiles) 
                        : 0.0
                    
                    // Progress bar with percentage
                    HStack(spacing: 8) {
                        ZStack(alignment: .leading) {
                            RoundedRectangle(cornerRadius: 4)
                                .fill(Color.white.opacity(0.2))
                                .frame(width: 120, height: 8)
                            
                            RoundedRectangle(cornerRadius: 4)
                                .fill(recordingManager.cloudProvider.color)
                                .frame(width: 120 * progress, height: 8)
                                .animation(.easeInOut(duration: 0.3), value: progress)
                        }
                        
                        Text("\(recordingManager.cloudUploadCurrentFile)/\(recordingManager.cloudUploadTotalFiles)")
                            .font(.system(size: 12, weight: .bold, design: .monospaced))
                            .foregroundColor(.white)
                        
                        Text("(\(Int(progress * 100))%)")
                            .font(.system(size: 11, weight: .medium))
                            .foregroundColor(.white.opacity(0.7))
                    }
                }
                .padding(.horizontal, 20)
                .padding(.vertical, 12)
                .background(
                    RoundedRectangle(cornerRadius: 16)
                        .fill(recordingManager.cloudProvider.color.opacity(0.25))
                        .overlay(
                            RoundedRectangle(cornerRadius: 16)
                                .stroke(recordingManager.cloudProvider.color.opacity(0.5), lineWidth: 1)
                        )
                )
            }
            
            HStack(spacing: 16) {
                if showLocalExitConfirmation {
                    // Confirmation mode with upload warning
                    VStack(spacing: 8) {
                        if recordingManager.isUploadingToCloud {
                            HStack(spacing: 4) {
                                Image(systemName: "exclamationmark.triangle.fill")
                                    .font(.system(size: 12))
                                    .foregroundColor(.orange)
                                Text("Upload in progress!")
                                    .font(.caption)
                                    .foregroundColor(.orange)
                            }
                        }
                        Text("Exit?")
                            .font(.headline)
                            .foregroundColor(.white)
                    }
                    
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
            
            // Storage location indicator (minimal line below buttons)
            HStack(spacing: 4) {
                if recordingManager.storageLocation == .cloud {
                    Image(systemName: recordingManager.cloudProvider.icon)
                        .font(.system(size: 10))
                    Text(recordingManager.cloudProvider.displayName)
                        .font(.system(size: 10))
                } else {
                    Image(systemName: "internaldrive")
                        .font(.system(size: 10))
                    Text("Local")
                        .font(.system(size: 10))
                }
            }
            .foregroundColor(.white.opacity(0.5))
        }
        .padding(30)
        .background(Color.black.opacity(0.6))
        .cornerRadius(36)
        .fixedSize()
    }

    private var headerSection: some View {
        VStack(spacing: 12) {
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
    }
    
    private var modeToggleSection: some View {
        HStack(spacing: 0) {
            ForEach(AppMode.allCases, id: \.self) { mode in
                Button {
                    withAnimation(.spring(response: 0.3, dampingFraction: 0.8)) {
                        // When switching to egorecord, save current video source and force UVC
                        if mode == .egorecord && appMode == .teleop {
                            teleopVideoSource = dataManager.videoSource.rawValue
                            dataManager.videoSource = .uvcCamera
                        }
                        // When switching back to teleop, restore the previous video source
                        else if mode == .teleop && appMode == .egorecord {
                            if let savedSource = VideoSource(rawValue: teleopVideoSource) {
                                dataManager.videoSource = savedSource
                            }
                        }
                        appMode = mode
                    }
                } label: {
                    Text(mode.displayName)
                        .foregroundColor(appMode == mode ? .white : .white.opacity(0.5))
                        .padding(.horizontal, 20)
                        .padding(.vertical, 8)
                        .font(.system(size: 18, weight: .bold))
                        .background(
                            appMode == mode 
                                ? (mode == .teleop ? Color.blue : Color.orange)
                                : Color.clear
                        )
                        .cornerRadius(16)
                }
                .buttonStyle(.plain)
            }
        }
        .padding(4)
        .background(Color.white.opacity(0.15))
        .cornerRadius(20)
    }

    private var networkInfoSection: some View {
        VStack(alignment: .leading, spacing: 10) {
            // IP addresses
            ForEach(ipAddresses, id: \.address) { ip in
                HStack(spacing: 8) {
                    Image(systemName: "network")
                        .font(.system(size: 13))
                        .foregroundColor(.blue.opacity(0.8))
                    Text(ip.name)
                        .font(.callout)
                        .foregroundColor(.white.opacity(0.6))
                    Spacer()
                    Text(ip.address)
                        .font(.system(.callout, design: .monospaced))
                        .foregroundColor(.white)
                        .fontWeight(.medium)
                }
            }
            
            // Connection status cards
            HStack(spacing: 8) {
                // Python connection card
                connectionStatusCard(
                    icon: "terminal.fill",
                    title: "Python",
                    status: pythonConnected ? pythonIP : "Waiting...",
                    isConnected: pythonConnected,
                    accentColor: .green
                )
                
                // WebRTC connection card (only if video mode)
                if showVideoStatus {
                    connectionStatusCard(
                        icon: "video.fill",
                        title: "WebRTC",
                        status: webrtcConnected ? "Connected" : "Waiting...",
                        isConnected: webrtcConnected,
                        accentColor: .purple
                    )
                }
            }
        }
    }
    
    private func connectionStatusCard(icon: String, title: String, status: String, isConnected: Bool, accentColor: Color) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack(spacing: 6) {
                Image(systemName: icon)
                    .font(.system(size: 10))
                    .foregroundColor(isConnected ? accentColor : .white.opacity(0.4))
                Text(title)
                    .font(.caption2)
                    .fontWeight(.semibold)
                    .foregroundColor(.white.opacity(0.7))
                Spacer()
                Circle()
                    .fill(isConnected ? accentColor : Color.white.opacity(0.2))
                    .frame(width: 6, height: 6)
            }
            
            Text(status)
                .font(.system(.caption2, design: .monospaced))
                .foregroundColor(isConnected ? .white : .white.opacity(0.4))
                .lineLimit(1)
                .truncationMode(.middle)
        }
        .padding(10)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(isConnected ? accentColor.opacity(0.15) : Color.white.opacity(0.05))
        .cornerRadius(10)
        .overlay(
            RoundedRectangle(cornerRadius: 10)
                .stroke(isConnected ? accentColor.opacity(0.3) : Color.clear, lineWidth: 1)
        )
    }
    
    private var egorecordInfoSection: some View {
        VStack(alignment: .leading, spacing: 8) {
            // Info description
            HStack(alignment: .top, spacing: 10) {
                Image(systemName: "info.circle.fill")
                    .font(.system(size: 16))
                    .foregroundColor(.white.opacity(0.5))
                
                Text("Records egocentric video and hand/head tracking. This mode requires:")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.7))
                    .fixedSize(horizontal: false, vertical: true)
            }
            
            // Requirements list - horizontal layout
            HStack(spacing: 12) {
                requirementRow(number: "1", text: "Camera")
                requirementRow(number: "2", text: "Dev Strap")
                requirementRow(number: "3", text: "Cam Mount")
            }
            .padding(.leading, 26) // Align with text above
        }
    }
    
    private func requirementRow(number: String, text: String) -> some View {
        HStack(spacing: 8) {
            Text(number + ".")
                .font(.caption)
                .fontWeight(.medium)
                .foregroundColor(.white.opacity(0.6))
                .frame(width: 14, alignment: .leading)
            Text(text)
                .font(.caption)
                .foregroundColor(.white.opacity(0.8))
        }
    }
    
    private var teleopInfoSection: some View {
        VStack(alignment: .leading, spacing: 8) {
            // Info description
            HStack(alignment: .top, spacing: 10) {
                Image(systemName: "info.circle.fill")
                    .font(.system(size: 16))
                    .foregroundColor(.white.opacity(0.5))
                
                Text("Requires a Python client to receive hand tracking data and stream back:")
                    .font(.caption)
                    .foregroundColor(.white.opacity(0.7))
                    .fixedSize(horizontal: false, vertical: true)
            }
            
            // Capabilities list
            HStack(spacing: 16) {
                capabilityRow(icon: "video.fill", text: "Video")
                capabilityRow(icon: "waveform", text: "Audio")
                capabilityRow(icon: "cube.transparent", text: "Sim states")
            }
            .padding(.leading, 26) // Align with text above
        }
    }
    
    private func capabilityRow(icon: String, text: String) -> some View {
        HStack(spacing: 8) {
            Image(systemName: icon)
                .font(.system(size: 10))
                .foregroundColor(.white.opacity(0.5))
                .frame(width: 14)
            Text(text)
                .font(.caption)
                .foregroundColor(.white.opacity(0.8))
        }
    }
    
    // MARK: - Version Incompatibility Warning
    
    private var versionWarningBanner: some View {
        VStack(alignment: .leading, spacing: 8) {
            // Header with warning icon
            HStack(spacing: 8) {
                Image(systemName: "exclamationmark.triangle.fill")
                    .font(.system(size: 16))
                    .foregroundColor(.yellow)
                    .opacity(flashingOpacity)
                
                Text("Python Library Update Required")
                    .font(.callout)
                    .fontWeight(.bold)
                    .foregroundColor(.white)
            }
            
            // Version info
            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    Text("Your version:")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.6))
                    Spacer()
                    Text(dataManager.pythonLibraryVersionString)
                        .font(.system(.caption, design: .monospaced))
                        .fontWeight(.medium)
                        .foregroundColor(.red)
                }
                
                HStack {
                    Text("Minimum required:")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.6))
                    Spacer()
                    Text(DataManager.minimumPythonVersionString)
                        .font(.system(.caption, design: .monospaced))
                        .fontWeight(.medium)
                        .foregroundColor(.green)
                }
            }
            .padding(8)
            .background(Color.black.opacity(0.3))
            .cornerRadius(6)
            
            // Upgrade instructions
            HStack(spacing: 4) {
                Text("Run:")
                    .font(.caption)
                    .foregroundColor(.white.opacity(0.7))
                Text("pip install --upgrade avp-stream")
                    .font(.system(.caption, design: .monospaced))
                    .foregroundColor(.cyan)
                    .padding(.horizontal, 6)
                    .padding(.vertical, 4)
                    .background(Color.black.opacity(0.4))
                    .cornerRadius(4)
            }
            
            // Warning message
            Text("Hand tracking is blocked until you upgrade. Please update your Python library and restart.")
                .font(.caption2)
                .foregroundColor(.white.opacity(0.6))
                .fixedSize(horizontal: false, vertical: true)
        }
        .padding(12)
        .background(
            RoundedRectangle(cornerRadius: 10)
                .fill(Color.orange.opacity(0.2))
                .overlay(
                    RoundedRectangle(cornerRadius: 10)
                        .stroke(Color.orange.opacity(0.5), lineWidth: 1)
                )
        )
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
                    .foregroundColor(videoActive ? .white : .white.opacity(0.4))
                    .padding(.horizontal, 8)
                    .padding(.vertical, 3)
                    .background(Color.white.opacity(videoActive ? 0.2 : 0.08))
                    .cornerRadius(8)
                
                if isUVCMode {
                    // UVC camera info
                    if uvcCameraManager.isCapturing {
                        if uvcCameraManager.stereoEnabled {
                            Text("Stereo")
                                .font(.caption)
                                .foregroundColor(.white)
                                .fontWeight(.medium)
                        } else {
                            Text("Mono")
                                .font(.caption)
                                .foregroundColor(.white.opacity(0.7))
                        }
                        
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
                            .foregroundColor(.white)
                            .fontWeight(.medium)
                    } else {
                        Text("Mono")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.7))
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
                    .foregroundColor(dataManager.audioEnabled ? .white : .white.opacity(0.4))
                    .padding(.horizontal, 8)
                    .padding(.vertical, 3)
                    .background(Color.white.opacity(dataManager.audioEnabled ? 0.2 : 0.08))
                    .cornerRadius(8)
                
                // Status
                if dataManager.audioEnabled {
                    if dataManager.stereoAudioEnabled {
                        Text("Stereo")
                            .font(.caption)
                            .foregroundColor(.white)
                            .fontWeight(.medium)
                    } else {
                        Text("Mono")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.7))
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
                    .foregroundColor(simActive ? .white : .white.opacity(0.4))
                    .padding(.horizontal, 8)
                    .padding(.vertical, 3)
                    .background(Color.white.opacity(simActive ? 0.2 : 0.08))
                    .cornerRadius(8)
                
                if let mujoco = mujocoManager, simActive {
                    // Status
                    if mujoco.poseStreamingViaWebRTC {
                        Text("WebRTC")
                            .font(.caption)
                            .foregroundColor(.white)
                            .fontWeight(.medium)
                    } else {
                        Text("gRPC")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.8))
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
        .padding(10)
        .background(Color.white.opacity(0.05))
        .cornerRadius(10)
    }

    private var expandedView: some View {
        HStack(alignment: .center, spacing: 0) {
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
                            
                            // Upload in progress warning
                            if recordingManager.isUploadingToCloud {
                                HStack(spacing: 8) {
                                    Image(systemName: "exclamationmark.triangle.fill")
                                        .font(.system(size: 16))
                                        .foregroundColor(.orange)
                                    VStack(alignment: .leading, spacing: 2) {
                                        Text("Upload in Progress")
                                            .font(.subheadline)
                                            .fontWeight(.semibold)
                                            .foregroundColor(.orange)
                                        Text("\(recordingManager.cloudUploadCurrentFile)/\(recordingManager.cloudUploadTotalFiles) files uploading to \(recordingManager.cloudProvider.displayName)")
                                            .font(.caption)
                                            .foregroundColor(.white.opacity(0.7))
                                    }
                                }
                                .padding(.horizontal, 16)
                                .padding(.vertical, 12)
                                .background(Color.orange.opacity(0.2))
                                .cornerRadius(10)
                            }
                            
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
                                    Text(recordingManager.isUploadingToCloud ? "Exit Anyway" : "Exit")
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
        VStack(alignment: .leading, spacing: 12) {
            headerSection
            
            Divider()
                .background(Color.white.opacity(0.3))
            
            // Info sections
            if appMode == .teleop {
                teleopInfoSection
            } else if appMode == .egorecord {
                egorecordInfoSection
            }
            
            Divider()
                .background(Color.white.opacity(0.3))
            
            // Mode toggle (between info sections and network info)
            modeToggleSection
            
            // Network info only shown in Teleop mode
            if appMode == .teleop {
                // Version incompatibility warning (shown prominently when connected but incompatible)
                if dataManager.shouldShowVersionWarning {
                    versionWarningBanner
                }
                
                Divider()
                    .background(Color.white.opacity(0.2))
                
                networkInfoSection
                
                // Show detailed track information when connected (either WebRTC or UVC camera)
                let showStreamDetails = showVideoStatus && (webrtcConnected || (dataManager.videoSource == .uvcCamera && uvcCameraManager.isCapturing))
                if showStreamDetails {
                    Divider()
                        .background(Color.white.opacity(0.2))
                    
                    streamDetailsSection
                }
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
                
                // Position & Layout menu item (combined video view + controller position)
                menuItem(
                    icon: "square.resize",
                    title: "Position & Layout",
                    subtitle: nil,
                    isExpanded: expandedPanel == .positionLayout,
                    accentColor: .green
                ) {
                    withAnimation(.spring(response: 0.35, dampingFraction: 0.85)) {
                        if expandedPanel == .positionLayout {
                            expandedPanel = .none
                            previewZDistance = nil
                            previewActive = false
                            previewStatusPosition = nil
                            previewStatusActive = false
                            hidePreviewTask?.cancel()
                        } else {
                            expandedPanel = .positionLayout
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
                subtitle: recordingManager.isRecording 
                    ? recordingManager.formatDuration(recordingManager.recordingDuration) 
                    : (recordingManager.storageLocation == .cloud 
                        ? recordingManager.cloudProvider.displayName 
                        : "Local"),
                isExpanded: expandedPanel == .recording,
                accentColor: .red,
                iconColor: recordingManager.isRecording ? .red : nil
            ) {
                withAnimation(.spring(response: 0.35, dampingFraction: 0.85)) {
                    expandedPanel = expandedPanel == .recording ? .none : .recording
                }
            }
            
            // Camera Calibration menu item:
            // - Teleop mode: hidden
            // - Egorecord mode: always shown, flashes if not calibrated
            if appMode == .egorecord {
                let hasIntrinsic = uvcCameraManager.selectedDevice.map { calibrationManager.hasCalibration(for: $0.id) } ?? false
                let hasExtrinsic = uvcCameraManager.selectedDevice.map { extrinsicCalibrationManager.hasCalibration(for: $0.id) } ?? false
                let isCalibrated = hasIntrinsic && hasExtrinsic
                
                cameraCalibrationMenuItem
                    .opacity(isCalibrated ? 1.0 : flashingOpacity)
            }
            
            // Visualizations menu - only show in teleop mode
            if appMode == .teleop {
                Divider()
                    .background(Color.white.opacity(0.3))
                
                // Count active visualizations for subtitle
                let activeCount = [dataManager.upperLimbVisible, dataManager.showHeadBeam, dataManager.showHandJoints].filter { $0 }.count
                
                menuItem(
                    icon: "eye.fill",
                    title: "Debugging Visualizations",
                    subtitle: "\(activeCount)/3 active",
                    isExpanded: expandedPanel == .visualizations,
                    accentColor: .cyan
                ) {
                    withAnimation(.spring(response: 0.35, dampingFraction: 0.85)) {
                        expandedPanel = expandedPanel == .visualizations ? .none : .visualizations
                    }
                }
                
                // Persona Preview menu item
                let personaController = PersonaCaptureController.shared
                menuItem(
                    icon: personaController.isRunning ? "person.crop.circle.fill" : "person.crop.circle",
                    title: "Persona Preview",
                    subtitle: personaController.isRunning ? "Live" : "Off",
                    isExpanded: expandedPanel == .personaCapture,
                    accentColor: .purple,
                    iconColor: personaController.isRunning ? .green : nil
                ) {
                    withAnimation(.spring(response: 0.35, dampingFraction: 0.85)) {
                        expandedPanel = expandedPanel == .personaCapture ? .none : .personaCapture
                    }
                }
            }
            
            // Egorecord mode: Start Recording button with requirement checks
            if appMode == .egorecord {
                Divider()
                    .background(Color.white.opacity(0.3))
                
                // Check requirements
                let hasIntrinsic = uvcCameraManager.selectedDevice.map { calibrationManager.hasCalibration(for: $0.id) } ?? false
                let hasExtrinsic = uvcCameraManager.selectedDevice.map { extrinsicCalibrationManager.hasCalibration(for: $0.id) } ?? false
                let isCalibrated = hasIntrinsic && hasExtrinsic
                
                // Cloud storage configured check (only for non-iCloud providers)
                let isCloudConfigured: Bool = {
                    if recordingManager.storageLocation == .local { return true }
                    switch recordingManager.cloudProvider {
                    case .iCloudDrive: return true
                    case .dropbox: return cloudStorageSettings.isDropboxAvailable
                    case .googleDrive: return cloudStorageSettings.isGoogleDriveAvailable
                    }
                }()
                
                let canRecord = isCalibrated && isCloudConfigured && uvcCameraManager.selectedDevice != nil
                
                // Requirement indicators
                if !canRecord {
                    VStack(alignment: .leading, spacing: 6) {
                        if !isCalibrated {
                            HStack(spacing: 6) {
                                Image(systemName: "exclamationmark.triangle.fill")
                                    .font(.caption)
                                    .foregroundColor(.orange)
                                Text("Camera calibration required")
                                    .font(.caption)
                                    .foregroundColor(.orange)
                            }
                            .opacity(flashingOpacity)
                        }
                        if !isCloudConfigured {
                            VStack(alignment: .leading, spacing: 4) {
                                HStack(spacing: 6) {
                                    Image(systemName: "icloud.slash.fill")
                                        .font(.caption)
                                        .foregroundColor(.orange)
                                    Text("\(recordingManager.cloudProvider.displayName) not configured")
                                        .font(.caption)
                                        .foregroundColor(.orange)
                                }
                                .opacity(flashingOpacity)
                                
                                // Show Google Drive sign-in button if Google Drive is selected
                                if recordingManager.cloudProvider == .googleDrive {
                                    Button {
                                        // Minimize status view so OAuth window is visible
                                        // Set directly first, then animate
                                        expandedPanel = .none
                                        isMinimized = true
                                        userInteracted = true
                                        
                                        Task {
                                            // Delay to let UI update and minimize
                                            try? await Task.sleep(nanoseconds: 500_000_000) // 0.5 seconds
                                            await GoogleDriveAuthManager.shared.startOAuthFlow()
                                            // Restore after OAuth completes
                                            await MainActor.run {
                                                isMinimized = false
                                            }
                                        }
                                    } label: {
                                        HStack(spacing: 6) {
                                            Image(systemName: "person.crop.circle.badge.plus")
                                                .font(.caption)
                                            Text("Sign in to Google Drive")
                                                .font(.caption)
                                                .fontWeight(.medium)
                                        }
                                        .foregroundColor(.white)
                                        .padding(.horizontal, 12)
                                        .padding(.vertical, 6)
                                        .background(Color.blue)
                                        .cornerRadius(8)
                                    }
                                    .buttonStyle(.plain)
                                    .padding(.top, 4)
                                }
                            }
                        }
                        if uvcCameraManager.selectedDevice == nil {
                            HStack(spacing: 6) {
                                Image(systemName: "video.slash.fill")
                                    .font(.caption)
                                    .foregroundColor(.orange)
                                Text("No USB camera connected")
                                    .font(.caption)
                                    .foregroundColor(.orange)
                            }
                        }
                    }
                    .padding(.vertical, 6)
                }
                
                // Start Recording button
                Button {
                    if !recordingManager.isRecording {
                        recordingManager.startRecording()
                        // Auto-minimize and hide video in egorecord mode
                        withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                            isMinimized = true
                            videoMinimized = true
                            userInteracted = true
                        }
                    } else {
                        recordingManager.stopRecordingManually()
                    }
                } label: {
                    HStack(spacing: 10) {
                        Image(systemName: recordingManager.isRecording ? "stop.fill" : "record.circle.fill")
                            .font(.system(size: 20, weight: .bold))
                        Text(recordingManager.isRecording ? "Stop Recording" : "Start Recording")
                            .font(.headline)
                            .fontWeight(.bold)
                    }
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 16)
                    .background(
                        recordingManager.isRecording 
                            ? Color.red
                            : (canRecord ? Color.red.opacity(0.9) : Color.gray.opacity(0.5))
                    )
                    .cornerRadius(12)
                }
                .buttonStyle(.plain)
                .disabled(!canRecord && !recordingManager.isRecording)
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
            .padding(.vertical, 12)
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
            case .positionLayout:
                positionLayoutPanelContent
            case .cameraCalibration:
                cameraCalibrationPanelContent
            case .cloudStorageDebug:
                cloudStorageDebugPanelContent
            case .visualizations:
                visualizationsPanelContent
            case .personaCapture:
                personaCapturePanelContent
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
        case .positionLayout: return "Position & Layout"
        case .cameraCalibration: return "Camera Calibration"
        case .cloudStorageDebug: return "Cloud Storage Debug"
        case .visualizations: return "Visualizations"
        case .personaCapture: return "Persona Preview"
        case .none: return ""
        }
    }
    
    // MARK: - Right Panel Content Views
    
    private var videoSourcePanelContent: some View {
        VStack(alignment: .leading, spacing: 10) {
            // Network Stream option (disabled in Egorecord mode)
            let networkDisabled = appMode == .egorecord
            
            Button {
                if !networkDisabled {
                    withAnimation {
                        dataManager.videoSource = .network
                    }
                }
            } label: {
                HStack(spacing: 12) {
                    Image(systemName: "wifi")
                        .font(.system(size: 18, weight: .medium))
                        .frame(width: 24)
                    VStack(alignment: .leading, spacing: 3) {
                        Text("Network Stream")
                            .font(.subheadline)
                            .fontWeight(.medium)
                        Text(networkDisabled ? "Not available in Egorecord" : "WebRTC from Python")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.5))
                    }
                    Spacer()
                    if networkDisabled {
                        Image(systemName: "lock.fill")
                            .font(.system(size: 14))
                            .foregroundColor(.white.opacity(0.3))
                    } else if dataManager.videoSource == .network {
                        Image(systemName: "checkmark.circle.fill")
                            .font(.system(size: 18))
                            .foregroundColor(.green)
                    }
                }
                .padding(.horizontal, 14)
                .padding(.vertical, 12)
                .background(networkDisabled ? Color.white.opacity(0.05) : (dataManager.videoSource == .network ? Color.blue.opacity(0.3) : Color.white.opacity(0.1)))
                .cornerRadius(10)
                .foregroundColor(networkDisabled ? .white.opacity(0.4) : .white)
            }
            .buttonStyle(.plain)
            .disabled(networkDisabled)
            
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
                HStack(spacing: 12) {
                    Image(systemName: "cable.connector")
                        .font(.system(size: 18, weight: .medium))
                        .frame(width: 24)
                    VStack(alignment: .leading, spacing: 3) {
                        Text("USB Camera")
                            .font(.subheadline)
                            .fontWeight(.medium)
                        Text("UVC via Developer Strap")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.5))
                    }
                    Spacer()
                    if dataManager.videoSource == .uvcCamera {
                        Image(systemName: "checkmark.circle.fill")
                            .font(.system(size: 18))
                            .foregroundColor(.green)
                    }
                }
                .padding(.horizontal, 14)
                .padding(.vertical, 12)
                .background(dataManager.videoSource == .uvcCamera ? Color.blue.opacity(0.3) : Color.white.opacity(0.1))
                .cornerRadius(10)
                .foregroundColor(.white)
            }
            .buttonStyle(.plain)
            
            // Available cameras section (show only when USB selected)
            if dataManager.videoSource == .uvcCamera && !uvcCameraManager.availableDevices.isEmpty {
                Divider()
                    .background(Color.white.opacity(0.3))
                
                Text("Available Cameras")
                    .font(.caption)
                    .fontWeight(.medium)
                    .foregroundColor(.white.opacity(0.6))
                
                ForEach(uvcCameraManager.availableDevices.prefix(2)) { device in
                    Button {
                        uvcCameraManager.selectDevice(device)
                        Task {
                            try? await Task.sleep(nanoseconds: 200_000_000)
                            await MainActor.run { uvcCameraManager.startCapture() }
                        }
                    } label: {
                        HStack(spacing: 12) {
                            Image(systemName: "video.fill")
                                .font(.system(size: 16, weight: .medium))
                                .frame(width: 22)
                            Text(device.name)
                                .font(.subheadline)
                                .fontWeight(.medium)
                                .lineLimit(1)
                            Spacer()
                            if uvcCameraManager.selectedDevice?.id == device.id && uvcCameraManager.isCapturing {
                                Circle()
                                    .fill(Color.green)
                                    .frame(width: 8, height: 8)
                            }
                        }
                        .padding(.horizontal, 14)
                        .padding(.vertical, 14)
                        .background(uvcCameraManager.selectedDevice?.id == device.id ? Color.green.opacity(0.2) : Color.white.opacity(0.1))
                        .cornerRadius(10)
                        .foregroundColor(.white.opacity(0.9))
                    }
                    .buttonStyle(.plain)
                }
                
                // Stereo/Mono picker (show when camera is capturing)
                if uvcCameraManager.isCapturing {
                    Divider()
                        .background(Color.white.opacity(0.3))
                    
                    Text("Feed Type")
                        .font(.caption)
                        .fontWeight(.medium)
                        .foregroundColor(.white.opacity(0.6))
                    
                    HStack(spacing: 10) {
                        // Mono option
                        Button {
                            uvcCameraManager.setStereoMode(false)
                        } label: {
                            HStack(spacing: 8) {
                                Image(systemName: "rectangle")
                                    .font(.system(size: 14, weight: .medium))
                                Text("Mono")
                                    .font(.subheadline)
                                    .fontWeight(.medium)
                            }
                            .foregroundColor(!uvcCameraManager.stereoEnabled ? .white : .white.opacity(0.6))
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 12)
                            .background(!uvcCameraManager.stereoEnabled ? Color.cyan.opacity(0.4) : Color.white.opacity(0.1))
                            .cornerRadius(10)
                        }
                        .buttonStyle(.plain)
                        
                        // Stereo option
                        Button {
                            uvcCameraManager.setStereoMode(true)
                        } label: {
                            HStack(spacing: 8) {
                                Image(systemName: "rectangle.split.2x1")
                                    .font(.system(size: 14, weight: .medium))
                                Text("Stereo")
                                    .font(.subheadline)
                                    .fontWeight(.medium)
                            }
                            .foregroundColor(uvcCameraManager.stereoEnabled ? .white : .white.opacity(0.6))
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 12)
                            .background(uvcCameraManager.stereoEnabled ? Color.cyan.opacity(0.4) : Color.white.opacity(0.1))
                            .cornerRadius(10)
                        }
                        .buttonStyle(.plain)
                    }
                    
                    Text("Stereo: Side-by-side left/right feed")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.4))
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
        VStack(alignment: .leading, spacing: 16) {
            if recordingManager.isRecording {
                // Active recording display
                VStack(alignment: .leading, spacing: 12) {
                    // Recording indicator header
                    HStack(spacing: 10) {
                        Circle()
                            .fill(Color.red)
                            .frame(width: 10, height: 10)
                            .shadow(color: .red.opacity(0.6), radius: 4)
                        
                        Text("Recording")
                            .font(.subheadline)
                            .fontWeight(.bold)
                            .foregroundColor(.red)
                        
                        Spacer()
                        
                        if recordingManager.isAutoRecording {
                            HStack(spacing: 4) {
                                Image(systemName: "sparkles")
                                    .font(.system(size: 10))
                                Text("Auto")
                                    .font(.caption2)
                            }
                            .foregroundColor(.orange)
                            .padding(.horizontal, 8)
                            .padding(.vertical, 3)
                            .background(Color.orange.opacity(0.2))
                            .cornerRadius(6)
                        }
                    }
                    
                    // Stats row
                    HStack(spacing: 16) {
                        VStack(alignment: .leading, spacing: 2) {
                            Text("Duration")
                                .font(.caption2)
                                .foregroundColor(.white.opacity(0.5))
                            Text(recordingManager.formatDuration(recordingManager.recordingDuration))
                                .font(.title3)
                                .fontWeight(.bold)
                                .foregroundColor(.white)
                                .monospacedDigit()
                        }
                        
                        Divider()
                            .frame(height: 36)
                            .background(Color.white.opacity(0.2))
                        
                        VStack(alignment: .leading, spacing: 2) {
                            Text("Frames")
                                .font(.caption2)
                                .foregroundColor(.white.opacity(0.5))
                            Text("\(recordingManager.frameCount)")
                                .font(.title3)
                                .fontWeight(.bold)
                                .foregroundColor(.white)
                                .monospacedDigit()
                        }
                        
                        Spacer()
                    }
                    .padding(12)
                    .background(Color.red.opacity(0.15))
                    .cornerRadius(10)
                    
                    // Stop button
                    Button { recordingManager.stopRecordingManually() } label: {
                        HStack(spacing: 8) {
                            Image(systemName: "stop.fill")
                                .font(.system(size: 14, weight: .bold))
                            Text("Stop Recording")
                                .font(.subheadline)
                                .fontWeight(.semibold)
                        }
                        .foregroundColor(.white)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(Color.red)
                        .cornerRadius(12)
                    }
                    .buttonStyle(.plain)
                }
            } else if recordingManager.isSaving {
                HStack(spacing: 12) {
                    ProgressView()
                        .progressViewStyle(CircularProgressViewStyle(tint: .white))
                        .scaleEffect(0.8)
                    VStack(alignment: .leading, spacing: 2) {
                        Text("Saving Recording")
                            .font(.subheadline)
                            .fontWeight(.medium)
                            .foregroundColor(.white)
                        Text("Please wait...")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.5))
                    }
                    Spacer()
                }
                .padding(14)
                .background(Color.white.opacity(0.1))
                .cornerRadius(12)
            } else if recordingManager.isUploadingToCloud {
                // Cloud upload progress display
                VStack(alignment: .leading, spacing: 12) {
                    HStack(spacing: 10) {
                        Image(systemName: "icloud.and.arrow.up")
                            .font(.system(size: 18))
                            .foregroundColor(recordingManager.cloudProvider.color)
                        VStack(alignment: .leading, spacing: 2) {
                            Text("Uploading to \(recordingManager.cloudProvider.displayName)")
                                .font(.subheadline)
                                .fontWeight(.semibold)
                                .foregroundColor(.white)
                            Text("Keep the app open until complete")
                                .font(.caption2)
                                .foregroundColor(.white.opacity(0.5))
                        }
                        Spacer()
                    }
                    
                    // Progress bar
                    let progress = recordingManager.cloudUploadTotalFiles > 0 
                        ? Double(recordingManager.cloudUploadCurrentFile) / Double(recordingManager.cloudUploadTotalFiles) 
                        : 0.0
                    
                    VStack(spacing: 8) {
                        GeometryReader { geometry in
                            ZStack(alignment: .leading) {
                                RoundedRectangle(cornerRadius: 5)
                                    .fill(Color.white.opacity(0.15))
                                    .frame(height: 8)
                                
                                RoundedRectangle(cornerRadius: 5)
                                    .fill(recordingManager.cloudProvider.color)
                                    .frame(width: geometry.size.width * progress, height: 8)
                                    .animation(.easeInOut(duration: 0.3), value: progress)
                            }
                        }
                        .frame(height: 8)
                        
                        HStack {
                            Text("\(recordingManager.cloudUploadCurrentFile) of \(recordingManager.cloudUploadTotalFiles) files")
                                .font(.caption)
                                .foregroundColor(.white.opacity(0.6))
                            Spacer()
                            Text("\(Int(progress * 100))%")
                                .font(.caption)
                                .fontWeight(.bold)
                                .foregroundColor(recordingManager.cloudProvider.color)
                        }
                    }
                }
                .padding(14)
                .background(recordingManager.cloudProvider.color.opacity(0.12))
                .cornerRadius(12)
            } else {
                // Recording configuration
                VStack(alignment: .leading, spacing: 16) {
                    // Auto-recording toggle with description
                    VStack(alignment: .leading, spacing: 10) {
                        Toggle(isOn: $recordingManager.autoRecordingEnabled) {
                            HStack(spacing: 10) {
                                ZStack {
                                    Circle()
                                        .fill(recordingManager.autoRecordingEnabled ? Color.orange.opacity(0.2) : Color.white.opacity(0.1))
                                        .frame(width: 36, height: 36)
                                    Image(systemName: recordingManager.autoRecordingEnabled ? "record.circle.fill" : "record.circle")
                                        .font(.system(size: 18, weight: .medium))
                                        .foregroundColor(recordingManager.autoRecordingEnabled ? .orange : .white.opacity(0.5))
                                }
                                
                                VStack(alignment: .leading, spacing: 3) {
                                    Text("Auto-Record")
                                        .font(.subheadline)
                                        .fontWeight(.semibold)
                                        .foregroundColor(.white)
                                    Text("Automatically start when video frames arrive")
                                        .font(.caption2)
                                        .foregroundColor(.white.opacity(0.5))
                                        .lineLimit(2)
                                }
                            }
                        }
                        .toggleStyle(SwitchToggleStyle(tint: .orange))
                    }
                    .padding(12)
                    .background(recordingManager.autoRecordingEnabled ? Color.orange.opacity(0.1) : Color.white.opacity(0.05))
                    .cornerRadius(12)
                    
                    // Storage Location Section
                    VStack(alignment: .leading, spacing: 12) {
                        HStack {
                            Image(systemName: "externaldrive.fill")
                                .font(.system(size: 14))
                                .foregroundColor(.white.opacity(0.6))
                            Text("Storage Location")
                                .font(.caption)
                                .fontWeight(.semibold)
                                .foregroundColor(.white.opacity(0.6))
                                .textCase(.uppercase)
                                .tracking(0.5)
                        }
                        
                        // Storage options
                        VStack(spacing: 8) {
                            storageOptionRow(
                                icon: "internaldrive",
                                label: "Local Storage",
                                description: "Save to device, transfer via Files app",
                                isSelected: recordingManager.storageLocation == .local,
                                color: .green
                            ) {
                                recordingManager.storageLocation = .local
                            }
                            
                            storageOptionRow(
                                icon: "icloud.fill",
                                label: "iCloud Drive",
                                description: "Sync across your Apple devices",
                                isSelected: recordingManager.storageLocation == .cloud && recordingManager.cloudProvider == .iCloudDrive,
                                color: .blue
                            ) {
                                recordingManager.storageLocation = .cloud
                                recordingManager.cloudProvider = .iCloudDrive
                                KeychainManager.shared.save(CloudStorageProvider.iCloudDrive.rawValue, forKey: .selectedCloudProvider)
                            }
                            
                            storageOptionRow(
                                icon: "g.circle.fill",
                                label: "Google Drive",
                                description: cloudStorageSettings.isGoogleDriveAvailable ? "Upload to your Google account" : "Sign in required",
                                isSelected: recordingManager.storageLocation == .cloud && recordingManager.cloudProvider == .googleDrive,
                                color: Color(red: 0.26, green: 0.52, blue: 0.96),
                                showWarning: !cloudStorageSettings.isGoogleDriveAvailable
                            ) {
                                recordingManager.storageLocation = .cloud
                                recordingManager.cloudProvider = .googleDrive
                                KeychainManager.shared.save(CloudStorageProvider.googleDrive.rawValue, forKey: .selectedCloudProvider)
                            }
                            
                            storageOptionRow(
                                icon: "shippingbox.fill",
                                label: "Dropbox",
                                description: cloudStorageSettings.isDropboxAvailable ? "Upload to your Dropbox account" : "Sign in required",
                                isSelected: recordingManager.storageLocation == .cloud && recordingManager.cloudProvider == .dropbox,
                                color: Color(red: 0, green: 0.4, blue: 1),
                                showWarning: !cloudStorageSettings.isDropboxAvailable
                            ) {
                                recordingManager.storageLocation = .cloud
                                recordingManager.cloudProvider = .dropbox
                                KeychainManager.shared.save(CloudStorageProvider.dropbox.rawValue, forKey: .selectedCloudProvider)
                            }
                        }
                    }
                    
                    // Sign-in prompt for Google Drive
                    if recordingManager.storageLocation == .cloud && recordingManager.cloudProvider == .googleDrive && !cloudStorageSettings.isGoogleDriveAvailable {
                        Button {
                            expandedPanel = .none
                            isMinimized = true
                            userInteracted = true
                            Task {
                                try? await Task.sleep(nanoseconds: 500_000_000)
                                await GoogleDriveAuthManager.shared.startOAuthFlow()
                                await MainActor.run { isMinimized = false }
                            }
                        } label: {
                            HStack(spacing: 8) {
                                Image(systemName: "person.crop.circle.badge.plus")
                                    .font(.system(size: 14))
                                Text("Sign in to Google Drive")
                                    .font(.subheadline)
                                    .fontWeight(.medium)
                            }
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 12)
                            .background(Color(red: 0.26, green: 0.52, blue: 0.96))
                            .cornerRadius(10)
                        }
                        .buttonStyle(.plain)
                    }
                }
            }
        }
    }
    
    // Helper for storage option rows with descriptions
    private func storageOptionRow(icon: String, label: String, description: String, isSelected: Bool, color: Color, showWarning: Bool = false, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            HStack(spacing: 12) {
                ZStack {
                    Circle()
                        .fill(isSelected ? color.opacity(0.2) : Color.white.opacity(0.08))
                        .frame(width: 40, height: 40)
                    Image(systemName: icon)
                        .font(.system(size: 18, weight: .medium))
                        .foregroundColor(isSelected ? color : .white.opacity(0.5))
                }
                
                VStack(alignment: .leading, spacing: 3) {
                    HStack(spacing: 6) {
                        Text(label)
                            .font(.subheadline)
                            .fontWeight(.medium)
                            .foregroundColor(isSelected ? .white : .white.opacity(0.8))
                        if showWarning {
                            Image(systemName: "exclamationmark.circle.fill")
                                .font(.system(size: 11))
                                .foregroundColor(.orange)
                        }
                    }
                    Text(description)
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.45))
                        .fixedSize(horizontal: false, vertical: true)
                }
                
                Spacer()
                
                if isSelected {
                    Image(systemName: "checkmark.circle.fill")
                        .font(.system(size: 20))
                        .foregroundColor(color)
                }
            }
            .padding(.horizontal, 12)
            .padding(.vertical, 10)
            .background(isSelected ? color.opacity(0.12) : Color.white.opacity(0.04))
            .cornerRadius(12)
            .overlay(
                RoundedRectangle(cornerRadius: 12)
                    .stroke(isSelected ? color.opacity(0.4) : Color.clear, lineWidth: 1)
            )
        }
        .buttonStyle(.plain)
    }
    
    // Legacy helper for backwards compatibility
    private func storageOptionButton(icon: String, label: String, isSelected: Bool, color: Color, showWarning: Bool = false, action: @escaping () -> Void) -> some View {
        storageOptionRow(icon: icon, label: label, description: "", isSelected: isSelected, color: color, showWarning: showWarning, action: action)
    }
    
    private var cloudStorageDebugPanelContent: some View {
        VStack(alignment: .leading, spacing: 10) {
            // Header
            Text("iCloud Keychain Sync Debug")
                .font(.caption)
                .fontWeight(.semibold)
                .foregroundColor(.white)
            
            Divider()
                .background(Color.white.opacity(0.2))
            
            // Current settings
            VStack(alignment: .leading, spacing: 6) {
                DebugInfoRow(label: "Selected Provider", value: recordingManager.cloudProvider.displayName)
                DebugInfoRow(label: "Is Dropbox Available", value: CloudStorageSettings.shared.isDropboxAvailable ? "Yes" : "No")
                if let lastSync = CloudStorageSettings.shared.lastSyncTime {
                    DebugInfoRow(label: "Last Sync", value: lastSync.formatted(date: .omitted, time: .shortened))
                }
            }
            
            Divider()
                .background(Color.white.opacity(0.2))
            
            // Raw keychain values
            Text("Raw Keychain Values:")
                .font(.caption)
                .fontWeight(.medium)
                .foregroundColor(.white.opacity(0.8))
            
            VStack(alignment: .leading, spacing: 6) {
                let keychain = KeychainManager.shared
                DebugInfoRow(label: "Provider Key", value: keychain.loadString(forKey: .selectedCloudProvider) ?? "nil")
                DebugInfoRow(label: "Access Token", value: keychain.exists(key: .dropboxAccessToken) ? "✓ Present" : "✗ Missing")
                DebugInfoRow(label: "Refresh Token", value: keychain.exists(key: .dropboxRefreshToken) ? "✓ Present" : "✗ Missing")
            }
            
            Divider()
                .background(Color.white.opacity(0.2))
            
            // Refresh button
            Button {
                CloudStorageSettings.shared.forceRefresh()
                recordingManager.loadCloudSettings()
            } label: {
                HStack {
                    Image(systemName: "arrow.clockwise")
                    Text("Force Refresh from Keychain")
                }
                .font(.caption)
                .foregroundColor(.white)
                .frame(maxWidth: .infinity)
                .padding(.vertical, 8)
                .background(Color.blue.opacity(0.3))
                .cornerRadius(8)
            }
            .buttonStyle(.plain)
            
            // Help text
            Text("Note: iCloud Keychain sync can take up to a few minutes. Ensure both devices use the same Apple ID with iCloud Keychain enabled.")
                .font(.caption2)
                .foregroundColor(.white.opacity(0.5))
                .fixedSize(horizontal: false, vertical: true)
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
    
    // MARK: - Position & Layout Panel (Combined)
    
    private var positionLayoutPanelContent: some View {
        VStack(alignment: .leading, spacing: 16) {
            // Section 1: Video Plane Position
            VStack(alignment: .leading, spacing: 10) {
                // Section header
                HStack(spacing: 8) {
                    Image(systemName: "rectangle.on.rectangle")
                        .font(.system(size: 14, weight: .medium))
                        .foregroundColor(.blue)
                    Text("Video Plane")
                        .font(.subheadline)
                        .fontWeight(.semibold)
                        .foregroundColor(.white)
                }
                
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
                .padding(12)
                .background(Color.blue.opacity(0.1))
                .cornerRadius(10)
                
                // Section 2: Controller Position
                VStack(alignment: .leading, spacing: 10) {
                    // Section header
                    HStack(spacing: 8) {
                        Image(systemName: "move.3d")
                            .font(.system(size: 14, weight: .medium))
                            .foregroundColor(.purple)
                        Text("Controller")
                            .font(.subheadline)
                            .fontWeight(.semibold)
                            .foregroundColor(.white)
                    }
                    
                    Text("Adjust where the minimized control buttons appear in your view.")
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.5))
                    
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
                .padding(12)
                .background(Color.purple.opacity(0.1))
                .cornerRadius(10)
            }
    }
    
    // MARK: - Visualizations Panel
    
    private var visualizationsPanelContent: some View {
        VStack(alignment: .leading, spacing: 12) {
            // Description
            Text("Control what visual elements are rendered in the immersive space.")
                .font(.caption)
                .foregroundColor(.white.opacity(0.6))
                .fixedSize(horizontal: false, vertical: true)
            
            Divider()
                .background(Color.white.opacity(0.2))
            
            // Show Hands toggle
            visualizationToggleRow(
                icon: dataManager.upperLimbVisible ? "hand.raised.fill" : "hand.raised.slash.fill",
                title: "Hands Over AR",
                description: "When enabled, your real hands appear on top of all AR content (video, 3D models). When disabled, hands are hidden behind AR objects.",
                isOn: $dataManager.upperLimbVisible,
                accentColor: .cyan
            )
            
            Divider()
                .background(Color.white.opacity(0.15))
            
            // Head Beam toggle
            visualizationToggleRow(
                icon: dataManager.showHeadBeam ? "rays" : "circle.dashed",
                title: "Head Gaze Ray",
                description: "Projects a ray from your head showing where you're looking. Helpful for debugging head tracking or aiming at objects.",
                isOn: $dataManager.showHeadBeam,
                accentColor: .yellow
            )
            
            Divider()
                .background(Color.white.opacity(0.15))
            
            // Hand Joints toggle
            visualizationToggleRow(
                icon: dataManager.showHandJoints ? "circle.grid.3x3.fill" : "circle.grid.3x3",
                title: "Hand Tracking",
                description: "Shows small spheres at each of the 27 tracked hand joints. Useful for debugging finger tracking accuracy.",
                isOn: $dataManager.showHandJoints,
                accentColor: .orange
            )
            
            // Hand Joints Opacity slider (always shown, disabled when hand tracking is off)
            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Image(systemName: "circle.lefthalf.filled")
                        .font(.system(size: 16, weight: .medium))
                        .foregroundColor(dataManager.showHandJoints ? .orange.opacity(0.7) : .white.opacity(0.3))
                        .frame(width: 28)
                    
                    Text("Skeleton Opacity")
                        .font(.subheadline)
                        .foregroundColor(dataManager.showHandJoints ? .white.opacity(0.9) : .white.opacity(0.4))
                    
                    Spacer()
                    
                    Text("\(Int(dataManager.handJointsOpacity * 100))%")
                        .font(.subheadline)
                        .fontWeight(.medium)
                        .foregroundColor(dataManager.showHandJoints ? .orange : .white.opacity(0.3))
                        .frame(width: 50, alignment: .trailing)
                }
                
                Slider(
                    value: Binding(
                        get: { Double(dataManager.handJointsOpacity) },
                        set: { dataManager.handJointsOpacity = Float($0) }
                    ),
                    in: 0.1...1.0,
                    step: 0.05
                )
                .tint(dataManager.showHandJoints ? .orange : .gray)
                .disabled(!dataManager.showHandJoints)
                .frame(height: 30)
                .padding(.leading, 36)
            }
            .padding(.top, 8)
            .opacity(dataManager.showHandJoints ? 1.0 : 0.5)
        }
    }
    
    private func visualizationToggleRow(icon: String, title: String, description: String, isOn: Binding<Bool>, accentColor: Color) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Image(systemName: icon)
                    .font(.system(size: 18, weight: .medium))
                    .foregroundColor(isOn.wrappedValue ? accentColor : .white.opacity(0.4))
                    .frame(width: 28)
                
                Text(title)
                    .font(.subheadline)
                    .fontWeight(.semibold)
                    .foregroundColor(.white)
                
                Spacer()
                
                Toggle("", isOn: isOn)
                    .labelsHidden()
                    .tint(accentColor)
            }
            
            Text(description)
                .font(.caption2)
                .foregroundColor(.white.opacity(0.5))
                .fixedSize(horizontal: false, vertical: true)
                .padding(.leading, 36)
        }
        .padding(.vertical, 4)
    }
    
    // MARK: - Persona Capture Panel
    
    private var personaCapturePanelContent: some View {
        VStack(alignment: .leading, spacing: 12) {
            // Description
            Text("Preview your Spatial Persona using the front-facing camera.")
                .font(.caption)
                .foregroundColor(.white.opacity(0.6))
                .fixedSize(horizontal: false, vertical: true)
            
            Divider()
                .background(Color.white.opacity(0.2))
            
            // Embed the PersonaPreviewView
            PersonaPreviewView()
        }
    }
    
    // MARK: - Unified Camera Calibration Panel
    
    @State private var showCalibrationWizard: Bool = false
    
    @ViewBuilder
    private var cameraCalibrationMenuItem: some View {
        let hasIntrinsic = uvcCameraManager.selectedDevice.map { calibrationManager.hasCalibration(for: $0.id) } ?? false
        let hasExtrinsic = uvcCameraManager.selectedDevice.map { extrinsicCalibrationManager.hasCalibration(for: $0.id) } ?? false
        let calibrationStatus: String = {
            if hasIntrinsic && hasExtrinsic {
                return "Fully Calibrated"
            } else if hasIntrinsic || hasExtrinsic {
                return "Partially Calibrated"
            } else {
                return "Not Calibrated"
            }
        }()
        let statusColor: Color = (hasIntrinsic && hasExtrinsic) ? .green : .orange
        
        menuItem(
            icon: "camera.aperture",
            title: "Camera Calibration",
            subtitle: calibrationStatus,
            isExpanded: expandedPanel == .cameraCalibration,
            accentColor: .cyan,
            iconColor: statusColor
        ) {
            withAnimation(.spring(response: 0.35, dampingFraction: 0.85)) {
                expandedPanel = expandedPanel == .cameraCalibration ? .none : .cameraCalibration
            }
        }
    }
    
    private var cameraCalibrationPanelContent: some View {
        VStack(alignment: .leading, spacing: 12) {
            if let device = uvcCameraManager.selectedDevice {
                let hasIntrinsic = calibrationManager.hasCalibration(for: device.id)
                let hasExtrinsic = extrinsicCalibrationManager.hasCalibration(for: device.id)
                
                // Camera name and overall status
                VStack(alignment: .leading, spacing: 8) {
                    Text(device.name)
                        .font(.subheadline)
                        .fontWeight(.semibold)
                        .foregroundColor(.white)
                    
                    // Status badges
                    HStack(spacing: 8) {
                        calibrationStatusBadge(
                            title: "Intrinsic",
                            isCalibrated: hasIntrinsic,
                            color: .cyan
                        )
                        calibrationStatusBadge(
                            title: "Extrinsic",
                            isCalibrated: hasExtrinsic,
                            color: .purple
                        )
                    }
                }
                .padding(12)
                .frame(maxWidth: .infinity, alignment: .leading)
                .background(
                    LinearGradient(
                        colors: [Color.white.opacity(0.1), Color.white.opacity(0.05)],
                        startPoint: .topLeading,
                        endPoint: .bottomTrailing
                    )
                )
                .cornerRadius(10)
                
                // Calibration Results (if calibrated)
                if hasIntrinsic || hasExtrinsic {
                    VStack(alignment: .leading, spacing: 10) {
                        // Intrinsic results
                        if hasIntrinsic, let intrinsic = calibrationManager.allCalibrations[device.id] {
                            calibrationResultRow(
                                icon: "camera.aperture",
                                title: "Intrinsic",
                                subtitle: "fx=\(Int(intrinsic.leftIntrinsics.fx)), fy=\(Int(intrinsic.leftIntrinsics.fy))",
                                detail: String(format: "%.4f px", intrinsic.leftIntrinsics.reprojectionError),
                                isGood: intrinsic.leftIntrinsics.reprojectionError < 0.5,
                                color: .cyan
                            )
                        }
                        
                        // Extrinsic results
                        if hasExtrinsic, let extrinsic = extrinsicCalibrationManager.allCalibrations[device.id] {
                            let translation = SIMD3<Float>(
                                extrinsic.leftHeadToCameraMatrix.columns.3.x,
                                extrinsic.leftHeadToCameraMatrix.columns.3.y,
                                extrinsic.leftHeadToCameraMatrix.columns.3.z
                            )
                            let distance = simd_length(translation) * 100 // cm
                            
                            calibrationResultRow(
                                icon: "arrow.triangle.swap",
                                title: "Extrinsic",
                                subtitle: String(format: "%.1f cm from head", distance),
                                detail: String(format: "%.4f m", extrinsic.leftReprojectionError),
                                isGood: extrinsic.leftReprojectionError < 0.01,
                                color: .purple
                            )
                        }
                    }
                    .padding(12)
                    .background(Color.white.opacity(0.05))
                    .cornerRadius(10)
                }
                
                // Info text when not calibrated
                if !hasIntrinsic && !hasExtrinsic {
                    VStack(alignment: .leading, spacing: 6) {
                        Label("Why Calibrate?", systemImage: "info.circle.fill")
                            .font(.caption)
                            .fontWeight(.medium)
                            .foregroundColor(.white.opacity(0.8))
                        
                        Text("Camera calibration enables accurate pose estimation and is required for teleoperation. The wizard will guide you through both intrinsic and extrinsic calibration.")
                            .font(.caption2)
                            .foregroundColor(.white.opacity(0.5))
                            .lineSpacing(2)
                    }
                    .padding(12)
                    .background(Color.blue.opacity(0.1))
                    .cornerRadius(10)
                }
                
                // Big calibrate button
                Button {
                    showCalibrationWizard = true
                } label: {
                    HStack(spacing: 8) {
                        Image(systemName: hasIntrinsic && hasExtrinsic ? "arrow.clockwise" : "camera.aperture")
                            .font(.system(size: 16, weight: .bold))
                        Text(hasIntrinsic && hasExtrinsic ? "Recalibrate" : "Calibrate Camera")
                            .font(.headline)
                    }
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 14)
                    .background(
                        LinearGradient(
                            colors: [Color.cyan, Color.purple],
                            startPoint: .leading,
                            endPoint: .trailing
                        )
                    )
                    .cornerRadius(12)
                }
                .buttonStyle(.plain)
                
                // Delete calibration button (if calibrated)
                if hasIntrinsic || hasExtrinsic {
                    Button {
                        if hasIntrinsic {
                            calibrationManager.deleteCalibration(for: device.id)
                        }
                        if hasExtrinsic {
                            extrinsicCalibrationManager.deleteCalibration(for: device.id)
                        }
                    } label: {
                        HStack(spacing: 4) {
                            Image(systemName: "trash")
                                .font(.caption)
                            Text("Delete All Calibration Data")
                                .font(.caption)
                        }
                        .foregroundColor(.red.opacity(0.8))
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 8)
                        .background(Color.red.opacity(0.1))
                        .cornerRadius(8)
                    }
                    .buttonStyle(.plain)
                }
                
            } else {
                // No camera selected
                VStack(spacing: 12) {
                    Image(systemName: "video.slash")
                        .font(.system(size: 32))
                        .foregroundColor(.white.opacity(0.4))
                    Text("No camera selected")
                        .font(.subheadline)
                        .foregroundColor(.white.opacity(0.5))
                    Text("Connect a USB camera to get started")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.3))
                }
                .frame(maxWidth: .infinity)
                .padding(.vertical, 20)
            }
        }
    }
    
    private func calibrationStatusBadge(title: String, isCalibrated: Bool, color: Color) -> some View {
        HStack(spacing: 4) {
            Image(systemName: isCalibrated ? "checkmark.circle.fill" : "circle.dashed")
                .font(.system(size: 10))
            Text(title)
                .font(.caption2)
                .fontWeight(.medium)
        }
        .foregroundColor(isCalibrated ? color : .white.opacity(0.5))
        .padding(.horizontal, 8)
        .padding(.vertical, 4)
        .background(isCalibrated ? color.opacity(0.2) : Color.white.opacity(0.1))
        .cornerRadius(6)
    }
    
    private func calibrationResultRow(icon: String, title: String, subtitle: String, detail: String, isGood: Bool, color: Color) -> some View {
        HStack(spacing: 10) {
            Image(systemName: icon)
                .font(.system(size: 14))
                .foregroundColor(color)
                .frame(width: 24)
            
            VStack(alignment: .leading, spacing: 2) {
                Text(title)
                    .font(.caption)
                    .fontWeight(.medium)
                    .foregroundColor(.white)
                Text(subtitle)
                    .font(.caption2)
                    .foregroundColor(.white.opacity(0.6))
            }
            
            Spacer()
            
            HStack(spacing: 4) {
                Image(systemName: isGood ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
                    .font(.system(size: 10))
                Text(detail)
                    .font(.caption2)
                    .fontWeight(.medium)
            }
            .foregroundColor(isGood ? .green : .orange)
        }
    }
    
    // Keep the old views for backward compatibility (will be removed later)
    private var calibrationPanelContent: some View {
        cameraCalibrationPanelContent
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
                        
                        // Display 4x4 matrix
                        let t = calibration.leftHeadToCameraMatrix
                        Matrix4x4View(matrix: t, color: .purple)
                        
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
                            
                            Matrix4x4View(matrix: rightMatrix, color: .cyan)
                            
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
                        
                        // 3D Visualization of camera pose
                        Divider()
                            .background(Color.white.opacity(0.2))
                            .padding(.vertical, 4)
                        
                        Text("3D View (Head Frame)")
                            .font(.caption2)
                            .fontWeight(.medium)
                            .foregroundColor(.white.opacity(0.7))
                        
                        ExtrinsicCalibration3DView(calibration: calibration)
                            .frame(height: 150)
                            .background(Color.black.opacity(0.3))
                            .cornerRadius(8)
                        
                        // Copy to clipboard button
                        Button(action: {
                            UIPasteboard.general.string = calibration.exportAsJSON()
                        }) {
                            HStack(spacing: 4) {
                                Image(systemName: "doc.on.doc")
                                    .font(.caption2)
                                Text("Copy JSON to Clipboard")
                                    .font(.caption2)
                            }
                            .foregroundColor(.blue)
                            .padding(.horizontal, 8)
                            .padding(.vertical, 4)
                            .background(Color.blue.opacity(0.15))
                            .cornerRadius(6)
                        }
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

// MARK: - 4x4 Matrix View

/// A view that displays a 4x4 matrix in a compact grid format
struct Matrix4x4View: View {
    let matrix: simd_float4x4
    let color: Color
    
    var body: some View {
        VStack(alignment: .leading, spacing: 1) {
            ForEach(0..<4, id: \.self) { row in
                HStack(spacing: 2) {
                    ForEach(0..<4, id: \.self) { col in
                        // matrix is column-major, so access as matrix.columns[col][row]
                        let value = matrix[col][row]
                        Text(formatValue(value))
                            .font(.system(size: 7, design: .monospaced))
                            .foregroundColor(color)
                            .frame(width: 42, alignment: .trailing)
                    }
                }
            }
        }
        .padding(4)
        .background(Color.black.opacity(0.2))
        .cornerRadius(4)
    }
    
    private func formatValue(_ value: Float) -> String {
        if abs(value) < 0.0001 {
            return "0.000"
        } else if abs(value) >= 100 {
            return String(format: "%.1f", value)
        } else if abs(value) >= 10 {
            return String(format: "%.2f", value)
        } else {
            return String(format: "%.3f", value)
        }
    }
}

// MARK: - 3D Visualization for Extrinsic Calibration

/// A 2D canvas that draws a 3D visualization of the camera pose relative to the head frame
struct ExtrinsicCalibration3DView: View {
    let calibration: ExtrinsicCalibrationData
    
    // View rotation state (radians)
    @State private var viewAngleX: Float = -0.4  // tilt down
    @State private var viewAngleY: Float = 0.3   // rotate right
    @State private var lastDragLocation: CGPoint? = nil
    
    var body: some View {
        Canvas { context, size in
            let center = CGPoint(x: size.width / 2, y: size.height / 2)
            let scale: CGFloat = 300  // pixels per meter
            
            // Draw grid on XZ plane (draw first so it's behind)
            drawGrid(context: context, center: center, scale: scale,
                    viewAngleX: viewAngleX, viewAngleY: viewAngleY)
            
            // Draw head frame at origin (coordinate axes)
            drawCoordinateAxes(context: context, center: center, scale: scale,
                             transform: matrix_identity_float4x4,
                             viewAngleX: viewAngleX, viewAngleY: viewAngleY,
                             label: "Head", axisLength: 0.05)
            
            // Draw left/mono camera
            // leftHeadToCameraMatrix is T^camera_head (transforms points from head frame to camera frame)
            // To get camera pose in head frame, we need the inverse
            // Camera axes are in OpenCV convention: X-right, Y-down, Z-forward (optical axis)
            let leftHeadToCamera = calibration.leftHeadToCameraMatrix
            let leftCameraInHead = simd_inverse(leftHeadToCamera)
            drawCoordinateAxes(context: context, center: center, scale: scale,
                             transform: leftCameraInHead,
                             viewAngleX: viewAngleX, viewAngleY: viewAngleY,
                             label: calibration.isStereo ? "L" : "Cam", axisLength: 0.03)
            
            // Draw camera frustum for left camera
            drawCameraFrustum(context: context, center: center, scale: scale,
                            transform: leftCameraInHead,
                            viewAngleX: viewAngleX, viewAngleY: viewAngleY,
                            color: .purple.opacity(0.5))
            
            // Draw right camera if stereo
            if calibration.isStereo, let rightHeadToCamera = calibration.rightHeadToCameraMatrix {
                let rightCameraInHead = simd_inverse(rightHeadToCamera)
                drawCoordinateAxes(context: context, center: center, scale: scale,
                                 transform: rightCameraInHead,
                                 viewAngleX: viewAngleX, viewAngleY: viewAngleY,
                                 label: "R", axisLength: 0.03)
                
                drawCameraFrustum(context: context, center: center, scale: scale,
                                transform: rightCameraInHead,
                                viewAngleX: viewAngleX, viewAngleY: viewAngleY,
                                color: .cyan.opacity(0.5))
            }
            
            // Draw rotation hint
            let hintText = "Drag to rotate"
            context.draw(Text(hintText).font(.system(size: 7)).foregroundColor(.white.opacity(0.3)),
                        at: CGPoint(x: size.width - 35, y: size.height - 8), anchor: .center)
        }
        .gesture(
            DragGesture()
                .onChanged { value in
                    if let last = lastDragLocation {
                        let deltaX = Float(value.location.x - last.x) * 0.01
                        let deltaY = Float(value.location.y - last.y) * 0.01
                        viewAngleY += deltaX
                        viewAngleX -= deltaY
                        // Clamp vertical angle to avoid flipping
                        viewAngleX = max(-Float.pi / 2 + 0.1, min(Float.pi / 2 - 0.1, viewAngleX))
                    }
                    lastDragLocation = value.location
                }
                .onEnded { _ in
                    lastDragLocation = nil
                }
        )
    }
    
    /// Project a 3D point to 2D screen coordinates
    /// 
    /// Input coordinate system (ARKit head frame):
    /// - X: right (wearer's right)
    /// - Y: up  
    /// - Z: backward (toward back of head)
    /// 
    /// Display coordinate system (standard graphics):
    /// - X: right on screen
    /// - Y: up on screen (but screen Y increases downward, handled in final projection)
    /// - Z: into screen (away from viewer)
    ///
    /// To convert: we negate Z so that "backward" (ARKit +Z) becomes "into screen" (display +Z)
    private func project3DTo2D(point: SIMD3<Float>, center: CGPoint, scale: CGFloat,
                               viewAngleX: Float, viewAngleY: Float) -> CGPoint {
        // Convert from ARKit convention to display convention
        // ARKit: X-right, Y-up, Z-backward (right-handed)
        // Display: X-right, Y-up, Z-into-screen (right-handed)
        // The conversion is just negating Z
        let displayPoint = SIMD3<Float>(point.x, point.y, -point.z)
        
        // Apply view rotation (right-handed convention)
        let cosX = cos(viewAngleX)
        let sinX = sin(viewAngleX)
        let cosY = cos(viewAngleY)
        let sinY = sin(viewAngleY)
        
        // Rotate around Y axis (right-handed)
        let x1 = displayPoint.x * cosY - displayPoint.z * sinY
        let z1 = displayPoint.x * sinY + displayPoint.z * cosY
        let y1 = displayPoint.y
        
        // Then rotate around X axis (right-handed)
        let y2 = y1 * cosX - z1 * sinX
        let z2 = y1 * sinX + z1 * cosX
        let x2 = x1
        
        // Simple orthographic projection (ignore z2 for depth)
        let screenX = center.x + CGFloat(x2) * scale
        let screenY = center.y - CGFloat(y2) * scale  // Y is up in 3D, down in screen
        
        return CGPoint(x: screenX, y: screenY)
    }
    
    /// Draw coordinate axes at a given transform
    private func drawCoordinateAxes(context: GraphicsContext, center: CGPoint, scale: CGFloat,
                                    transform: simd_float4x4, viewAngleX: Float, viewAngleY: Float,
                                    label: String, axisLength: Float) {
        let origin = SIMD3<Float>(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
        
        // Extract rotation axes from transform
        let xAxis = SIMD3<Float>(transform.columns.0.x, transform.columns.0.y, transform.columns.0.z)
        let yAxis = SIMD3<Float>(transform.columns.1.x, transform.columns.1.y, transform.columns.1.z)
        let zAxis = SIMD3<Float>(transform.columns.2.x, transform.columns.2.y, transform.columns.2.z)
        
        let originScreen = project3DTo2D(point: origin, center: center, scale: scale,
                                         viewAngleX: viewAngleX, viewAngleY: viewAngleY)
        
        // Draw X axis (red)
        let xEnd = origin + xAxis * axisLength
        let xEndScreen = project3DTo2D(point: xEnd, center: center, scale: scale,
                                       viewAngleX: viewAngleX, viewAngleY: viewAngleY)
        var xPath = Path()
        xPath.move(to: originScreen)
        xPath.addLine(to: xEndScreen)
        context.stroke(xPath, with: .color(.red), lineWidth: 2)
        
        // Draw Y axis (green)
        let yEnd = origin + yAxis * axisLength
        let yEndScreen = project3DTo2D(point: yEnd, center: center, scale: scale,
                                       viewAngleX: viewAngleX, viewAngleY: viewAngleY)
        var yPath = Path()
        yPath.move(to: originScreen)
        yPath.addLine(to: yEndScreen)
        context.stroke(yPath, with: .color(.green), lineWidth: 2)
        
        // Draw Z axis (blue)
        let zEnd = origin + zAxis * axisLength
        let zEndScreen = project3DTo2D(point: zEnd, center: center, scale: scale,
                                       viewAngleX: viewAngleX, viewAngleY: viewAngleY)
        var zPath = Path()
        zPath.move(to: originScreen)
        zPath.addLine(to: zEndScreen)
        context.stroke(zPath, with: .color(.blue), lineWidth: 2)
        
        // Draw label
        let labelPoint = CGPoint(x: originScreen.x + 5, y: originScreen.y - 10)
        context.draw(Text(label).font(.system(size: 8, weight: .bold)).foregroundColor(.white),
                    at: labelPoint, anchor: .leading)
    }
    
    /// Draw a simple camera frustum
    /// The camera is in OpenCV convention where +Z is forward (optical axis)
    private func drawCameraFrustum(context: GraphicsContext, center: CGPoint, scale: CGFloat,
                                   transform: simd_float4x4, viewAngleX: Float, viewAngleY: Float,
                                   color: Color) {
        let origin = SIMD3<Float>(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
        let zAxis = SIMD3<Float>(transform.columns.2.x, transform.columns.2.y, transform.columns.2.z)
        let xAxis = SIMD3<Float>(transform.columns.0.x, transform.columns.0.y, transform.columns.0.z)
        let yAxis = SIMD3<Float>(transform.columns.1.x, transform.columns.1.y, transform.columns.1.z)
        
        let frustumDepth: Float = 0.04
        let frustumWidth: Float = 0.02
        let frustumHeight: Float = 0.015
        
        // In OpenCV camera convention, +Z is forward (optical axis)
        // So frustum points in +Z direction
        let zOffset = zAxis * frustumDepth
        let xOffset = xAxis * frustumWidth
        let yOffset = yAxis * frustumHeight
        
        let corner0 = origin + zOffset + xOffset + yOffset
        let corner1 = origin + zOffset - xOffset + yOffset
        let corner2 = origin + zOffset - xOffset - yOffset
        let corner3 = origin + zOffset + xOffset - yOffset
        let corners = [corner0, corner1, corner2, corner3]
        
        let originScreen = project3DTo2D(point: origin, center: center, scale: scale,
                                         viewAngleX: viewAngleX, viewAngleY: viewAngleY)
        let cornerScreens = corners.map { project3DTo2D(point: $0, center: center, scale: scale,
                                                        viewAngleX: viewAngleX, viewAngleY: viewAngleY) }
        
        // Draw frustum edges from origin to corners
        for cornerScreen in cornerScreens {
            var edgePath = Path()
            edgePath.move(to: originScreen)
            edgePath.addLine(to: cornerScreen)
            context.stroke(edgePath, with: .color(color), lineWidth: 1)
        }
        
        // Draw frustum rectangle
        var rectPath = Path()
        rectPath.move(to: cornerScreens[0])
        for i in 1..<cornerScreens.count {
            rectPath.addLine(to: cornerScreens[i])
        }
        rectPath.closeSubpath()
        context.stroke(rectPath, with: .color(color), lineWidth: 1)
    }
    
    /// Draw a reference grid on the XZ plane
    private func drawGrid(context: GraphicsContext, center: CGPoint, scale: CGFloat,
                         viewAngleX: Float, viewAngleY: Float) {
        let gridSize: Float = 0.1  // 10cm
        let gridLines = 3
        
        for i in -gridLines...gridLines {
            let offset = Float(i) * gridSize / Float(gridLines)
            
            // Lines parallel to X
            let xStart = project3DTo2D(point: SIMD3<Float>(-gridSize, 0, offset), center: center, scale: scale,
                                       viewAngleX: viewAngleX, viewAngleY: viewAngleY)
            let xEnd = project3DTo2D(point: SIMD3<Float>(gridSize, 0, offset), center: center, scale: scale,
                                     viewAngleX: viewAngleX, viewAngleY: viewAngleY)
            var xPath = Path()
            xPath.move(to: xStart)
            xPath.addLine(to: xEnd)
            context.stroke(xPath, with: .color(.white.opacity(0.1)), lineWidth: 0.5)
            
            // Lines parallel to Z
            let zStart = project3DTo2D(point: SIMD3<Float>(offset, 0, -gridSize), center: center, scale: scale,
                                       viewAngleX: viewAngleX, viewAngleY: viewAngleY)
            let zEnd = project3DTo2D(point: SIMD3<Float>(offset, 0, gridSize), center: center, scale: scale,
                                     viewAngleX: viewAngleX, viewAngleY: viewAngleY)
            var zPath = Path()
            zPath.move(to: zStart)
            zPath.addLine(to: zEnd)
            context.stroke(zPath, with: .color(.white.opacity(0.1)), lineWidth: 0.5)
        }
    }
}

// MARK: - Debug Info Row Helper

struct DebugInfoRow: View {
    let label: String
    let value: String
    
    var body: some View {
        HStack {
            Text(label)
                .font(.caption2)
                .foregroundColor(.white.opacity(0.6))
            Spacer()
            Text(value)
                .font(.caption2)
                .foregroundColor(.white)
                .fontWeight(.medium)
        }
    }
}

/// Creates a floating status entity that follows the head
func createStatusEntity() -> Entity {
    let statusEntity = Entity()
    statusEntity.name = "statusDisplay"
    return statusEntity
}
