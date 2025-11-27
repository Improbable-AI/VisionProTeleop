import SwiftUI
import RealityKit
import Network
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
    @State private var ipAddresses: [(name: String, address: String)] = []
    @State private var pythonConnected: Bool = false
    @State private var pythonIP: String = "Not connected"
    @State private var webrtcConnected: Bool = false
    @State private var hidePreviewTask: Task<Void, Never>?
    @State private var showStatusPositionControls: Bool = false
    @State private var showLocalExitConfirmation: Bool = false
    @State private var mujocoStatusUpdateTrigger: Bool = false  // Trigger for MuJoCo status updates
    
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
        return Group {
            if isMinimized {
                minimizedView
            } else {
                expandedView
            }
        }
        .animation(.spring(response: 0.45, dampingFraction: 0.85), value: isMinimized)
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
                // Title badge
                Text("Video")
                    .font(.caption2)
                    .fontWeight(.semibold)
                    .foregroundColor(dataManager.videoEnabled ? .blue : .gray)
                    .padding(.horizontal, 8)
                    .padding(.vertical, 3)
                    .background((dataManager.videoEnabled ? Color.blue : Color.gray).opacity(0.2))
                    .cornerRadius(8)
                
                if dataManager.videoEnabled {
                    // Status
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
            VStack(alignment: .leading, spacing: 15) {
                headerSection
                
                Divider()
                .background(Color.white.opacity(0.3))
            
                networkInfoSection
                
                // Show detailed track information when connected
                if showVideoStatus && webrtcConnected {
                    Divider()
                        .background(Color.white.opacity(0.2))
                    
                    streamDetailsSection
                }
            
            // Show waiting message when no frames are available (only for video mode)
            if showVideoStatus && !hasFrames {
                Divider()
                    .background(Color.white.opacity(0.3))
                
                HStack {
                    ProgressView()
                        .progressViewStyle(CircularProgressViewStyle(tint: .white))
                        .scaleEffect(1.0)
                    Text(dataManager.connectionStatus)
                        .foregroundColor(.white.opacity(0.9))
                        .font(.body)
                        .fontWeight(.medium)
                        .lineLimit(2)
                }
            }
            
            // View controls section (expandable)
            if showVideoStatus {
                if showViewControls {
                    Divider()
                        .background(Color.white.opacity(0.3))
                    
                    viewControlsSection
                } else {
                    // Show expand button when collapsed
                    Divider()
                        .background(Color.white.opacity(0.3))
                    
                    Button {
                        withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                            showViewControls = true
                        }
                    } label: {
                        HStack {
                            Image(systemName: "slider.horizontal.3")
                                .font(.system(size: 17, weight: .medium))
                            Text("Modify Video View")
                                .font(.body)
                                .fontWeight(.medium)
                            Spacer()
                            Image(systemName: "chevron.down")
                                .font(.system(size: 14, weight: .bold))
                        }
                        .foregroundColor(.white.opacity(0.9))
                        .padding(.vertical, 10)
                    }
                    .buttonStyle(.plain)
                }
            }
            
            // Status position controls section (collapsible, separate from video controls)
            Divider()
                .background(Color.white.opacity(0.3))
            
            if showStatusPositionControls {
                statusPositionControlsSection
            } else {
                // Show expand button when collapsed
                Button {
                    withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                        showStatusPositionControls = true
                    }
                } label: {
                    HStack {
                        Image(systemName: "move.3d")
                            .font(.system(size: 17, weight: .medium))
                        Text("Modify Controller Position")
                            .font(.body)
                            .fontWeight(.medium)
                        Spacer()
                        Image(systemName: "chevron.down")
                            .font(.system(size: 14, weight: .bold))
                    }
                    .foregroundColor(.white.opacity(0.9))
                    .padding(.vertical, 10)
                }
                .buttonStyle(.plain)
            }
        }
        .padding(24)
        .frame(width: 360)
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
                }
            }
        )
        .background(Color.black.opacity(0.7))
        .cornerRadius(16)
    }
    
    private var viewControlsSection: some View {
        VStack(alignment: .leading, spacing: 15) {
            Button {
                withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                    showViewControls = false
                }
                previewZDistance = nil
                previewActive = false
                hidePreviewTask?.cancel()
            } label: {
                HStack {
                    Image(systemName: "slider.horizontal.3")
                        .font(.system(size: 17, weight: .medium))
                    Text("Modify Video View")
                        .font(.body)
                        .fontWeight(.medium)
                    Spacer()
                    Image(systemName: "chevron.up")
                        .font(.system(size: 14, weight: .bold))
                }
                .foregroundColor(.white.opacity(0.9))
            }
            .buttonStyle(.plain)
            
            // Distance control
            VStack(alignment: .leading, spacing: 10) {
                Text("Distance (Z-axis)")
                    .font(.subheadline)
                    .foregroundColor(.white.opacity(0.7))
                
                HStack {
                    Text("\(String(format: "%.1f", -dataManager.videoPlaneZDistance))m")
                        .font(.system(size: 15, weight: .medium, design: .monospaced))
                        .foregroundColor(.white)
                        .frame(width: 50, alignment: .leading)
                     
                    Slider(value: Binding(
                        get: { -dataManager.videoPlaneZDistance },
                        set: { positiveValue in
                            let negativeValue = -positiveValue
                            dataManager.videoPlaneZDistance = negativeValue
                            previewZDistance = negativeValue
                            
                            // Cancel any existing hide task
                            hidePreviewTask?.cancel()
                            
                            // Schedule hiding the preview after 3 seconds of inactivity
                            hidePreviewTask = Task { @MainActor in
                                try? await Task.sleep(nanoseconds: 3_000_000_000)
                                if !Task.isCancelled {
                                    previewZDistance = nil
                                }
                            }
                        }
                    ), in: 2.0...20.0, step: 0.5)
                    .tint(.blue)
                }
                
                HStack {
                    Text("Near (2m)")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                    Spacer()
                    Text("Far (20m)")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                }
            }
            
            Divider()
                .background(Color.white.opacity(0.2))
            
            // Height control
            VStack(alignment: .leading, spacing: 10) {
                Text("Height (Y-axis)")
                    .font(.subheadline)
                    .foregroundColor(.white.opacity(0.7))
                
                HStack {
                    Text("\(String(format: "%.2f", dataManager.videoPlaneYPosition))m")
                        .font(.system(size: 15, weight: .medium, design: .monospaced))
                        .foregroundColor(.white)
                        .frame(width: 50, alignment: .leading)
                    
                    Slider(value: Binding(
                        get: { dataManager.videoPlaneYPosition },
                        set: { newValue in
                            dataManager.videoPlaneYPosition = newValue
                            // Show preview when adjusting
                            previewActive = true
                            
                            // Cancel any existing hide task
                            hidePreviewTask?.cancel()
                            
                            // Schedule hiding the preview after 3 seconds of inactivity
                            hidePreviewTask = Task { @MainActor in
                                try? await Task.sleep(nanoseconds: 3_000_000_000)
                                if !Task.isCancelled {
                                    previewActive = false
                                }
                            }
                        }
                    ), in: -2.0...2.0, step: 0.1)
                    .tint(.green)
                }
                
                HStack {
                    Text("Down (2m)")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                    Spacer()
                    Text("Up (2m)")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                }
            }
            
            Divider()
                .background(Color.white.opacity(0.2))
            
            HStack {
                VStack(alignment: .leading, spacing: 6) {
                    Text("Lock To World")
                        .font(.subheadline)
                        .foregroundColor(.white.opacity(0.7))
                    Text("Keep panel stationary")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                }
                Spacer()
                Toggle("", isOn: $videoFixed)
                    .labelsHidden()
                    .tint(.orange)
                    .scaleEffect(1.2)
            }
            
            HStack(spacing: 15) {
                Spacer()
                
                // Video minimize/maximize button
                Button {
                    withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                        videoMinimized.toggle()
                    }
                } label: {
                    HStack(spacing: 8) {
                        Image(systemName: videoMinimized ? "eye.fill" : "eye.slash.fill")
                            .font(.system(size: 14, weight: .bold))
                        Text(videoMinimized ? "Show Video" : "Hide Video")
                            .font(.subheadline)
                    }
                    .foregroundColor(.blue)
                    .padding(.horizontal, 14)
                    .padding(.vertical, 8)
                    .background(Color.white.opacity(0.15))
                    .cornerRadius(8)
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
                    Text("Reset All")
                        .font(.subheadline)
                        .foregroundColor(.blue)
                        .padding(.horizontal, 14)
                        .padding(.vertical, 8)
                        .background(Color.white.opacity(0.15))
                        .cornerRadius(8)
                }
                .buttonStyle(.plain)
                Spacer()
            }
            
            Text("💡 Adjust position and orientation of the video plane")
                .font(.caption)
                .foregroundColor(.white.opacity(0.5))
                .multilineTextAlignment(.leading)
        }
    }
    
    private var statusPositionControlsSection: some View {
        VStack(alignment: .leading, spacing: 15) {
            Button {
                withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                    showStatusPositionControls = false
                }
                previewStatusPosition = nil
                previewStatusActive = false
                hidePreviewTask?.cancel()
            } label: {
                HStack {
                    Image(systemName: "move.3d")
                        .font(.system(size: 17, weight: .medium))
                    Text("Modify Controller Position")
                        .font(.body)
                        .fontWeight(.medium)
                    Spacer()
                    Image(systemName: "chevron.up")
                        .font(.system(size: 14, weight: .bold))
                }
                .foregroundColor(.white.opacity(0.9))
            }
            .buttonStyle(.plain)
            
            // X position control
            VStack(alignment: .leading, spacing: 10) {
                Text("X Position (Left-Right)")
                    .font(.subheadline)
                    .foregroundColor(.white.opacity(0.7))
                
                HStack {
                    Text("\(String(format: "%.2f", dataManager.statusMinimizedXPosition))m")
                        .font(.system(size: 15, weight: .medium, design: .monospaced))
                        .foregroundColor(.white)
                        .frame(width: 50, alignment: .leading)
                    
                    Slider(value: Binding(
                        get: { dataManager.statusMinimizedXPosition },
                        set: { newValue in
                            print("🎚️ [StatusView] X slider changed to: \(newValue)")
                            dataManager.statusMinimizedXPosition = newValue
                            previewStatusPosition = (x: newValue, y: dataManager.statusMinimizedYPosition)
                            previewStatusActive = true
                            print("🎚️ [StatusView] Set previewStatusPosition to: \(String(describing: previewStatusPosition)), previewStatusActive: \(previewStatusActive)")
                            
                            // Cancel any existing hide task
                            hidePreviewTask?.cancel()
                            
                            // Schedule hiding the preview after 3 seconds of inactivity
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
                
                HStack {
                    Text("Left (0.5m)")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                    Spacer()
                    Text("Right (0.5m)")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                }
            }
            
            Divider()
                .background(Color.white.opacity(0.2))
            
            // Y position control
            VStack(alignment: .leading, spacing: 10) {
                Text("Y Position (Up-Down)")
                    .font(.subheadline)
                    .foregroundColor(.white.opacity(0.7))
                
                HStack {
                    Text("\(String(format: "%.2f", dataManager.statusMinimizedYPosition))m")
                        .font(.system(size: 15, weight: .medium, design: .monospaced))
                        .foregroundColor(.white)
                        .frame(width: 50, alignment: .leading)
                    
                    Slider(value: Binding(
                        get: { dataManager.statusMinimizedYPosition },
                        set: { newValue in
                            print("🎚️ [StatusView] Y slider changed to: \(newValue)")
                            dataManager.statusMinimizedYPosition = newValue
                            previewStatusPosition = (x: dataManager.statusMinimizedXPosition, y: newValue)
                            previewStatusActive = true
                            print("🎚️ [StatusView] Set previewStatusPosition to: \(String(describing: previewStatusPosition)), previewStatusActive: \(previewStatusActive)")
                            
                            // Cancel any existing hide task
                            hidePreviewTask?.cancel()
                            
                            // Schedule hiding the preview after 3 seconds of inactivity
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
                
                HStack {
                    Text("Down (0.5m)")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                    Spacer()
                    Text("Up (0.5m)")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.5))
                }
            }
            
            HStack {
                Spacer()
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
                    Text("Reset Position")
                        .font(.subheadline)
                        .foregroundColor(.purple)
                        .padding(.horizontal, 14)
                        .padding(.vertical, 8)
                        .background(Color.white.opacity(0.15))
                        .cornerRadius(8)
                }
                .buttonStyle(.plain)
                Spacer()
            }
            
            Text("💡 Preview will show where the minimized status will appear")
                .font(.caption)
                .foregroundColor(.white.opacity(0.5))
                .multilineTextAlignment(.leading)
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
