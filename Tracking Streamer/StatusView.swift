import SwiftUI
import RealityKit
import Network

/// Network information manager for displaying connection status
class NetworkInfoManager: ObservableObject {
    @Published var ipAddress: String = ""
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
        ipAddress = getIPAddress()
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
    @Binding var previewStatusPosition: (x: Float, y: Float)?
    @Binding var previewStatusActive: Bool
    @ObservedObject var dataManager = DataManager.shared
    @State private var ipAddress: String = ""
    @State private var pythonConnected: Bool = false
    @State private var pythonIP: String = "Not connected"
    @State private var webrtcConnected: Bool = false
    @State private var hidePreviewTask: Task<Void, Never>?
    @State private var showStatusPositionControls: Bool = false
    
    init(hasFrames: Binding<Bool> = .constant(false), showVideoStatus: Bool = true, isMinimized: Binding<Bool> = .constant(false), showViewControls: Binding<Bool> = .constant(false), previewZDistance: Binding<Float?> = .constant(nil), previewActive: Binding<Bool> = .constant(false), userInteracted: Binding<Bool> = .constant(false), videoMinimized: Binding<Bool> = .constant(false), previewStatusPosition: Binding<(x: Float, y: Float)?> = .constant(nil), previewStatusActive: Binding<Bool> = .constant(false)) {
        self._hasFrames = hasFrames
        self.showVideoStatus = showVideoStatus
        self._isMinimized = isMinimized
        self._showViewControls = showViewControls
        self._previewZDistance = previewZDistance
        self._previewActive = previewActive
        self._userInteracted = userInteracted
        self._videoMinimized = videoMinimized
        self._previewStatusPosition = previewStatusPosition
        self._previewStatusActive = previewStatusActive
        print("🟢 [StatusView] StatusOverlay init called, hasFrames: \(hasFrames.wrappedValue), showVideoStatus: \(showVideoStatus)")
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
            ipAddress = getIPAddress()
            print("🔴 [StatusView] IP Address: \(ipAddress)")
            
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
                
                // Detect disconnection and maximize status view
                if (wasPythonConnected && !pythonConnected) || (wasWebrtcConnected && !webrtcConnected) {
                    print("🔌 [StatusView] Connection lost - maximizing status view")
                    withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                        isMinimized = false
                        userInteracted = false  // Reset so it can auto-minimize again on next connection
                        hasFrames = false  // Clear frames flag
                    }
                }
            }
        }
    }
    
    private var minimizedView: some View {
        HStack(spacing: 24) {
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
            }
            
            Button {
                exit(0)
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
        .padding(30)
        .background(Color.black.opacity(0.6))
        .cornerRadius(36)
    }
    
    private var expandedView: some View {
        VStack(alignment: .leading, spacing: 12) {
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
                            .frame(width: 30, height: 30)
                        Image(systemName: "arrow.down.right.and.arrow.up.left")
                            .font(.system(size: 12, weight: .bold))
                            .foregroundColor(.white)
                    }
                }
                .buttonStyle(.plain)
                
                Text("Network Status")
                    .font(.headline)
                    .foregroundColor(.white)
                Spacer()
                
                Button {
                    exit(0)
                } label: {
                    ZStack {
                        Circle()
                            .fill(Color.red)
                            .frame(width: 30, height: 30)
                        Text("✕")
                            .font(.body.bold())
                            .foregroundColor(.white)
                    }
                }
                .buttonStyle(.plain)
            }
            
            Divider()
                .background(Color.white.opacity(0.3))
            
            HStack {
                Circle()
                    .fill(Color.green)
                    .frame(width: 10, height: 10)
                Text("Vision Pro IP:")
                    .foregroundColor(.white.opacity(0.8))
                Text(ipAddress)
                    .foregroundColor(.white)
                    .fontWeight(.medium)
            }
            .font(.subheadline)
            
            HStack {
                Circle()
                    .fill(pythonConnected ? Color.green : Color.red)
                    .frame(width: 10, height: 10)
                Text("Python Client:")
                    .foregroundColor(.white.opacity(0.8))
                Text(pythonIP)
                    .foregroundColor(.white)
                    .fontWeight(.medium)
            }
            .font(.subheadline)
            
            // Only show WebRTC status if video streaming is enabled
            if showVideoStatus {
                HStack {
                    Circle()
                        .fill(webrtcConnected ? Color.green : Color.orange)
                        .frame(width: 10, height: 10)
                    Text("WebRTC:")
                        .foregroundColor(.white.opacity(0.8))
                    Text(webrtcConnected ? "Connected" : "Waiting...")
                        .foregroundColor(.white)
                        .fontWeight(.medium)
                }
                .font(.subheadline)
                
                // Show detailed track information when connected
                if webrtcConnected {
                    Divider()
                        .background(Color.white.opacity(0.2))
                    
                    VStack(alignment: .leading, spacing: 6) {
                        Text("Stream Details")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.6))
                            .fontWeight(.semibold)
                        
                        // Video track info
                        HStack(spacing: 6) {
                            Image(systemName: "video.fill")
                                .font(.system(size: 10))
                                .foregroundColor(.blue)
                            Text("Video:")
                                .font(.caption)
                                .foregroundColor(.white.opacity(0.7))
                            if dataManager.stereoEnabled {
                                Text("Stereo")
                                    .font(.caption)
                                    .foregroundColor(.cyan)
                                    .fontWeight(.medium)
                            } else {
                                Text("Mono")
                                    .font(.caption)
                                    .foregroundColor(.white)
                                    .fontWeight(.medium)
                            }
                        }
                        
                        // Audio track info (optional - may not be present)
                        HStack(spacing: 6) {
                            Image(systemName: "waveform")
                                .font(.system(size: 10))
                                .foregroundColor(dataManager.audioEnabled ? .green : .gray)
                            Text("Audio:")
                                .font(.caption)
                                .foregroundColor(.white.opacity(0.7))
                            if dataManager.audioEnabled {
                                if dataManager.stereoAudioEnabled {
                                    Text("Stereo")
                                        .font(.caption)
                                        .foregroundColor(.green)
                                        .fontWeight(.medium)
                                } else {
                                    Text("Mono")
                                        .font(.caption)
                                        .foregroundColor(.white)
                                        .fontWeight(.medium)
                                }
                            } else {
                                Text("N/A")
                                    .font(.caption)
                                    .foregroundColor(.white.opacity(0.5))
                                    .fontWeight(.medium)
                            }
                        }
                    }
                    .padding(.vertical, 4)
                }
            }
            
            // Show waiting message when no frames are available (only for video mode)
            if showVideoStatus && !hasFrames {
                Divider()
                    .background(Color.white.opacity(0.3))
                
                HStack {
                    ProgressView()
                        .progressViewStyle(CircularProgressViewStyle(tint: .white))
                        .scaleEffect(0.8)
                    Text(dataManager.connectionStatus)
                        .foregroundColor(.white.opacity(0.9))
                        .font(.subheadline)
                        .fontWeight(.medium)
                        .lineLimit(2)
                }
            }
            
            // View controls section (expandable)
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
                            .font(.system(size: 14, weight: .medium))
                        Text("Modify Video View")
                            .font(.subheadline)
                            .fontWeight(.medium)
                        Spacer()
                        Image(systemName: "chevron.down")
                            .font(.system(size: 12, weight: .bold))
                    }
                    .foregroundColor(.white.opacity(0.9))
                    .padding(.vertical, 8)
                }
                .buttonStyle(.plain)
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
                            .font(.system(size: 14, weight: .medium))
                        Text("Modify Controller Position")
                            .font(.subheadline)
                            .fontWeight(.medium)
                        Spacer()
                        Image(systemName: "chevron.down")
                            .font(.system(size: 12, weight: .bold))
                    }
                    .foregroundColor(.white.opacity(0.9))
                    .padding(.vertical, 8)
                }
                .buttonStyle(.plain)
            }
        }
        .padding(20)
        .background(Color.black.opacity(0.7))
        .cornerRadius(16)
    }
    
    private var viewControlsSection: some View {
        VStack(alignment: .leading, spacing: 12) {
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
                        .font(.system(size: 14, weight: .medium))
                    Text("Modify Video View")
                        .font(.subheadline)
                        .fontWeight(.medium)
                    Spacer()
                    Image(systemName: "chevron.up")
                        .font(.system(size: 12, weight: .bold))
                }
                .foregroundColor(.white.opacity(0.9))
            }
            .buttonStyle(.plain)
            
            // Distance control
            VStack(alignment: .leading, spacing: 8) {
                Text("Distance (Z-axis)")
                    .font(.caption)
                    .foregroundColor(.white.opacity(0.7))
                
                HStack {
                    Text("\(String(format: "%.1f", -dataManager.videoPlaneZDistance))m")
                        .font(.system(size: 13, weight: .medium, design: .monospaced))
                        .foregroundColor(.white)
                        .frame(width: 45, alignment: .leading)
                    
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
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.5))
                    Spacer()
                    Text("Far (20m)")
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.5))
                }
            }
            
            Divider()
                .background(Color.white.opacity(0.2))
            
            // Height control
            VStack(alignment: .leading, spacing: 8) {
                Text("Height (Y-axis)")
                    .font(.caption)
                    .foregroundColor(.white.opacity(0.7))
                
                HStack {
                    Text("\(String(format: "%.2f", dataManager.videoPlaneYPosition))m")
                        .font(.system(size: 13, weight: .medium, design: .monospaced))
                        .foregroundColor(.white)
                        .frame(width: 45, alignment: .leading)
                    
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
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.5))
                    Spacer()
                    Text("Up (2m)")
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.5))
                }
            }
            
            Divider()
                .background(Color.white.opacity(0.2))
            
            // Auto-perpendicular toggle
            HStack {
                VStack(alignment: .leading, spacing: 4) {
                    Text("Auto-Perpendicular")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.7))
                    Text("Tilt plane to face head")
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.5))
                }
                Spacer()
                Toggle("", isOn: Binding(
                    get: { dataManager.videoPlaneAutoPerpendicular },
                    set: { newValue in
                        dataManager.videoPlaneAutoPerpendicular = newValue
                        // Show preview when toggling
                        previewActive = true
                        
                        // Cancel any existing hide task
                        hidePreviewTask?.cancel()
                        
                        // Schedule hiding the preview after 3 seconds
                        hidePreviewTask = Task { @MainActor in
                            try? await Task.sleep(nanoseconds: 3_000_000_000)
                            if !Task.isCancelled {
                                previewActive = false
                            }
                        }
                    }
                ))
                .labelsHidden()
                .tint(.blue)
            }
            
            HStack(spacing: 12) {
                Spacer()
                
                // Video minimize/maximize button
                Button {
                    withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                        videoMinimized.toggle()
                    }
                } label: {
                    HStack(spacing: 6) {
                        Image(systemName: videoMinimized ? "eye.fill" : "eye.slash.fill")
                            .font(.system(size: 11, weight: .bold))
                        Text(videoMinimized ? "Show Video" : "Hide Video")
                            .font(.caption)
                    }
                    .foregroundColor(.blue)
                    .padding(.horizontal, 10)
                    .padding(.vertical, 4)
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
                    Text("Reset All")
                        .font(.caption)
                        .foregroundColor(.blue)
                        .padding(.horizontal, 10)
                        .padding(.vertical, 4)
                        .background(Color.white.opacity(0.15))
                        .cornerRadius(6)
                }
                .buttonStyle(.plain)
                Spacer()
            }
            
            Text("💡 Adjust position and orientation of the video plane")
                .font(.caption2)
                .foregroundColor(.white.opacity(0.5))
                .multilineTextAlignment(.leading)
        }
    }
    
    private var statusPositionControlsSection: some View {
        VStack(alignment: .leading, spacing: 12) {
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
                        .font(.system(size: 14, weight: .medium))
                    Text("Modify Controller Position")
                        .font(.subheadline)
                        .fontWeight(.medium)
                    Spacer()
                    Image(systemName: "chevron.up")
                        .font(.system(size: 12, weight: .bold))
                }
                .foregroundColor(.white.opacity(0.9))
            }
            .buttonStyle(.plain)
            
            // X position control
            VStack(alignment: .leading, spacing: 8) {
                Text("X Position (Left-Right)")
                    .font(.caption)
                    .foregroundColor(.white.opacity(0.7))
                
                HStack {
                    Text("\(String(format: "%.2f", dataManager.statusMinimizedXPosition))m")
                        .font(.system(size: 13, weight: .medium, design: .monospaced))
                        .foregroundColor(.white)
                        .frame(width: 45, alignment: .leading)
                    
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
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.5))
                    Spacer()
                    Text("Right (0.5m)")
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.5))
                }
            }
            
            Divider()
                .background(Color.white.opacity(0.2))
            
            // Y position control
            VStack(alignment: .leading, spacing: 8) {
                Text("Y Position (Up-Down)")
                    .font(.caption)
                    .foregroundColor(.white.opacity(0.7))
                
                HStack {
                    Text("\(String(format: "%.2f", dataManager.statusMinimizedYPosition))m")
                        .font(.system(size: 13, weight: .medium, design: .monospaced))
                        .foregroundColor(.white)
                        .frame(width: 45, alignment: .leading)
                    
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
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.5))
                    Spacer()
                    Text("Up (0.5m)")
                        .font(.caption2)
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
                        .font(.caption)
                        .foregroundColor(.purple)
                        .padding(.horizontal, 10)
                        .padding(.vertical, 4)
                        .background(Color.white.opacity(0.15))
                        .cornerRadius(6)
                }
                .buttonStyle(.plain)
                Spacer()
            }
            
            Text("💡 Preview will show where the minimized status will appear")
                .font(.caption2)
                .foregroundColor(.white.opacity(0.5))
                .multilineTextAlignment(.leading)
        }
    }
}

/// Preview view that looks exactly like the minimized status but with 50% opacity
struct StatusPreviewView: View {
    let showVideoStatus: Bool
    
    var body: some View {
        HStack(spacing: 24) {
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
        .opacity(0.5)  // 50% transparent
    }
}

/// Creates a floating status entity that follows the head
func createStatusEntity() -> Entity {
    let statusEntity = Entity()
    statusEntity.name = "statusDisplay"
    return statusEntity
}
