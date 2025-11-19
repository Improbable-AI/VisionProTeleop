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
    let hasFrames: Bool
    let showVideoStatus: Bool  // Whether to show WebRTC and frame status
    @Binding var isMinimized: Bool
    @State private var ipAddress: String = ""
    @State private var pythonConnected: Bool = false
    @State private var pythonIP: String = "Not connected"
    @State private var webrtcConnected: Bool = false
    
    init(hasFrames: Bool = false, showVideoStatus: Bool = true, isMinimized: Binding<Bool> = .constant(false)) {
        self.hasFrames = hasFrames
        self.showVideoStatus = showVideoStatus
        self._isMinimized = isMinimized
        print("🟢 [StatusView] StatusOverlay init called, hasFrames: \(hasFrames), showVideoStatus: \(showVideoStatus)")
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
                if let pythonClientIP = DataManager.shared.pythonClientIP {
                    pythonConnected = true
                    pythonIP = pythonClientIP
                    print("🟣 [StatusView] Python connected: \(pythonIP)")
                }
                
                webrtcConnected = DataManager.shared.webrtcServerInfo != nil
                print("🟣 [StatusView] WebRTC connected: \(webrtcConnected)")
            }
        }
    }
    
    private var minimizedView: some View {
        HStack(spacing: 8) {
            // Expand button
            Button {
                withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                    isMinimized = false
                }
            } label: {
                ZStack {
                    Circle()
                        .fill(Color.white.opacity(0.3))
                        .frame(width: 24, height: 24)
                    Image(systemName: "arrow.up.left.and.arrow.down.right")
                        .font(.system(size: 10, weight: .bold))
                        .foregroundColor(.white)
                }
            }
            .buttonStyle(.plain)
            
            Circle()
                .fill(Color.green)
                .frame(width: 12, height: 12)
            Circle()
                .fill(pythonConnected ? Color.green : Color.red)
                .frame(width: 12, height: 12)
            if showVideoStatus {
                Circle()
                    .fill(webrtcConnected ? Color.green : Color.orange)
                    .frame(width: 12, height: 12)
            }
            Button {
                exit(0)
            } label: {
                ZStack {
                    Circle()
                        .fill(Color.red)
                        .frame(width: 24, height: 24)
                    Text("✕")
                        .font(.system(size: 12, weight: .bold))
                        .foregroundColor(.white)
                }
            }
            .buttonStyle(.plain)
        }
        .padding(12)
        .background(Color.black.opacity(0.6))
        .cornerRadius(20)
    }
    
    private var expandedView: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                // Minimize button
                Button {
                    withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                        isMinimized = true
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
            }
            
            // Show waiting message when no frames are available (only for video mode)
            if showVideoStatus && !hasFrames {
                Divider()
                    .background(Color.white.opacity(0.3))
                
                HStack {
                    ProgressView()
                        .progressViewStyle(CircularProgressViewStyle(tint: .white))
                        .scaleEffect(0.8)
                    Text("Waiting for frames to arrive...")
                        .foregroundColor(.white.opacity(0.9))
                        .font(.subheadline)
                        .fontWeight(.medium)
                }
            }
        }
        .padding(20)
        .background(Color.black.opacity(0.7))
        .cornerRadius(16)
    }
}

/// Creates a floating status entity that follows the head
func createStatusEntity() -> Entity {
    let statusEntity = Entity()
    statusEntity.name = "statusDisplay"
    return statusEntity
}
