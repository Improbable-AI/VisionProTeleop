import SwiftUI
import CoreLocation
import UIKit
import SystemConfiguration.CaptiveNetwork

// Version key for tracking "What's New" display
private let currentAppVersion = "2.0"
private let lastSeenVersionKey = "lastSeenAppVersion"

struct ContentView: View {
    @Environment(\.openImmersiveSpace) var openImmersiveSpace
    @Environment(\.dismissWindow) var dismissWindow
    @State private var showVideoStream = false
    @AppStorage("pythonServerIP") private var pythonServerIP = "10.29.239.70"
    @State private var showSettings = false
    @State private var serverReady = false
    @State private var showWhatsNew = false
    @State private var pulseNewBadge = false
    
    private var isNewUser: Bool {
        UserDefaults.standard.string(forKey: lastSeenVersionKey) != currentAppVersion
    }
    
    var body: some View {
        VStack(spacing: 32) {
            VStack(spacing: 4) {
                Text("VisionProTeleop")
                    .font(.system(size: 72, weight: .bold))
                Text("Tracking Streamer")
                    .font(.largeTitle)
                    .foregroundColor(.secondary)
            }
            .padding(.top, 32)
            
            // Main START button with NEW badge for updated features
            ZStack(alignment: .topTrailing) {
                Button {
                    Task {
                        await self.openImmersiveSpace(id: "combinedStreamSpace")
                        self.dismissWindow()
                    }
                } label: {
                    VStack(spacing: 8) {
                        Text("START")
                            .font(.largeTitle.bold())
                        Text("Hand Tracking + Video · Audio · Sim AR ")
                            .font(.title3)
                            .foregroundColor(.secondary)
                    }
                    .padding(.vertical, 20)
                    .padding(.horizontal, 32)
                    .background(
                        LinearGradient(
                            colors: [Color.purple.opacity(0.3), Color.blue.opacity(0.2)],
                            startPoint: .topLeading,
                            endPoint: .bottomTrailing
                        )
                    )
                    .cornerRadius(16)
                }
                .buttonStyle(.plain)
                
                // "NEW" badge for returning users
                if isNewUser {
                    Text("NEW")
                        .font(.caption.bold())
                        .foregroundColor(.white)
                        .padding(.horizontal, 8)
                        .padding(.vertical, 4)
                        .background(Color.orange)
                        .cornerRadius(8)
                        .offset(x: 10, y: -10)
                        .scaleEffect(pulseNewBadge ? 1.1 : 1.0)
                        .animation(.easeInOut(duration: 0.8).repeatForever(autoreverses: true), value: pulseNewBadge)
                        .onAppear { pulseNewBadge = true }
                }
            }
            .padding(.top, 16)
            
            VStack(spacing: 8) {
                VStack(spacing: 4) {
                    Text("Device IP Addresses:")
                        .font(.title2)
                        .foregroundColor(.secondary)
                    
                    ForEach(getIPAddresses(), id: \.address) { ip in
                        HStack {
                            Text(ip.name + ":")
                                .font(.title3.bold())
                                .foregroundColor(.secondary)
                            Text(ip.address)
                                .font(.title3)
                        }
                    }
                }
                .padding(.bottom, 8)
                
                HStack(spacing: 8) {
                    Circle()
                        .fill(serverReady ? Color.green : Color.orange)
                        .frame(width: 16, height: 16)
                    Text(serverReady ? "gRPC Server Ready" : "Starting gRPC Server...")
                        .font(.title3)
                        .foregroundColor(serverReady ? .green : .orange)
                }
                .onAppear {
                    // Poll for server ready status
                    Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { timer in
                        serverReady = DataManager.shared.grpcServerReady
                        if serverReady {
                            timer.invalidate()
                        }
                    }
                }
            }
            
            // Settings and Exit buttons
            HStack(spacing: 24) {
                
                // What's New button (always available)
                Button {
                    showWhatsNew = true
                } label: {
                    ZStack {
                        Circle()
                            .fill(Color.blue.opacity(0.8))
                            .frame(width: 50, height: 50)
                        Image(systemName: "sparkles")
                            .font(.title2.bold())
                            .foregroundColor(.white)
                    }
                }
                .buttonStyle(.plain)
                
                Button {
                    exit(0)
                } label: {
                    ZStack {
                        Circle()
                            .fill(Color.red)
                            .frame(width: 50, height: 50)
                        Text("✕")
                            .font(.title.bold())
                            .foregroundColor(.white)
                    }
                }
                .buttonStyle(.plain)
            }
            
        }
        .padding(32)
        .frame(minWidth: 700, minHeight: 600)
        .onAppear {
            // Show What's New automatically for returning users
            if isNewUser {
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                    showWhatsNew = true
                }
            }
        }
        .sheet(isPresented: $showWhatsNew) {
            WhatsNewView {
                // Mark as seen when dismissed
                UserDefaults.standard.set(currentAppVersion, forKey: lastSeenVersionKey)
                showWhatsNew = false
            }
        }
    }
}

// MARK: - What's New View
struct WhatsNewView: View {
    let onDismiss: () -> Void
    
    var body: some View {
        VStack(spacing: 24) {
            Text("What's New")
                .font(.system(size: 48, weight: .bold))
                .padding(.top, 32)
            
            Text("VisionProTeleop 2.0")
                .font(.title2)
                .foregroundColor(.secondary)
            
            VStack(alignment: .leading, spacing: 20) {
                FeatureRow(
                    icon: "video.fill",
                    iconColor: .blue,
                    title: "Video Streaming",
                    description: "Stream video from any client machine to Vision Pro"
                )
                
                FeatureRow(
                    icon: "speaker.wave.3.fill",
                    iconColor: .green,
                    title: "Audio Streaming",
                    description: "Bidirectional audio support for immersive experiences"
                )
                
                FeatureRow(
                    icon: "cube.transparent",
                    iconColor: .purple,
                    title: "MuJoCo AR Streaming",
                    description: "Visualize MuJoCo simulations in augmented reality"
                )
                
                FeatureRow(
                    icon: "hand.raised.fill",
                    iconColor: .orange,
                    title: "Enhanced Hand Tracking",
                    description: "Improved accuracy and lower latency for teleoperation"
                )
            }
            .padding(.horizontal, 32)
            .padding(.vertical, 16)
            
            // Upgrade notice
            VStack(alignment: .leading, spacing: 8) {
                HStack(spacing: 8) {
                    Image(systemName: "exclamationmark.triangle.fill")
                        .foregroundColor(.yellow)
                    Text("Don't forget to upgrade the Python package:")
                        .font(.subheadline.bold())
                }
                
                Text("pip install --upgrade avp_stream")
                    .font(.system(.footnote, design: .monospaced))
                    .padding(10)
                    .frame(maxWidth: .infinity, alignment: .leading)
                    .background(Color.black.opacity(0.3))
                    .cornerRadius(8)
            }
            .padding(.horizontal, 32)
            
            Spacer()
            
            Button {
                onDismiss()
            } label: {
                Text("Get Started")
                    .font(.title2.bold())
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 16)
                    .background(Color.blue)
                    .cornerRadius(12)
            }
            .buttonStyle(.plain)
            .padding(.horizontal, 32)
            .padding(.bottom, 32)
        }
        .frame(width: 500, height: 700)
    }
}

struct FeatureRow: View {
    let icon: String
    let iconColor: Color
    let title: String
    let description: String
    
    var body: some View {
        HStack(alignment: .top, spacing: 16) {
            Image(systemName: icon)
                .font(.title)
                .foregroundColor(iconColor)
                .frame(width: 40)
            
            VStack(alignment: .leading, spacing: 4) {
                Text(title)
                    .font(.headline)
                Text(description)
                    .font(.subheadline)
                    .foregroundColor(.secondary)
            }
        }
    }
}

func getIPAddresses() -> [(name: String, address: String)] {
    var addresses: [(name: String, address: String)] = []
    var ifaddr: UnsafeMutablePointer<ifaddrs>? = nil
    if getifaddrs(&ifaddr) == 0 {
        var ptr = ifaddr
        while ptr != nil {
            defer { ptr = ptr?.pointee.ifa_next }

            guard let interface = ptr?.pointee else { continue }
            let addrFamily = interface.ifa_addr.pointee.sa_family
            if addrFamily == UInt8(AF_INET) {
                let name: String = String(cString: (interface.ifa_name))
                var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                getnameinfo(interface.ifa_addr, socklen_t((interface.ifa_addr.pointee.sa_len)), &hostname, socklen_t(hostname.count), nil, socklen_t(0), NI_NUMERICHOST)
                let address = String(cString: hostname)
                if address != "127.0.0.1" {
                    addresses.append((name: name, address: address))
                }
            }
        }
        freeifaddrs(ifaddr)
    }
    return addresses
}


func getWiFiName() -> String? {
  // CNCopyCurrentNetworkInfo is unavailable in visionOS
  // WiFi SSID access is restricted on this platform
  return nil
}
