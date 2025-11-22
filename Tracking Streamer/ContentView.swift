import SwiftUI
import CoreLocation
import UIKit
import SystemConfiguration.CaptiveNetwork


struct ContentView: View {
    @Environment(\.openImmersiveSpace) var openImmersiveSpace
    @Environment(\.dismissWindow) var dismissWindow
    @State private var showVideoStream = false
    @AppStorage("pythonServerIP") private var pythonServerIP = "10.29.239.70"
    @State private var showSettings = false
    @State private var serverReady = false
    
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
            
            if showSettings {
                VStack(spacing: 16) {
                    Text("Python Server IP:")
                        .font(.title2)
                    TextField("e.g., 10.29.239.70", text: $pythonServerIP)
                        .textFieldStyle(.roundedBorder)
                        .font(.title3)
                        .padding(.horizontal, 32)
                        .multilineTextAlignment(.center)
                }
            }
            
            // Two start buttons side by side
            HStack(spacing: 40) {
                Button {
                    Task {
                        await self.openImmersiveSpace(id: "immersiveSpace")
                        self.dismissWindow()
                    }
                } label: {
                    VStack(spacing: 8) {
                        Text("START")
                            .font(.largeTitle.bold())
                        Text("Only Hand Tracking")
                            .font(.title3)
                            .foregroundColor(.secondary)
                    }
                    .padding(.vertical, 20)
                    .padding(.horizontal, 40)
                    .background(Color.blue.opacity(0.2))
                    .cornerRadius(16)
                }
                .buttonStyle(.plain)
                
                Button {
                    Task {
                        await self.openImmersiveSpace(id: "videoStreamSpace")
                        self.dismissWindow()
                    }
                } label: {
                    VStack(spacing: 8) {
                        Text("START")
                            .font(.largeTitle.bold())
                        Text("with Video/Audio Streaming")
                            .font(.title3)
                            .foregroundColor(.secondary)
                            .fixedSize()
                    }
                    .padding(.vertical, 20)
                    .padding(.horizontal, 40)
                    .background(Color.green.opacity(0.2))
                    .cornerRadius(16)
                }
                .buttonStyle(.plain)
            }
            .padding(.top, 16)
            
            VStack(spacing: 8) {
                Text("You're on IP address [\(getIPAddress())]")
                    .font(.largeTitle.weight(.medium))
                
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
                Button {
                    showSettings.toggle()
                } label: {
                    ZStack {
                        Circle()
                            .fill(Color.gray.opacity(0.3))
                            .frame(width: 50, height: 50)
                        Image(systemName: "gearshape.fill")
                            .font(.title2)
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
    }
}

func getIPAddress() -> String {
    var address: String?
    var ifaddr: UnsafeMutablePointer<ifaddrs>? = nil
    if getifaddrs(&ifaddr) == 0 {
        var ptr = ifaddr
        while ptr != nil {
            defer { ptr = ptr?.pointee.ifa_next }

            guard let interface = ptr?.pointee else { return "" }
            let addrFamily = interface.ifa_addr.pointee.sa_family
            if addrFamily == UInt8(AF_INET) {
                // Only check en0 (WiFi interface)
                let name: String = String(cString: (interface.ifa_name))
                if name == "en0" {
                    var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                    getnameinfo(interface.ifa_addr, socklen_t((interface.ifa_addr.pointee.sa_len)), &hostname, socklen_t(hostname.count), nil, socklen_t(0), NI_NUMERICHOST)
                    address = String(cString: hostname)
                    break  // Found en0, no need to continue
                }
            }
        }
        freeifaddrs(ifaddr)
    }
    return address ?? ""
}


func getWiFiName() -> String? {
  // CNCopyCurrentNetworkInfo is unavailable in visionOS
  // WiFi SSID access is restricted on this platform
  return nil
}
