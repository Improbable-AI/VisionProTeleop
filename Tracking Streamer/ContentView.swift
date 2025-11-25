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
