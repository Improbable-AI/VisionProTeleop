import SwiftUI
import CoreLocation
import UIKit
import SystemConfiguration.CaptiveNetwork


struct ContentView: View {
    @Environment(\.openImmersiveSpace) var openImmersiveSpace
    @Environment(\.dismissWindow) var dismissWindow
    var body: some View {
        VStack(spacing: 32) {
            HStack(spacing: 28) {
                Image(.graph2)
                    .resizable()
                    .aspectRatio(contentMode: .fit)
                    .frame(width: 1200)
                    .clipShape(.rect(cornerRadius: 24))
            }
            Text("You're on IP address [\(getIPAddress())]")
                .font(.largeTitle.weight(.medium))
                
            Button {
                Task {
                    await self.openImmersiveSpace(id: "immersiveSpace")
                    self.dismissWindow()
                }
            } label: {
                Text("Start")
                    .font(.largeTitle)
                    .padding(.vertical, 12)
                    .padding(.horizontal, 4)
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
            if addrFamily == UInt8(AF_INET) || addrFamily == UInt8(AF_INET6) {

                // wifi = ["en0"]
                // wired = ["en2", "en3", "en4"]
                // cellular = ["pdp_ip0","pdp_ip1","pdp_ip2","pdp_ip3"]

                let name: String = String(cString: (interface.ifa_name))
                if  name == "en0" || name == "en2" || name == "en3" || name == "en4" || name == "pdp_ip0" || name == "pdp_ip1" || name == "pdp_ip2" || name == "pdp_ip3" {
                    var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                    getnameinfo(interface.ifa_addr, socklen_t((interface.ifa_addr.pointee.sa_len)), &hostname, socklen_t(hostname.count), nil, socklen_t(0), NI_NUMERICHOST)
                    address = String(cString: hostname)
                }
            }
        }
        freeifaddrs(ifaddr)
    }
    return address ?? ""
}


func getWiFiName() -> String? {
  var ssid: String?

  if let interfaces = CNCopySupportedInterfaces() as NSArray? {
    for interface in interfaces {
      if let interfaceInfo = CNCopyCurrentNetworkInfo(interface as! CFString) as NSDictionary? {
        ssid = interfaceInfo[kCNNetworkInfoKeySSID as String] as? String
        break
      }
    }
  }

  return ssid
}
