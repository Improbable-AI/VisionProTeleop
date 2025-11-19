import SwiftUI
import Combine

/// Observable object to hold video frames from WebRTC stream
/// This will be used to pass video data from WebRTC client to the RealityView
@MainActor
class ImageData: ObservableObject {
    @Published var left: UIImage?
    @Published var right: UIImage?
    
    init() {
        self.left = nil
        self.right = nil
    }
}
