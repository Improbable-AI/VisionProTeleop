import SwiftUI

@main
struct VisionProTeleopApp: App {
    @StateObject private var imageData = ImageData()
    @StateObject private var appModel = 🥽AppModel()
    
    var body: some Scene {
        WindowGroup {
            ContentView()
        }
        .windowResizability(.contentSize)
        
        // Hand tracking view (existing)
        ImmersiveSpace(id: "immersiveSpace") {
            🌐RealityView(model: appModel)
        }
        
        // Video streaming view (new)
        ImmersiveSpace(id: "videoStreamSpace") {
            ImmersiveView()
                .environmentObject(imageData)
        }
    }
    
    init() {
        print("🚀 [DEBUG] VisionProTeleopApp.init() - App launching...")
        🧑HeadTrackingComponent.registerComponent()
        🧑HeadTrackingSystem.registerSystem()
        
        // Start gRPC server immediately when app launches
        print("🌐 [DEBUG] Starting gRPC server on app launch...")
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
            print("🔧 [DEBUG] Calling startServer() from app init...")
            startServer()
        }
    }
}

