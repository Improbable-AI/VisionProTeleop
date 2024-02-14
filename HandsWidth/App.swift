import SwiftUI

@main
struct HandsWidthApp: App {
    var body: some Scene {
        WindowGroup {
            ContentView()
        }
        .windowResizability(.contentSize)
        ImmersiveSpace(id: "immersiveSpace") {
            🌐RealityView(model: 🥽AppModel())
        }
    }
    init() {
        🧑HeadTrackingComponent.registerComponent()
        🧑HeadTrackingSystem.registerSystem()
    }
}
