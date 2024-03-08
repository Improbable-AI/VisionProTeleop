import RealityKit
import ARKit
import SwiftUI

struct 🧑HeadTrackingComponent: Component, Codable {
    init() {}
}

struct 🧑HeadTrackingSystem: System {
    private static let query = EntityQuery(where: .has(🧑HeadTrackingComponent.self))
    
    private let session = ARKitSession()
    private let provider = WorldTrackingProvider()
    
    init(scene: RealityKit.Scene) {
        self.setUpSession()
    }
    
    private func setUpSession() {
        Task {
            do {
                try await self.session.run([self.provider])
            } catch {
                assertionFailure()
            }
        }
    }
    
    func update(context: SceneUpdateContext) {
        let entities = context.scene.performQuery(Self.query).map { $0 }
        
        guard !entities.isEmpty,
              let deviceAnchor = self.provider.queryDeviceAnchor(atTimestamp: CACurrentMediaTime()) else { return }
        
        let cameraTransform = Transform(matrix: deviceAnchor.originFromAnchorTransform)
        
        for entity in entities {
            entity.look(at: cameraTransform.translation,
                        from: entity.position(relativeTo: nil),
                        relativeTo: nil,
                        forward: .positiveZ)
        }
    }
}

