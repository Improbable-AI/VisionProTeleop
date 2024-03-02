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

//==== Ref: https://developer.apple.com/documentation/visionos/swift-splash ====
//import ARKit
//import RealityKit
//import SwiftUI
//
//// An ECS system that points all entities containing a billboard component at the camera.
//public struct BillboardSystem: System {
//
//    static let query = EntityQuery(where: .has(SwiftSplashTrackPieces.BillboardComponent.self))
//
//    private let arkitSession = ARKitSession()
//    private let worldTrackingProvider = WorldTrackingProvider()
//    
//    public init(scene: RealityKit.Scene) {
//        setUpSession()
//    }
//    
//    func setUpSession() {
//        Task {
//            do {
//                try await arkitSession.run([worldTrackingProvider])
//            } catch {
//                print("Error: \(error)")
//            }
//        }
//    }
//    
//    public func update(context: SceneUpdateContext) {
//        
//        let entities = context.scene.performQuery(Self.query).map({ $0 })
//        
//        guard !entities.isEmpty,
//              let pose = worldTrackingProvider.queryDeviceAnchor(atTimestamp: CACurrentMediaTime()) else { return }
//        
//        let cameraTransform = Transform(matrix: pose.originFromAnchorTransform)
//        
//        for entity in entities {
//            entity.look(at: cameraTransform.translation,
//                        from: entity.scenePosition,//from: entity.position(relativeTo: nil),
//                        relativeTo: nil,
//                        forward: .positiveZ)
//        }
//    }
//}
