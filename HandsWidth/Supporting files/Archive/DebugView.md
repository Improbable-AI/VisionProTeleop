# 👆DebugView

import SwiftUI
import RealityKit
import ARKit

struct 👆DebugView: View {
    @EnvironmentObject var model: 📱AppModel
    @State private var rootEntity: Entity?
    @State private var text: String = "placeholder"
    var body: some View {
        RealityView { content, _ in
            let rootEntity = Entity()
            rootEntity.name = "ROOT"
            content.add(rootEntity)
            self.rootEntity = rootEntity
            let entity = Entity()
            entity.name = "POINTER"
            entity.components.set(📍HeadAnchorComponent())
            entity.components.set(InputTargetComponent())
            entity.components.set(CollisionComponent(shapes: [.generateConvex(from: .generateSphere(radius: 0.1))]))
            entity.components.set(ModelComponent(mesh: .generateSphere(radius: 0.02),
                                                 materials: [SimpleMaterial(color: .white, isMetallic: false)]))
            rootEntity.addChild(entity)
            do {
                let entity = Entity()
                entity.name = "LINE"
                entity.components.set(OpacityComponent(opacity: 0.9))
                rootEntity.addChild(entity)
            }
        } update: { content, attachments in
            let entity = attachments.entity(for: "resultLabel")!
            entity.components.set(📍HeadAnchorComponent())
            entity.name = "resultLabel"
            rootEntity?.addChild(entity)
            if let p1 = rootEntity?.findEntity(named: "1")?.position,
               let p2 = rootEntity?.findEntity(named: "2")?.position {
                entity.position = (p1 + p2) / 2
            }
        } attachments: {
            Attachment(id: "resultLabel") {
                Text(self.text)
                    .font(.system(size: 54).bold())
                    .padding(24)
                    .glassBackgroundEffect()
            }
        }
        .onTapGesture {
            self.setPoints()
            self.setText()
            self.setLine()
        }
    }
}

fileprivate extension 👆DebugView {
    func setPoints() {
        guard let pointer = rootEntity?.findEntity(named: "POINTER") else { return }
        if rootEntity?.findEntity(named: "1") == nil {
            let entity = Entity()
            entity.name = "1"
            entity.position = pointer.position
            entity.components.set(ModelComponent(mesh: .generateSphere(radius: 0.025),
                                                 materials: [SimpleMaterial(color: .red, isMetallic: false)]))
            rootEntity?.addChild(entity)
        } else {
            if let entity2 = rootEntity?.findEntity(named: "2") {
                rootEntity?.removeChild(entity2)
            }
            let entity = Entity()
            entity.name = "2"
            entity.position = pointer.position
            entity.components.set(ModelComponent(mesh: .generateSphere(radius: 0.025),
                                                 materials: [SimpleMaterial(color: .green, isMetallic: false)]))
            rootEntity?.addChild(entity)
        }
    }
    func setText() {
        guard let p1 = rootEntity?.findEntity(named: "1")?.position,
              let p2 = rootEntity?.findEntity(named: "2")?.position else {
            return
        }
        let lengthFormatter = LengthFormatter()
        lengthFormatter.numberFormatter.maximumFractionDigits = 2
        self.text = lengthFormatter.string(fromValue: .init(distance(p1, p2)), unit: .meter)
    }
    func setLine() {
        guard let p1 = rootEntity?.findEntity(named: "1")?.position,
              let p2 = rootEntity?.findEntity(named: "2")?.position else {
            return
        }
        if let entity = rootEntity?.findEntity(named: "LINE") {
            entity.position = (p1 + p2) / 2
            entity.components.set(ModelComponent(mesh: .generateBox(width: 0.01,
                                                                    height: 0.01,
                                                                    depth: distance(p1, p2),
                                                                    cornerRadius: 0.005),
                                                 materials: [SimpleMaterial(color: .white, isMetallic: false)]))
            entity.look(at: p1,
                        from: entity.position,
                        relativeTo: nil)
            let occlusionEntity = Entity()
            occlusionEntity.components.set(ModelComponent(mesh: .generateSphere(radius: 0.08),
                                                          materials: [OcclusionMaterial()]))
            entity.addChild(occlusionEntity)
        }
    }
}

## system

func update(context: SceneUpdateContext) {
    guard let deviceAnchor = self.provider.queryDeviceAnchor(atTimestamp: CACurrentMediaTime()) else {
        return
    }
    for entity in context.entities(matching: .init(where: .has(📍HeadAnchorComponent.self)), 
                                   updatingSystemWhen: .rendering) {
        if entity.name == "resultLabel" {
            entity.look(at: Transform(matrix: deviceAnchor.originFromAnchorTransform).translation,
                        from: entity.position(relativeTo: nil),
                        relativeTo: nil,
                        forward: .positiveZ)
        }
if DEBUG
        if entity.name == "POINTER" {
            entity.transform = Transform(matrix: deviceAnchor.originFromAnchorTransform)
            entity.setPosition([0, 0, -1], relativeTo: entity)
        }
endif
    }
}
