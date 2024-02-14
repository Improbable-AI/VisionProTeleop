# Hand to ground Mode

```
import SwiftUI

enum 🪄Mode: String {
    case handToHand, handToGround
}

extension 🪄Mode: CaseIterable, Identifiable {
    var id: Self { self }
    var localizedTitle: LocalizedStringResource {
        switch self {
            case .handToHand: "Hand to hand"
            case .handToGround: "Hand to ground"
        }
    }
}
```

```
@AppStorage("mode") var mode: 🪄Mode = .handToHand
```

```
Section {
    Picker("Mode", selection: self.$model.mode) {
        ForEach(🪄Mode.allCases) {
            Text($0.localizedTitle)
        }
    }
}
```

```
let heightLineEntity = Entity()
let groundPointEntity: Entity = {
    let radius: Float = 0.03
    let value = ModelEntity(mesh: .generateSphere(radius: radius),
                            materials: [SimpleMaterial(color: .yellow, isMetallic: false)])
    let occlusion = ModelEntity(mesh: .generateCylinder(height: radius, radius: radius),
                                materials: [OcclusionMaterial()])
    occlusion.position.y -= radius / 2
    value.addChild(occlusion)
    return value
}()
```

```
guard let rightPosition = self.indexTipEntities[.right]?.position else {
    assertionFailure(); return
}
self.heightLineEntity.position = (self.groundPointEntity.position + rightPosition) / 2
self.heightLineEntity.components.set(
    ModelComponent(mesh: .generateBox(width: 0.01,
                                      height: 0.01,
                                      depth: distance(self.groundPointEntity.position, rightPosition),
                                      cornerRadius: 0.005),
                   materials: [SimpleMaterial(color: .white, isMetallic: false)])
)
self.heightLineEntity.look(at: self.groundPointEntity.position,
                           from: self.heightLineEntity.position,
                           relativeTo: nil)
self.heightLineEntity.addChild(ModelEntity(mesh: .generateSphere(radius: 0.08),
                                           materials: [OcclusionMaterial()]))
```
