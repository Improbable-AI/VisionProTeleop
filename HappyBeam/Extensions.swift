/*
See the LICENSE.txt file for this sample’s licensing information.

Abstract:
Extensions and utilities.
*/

import SwiftUI
import RealityKit

extension Entity {
    /// Property for getting or setting an entity's `modelComponent`.
    var modelComponent: ModelComponent? {
        get { components[ModelComponent.self] }
        set { components[ModelComponent.self] = newValue }
    }
    
    var descendentsWithModelComponent: [Entity] {
        var descendents = [Entity]()
        
        for child in children {
            if child.components[ModelComponent.self] != nil {
                descendents.append(child)
            }
            descendents.append(contentsOf: child.descendentsWithModelComponent)
        }
        return descendents
    }
}

extension Entity {
    func setMaterialParameterValues(parameter: String, value: MaterialParameters.Value) {
        let modelEntities = descendentsWithModelComponent
        for entity in modelEntities {
            if var modelComponent = entity.modelComponent {
               
                modelComponent.materials = modelComponent.materials.map {
                    
                    guard var material = $0 as? ShaderGraphMaterial else { return $0 }
                    if material.parameterNames.contains(parameter) {
                        do {
                            try material.setParameter(name: parameter, value: value)
                        } catch {
                            print("Error setting parameter: \(error.localizedDescription)")
                        }
                    }
                    return material
                }
                entity.modelComponent = modelComponent
            }
        }
    }
    
    subscript(parentMatching targetName: String) -> Entity? {
        if name.contains(targetName) {
            return self
        }
        
        guard let nextParent = parent else {
            return nil
        }
        
        return nextParent[parentMatching: targetName]
    }
    
    func getParent(nameBeginsWith name: String) -> Entity? {
        if self.name.hasPrefix(name) {
            return self
        }
        guard let nextParent = parent else {
            return nil
        }
        
        return nextParent.getParent(nameBeginsWith: name)
    }
    
    func getParent(withName name: String) -> Entity? {
        if self.name == name {
            return self
        }
        guard let nextParent = parent else {
            return nil
        }
        
        return nextParent.getParent(withName: name)
    }
    
    subscript(descendentMatching targetName: String) -> Entity? {
        if name.contains(targetName) {
            return self
        }
        
        var match: Entity? = nil
        for child in children {
            match = child[descendentMatching: targetName]
            if let match = match {
                return match
            }
        }
        
        return match
    }
    
    func getSelfOrDescendent(withName name: String) -> Entity? {
        if self.name == name {
            return self
        }
        var match: Entity? = nil
        for child in children {
            match = child.getSelfOrDescendent(withName: name)
            if match != nil {
                return match
            }
        }
        
        return match
    }
    
    func forward(relativeTo referenceEntity: Entity?) -> SIMD3<Float> {
        normalize(convert(direction: SIMD3<Float>(0, 0, +1), to: referenceEntity))
    }
    
    var forward: SIMD3<Float> {
        forward(relativeTo: nil)
    }
}

extension Comparable {
    func clamped(to limits: ClosedRange<Self>) -> Self {
        min(max(self, limits.lowerBound), limits.upperBound)
    }
}

extension SIMD4 {
    var xyz: SIMD3<Scalar> {
        self[SIMD3(0, 1, 2)]
    }
}

