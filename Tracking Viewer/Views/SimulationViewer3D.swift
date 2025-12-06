//
//  SimulationViewer3D.swift
//  Tracking Viewer
//
//  Created on 12/04/25.
//

import SwiftUI
import SceneKit
import simd

struct SimulationViewer3D: UIViewRepresentable {
    let usdzURL: URL
    let currentFrame: SimulationFrame?
    
    func makeUIView(context: Context) -> SCNView {
        let scnView = SCNView()
        scnView.backgroundColor = .systemBackground
        scnView.autoenablesDefaultLighting = true
        scnView.allowsCameraControl = true
        
        // Load the scene
        do {
            let scene = try SCNScene(url: usdzURL, options: nil)
            scnView.scene = scene
            
            // Index the nodes for faster updates
            context.coordinator.indexNodes(scene: scene)
            
        } catch {
            print("❌ Failed to load USDZ: \(error)")
        }
        
        return scnView
    }
    
    func updateUIView(_ uiView: SCNView, context: Context) {
        guard let frame = currentFrame else { return }
        context.coordinator.updateNodes(with: frame)
    }
    
    func makeCoordinator() -> Coordinator {
        Coordinator()
    }
    
    class Coordinator {
        var nodeMap: [String: SCNNode] = [:]
        var initialLocalTransforms: [String: simd_float4x4] = [:]
        
        // Name mapping from Python body names to Swift node names
        var pythonToSwiftNameMap: [String: String] = [:]
        var nameMappingInitialized = false
        
        func indexNodes(scene: SCNScene) {
            let rootNode = scene.rootNode
            
            // Recursive function to index nodes
            func traverse(_ node: SCNNode) {
                if let name = node.name, !name.isEmpty {
                    nodeMap[name] = node
                    // Store initial LOCAL transform (relative to parent)
                    initialLocalTransforms[name] = simd_float4x4(node.transform)
                }
                
                for child in node.childNodes {
                    traverse(child)
                }
            }
            
            traverse(rootNode)
            print("✅ [SimulationViewer3D] Indexed \(nodeMap.count) nodes from USDZ")
            print("   Node names: \(Array(nodeMap.keys).sorted().prefix(20))...")
        }
        
        func updateNodes(with frame: SimulationFrame) {
            // Initialize name mapping on first frame using shared utilities
            if !nameMappingInitialized && !frame.poses.isEmpty {
                let pythonNames = Array(frame.poses.keys)
                let swiftNames = Array(nodeMap.keys)
                pythonToSwiftNameMap = MuJoCoTransformUtils.initializeNameMapping(
                    pythonNames: pythonNames,
                    swiftNames: swiftNames
                )
                nameMappingInitialized = true
                
                // Debug: Log name mapping
                print("🔤 [SimulationViewer3D] Name mapping initialized:")
                print("   Python names: \(pythonNames.sorted().prefix(10))...")
                for (py, swift) in pythonToSwiftNameMap.sorted(by: { $0.key < $1.key }).prefix(10) {
                    let found = nodeMap[swift] != nil ? "✓" : "✗"
                    print("   \(found) '\(py)' → '\(swift)'")
                }
            }
            
            // Collect all transforms first
            var transformsToApply: [(node: SCNNode, swiftName: String, data: [Float])] = []
            
            for (pyName, data) in frame.poses {
                let swiftName = pythonToSwiftNameMap[pyName] ?? pyName
                guard let node = nodeMap[swiftName],
                      initialLocalTransforms[swiftName] != nil,
                      data.count >= 7 else { continue }
                transformsToApply.append((node, swiftName, data))
            }
            
            // Sort by depth (parents first) - this ensures parent transforms are applied
            // before children, so children compute correct relative positions
            transformsToApply.sort { lhs, rhs in
                depth(of: lhs.node) < depth(of: rhs.node)
            }
            
            // Apply in depth order
            for (node, swiftName, data) in transformsToApply {
                // Compute the desired WORLD transform (MuJoCo provides world poses)
                let desiredWorld = MuJoCoTransformUtils.computeWorldTransform(values: data)
                
                // In SceneKit, node.transform is LOCAL (relative to parent)
                // We need to convert world transform to local transform
                if let parentNode = node.parent {
                    // Get parent's CURRENT world transform (after its transform may have been updated)
                    let parentWorld = simd_float4x4(parentNode.worldTransform)
                    // Compute required local transform: local = inverse(parentWorld) * desiredWorld
                    let parentWorldInverse = simd_inverse(parentWorld)
                    let localTransform = parentWorldInverse * desiredWorld
                    node.transform = SCNMatrix4(localTransform)
                } else {
                    // No parent, world = local
                    node.transform = SCNMatrix4(desiredWorld)
                }
            }
        }
        
        /// Calculate node depth in scene graph
        private func depth(of node: SCNNode) -> Int {
            var d = 0
            var current = node.parent
            while current != nil {
                d += 1
                current = current?.parent
            }
            return d
        }
    }
}

