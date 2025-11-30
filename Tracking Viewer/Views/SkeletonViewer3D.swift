//
//  SkeletonViewer3D.swift
//  Tracking Viewer
//
//  Created on 11/29/25.
//

import SwiftUI
import SceneKit

/// 3D Skeleton viewer using SceneKit
struct SkeletonViewer3D: UIViewRepresentable {
    let leftHand: HandJointData?
    let rightHand: HandJointData?
    let headMatrix: [Float]?
    var followHead: Bool = true
    
    func makeUIView(context: Context) -> SCNView {
        let sceneView = SCNView()
        sceneView.scene = SCNScene()
        sceneView.backgroundColor = UIColor(white: 0.1, alpha: 1.0)
        sceneView.allowsCameraControl = true
        sceneView.autoenablesDefaultLighting = true
        
        // Setup camera
        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        cameraNode.position = SCNVector3(0, 1.5, 2.0)
        cameraNode.look(at: SCNVector3(0, 1.0, 0))
        cameraNode.camera?.zNear = 0.01
        cameraNode.camera?.zFar = 100
        sceneView.scene?.rootNode.addChildNode(cameraNode)
        context.coordinator.cameraNode = cameraNode
        
        // Add ambient light
        let ambientLight = SCNNode()
        ambientLight.light = SCNLight()
        ambientLight.light?.type = .ambient
        ambientLight.light?.intensity = 500
        sceneView.scene?.rootNode.addChildNode(ambientLight)
        
        // Add directional light
        let directionalLight = SCNNode()
        directionalLight.light = SCNLight()
        directionalLight.light?.type = .directional
        directionalLight.light?.intensity = 1000
        directionalLight.position = SCNVector3(0, 5, 5)
        directionalLight.look(at: SCNVector3(0, 0, 0))
        sceneView.scene?.rootNode.addChildNode(directionalLight)
        
        // Add grid floor
        addGridFloor(to: sceneView.scene!)
        
        // Add coordinate axes for reference
        addCoordinateAxes(to: sceneView.scene!)
        
        // Store reference for updates
        context.coordinator.sceneView = sceneView
        context.coordinator.leftHandNode = SCNNode()
        context.coordinator.rightHandNode = SCNNode()
        context.coordinator.headNode = SCNNode()
        
        sceneView.scene?.rootNode.addChildNode(context.coordinator.leftHandNode!)
        sceneView.scene?.rootNode.addChildNode(context.coordinator.rightHandNode!)
        sceneView.scene?.rootNode.addChildNode(context.coordinator.headNode!)
        
        return sceneView
    }
    
    func updateUIView(_ uiView: SCNView, context: Context) {
        // Update left hand
        if let leftHand = leftHand {
            updateHandSkeleton(
                node: context.coordinator.leftHandNode!,
                hand: leftHand,
                color: UIColor.systemBlue
            )
        } else {
            context.coordinator.leftHandNode?.childNodes.forEach { $0.removeFromParentNode() }
        }
        
        // Update right hand
        if let rightHand = rightHand {
            updateHandSkeleton(
                node: context.coordinator.rightHandNode!,
                hand: rightHand,
                color: UIColor.systemGreen
            )
        } else {
            context.coordinator.rightHandNode?.childNodes.forEach { $0.removeFromParentNode() }
        }
        
        // Update head and camera
        if let headMatrix = headMatrix {
            updateHead(
                node: context.coordinator.headNode!,
                matrix: headMatrix
            )
            
            // Follow head with camera
            if followHead, let cameraNode = context.coordinator.cameraNode {
                updateCameraToFollowHead(cameraNode: cameraNode, headMatrix: headMatrix)
            }
        } else {
            context.coordinator.headNode?.childNodes.forEach { $0.removeFromParentNode() }
        }
    }
    
    func makeCoordinator() -> Coordinator {
        Coordinator()
    }
    
    class Coordinator {
        var sceneView: SCNView?
        var cameraNode: SCNNode?
        var leftHandNode: SCNNode?
        var rightHandNode: SCNNode?
        var headNode: SCNNode?
    }
    
    // MARK: - Grid Floor
    
    private func addGridFloor(to scene: SCNScene) {
        let gridSize: Float = 5.0
        let gridSpacing: Float = 0.25
        let lineRadius: CGFloat = 0.001
        
        let gridNode = SCNNode()
        gridNode.name = "gridFloor"
        
        // Create grid lines along X axis
        let numLines = Int(gridSize / gridSpacing) * 2 + 1
        let halfSize = gridSize
        
        for i in 0..<numLines {
            let offset = Float(i) * gridSpacing - halfSize
            
            // Line parallel to Z axis
            let lineZ = SCNCylinder(radius: lineRadius, height: CGFloat(gridSize * 2))
            lineZ.firstMaterial?.diffuse.contents = UIColor.gray.withAlphaComponent(0.5)
            let lineZNode = SCNNode(geometry: lineZ)
            lineZNode.position = SCNVector3(offset, 0, 0)
            lineZNode.eulerAngles = SCNVector3(Float.pi/2, 0, 0)
            gridNode.addChildNode(lineZNode)
            
            // Line parallel to X axis
            let lineX = SCNCylinder(radius: lineRadius, height: CGFloat(gridSize * 2))
            lineX.firstMaterial?.diffuse.contents = UIColor.gray.withAlphaComponent(0.5)
            let lineXNode = SCNNode(geometry: lineX)
            lineXNode.position = SCNVector3(0, 0, offset)
            lineXNode.eulerAngles = SCNVector3(0, 0, Float.pi/2)
            gridNode.addChildNode(lineXNode)
        }
        
        // Add a semi-transparent floor plane
        let floorGeometry = SCNPlane(width: CGFloat(gridSize * 2), height: CGFloat(gridSize * 2))
        floorGeometry.firstMaterial?.diffuse.contents = UIColor.darkGray.withAlphaComponent(0.3)
        floorGeometry.firstMaterial?.isDoubleSided = true
        let floorNode = SCNNode(geometry: floorGeometry)
        floorNode.eulerAngles = SCNVector3(-Float.pi/2, 0, 0)
        floorNode.position = SCNVector3(0, -0.001, 0) // Slightly below grid lines
        gridNode.addChildNode(floorNode)
        
        scene.rootNode.addChildNode(gridNode)
    }
    
    private func addCoordinateAxes(to scene: SCNScene) {
        let axisLength: Float = 0.3
        let axisRadius: CGFloat = 0.003
        
        // X axis (red)
        let xAxis = SCNCylinder(radius: axisRadius, height: CGFloat(axisLength))
        xAxis.firstMaterial?.diffuse.contents = UIColor.red
        let xNode = SCNNode(geometry: xAxis)
        xNode.position = SCNVector3(axisLength/2, 0, 0)
        xNode.eulerAngles = SCNVector3(0, 0, Float.pi/2)
        scene.rootNode.addChildNode(xNode)
        
        // X label
        let xLabel = createAxisLabel("X", color: .red)
        xLabel.position = SCNVector3(axisLength + 0.05, 0, 0)
        scene.rootNode.addChildNode(xLabel)
        
        // Y axis (green)
        let yAxis = SCNCylinder(radius: axisRadius, height: CGFloat(axisLength))
        yAxis.firstMaterial?.diffuse.contents = UIColor.green
        let yNode = SCNNode(geometry: yAxis)
        yNode.position = SCNVector3(0, axisLength/2, 0)
        scene.rootNode.addChildNode(yNode)
        
        // Y label
        let yLabel = createAxisLabel("Y", color: .green)
        yLabel.position = SCNVector3(0, axisLength + 0.05, 0)
        scene.rootNode.addChildNode(yLabel)
        
        // Z axis (blue)
        let zAxis = SCNCylinder(radius: axisRadius, height: CGFloat(axisLength))
        zAxis.firstMaterial?.diffuse.contents = UIColor.blue
        let zNode = SCNNode(geometry: zAxis)
        zNode.position = SCNVector3(0, 0, axisLength/2)
        zNode.eulerAngles = SCNVector3(Float.pi/2, 0, 0)
        scene.rootNode.addChildNode(zNode)
        
        // Z label
        let zLabel = createAxisLabel("Z", color: .blue)
        zLabel.position = SCNVector3(0, 0, axisLength + 0.05)
        scene.rootNode.addChildNode(zLabel)
    }
    
    private func createAxisLabel(_ text: String, color: UIColor) -> SCNNode {
        let textGeometry = SCNText(string: text, extrusionDepth: 0.01)
        textGeometry.font = UIFont.boldSystemFont(ofSize: 0.05)
        textGeometry.firstMaterial?.diffuse.contents = color
        let textNode = SCNNode(geometry: textGeometry)
        textNode.scale = SCNVector3(0.5, 0.5, 0.5)
        
        // Add billboard constraint so text always faces camera
        let billboardConstraint = SCNBillboardConstraint()
        billboardConstraint.freeAxes = .all
        textNode.constraints = [billboardConstraint]
        
        return textNode
    }
    
    // MARK: - Camera Follow
    
    private func updateCameraToFollowHead(cameraNode: SCNNode, headMatrix: [Float]) {
        guard headMatrix.count >= 16 else { return }
        
        // Extract head position from matrix
        let headX = headMatrix[12]
        let headY = headMatrix[13]
        let headZ = headMatrix[14]
        
        // Position camera behind and above the head
        let cameraOffset = SCNVector3(0, 0.3, 0.8) // Behind and above
        
        // Smoothly interpolate camera position
        let targetPosition = SCNVector3(
            headX + cameraOffset.x,
            headY + cameraOffset.y,
            headZ + cameraOffset.z
        )
        
        // Smooth camera movement
        SCNTransaction.begin()
        SCNTransaction.animationDuration = 0.1
        cameraNode.position = targetPosition
        cameraNode.look(at: SCNVector3(headX, headY, headZ))
        SCNTransaction.commit()
    }
    
    // MARK: - Hand Skeleton
    
    private func updateHandSkeleton(node: SCNNode, hand: HandJointData, color: UIColor) {
        // Clear existing children
        node.childNodes.forEach { $0.removeFromParentNode() }
        
        // Get world-space positions (wrist @ finger_joint for each joint)
        let positions = hand.worldJointPositions
        guard positions.count >= 21 else { return }
        
        // Draw joints as spheres
        for (index, position) in positions.prefix(21).enumerated() {
            let sphere = SCNSphere(radius: 0.005)
            sphere.firstMaterial?.diffuse.contents = index == 0 ? UIColor.white : color
            let jointNode = SCNNode(geometry: sphere)
            jointNode.position = SCNVector3(position.x, position.y, position.z)
            node.addChildNode(jointNode)
        }
        
        // Draw bones
        for (startIdx, endIdx) in HandSkeleton.fingerConnections {
            guard startIdx < positions.count, endIdx < positions.count else { continue }
            
            let start = positions[startIdx]
            let end = positions[endIdx]
            
            let boneNode = createBone(from: start, to: end, color: color)
            node.addChildNode(boneNode)
        }
    }
    
    private func createBone(from start: SIMD3<Float>, to end: SIMD3<Float>, color: UIColor) -> SCNNode {
        let vector = end - start
        let length = simd_length(vector)
        
        let cylinder = SCNCylinder(radius: 0.003, height: CGFloat(length))
        cylinder.firstMaterial?.diffuse.contents = color.withAlphaComponent(0.8)
        
        let node = SCNNode(geometry: cylinder)
        
        // Position at midpoint
        let midpoint = (start + end) / 2
        node.position = SCNVector3(midpoint.x, midpoint.y, midpoint.z)
        
        // Orient along the bone direction
        let direction = simd_normalize(vector)
        let up = SIMD3<Float>(0, 1, 0)
        
        if abs(simd_dot(direction, up)) < 0.999 {
            let axis = simd_normalize(simd_cross(up, direction))
            let angle = acos(simd_dot(up, direction))
            node.rotation = SCNVector4(axis.x, axis.y, axis.z, angle)
        } else if direction.y < 0 {
            node.eulerAngles = SCNVector3(Float.pi, 0, 0)
        }
        
        return node
    }
    
    private func updateHead(node: SCNNode, matrix: [Float]) {
        node.childNodes.forEach { $0.removeFromParentNode() }
        
        guard matrix.count >= 16 else { return }
        
        // Create head representation (simple box/pyramid)
        let headGeometry = SCNBox(width: 0.15, height: 0.2, length: 0.15, chamferRadius: 0.02)
        headGeometry.firstMaterial?.diffuse.contents = UIColor.systemOrange.withAlphaComponent(0.7)
        
        let headNode = SCNNode(geometry: headGeometry)
        
        // Apply transform from matrix (column-major)
        let transform = SCNMatrix4(
            m11: matrix[0], m12: matrix[1], m13: matrix[2], m14: matrix[3],
            m21: matrix[4], m22: matrix[5], m23: matrix[6], m24: matrix[7],
            m31: matrix[8], m32: matrix[9], m33: matrix[10], m34: matrix[11],
            m41: matrix[12], m42: matrix[13], m43: matrix[14], m44: matrix[15]
        )
        headNode.transform = transform
        
        // Add nose indicator for direction
        let nose = SCNCone(topRadius: 0, bottomRadius: 0.02, height: 0.05)
        nose.firstMaterial?.diffuse.contents = UIColor.systemYellow
        let noseNode = SCNNode(geometry: nose)
        noseNode.position = SCNVector3(0, 0, 0.1)
        noseNode.eulerAngles = SCNVector3(Float.pi/2, 0, 0)
        headNode.addChildNode(noseNode)
        
        node.addChildNode(headNode)
    }
}

#Preview {
    SkeletonViewer3D(leftHand: nil, rightHand: nil, headMatrix: nil)
        .frame(height: 300)
}
