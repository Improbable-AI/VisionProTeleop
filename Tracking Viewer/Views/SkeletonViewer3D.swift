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
        cameraNode.name = "mainCamera"
        cameraNode.camera = SCNCamera()
        cameraNode.position = SCNVector3(0, 1.5, 2.0)
        cameraNode.look(at: SCNVector3(0, 1.0, 0))
        cameraNode.camera?.zNear = 0.01
        cameraNode.camera?.zFar = 100
        sceneView.scene?.rootNode.addChildNode(cameraNode)
        sceneView.pointOfView = cameraNode  // Explicitly set as point of view
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
        // Safety check - ensure coordinator nodes exist
        guard let leftHandNode = context.coordinator.leftHandNode,
              let rightHandNode = context.coordinator.rightHandNode,
              let headNode = context.coordinator.headNode else {
            print("⚠️ [SkeletonViewer3D] Coordinator nodes not initialized")
            return
        }
        
        // Update left hand
        if let leftHand = leftHand {
            updateHandSkeleton(
                node: leftHandNode,
                hand: leftHand,
                color: UIColor(red: 0.4, green: 0.6, blue: 1.0, alpha: 1.0)  // Blue like Python LEFT_HAND_COLOR
            )
        } else {
            leftHandNode.childNodes.forEach { $0.removeFromParentNode() }
        }
        
        // Update right hand
        if let rightHand = rightHand {
            updateHandSkeleton(
                node: rightHandNode,
                hand: rightHand,
                color: UIColor(red: 1.0, green: 0.5, blue: 0.4, alpha: 1.0)  // Red/Orange like Python
            )
        } else {
            rightHandNode.childNodes.forEach { $0.removeFromParentNode() }
        }
        
        // Update head and camera
        if let headMatrix = headMatrix {
            updateHead(
                node: headNode,
                matrix: headMatrix
            )
            
            // Follow head with camera
            if followHead, let cameraNode = context.coordinator.cameraNode, let sceneView = context.coordinator.sceneView {
                // Reset point of view to our camera node when following is enabled
                // This ensures we regain control after user interaction
                if sceneView.pointOfView != cameraNode {
                    sceneView.pointOfView = cameraNode
                }
                updateCameraToFollowHead(cameraNode: cameraNode, headMatrix: headMatrix)
            }
        } else {
            headNode.childNodes.forEach { $0.removeFromParentNode() }
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
        
        // Extract head position from matrix (column-major: translation at indices 12, 13, 14)
        let headPos = SIMD3<Float>(headMatrix[12], headMatrix[13], headMatrix[14])
        
        // Extract rotation matrix columns (column-major order)
        // Column 0 (X-axis): indices 0, 1, 2 - points right
        // Column 1 (Y-axis): indices 4, 5, 6 - points up
        // Column 2 (Z-axis): indices 8, 9, 10 - points backward (in ARKit, -Z is forward)
        let headXAxis = SIMD3<Float>(headMatrix[0], headMatrix[1], headMatrix[2])
        let headYAxis = SIMD3<Float>(headMatrix[4], headMatrix[5], headMatrix[6])
        let headZAxis = SIMD3<Float>(headMatrix[8], headMatrix[9], headMatrix[10])
        
        // Camera position: behind and slightly above the head
        // In ARKit, -Z is forward, so +Z is backward (behind the head)
        let followDistance: Float = 0.6
        let heightOffset: Float = 0.2
        
        // Position camera behind head (along head's +Z) and above (along head's +Y)
        let cameraPos = headPos + headZAxis * followDistance + headYAxis * heightOffset
        
        // The camera should look at a point slightly in front of the head
        // This is along the head's -Z direction
        let lookAtPos = headPos - headZAxis * 0.3
        
        // Calculate camera orientation to match head's horizontal orientation
        // We want the camera's "up" to align with the head's Y axis
        let cameraUp = headYAxis
        
        // Smooth camera movement
        SCNTransaction.begin()
        SCNTransaction.animationDuration = 0.15
        
        cameraNode.position = SCNVector3(cameraPos.x, cameraPos.y, cameraPos.z)
        
        // Use look(at:up:localFront:) for proper orientation matching
        cameraNode.look(at: SCNVector3(lookAtPos.x, lookAtPos.y, lookAtPos.z),
                        up: SCNVector3(cameraUp.x, cameraUp.y, cameraUp.z),
                        localFront: SCNVector3(0, 0, -1))
        
        SCNTransaction.commit()
    }
    
    // MARK: - Hand Skeleton
    
    /// Fingertip indices for highlighting (ARKit 27-joint structure)
    /// thumbTip=6, indexTip=11, middleTip=16, ringTip=21, littleTip=26
    private static let fingertipIndices27: Set<Int> = [6, 11, 16, 21, 26]
    
    /// Fingertip indices for legacy 25-joint structure
    /// thumbTip=4, indexTip=9, middleTip=14, ringTip=19, littleTip=24
    private static let fingertipIndices25: Set<Int> = [4, 9, 14, 19, 24]
    
    /// Thumb tip and index tip indices for pinch detection (27-joint)
    private static let thumbTipIdx27 = 6
    private static let indexTipIdx27 = 11
    
    /// Thumb tip and index tip indices for legacy pinch detection (25-joint)
    private static let thumbTipIdx25 = 4
    private static let indexTipIdx25 = 9
    
    /// Forearm joint indices
    private static let forearmArmIdx = 0
    private static let forearmWristIdx = 1
    private static let wristIdx27 = 2
    
    /// Base joint size (matches Python joint_size default of 0.015)
    private static let baseJointSize: CGFloat = 0.015
    
    /// Pinch color for highlighting (matches Python PINCH_COLOR)
    private static let pinchColor = UIColor(red: 0.2, green: 1.0, blue: 0.2, alpha: 1.0)  // Bright green
    
    /// Forearm color (slightly darker than hand color)
    private static let forearmColor = UIColor(red: 0.5, green: 0.5, blue: 0.6, alpha: 1.0)  // Gray-blue
    
    private func updateHandSkeleton(node: SCNNode, hand: HandJointData, color: UIColor) {
        // Clear existing children
        node.childNodes.forEach { $0.removeFromParentNode() }
        
        // Check if this recording has forearm data
        let hasForearmData = hand.hasForearmData
        
        if hasForearmData {
            // Use 27-joint format with forearm
            updateHandSkeleton27(node: node, hand: hand, color: color)
        } else {
            // Use legacy 25-joint format (backward compatibility)
            updateHandSkeleton25(node: node, hand: hand, color: color)
        }
    }
    
    /// Update hand skeleton with 27-joint format (includes forearm)
    private func updateHandSkeleton27(node: SCNNode, hand: HandJointData, color: UIColor) {
        // Get world-space positions (27 joints)
        let positions = hand.worldJointPositions
        
        // Count valid positions (non-nil)
        var validPositions = 0
        for pos in positions {
            if let p = pos, !p.x.isNaN && !p.y.isNaN && !p.z.isNaN &&
               !p.x.isInfinite && !p.y.isInfinite && !p.z.isInfinite {
                if p.x != 0 || p.y != 0 || p.z != 0 {
                    validPositions += 1
                }
            }
        }
        
        if validPositions < 25 {
            print("⚠️ [SkeletonViewer3D] Only \(validPositions)/27 positions are valid")
            return
        }
        
        guard positions.count >= 27 else {
            print("⚠️ [SkeletonViewer3D] Not enough positions: \(positions.count) < 27")
            return
        }
        
        // Compute pinch distance for highlighting
        var isPinching = false
        if let thumbTip = positions[Self.thumbTipIdx27], let indexTip = positions[Self.indexTipIdx27] {
            let pinchDist = simd_distance(thumbTip, indexTip)
            isPinching = pinchDist < 0.02
        }
        
        // Draw joints as spheres (all 27 joints)
        for (index, positionOpt) in positions.enumerated() {
            guard let position = positionOpt else { continue }  // Skip nil forearm joints
            
            let isFingertip = Self.fingertipIndices27.contains(index)
            let isForearm = (index == Self.forearmArmIdx || index == Self.forearmWristIdx)
            let isWrist = (index == Self.wristIdx27)
            
            // Fingertips are larger (1.3x), forearm joints slightly larger too
            let jointSize: CGFloat
            if isFingertip {
                jointSize = Self.baseJointSize * 1.3
            } else if isForearm {
                jointSize = Self.baseJointSize * 1.2
            } else {
                jointSize = Self.baseJointSize
            }
            
            let sphere = SCNSphere(radius: jointSize)
            
            // Color: wrist=white, forearm=gray, fingertips during pinch=green, others=hand color
            let jointColor: UIColor
            if isWrist {
                jointColor = UIColor.white
            } else if isForearm {
                jointColor = Self.forearmColor
            } else if isPinching && (index == Self.thumbTipIdx27 || index == Self.indexTipIdx27) {
                jointColor = Self.pinchColor
            } else {
                jointColor = color
            }
            
            sphere.firstMaterial?.diffuse.contents = jointColor
            let jointNode = SCNNode(geometry: sphere)
            jointNode.position = SCNVector3(position.x, position.y, position.z)
            node.addChildNode(jointNode)
        }
        
        // Draw forearm bones (if available)
        for (startIdx, endIdx) in HandSkeleton.forearmConnections {
            guard startIdx < positions.count, endIdx < positions.count,
                  let start = positions[startIdx], let end = positions[endIdx] else { continue }
            
            let boneNode = createBone(from: start, to: end, color: Self.forearmColor)
            node.addChildNode(boneNode)
        }
        
        // Draw finger bones (27-joint connections)
        for (startIdx, endIdx) in HandSkeleton.fingerConnections {
            guard startIdx < positions.count, endIdx < positions.count,
                  let start = positions[startIdx], let end = positions[endIdx] else { continue }
            
            let boneNode = createBone(from: start, to: end, color: color)
            node.addChildNode(boneNode)
        }
        
        // Draw palm connections (27-joint)
        for (startIdx, endIdx) in HandSkeleton.palmConnections {
            guard startIdx < positions.count, endIdx < positions.count,
                  let start = positions[startIdx], let end = positions[endIdx] else { continue }
            
            let boneNode = createBone(from: start, to: end, color: color)
            node.addChildNode(boneNode)
        }
    }
    
    /// Update hand skeleton with legacy 25-joint format (no forearm)
    private func updateHandSkeleton25(node: SCNNode, hand: HandJointData, color: UIColor) {
        // Validate hand data first
        let jointArrays = hand.allJointMatricesRelativeLegacy
        var validJointCount = 0
        for arr in jointArrays {
            if arr.count >= 16 {
                validJointCount += 1
            }
        }
        
        if validJointCount < 25 {
            print("⚠️ [SkeletonViewer3D] Hand data has only \(validJointCount)/25 valid joint matrices")
        }
        
        // Get world-space positions (25 joints, legacy format)
        let positions = hand.worldJointPositionsLegacy
        
        // Debug: Check if positions seem valid
        var validPositions = 0
        var hasNonZeroPosition = false
        for pos in positions {
            if pos.x != 0 || pos.y != 0 || pos.z != 0 {
                hasNonZeroPosition = true
            }
            if !pos.x.isNaN && !pos.y.isNaN && !pos.z.isNaN &&
               !pos.x.isInfinite && !pos.y.isInfinite && !pos.z.isInfinite {
                validPositions += 1
            }
        }
        
        if !hasNonZeroPosition {
            print("⚠️ [SkeletonViewer3D] All joint positions are at origin - check wrist matrix")
            return
        }
        
        if validPositions < 25 {
            print("⚠️ [SkeletonViewer3D] Only \(validPositions)/25 positions are valid (non-NaN/Inf)")
        }
        
        guard positions.count >= 25 else {
            print("⚠️ [SkeletonViewer3D] Not enough positions: \(positions.count) < 25")
            return
        }
        
        // Compute pinch distance for highlighting
        var isPinching = false
        if Self.thumbTipIdx25 < positions.count && Self.indexTipIdx25 < positions.count {
            let pinchDist = simd_distance(positions[Self.thumbTipIdx25], positions[Self.indexTipIdx25])
            isPinching = pinchDist < 0.02
        }
        
        // Draw joints as spheres (all 25 joints)
        for (index, position) in positions.enumerated() {
            let isFingertip = Self.fingertipIndices25.contains(index)
            
            let jointSize = isFingertip ? Self.baseJointSize * 1.3 : Self.baseJointSize
            let sphere = SCNSphere(radius: jointSize)
            
            let jointColor: UIColor
            if index == 0 {
                jointColor = UIColor.white
            } else if isPinching && (index == Self.thumbTipIdx25 || index == Self.indexTipIdx25) {
                jointColor = Self.pinchColor
            } else {
                jointColor = color
            }
            
            sphere.firstMaterial?.diffuse.contents = jointColor
            let jointNode = SCNNode(geometry: sphere)
            jointNode.position = SCNVector3(position.x, position.y, position.z)
            node.addChildNode(jointNode)
        }
        
        // Draw bones (legacy 25-joint connections)
        for (startIdx, endIdx) in HandSkeleton.fingerConnectionsLegacy {
            guard startIdx < positions.count, endIdx < positions.count else { continue }
            
            let start = positions[startIdx]
            let end = positions[endIdx]
            
            let boneNode = createBone(from: start, to: end, color: color)
            node.addChildNode(boneNode)
        }
        
        // Draw palm connections (legacy 25-joint)
        for (startIdx, endIdx) in HandSkeleton.palmConnectionsLegacy {
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
        
        // Create coordinate frame like Python's add_frame
        // Axes length and radius match Python (0.1, 0.003)
        let axisLength: Float = 0.1
        let axisRadius: CGFloat = 0.003
        
        let headFrameNode = SCNNode()
        
        // Apply transform from matrix (column-major)
        let transform = SCNMatrix4(
            m11: matrix[0], m12: matrix[1], m13: matrix[2], m14: matrix[3],
            m21: matrix[4], m22: matrix[5], m23: matrix[6], m24: matrix[7],
            m31: matrix[8], m32: matrix[9], m33: matrix[10], m34: matrix[11],
            m41: matrix[12], m42: matrix[13], m43: matrix[14], m44: matrix[15]
        )
        headFrameNode.transform = transform
        
        // X axis (red) - points right
        let xAxis = SCNCylinder(radius: axisRadius, height: CGFloat(axisLength))
        xAxis.firstMaterial?.diffuse.contents = UIColor.red
        let xNode = SCNNode(geometry: xAxis)
        xNode.position = SCNVector3(axisLength/2, 0, 0)
        xNode.eulerAngles = SCNVector3(0, 0, Float.pi/2)
        headFrameNode.addChildNode(xNode)
        
        // Y axis (green) - points up
        let yAxis = SCNCylinder(radius: axisRadius, height: CGFloat(axisLength))
        yAxis.firstMaterial?.diffuse.contents = UIColor.green
        let yNode = SCNNode(geometry: yAxis)
        yNode.position = SCNVector3(0, axisLength/2, 0)
        headFrameNode.addChildNode(yNode)
        
        // Z axis (blue) - points backward (in ARKit, -Z is forward)
        let zAxis = SCNCylinder(radius: axisRadius, height: CGFloat(axisLength))
        zAxis.firstMaterial?.diffuse.contents = UIColor.blue
        let zNode = SCNNode(geometry: zAxis)
        zNode.position = SCNVector3(0, 0, axisLength/2)
        zNode.eulerAngles = SCNVector3(Float.pi/2, 0, 0)
        headFrameNode.addChildNode(zNode)
        
        // Add small sphere at origin for visibility
        let originSphere = SCNSphere(radius: 0.008)
        originSphere.firstMaterial?.diffuse.contents = UIColor.white
        let originNode = SCNNode(geometry: originSphere)
        headFrameNode.addChildNode(originNode)
        
        node.addChildNode(headFrameNode)
    }
}

#Preview {
    SkeletonViewer3D(leftHand: nil, rightHand: nil, headMatrix: nil)
        .frame(height: 300)
}
