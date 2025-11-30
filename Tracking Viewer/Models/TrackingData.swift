//
//  TrackingData.swift
//  Tracking Viewer
//
//  Created on 11/29/25.
//

import Foundation
import simd

/// A single frame of recorded tracking data
struct RecordedFrame: Codable {
    let timestamp: Double
    let systemTime: Double
    let headMatrix: [Float]?
    let leftHand: HandJointData?
    let rightHand: HandJointData?
    let videoFrameIndex: Int
    let videoWidth: Int
    let videoHeight: Int
}

/// Hand joint positions for all joints
struct HandJointData: Codable {
    let wrist: [Float]
    let thumbCMC: [Float]
    let thumbMP: [Float]
    let thumbIP: [Float]
    let thumbTip: [Float]
    let indexMCP: [Float]
    let indexPIP: [Float]
    let indexDIP: [Float]
    let indexTip: [Float]
    let middleMCP: [Float]
    let middlePIP: [Float]
    let middleDIP: [Float]
    let middleTip: [Float]
    let ringMCP: [Float]
    let ringPIP: [Float]
    let ringDIP: [Float]
    let ringTip: [Float]
    let littleMCP: [Float]
    let littlePIP: [Float]
    let littleDIP: [Float]
    let littleTip: [Float]
    let forearmWrist: [Float]?
    let forearmArm: [Float]?
    
    /// Get all joint matrices as an array (relative to wrist)
    var allJointMatricesRelative: [[Float]] {
        var joints = [
            wrist, thumbCMC, thumbMP, thumbIP, thumbTip,
            indexMCP, indexPIP, indexDIP, indexTip,
            middleMCP, middlePIP, middleDIP, middleTip,
            ringMCP, ringPIP, ringDIP, ringTip,
            littleMCP, littlePIP, littleDIP, littleTip
        ]
        if let fw = forearmWrist { joints.append(fw) }
        if let fa = forearmArm { joints.append(fa) }
        return joints
    }
    
    /// Convert flat array to simd_float4x4 matrix
    static func arrayToMatrix(_ arr: [Float]) -> simd_float4x4 {
        guard arr.count >= 16 else { return matrix_identity_float4x4 }
        // Column-major order
        return simd_float4x4(
            SIMD4<Float>(arr[0], arr[1], arr[2], arr[3]),
            SIMD4<Float>(arr[4], arr[5], arr[6], arr[7]),
            SIMD4<Float>(arr[8], arr[9], arr[10], arr[11]),
            SIMD4<Float>(arr[12], arr[13], arr[14], arr[15])
        )
    }
    
    /// Get position (x, y, z) from a 4x4 matrix
    static func positionFromMatrix(_ matrix: simd_float4x4) -> SIMD3<Float> {
        return SIMD3<Float>(matrix.columns.3.x, matrix.columns.3.y, matrix.columns.3.z)
    }
    
    /// Get position (x, y, z) from a flat 4x4 matrix array
    static func positionFromArray(_ matrix: [Float]) -> SIMD3<Float> {
        guard matrix.count >= 16 else { return .zero }
        // Column-major: translation is at indices 12, 13, 14
        return SIMD3<Float>(matrix[12], matrix[13], matrix[14])
    }
    
    /// Get wrist matrix
    var wristMatrix: simd_float4x4 {
        Self.arrayToMatrix(wrist)
    }
    
    /// Get all joint positions in world space (wrist @ joint for each finger joint)
    /// The wrist position is absolute, finger joints are transformed by wrist
    var worldJointPositions: [SIMD3<Float>] {
        let wristMat = wristMatrix
        let relativeMatrices = allJointMatricesRelative
        
        var positions: [SIMD3<Float>] = []
        
        for (index, jointArr) in relativeMatrices.enumerated() {
            let jointMat = Self.arrayToMatrix(jointArr)
            
            if index == 0 {
                // Wrist is already in world space
                positions.append(Self.positionFromMatrix(wristMat))
            } else {
                // Finger joints: world_pos = wrist @ local_joint
                let worldMat = wristMat * jointMat
                positions.append(Self.positionFromMatrix(worldMat))
            }
        }
        
        return positions
    }
    
    /// Get all joint matrices in world space
    var worldJointMatrices: [simd_float4x4] {
        let wristMat = wristMatrix
        let relativeMatrices = allJointMatricesRelative
        
        var matrices: [simd_float4x4] = []
        
        for (index, jointArr) in relativeMatrices.enumerated() {
            let jointMat = Self.arrayToMatrix(jointArr)
            
            if index == 0 {
                // Wrist is already in world space
                matrices.append(wristMat)
            } else {
                // Finger joints: world = wrist @ local
                matrices.append(wristMat * jointMat)
            }
        }
        
        return matrices
    }
}

/// Tracking data loader
class TrackingDataLoader {
    
    /// Load all tracking frames from a recording
    static func loadTrackingData(from url: URL) async throws -> [RecordedFrame] {
        // Download from iCloud if needed
        _ = await RecordingsManager.shared.downloadFromiCloud(url)
        
        let data = try Data(contentsOf: url)
        guard let content = String(data: data, encoding: .utf8) else {
            throw TrackingError.invalidData
        }
        
        var frames: [RecordedFrame] = []
        let decoder = JSONDecoder()
        
        for line in content.components(separatedBy: .newlines) {
            guard !line.isEmpty else { continue }
            if let lineData = line.data(using: .utf8) {
                do {
                    let frame = try decoder.decode(RecordedFrame.self, from: lineData)
                    frames.append(frame)
                } catch {
                    print("⚠️ Failed to decode frame: \(error)")
                }
            }
        }
        
        return frames
    }
}

enum TrackingError: Error {
    case invalidData
    case fileNotFound
}

/// Hand skeleton connection definitions for rendering
struct HandSkeleton {
    /// Finger connections: pairs of joint indices to draw bones
    static let fingerConnections: [(Int, Int)] = [
        // Thumb
        (0, 1), (1, 2), (2, 3), (3, 4),
        // Index
        (0, 5), (5, 6), (6, 7), (7, 8),
        // Middle
        (0, 9), (9, 10), (10, 11), (11, 12),
        // Ring
        (0, 13), (13, 14), (14, 15), (15, 16),
        // Little
        (0, 17), (17, 18), (18, 19), (19, 20)
    ]
    
    /// Joint names for reference
    static let jointNames = [
        "wrist",
        "thumbCMC", "thumbMP", "thumbIP", "thumbTip",
        "indexMCP", "indexPIP", "indexDIP", "indexTip",
        "middleMCP", "middlePIP", "middleDIP", "middleTip",
        "ringMCP", "ringPIP", "ringDIP", "ringTip",
        "littleMCP", "littlePIP", "littleDIP", "littleTip"
    ]
}
