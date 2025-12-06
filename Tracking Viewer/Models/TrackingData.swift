//
//  TrackingData.swift
//  Tracking Viewer
//
//  Created on 11/29/25.
//

import Foundation
import simd
import Combine

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

/// A single frame of simulation data
struct SimulationFrame: Codable {
    let timestamp: Double
    let poses: [String: [Float]]
    let qpos: [Float]?
    let ctrl: [Float]?
}

/// Hand joint positions for all 27 joints tracked by ARKit HandSkeleton
/// Joint order matches RecordingManager.swift:
///   0: forearmArm (optional)
///   1: forearmWrist (optional)
///   2: wrist
///   3-6: thumb (knuckle, intermediateBase, intermediateTip, tip)
///   7-11: index (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   12-16: middle (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   17-21: ring (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   22-26: little (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///
/// For backward compatibility, forearmArm and forearmWrist are optional
/// (older recordings may not include them)
struct HandJointData: Codable {
    // Forearm joints (optional - may not be tracked or from older recordings)
    let forearmArm: [Float]?    // 4x4 matrix flattened (index 0)
    let forearmWrist: [Float]?  // 4x4 matrix flattened (index 1)
    
    let wrist: [Float]  // 4x4 matrix flattened (index 2)
    
    // Thumb (4 joints, indices 3-6) - no metacarpal for thumb
    let thumbKnuckle: [Float]
    let thumbIntermediateBase: [Float]
    let thumbIntermediateTip: [Float]
    let thumbTip: [Float]
    
    // Index finger (5 joints, indices 7-11)
    let indexMetacarpal: [Float]
    let indexKnuckle: [Float]
    let indexIntermediateBase: [Float]
    let indexIntermediateTip: [Float]
    let indexTip: [Float]
    
    // Middle finger (5 joints, indices 12-16)
    let middleMetacarpal: [Float]
    let middleKnuckle: [Float]
    let middleIntermediateBase: [Float]
    let middleIntermediateTip: [Float]
    let middleTip: [Float]
    
    // Ring finger (5 joints, indices 17-21)
    let ringMetacarpal: [Float]
    let ringKnuckle: [Float]
    let ringIntermediateBase: [Float]
    let ringIntermediateTip: [Float]
    let ringTip: [Float]
    
    // Little finger (5 joints, indices 22-26)
    let littleMetacarpal: [Float]
    let littleKnuckle: [Float]
    let littleIntermediateBase: [Float]
    let littleIntermediateTip: [Float]
    let littleTip: [Float]
    
    /// Get all joint matrices as an array (relative to wrist)
    /// Order: forearmArm, forearmWrist, wrist, thumb(4), index(5), middle(5), ring(5), little(5) = 27 joints
    /// For backward compatibility, forearm joints are included as nil arrays if not available
    var allJointMatricesRelative: [[Float]?] {
        return [
            forearmArm,      // 0
            forearmWrist,    // 1
            wrist,           // 2
            // Thumb (3-6)
            thumbKnuckle, thumbIntermediateBase, thumbIntermediateTip, thumbTip,
            // Index (7-11)
            indexMetacarpal, indexKnuckle, indexIntermediateBase, indexIntermediateTip, indexTip,
            // Middle (12-16)
            middleMetacarpal, middleKnuckle, middleIntermediateBase, middleIntermediateTip, middleTip,
            // Ring (17-21)
            ringMetacarpal, ringKnuckle, ringIntermediateBase, ringIntermediateTip, ringTip,
            // Little (22-26)
            littleMetacarpal, littleKnuckle, littleIntermediateBase, littleIntermediateTip, littleTip
        ]
    }
    
    /// Get all joint matrices as an array (relative to wrist) - legacy 25 joint format (no forearm)
    /// Order: wrist, thumb(4), index(5), middle(5), ring(5), little(5) = 25 joints
    var allJointMatricesRelativeLegacy: [[Float]] {
        return [
            wrist,
            // Thumb (1-4)
            thumbKnuckle, thumbIntermediateBase, thumbIntermediateTip, thumbTip,
            // Index (5-9)
            indexMetacarpal, indexKnuckle, indexIntermediateBase, indexIntermediateTip, indexTip,
            // Middle (10-14)
            middleMetacarpal, middleKnuckle, middleIntermediateBase, middleIntermediateTip, middleTip,
            // Ring (15-19)
            ringMetacarpal, ringKnuckle, ringIntermediateBase, ringIntermediateTip, ringTip,
            // Little (20-24)
            littleMetacarpal, littleKnuckle, littleIntermediateBase, littleIntermediateTip, littleTip
        ]
    }
    
    /// Convert flat array to simd_float4x4 matrix
    static func arrayToMatrix(_ arr: [Float]) -> simd_float4x4 {
        guard arr.count >= 16 else {
            // Return identity matrix if array is too short
            return matrix_identity_float4x4
        }
        
        // Check for NaN or Inf values which would cause rendering issues
        for i in 0..<16 {
            if arr[i].isNaN || arr[i].isInfinite {
                return matrix_identity_float4x4
            }
        }
        
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
    
    /// Get forearm arm matrix (optional)
    var forearmArmMatrix: simd_float4x4? {
        guard let forearmArm = forearmArm else { return nil }
        return Self.arrayToMatrix(forearmArm)
    }
    
    /// Get forearm wrist matrix (optional)
    var forearmWristMatrix: simd_float4x4? {
        guard let forearmWrist = forearmWrist else { return nil }
        return Self.arrayToMatrix(forearmWrist)
    }
    
    /// Whether this data includes forearm joints
    var hasForearmData: Bool {
        return forearmArm != nil && forearmWrist != nil
    }
    
    /// Get all joint positions in world space (wrist @ joint for each finger joint)
    /// The wrist position is absolute, finger joints are transformed by wrist
    /// Returns 27 positions if forearm data is available, otherwise 25 positions with forearm as nil
    var worldJointPositions: [SIMD3<Float>?] {
        let wristMat = wristMatrix
        let relativeMatrices = allJointMatricesRelative
        
        var positions: [SIMD3<Float>?] = []
        
        for (index, jointArr) in relativeMatrices.enumerated() {
            guard let arr = jointArr else {
                // Forearm joint not tracked
                positions.append(nil)
                continue
            }
            
            let jointMat = Self.arrayToMatrix(arr)
            
            if index == 2 {
                // Wrist (index 2) is already in world space
                positions.append(Self.positionFromMatrix(wristMat))
            } else if index < 2 {
                // Forearm joints (0, 1): transform by wrist
                let worldMat = wristMat * jointMat
                positions.append(Self.positionFromMatrix(worldMat))
            } else {
                // Finger joints: world_pos = wrist @ local_joint
                let worldMat = wristMat * jointMat
                positions.append(Self.positionFromMatrix(worldMat))
            }
        }
        
        return positions
    }
    
    /// Get all joint positions in world space (legacy 25 joint format - no forearm)
    var worldJointPositionsLegacy: [SIMD3<Float>] {
        let wristMat = wristMatrix
        let relativeMatrices = allJointMatricesRelativeLegacy
        
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
    /// Returns 27 matrices if forearm data is available, otherwise matrices with forearm as nil
    var worldJointMatrices: [simd_float4x4?] {
        let wristMat = wristMatrix
        let relativeMatrices = allJointMatricesRelative
        
        var matrices: [simd_float4x4?] = []
        
        for (index, jointArr) in relativeMatrices.enumerated() {
            guard let arr = jointArr else {
                // Forearm joint not tracked
                matrices.append(nil)
                continue
            }
            
            let jointMat = Self.arrayToMatrix(arr)
            
            if index == 2 {
                // Wrist (index 2) is already in world space
                matrices.append(wristMat)
            } else if index < 2 {
                // Forearm joints (0, 1): transform by wrist
                matrices.append(wristMat * jointMat)
            } else {
                // Finger joints: world = wrist @ local
                matrices.append(wristMat * jointMat)
            }
        }
        
        return matrices
    }
    
    /// Get all joint matrices in world space (legacy 25 joint format - no forearm)
    var worldJointMatricesLegacy: [simd_float4x4] {
        let wristMat = wristMatrix
        let relativeMatrices = allJointMatricesRelativeLegacy
        
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

/// Simulation data loader
class SimulationDataLoader {
    
    /// Load all simulation frames from a recording
    static func loadSimulationData(from url: URL) async throws -> [SimulationFrame] {
        // Download from iCloud if needed
        _ = await RecordingsManager.shared.downloadFromiCloud(url)
        
        let data = try Data(contentsOf: url)
        guard let content = String(data: data, encoding: .utf8) else {
            throw TrackingError.invalidData
        }
        
        var frames: [SimulationFrame] = []
        let decoder = JSONDecoder()
        
        for line in content.components(separatedBy: .newlines) {
            guard !line.isEmpty else { continue }
            if let lineData = line.data(using: .utf8) {
                do {
                    let frame = try decoder.decode(SimulationFrame.self, from: lineData)
                    frames.append(frame)
                } catch {
                    print("⚠️ Failed to decode simulation frame: \(error)")
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
/// Based on ARKit HandSkeleton joint structure
/// 27 joints total per hand:
///   0: forearmArm (optional)
///   1: forearmWrist (optional)
///   2: wrist
///   3-6: thumb (knuckle, intermediateBase, intermediateTip, tip)
///   7-11: index (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   12-16: middle (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   17-21: ring (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   22-26: little (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
struct HandSkeleton {
    /// Forearm connections: forearmArm(0) -> forearmWrist(1) -> wrist(2)
    /// These are optional and only rendered if forearm data is available
    static let forearmConnections: [(Int, Int)] = [
        (0, 1), (1, 2)  // forearmArm -> forearmWrist -> wrist
    ]
    
    /// Finger connections: pairs of joint indices to draw bones (27-joint indexing)
    /// Based on ARKit HandSkeleton structure
    static let fingerConnections: [(Int, Int)] = [
        // Thumb: wrist(2) -> knuckle(3) -> intermediateBase(4) -> intermediateTip(5) -> tip(6)
        (2, 3), (3, 4), (4, 5), (5, 6),
        // Index: wrist(2) -> metacarpal(7) -> knuckle(8) -> intermediateBase(9) -> intermediateTip(10) -> tip(11)
        (2, 7), (7, 8), (8, 9), (9, 10), (10, 11),
        // Middle: wrist(2) -> metacarpal(12) -> knuckle(13) -> intermediateBase(14) -> intermediateTip(15) -> tip(16)
        (2, 12), (12, 13), (13, 14), (14, 15), (15, 16),
        // Ring: wrist(2) -> metacarpal(17) -> knuckle(18) -> intermediateBase(19) -> intermediateTip(20) -> tip(21)
        (2, 17), (17, 18), (18, 19), (19, 20), (20, 21),
        // Little: wrist(2) -> metacarpal(22) -> knuckle(23) -> intermediateBase(24) -> intermediateTip(25) -> tip(26)
        (2, 22), (22, 23), (23, 24), (24, 25), (25, 26)
    ]
    
    /// Legacy finger connections for 25-joint format (no forearm, wrist at index 0)
    static let fingerConnectionsLegacy: [(Int, Int)] = [
        // Thumb: wrist(0) -> knuckle(1) -> intermediateBase(2) -> intermediateTip(3) -> tip(4)
        (0, 1), (1, 2), (2, 3), (3, 4),
        // Index: wrist(0) -> metacarpal(5) -> knuckle(6) -> intermediateBase(7) -> intermediateTip(8) -> tip(9)
        (0, 5), (5, 6), (6, 7), (7, 8), (8, 9),
        // Middle: wrist(0) -> metacarpal(10) -> knuckle(11) -> intermediateBase(12) -> intermediateTip(13) -> tip(14)
        (0, 10), (10, 11), (11, 12), (12, 13), (13, 14),
        // Ring: wrist(0) -> metacarpal(15) -> knuckle(16) -> intermediateBase(17) -> intermediateTip(18) -> tip(19)
        (0, 15), (15, 16), (16, 17), (17, 18), (18, 19),
        // Little: wrist(0) -> metacarpal(20) -> knuckle(21) -> intermediateBase(22) -> intermediateTip(23) -> tip(24)
        (0, 20), (20, 21), (21, 22), (22, 23), (23, 24)
    ]
    
    /// Palm connections: connects knuckle joints across the palm (27-joint indexing)
    static let palmConnections: [(Int, Int)] = [
        (8, 13),   // indexKnuckle -> middleKnuckle
        (13, 18),  // middleKnuckle -> ringKnuckle
        (18, 23)   // ringKnuckle -> littleKnuckle
    ]
    
    /// Legacy palm connections for 25-joint format
    static let palmConnectionsLegacy: [(Int, Int)] = [
        (6, 11),   // indexKnuckle -> middleKnuckle
        (11, 16),  // middleKnuckle -> ringKnuckle
        (16, 21)   // ringKnuckle -> littleKnuckle
    ]
    
    /// Joint names for reference (27 joints)
    static let jointNames = [
        "forearmArm", "forearmWrist", "wrist",
        // Thumb (3-6)
        "thumbKnuckle", "thumbIntermediateBase", "thumbIntermediateTip", "thumbTip",
        // Index (7-11)
        "indexMetacarpal", "indexKnuckle", "indexIntermediateBase", "indexIntermediateTip", "indexTip",
        // Middle (12-16)
        "middleMetacarpal", "middleKnuckle", "middleIntermediateBase", "middleIntermediateTip", "middleTip",
        // Ring (17-21)
        "ringMetacarpal", "ringKnuckle", "ringIntermediateBase", "ringIntermediateTip", "ringTip",
        // Little (22-26)
        "littleMetacarpal", "littleKnuckle", "littleIntermediateBase", "littleIntermediateTip", "littleTip"
    ]
    
    /// Legacy joint names (25 joints - no forearm)
    static let jointNamesLegacy = [
        "wrist",
        // Thumb (1-4)
        "thumbKnuckle", "thumbIntermediateBase", "thumbIntermediateTip", "thumbTip",
        // Index (5-9)
        "indexMetacarpal", "indexKnuckle", "indexIntermediateBase", "indexIntermediateTip", "indexTip",
        // Middle (10-14)
        "middleMetacarpal", "middleKnuckle", "middleIntermediateBase", "middleIntermediateTip", "middleTip",
        // Ring (15-19)
        "ringMetacarpal", "ringKnuckle", "ringIntermediateBase", "ringIntermediateTip", "ringTip",
        // Little (20-24)
        "littleMetacarpal", "littleKnuckle", "littleIntermediateBase", "littleIntermediateTip", "littleTip"
    ]
    
    /// Fingertip joint names for highlighting
    static let fingertipNames = ["thumbTip", "indexTip", "middleTip", "ringTip", "littleTip"]
    
    /// Fingertip indices in the 27-joint array
    static let fingertipIndices: Set<Int> = [6, 11, 16, 21, 26]
    
    /// Fingertip indices in the legacy 25-joint array
    static let fingertipIndicesLegacy: Set<Int> = [4, 9, 14, 19, 24]
    
    /// Forearm joint indices (optional)
    static let forearmIndices: Set<Int> = [0, 1]
    
    /// Wrist index in 27-joint format
    static let wristIndex: Int = 2
    
    /// Wrist index in legacy 25-joint format
    static let wristIndexLegacy: Int = 0
}
