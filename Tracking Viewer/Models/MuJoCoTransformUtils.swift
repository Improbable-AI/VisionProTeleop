//
//  MuJoCoTransformUtils.swift
//  Shared utilities for MuJoCo body name mapping and transform computation
//
//  This file can be added to both iOS and visionOS targets for code reuse.
//

import Foundation
import simd

/// Shared utilities for MuJoCo body name mapping and transform computation.
/// These functions are used to match Python body names from MuJoCo to Swift entity/node names
/// and to compute the correct transforms for visualization.
enum MuJoCoTransformUtils {
    
    /// Axis correction quaternion to convert from MuJoCo Z-up to RealityKit/SceneKit Y-up
    static let axisCorrection = simd_quatf(angle: -.pi / 2, axis: SIMD3<Float>(1, 0, 0))
    
    // MARK: - Name Mapping
    
    /// Sanitize a name for fuzzy matching by converting to lowercase and keeping only alphanumeric characters.
    /// - Parameter name: The original name (e.g., "left_finger_link" or "LeftFingerLink")
    /// - Returns: Sanitized name (e.g., "leftfingerlink")
    static func sanitizeName(_ name: String) -> String {
        let lowered = name.lowercased()
        let filtered = lowered.unicodeScalars.filter { CharacterSet.alphanumerics.contains($0) }
        return String(String.UnicodeScalarView(filtered))
    }
    
    /// Find the best Swift name match for a given Python name using fuzzy matching.
    /// Matches if the sanitized Swift name contains the sanitized Python name.
    /// Prefers shorter matches (closer to exact match).
    ///
    /// - Parameters:
    ///   - pyName: The Python body name to match
    ///   - candidates: Array of (original, sanitized) Swift name pairs
    ///   - used: Set of Swift names already used (to avoid duplicate mappings)
    /// - Returns: The best matching Swift name, or nil if no match found
    static func findBestSwiftMatch(
        for pyName: String,
        candidates: [(original: String, sanitized: String)],
        excluding used: Set<String>
    ) -> String? {
        let pySanitized = sanitizeName(pyName)
        var best: (swiftOriginal: String, delta: Int)? = nil
        
        for (original, sanitized) in candidates where !used.contains(original) {
            if sanitized.contains(pySanitized) {
                let delta = max(0, sanitized.count - pySanitized.count)
                if let current = best {
                    // Prefer smaller delta (closer match), or shorter name on tie
                    if delta < current.delta || (delta == current.delta && original.count < current.swiftOriginal.count) {
                        best = (original, delta)
                    }
                } else {
                    best = (original, delta)
                }
            }
        }
        
        return best?.swiftOriginal
    }
    
    /// Initialize a mapping from Python body names to Swift entity/node names.
    /// Uses fuzzy matching to handle naming convention differences.
    ///
    /// - Parameters:
    ///   - pythonNames: Array of body names from Python/MuJoCo
    ///   - swiftNames: Array of entity/node names from USDZ
    /// - Returns: Dictionary mapping Python names to Swift names
    static func initializeNameMapping(
        pythonNames: [String],
        swiftNames: [String]
    ) -> [String: String] {
        let sanitizedSwiftPairs: [(original: String, sanitized: String)] = swiftNames.map {
            ($0, sanitizeName($0))
        }
        var usedSwift: Set<String> = []
        var mapping: [String: String] = [:]
        
        for pyName in pythonNames {
            if let match = findBestSwiftMatch(for: pyName, candidates: sanitizedSwiftPairs, excluding: usedSwift) {
                mapping[pyName] = match
                usedSwift.insert(match)
            } else {
                // Fall back to using the Python name directly
                mapping[pyName] = pyName
            }
        }
        
        return mapping
    }
    
    // MARK: - Transform Computation
    
    /// Compute the WORLD transform from MuJoCo pose data.
    /// Applies axis correction (Z-up to Y-up) only.
    /// MuJoCo already provides world-space poses, no need to multiply by initial transform.
    ///
    /// - Parameters:
    ///   - values: Array of 7 floats [x, y, z, qx, qy, qz, qw] representing position and quaternion rotation
    /// - Returns: The world transform to apply
    static func computeWorldTransform(values: [Float]) -> simd_float4x4 {
        guard values.count >= 7 else { return matrix_identity_float4x4 }
        
        // Parse position and quaternion from MuJoCo (Z-up coordinate system)
        let mjPos = SIMD3<Float>(values[0], values[1], values[2])
        let mjRot = simd_quatf(ix: values[3], iy: values[4], iz: values[5], r: values[6])
        
        // Build MuJoCo world-space transform (Z-up)
        var mjWorldTransform = matrix_identity_float4x4
        mjWorldTransform = simd_mul(matrix_float4x4(mjRot), mjWorldTransform)
        mjWorldTransform.columns.3 = SIMD4<Float>(mjPos, 1.0)
        
        // Apply axis correction: convert Z-up to Y-up
        let rkPos = axisCorrection.act(SIMD3<Float>(
            mjWorldTransform.columns.3.x,
            mjWorldTransform.columns.3.y,
            mjWorldTransform.columns.3.z
        ))
        let mjRotFromMatrix = simd_quatf(mjWorldTransform)
        let rkRot = axisCorrection * mjRotFromMatrix
        
        // Build RealityKit/SceneKit world transform (Y-up)
        var rkWorldTransform = matrix_identity_float4x4
        rkWorldTransform = simd_mul(matrix_float4x4(rkRot), rkWorldTransform)
        rkWorldTransform.columns.3 = SIMD4<Float>(rkPos, 1.0)
        
        return rkWorldTransform
    }
    
    /// Compute the final transform for a body from MuJoCo pose data.
    /// Applies axis correction (Z-up to Y-up) and combines with the initial local transform.
    ///
    /// - Parameters:
    ///   - values: Array of 7 floats [x, y, z, qx, qy, qz, qw] representing position and quaternion rotation
    ///   - initialTransform: The initial transform of the entity/node from USDZ import
    /// - Returns: The final world transform to apply
    static func computeFinalTransform(
        values: [Float],
        initialTransform: simd_float4x4
    ) -> simd_float4x4 {
        let worldTransform = computeWorldTransform(values: values)
        // Apply initial local transform from USDZ import
        return worldTransform * initialTransform
    }
}
