//
//  CalibrationConfig.swift
//  Tracking Viewer
//
//  Configuration for calibration marker display on iPhone.
//  Contains iPhone display specifications for accurate physical sizing.
//

import Foundation
import UIKit

// MARK: - iPhone Display Specifications

/// iPhone model identifiers and their display specifications
/// Screen sizes are the usable display area (excluding notch/dynamic island for full-screen content)
struct iPhoneDisplaySpec {
    let modelIdentifier: String
    let displayName: String
    /// Screen width in millimeters
    let screenWidthMM: Double
    /// Screen height in millimeters  
    let screenHeightMM: Double
    /// Screen width in points
    let screenWidthPoints: Double
    /// Screen height in points
    let screenHeightPoints: Double
    /// Pixels per inch
    let ppi: Double
    
    /// Points per millimeter
    var pointsPerMM: Double {
        return screenWidthPoints / screenWidthMM
    }
    
    /// Millimeters per point
    var mmPerPoint: Double {
        return screenWidthMM / screenWidthPoints
    }
}

/// Database of iPhone display specifications
/// Data sourced from Apple device specifications
enum iPhoneDisplayDatabase {
    // iPhone 15 Series
    static let iPhone15ProMax = iPhoneDisplaySpec(
        modelIdentifier: "iPhone16,2",
        displayName: "iPhone 15 Pro Max",
        screenWidthMM: 70.6,
        screenHeightMM: 153.0,
        screenWidthPoints: 430,
        screenHeightPoints: 932,
        ppi: 460
    )
    
    static let iPhone15Pro = iPhoneDisplaySpec(
        modelIdentifier: "iPhone16,1",
        displayName: "iPhone 15 Pro",
        screenWidthMM: 64.0,
        screenHeightMM: 139.1,
        screenWidthPoints: 393,
        screenHeightPoints: 852,
        ppi: 460
    )
    
    static let iPhone15Plus = iPhoneDisplaySpec(
        modelIdentifier: "iPhone15,5",
        displayName: "iPhone 15 Plus",
        screenWidthMM: 70.6,
        screenHeightMM: 153.0,
        screenWidthPoints: 430,
        screenHeightPoints: 932,
        ppi: 460
    )
    
    static let iPhone15 = iPhoneDisplaySpec(
        modelIdentifier: "iPhone15,4",
        displayName: "iPhone 15",
        screenWidthMM: 64.0,
        screenHeightMM: 139.1,
        screenWidthPoints: 393,
        screenHeightPoints: 852,
        ppi: 460
    )
    
    // iPhone 14 Series
    static let iPhone14ProMax = iPhoneDisplaySpec(
        modelIdentifier: "iPhone15,3",
        displayName: "iPhone 14 Pro Max",
        screenWidthMM: 70.6,
        screenHeightMM: 153.0,
        screenWidthPoints: 430,
        screenHeightPoints: 932,
        ppi: 460
    )
    
    static let iPhone14Pro = iPhoneDisplaySpec(
        modelIdentifier: "iPhone15,2",
        displayName: "iPhone 14 Pro",
        screenWidthMM: 64.0,
        screenHeightMM: 139.1,
        screenWidthPoints: 393,
        screenHeightPoints: 852,
        ppi: 460
    )
    
    static let iPhone14Plus = iPhoneDisplaySpec(
        modelIdentifier: "iPhone14,8",
        displayName: "iPhone 14 Plus",
        screenWidthMM: 68.8,
        screenHeightMM: 149.2,
        screenWidthPoints: 428,
        screenHeightPoints: 926,
        ppi: 458
    )
    
    static let iPhone14 = iPhoneDisplaySpec(
        modelIdentifier: "iPhone14,7",
        displayName: "iPhone 14",
        screenWidthMM: 62.3,
        screenHeightMM: 135.1,
        screenWidthPoints: 390,
        screenHeightPoints: 844,
        ppi: 460
    )
    
    // iPhone 13 Series
    static let iPhone13ProMax = iPhoneDisplaySpec(
        modelIdentifier: "iPhone14,3",
        displayName: "iPhone 13 Pro Max",
        screenWidthMM: 68.8,
        screenHeightMM: 149.2,
        screenWidthPoints: 428,
        screenHeightPoints: 926,
        ppi: 458
    )
    
    static let iPhone13Pro = iPhoneDisplaySpec(
        modelIdentifier: "iPhone14,2",
        displayName: "iPhone 13 Pro",
        screenWidthMM: 62.3,
        screenHeightMM: 135.1,
        screenWidthPoints: 390,
        screenHeightPoints: 844,
        ppi: 460
    )
    
    static let iPhone13 = iPhoneDisplaySpec(
        modelIdentifier: "iPhone14,5",
        displayName: "iPhone 13",
        screenWidthMM: 62.3,
        screenHeightMM: 135.1,
        screenWidthPoints: 390,
        screenHeightPoints: 844,
        ppi: 460
    )
    
    static let iPhone13Mini = iPhoneDisplaySpec(
        modelIdentifier: "iPhone14,4",
        displayName: "iPhone 13 mini",
        screenWidthMM: 56.0,
        screenHeightMM: 121.3,
        screenWidthPoints: 375,
        screenHeightPoints: 812,
        ppi: 476
    )
    
    // iPhone 12 Series
    static let iPhone12ProMax = iPhoneDisplaySpec(
        modelIdentifier: "iPhone13,4",
        displayName: "iPhone 12 Pro Max",
        screenWidthMM: 68.8,
        screenHeightMM: 149.2,
        screenWidthPoints: 428,
        screenHeightPoints: 926,
        ppi: 458
    )
    
    static let iPhone12Pro = iPhoneDisplaySpec(
        modelIdentifier: "iPhone13,3",
        displayName: "iPhone 12 Pro",
        screenWidthMM: 62.3,
        screenHeightMM: 135.1,
        screenWidthPoints: 390,
        screenHeightPoints: 844,
        ppi: 460
    )
    
    static let iPhone12 = iPhoneDisplaySpec(
        modelIdentifier: "iPhone13,2",
        displayName: "iPhone 12",
        screenWidthMM: 62.3,
        screenHeightMM: 135.1,
        screenWidthPoints: 390,
        screenHeightPoints: 844,
        ppi: 460
    )
    
    static let iPhone12Mini = iPhoneDisplaySpec(
        modelIdentifier: "iPhone13,1",
        displayName: "iPhone 12 mini",
        screenWidthMM: 56.0,
        screenHeightMM: 121.3,
        screenWidthPoints: 375,
        screenHeightPoints: 812,
        ppi: 476
    )
    
    // iPhone SE (3rd gen)
    static let iPhoneSE3 = iPhoneDisplaySpec(
        modelIdentifier: "iPhone14,6",
        displayName: "iPhone SE (3rd gen)",
        screenWidthMM: 58.5,
        screenHeightMM: 104.0,
        screenWidthPoints: 375,
        screenHeightPoints: 667,
        ppi: 326
    )
    
    // iPhone 11 Series
    static let iPhone11ProMax = iPhoneDisplaySpec(
        modelIdentifier: "iPhone12,5",
        displayName: "iPhone 11 Pro Max",
        screenWidthMM: 68.8,
        screenHeightMM: 143.2,
        screenWidthPoints: 414,
        screenHeightPoints: 896,
        ppi: 458
    )
    
    static let iPhone11Pro = iPhoneDisplaySpec(
        modelIdentifier: "iPhone12,3",
        displayName: "iPhone 11 Pro",
        screenWidthMM: 62.3,
        screenHeightMM: 130.0,
        screenWidthPoints: 375,
        screenHeightPoints: 812,
        ppi: 458
    )
    
    static let iPhone11 = iPhoneDisplaySpec(
        modelIdentifier: "iPhone12,1",
        displayName: "iPhone 11",
        screenWidthMM: 68.4,
        screenHeightMM: 142.6,
        screenWidthPoints: 414,
        screenHeightPoints: 896,
        ppi: 326
    )
    
    // iPhone 16 Series (2024)
    static let iPhone16ProMax = iPhoneDisplaySpec(
        modelIdentifier: "iPhone17,2",
        displayName: "iPhone 16 Pro Max",
        screenWidthMM: 72.5,
        screenHeightMM: 159.0,
        screenWidthPoints: 440,
        screenHeightPoints: 956,
        ppi: 460
    )
    
    static let iPhone16Pro = iPhoneDisplaySpec(
        modelIdentifier: "iPhone17,1",
        displayName: "iPhone 16 Pro",
        screenWidthMM: 66.2,
        screenHeightMM: 145.2,
        screenWidthPoints: 402,
        screenHeightPoints: 874,
        ppi: 460
    )
    
    static let iPhone16Plus = iPhoneDisplaySpec(
        modelIdentifier: "iPhone17,4",
        displayName: "iPhone 16 Plus",
        screenWidthMM: 70.6,
        screenHeightMM: 153.0,
        screenWidthPoints: 430,
        screenHeightPoints: 932,
        ppi: 460
    )
    
    static let iPhone16 = iPhoneDisplaySpec(
        modelIdentifier: "iPhone17,3",
        displayName: "iPhone 16",
        screenWidthMM: 64.0,
        screenHeightMM: 139.1,
        screenWidthPoints: 393,
        screenHeightPoints: 852,
        ppi: 460
    )
    
    /// All known iPhone specs indexed by model identifier
    static let allSpecs: [String: iPhoneDisplaySpec] = [
        // iPhone 16 Series
        "iPhone17,1": iPhone16Pro,
        "iPhone17,2": iPhone16ProMax,
        "iPhone17,3": iPhone16,
        "iPhone17,4": iPhone16Plus,
        // iPhone 15 Series
        "iPhone16,1": iPhone15Pro,
        "iPhone16,2": iPhone15ProMax,
        "iPhone15,4": iPhone15,
        "iPhone15,5": iPhone15Plus,
        // iPhone 14 Series
        "iPhone15,2": iPhone14Pro,
        "iPhone15,3": iPhone14ProMax,
        "iPhone14,7": iPhone14,
        "iPhone14,8": iPhone14Plus,
        // iPhone 13 Series
        "iPhone14,2": iPhone13Pro,
        "iPhone14,3": iPhone13ProMax,
        "iPhone14,4": iPhone13Mini,
        "iPhone14,5": iPhone13,
        // iPhone SE 3rd gen
        "iPhone14,6": iPhoneSE3,
        // iPhone 12 Series
        "iPhone13,1": iPhone12Mini,
        "iPhone13,2": iPhone12,
        "iPhone13,3": iPhone12Pro,
        "iPhone13,4": iPhone12ProMax,
        // iPhone 11 Series
        "iPhone12,1": iPhone11,
        "iPhone12,3": iPhone11Pro,
        "iPhone12,5": iPhone11ProMax,
    ]
    
    /// Get display spec for the current device
    static func currentDeviceSpec() -> iPhoneDisplaySpec? {
        var systemInfo = utsname()
        uname(&systemInfo)
        let machineMirror = Mirror(reflecting: systemInfo.machine)
        let identifier = machineMirror.children.reduce("") { identifier, element in
            guard let value = element.value as? Int8, value != 0 else { return identifier }
            return identifier + String(UnicodeScalar(UInt8(value)))
        }
        
        print("📱 [CalibrationConfig] Device identifier: \(identifier)")
        return allSpecs[identifier]
    }
    
    /// Get a fallback spec based on screen size (for simulators or unknown devices)
    static func fallbackSpec() -> iPhoneDisplaySpec {
        let screenBounds = UIScreen.main.bounds
        let screenWidth = screenBounds.width
        let screenHeight = screenBounds.height
        
        // Find the closest match by screen dimensions
        var bestMatch: iPhoneDisplaySpec = iPhone15Pro
        var smallestDiff = Double.infinity
        
        for spec in allSpecs.values {
            let widthDiff = abs(spec.screenWidthPoints - screenWidth)
            let heightDiff = abs(spec.screenHeightPoints - screenHeight)
            let totalDiff = widthDiff + heightDiff
            
            if totalDiff < smallestDiff {
                smallestDiff = totalDiff
                bestMatch = spec
            }
        }
        
        print("📱 [CalibrationConfig] Using fallback spec: \(bestMatch.displayName)")
        return bestMatch
    }
}

// MARK: - Calibration Marker Configuration

/// ArUco dictionary types (matching the Vision Pro app)
enum ArucoDictionary: Int, Codable, CaseIterable {
    case dict4X4_50 = 0
    case dict4X4_100 = 1
    case dict4X4_250 = 2
    case dict4X4_1000 = 3
    case dict5X5_50 = 4
    case dict5X5_100 = 5
    case dict5X5_250 = 6
    case dict5X5_1000 = 7
    case dict6X6_50 = 8
    case dict6X6_100 = 9
    case dict6X6_250 = 10
    case dict6X6_1000 = 11
    case dict7X7_50 = 12
    case dict7X7_100 = 13
    case dict7X7_250 = 14
    case dict7X7_1000 = 15
    
    var displayName: String {
        switch self {
        case .dict4X4_50: return "4×4 (50)"
        case .dict4X4_100: return "4×4 (100)"
        case .dict4X4_250: return "4×4 (250)"
        case .dict4X4_1000: return "4×4 (1000)"
        case .dict5X5_50: return "5×5 (50)"
        case .dict5X5_100: return "5×5 (100)"
        case .dict5X5_250: return "5×5 (250)"
        case .dict5X5_1000: return "5×5 (1000)"
        case .dict6X6_50: return "6×6 (50)"
        case .dict6X6_100: return "6×6 (100)"
        case .dict6X6_250: return "6×6 (250)"
        case .dict6X6_1000: return "6×6 (1000)"
        case .dict7X7_50: return "7×7 (50)"
        case .dict7X7_100: return "7×7 (100)"
        case .dict7X7_250: return "7×7 (250)"
        case .dict7X7_1000: return "7×7 (1000)"
        }
    }
    
    /// Grid size for the ArUco markers
    var gridSize: Int {
        switch self {
        case .dict4X4_50, .dict4X4_100, .dict4X4_250, .dict4X4_1000: return 4
        case .dict5X5_50, .dict5X5_100, .dict5X5_250, .dict5X5_1000: return 5
        case .dict6X6_50, .dict6X6_100, .dict6X6_250, .dict6X6_1000: return 6
        case .dict7X7_50, .dict7X7_100, .dict7X7_250, .dict7X7_1000: return 7
        }
    }
    
    /// Maximum marker ID for this dictionary
    var maxMarkerId: Int {
        switch self {
        case .dict4X4_50, .dict5X5_50, .dict6X6_50, .dict7X7_50: return 49
        case .dict4X4_100, .dict5X5_100, .dict6X6_100, .dict7X7_100: return 99
        case .dict4X4_250, .dict5X5_250, .dict6X6_250, .dict7X7_250: return 249
        case .dict4X4_1000, .dict5X5_1000, .dict6X6_1000, .dict7X7_1000: return 999
        }
    }
}

/// Checkerboard configuration
struct CheckerboardDisplayConfig: Codable, Equatable {
    /// Number of inner corners horizontally
    var innerCornersX: Int = 5
    /// Number of inner corners vertically
    var innerCornersY: Int = 4
    /// Physical size of one square in millimeters
    var squareSizeMM: Float = 10.0
    
    /// Total pattern width in mm (includes border squares)
    var patternWidthMM: Float {
        return Float(innerCornersX + 1) * squareSizeMM
    }
    
    /// Total pattern height in mm
    var patternHeightMM: Float {
        return Float(innerCornersY + 1) * squareSizeMM
    }
}

/// ArUco marker configuration
struct ArucoMarkerConfig: Codable, Equatable {
    /// ArUco dictionary type
    var dictionary: ArucoDictionary = .dict4X4_50
    /// Physical marker size in millimeters (including white border)
    var markerSizeMM: Float = 50.0
    /// Marker IDs available for calibration (user moves phone showing different IDs)
    var availableMarkerIds: [Int] = [0, 1, 2, 3]
    /// Minimum number of different marker positions needed
    var minUniquePositions: Int = 3
    /// Samples to collect per position
    var samplesPerPosition: Int = 30
}

// MARK: - Shared Calibration State

/// State of a calibration session synced between iPhone and Vision Pro
struct CalibrationSessionState: Codable {
    enum SessionPhase: String, Codable {
        case idle
        case displayingMarker
        case collectingSamples
        case processingResults
        case complete
        case error
    }
    
    var phase: SessionPhase = .idle
    var currentMarkerId: Int = 0
    var currentPositionIndex: Int = 0  // Which position (0, 1, 2...) the user is at
    var samplesCollectedForCurrentPosition: Int = 0
    var totalSamplesCollected: Int = 0
    var errorMessage: String?
    var lastUpdate: Date = Date()
    
    /// Whether the Vision Pro confirmed it's ready to receive markers
    var visionProReady: Bool = false
    
    /// Message to display to the user
    var userMessage: String {
        switch phase {
        case .idle:
            return "Press Start to begin calibration"
        case .displayingMarker:
            return "Position \(currentPositionIndex + 1): Hold phone steady, facing the Vision Pro camera"
        case .collectingSamples:
            return "Position \(currentPositionIndex + 1): Collecting samples... (\(samplesCollectedForCurrentPosition)/20)"
        case .processingResults:
            return "Processing calibration data..."
        case .complete:
            return "Calibration complete!"
        case .error:
            return errorMessage ?? "An error occurred"
        }
    }
}

/// Calibration results from the Vision Pro
struct ExtrinsicCalibrationResult: Codable {
    let cameraDeviceId: String
    let cameraDeviceName: String
    let isStereo: Bool
    /// Transform matrix (4x4, column-major) from head to left/mono camera
    let headToCamera: [Double]
    /// Transform matrix (4x4, column-major) from head to right camera (stereo only)
    let rightHeadToCamera: [Double]?
    /// Reprojection error in meters (left/mono camera)
    let reprojectionError: Double
    /// Reprojection error for right camera (stereo only)
    let rightReprojectionError: Double?
    let sampleCount: Int
    /// Sample count for right camera (stereo only)
    let rightSampleCount: Int?
    let calibrationDate: Date
    
    /// Human-readable transform description
    var transformDescription: String {
        guard headToCamera.count == 16 else { return "Invalid transform" }
        let tx = headToCamera[12]
        let ty = headToCamera[13]
        let tz = headToCamera[14]
        return String(format: "Translation: (%.3f, %.3f, %.3f) m", tx, ty, tz)
    }
    
    /// Extract translation from transform matrix
    var translation: (x: Double, y: Double, z: Double) {
        guard headToCamera.count == 16 else { return (0, 0, 0) }
        return (headToCamera[12], headToCamera[13], headToCamera[14])
    }
    
    /// Extract translation from right camera transform matrix (stereo only)
    var rightTranslation: (x: Double, y: Double, z: Double)? {
        guard let rightMatrix = rightHeadToCamera, rightMatrix.count == 16 else { return nil }
        return (rightMatrix[12], rightMatrix[13], rightMatrix[14])
    }
    
    /// Extract rotation matrix (3x3) from transform
    var rotationMatrix: [[Double]] {
        guard headToCamera.count == 16 else { return [[1,0,0],[0,1,0],[0,0,1]] }
        return [
            [headToCamera[0], headToCamera[1], headToCamera[2]],
            [headToCamera[4], headToCamera[5], headToCamera[6]],
            [headToCamera[8], headToCamera[9], headToCamera[10]]
        ]
    }
    
    /// Convert rotation matrix to Euler angles (degrees)
    var eulerAngles: (pitch: Double, yaw: Double, roll: Double) {
        let r = rotationMatrix
        let pitch = atan2(-r[2][0], sqrt(r[0][0]*r[0][0] + r[1][0]*r[1][0])) * 180 / .pi
        let yaw = atan2(r[1][0], r[0][0]) * 180 / .pi
        let roll = atan2(r[2][1], r[2][2]) * 180 / .pi
        return (pitch, yaw, roll)
    }
    
    // Custom decoder to handle TimeInterval from visionOS and optional stereo fields
    init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        cameraDeviceId = try container.decode(String.self, forKey: .cameraDeviceId)
        cameraDeviceName = try container.decode(String.self, forKey: .cameraDeviceName)
        isStereo = try container.decode(Bool.self, forKey: .isStereo)
        headToCamera = try container.decode([Double].self, forKey: .headToCamera)
        rightHeadToCamera = try container.decodeIfPresent([Double].self, forKey: .rightHeadToCamera)
        reprojectionError = try container.decode(Double.self, forKey: .reprojectionError)
        rightReprojectionError = try container.decodeIfPresent(Double.self, forKey: .rightReprojectionError)
        sampleCount = try container.decode(Int.self, forKey: .sampleCount)
        rightSampleCount = try container.decodeIfPresent(Int.self, forKey: .rightSampleCount)
        
        // Handle calibrationDate as either Date or TimeInterval
        if let date = try? container.decode(Date.self, forKey: .calibrationDate) {
            calibrationDate = date
        } else if let interval = try? container.decode(Double.self, forKey: .calibrationDate) {
            calibrationDate = Date(timeIntervalSince1970: interval)
        } else {
            calibrationDate = Date()
        }
    }
}

/// Intrinsic calibration results from the Vision Pro
struct IntrinsicCalibrationResult: Codable {
    let cameraDeviceId: String
    let cameraDeviceName: String
    let isStereo: Bool
    let fx: Double
    let fy: Double
    let cx: Double
    let cy: Double
    let distortionCoeffs: [Double]
    let reprojectionError: Double
    let imageWidth: Int
    let imageHeight: Int
    let calibrationDate: Date
    
    /// Human-readable description
    var description: String {
        return String(format: "fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f, err=%.4f", fx, fy, cx, cy, reprojectionError)
    }
    
    // Custom decoder to handle TimeInterval from visionOS
    init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        cameraDeviceId = try container.decode(String.self, forKey: .cameraDeviceId)
        cameraDeviceName = try container.decode(String.self, forKey: .cameraDeviceName)
        isStereo = try container.decode(Bool.self, forKey: .isStereo)
        fx = try container.decode(Double.self, forKey: .fx)
        fy = try container.decode(Double.self, forKey: .fy)
        cx = try container.decode(Double.self, forKey: .cx)
        cy = try container.decode(Double.self, forKey: .cy)
        distortionCoeffs = try container.decode([Double].self, forKey: .distortionCoeffs)
        reprojectionError = try container.decode(Double.self, forKey: .reprojectionError)
        imageWidth = try container.decode(Int.self, forKey: .imageWidth)
        imageHeight = try container.decode(Int.self, forKey: .imageHeight)
        
        // Handle calibrationDate as either Date or TimeInterval
        if let date = try? container.decode(Date.self, forKey: .calibrationDate) {
            calibrationDate = date
        } else if let interval = try? container.decode(Double.self, forKey: .calibrationDate) {
            calibrationDate = Date(timeIntervalSince1970: interval)
        } else {
            calibrationDate = Date()
        }
    }
}

// MARK: - Standard Credit Card Dimensions

/// ISO/IEC 7810 ID-1 standard credit card dimensions
enum CreditCardDimensions {
    /// Width in millimeters
    static let widthMM: Double = 85.6
    /// Height in millimeters  
    static let heightMM: Double = 53.98
    /// Corner radius in millimeters
    static let cornerRadiusMM: Double = 3.18
}
