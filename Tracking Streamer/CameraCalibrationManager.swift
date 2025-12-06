import Foundation
import SwiftUI
import simd

// MARK: - Calibration Data Models

/// Represents intrinsic calibration data for a camera
struct CameraIntrinsics: Codable, Equatable {
    /// Focal length in x (pixels)
    let fx: Double
    /// Focal length in y (pixels)
    let fy: Double
    /// Principal point x coordinate (pixels)
    let cx: Double
    /// Principal point y coordinate (pixels)
    let cy: Double
    /// Distortion coefficients (k1, k2, p1, p2, k3, ...)
    let distortionCoeffs: [Double]
    /// Reprojection error from calibration
    let reprojectionError: Double
    /// Image width used during calibration
    let imageWidth: Int
    /// Image height used during calibration
    let imageHeight: Int
    
    /// Creates a 3x3 intrinsic matrix
    var intrinsicMatrix: simd_float3x3 {
        simd_float3x3(
            SIMD3<Float>(Float(fx), 0, 0),
            SIMD3<Float>(0, Float(fy), 0),
            SIMD3<Float>(Float(cx), Float(cy), 1)
        )
    }
    
    /// Creates intrinsics from a row-major 3x3 array
    init(from matrix: [Double], distortion: [Double], reprojError: Double, width: Int, height: Int) {
        // Matrix is row-major: [m00, m01, m02, m10, m11, m12, m20, m21, m22]
        self.fx = matrix[0]
        self.fy = matrix[4]
        self.cx = matrix[2]
        self.cy = matrix[5]
        self.distortionCoeffs = distortion
        self.reprojectionError = reprojError
        self.imageWidth = width
        self.imageHeight = height
    }
}

/// Represents stereo calibration data (extrinsics between two cameras)
struct StereoExtrinsics: Codable, Equatable {
    /// Rotation matrix from left to right camera (row-major 3x3)
    let rotationMatrix: [Double]
    /// Translation vector from left to right camera (3 elements)
    let translationVector: [Double]
    /// Stereo reprojection error
    let stereoReprojectionError: Double
}

/// Complete calibration data for a camera device
struct CameraCalibrationData: Codable, Equatable {
    /// Unique identifier for the camera device
    let deviceId: String
    /// Human-readable device name
    let deviceName: String
    /// Whether this is a stereo camera
    let isStereo: Bool
    /// Left camera intrinsics (or mono camera intrinsics)
    let leftIntrinsics: CameraIntrinsics
    /// Right camera intrinsics (nil for mono cameras)
    let rightIntrinsics: CameraIntrinsics?
    /// Stereo extrinsics (nil for mono cameras)
    let stereoExtrinsics: StereoExtrinsics?
    /// Checkerboard configuration used for calibration
    let checkerboardConfig: CheckerboardConfig
    /// Date when calibration was performed
    let calibrationDate: Date
    /// Number of samples used for calibration
    let sampleCount: Int
    /// App version when calibration was performed
    let appVersion: String
}

/// Checkerboard configuration used for calibration
struct CheckerboardConfig: Codable, Equatable {
    /// Number of inner corners horizontally
    let innerCornersX: Int
    /// Number of inner corners vertically
    let innerCornersY: Int
    /// Physical size of one square in meters
    let squareSize: Float
}

// MARK: - Camera Calibration Manager

/// Manages camera calibration data storage and retrieval
@MainActor
class CameraCalibrationManager: ObservableObject {
    static let shared = CameraCalibrationManager()
    
    // MARK: - Published Properties
    
    /// Currently loaded calibration data (for selected camera)
    @Published var currentCalibration: CameraCalibrationData? = nil
    
    /// All available calibrations
    @Published var allCalibrations: [String: CameraCalibrationData] = [:]
    
    /// Whether calibration is in progress
    @Published var isCalibrating: Bool = false
    
    /// Current calibration progress (0-1)
    @Published var calibrationProgress: Float = 0.0
    
    /// Number of samples collected so far
    @Published var samplesCollected: Int = 0
    
    /// Status message during calibration
    @Published var statusMessage: String = ""
    
    /// Last calibration error (if any)
    @Published var lastError: String? = nil
    
    // MARK: - Configuration
    
    /// Minimum samples required for calibration
    var minSamples: Int = 20
    
    /// Current checkerboard configuration
    @Published var checkerboardConfig = CheckerboardConfig(
        innerCornersX: 5,
        innerCornersY: 4,
        squareSize: 0.010  // 10mm squares to fit on iPhone screen
    )
    
    /// Minimum time between auto-captures (seconds)
    var minCaptureInterval: TimeInterval = 0.7
    
    /// Minimum corner movement for auto-capture (pixels)
    var minCornerMovement: Float = 15.0
    
    // MARK: - Private Properties
    
    private let storageKey = "cameraCalibrations"
    private var calibrator: OpenCVCalibrator?
    private var lastCaptureTime: Date = .distantPast
    private var lastCornersLeft: [NSValue]? = nil
    private var lastCornersRight: [NSValue]? = nil
    
    // MARK: - Initialization
    
    private init() {
        loadAllCalibrations()
    }
    
    // MARK: - Storage
    
    /// Loads all calibrations from UserDefaults
    private func loadAllCalibrations() {
        guard let data = UserDefaults.standard.data(forKey: storageKey) else {
            print("📷 [CameraCalibrationManager] No saved calibrations found")
            return
        }
        
        do {
            let decoder = JSONDecoder()
            allCalibrations = try decoder.decode([String: CameraCalibrationData].self, from: data)
            print("📷 [CameraCalibrationManager] Loaded \(allCalibrations.count) calibration(s)")
        } catch {
            print("❌ [CameraCalibrationManager] Failed to load calibrations: \(error)")
        }
    }
    
    /// Saves all calibrations to UserDefaults
    private func saveAllCalibrations() {
        do {
            let encoder = JSONEncoder()
            let data = try encoder.encode(allCalibrations)
            UserDefaults.standard.set(data, forKey: storageKey)
            print("📷 [CameraCalibrationManager] Saved \(allCalibrations.count) calibration(s)")
            
            // Also sync to iCloud for iPhone app
            syncToiCloud()
        } catch {
            print("❌ [CameraCalibrationManager] Failed to save calibrations: \(error)")
        }
    }
    
    // MARK: - iCloud Sync for iPhone App
    
    /// iCloud KVS key for intrinsic calibration results
    private static let iCloudIntrinsicResultsKey = "intrinsicCalibrationResults"
    
    /// Sync all intrinsic calibrations to iCloud KVS for the iPhone app to display
    private func syncToiCloud() {
        var results: [[String: Any]] = []
        
        for calibration in allCalibrations.values {
            let result: [String: Any] = [
                "cameraDeviceId": calibration.deviceId,
                "cameraDeviceName": calibration.deviceName,
                "isStereo": calibration.isStereo,
                "fx": calibration.leftIntrinsics.fx,
                "fy": calibration.leftIntrinsics.fy,
                "cx": calibration.leftIntrinsics.cx,
                "cy": calibration.leftIntrinsics.cy,
                "distortionCoeffs": calibration.leftIntrinsics.distortionCoeffs,
                "reprojectionError": calibration.leftIntrinsics.reprojectionError,
                "imageWidth": calibration.leftIntrinsics.imageWidth,
                "imageHeight": calibration.leftIntrinsics.imageHeight,
                "calibrationDate": calibration.calibrationDate.timeIntervalSince1970
            ]
            results.append(result)
        }
        
        do {
            let jsonData = try JSONSerialization.data(withJSONObject: results, options: [])
            NSUbiquitousKeyValueStore.default.set(jsonData, forKey: Self.iCloudIntrinsicResultsKey)
            NSUbiquitousKeyValueStore.default.synchronize()
            print("☁️ [CameraCalibrationManager] Synced \(results.count) calibration(s) to iCloud")
        } catch {
            print("❌ [CameraCalibrationManager] Failed to sync to iCloud: \(error)")
        }
    }
    
    // MARK: - Calibration Loading
    
    /// Loads calibration for a specific camera device
    /// - Parameter deviceId: The unique identifier for the camera
    /// - Returns: The calibration data if available
    func loadCalibration(for deviceId: String) -> CameraCalibrationData? {
        if let calibration = allCalibrations[deviceId] {
            currentCalibration = calibration
            print("📷 [CameraCalibrationManager] Loaded calibration for device: \(calibration.deviceName)")
            return calibration
        }
        currentCalibration = nil
        return nil
    }
    
    /// Checks if a calibration exists for a device
    func hasCalibration(for deviceId: String) -> Bool {
        return allCalibrations[deviceId] != nil
    }
    
    /// Deletes calibration for a specific device
    func deleteCalibration(for deviceId: String) {
        allCalibrations.removeValue(forKey: deviceId)
        if currentCalibration?.deviceId == deviceId {
            currentCalibration = nil
        }
        saveAllCalibrations()
        print("📷 [CameraCalibrationManager] Deleted calibration for device: \(deviceId)")
    }
    
    // MARK: - Calibration Process
    
    /// Starts a new calibration session
    /// - Parameters:
    ///   - deviceId: Unique identifier for the camera
    ///   - deviceName: Human-readable name for the camera
    ///   - isStereo: Whether this is a stereo camera
    func startCalibration(deviceId: String, deviceName: String, isStereo: Bool) {
        print("📷 [CameraCalibrationManager] ========== START CALIBRATION ==========")
        print("📷 [CameraCalibrationManager] Device: \(deviceName) (id: \(deviceId))")
        print("📷 [CameraCalibrationManager] Stereo: \(isStereo)")
        print("📷 [CameraCalibrationManager] Checkerboard config: \(checkerboardConfig.innerCornersX)x\(checkerboardConfig.innerCornersY), square: \(checkerboardConfig.squareSize)m")
        
        calibrator = OpenCVCalibrator(
            checkerboardCornersX: Int32(checkerboardConfig.innerCornersX),
            cornersY: Int32(checkerboardConfig.innerCornersY),
            squareSize: checkerboardConfig.squareSize
        )
        
        print("📷 [CameraCalibrationManager] OpenCVCalibrator created: \(calibrator != nil)")
        
        isCalibrating = true
        calibrationProgress = 0.0
        samplesCollected = 0
        lastError = nil
        lastCaptureTime = .distantPast
        lastCornersLeft = nil
        lastCornersRight = nil
        statusMessage = "Move the checkerboard around..."
        
        print("📷 [CameraCalibrationManager] isCalibrating set to: \(isCalibrating)")
        print("📷 [CameraCalibrationManager] ========== START CALIBRATION COMPLETE ==========")
    }
    
    /// Processes a stereo frame for calibration
    /// - Parameter pixelBuffer: Side-by-side stereo frame
    /// - Returns: Detection result with visualization
    func processStereoFrame(_ pixelBuffer: CVPixelBuffer) -> CheckerboardDetection? {
        guard isCalibrating, let calibrator = calibrator else { return nil }
        
        let detection = calibrator.detectCheckerboard(stereoFrame: pixelBuffer)
        
        guard let result = detection, result.foundLeft && result.foundRight else {
            statusMessage = "Looking for checkerboard in BOTH views..."
            return detection
        }
        
        // Check if enough time has passed
        let now = Date()
        let timeSinceLastCapture = now.timeIntervalSince(lastCaptureTime)
        
        guard timeSinceLastCapture >= minCaptureInterval else {
            statusMessage = "Hold still..."
            return detection
        }
        
        // Check if the pose has changed enough
        guard let leftCorners = result.leftCorners,
              let rightCorners = result.rightCorners else {
            return detection
        }
        
        let movementLeft = lastCornersLeft.map { calibrator.meanCornerMovement(from: $0, to: leftCorners) } ?? Float.infinity
        let movementRight = lastCornersRight.map { calibrator.meanCornerMovement(from: $0, to: rightCorners) } ?? Float.infinity
        let movement = min(movementLeft, movementRight)
        
        guard movement >= minCornerMovement else {
            statusMessage = "Move checkerboard to a new position..."
            return detection
        }
        
        // Add calibration sample
        let width = CVPixelBufferGetWidth(pixelBuffer) / 2
        let height = CVPixelBufferGetHeight(pixelBuffer)
        
        samplesCollected = Int(calibrator.addCalibrationSample(
            leftCorners: leftCorners,
            rightCorners: rightCorners,
            width: Int32(width),
            height: Int32(height)
        ))
        
        lastCaptureTime = now
        lastCornersLeft = leftCorners
        lastCornersRight = rightCorners
        
        calibrationProgress = Float(samplesCollected) / Float(minSamples)
        statusMessage = "Captured sample \(samplesCollected)/\(minSamples)"
        
        print("📷 [CameraCalibrationManager] Auto-captured sample #\(samplesCollected)")
        
        return detection
    }
    
    /// Processes a mono frame for calibration
    /// - Parameter pixelBuffer: Mono camera frame
    /// - Returns: Detection result with visualization
    func processMonoFrame(_ pixelBuffer: CVPixelBuffer) -> CheckerboardDetection? {
        print("🖼️ [CameraCalibrationManager] processMonoFrame called")
        print("🖼️ [CameraCalibrationManager] isCalibrating: \(isCalibrating), calibrator: \(calibrator != nil)")
        
        guard isCalibrating, let calibrator = calibrator else {
            print("❌ [CameraCalibrationManager] Guard failed - isCalibrating: \(isCalibrating), calibrator: \(calibrator != nil)")
            return nil
        }
        
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        print("🖼️ [CameraCalibrationManager] Frame size: \(width)x\(height)")
        
        print("🔍 [CameraCalibrationManager] Calling detectCheckerboard...")
        let detection = calibrator.detectCheckerboard(monoFrame: pixelBuffer)
        print("🔍 [CameraCalibrationManager] Detection result: \(detection != nil), foundLeft: \(detection?.foundLeft ?? false)")
        
        guard let result = detection, result.foundLeft else {
            statusMessage = "Looking for checkerboard..."
            return detection
        }
        
        // Check if enough time has passed
        let now = Date()
        let timeSinceLastCapture = now.timeIntervalSince(lastCaptureTime)
        
        guard timeSinceLastCapture >= minCaptureInterval else {
            statusMessage = "Hold still..."
            return detection
        }
        
        // Check if the pose has changed enough
        guard let corners = result.leftCorners else {
            return detection
        }
        
        let movement = lastCornersLeft.map { calibrator.meanCornerMovement(from: $0, to: corners) } ?? Float.infinity
        
        guard movement >= minCornerMovement else {
            statusMessage = "Move checkerboard to a new position..."
            return detection
        }
        
        // Add calibration sample (reuse width/height from above)
        samplesCollected = Int(calibrator.addMonoCalibrationSample(
            corners: corners,
            width: Int32(width),
            height: Int32(height)
        ))
        
        lastCaptureTime = now
        lastCornersLeft = corners
        
        calibrationProgress = Float(samplesCollected) / Float(minSamples)
        statusMessage = "Captured sample \(samplesCollected)/\(minSamples)"
        
        print("📷 [CameraCalibrationManager] Auto-captured sample #\(samplesCollected)")
        
        return detection
    }
    
    /// Finishes calibration and computes the intrinsics (async version - runs heavy computation on background thread)
    /// - Parameters:
    ///   - deviceId: Unique identifier for the camera
    ///   - deviceName: Human-readable name for the camera
    ///   - isStereo: Whether this is a stereo camera
    /// - Returns: The computed calibration data, or nil on failure
    func finishCalibrationAsync(deviceId: String, deviceName: String, isStereo: Bool) async -> CameraCalibrationData? {
        guard let calibrator = calibrator else {
            lastError = "No calibrator available"
            isCalibrating = false
            return nil
        }
        
        statusMessage = "Computing calibration..."
        
        // Capture minSamples before entering detached task
        let minSamplesCount = Int32(minSamples)
        
        // Run heavy OpenCV computation on background thread
        let result: StereoCalibrationResult? = await Task.detached(priority: .userInitiated) {
            if isStereo {
                return calibrator.performStereoCalibration(minSamples: minSamplesCount)
            } else {
                return calibrator.performMonoCalibration(minSamples: minSamplesCount)
            }
        }.value
        
        guard let calibResult = result, calibResult.success else {
            lastError = result?.errorMessage ?? "Calibration failed"
            statusMessage = "Calibration failed"
            isCalibrating = false
            return nil
        }
        
        // Build calibration data
        let leftIntrinsics = CameraIntrinsics(
            from: calibResult.leftIntrinsicMatrix.map { $0.doubleValue },
            distortion: calibResult.leftDistortionCoeffs.map { $0.doubleValue },
            reprojError: calibResult.leftReprojectionError,
            width: Int(calibResult.imageWidth),
            height: Int(calibResult.imageHeight)
        )
        
        var rightIntrinsics: CameraIntrinsics? = nil
        var stereoExtrinsics: StereoExtrinsics? = nil
        
        if isStereo {
            rightIntrinsics = CameraIntrinsics(
                from: calibResult.rightIntrinsicMatrix.map { $0.doubleValue },
                distortion: calibResult.rightDistortionCoeffs.map { $0.doubleValue },
                reprojError: calibResult.rightReprojectionError,
                width: Int(calibResult.imageWidth),
                height: Int(calibResult.imageHeight)
            )
            
            stereoExtrinsics = StereoExtrinsics(
                rotationMatrix: calibResult.rotationMatrix.map { $0.doubleValue },
                translationVector: calibResult.translationVector.map { $0.doubleValue },
                stereoReprojectionError: calibResult.stereoReprojectionError
            )
        }
        
        let appVersion = Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "1.0"
        
        let calibrationData = CameraCalibrationData(
            deviceId: deviceId,
            deviceName: deviceName,
            isStereo: isStereo,
            leftIntrinsics: leftIntrinsics,
            rightIntrinsics: rightIntrinsics,
            stereoExtrinsics: stereoExtrinsics,
            checkerboardConfig: checkerboardConfig,
            calibrationDate: Date(),
            sampleCount: samplesCollected,
            appVersion: appVersion
        )
        
        // Save calibration
        allCalibrations[deviceId] = calibrationData
        currentCalibration = calibrationData
        saveAllCalibrations()
        
        statusMessage = "Calibration complete!"
        isCalibrating = false
        
        print("✅ [CameraCalibrationManager] Calibration saved for: \(deviceName)")
        print("   Left reproj error: \(leftIntrinsics.reprojectionError)")
        if let right = rightIntrinsics {
            print("   Right reproj error: \(right.reprojectionError)")
        }
        if let stereo = stereoExtrinsics {
            print("   Stereo reproj error: \(stereo.stereoReprojectionError)")
        }
        
        return calibrationData
    }
    
    /// Finishes calibration and computes the intrinsics (synchronous version - may block UI)
    /// - Parameters:
    ///   - deviceId: Unique identifier for the camera
    ///   - deviceName: Human-readable name for the camera
    ///   - isStereo: Whether this is a stereo camera
    /// - Returns: The computed calibration data, or nil on failure
    func finishCalibration(deviceId: String, deviceName: String, isStereo: Bool) -> CameraCalibrationData? {
        guard let calibrator = calibrator else {
            lastError = "No calibrator available"
            isCalibrating = false
            return nil
        }
        
        statusMessage = "Computing calibration..."
        
        let result: StereoCalibrationResult?
        if isStereo {
            result = calibrator.performStereoCalibration(minSamples: Int32(minSamples))
        } else {
            result = calibrator.performMonoCalibration(minSamples: Int32(minSamples))
        }
        
        guard let calibResult = result, calibResult.success else {
            lastError = result?.errorMessage ?? "Calibration failed"
            statusMessage = "Calibration failed"
            isCalibrating = false
            return nil
        }
        
        // Build calibration data
        let leftIntrinsics = CameraIntrinsics(
            from: calibResult.leftIntrinsicMatrix.map { $0.doubleValue },
            distortion: calibResult.leftDistortionCoeffs.map { $0.doubleValue },
            reprojError: calibResult.leftReprojectionError,
            width: Int(calibResult.imageWidth),
            height: Int(calibResult.imageHeight)
        )
        
        var rightIntrinsics: CameraIntrinsics? = nil
        var stereoExtrinsics: StereoExtrinsics? = nil
        
        if isStereo {
            rightIntrinsics = CameraIntrinsics(
                from: calibResult.rightIntrinsicMatrix.map { $0.doubleValue },
                distortion: calibResult.rightDistortionCoeffs.map { $0.doubleValue },
                reprojError: calibResult.rightReprojectionError,
                width: Int(calibResult.imageWidth),
                height: Int(calibResult.imageHeight)
            )
            
            stereoExtrinsics = StereoExtrinsics(
                rotationMatrix: calibResult.rotationMatrix.map { $0.doubleValue },
                translationVector: calibResult.translationVector.map { $0.doubleValue },
                stereoReprojectionError: calibResult.stereoReprojectionError
            )
        }
        
        let appVersion = Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "1.0"
        
        let calibrationData = CameraCalibrationData(
            deviceId: deviceId,
            deviceName: deviceName,
            isStereo: isStereo,
            leftIntrinsics: leftIntrinsics,
            rightIntrinsics: rightIntrinsics,
            stereoExtrinsics: stereoExtrinsics,
            checkerboardConfig: checkerboardConfig,
            calibrationDate: Date(),
            sampleCount: samplesCollected,
            appVersion: appVersion
        )
        
        // Save calibration
        allCalibrations[deviceId] = calibrationData
        currentCalibration = calibrationData
        saveAllCalibrations()
        
        statusMessage = "Calibration complete!"
        isCalibrating = false
        
        print("✅ [CameraCalibrationManager] Calibration saved for: \(deviceName)")
        print("   Left reproj error: \(leftIntrinsics.reprojectionError)")
        if let right = rightIntrinsics {
            print("   Right reproj error: \(right.reprojectionError)")
        }
        if let stereo = stereoExtrinsics {
            print("   Stereo reproj error: \(stereo.stereoReprojectionError)")
        }
        
        return calibrationData
    }
    
    /// Cancels the current calibration session
    func cancelCalibration() {
        calibrator?.clearSamples()
        calibrator = nil
        isCalibrating = false
        calibrationProgress = 0.0
        samplesCollected = 0
        statusMessage = ""
        lastCornersLeft = nil
        lastCornersRight = nil
        print("📷 [CameraCalibrationManager] Calibration cancelled")
    }
    
    // MARK: - Export/Import
    
    /// Exports calibration data to a JSON file
    func exportCalibration(_ calibration: CameraCalibrationData) -> Data? {
        let encoder = JSONEncoder()
        encoder.outputFormatting = .prettyPrinted
        encoder.dateEncodingStrategy = .iso8601
        return try? encoder.encode(calibration)
    }
    
    /// Imports calibration data from JSON
    func importCalibration(from data: Data) -> CameraCalibrationData? {
        let decoder = JSONDecoder()
        decoder.dateDecodingStrategy = .iso8601
        guard let calibration = try? decoder.decode(CameraCalibrationData.self, from: data) else {
            return nil
        }
        
        allCalibrations[calibration.deviceId] = calibration
        saveAllCalibrations()
        return calibration
    }
}

// MARK: - Calibration Data Extension for Recording

extension CameraCalibrationData {
    /// Converts to a dictionary for inclusion in recording metadata
    func toMetadataDictionary() -> [String: Any] {
        var dict: [String: Any] = [
            "deviceId": deviceId,
            "deviceName": deviceName,
            "isStereo": isStereo,
            "calibrationDate": ISO8601DateFormatter().string(from: calibrationDate),
            "sampleCount": sampleCount,
            "appVersion": appVersion,
            "checkerboard": [
                "innerCornersX": checkerboardConfig.innerCornersX,
                "innerCornersY": checkerboardConfig.innerCornersY,
                "squareSize": checkerboardConfig.squareSize
            ]
        ]
        
        // Left/Mono intrinsics
        dict["leftIntrinsics"] = [
            "fx": leftIntrinsics.fx,
            "fy": leftIntrinsics.fy,
            "cx": leftIntrinsics.cx,
            "cy": leftIntrinsics.cy,
            "distortionCoeffs": leftIntrinsics.distortionCoeffs,
            "reprojectionError": leftIntrinsics.reprojectionError,
            "imageWidth": leftIntrinsics.imageWidth,
            "imageHeight": leftIntrinsics.imageHeight
        ]
        
        // Right intrinsics (stereo only)
        if let right = rightIntrinsics {
            dict["rightIntrinsics"] = [
                "fx": right.fx,
                "fy": right.fy,
                "cx": right.cx,
                "cy": right.cy,
                "distortionCoeffs": right.distortionCoeffs,
                "reprojectionError": right.reprojectionError,
                "imageWidth": right.imageWidth,
                "imageHeight": right.imageHeight
            ]
        }
        
        // Stereo extrinsics
        if let stereo = stereoExtrinsics {
            dict["stereoExtrinsics"] = [
                "rotationMatrix": stereo.rotationMatrix,
                "translationVector": stereo.translationVector,
                "stereoReprojectionError": stereo.stereoReprojectionError
            ]
        }
        
        return dict
    }
}
