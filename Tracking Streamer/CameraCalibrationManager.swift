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

/// ChArUco board configuration used for calibration
struct CharucoConfig: Codable, Equatable {
    /// Number of squares horizontally
    var squaresX: Int = 3
    /// Number of squares vertically
    var squaresY: Int = 4
    /// Physical size of one square in meters
    var squareSize: Float = 0.020
    /// Physical size of the marker in meters
    var markerSize: Float = 0.015
    /// ArUco dictionary type
    var dictionary: Int = 0 // Map to ArucoDictionaryType
}

/// Calibration mode
enum CalibrationMode: String, Codable {
    case checkerboard
    case charuco
}

// MARK: - Checkerboard Detection Feedback

/// Feedback about the current checkerboard detection state for UI
struct CheckerboardDetectionFeedback {
    /// Whether checkerboard was detected in the current frame
    var isDetected: Bool = false
    /// Number of corners detected (should match innerCornersX * innerCornersY when fully detected)
    var detectedCornerCount: Int = 0
    /// Estimated average corner spacing in pixels (smaller = further away)
    var averageCornerSpacing: Float = 0
    /// Whether the checkerboard appears too far/small for reliable calibration
    var isTooFar: Bool = false
    /// Whether this is for ChArUco mode (affects messages)
    var isCharucoMode: Bool = false
    /// Minimum recommended corner spacing threshold (pixels)
    static let minCornerSpacing: Float = 20.0
    /// User guidance message
    var guidanceMessage: String {
        let patternName = isCharucoMode ? "ChArUco" : "checkerboard"
        if !isDetected {
            return "Looking for \(patternName)..."
        } else if isTooFar {
            return "Move closer - \(patternName) is too small"
        } else {
            return "\(patternName.capitalized) detected!"
        }
    }
}

// MARK: - Camera Calibration Manager

/// Manages camera calibration data storage and retrieval
@MainActor
class CameraCalibrationManager: ObservableObject {
    static let shared = CameraCalibrationManager()
    
    // MARK: - Published Properties
    
    /// Current calibration mode
    @Published var calibrationMode: CalibrationMode = .charuco // Default to ChArUco now? Or keep checkerboard as default? User chose ChArUco. Enforcing .charuco for now logic.

    
    /// Currently loaded calibration data (for selected camera)
    @Published var currentCalibration: CameraCalibrationData? = nil
    
    /// All available calibrations
    @Published var allCalibrations: [String: CameraCalibrationData] = [:]
    
    /// Whether calibration is in progress
    @Published var isCalibrating: Bool = false
    
    /// Current calibration progress (0-1)
    @Published var calibrationProgress: Float = 0.0
    
    /// Number of samples collected so far (for mono calibration)
    @Published var samplesCollected: Int = 0
    
    /// Number of left samples collected (for stereo calibration)
    @Published var leftSamplesCollected: Int = 0
    
    /// Number of right samples collected (for stereo calibration)
    @Published var rightSamplesCollected: Int = 0
    
    /// Status message during calibration
    @Published var statusMessage: String = ""
    
    /// Last calibration error (if any)
    @Published var lastError: String? = nil
    
    /// Preview frame for calibration wizard UI (throttled UIImage from UVC stream)
    @Published var previewFrame: UIImage? = nil
    private var previewFrameCounter: Int = 0
    
    /// Preview frame with checkerboard visualization overlay
    @Published var previewWithVisualization: UIImage? = nil
    
    /// Current detection feedback for UI display (mono or left side for stereo)
    @Published var detectionFeedback: CheckerboardDetectionFeedback = .init()
    
    /// Detection feedback for right side (stereo only)
    @Published var rightDetectionFeedback: CheckerboardDetectionFeedback = .init()
    
    // MARK: - Real-time Calibration (updated during collection)
    
    /// Live intrinsic estimates for left camera (updated every N samples)
    @Published var liveLeftIntrinsics: CameraIntrinsics?
    /// Live intrinsic estimates for right camera (updated every N samples)
    @Published var liveRightIntrinsics: CameraIntrinsics?
    /// Stability percentage for left camera (0-100% based on fx coefficient of variation)
    @Published var leftStability: Float = 0
    /// Stability percentage for right camera (0-100%)
    @Published var rightStability: Float = 0
    /// Whether left camera is ready (stability > 90% and enough samples)
    @Published var leftReady: Bool = false
    /// Whether right camera is ready
    @Published var rightReady: Bool = false
    
    /// Last time intrinsics were updated (for UI display)
    @Published var lastIntrinsicsUpdateTime: Date?
    /// Sample count when intrinsics were last updated
    @Published var lastIntrinsicsSampleCount: Int = 0
    
    /// fx history for convergence calculation
    private var leftFxHistory: [Double] = []
    private var rightFxHistory: [Double] = []
    private let convergenceWindow = 10
    private let minSamplesForCalibration = 5
    
    // MARK: - Configuration
    
    /// Minimum samples required for calibration
    var minSamples: Int = 20
    
    /// Current checkerboard configuration
    @Published var checkerboardConfig = CheckerboardConfig(
        innerCornersX: 11,  // 12 rows (vertical)
        innerCornersY: 5,   // 6 columns (horizontal)
        squareSize: 0.010   // 10mm squares
    )
    
    /// Current ChArUco configuration
    @Published var charucoConfig = CharucoConfig()
    
    /// Minimum time between auto-captures (seconds)
    var minCaptureInterval: TimeInterval = 0.7
    
    /// Minimum corner movement for auto-capture (pixels)
    var minCornerMovement: Float = 15.0
    
    // MARK: - Private Properties
    
    private let storageKey = "cameraCalibrations"
    private var calibrator: OpenCVCalibrator?
    /// Separate calibrator for right camera (stereo only)
    private var rightCalibrator: OpenCVCalibrator?
    private var lastCaptureTime: Date = .distantPast
    /// Separate capture time for left side (stereo independent collection)
    private var lastLeftCaptureTime: Date = .distantPast
    /// Separate capture time for right side (stereo independent collection)
    private var lastRightCaptureTime: Date = .distantPast
    private var lastCornersLeft: [NSValue]? = nil
    private var lastCornersRight: [NSValue]? = nil
    
    /// Frame counter for detection throttling (only detect every Nth frame)
    private var detectionFrameCounter: Int = 0
    private let detectionFrameSkip: Int = 2  // Only detect every 2nd frame
    
    // MARK: - Initialization
    
    private init() {
        loadAllCalibrations()
    }
    
    // MARK: - Storage
    
    /// Loads all calibrations from UserDefaults
    private func loadAllCalibrations() {
        guard let data = UserDefaults.standard.data(forKey: storageKey) else {
            dlog("📷 [CameraCalibrationManager] No saved calibrations found")
            return
        }
        
        do {
            let decoder = JSONDecoder()
            allCalibrations = try decoder.decode([String: CameraCalibrationData].self, from: data)
            dlog("📷 [CameraCalibrationManager] Loaded \(allCalibrations.count) calibration(s)")
        } catch {
            dlog("❌ [CameraCalibrationManager] Failed to load calibrations: \(error)")
        }
    }
    
    /// Saves all calibrations to UserDefaults
    private func saveAllCalibrations() {
        do {
            let encoder = JSONEncoder()
            let data = try encoder.encode(allCalibrations)
            UserDefaults.standard.set(data, forKey: storageKey)
            dlog("📷 [CameraCalibrationManager] Saved \(allCalibrations.count) calibration(s)")
            
            // Also sync to iCloud for iPhone app
            syncToiCloud()
        } catch {
            dlog("❌ [CameraCalibrationManager] Failed to save calibrations: \(error)")
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
            dlog("☁️ [CameraCalibrationManager] Synced \(results.count) calibration(s) to iCloud")
        } catch {
            dlog("❌ [CameraCalibrationManager] Failed to sync to iCloud: \(error)")
        }
    }
    
    // MARK: - Calibration Loading
    
    /// Loads calibration for a specific camera device
    /// - Parameter deviceId: The unique identifier for the camera
    /// - Returns: The calibration data if available
    func loadCalibration(for deviceId: String) -> CameraCalibrationData? {
        if let calibration = allCalibrations[deviceId] {
            currentCalibration = calibration
            dlog("📷 [CameraCalibrationManager] Loaded calibration for device: \(calibration.deviceName)")
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
        dlog("📷 [CameraCalibrationManager] Deleted calibration for device: \(deviceId)")
    }
    
    // MARK: - Calibration Process
    
    /// Starts a new calibration session
    /// - Parameters:
    ///   - deviceId: Unique identifier for the camera
    ///   - deviceName: Human-readable name for the camera
    ///   - isStereo: Whether this is a stereo camera
    func startCalibration(deviceId: String, deviceName: String, isStereo: Bool) {
        dlog("📷 [CameraCalibrationManager] ========== START CALIBRATION ==========")
        dlog("📷 [CameraCalibrationManager] Device: \(deviceName) (id: \(deviceId))")
        dlog("📷 [CameraCalibrationManager] Stereo: \(isStereo)")
        dlog("📷 [CameraCalibrationManager] Calibration Mode: \(calibrationMode.rawValue)")
        if calibrationMode == .checkerboard {
            dlog("📷 [CameraCalibrationManager] Checkerboard config: \(checkerboardConfig.innerCornersX)x\(checkerboardConfig.innerCornersY), square: \(checkerboardConfig.squareSize)m")
        } else {
            dlog("📷 [CameraCalibrationManager] ChArUco config: \(charucoConfig.squaresX)x\(charucoConfig.squaresY), square: \(charucoConfig.squareSize)m, marker: \(charucoConfig.markerSize)m")
        }
        
        // Create left/main calibrator
        if calibrationMode == .checkerboard {
            calibrator = OpenCVCalibrator(
                checkerboardCornersX: Int32(checkerboardConfig.innerCornersX),
                cornersY: Int32(checkerboardConfig.innerCornersY),
                squareSize: checkerboardConfig.squareSize
            )
        } else {
            // ChArUco
            calibrator = OpenCVCalibrator(
                charucoSquaresX: Int32(charucoConfig.squaresX),
                squaresY: Int32(charucoConfig.squaresY),
                squareSize: charucoConfig.squareSize,
                markerSize: charucoConfig.markerSize,
                dictionary: .dict4X4_50 // Assuming ArucoDictionaryType maps to this. If not, will need to fix compilation error.
            )
        }
        
        // Create right calibrator for stereo (separate from left)
        if isStereo {
            if calibrationMode == .checkerboard {
                rightCalibrator = OpenCVCalibrator(
                    checkerboardCornersX: Int32(checkerboardConfig.innerCornersX),
                    cornersY: Int32(checkerboardConfig.innerCornersY),
                    squareSize: checkerboardConfig.squareSize
                )
            } else {
                rightCalibrator = OpenCVCalibrator(
                    charucoSquaresX: Int32(charucoConfig.squaresX),
                    squaresY: Int32(charucoConfig.squaresY),
                    squareSize: charucoConfig.squareSize,
                    markerSize: charucoConfig.markerSize,
                    dictionary: .dict4X4_50
                )
            }
        } else {
            rightCalibrator = nil
        }
        

        
        dlog("📷 [CameraCalibrationManager] OpenCVCalibrator created: \(calibrator != nil), right: \(rightCalibrator != nil)")
        
        isCalibrating = true
        calibrationProgress = 0.0
        samplesCollected = 0
        leftSamplesCollected = 0
        rightSamplesCollected = 0
        lastError = nil
        lastCaptureTime = .distantPast
        lastLeftCaptureTime = .distantPast
        lastRightCaptureTime = .distantPast
        lastCornersLeft = nil
        lastCornersRight = nil
        statusMessage = "Move the checkerboard around..."
        detectionFeedback = .init()
        rightDetectionFeedback = .init()
        
        // Reset live calibration tracking
        liveLeftIntrinsics = nil
        liveRightIntrinsics = nil
        leftStability = 0
        rightStability = 0
        leftReady = false
        rightReady = false
        leftFxHistory = []
        rightFxHistory = []
        
        dlog("📷 [CameraCalibrationManager] isCalibrating set to: \(isCalibrating)")
        dlog("📷 [CameraCalibrationManager] ========== START CALIBRATION COMPLETE ==========")
    }
    
    /// Processes a stereo frame for calibration - collects left and right samples INDEPENDENTLY
    /// - Parameter pixelBuffer: Side-by-side stereo frame
    /// - Returns: Detection result with visualization
    func processStereoFrame(_ pixelBuffer: CVPixelBuffer) -> CheckerboardDetection? {
        if calibrationMode == .charuco {
            return processStereoFrameCharuco(pixelBuffer)
        }
        guard isCalibrating, let leftCalibrator = calibrator, let rightCalibrator = rightCalibrator else { return nil }
        
        // Frame skip to reduce CPU load - only detect every Nth frame
        detectionFrameCounter += 1
        guard detectionFrameCounter % detectionFrameSkip == 0 else {
            return nil  // Skip this frame for detection
        }
        
        let detection = leftCalibrator.detectCheckerboard(stereoFrame: pixelBuffer)
        
        let leftCorners = detection?.leftCorners
        let rightCorners = detection?.rightCorners
        let foundLeft = detection?.foundLeft ?? false
        let foundRight = detection?.foundRight ?? false
        
        // Calculate corner spacing for size validation
        let leftSpacing = leftCorners.map { calculateAverageCornerSpacing($0) } ?? 0
        let rightSpacing = rightCorners.map { calculateAverageCornerSpacing($0) } ?? 0
        let leftTooFar = leftSpacing < CheckerboardDetectionFeedback.minCornerSpacing
        let rightTooFar = rightSpacing < CheckerboardDetectionFeedback.minCornerSpacing
        
        // Generate preview frame for wizard UI (throttled to every 3rd frame for performance)
        previewFrameCounter += 1
        if previewFrameCounter % 3 == 0 {
            Task { @MainActor in
                // Create basic preview
                if let image = self.pixelBufferToUIImage(pixelBuffer) {
                    self.previewFrame = image
                }
                
                // Create visualized preview with corners drawn on BOTH halves
                if let visualized = self.createStereoVisualizedPreview(pixelBuffer, leftCorners: leftCorners, rightCorners: rightCorners, foundLeft: foundLeft, foundRight: foundRight) {
                    self.previewWithVisualization = visualized
                }
                
                // Update detection feedback for left (with size check)
                var leftFeedback = CheckerboardDetectionFeedback()
                leftFeedback.isDetected = foundLeft
                leftFeedback.detectedCornerCount = leftCorners?.count ?? 0
                leftFeedback.averageCornerSpacing = leftSpacing
                leftFeedback.isTooFar = leftTooFar
                self.detectionFeedback = leftFeedback
                
                // Update detection feedback for right (with size check)
                var rightFeedback = CheckerboardDetectionFeedback()
                rightFeedback.isDetected = foundRight
                rightFeedback.detectedCornerCount = rightCorners?.count ?? 0
                rightFeedback.averageCornerSpacing = rightSpacing
                rightFeedback.isTooFar = rightTooFar
                self.rightDetectionFeedback = rightFeedback
            }
        }
        
        let now = Date()
        let width = CVPixelBufferGetWidth(pixelBuffer) / 2
        let height = CVPixelBufferGetHeight(pixelBuffer)
        
        // ========== PROCESS LEFT SIDE INDEPENDENTLY ==========
        if foundLeft, let corners = leftCorners {  // Removed tooFar check - user decides quality
            let timeSinceLastLeft = now.timeIntervalSince(lastLeftCaptureTime)
            let movement = lastCornersLeft.map { leftCalibrator.meanCornerMovement(from: $0, to: corners) } ?? Float.infinity
            
            if timeSinceLastLeft >= minCaptureInterval && movement >= minCornerMovement {
                // Add sample to LEFT calibrator (mono sample)
                leftSamplesCollected = Int(leftCalibrator.addMonoCalibrationSample(
                    corners: corners,
                    width: Int32(width),
                    height: Int32(height)
                ))
                
                lastLeftCaptureTime = now
                lastCornersLeft = corners
                
                dlog("📷 [CameraCalibrationManager] LEFT sample #\(leftSamplesCollected) captured")
                
                // Trigger incremental calibration in background
                triggerIncrementalCalibration(isLeft: true)
            }
        }
        
        // ========== PROCESS RIGHT SIDE INDEPENDENTLY ==========
        if foundRight, let corners = rightCorners {  // Removed tooFar check - user decides quality
            let timeSinceLastRight = now.timeIntervalSince(lastRightCaptureTime)
            let movement = lastCornersRight.map { rightCalibrator.meanCornerMovement(from: $0, to: corners) } ?? Float.infinity
            
            if timeSinceLastRight >= minCaptureInterval && movement >= minCornerMovement {
                // Add sample to RIGHT calibrator (mono sample)
                rightSamplesCollected = Int(rightCalibrator.addMonoCalibrationSample(
                    corners: corners,
                    width: Int32(width),
                    height: Int32(height)
                ))
                
                lastRightCaptureTime = now
                lastCornersRight = corners
                
                dlog("📷 [CameraCalibrationManager] RIGHT sample #\(rightSamplesCollected) captured")
                
                // Trigger incremental calibration in background
                triggerIncrementalCalibration(isLeft: false)
            }
        }
        
        // Update overall progress and status (use minimum of both sides)
        let minSidesSamples = min(leftSamplesCollected, rightSamplesCollected)
        samplesCollected = minSidesSamples
        calibrationProgress = Float(minSidesSamples) / Float(minSamples)
        
        // Build status message showing both sides
        var statusParts: [String] = []
        if !foundLeft {
            statusParts.append("L: not found")
        } else if leftTooFar {
            statusParts.append("L: too far")
        } else {
            statusParts.append("L: \(leftSamplesCollected)")
        }
        
        if !foundRight {
            statusParts.append("R: not found")
        } else if rightTooFar {
            statusParts.append("R: too far")
        } else {
            statusParts.append("R: \(rightSamplesCollected)")
        }
        statusMessage = statusParts.joined(separator: " | ")
        
        // Record debug frame periodically
        if CalibrationDebugRecorder.shared.isRecording && (leftSamplesCollected + rightSamplesCollected) % 5 == 0 {
            CalibrationDebugRecorder.shared.recordIntrinsicFrame(pixelBuffer)
        }
        
        return detection
    }
    
    /// Processes a mono frame for calibration
    /// - Parameter pixelBuffer: Mono camera frame
    /// - Returns: Detection result with visualization
    func processMonoFrame(_ pixelBuffer: CVPixelBuffer) -> CheckerboardDetection? {
        if calibrationMode == .charuco {
            return processMonoFrameCharuco(pixelBuffer)
        }
        dlog("🖼️ [CameraCalibrationManager] processMonoFrame called")
        dlog("🖼️ [CameraCalibrationManager] isCalibrating: \(isCalibrating), calibrator: \(calibrator != nil)")
        
        guard isCalibrating, let calibrator = calibrator else {
            dlog("❌ [CameraCalibrationManager] Guard failed - isCalibrating: \(isCalibrating), calibrator: \(calibrator != nil)")
            return nil
        }
        
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        dlog("🖼️ [CameraCalibrationManager] Frame size: \(width)x\(height)")
        
        dlog("🔍 [CameraCalibrationManager] Calling detectCheckerboard...")
        let detection = calibrator.detectCheckerboard(monoFrame: pixelBuffer)
        dlog("🔍 [CameraCalibrationManager] Detection result: \(detection != nil), foundLeft: \(detection?.foundLeft ?? false)")
        
        let corners = detection?.leftCorners
        let found = detection?.foundLeft ?? false
        
        // Calculate corner spacing for size validation
        let cornerSpacing = corners.map { calculateAverageCornerSpacing($0) } ?? 0
        let isTooFar = cornerSpacing < CheckerboardDetectionFeedback.minCornerSpacing
        
        // Generate preview frame for wizard UI (throttled to every 3rd frame for performance)
        previewFrameCounter += 1
        if previewFrameCounter % 3 == 0 {
            Task { @MainActor in
                // Create basic preview
                if let image = self.pixelBufferToUIImage(pixelBuffer) {
                    self.previewFrame = image
                }
                
                // Create visualized preview with corners drawn
                if let visualized = self.createVisualizedPreview(pixelBuffer, corners: corners, found: found) {
                    self.previewWithVisualization = visualized
                }
                
                // Update detection feedback with size information
                var feedback = CheckerboardDetectionFeedback()
                feedback.isDetected = found
                feedback.detectedCornerCount = corners?.count ?? 0
                feedback.averageCornerSpacing = cornerSpacing
                feedback.isTooFar = isTooFar
                self.detectionFeedback = feedback
            }
        }
        
        guard let result = detection, result.foundLeft else {
            statusMessage = "Looking for checkerboard..."
            return detection
        }
        
        // Note: tooFar check removed - user decides sample quality via stability %
        // The reprojection error will naturally increase if samples are too small
        
        // Check if enough time has passed
        let now = Date()
        let timeSinceLastCapture = now.timeIntervalSince(lastCaptureTime)
        
        guard timeSinceLastCapture >= minCaptureInterval else {
            statusMessage = "Hold still..."
            return detection
        }
        
        // Check if the pose has changed enough
        guard let validCorners = result.leftCorners else {
            return detection
        }
        
        let movement = lastCornersLeft.map { calibrator.meanCornerMovement(from: $0, to: validCorners) } ?? Float.infinity
        
        guard movement >= minCornerMovement else {
            statusMessage = "Move checkerboard to a new position..."
            return detection
        }
        
        // Add calibration sample (reuse width/height from above)
        samplesCollected = Int(calibrator.addMonoCalibrationSample(
            corners: validCorners,
            width: Int32(width),
            height: Int32(height)
        ))
        
        // Record debug frame when sample is actually collected
        if CalibrationDebugRecorder.shared.isRecording {
            CalibrationDebugRecorder.shared.recordIntrinsicFrame(pixelBuffer)
        }
        
        lastCaptureTime = now
        lastCornersLeft = validCorners
        
        calibrationProgress = Float(samplesCollected) / Float(minSamples)
        statusMessage = "Captured sample \(samplesCollected)/\(minSamples)"
        
        dlog("📷 [CameraCalibrationManager] Auto-captured sample #\(samplesCollected)")
        
        return detection
    }
    
    /// Finishes calibration and computes the intrinsics (async version - runs heavy computation on background thread)
    /// - Parameters:
    ///   - deviceId: Unique identifier for the camera
    ///   - deviceName: Human-readable name for the camera
    ///   - isStereo: Whether this is a stereo camera
    /// - Returns: The computed calibration data, or nil on failure
    func finishCalibrationAsync(deviceId: String, deviceName: String, isStereo: Bool) async -> CameraCalibrationData? {
        guard let leftCalibrator = calibrator else {
            lastError = "No calibrator available"
            isCalibrating = false
            return nil
        }
        
        statusMessage = "Computing calibration..."
        
        // Capture minSamples before entering detached task
        let minSamplesCount = Int32(minSamples)
        
        if isStereo {
            // For stereo: run mono calibration on each camera independently
            guard let rightCal = rightCalibrator else {
                lastError = "No right calibrator available for stereo"
                isCalibrating = false
                return nil
            }
            
            // Run both calibrations in parallel
            let leftResult: StereoCalibrationResult? = await Task.detached(priority: .userInitiated) {
                return leftCalibrator.performMonoCalibration(minSamples: minSamplesCount, outlierRejection: true)
            }.value
            
            let rightResult: StereoCalibrationResult? = await Task.detached(priority: .userInitiated) {
                return rightCal.performMonoCalibration(minSamples: minSamplesCount, outlierRejection: true)
            }.value
            
            guard let leftCalibResult = leftResult, leftCalibResult.success else {
                lastError = leftResult?.errorMessage ?? "Left calibration failed"
                statusMessage = "Left calibration failed"
                isCalibrating = false
                return nil
            }
            
            guard let rightCalibResult = rightResult, rightCalibResult.success else {
                lastError = rightResult?.errorMessage ?? "Right calibration failed"
                statusMessage = "Right calibration failed"
                isCalibrating = false
                return nil
            }
            
            // Build calibration data from independent calibrations
            let leftIntrinsics = CameraIntrinsics(
                from: leftCalibResult.leftIntrinsicMatrix.map { $0.doubleValue },
                distortion: leftCalibResult.leftDistortionCoeffs.map { $0.doubleValue },
                reprojError: leftCalibResult.leftReprojectionError,
                width: Int(leftCalibResult.imageWidth),
                height: Int(leftCalibResult.imageHeight)
            )
            
            let rightIntrinsics = CameraIntrinsics(
                from: rightCalibResult.leftIntrinsicMatrix.map { $0.doubleValue },  // Right uses its own mono result
                distortion: rightCalibResult.leftDistortionCoeffs.map { $0.doubleValue },
                reprojError: rightCalibResult.leftReprojectionError,
                width: Int(rightCalibResult.imageWidth),
                height: Int(rightCalibResult.imageHeight)
            )
            
            // No stereo extrinsics since we calibrated independently
            // User can rely on ARKit for stereo geometry if needed
            let stereoExtrinsics: StereoExtrinsics? = nil
            
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
                sampleCount: min(leftSamplesCollected, rightSamplesCollected),
                appVersion: appVersion
            )
            
            // Save and update state
            allCalibrations[deviceId] = calibrationData
            currentCalibration = calibrationData
            saveAllCalibrations()
            
            statusMessage = "Calibration complete!"
            isCalibrating = false
            
            dlog("📷 [CameraCalibrationManager] STEREO Calibration complete - L reprojError: \(leftCalibResult.leftReprojectionError), R reprojError: \(rightCalibResult.leftReprojectionError)")
            
            return calibrationData
        } else {
            // Mono calibration (unchanged logic)
            let result: StereoCalibrationResult? = await Task.detached(priority: .userInitiated) {
                return leftCalibrator.performMonoCalibration(minSamples: minSamplesCount, outlierRejection: true)
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
            
            let appVersion = Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "1.0"
            
            let calibrationData = CameraCalibrationData(
                deviceId: deviceId,
                deviceName: deviceName,
                isStereo: false,
                leftIntrinsics: leftIntrinsics,
                rightIntrinsics: nil,
                stereoExtrinsics: nil,
                checkerboardConfig: checkerboardConfig,
                calibrationDate: Date(),
                sampleCount: samplesCollected,
                appVersion: appVersion
            )
            
            // Save and update state
            allCalibrations[deviceId] = calibrationData
            currentCalibration = calibrationData
            saveAllCalibrations()
            
            statusMessage = "Calibration complete!"
            isCalibrating = false
            
            dlog("📷 [CameraCalibrationManager] MONO Calibration complete - reprojError: \(calibResult.leftReprojectionError)")
            
            return calibrationData
        }
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
            result = calibrator.performMonoCalibration(minSamples: Int32(minSamples), outlierRejection: true)
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
        
        dlog("✅ [CameraCalibrationManager] Calibration saved for: \(deviceName)")
        dlog("   Left reproj error: \(leftIntrinsics.reprojectionError)")
        if let right = rightIntrinsics {
            dlog("   Right reproj error: \(right.reprojectionError)")
        }
        if let stereo = stereoExtrinsics {
            dlog("   Stereo reproj error: \(stereo.stereoReprojectionError)")
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
        dlog("📷 [CameraCalibrationManager] Calibration cancelled")
    }
    
    /// Convert CVPixelBuffer to UIImage for preview display
    private func pixelBufferToUIImage(_ pixelBuffer: CVPixelBuffer) -> UIImage? {
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let context = CIContext()
        guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else {
            return nil
        }
        return UIImage(cgImage: cgImage)
    }
    
    /// Calculate average corner spacing from detected corners
    /// This estimates how "big" the checkerboard appears in the image
    private func calculateAverageCornerSpacing(_ corners: [NSValue]) -> Float {
        guard corners.count >= 2 else { return 0 }
        
        var totalSpacing: Float = 0
        var count = 0
        
        // Calculate spacing between adjacent corners
        let cols = checkerboardConfig.innerCornersX
        
        for i in 0..<corners.count {
            // Horizontal neighbor
            if (i + 1) % cols != 0 && i + 1 < corners.count {
                let p1 = corners[i].cgPointValue
                let p2 = corners[i + 1].cgPointValue
                let dx = Float(p2.x - p1.x)
                let dy = Float(p2.y - p1.y)
                totalSpacing += sqrt(dx * dx + dy * dy)
                count += 1
            }
            // Vertical neighbor
            if i + cols < corners.count {
                let p1 = corners[i].cgPointValue
                let p2 = corners[i + cols].cgPointValue
                let dx = Float(p2.x - p1.x)
                let dy = Float(p2.y - p1.y)
                totalSpacing += sqrt(dx * dx + dy * dy)
                count += 1
            }
        }
        
        return count > 0 ? totalSpacing / Float(count) : 0
    }
    
    /// Create visualized preview image from detection
    private func createVisualizedPreview(_ pixelBuffer: CVPixelBuffer, corners: [NSValue]?, found: Bool) -> UIImage? {
        guard let calibrator = calibrator else { return nil }
        
        if let jpegData = calibrator.visualizeCheckerboard(pixelBuffer: pixelBuffer, corners: corners, found: found) {
            return UIImage(data: jpegData)
        }
        return pixelBufferToUIImage(pixelBuffer)
    }
    
    /// Create visualized preview image from stereo detection (both halves)
    private func createStereoVisualizedPreview(_ pixelBuffer: CVPixelBuffer, leftCorners: [NSValue]?, rightCorners: [NSValue]?, foundLeft: Bool, foundRight: Bool) -> UIImage? {
        guard let calibrator = calibrator else { return nil }
        
        if let jpegData = calibrator.visualizeStereoCheckerboard(pixelBuffer: pixelBuffer, leftCorners: leftCorners, rightCorners: rightCorners, foundLeft: foundLeft, foundRight: foundRight) {
            return UIImage(data: jpegData)
        }
        return pixelBufferToUIImage(pixelBuffer)
    }
    
    /// Update detection feedback based on current detection results
    private func updateDetectionFeedback(isDetected: Bool, corners: [NSValue]?) {
        var feedback = CheckerboardDetectionFeedback()
        feedback.isDetected = isDetected
        
        if let corners = corners {
            feedback.detectedCornerCount = corners.count
            feedback.averageCornerSpacing = calculateAverageCornerSpacing(corners)
            feedback.isTooFar = feedback.averageCornerSpacing < CheckerboardDetectionFeedback.minCornerSpacing
        }
        
        detectionFeedback = feedback
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
    
    // MARK: - Incremental Calibration (Background)
    
    /// Flags to prevent concurrent calibrations
    private var isLeftCalibrationRunning = false
    private var isRightCalibrationRunning = false
    
    /// Timestamps for timeout detection
    private var leftCalibrationStartTime: Date?
    private var rightCalibrationStartTime: Date?
    private let calibrationTimeout: TimeInterval = 10.0  // 10 second timeout
    
    /// Triggers background incremental calibration for a side
    private func triggerIncrementalCalibration(isLeft: Bool) {
        guard let cal = isLeft ? calibrator : rightCalibrator else { return }
        let sampleCount = isLeft ? leftSamplesCollected : rightSamplesCollected
        
        // Only run incremental calibration after minSamplesForCalibration samples
        guard sampleCount >= minSamplesForCalibration else { return }
        
        // Check if calibration is already running for this side
        if isLeft {
            // Check for timeout - if previous calibration has been running too long, reset the flag
            if isLeftCalibrationRunning, let startTime = leftCalibrationStartTime {
                if Date().timeIntervalSince(startTime) > calibrationTimeout {
                    dlog("⚠️ [CameraCalibrationManager] LEFT calibration timed out, resetting flag")
                    isLeftCalibrationRunning = false
                }
            }
            guard !isLeftCalibrationRunning else { return }
            isLeftCalibrationRunning = true
            leftCalibrationStartTime = Date()
        } else {
            // Check for timeout
            if isRightCalibrationRunning, let startTime = rightCalibrationStartTime {
                if Date().timeIntervalSince(startTime) > calibrationTimeout {
                    dlog("⚠️ [CameraCalibrationManager] RIGHT calibration timed out, resetting flag")
                    isRightCalibrationRunning = false
                }
            }
            guard !isRightCalibrationRunning else { return }
            isRightCalibrationRunning = true
            rightCalibrationStartTime = Date()
        }
        
        dlog("🔄 [CameraCalibrationManager] Starting \(isLeft ? "LEFT" : "RIGHT") calibration (samples: \(sampleCount))...")
        
        // Capture needed values for background task
        let minSamplesInt32 = Int32(minSamplesForCalibration)
        
        // Run calibration in background task
        Task.detached(priority: .background) { [weak self] in
            let result = cal.performMonoCalibration(minSamples: minSamplesInt32, outlierRejection: false)  // Fast: no outlier rejection for live updates
            
            guard let self = self else {
                dlog("⚠️ [CameraCalibrationManager] Self became nil during calibration")
                return
            }
            
            // Update on main actor
            await MainActor.run {
                // Always mark calibration as finished first
                if isLeft {
                    self.isLeftCalibrationRunning = false
                } else {
                    self.isRightCalibrationRunning = false
                }
                
                // Get current sample count (may have increased since we started)
                let currentSampleCount = isLeft ? self.leftSamplesCollected : self.rightSamplesCollected
                
                guard let result = result, result.success else {
                    dlog("⚠️ [CameraCalibrationManager] Calibration failed for \(isLeft ? "LEFT" : "RIGHT"), will retry if still calibrating")
                    // If still collecting samples, trigger another calibration
                    if self.isCalibrating {
                        self.triggerIncrementalCalibration(isLeft: isLeft)
                    }
                    return
                }
                
                // Build intrinsics from result
                let intrinsics = CameraIntrinsics(
                    from: result.leftIntrinsicMatrix.map { $0.doubleValue },
                    distortion: result.leftDistortionCoeffs.map { $0.doubleValue },
                    reprojError: result.leftReprojectionError,
                    width: Int(result.imageWidth),
                    height: Int(result.imageHeight)
                )
                
                // Track when intrinsics were updated - use current sample count, not captured one
                self.lastIntrinsicsUpdateTime = Date()
                self.lastIntrinsicsSampleCount = currentSampleCount
                
                dlog("📷 [CameraCalibrationManager] \(isLeft ? "LEFT" : "RIGHT") calibration complete: fx=\(String(format: "%.1f", intrinsics.fx)), err=\(String(format: "%.3f", intrinsics.reprojectionError)), samples=\(currentSampleCount)")
                
                if isLeft {
                    self.liveLeftIntrinsics = intrinsics
                    self.leftFxHistory.append(intrinsics.fx)
                    let (stability, ready) = self.computeStability(fxHistory: self.leftFxHistory, sampleCount: currentSampleCount)
                    self.leftStability = stability
                    self.leftReady = ready
                } else {
                    self.liveRightIntrinsics = intrinsics
                    self.rightFxHistory.append(intrinsics.fx)
                    let (stability, ready) = self.computeStability(fxHistory: self.rightFxHistory, sampleCount: currentSampleCount)
                    self.rightStability = stability
                    self.rightReady = ready
                }
                
                // Continuously trigger next calibration if still collecting
                if self.isCalibrating {
                    dlog("🔄 [CameraCalibrationManager] Triggering next \(isLeft ? "LEFT" : "RIGHT") calibration...")
                    self.triggerIncrementalCalibration(isLeft: isLeft)
                } else {
                    dlog("⏹️ [CameraCalibrationManager] Calibration stopped - not in calibrating state")
                }
            }
        }
    }
    
    /// Computes stability percentage from fx history using coefficient of variation
    private func computeStability(fxHistory: [Double], sampleCount: Int) -> (Float, Bool) {
        guard fxHistory.count >= 3 else { return (0, false) }
        
        // Use last convergenceWindow values
        let window = Array(fxHistory.suffix(convergenceWindow))
        guard window.count >= 3 else { return (0, false) }
        
        let mean = window.reduce(0, +) / Double(window.count)
        guard mean > 0 else { return (0, false) }
        
        let variance = window.map { pow($0 - mean, 2) }.reduce(0, +) / Double(window.count)
        let stdDev = sqrt(variance)
        let cv = (stdDev / mean) * 100  // Coefficient of variation as percentage
        
        // Convert CV to stability (lower CV = higher stability)
        // CV < 0.1% = 100% stability, CV > 2% = 0% stability
        let stability = max(0, min(100, (2.0 - cv) * 50))
        let ready = stability > 90 && sampleCount >= 20
        
        return (Float(stability), ready)
    }
    
    // MARK: - ChArUco Processing
    
    private func processStereoFrameCharuco(_ pixelBuffer: CVPixelBuffer) -> CheckerboardDetection? { // Returning nil usually
        guard isCalibrating, let leftCalibrator = calibrator, let rightCalibrator = rightCalibrator else { return nil }
        
        let detection = leftCalibrator.detectCharuco(stereoFrame: pixelBuffer)
        
        let leftCorners = detection?.leftCorners
        let rightCorners = detection?.rightCorners
        let leftIds = detection?.leftIds
        let rightIds = detection?.rightIds
        let foundLeft = detection?.foundLeft ?? false
        let foundRight = detection?.foundRight ?? false
        
        // Calculate spacing
        let leftSpacing = calculateAverageCornerSpacing(leftCorners ?? [])
        let rightSpacing = calculateAverageCornerSpacing(rightCorners ?? [])
        let leftTooFar = leftSpacing < CheckerboardDetectionFeedback.minCornerSpacing
        let rightTooFar = rightSpacing < CheckerboardDetectionFeedback.minCornerSpacing
        
        // Preview
        previewFrameCounter += 1
        if previewFrameCounter % 3 == 0 {
            Task { @MainActor in
                if let image = self.pixelBufferToUIImage(pixelBuffer) {
                    self.previewFrame = image
                }
                
                if let visualized = self.createStereoCharucoVisualizedPreview(pixelBuffer,
                                                                              leftCorners: leftCorners, leftIds: leftIds,
                                                                              rightCorners: rightCorners, rightIds: rightIds,
                                                                              foundLeft: foundLeft, foundRight: foundRight) {
                    self.previewWithVisualization = visualized
                }
                
                var leftFeedback = CheckerboardDetectionFeedback()
                leftFeedback.isDetected = foundLeft
                leftFeedback.detectedCornerCount = leftCorners?.count ?? 0
                leftFeedback.averageCornerSpacing = leftSpacing
                leftFeedback.isTooFar = leftTooFar
                leftFeedback.isCharucoMode = true
                self.detectionFeedback = leftFeedback
                
                var rightFeedback = CheckerboardDetectionFeedback()
                rightFeedback.isDetected = foundRight
                rightFeedback.detectedCornerCount = rightCorners?.count ?? 0
                rightFeedback.averageCornerSpacing = rightSpacing
                rightFeedback.isTooFar = rightTooFar
                rightFeedback.isCharucoMode = true
                self.rightDetectionFeedback = rightFeedback
            }
        }
        
        let now = Date()
        let width = CVPixelBufferGetWidth(pixelBuffer) / 2
        let height = CVPixelBufferGetHeight(pixelBuffer)
        
        // Process LEFT
        if foundLeft, let corners = leftCorners, let ids = leftIds, !leftTooFar, corners.count >= 4 {
            let timeSinceLast = now.timeIntervalSince(lastLeftCaptureTime)
            let movement = lastCornersLeft.map { leftCalibrator.meanCornerMovement(from: $0, to: corners) } ?? Float.infinity
            
            if timeSinceLast >= minCaptureInterval && movement >= minCornerMovement {
                leftSamplesCollected = Int(leftCalibrator.addMonoCharucoSample(corners: corners, ids: ids, width: Int32(width), height: Int32(height)))
                lastLeftCaptureTime = now
                lastCornersLeft = corners
                dlog("📷 [CameraCalibrationManager] LEFT ChArUco sample #\(leftSamplesCollected)")
            }
        }
        
        // Process RIGHT
        if foundRight, let corners = rightCorners, let ids = rightIds, !rightTooFar, corners.count >= 4 {
            let timeSinceLast = now.timeIntervalSince(lastRightCaptureTime)
            let movement = lastCornersRight.map { rightCalibrator.meanCornerMovement(from: $0, to: corners) } ?? Float.infinity
            
            if timeSinceLast >= minCaptureInterval && movement >= minCornerMovement {
                rightSamplesCollected = Int(rightCalibrator.addMonoCharucoSample(corners: corners, ids: ids, width: Int32(width), height: Int32(height)))
                lastRightCaptureTime = now
                lastCornersRight = corners
                dlog("📷 [CameraCalibrationManager] RIGHT ChArUco sample #\(rightSamplesCollected)")
            }
        }
        
        // Update Status
        let minSidesSamples = min(leftSamplesCollected, rightSamplesCollected)
        samplesCollected = minSidesSamples
        calibrationProgress = Float(minSidesSamples) / Float(minSamples)
        
        var statusParts: [String] = []
        statusParts.append(foundLeft ? "L: \(leftCorners?.count ?? 0) pts" : "L: not found")
        statusParts.append(foundRight ? "R: \(rightCorners?.count ?? 0) pts" : "R: not found")
        statusMessage = statusParts.joined(separator: " | ") + " [\(minSidesSamples)/\(minSamples)]"
        
        // Record debug
        if CalibrationDebugRecorder.shared.isRecording && (leftSamplesCollected + rightSamplesCollected) % 5 == 0 {
            CalibrationDebugRecorder.shared.recordIntrinsicFrame(pixelBuffer)
        }
        
        return nil // Return nil as we use custom CharucoDetection type not exposed here
    }
    
    private func processMonoFrameCharuco(_ pixelBuffer: CVPixelBuffer) -> CheckerboardDetection? {
        guard isCalibrating, let calibrator = calibrator else { return nil }
        
        let detection = calibrator.detectCharuco(monoFrame: pixelBuffer)
        let corners = detection?.leftCorners
        let ids = detection?.leftIds
        let found = detection?.foundLeft ?? false
        
        let spacing = calculateAverageCornerSpacing(corners ?? [])
        let isTooFar = spacing < CheckerboardDetectionFeedback.minCornerSpacing
        
        // Preview
        previewFrameCounter += 1
        if previewFrameCounter % 3 == 0 {
            Task { @MainActor in
                if let image = self.pixelBufferToUIImage(pixelBuffer) {
                    self.previewFrame = image
                }
                
                if let visualized = self.createCharucoVisualizedPreview(pixelBuffer, corners: corners, ids: ids, found: found) {
                    self.previewWithVisualization = visualized
                }
                
                var feedback = CheckerboardDetectionFeedback()
                feedback.isDetected = found
                feedback.detectedCornerCount = corners?.count ?? 0
                feedback.averageCornerSpacing = spacing
                feedback.isTooFar = isTooFar
                feedback.isCharucoMode = true
                self.detectionFeedback = feedback
            }
        }
        
        if !found {
            statusMessage = "Looking for ChArUco..."
            return nil
        }
        
        if isTooFar {
            statusMessage = "Too far..."
            return nil
        }
        
        let now = Date()
        let timeSinceLast = now.timeIntervalSince(lastCaptureTime)
        if timeSinceLast < minCaptureInterval {
            statusMessage = "Hold still..."
            return nil
        }
        
        let validCorners = corners ?? []
        let movement = lastCornersLeft.map { calibrator.meanCornerMovement(from: $0, to: validCorners) } ?? Float.infinity
        
        if movement < minCornerMovement {
            statusMessage = "Move board..."
            return nil
        }
        
        // Add sample
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        
        samplesCollected = Int(calibrator.addMonoCharucoSample(corners: validCorners, ids: ids ?? [], width: Int32(width), height: Int32(height)))
        
        lastCaptureTime = now
        lastCornersLeft = validCorners
        calibrationProgress = Float(samplesCollected) / Float(minSamples)
        statusMessage = "Captured \(samplesCollected)/\(minSamples)"
        
        dlog("📷 [CameraCalibrationManager] Auto-captured ChArUco sample #\(samplesCollected)")
        
        if CalibrationDebugRecorder.shared.isRecording {
            CalibrationDebugRecorder.shared.recordIntrinsicFrame(pixelBuffer)
        }
        
        return nil
    }
    
    private func createCharucoVisualizedPreview(_ pixelBuffer: CVPixelBuffer, corners: [NSValue]?, ids: [NSNumber]?, found: Bool) -> UIImage? {
        guard let calibrator = calibrator else { return nil }
        if let jpegData = calibrator.visualizeCharuco(pixelBuffer: pixelBuffer, corners: corners, ids: ids, found: found) {
            return UIImage(data: jpegData)
        }
        return pixelBufferToUIImage(pixelBuffer)
    }
    
    private func createStereoCharucoVisualizedPreview(_ pixelBuffer: CVPixelBuffer,
                                                      leftCorners: [NSValue]?, leftIds: [NSNumber]?,
                                                      rightCorners: [NSValue]?, rightIds: [NSNumber]?,
                                                      foundLeft: Bool, foundRight: Bool) -> UIImage? {
        guard let calibrator = calibrator else { return nil }
        if let jpegData = calibrator.visualizeStereoCharuco(pixelBuffer: pixelBuffer,
                                                            leftCorners: leftCorners, leftIds: leftIds,
                                                            rightCorners: rightCorners, rightIds: rightIds,
                                                            foundLeft: foundLeft, foundRight: foundRight) {
            return UIImage(data: jpegData)
        }
        return pixelBufferToUIImage(pixelBuffer)
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
