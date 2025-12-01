import Foundation
import SwiftUI
import simd
import ARKit
import Accelerate
import CoreImage

// MARK: - Extrinsic Calibration Data Models

/// Camera side for stereo calibration
enum CameraSide: String, Codable {
    case mono = "mono"
    case left = "left"
    case right = "right"
}

/// Represents a single calibration sample with paired observations
struct ExtrinsicCalibrationSample: Codable, Identifiable {
    let id: UUID
    let markerId: Int
    let timestamp: Date
    
    /// Which camera (mono, left, or right) this sample is from
    let cameraSide: CameraSide
    
    /// T_world^head: Head pose in ARKit world frame
    let headPoseInWorld: [Double]  // 16 elements, column-major 4x4
    
    /// T_world^marker: Marker pose in ARKit world frame (from ImageAnchor)
    let markerPoseInWorld: [Double]  // 16 elements, column-major 4x4
    
    /// T_camera^marker: Marker pose in camera frame (from OpenCV)
    let markerPoseInCamera: [Double]  // 16 elements, column-major 4x4
    
    init(markerId: Int, cameraSide: CameraSide = .mono, headPoseInWorld: simd_float4x4, markerPoseInWorld: simd_float4x4, markerPoseInCamera: simd_float4x4) {
        self.id = UUID()
        self.markerId = markerId
        self.cameraSide = cameraSide
        self.timestamp = Date()
        self.headPoseInWorld = Self.matrixToArray(headPoseInWorld)
        self.markerPoseInWorld = Self.matrixToArray(markerPoseInWorld)
        self.markerPoseInCamera = Self.matrixToArray(markerPoseInCamera)
    }
    
    private static func matrixToArray(_ m: simd_float4x4) -> [Double] {
        // Column-major order for simd
        return [
            Double(m.columns.0.x), Double(m.columns.0.y), Double(m.columns.0.z), Double(m.columns.0.w),
            Double(m.columns.1.x), Double(m.columns.1.y), Double(m.columns.1.z), Double(m.columns.1.w),
            Double(m.columns.2.x), Double(m.columns.2.y), Double(m.columns.2.z), Double(m.columns.2.w),
            Double(m.columns.3.x), Double(m.columns.3.y), Double(m.columns.3.z), Double(m.columns.3.w),
        ]
    }
    
    static func arrayToMatrix(_ arr: [Double]) -> simd_float4x4 {
        guard arr.count == 16 else { return matrix_identity_float4x4 }
        return simd_float4x4(
            SIMD4<Float>(Float(arr[0]), Float(arr[1]), Float(arr[2]), Float(arr[3])),
            SIMD4<Float>(Float(arr[4]), Float(arr[5]), Float(arr[6]), Float(arr[7])),
            SIMD4<Float>(Float(arr[8]), Float(arr[9]), Float(arr[10]), Float(arr[11])),
            SIMD4<Float>(Float(arr[12]), Float(arr[13]), Float(arr[14]), Float(arr[15]))
        )
    }
    
    var headPoseMatrix: simd_float4x4 { Self.arrayToMatrix(headPoseInWorld) }
    var markerInWorldMatrix: simd_float4x4 { Self.arrayToMatrix(markerPoseInWorld) }
    var markerInCameraMatrix: simd_float4x4 { Self.arrayToMatrix(markerPoseInCamera) }
}

/// Complete extrinsic calibration result
struct ExtrinsicCalibrationData: Codable, Equatable {
    /// Unique identifier for the camera device
    let cameraDeviceId: String
    /// Human-readable device name
    let cameraDeviceName: String
    /// Whether this is a stereo camera
    let isStereo: Bool
    /// The computed transform: T_head^camera (for mono) or T_head^left_camera (for stereo)
    let leftHeadToCamera: [Double]  // 16 elements, column-major 4x4
    /// The computed transform: T_head^right_camera (nil for mono cameras)
    let rightHeadToCamera: [Double]?  // 16 elements, column-major 4x4
    /// Reprojection error (RMS in meters) for left/mono camera
    let leftReprojectionError: Double
    /// Reprojection error (RMS in meters) for right camera (nil for mono)
    let rightReprojectionError: Double?
    /// Number of samples used for calibration (left camera)
    let leftSampleCount: Int
    /// Number of samples used for calibration (right camera, nil for mono)
    let rightSampleCount: Int?
    /// Number of unique markers used
    let markerCount: Int
    /// Date when calibration was performed
    let calibrationDate: Date
    /// App version when calibration was performed
    let appVersion: String
    /// ArUco dictionary used
    let arucoDictionary: String
    /// Physical marker size in meters
    let markerSizeMeters: Float
    
    // MARK: - Computed Properties for Backward Compatibility
    
    /// For mono cameras, this is the same as leftHeadToCamera. For stereo, use left/right explicitly.
    var headToCamera: [Double] { leftHeadToCamera }
    var headToCameraMatrix: simd_float4x4 { leftHeadToCameraMatrix }
    var reprojectionError: Double { leftReprojectionError }
    var sampleCount: Int { leftSampleCount + (rightSampleCount ?? 0) }
    
    var leftHeadToCameraMatrix: simd_float4x4 {
        ExtrinsicCalibrationSample.arrayToMatrix(leftHeadToCamera)
    }
    
    var rightHeadToCameraMatrix: simd_float4x4? {
        guard let right = rightHeadToCamera else { return nil }
        return ExtrinsicCalibrationSample.arrayToMatrix(right)
    }
    
    /// Converts to a dictionary for inclusion in recording metadata
    func toMetadataDictionary() -> [String: Any] {
        var dict: [String: Any] = [
            "cameraDeviceId": cameraDeviceId,
            "cameraDeviceName": cameraDeviceName,
            "isStereo": isStereo,
            "leftHeadToCamera": leftHeadToCamera,
            "leftReprojectionError": leftReprojectionError,
            "leftSampleCount": leftSampleCount,
            "markerCount": markerCount,
            "calibrationDate": ISO8601DateFormatter().string(from: calibrationDate),
            "appVersion": appVersion,
            "arucoDictionary": arucoDictionary,
            "markerSizeMeters": markerSizeMeters
        ]
        if let rightHeadToCamera = rightHeadToCamera {
            dict["rightHeadToCamera"] = rightHeadToCamera
        }
        if let rightReprojectionError = rightReprojectionError {
            dict["rightReprojectionError"] = rightReprojectionError
        }
        if let rightSampleCount = rightSampleCount {
            dict["rightSampleCount"] = rightSampleCount
        }
        return dict
    }
}

/// ArUco marker reference for ARKit image tracking
struct ArucoMarkerReference {
    let markerId: Int
    let physicalWidth: Float  // in meters
    let imageData: Data  // PNG image data
}

// MARK: - Extrinsic Calibration Manager

/// Manages extrinsic calibration between Vision Pro head and external camera
@MainActor
class ExtrinsicCalibrationManager: ObservableObject {
    static let shared = ExtrinsicCalibrationManager()
    
    // MARK: - Published Properties
    
    /// Current calibration data (for selected camera)
    @Published var currentCalibration: ExtrinsicCalibrationData? = nil
    
    /// All available calibrations
    @Published var allCalibrations: [String: ExtrinsicCalibrationData] = [:]
    
    /// Whether calibration is in progress
    @Published var isCalibrating: Bool = false
    
    /// Current calibration progress (0-1)
    @Published var calibrationProgress: Float = 0.0
    
    /// Number of samples collected so far (total for stereo, single for mono)
    @Published var samplesCollected: Int = 0
    
    /// Number of left camera samples (for stereo)
    @Published var leftSamplesCollected: Int = 0
    
    /// Number of right camera samples (for stereo)
    @Published var rightSamplesCollected: Int = 0
    
    /// Status message during calibration
    @Published var statusMessage: String = ""
    
    /// Last calibration error (if any)
    @Published var lastError: String? = nil
    
    /// Currently tracked markers from ARKit
    @Published var arkitTrackedMarkers: [Int: simd_float4x4] = [:]  // markerId -> T_world^marker
    
    /// Currently detected markers from camera (mono or left for stereo)
    @Published var cameraDetectedMarkers: [Int: simd_float4x4] = [:]  // markerId -> T_camera^marker
    
    /// Currently detected markers from right camera (stereo only)
    @Published var rightCameraDetectedMarkers: [Int: simd_float4x4] = [:]  // markerId -> T_camera^marker
    
    /// Current head pose from ARKit
    @Published var currentHeadPose: simd_float4x4 = matrix_identity_float4x4
    
    // MARK: - Configuration
    
    /// Minimum samples required for calibration (per camera for stereo)
    var minSamples: Int = 30
    
    /// ArUco dictionary type (using raw value: 0 = DICT_4X4_50)
    @Published var arucoDictionary: ArucoDictionaryType = ArucoDictionaryType(rawValue: 0)!
    
    /// Physical marker size in meters
    @Published var markerSizeMeters: Float = 0.11  // 110mm default (100mm marker + 10mm border)
    
    /// Marker IDs to track
    @Published var markerIds: [Int] = [0, 1, 2, 3]
    
    /// Minimum time between auto-captures (seconds)
    var minCaptureInterval: TimeInterval = 0.5
    
    /// Minimum head movement for auto-capture (meters)
    var minHeadMovement: Float = 0.05
    
    /// Whether we're calibrating a stereo camera
    private var isStereoCalibration: Bool = false
    
    // MARK: - Private Properties
    
    private let storageKey = "extrinsicCalibrations"
    private var samples: [ExtrinsicCalibrationSample] = []
    private var lastCaptureTime: Date = .distantPast
    private var lastHeadPosition: SIMD3<Float> = .zero
    private var arucoDetector: OpenCVArucoDetector?
    
    // ARKit session for image tracking
    private var arkitSession: ARKitSession?
    private var imageTrackingProvider: ImageTrackingProvider?
    
    // MARK: - Initialization
    
    private init() {
        loadAllCalibrations()
    }
    
    // MARK: - Storage
    
    private func loadAllCalibrations() {
        guard let data = UserDefaults.standard.data(forKey: storageKey) else {
            print("📐 [ExtrinsicCalibrationManager] No saved calibrations found")
            return
        }
        
        do {
            let decoder = JSONDecoder()
            allCalibrations = try decoder.decode([String: ExtrinsicCalibrationData].self, from: data)
            print("📐 [ExtrinsicCalibrationManager] Loaded \(allCalibrations.count) calibration(s)")
        } catch {
            print("❌ [ExtrinsicCalibrationManager] Failed to load calibrations: \(error)")
        }
    }
    
    private func saveAllCalibrations() {
        do {
            let encoder = JSONEncoder()
            let data = try encoder.encode(allCalibrations)
            UserDefaults.standard.set(data, forKey: storageKey)
            print("📐 [ExtrinsicCalibrationManager] Saved \(allCalibrations.count) calibration(s)")
        } catch {
            print("❌ [ExtrinsicCalibrationManager] Failed to save calibrations: \(error)")
        }
    }
    
    // MARK: - Calibration Loading
    
    func loadCalibration(for deviceId: String) -> ExtrinsicCalibrationData? {
        if let calibration = allCalibrations[deviceId] {
            currentCalibration = calibration
            print("📐 [ExtrinsicCalibrationManager] Loaded calibration for device: \(calibration.cameraDeviceName)")
            return calibration
        }
        currentCalibration = nil
        return nil
    }
    
    func hasCalibration(for deviceId: String) -> Bool {
        return allCalibrations[deviceId] != nil
    }
    
    func deleteCalibration(for deviceId: String) {
        allCalibrations.removeValue(forKey: deviceId)
        if currentCalibration?.cameraDeviceId == deviceId {
            currentCalibration = nil
        }
        saveAllCalibrations()
        print("📐 [ExtrinsicCalibrationManager] Deleted calibration for device: \(deviceId)")
    }
    
    // MARK: - ARKit Image Tracking Setup
    
    /// Generate reference images for ARKit from ArUco markers
    func generateReferenceImages() -> [ReferenceImage] {
        var referenceImages: [ReferenceImage] = []
        
        for markerId in markerIds {
            guard let imageData = OpenCVArucoDetector.generateMarkerImage(
                id: Int32(markerId),
                sizePixels: 1000,  // High resolution for ARKit
                dictionary: arucoDictionary
            ) else {
                print("❌ [ExtrinsicCalibrationManager] Failed to generate marker \(markerId)")
                continue
            }
            
            guard let cgImageSource = CGImageSourceCreateWithData(imageData as CFData, nil),
                  let cgImage = CGImageSourceCreateImageAtIndex(cgImageSource, 0, nil) else {
                print("❌ [ExtrinsicCalibrationManager] Failed to create CGImage for marker \(markerId)")
                continue
            }
            
            // Create ReferenceImage with physical size
            var refImage = ReferenceImage(
                cgimage: cgImage,
                physicalSize: CGSize(width: CGFloat(markerSizeMeters), height: CGFloat(markerSizeMeters)),
                orientation: .up
            )
            refImage.name = "aruco_\(markerId)"
            
            referenceImages.append(refImage)
            print("📐 [ExtrinsicCalibrationManager] Created reference image for marker \(markerId)")
        }
        
        return referenceImages
    }
    
    // MARK: - Calibration Process
    
    /// Starts a new extrinsic calibration session
    /// - Parameters:
    ///   - deviceId: Unique identifier for the camera device
    ///   - deviceName: Human-readable device name
    ///   - isStereo: Whether this is a stereo camera (side-by-side format)
    func startCalibration(deviceId: String, deviceName: String, isStereo: Bool) async {
        print("📐 [ExtrinsicCalibrationManager] ========== START EXTRINSIC CALIBRATION ==========")
        print("📐 [ExtrinsicCalibrationManager] Device: \(deviceName) (id: \(deviceId))")
        print("📐 [ExtrinsicCalibrationManager] Stereo: \(isStereo)")
        print("📐 [ExtrinsicCalibrationManager] Marker size: \(markerSizeMeters)m")
        print("📐 [ExtrinsicCalibrationManager] Marker IDs: \(markerIds)")
        
        // Store stereo mode
        isStereoCalibration = isStereo
        
        // Initialize ArUco detector
        arucoDetector = OpenCVArucoDetector(dictionary: arucoDictionary)
        
        // Reset state
        samples = []
        isCalibrating = true
        calibrationProgress = 0.0
        samplesCollected = 0
        leftSamplesCollected = 0
        rightSamplesCollected = 0
        lastError = nil
        lastCaptureTime = .distantPast
        lastHeadPosition = .zero
        arkitTrackedMarkers = [:]
        cameraDetectedMarkers = [:]
        rightCameraDetectedMarkers = [:]
        statusMessage = "Initializing ARKit image tracking..."
        
        // Setup ARKit image tracking
        do {
            let referenceImages = generateReferenceImages()
            guard !referenceImages.isEmpty else {
                lastError = "Failed to generate reference images"
                isCalibrating = false
                return
            }
            
            arkitSession = ARKitSession()
            imageTrackingProvider = ImageTrackingProvider(referenceImages: referenceImages)
            
            guard ImageTrackingProvider.isSupported else {
                lastError = "Image tracking is not supported on this device"
                isCalibrating = false
                return
            }
            
            try await arkitSession?.run([imageTrackingProvider!])
            
            // Start listening for anchor updates
            Task {
                await processImageAnchorUpdates()
            }
            
            statusMessage = "Point camera at ArUco markers..."
            print("📐 [ExtrinsicCalibrationManager] ARKit image tracking started")
            
        } catch {
            lastError = "Failed to start ARKit: \(error.localizedDescription)"
            isCalibrating = false
            print("❌ [ExtrinsicCalibrationManager] ARKit setup failed: \(error)")
        }
    }
    
    /// Process ARKit image anchor updates
    private func processImageAnchorUpdates() async {
        guard let provider = imageTrackingProvider else { return }
        
        for await update in provider.anchorUpdates {
            let anchor = update.anchor
            
            // Extract marker ID from reference image name
            guard let name = anchor.referenceImage.name,
                  name.hasPrefix("aruco_"),
                  let markerId = Int(name.dropFirst(6)) else {
                continue
            }
            
            await MainActor.run {
                if anchor.isTracked {
                    // Store T_world^marker
                    arkitTrackedMarkers[markerId] = anchor.originFromAnchorTransform
                } else {
                    arkitTrackedMarkers.removeValue(forKey: markerId)
                }
            }
        }
    }
    
    /// Update head pose from ARKit (called from HeadTrackingSystem)
    func updateHeadPose(_ headPose: simd_float4x4) {
        currentHeadPose = headPose
    }
    
    /// Process a camera frame for ArUco detection (mono camera or full stereo frame)
    /// - Parameters:
    ///   - pixelBuffer: The camera frame
    ///   - intrinsics: Left/mono camera intrinsics (nil for no pose estimation)
    ///   - rightIntrinsics: Right camera intrinsics for stereo (nil for mono)
    func processCameraFrame(_ pixelBuffer: CVPixelBuffer, intrinsics: CameraIntrinsics?, rightIntrinsics: CameraIntrinsics? = nil) {
        guard isCalibrating, let detector = arucoDetector else { return }
        
        if isStereoCalibration {
            // For stereo, split the frame and process each half
            processStereoFrame(pixelBuffer, leftIntrinsics: intrinsics, rightIntrinsics: rightIntrinsics)
        } else {
            // For mono, process the full frame
            processMonoFrame(pixelBuffer, intrinsics: intrinsics)
        }
        
        // Try to collect a sample if we have matching observations
        tryCollectSample()
    }
    
    /// Process a mono camera frame
    private func processMonoFrame(_ pixelBuffer: CVPixelBuffer, intrinsics: CameraIntrinsics?) {
        guard let detector = arucoDetector else { return }
        
        var detections: [ArucoDetectionResult]?
        
        if let intrinsics = intrinsics {
            // Build camera matrix (row-major for OpenCV)
            let cameraMatrix: [NSNumber] = [
                NSNumber(value: intrinsics.fx), NSNumber(value: 0), NSNumber(value: intrinsics.cx),
                NSNumber(value: 0), NSNumber(value: intrinsics.fy), NSNumber(value: intrinsics.cy),
                NSNumber(value: 0), NSNumber(value: 0), NSNumber(value: 1)
            ]
            let distCoeffs = intrinsics.distortionCoeffs.map { NSNumber(value: $0) }
            
            detections = detector.detectMarkers(
                in: pixelBuffer,
                cameraMatrix: cameraMatrix,
                distCoeffs: distCoeffs,
                markerLength: markerSizeMeters
            )
        } else {
            // Detect without pose estimation
            detections = detector.detectMarkers(in: pixelBuffer)
        }
        
        guard let results = detections else { return }
        
        // Update detected markers
        cameraDetectedMarkers.removeAll()
        for result in results {
            if result.poseValid, let transformArray = result.transformMatrix {
                let transform = arrayToSimdMatrix(transformArray.map { $0.doubleValue })
                cameraDetectedMarkers[Int(result.markerId)] = transform
            }
        }
    }
    
    /// Process a stereo camera frame (side-by-side format)
    private func processStereoFrame(_ pixelBuffer: CVPixelBuffer, leftIntrinsics: CameraIntrinsics?, rightIntrinsics: CameraIntrinsics?) {
        guard let detector = arucoDetector else { return }
        
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let halfWidth = width / 2
        let height = CVPixelBufferGetHeight(pixelBuffer)
        
        // Lock the pixel buffer
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }
        
        // Create left and right cropped pixel buffers
        // For now, we'll use the full frame detection and adjust cx for each half
        // A more robust approach would crop the pixel buffer, but this is simpler
        
        // Process left half
        if let leftIntrinsics = leftIntrinsics {
            // For side-by-side, the left half has cx relative to 0
            let cameraMatrix: [NSNumber] = [
                NSNumber(value: leftIntrinsics.fx), NSNumber(value: 0), NSNumber(value: leftIntrinsics.cx),
                NSNumber(value: 0), NSNumber(value: leftIntrinsics.fy), NSNumber(value: leftIntrinsics.cy),
                NSNumber(value: 0), NSNumber(value: 0), NSNumber(value: 1)
            ]
            let distCoeffs = leftIntrinsics.distortionCoeffs.map { NSNumber(value: $0) }
            
            // Crop left half of the pixel buffer
            if let leftBuffer = cropPixelBuffer(pixelBuffer, rect: CGRect(x: 0, y: 0, width: halfWidth, height: height)) {
                if let detections = detector.detectMarkers(
                    in: leftBuffer,
                    cameraMatrix: cameraMatrix,
                    distCoeffs: distCoeffs,
                    markerLength: markerSizeMeters
                ) {
                    cameraDetectedMarkers.removeAll()
                    for result in detections {
                        if result.poseValid, let transformArray = result.transformMatrix {
                            let transform = arrayToSimdMatrix(transformArray.map { $0.doubleValue })
                            cameraDetectedMarkers[Int(result.markerId)] = transform
                        }
                    }
                }
            }
        }
        
        // Process right half
        if let rightIntrinsics = rightIntrinsics {
            let cameraMatrix: [NSNumber] = [
                NSNumber(value: rightIntrinsics.fx), NSNumber(value: 0), NSNumber(value: rightIntrinsics.cx),
                NSNumber(value: 0), NSNumber(value: rightIntrinsics.fy), NSNumber(value: rightIntrinsics.cy),
                NSNumber(value: 0), NSNumber(value: 0), NSNumber(value: 1)
            ]
            let distCoeffs = rightIntrinsics.distortionCoeffs.map { NSNumber(value: $0) }
            
            // Crop right half of the pixel buffer
            if let rightBuffer = cropPixelBuffer(pixelBuffer, rect: CGRect(x: halfWidth, y: 0, width: halfWidth, height: height)) {
                if let detections = detector.detectMarkers(
                    in: rightBuffer,
                    cameraMatrix: cameraMatrix,
                    distCoeffs: distCoeffs,
                    markerLength: markerSizeMeters
                ) {
                    rightCameraDetectedMarkers.removeAll()
                    for result in detections {
                        if result.poseValid, let transformArray = result.transformMatrix {
                            let transform = arrayToSimdMatrix(transformArray.map { $0.doubleValue })
                            rightCameraDetectedMarkers[Int(result.markerId)] = transform
                        }
                    }
                }
            }
        }
    }
    
    /// Crop a pixel buffer to a specific region
    private func cropPixelBuffer(_ pixelBuffer: CVPixelBuffer, rect: CGRect) -> CVPixelBuffer? {
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer).cropped(to: rect)
        
        // Create a new pixel buffer for the cropped image
        var newPixelBuffer: CVPixelBuffer?
        let attrs: [String: Any] = [
            kCVPixelBufferCGImageCompatibilityKey as String: true,
            kCVPixelBufferCGBitmapContextCompatibilityKey as String: true
        ]
        
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            Int(rect.width),
            Int(rect.height),
            CVPixelBufferGetPixelFormatType(pixelBuffer),
            attrs as CFDictionary,
            &newPixelBuffer
        )
        
        guard status == kCVReturnSuccess, let outputBuffer = newPixelBuffer else {
            return nil
        }
        
        // Render the cropped image to the new buffer
        let context = CIContext()
        context.render(ciImage, to: outputBuffer)
        
        return outputBuffer
    }
    
    /// Try to collect a calibration sample if conditions are met
    private func tryCollectSample() {
        // Check time since last capture
        let now = Date()
        guard now.timeIntervalSince(lastCaptureTime) >= minCaptureInterval else {
            return
        }
        
        // Check head movement
        let currentPosition = SIMD3<Float>(
            currentHeadPose.columns.3.x,
            currentHeadPose.columns.3.y,
            currentHeadPose.columns.3.z
        )
        let movement = simd_length(currentPosition - lastHeadPosition)
        let totalSamples = leftSamplesCollected + rightSamplesCollected
        guard totalSamples == 0 || movement >= minHeadMovement else {
            statusMessage = "Move your head to a new position..."
            return
        }
        
        // Find markers detected by both ARKit and left camera
        let leftCommonMarkers = Set(arkitTrackedMarkers.keys).intersection(Set(cameraDetectedMarkers.keys))
        
        // For stereo, also check right camera
        let rightCommonMarkers = isStereoCalibration ? 
            Set(arkitTrackedMarkers.keys).intersection(Set(rightCameraDetectedMarkers.keys)) : Set<Int>()
        
        if leftCommonMarkers.isEmpty && rightCommonMarkers.isEmpty {
            statusMessage = "Point camera at ArUco markers..."
            return
        }
        
        var capturedAny = false
        
        // Collect samples for left camera
        for markerId in leftCommonMarkers {
            guard let markerInWorld = arkitTrackedMarkers[markerId],
                  let markerInCamera = cameraDetectedMarkers[markerId] else {
                continue
            }
            
            let sample = ExtrinsicCalibrationSample(
                markerId: markerId,
                cameraSide: isStereoCalibration ? .left : .mono,
                headPoseInWorld: currentHeadPose,
                markerPoseInWorld: markerInWorld,
                markerPoseInCamera: markerInCamera
            )
            samples.append(sample)
            capturedAny = true
        }
        
        // Collect samples for right camera (stereo only)
        if isStereoCalibration {
            for markerId in rightCommonMarkers {
                guard let markerInWorld = arkitTrackedMarkers[markerId],
                      let markerInCamera = rightCameraDetectedMarkers[markerId] else {
                    continue
                }
                
                let sample = ExtrinsicCalibrationSample(
                    markerId: markerId,
                    cameraSide: .right,
                    headPoseInWorld: currentHeadPose,
                    markerPoseInWorld: markerInWorld,
                    markerPoseInCamera: markerInCamera
                )
                samples.append(sample)
                capturedAny = true
            }
        }
        
        if capturedAny {
            // Update sample counts
            let leftSamples = samples.filter { $0.cameraSide == .left || $0.cameraSide == .mono }
            let rightSamples = samples.filter { $0.cameraSide == .right }
            leftSamplesCollected = leftSamples.count
            rightSamplesCollected = rightSamples.count
            samplesCollected = samples.count
            
            // Calculate progress based on minimum of both cameras for stereo
            if isStereoCalibration {
                let minProgress = min(Float(leftSamplesCollected), Float(rightSamplesCollected)) / Float(minSamples)
                calibrationProgress = minProgress
            } else {
                calibrationProgress = Float(samplesCollected) / Float(minSamples)
            }
            
            lastCaptureTime = now
            lastHeadPosition = currentPosition
            
            let uniqueMarkers = Set(samples.map { $0.markerId }).count
            if isStereoCalibration {
                statusMessage = "L: \(leftSamplesCollected)/\(minSamples), R: \(rightSamplesCollected)/\(minSamples) (\(uniqueMarkers) markers)"
            } else {
                statusMessage = "Captured \(samplesCollected)/\(minSamples) samples (\(uniqueMarkers) markers)"
            }
            
            print("📐 [ExtrinsicCalibrationManager] Collected samples - L: \(leftSamplesCollected), R: \(rightSamplesCollected)")
        }
    }
    
    /// Finish calibration and compute the transform(s)
    /// - Parameters:
    ///   - deviceId: Unique identifier for the camera device
    ///   - deviceName: Human-readable device name
    ///   - isStereo: Whether this is a stereo camera
    func finishCalibration(deviceId: String, deviceName: String, isStereo: Bool) -> ExtrinsicCalibrationData? {
        // Separate samples by camera side
        let leftSamples = samples.filter { $0.cameraSide == .left || $0.cameraSide == .mono }
        let rightSamples = samples.filter { $0.cameraSide == .right }
        
        // Check minimum samples
        if isStereo {
            if leftSamples.count < minSamples {
                lastError = "Not enough left camera samples: \(leftSamples.count) < \(minSamples)"
                return nil
            }
            if rightSamples.count < minSamples {
                lastError = "Not enough right camera samples: \(rightSamples.count) < \(minSamples)"
                return nil
            }
        } else {
            if leftSamples.count < minSamples {
                lastError = "Not enough samples: \(leftSamples.count) < \(minSamples)"
                return nil
            }
        }
        
        statusMessage = "Computing calibration..."
        
        // Compute T_head^camera for each set of samples
        // Using hand-eye calibration:
        //   T_world^marker = T_world^head * T_head^camera * T_camera^marker
        //   T_head^camera = (T_world^head)^-1 * T_world^marker * (T_camera^marker)^-1
        
        let leftTransform = computeHeadToCamera(from: leftSamples)
        let leftError = calculateReprojectionError(samples: leftSamples, headToCamera: leftTransform)
        
        var rightTransform: simd_float4x4? = nil
        var rightError: Double? = nil
        
        if isStereo {
            rightTransform = computeHeadToCamera(from: rightSamples)
            rightError = calculateReprojectionError(samples: rightSamples, headToCamera: rightTransform!)
        }
        
        // Create calibration result
        let uniqueMarkers = Set(samples.map { $0.markerId }).count
        let appVersion = Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "1.0"
        
        let calibrationData = ExtrinsicCalibrationData(
            cameraDeviceId: deviceId,
            cameraDeviceName: deviceName,
            isStereo: isStereo,
            leftHeadToCamera: matrixToArray(leftTransform),
            rightHeadToCamera: rightTransform.map { matrixToArray($0) },
            leftReprojectionError: leftError,
            rightReprojectionError: rightError,
            leftSampleCount: leftSamples.count,
            rightSampleCount: isStereo ? rightSamples.count : nil,
            markerCount: uniqueMarkers,
            calibrationDate: Date(),
            appVersion: appVersion,
            arucoDictionary: dictionaryName(arucoDictionary),
            markerSizeMeters: markerSizeMeters
        )
        
        // Save calibration
        allCalibrations[deviceId] = calibrationData
        currentCalibration = calibrationData
        saveAllCalibrations()
        
        statusMessage = "Calibration complete!"
        isCalibrating = false
        isStereoCalibration = false
        
        // Stop ARKit
        arkitSession = nil
        imageTrackingProvider = nil
        
        print("✅ [ExtrinsicCalibrationManager] Calibration saved for: \(deviceName)")
        print("   Left reproj error: \(leftError) meters (\(leftSamples.count) samples)")
        if let rightError = rightError {
            print("   Right reproj error: \(rightError) meters (\(rightSamples.count) samples)")
        }
        print("   Markers used: \(uniqueMarkers)")
        
        return calibrationData
    }
    
    /// Compute T_head^camera from a set of samples
    private func computeHeadToCamera(from samples: [ExtrinsicCalibrationSample]) -> simd_float4x4 {
        var estimatedTransforms: [simd_float4x4] = []
        
        for sample in samples {
            let headInWorld = sample.headPoseMatrix
            let markerInWorld = sample.markerInWorldMatrix
            let markerInCamera = sample.markerInCameraMatrix
            
            // T_head^world = inverse(T_world^head)
            let worldToHead = simd_inverse(headInWorld)
            
            // T_marker^camera = inverse(T_camera^marker)
            let cameraToMarker = simd_inverse(markerInCamera)
            
            // T_head^camera = T_head^world * T_world^marker * T_marker^camera
            //               = inv(T_world^head) * T_world^marker * inv(T_camera^marker)
            let headToCamera = worldToHead * markerInWorld * cameraToMarker
            
            estimatedTransforms.append(headToCamera)
        }
        
        return averageTransforms(estimatedTransforms)
    }
    
    /// Cancel the current calibration session
    func cancelCalibration() {
        samples = []
        arucoDetector = nil
        arkitSession = nil
        imageTrackingProvider = nil
        isCalibrating = false
        isStereoCalibration = false
        calibrationProgress = 0.0
        samplesCollected = 0
        leftSamplesCollected = 0
        rightSamplesCollected = 0
        statusMessage = ""
        arkitTrackedMarkers = [:]
        cameraDetectedMarkers = [:]
        rightCameraDetectedMarkers = [:]
        print("📐 [ExtrinsicCalibrationManager] Calibration cancelled")
    }
    
    // MARK: - Helper Functions
    
    private func arrayToSimdMatrix(_ arr: [Double]) -> simd_float4x4 {
        ExtrinsicCalibrationSample.arrayToMatrix(arr)
    }
    
    private func matrixToArray(_ m: simd_float4x4) -> [Double] {
        [
            Double(m.columns.0.x), Double(m.columns.0.y), Double(m.columns.0.z), Double(m.columns.0.w),
            Double(m.columns.1.x), Double(m.columns.1.y), Double(m.columns.1.z), Double(m.columns.1.w),
            Double(m.columns.2.x), Double(m.columns.2.y), Double(m.columns.2.z), Double(m.columns.2.w),
            Double(m.columns.3.x), Double(m.columns.3.y), Double(m.columns.3.z), Double(m.columns.3.w),
        ]
    }
    
    /// Average multiple 4x4 transforms
    private func averageTransforms(_ transforms: [simd_float4x4]) -> simd_float4x4 {
        guard !transforms.isEmpty else { return matrix_identity_float4x4 }
        
        // Average translation
        var avgTranslation = SIMD3<Float>.zero
        for t in transforms {
            avgTranslation += SIMD3<Float>(t.columns.3.x, t.columns.3.y, t.columns.3.z)
        }
        avgTranslation /= Float(transforms.count)
        
        // Average rotation using quaternions
        var quaternions: [simd_quatf] = []
        for t in transforms {
            // Extract rotation matrix
            let rotMat = simd_float3x3(
                SIMD3<Float>(t.columns.0.x, t.columns.0.y, t.columns.0.z),
                SIMD3<Float>(t.columns.1.x, t.columns.1.y, t.columns.1.z),
                SIMD3<Float>(t.columns.2.x, t.columns.2.y, t.columns.2.z)
            )
            let quat = simd_quatf(rotMat)
            quaternions.append(quat)
        }
        
        // Simple quaternion averaging (works well for similar quaternions)
        let avgQuat = averageQuaternions(quaternions)
        let avgRotMat = simd_float3x3(avgQuat)
        
        // Build result matrix
        var result = simd_float4x4(1)
        result.columns.0 = SIMD4<Float>(avgRotMat.columns.0, 0)
        result.columns.1 = SIMD4<Float>(avgRotMat.columns.1, 0)
        result.columns.2 = SIMD4<Float>(avgRotMat.columns.2, 0)
        result.columns.3 = SIMD4<Float>(avgTranslation, 1)
        
        return result
    }
    
    /// Average quaternions (simple approach - assumes similar orientations)
    private func averageQuaternions(_ quats: [simd_quatf]) -> simd_quatf {
        guard !quats.isEmpty else { return simd_quatf(ix: 0, iy: 0, iz: 0, r: 1) }
        
        // Ensure all quaternions are in the same hemisphere
        var sum = SIMD4<Float>.zero
        let reference = quats[0].vector
        
        for q in quats {
            var v = q.vector
            // Flip if in opposite hemisphere
            if simd_dot(v, reference) < 0 {
                v = -v
            }
            sum += v
        }
        
        // Normalize
        let normalized = simd_normalize(sum)
        return simd_quatf(vector: normalized)
    }
    
    /// Calculate reprojection error
    private func calculateReprojectionError(samples: [ExtrinsicCalibrationSample], headToCamera: simd_float4x4) -> Double {
        var totalError: Float = 0
        
        for sample in samples {
            let headInWorld = sample.headPoseMatrix
            let markerInWorld = sample.markerInWorldMatrix
            let markerInCameraObserved = sample.markerInCameraMatrix
            
            // Predicted: T_camera^marker = inv(T_head^camera) * inv(T_world^head) * T_world^marker
            let cameraToHead = simd_inverse(headToCamera)
            let worldToHead = simd_inverse(headInWorld)
            let markerInCameraPredicted = cameraToHead * worldToHead * markerInWorld
            
            // Compare translation
            let observedT = SIMD3<Float>(markerInCameraObserved.columns.3.x, 
                                          markerInCameraObserved.columns.3.y, 
                                          markerInCameraObserved.columns.3.z)
            let predictedT = SIMD3<Float>(markerInCameraPredicted.columns.3.x, 
                                           markerInCameraPredicted.columns.3.y, 
                                           markerInCameraPredicted.columns.3.z)
            
            totalError += simd_length(observedT - predictedT)
        }
        
        return Double(totalError / Float(samples.count))
    }
    
    private func dictionaryName(_ dict: ArucoDictionaryType) -> String {
        let names = [
            "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
            "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
            "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
            "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000",
            "DICT_ARUCO_ORIGINAL",
            "DICT_APRILTAG_16h5", "DICT_APRILTAG_25h9", "DICT_APRILTAG_36h10", "DICT_APRILTAG_36h11"
        ]
        let idx = Int(dict.rawValue)
        return idx >= 0 && idx < names.count ? names[idx] : "UNKNOWN"
    }
}
