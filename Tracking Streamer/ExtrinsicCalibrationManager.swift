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
    /// Known stereo baseline used during calibration (nil if not enforced)
    let stereoBaselineMeters: Float?
    
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
        if let stereoBaselineMeters = stereoBaselineMeters {
            dict["stereoBaselineMeters"] = stereoBaselineMeters
        }
        return dict
    }
    
    /// Exports the extrinsic calibration as a JSON string with 4x4 matrices
    func exportAsJSON() -> String {
        // Helper to format a 16-element column-major array as a 4x4 row-major matrix
        func formatMatrix(_ arr: [Double]) -> [[Double]] {
            // Column-major to row-major conversion
            var rows: [[Double]] = []
            for row in 0..<4 {
                var rowData: [Double] = []
                for col in 0..<4 {
                    rowData.append(arr[col * 4 + row])
                }
                rows.append(rowData)
            }
            return rows
        }
        
        var json: [String: Any] = [
            "description": "Extrinsic calibration from Vision Pro head frame to camera frame",
            "convention": "T_head_to_camera transforms points from head frame to camera frame",
            "matrix_format": "4x4 homogeneous transformation matrix, row-major",
            "camera_device_id": cameraDeviceId,
            "camera_device_name": cameraDeviceName,
            "is_stereo": isStereo,
            "calibration_date": ISO8601DateFormatter().string(from: calibrationDate),
            "left_head_to_camera": formatMatrix(leftHeadToCamera),
            "left_reprojection_error_meters": leftReprojectionError,
            "left_sample_count": leftSampleCount,
            "marker_count": markerCount,
            "aruco_dictionary": arucoDictionary,
            "marker_size_meters": markerSizeMeters
        ]
        
        if let rightHeadToCamera = rightHeadToCamera {
            json["right_head_to_camera"] = formatMatrix(rightHeadToCamera)
        }
        if let rightReprojectionError = rightReprojectionError {
            json["right_reprojection_error_meters"] = rightReprojectionError
        }
        if let rightSampleCount = rightSampleCount {
            json["right_sample_count"] = rightSampleCount
        }
        if let stereoBaselineMeters = stereoBaselineMeters {
            json["stereo_baseline_meters"] = stereoBaselineMeters
        }
        
        // Pretty print JSON
        if let jsonData = try? JSONSerialization.data(withJSONObject: json, options: [.prettyPrinted, .sortedKeys]),
           let jsonString = String(data: jsonData, encoding: .utf8) {
            return jsonString
        }
        return "{\"error\": \"Failed to serialize calibration data\"}"
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
    
    /// Currently tracked markers from ARKit (actively being tracked right now)
    @Published var arkitTrackedMarkers: [Int: simd_float4x4] = [:]  // markerId -> T_world^marker
    
    /// Remembered marker positions in world frame (persist even when ARKit stops tracking)
    /// Since markers are stationary, once we know their world position, we can keep using it
    @Published var rememberedMarkerPositions: [Int: simd_float4x4] = [:]  // markerId -> T_world^marker
    
    /// Currently detected markers from camera (mono or left for stereo)
    @Published var cameraDetectedMarkers: [Int: simd_float4x4] = [:]  // markerId -> T_camera^marker
    
    /// Currently detected markers from right camera (stereo only)
    @Published var rightCameraDetectedMarkers: [Int: simd_float4x4] = [:]  // markerId -> T_camera^marker
    
    /// Current head pose from ARKit
    @Published var currentHeadPose: simd_float4x4 = matrix_identity_float4x4
    
    // MARK: - Configuration
    
    /// Minimum samples required for calibration (per camera for stereo)
    @Published var minSamples: Int = 30
    
    /// Minimum number of unique markers required for good calibration
    /// Need at least 3 markers at different 3D positions for proper rotation estimation
    var minUniqueMarkers: Int = 3
    
    /// ArUco dictionary type (using raw value: 0 = DICT_4X4_50)
    @Published var arucoDictionary: ArucoDictionaryType = ArucoDictionaryType(rawValue: 0)!
    
    /// Physical marker size in meters
    @Published var markerSizeMeters: Float = 0.11  // 110mm default (100mm marker + 10mm border)
    
    /// Marker IDs to track with OpenCV (can detect multiple)
    @Published var markerIds: [Int] = [0, 1, 2, 3]
    
    /// Single marker ID for ARKit tracking (ARKit can only reliably track 1 image at a time)
    /// This should match one of the printed markers
    @Published var arkitMarkerId: Int = 0
    
    /// Minimum time between auto-captures (seconds)
    var minCaptureInterval: TimeInterval = 0.5
    
    /// Minimum head movement for auto-capture (meters)
    var minHeadMovement: Float = 0.05
    
    /// Whether we're calibrating a stereo camera
    private var isStereoCalibration: Bool = false
    
    /// Known stereo baseline in meters (distance between left and right camera optical centers)
    /// Set this to a positive value to enforce baseline constraint during stereo calibration
    /// The baseline is measured along the X-axis (left camera at -baseline/2, right at +baseline/2)
    @Published var knownStereoBaseline: Float? = nil  // e.g., 0.065 for 65mm baseline
    
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
    
    /// Generate reference images for ARKit - multiple markers for better calibration
    func generateReferenceImages() -> [ReferenceImage] {
        var referenceImages: [ReferenceImage] = []
        
        // Generate reference images for ALL markers
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
            print("📐 [ExtrinsicCalibrationManager] Created ARKit reference image for marker \(markerId) - size: \(cgImage.width)x\(cgImage.height)")
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
        print("📐 [ExtrinsicCalibrationManager] Tracking marker IDs: \(markerIds)")
        
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
        rememberedMarkerPositions = [:]  // Clear remembered positions for fresh calibration
        cameraDetectedMarkers = [:]
        rightCameraDetectedMarkers = [:]
        statusMessage = "Initializing ARKit image tracking..."
        
        // Setup ARKit image tracking
        do {
            let referenceImages = generateReferenceImages()
            print("📐 [ExtrinsicCalibrationManager] Generated \(referenceImages.count) reference images for markers: \(referenceImages.compactMap { $0.name })")
            
            guard !referenceImages.isEmpty else {
                lastError = "Failed to generate reference images"
                isCalibrating = false
                return
            }
            
            arkitSession = ARKitSession()
            imageTrackingProvider = ImageTrackingProvider(referenceImages: referenceImages)
            
            print("📐 [ExtrinsicCalibrationManager] ImageTrackingProvider created with \(referenceImages.count) images")
            
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
        
        print("📐 [ExtrinsicCalibrationManager] Started listening for image anchor updates...")
        
        for await update in provider.anchorUpdates {
            let anchor = update.anchor
            
            print("📐 [ExtrinsicCalibrationManager] Anchor update - name: \(anchor.referenceImage.name ?? "nil"), isTracked: \(anchor.isTracked), event: \(update.event)")
            
            // Extract marker ID from reference image name
            guard let name = anchor.referenceImage.name,
                  name.hasPrefix("aruco_"),
                  let markerId = Int(name.dropFirst(6)) else {
                print("⚠️ [ExtrinsicCalibrationManager] Could not parse marker ID from anchor name: \(anchor.referenceImage.name ?? "nil")")
                continue
            }
            
            await MainActor.run {
                if anchor.isTracked {
                    // Store T_world^marker in both active tracking and remembered positions
                    let transform = anchor.originFromAnchorTransform
                    arkitTrackedMarkers[markerId] = transform
                    rememberedMarkerPositions[markerId] = transform  // Remember for later!
                    print("📐 [ExtrinsicCalibrationManager] Tracking marker \(markerId) - now have \(rememberedMarkerPositions.count) remembered markers")
                } else {
                    // Remove from active tracking but KEEP in remembered positions
                    // Markers are stationary, so their world position doesn't change
                    arkitTrackedMarkers.removeValue(forKey: markerId)
                    print("📐 [ExtrinsicCalibrationManager] Lost active tracking of marker \(markerId) (still remembered)")
                }
            }
        }
        
        print("📐 [ExtrinsicCalibrationManager] Stopped listening for image anchor updates")
    }
    
    /// Update head pose from ARKit (called from HeadTrackingSystem)
    /// Note: This method is no longer used - we now get head pose directly from DataManager
    func updateHeadPose(_ headPose: simd_float4x4) {
        currentHeadPose = headPose
    }
    
    /// Get the current head pose from DataManager
    private func getCurrentHeadPose() -> simd_float4x4 {
        return DataManager.shared.latestHandTrackingData.Head
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
        
        // Get current head pose from DataManager
        let headPose = getCurrentHeadPose()
        
        // Check head movement
        let currentPosition = SIMD3<Float>(
            headPose.columns.3.x,
            headPose.columns.3.y,
            headPose.columns.3.z
        )
        let movement = simd_length(currentPosition - lastHeadPosition)
        let totalSamples = leftSamplesCollected + rightSamplesCollected
        
        // Debug logging
        if totalSamples > 0 {
            print("📐 [ExtrinsicCalibrationManager] Head movement: \(String(format: "%.3f", movement))m (need \(minHeadMovement)m)")
        }
        
        guard totalSamples == 0 || movement >= minHeadMovement else {
            statusMessage = "Move your head (\(String(format: "%.0f", movement * 100))/\(String(format: "%.0f", minHeadMovement * 100))cm)..."
            return
        }
        
        // Use remembered marker positions (includes all markers ever seen by ARKit)
        // This allows us to use markers even when ARKit is not actively tracking them
        // Find markers detected by both ARKit (remembered) and left camera
        let leftCommonMarkers = Set(rememberedMarkerPositions.keys).intersection(Set(cameraDetectedMarkers.keys))
        
        // For stereo, also check right camera
        let rightCommonMarkers = isStereoCalibration ? 
            Set(rememberedMarkerPositions.keys).intersection(Set(rightCameraDetectedMarkers.keys)) : Set<Int>()
        
        // Debug logging
        print("📐 [ExtrinsicCalibrationManager] Remembered: \(rememberedMarkerPositions.keys.sorted()), Active: \(arkitTrackedMarkers.keys.sorted()), Camera: \(cameraDetectedMarkers.keys.sorted()), Common: \(leftCommonMarkers.sorted())")
        
        if leftCommonMarkers.isEmpty && rightCommonMarkers.isEmpty {
            let rememberedCount = rememberedMarkerPositions.count
            let cameraCount = cameraDetectedMarkers.count
            if rememberedCount == 0 && cameraCount == 0 {
                statusMessage = "Point camera at ArUco markers..."
            } else if rememberedCount == 0 {
                statusMessage = "Camera sees \(cameraDetectedMarkers.keys.sorted()) - look at markers to register"
            } else if cameraCount == 0 {
                statusMessage = "Registered \(rememberedCount) markers - point camera at them"
            } else {
                statusMessage = "Known: \(rememberedMarkerPositions.keys.sorted()), Camera: \(cameraDetectedMarkers.keys.sorted()) - no overlap"
            }
            return
        }
        
        var capturedAny = false
        
        // Collect samples for left camera (using remembered positions)
        for markerId in leftCommonMarkers {
            guard let markerInWorld = rememberedMarkerPositions[markerId],
                  let markerInCamera = cameraDetectedMarkers[markerId] else {
                continue
            }
            
            let sample = ExtrinsicCalibrationSample(
                markerId: markerId,
                cameraSide: isStereoCalibration ? .left : .mono,
                headPoseInWorld: headPose,
                markerPoseInWorld: markerInWorld,
                markerPoseInCamera: markerInCamera
            )
            samples.append(sample)
            capturedAny = true
        }
        
        // Collect samples for right camera (stereo only, using remembered positions)
        if isStereoCalibration {
            for markerId in rightCommonMarkers {
                guard let markerInWorld = rememberedMarkerPositions[markerId],
                      let markerInCamera = rightCameraDetectedMarkers[markerId] else {
                    continue
                }
                
                let sample = ExtrinsicCalibrationSample(
                    markerId: markerId,
                    cameraSide: .right,
                    headPoseInWorld: headPose,
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
            
            // Count unique markers
            let uniqueMarkers = Set(samples.map { $0.markerId }).count
            let markersNeeded = max(0, minUniqueMarkers - uniqueMarkers)
            
            // Calculate progress - need both enough samples AND enough unique markers
            let sampleProgress: Float
            if isStereoCalibration {
                sampleProgress = min(Float(leftSamplesCollected), Float(rightSamplesCollected)) / Float(minSamples)
            } else {
                sampleProgress = Float(samplesCollected) / Float(minSamples)
            }
            let markerProgress = Float(uniqueMarkers) / Float(minUniqueMarkers)
            calibrationProgress = min(sampleProgress, markerProgress)
            
            lastCaptureTime = now
            lastHeadPosition = currentPosition
            
            // Provide helpful status message
            if uniqueMarkers < minUniqueMarkers {
                if isStereoCalibration {
                    statusMessage = "L:\(leftSamplesCollected) R:\(rightSamplesCollected) - Need \(markersNeeded) more markers!"
                } else {
                    statusMessage = "\(samplesCollected) samples - Need \(markersNeeded) more markers!"
                }
            } else if isStereoCalibration {
                statusMessage = "L: \(leftSamplesCollected)/\(minSamples), R: \(rightSamplesCollected)/\(minSamples) (\(uniqueMarkers) markers ✓)"
            } else {
                statusMessage = "Captured \(samplesCollected)/\(minSamples) samples (\(uniqueMarkers) markers ✓)"
            }
            
            print("📐 [ExtrinsicCalibrationManager] Collected samples - L: \(leftSamplesCollected), R: \(rightSamplesCollected), markers: \(uniqueMarkers)")
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
        
        // Check minimum unique markers (critical for rotation estimation)
        let uniqueMarkers = Set(samples.map { $0.markerId }).count
        if uniqueMarkers < minUniqueMarkers {
            lastError = "Need at least \(minUniqueMarkers) unique markers for calibration, got \(uniqueMarkers). Place markers at different 3D positions."
            return nil
        }
        
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
        
        var leftTransform = computeHeadToCamera(from: leftSamples)
        var rightTransform: simd_float4x4? = nil
        
        if isStereo {
            rightTransform = computeHeadToCamera(from: rightSamples)
            
            // If we have a known baseline, enforce it
            if let baseline = knownStereoBaseline, baseline > 0 {
                print("📐 [ExtrinsicCalibrationManager] Enforcing stereo baseline constraint: \(baseline * 100)cm")
                (leftTransform, rightTransform) = enforceBaselineConstraint(
                    leftTransform: leftTransform,
                    rightTransform: rightTransform!,
                    baseline: baseline
                )
            }
        }
        
        let leftError = calculateReprojectionError(samples: leftSamples, headToCamera: leftTransform)
        var rightError: Double? = nil
        
        if isStereo, let rt = rightTransform {
            rightError = calculateReprojectionError(samples: rightSamples, headToCamera: rt)
        }
        
        // Create calibration result
        // (uniqueMarkers already computed above)
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
            markerSizeMeters: markerSizeMeters,
            stereoBaselineMeters: isStereo ? knownStereoBaseline : nil
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
    
    /// Compute T_head^camera from a set of samples using point-based registration
    /// Uses only marker positions (not rotations) to avoid axis convention mismatches
    /// between ARKit and OpenCV
    private func computeHeadToCamera(from samples: [ExtrinsicCalibrationSample]) -> simd_float4x4 {
        // Collect corresponding point pairs:
        // - Point in head frame (from ARKit marker position transformed to head frame)
        // - Point in camera frame (from OpenCV detection)
        //
        // ARKit/visionOS head frame convention (right-handed):
        //   X: RIGHT (wearer's right)
        //   Y: UP
        //   Z: BACKWARD (toward back of head)
        //
        // OpenCV camera frame convention (right-handed):
        //   X: RIGHT (in image)
        //   Y: DOWN (in image)
        //   Z: FORWARD (optical axis, into the scene)
        //
        // For a camera mounted on the head looking forward:
        // - Camera's +Z (forward) aligns with Head's -Z (forward = opposite of backward)
        // - Camera's +Y (down) aligns with Head's -Y (down = opposite of up)
        // - Camera's +X (right) aligns with Head's +X (right)
        //
        // So the nominal rotation from head to camera is Rx(180°):
        //   [1   0   0]
        //   [0  -1   0]
        //   [0   0  -1]
        
        var pointsInHead: [SIMD3<Float>] = []
        var pointsInCameraOpenCV: [SIMD3<Float>] = []
        
        for sample in samples {
            let headInWorld = sample.headPoseMatrix
            let markerInWorld = sample.markerInWorldMatrix
            let markerInCamera = sample.markerInCameraMatrix
            
            // Get marker position in world frame (ARKit)
            let markerPosWorld = SIMD3<Float>(
                markerInWorld.columns.3.x,
                markerInWorld.columns.3.y,
                markerInWorld.columns.3.z
            )
            
            // Transform marker position to head frame
            // P_head = inv(T_world^head) * P_world
            let worldToHead = simd_inverse(headInWorld)
            let markerPosWorld4 = SIMD4<Float>(markerPosWorld, 1)
            let markerPosHead4 = worldToHead * markerPosWorld4
            let markerPosHead = SIMD3<Float>(markerPosHead4.x, markerPosHead4.y, markerPosHead4.z)
            
            // Get marker position in camera frame (OpenCV convention)
            let markerPosCameraOpenCV = SIMD3<Float>(
                markerInCamera.columns.3.x,
                markerInCamera.columns.3.y,
                markerInCamera.columns.3.z
            )
            
            pointsInHead.append(markerPosHead)
            pointsInCameraOpenCV.append(markerPosCameraOpenCV)
        }
        
        // Compute Kabsch directly: find T such that P_camera_opencv = T * P_head
        // This T will naturally include the 180° rotation around X plus any mounting offset
        let T_head_to_camera_opencv = kabschAlgorithm(sourcePoints: pointsInHead, targetPoints: pointsInCameraOpenCV)
        
        // The result is T_head^camera in a "mixed" convention:
        // - Input points are in head frame (ARKit)
        // - Output points are in camera frame (OpenCV)
        //
        // For use in the app (which works in ARKit convention), we may want to
        // convert this to pure ARKit convention. The camera frame in ARKit convention
        // would have Z pointing backward (opposite to optical axis).
        //
        // T_head^camera_arkit = Rx(180°) * T_head^camera_opencv
        // where Rx(180°) converts from OpenCV camera frame to ARKit camera frame
        
        // Rx(180°) rotation matrix: flip Y and Z
        let Rx180 = simd_float4x4(
            SIMD4<Float>(1,  0,  0, 0),
            SIMD4<Float>(0, -1,  0, 0),
            SIMD4<Float>(0,  0, -1, 0),
            SIMD4<Float>(0,  0,  0, 1)
        )
        
        // Convert to ARKit camera convention
        let T_head_to_camera_arkit = Rx180 * T_head_to_camera_opencv
        
        return T_head_to_camera_arkit
    }
    
    /// Kabsch algorithm: find optimal rigid transform (rotation + translation) 
    /// that minimizes RMSD between two point sets
    /// Returns T such that: target ≈ T * source
    private func kabschAlgorithm(sourcePoints: [SIMD3<Float>], targetPoints: [SIMD3<Float>]) -> simd_float4x4 {
        guard sourcePoints.count == targetPoints.count, sourcePoints.count >= 3 else {
            print("⚠️ [ExtrinsicCalibrationManager] Kabsch: need at least 3 point pairs")
            return matrix_identity_float4x4
        }
        
        let n = Float(sourcePoints.count)
        
        // 1. Compute centroids
        var centroidSource = SIMD3<Float>.zero
        var centroidTarget = SIMD3<Float>.zero
        for i in 0..<sourcePoints.count {
            centroidSource += sourcePoints[i]
            centroidTarget += targetPoints[i]
        }
        centroidSource /= n
        centroidTarget /= n
        
        // 2. Center the point sets
        var centeredSource: [SIMD3<Float>] = []
        var centeredTarget: [SIMD3<Float>] = []
        for i in 0..<sourcePoints.count {
            centeredSource.append(sourcePoints[i] - centroidSource)
            centeredTarget.append(targetPoints[i] - centroidTarget)
        }
        
        // 3. Compute covariance matrix H = sum(source_i * target_i^T)
        // H is a 3x3 matrix
        var H = simd_float3x3(0)
        for i in 0..<sourcePoints.count {
            let s = centeredSource[i]
            let t = centeredTarget[i]
            // Outer product: s * t^T
            H.columns.0 += s * t.x
            H.columns.1 += s * t.y
            H.columns.2 += s * t.z
        }
        
        // 4. SVD of H: H = U * S * V^T
        // We want R = V * U^T
        let (U, _, Vt) = svd3x3(H)
        let V = Vt.transpose
        
        // 5. Compute rotation R = V * U^T
        var R = V * U.transpose
        
        // Handle reflection case (det(R) = -1)
        if simd_determinant(R) < 0 {
            // Flip sign of last column of V
            var Vcorrected = V
            Vcorrected.columns.2 = -Vcorrected.columns.2
            R = Vcorrected * U.transpose
        }
        
        // 6. Compute translation: t = centroid_target - R * centroid_source
        let t = centroidTarget - R * centroidSource
        
        // 7. Build 4x4 transform matrix
        var result = simd_float4x4(1)
        result.columns.0 = SIMD4<Float>(R.columns.0, 0)
        result.columns.1 = SIMD4<Float>(R.columns.1, 0)
        result.columns.2 = SIMD4<Float>(R.columns.2, 0)
        result.columns.3 = SIMD4<Float>(t, 1)
        
        return result
    }
    
    /// Simple 3x3 SVD using Jacobi rotations
    /// Returns (U, S, Vt) where A = U * diag(S) * Vt
    private func svd3x3(_ A: simd_float3x3) -> (simd_float3x3, SIMD3<Float>, simd_float3x3) {
        // Compute A^T * A
        let AtA = A.transpose * A
        
        // Eigendecomposition of A^T * A using Jacobi iterations
        var V = simd_float3x3(1) // Accumulates rotations
        var D = AtA // Becomes diagonal
        
        // Jacobi iterations
        for _ in 0..<20 {
            // Find largest off-diagonal element
            let d01 = abs(D.columns.1[0])
            let d02 = abs(D.columns.2[0])
            let d12 = abs(D.columns.2[1])
            
            var p = 0, q = 1
            var maxVal = d01
            if d02 > maxVal { p = 0; q = 2; maxVal = d02 }
            if d12 > maxVal { p = 1; q = 2 }
            
            if maxVal < 1e-10 { break }
            
            // Compute Jacobi rotation
            let app = D[p][p]
            let aqq = D[q][q]
            let apq = D[q][p]
            
            let tau = (aqq - app) / (2 * apq)
            let t = (tau >= 0 ? 1 : -1) / (abs(tau) + sqrt(1 + tau * tau))
            let c = 1 / sqrt(1 + t * t)
            let s = t * c
            
            // Apply rotation to D
            var G = simd_float3x3(1)
            G[p][p] = c; G[q][q] = c
            G[q][p] = s; G[p][q] = -s
            
            D = G.transpose * D * G
            V = V * G
        }
        
        // Singular values (sqrt of eigenvalues)
        let singularValues = SIMD3<Float>(sqrt(max(D[0][0], 0)), sqrt(max(D[1][1], 0)), sqrt(max(D[2][2], 0)))
        
        // U = A * V * S^(-1)
        var U = simd_float3x3(0)
        for i in 0..<3 {
            if singularValues[i] > 1e-10 {
                let col = A * V[i] / singularValues[i]
                U[i] = col
            } else {
                // Handle zero singular value
                U[i] = SIMD3<Float>(i == 0 ? 1 : 0, i == 1 ? 1 : 0, i == 2 ? 1 : 0)
            }
        }
        
        // Orthogonalize U using Gram-Schmidt if needed
        U = orthogonalize3x3(U)
        
        return (U, singularValues, V.transpose)
    }
    
    /// Orthogonalize a 3x3 matrix using modified Gram-Schmidt
    private func orthogonalize3x3(_ M: simd_float3x3) -> simd_float3x3 {
        var result = simd_float3x3(0)
        
        // First column: normalize
        result.columns.0 = simd_normalize(M.columns.0)
        
        // Second column: subtract projection onto first, normalize
        let proj1 = simd_dot(M.columns.1, result.columns.0) * result.columns.0
        result.columns.1 = simd_normalize(M.columns.1 - proj1)
        
        // Third column: cross product of first two
        result.columns.2 = simd_cross(result.columns.0, result.columns.1)
        
        return result
    }
    
    /// Enforce stereo baseline constraint on the calibration result
    /// Assumes cameras are rigidly mounted with:
    /// - Same rotation (looking in the same direction)
    /// - Known X-axis separation (baseline)
    /// - Symmetric around the head center (midpoint at x ≈ 0)
    ///
    /// Returns refined (leftTransform, rightTransform)
    private func enforceBaselineConstraint(
        leftTransform: simd_float4x4,
        rightTransform: simd_float4x4,
        baseline: Float
    ) -> (simd_float4x4, simd_float4x4) {
        // Extract translations (camera positions in head frame - need inverse!)
        // T_head^camera gives us where the camera is in head frame via inverse
        let leftCamInHead = simd_inverse(leftTransform)
        let rightCamInHead = simd_inverse(rightTransform)
        
        let leftPos = SIMD3<Float>(leftCamInHead.columns.3.x, leftCamInHead.columns.3.y, leftCamInHead.columns.3.z)
        let rightPos = SIMD3<Float>(rightCamInHead.columns.3.x, rightCamInHead.columns.3.y, rightCamInHead.columns.3.z)
        
        print("📐 [Baseline] Original positions - Left: (\(leftPos.x*100), \(leftPos.y*100), \(leftPos.z*100))cm, Right: (\(rightPos.x*100), \(rightPos.y*100), \(rightPos.z*100))cm")
        
        // Compute current baseline
        let currentBaseline = rightPos.x - leftPos.x
        print("📐 [Baseline] Current X baseline: \(currentBaseline * 100)cm, Target: \(baseline * 100)cm")
        
        // Average rotation from both cameras (they should be the same for rigidly mounted stereo)
        let leftRot = simd_float3x3(
            SIMD3<Float>(leftTransform.columns.0.x, leftTransform.columns.0.y, leftTransform.columns.0.z),
            SIMD3<Float>(leftTransform.columns.1.x, leftTransform.columns.1.y, leftTransform.columns.1.z),
            SIMD3<Float>(leftTransform.columns.2.x, leftTransform.columns.2.y, leftTransform.columns.2.z)
        )
        let rightRot = simd_float3x3(
            SIMD3<Float>(rightTransform.columns.0.x, rightTransform.columns.0.y, rightTransform.columns.0.z),
            SIMD3<Float>(rightTransform.columns.1.x, rightTransform.columns.1.y, rightTransform.columns.1.z),
            SIMD3<Float>(rightTransform.columns.2.x, rightTransform.columns.2.y, rightTransform.columns.2.z)
        )
        
        // Average rotation using quaternions
        let leftQuat = simd_quatf(leftRot)
        let rightQuat = simd_quatf(rightRot)
        let avgQuat = averageQuaternions([leftQuat, rightQuat])
        let avgRot = simd_float3x3(avgQuat)
        
        // Average Y and Z positions (only X should differ due to baseline)
        let avgY = (leftPos.y + rightPos.y) / 2
        let avgZ = (leftPos.z + rightPos.z) / 2
        
        // Current midpoint X
        let midpointX = (leftPos.x + rightPos.x) / 2
        print("📐 [Baseline] Current midpoint X: \(midpointX * 100)cm (ideally ~0)")
        
        // Enforce baseline with centering at x=0
        // Left camera at -baseline/2, right camera at +baseline/2
        let newLeftX = -baseline / 2
        let newRightX = baseline / 2
        
        let newLeftPos = SIMD3<Float>(newLeftX, avgY, avgZ)
        let newRightPos = SIMD3<Float>(newRightX, avgY, avgZ)
        
        print("📐 [Baseline] Refined positions - Left: (\(newLeftPos.x*100), \(newLeftPos.y*100), \(newLeftPos.z*100))cm, Right: (\(newRightPos.x*100), \(newRightPos.y*100), \(newRightPos.z*100))cm")
        
        // Build new camera poses in head frame
        var newLeftCamInHead = simd_float4x4(1)
        let leftCamRot = simd_float3x3(
            SIMD3<Float>(leftCamInHead.columns.0.x, leftCamInHead.columns.0.y, leftCamInHead.columns.0.z),
            SIMD3<Float>(leftCamInHead.columns.1.x, leftCamInHead.columns.1.y, leftCamInHead.columns.1.z),
            SIMD3<Float>(leftCamInHead.columns.2.x, leftCamInHead.columns.2.y, leftCamInHead.columns.2.z)
        )
        let rightCamRot = simd_float3x3(
            SIMD3<Float>(rightCamInHead.columns.0.x, rightCamInHead.columns.0.y, rightCamInHead.columns.0.z),
            SIMD3<Float>(rightCamInHead.columns.1.x, rightCamInHead.columns.1.y, rightCamInHead.columns.1.z),
            SIMD3<Float>(rightCamInHead.columns.2.x, rightCamInHead.columns.2.y, rightCamInHead.columns.2.z)
        )
        let avgCamQuat = averageQuaternions([simd_quatf(leftCamRot), simd_quatf(rightCamRot)])
        let avgCamRot = simd_float3x3(avgCamQuat)
        
        newLeftCamInHead.columns.0 = SIMD4<Float>(avgCamRot.columns.0, 0)
        newLeftCamInHead.columns.1 = SIMD4<Float>(avgCamRot.columns.1, 0)
        newLeftCamInHead.columns.2 = SIMD4<Float>(avgCamRot.columns.2, 0)
        newLeftCamInHead.columns.3 = SIMD4<Float>(newLeftPos, 1)
        
        var newRightCamInHead = simd_float4x4(1)
        newRightCamInHead.columns.0 = SIMD4<Float>(avgCamRot.columns.0, 0)
        newRightCamInHead.columns.1 = SIMD4<Float>(avgCamRot.columns.1, 0)
        newRightCamInHead.columns.2 = SIMD4<Float>(avgCamRot.columns.2, 0)
        newRightCamInHead.columns.3 = SIMD4<Float>(newRightPos, 1)
        
        // Convert back to T_head^camera (inverse of camera pose in head frame)
        let newLeftTransform = simd_inverse(newLeftCamInHead)
        let newRightTransform = simd_inverse(newRightCamInHead)
        
        return (newLeftTransform, newRightTransform)
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
    
    /// Calculate reprojection error (position-only, ignoring rotation)
    /// Note: headToCamera is in ARKit convention, so we convert OpenCV observations to match
    private func calculateReprojectionError(samples: [ExtrinsicCalibrationSample], headToCamera: simd_float4x4) -> Double {
        var totalError: Float = 0
        
        for sample in samples {
            let headInWorld = sample.headPoseMatrix
            let markerInWorld = sample.markerInWorldMatrix
            let markerInCamera = sample.markerInCameraMatrix
            
            // Get marker position in head frame (from ARKit via world)
            let markerPosWorld = SIMD3<Float>(
                markerInWorld.columns.3.x,
                markerInWorld.columns.3.y,
                markerInWorld.columns.3.z
            )
            let worldToHead = simd_inverse(headInWorld)
            let markerPosWorld4 = SIMD4<Float>(markerPosWorld, 1)
            let markerPosHead4 = worldToHead * markerPosWorld4
            let markerPosHead = SIMD3<Float>(markerPosHead4.x, markerPosHead4.y, markerPosHead4.z)
            
            // Observed marker position in camera frame (from OpenCV) - convert to ARKit convention
            let observedPosCameraOpenCV = SIMD3<Float>(
                markerInCamera.columns.3.x,
                markerInCamera.columns.3.y,
                markerInCamera.columns.3.z
            )
            // Convert OpenCV (Y-down, Z-forward) to ARKit (Y-up, Z-backward)
            let observedPosCameraARKit = SIMD3<Float>(observedPosCameraOpenCV.x, -observedPosCameraOpenCV.y, -observedPosCameraOpenCV.z)
            
            // Predicted marker position in camera frame (ARKit convention)
            // P_camera = T_head^camera * P_head
            let predictedPosCamera4 = headToCamera * SIMD4<Float>(markerPosHead, 1)
            let predictedPosCameraARKit = SIMD3<Float>(predictedPosCamera4.x, predictedPosCamera4.y, predictedPosCamera4.z)
            
            totalError += simd_length(observedPosCameraARKit - predictedPosCameraARKit)
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
