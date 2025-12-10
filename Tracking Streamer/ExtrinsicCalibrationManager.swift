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
/// 
/// Coordinate conventions:
/// - Head frame: ARKit convention (X-right, Y-up, Z-backward)
/// - Camera frame: ARKit convention (X-right, Y-up, Z-backward) - converted from OpenCV
/// 
/// The transform T_head^camera converts points from head frame to camera frame:
///   P_camera = T_head^camera * P_head
/// 
/// For a camera mounted on the front of Vision Pro looking forward:
/// - translation.z should be POSITIVE (camera is in front of head in ARKit Z-backward frame,
///   so head origin is behind camera, which is +Z in camera's ARKit-convention frame)
struct ExtrinsicCalibrationData: Codable, Equatable {
    /// Unique identifier for the camera device
    let cameraDeviceId: String
    /// Human-readable device name
    let cameraDeviceName: String
    /// Whether this is a stereo camera
    let isStereo: Bool
    /// The computed transform: T_head^camera (head to camera frame, ARKit convention)
    /// For mono cameras, or left camera for stereo
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
            "description": "Extrinsic calibration from Vision Pro head frame (ARKit: X-right, Y-up, Z-backward) to camera frame (OpenCV: X-right, Y-down, Z-forward)",
            "convention": "T_head_to_camera transforms points from head frame to OpenCV camera frame: P_camera = T * P_head",
            "note": "For front-mounted forward-looking camera, translation.z should be negative (head is behind camera)",
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
    @Published var minSamples: Int = 60  // Total samples needed (20 per marker position × 3 positions)
    
    /// Samples to collect per marker position
    var samplesPerPosition: Int = 30
    
    /// Minimum number of unique markers required for good calibration
    /// Need at least 3 markers at different 3D positions for proper rotation estimation
    var minUniqueMarkers: Int = 3
    
    /// ArUco dictionary type (using raw value: 0 = DICT_4X4_50)
    @Published var arucoDictionary: ArucoDictionaryType = ArucoDictionaryType(rawValue: 0)!
    
    /// Physical marker size in meters
    @Published var markerSizeMeters: Float = 0.055  // 55mm optimized for iPhone display (matches iPhone CalibrationDisplayManager)
    
    /// Marker IDs to track - sequential workflow (one at a time)
    /// iPhone displays these IDs one at a time, VisionOS tracks the current one
    @Published var markerIds: [Int] = [0, 2, 3]  // ID 1 removed for cleaner workflow
    
    /// Current marker ID being tracked (sequential workflow)
    /// iPhone should display this marker ID, both ARKit and OpenCV look for this ID
    @Published var currentMarkerId: Int = 0
    
    /// Index into markerIds for current position
    @Published var currentMarkerIndex: Int = 0
    
    /// Samples collected for current marker position
    @Published var samplesForCurrentMarker: Int = 0
    
    /// Minimum time between auto-captures (seconds)
    var minCaptureInterval: TimeInterval = 0.3  // Faster capture for better data collection
    
    /// Minimum head movement for auto-capture (meters)
    var minHeadMovement: Float = 0.02  // Smaller movement threshold for more samples
    
    /// Whether we're calibrating a stereo camera
    private var isStereoCalibration: Bool = false
    
    /// Known stereo baseline in meters (distance between left and right camera optical centers)
    /// Set this to a positive value to enforce baseline constraint during stereo calibration
    /// The baseline is measured along the X-axis (left camera at -baseline/2, right at +baseline/2)
    @Published var knownStereoBaseline: Float? = nil  // e.g., 0.065 for 65mm baseline
    
    /// Whether sample collection is paused (from iPhone motion detection via Multipeer)
    @Published var isCollectionPaused: Bool = false
    
    /// Whether to use iPhone motion detection for auto pause/resume
    @Published var useMotionDetection: Bool = true
    
    // MARK: - Private Properties
    
    private let storageKey = "extrinsicCalibrations"
    
    /// All collected samples (accessible for UI)
    @Published var samples: [ExtrinsicCalibrationSample] = []
    
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
            dlog("📐 [ExtrinsicCalibrationManager] No saved calibrations found")
            return
        }
        
        do {
            let decoder = JSONDecoder()
            allCalibrations = try decoder.decode([String: ExtrinsicCalibrationData].self, from: data)
            dlog("📐 [ExtrinsicCalibrationManager] Loaded \(allCalibrations.count) calibration(s)")
        } catch {
            dlog("❌ [ExtrinsicCalibrationManager] Failed to load calibrations: \(error)")
        }
    }
    
    private func saveAllCalibrations() {
        do {
            let encoder = JSONEncoder()
            let data = try encoder.encode(allCalibrations)
            UserDefaults.standard.set(data, forKey: storageKey)
            dlog("📐 [ExtrinsicCalibrationManager] Saved \(allCalibrations.count) calibration(s)")
            
            // Also sync to iCloud for iPhone app
            syncToiCloud()
        } catch {
            dlog("❌ [ExtrinsicCalibrationManager] Failed to save calibrations: \(error)")
        }
    }
    
    // MARK: - Calibration Loading
    
    func loadCalibration(for deviceId: String) -> ExtrinsicCalibrationData? {
        if let calibration = allCalibrations[deviceId] {
            currentCalibration = calibration
            dlog("📐 [ExtrinsicCalibrationManager] Loaded calibration for device: \(calibration.cameraDeviceName)")
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
        dlog("📐 [ExtrinsicCalibrationManager] Deleted calibration for device: \(deviceId)")
    }
    
    // MARK: - ARKit Image Tracking Setup
    
    /// Generate reference image for ARKit - single marker for current position
    /// ARKit can only reliably track ONE image at a time, so we generate just the current marker
    func generateReferenceImage(for markerId: Int) -> ReferenceImage? {
        guard let imageData = OpenCVArucoDetector.generateMarkerImage(
            id: Int32(markerId),
            sizePixels: 1000,  // High resolution for ARKit
            dictionary: arucoDictionary
        ) else {
            dlog("❌ [ExtrinsicCalibrationManager] Failed to generate marker \(markerId)")
            return nil
        }
        
        guard let cgImageSource = CGImageSourceCreateWithData(imageData as CFData, nil),
              let cgImage = CGImageSourceCreateImageAtIndex(cgImageSource, 0, nil) else {
            dlog("❌ [ExtrinsicCalibrationManager] Failed to create CGImage for marker \(markerId)")
            return nil
        }
        
        // Create ReferenceImage with physical size
        var refImage = ReferenceImage(
            cgimage: cgImage,
            physicalSize: CGSize(width: CGFloat(markerSizeMeters), height: CGFloat(markerSizeMeters)),
            orientation: .up
        )
        refImage.name = "aruco_\(markerId)"
        
        dlog("📐 [ExtrinsicCalibrationManager] Created ARKit reference image for marker \(markerId) - size: \(cgImage.width)x\(cgImage.height), physical: \(markerSizeMeters)m")
        
        return refImage
    }
    
    /// Generate reference images for ARKit - multiple markers (legacy, but updated for sequential workflow)
    func generateReferenceImages() -> [ReferenceImage] {
        // For sequential workflow, we only need the current marker
        // But generate all for flexibility
        var referenceImages: [ReferenceImage] = []
        
        for markerId in markerIds {
            if let refImage = generateReferenceImage(for: markerId) {
                referenceImages.append(refImage)
            }
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
        dlog("📐 [ExtrinsicCalibrationManager] ========== START EXTRINSIC CALIBRATION ==========")
        dlog("📐 [ExtrinsicCalibrationManager] Device: \(deviceName) (id: \(deviceId))")
        dlog("📐 [ExtrinsicCalibrationManager] Stereo: \(isStereo)")
        dlog("📐 [ExtrinsicCalibrationManager] Marker size: \(markerSizeMeters)m (\(markerSizeMeters * 1000)mm)")
        dlog("📐 [ExtrinsicCalibrationManager] Sequential workflow: markers \(markerIds)")
        dlog("📐 [ExtrinsicCalibrationManager] Samples per position: \(samplesPerPosition)")
        
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
        
        // Sequential workflow state
        currentMarkerIndex = 0
        currentMarkerId = markerIds.first ?? 0
        samplesForCurrentMarker = 0
        
        statusMessage = "Display marker ID \(currentMarkerId) on iPhone..."
        
        // Setup ARKit image tracking with ALL markers
        // ARKit will track whichever marker is visible
        do {
            let referenceImages = generateReferenceImages()
            dlog("📐 [ExtrinsicCalibrationManager] Generated \(referenceImages.count) reference images for markers: \(referenceImages.compactMap { $0.name })")
            
            guard !referenceImages.isEmpty else {
                lastError = "Failed to generate reference images"
                isCalibrating = false
                return
            }
            
            arkitSession = ARKitSession()
            imageTrackingProvider = ImageTrackingProvider(referenceImages: referenceImages)
            
            dlog("📐 [ExtrinsicCalibrationManager] ImageTrackingProvider created with \(referenceImages.count) images")
            
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
            
            statusMessage = "Show marker ID \(currentMarkerId) on iPhone, look at it with Vision Pro"
            dlog("📐 [ExtrinsicCalibrationManager] ARKit image tracking started - waiting for marker \(currentMarkerId)")
            
        } catch {
            lastError = "Failed to start ARKit: \(error.localizedDescription)"
            isCalibrating = false
            dlog("❌ [ExtrinsicCalibrationManager] ARKit setup failed: \(error)")
        }
    }
    
    /// Advance to the next marker in the sequence
    /// Call this after collecting enough samples for the current marker
    func advanceToNextMarker() {
        guard currentMarkerIndex < markerIds.count - 1 else {
            dlog("📐 [ExtrinsicCalibrationManager] Already at last marker")
            return
        }
        
        currentMarkerIndex += 1
        currentMarkerId = markerIds[currentMarkerIndex]
        samplesForCurrentMarker = 0
        
        // Reset head tracking for new position
        lastHeadPosition = .zero
        lastCaptureTime = .distantPast
        
        statusMessage = "Now show marker ID \(currentMarkerId) at a DIFFERENT position"
        dlog("📐 [ExtrinsicCalibrationManager] Advanced to marker \(currentMarkerId) (position \(currentMarkerIndex + 1)/\(markerIds.count))")
    }
    
    /// Check if current marker position has enough samples
    var hasEnoughSamplesForCurrentMarker: Bool {
        return samplesForCurrentMarker >= samplesPerPosition
    }
    
    /// Check if we can advance to next marker
    var canAdvanceToNextMarker: Bool {
        return hasEnoughSamplesForCurrentMarker && currentMarkerIndex < markerIds.count - 1
    }
    
    /// Check if all marker positions have been sampled
    var allMarkersCalibrated: Bool {
        let uniqueMarkers = Set(samples.map { $0.markerId }).count
        return uniqueMarkers >= minUniqueMarkers && samplesCollected >= minSamples
    }
    
    /// Process ARKit image anchor updates
    private func processImageAnchorUpdates() async {
        guard let provider = imageTrackingProvider else { return }
        
        dlog("📐 [ExtrinsicCalibrationManager] Started listening for image anchor updates...")
        
        for await update in provider.anchorUpdates {
            let anchor = update.anchor
            
            dlog("📐 [ExtrinsicCalibrationManager] Anchor update - name: \(anchor.referenceImage.name ?? "nil"), isTracked: \(anchor.isTracked), event: \(update.event)")
            
            // Extract marker ID from reference image name
            guard let name = anchor.referenceImage.name,
                  name.hasPrefix("aruco_"),
                  let markerId = Int(name.dropFirst(6)) else {
                dlog("⚠️ [ExtrinsicCalibrationManager] Could not parse marker ID from anchor name: \(anchor.referenceImage.name ?? "nil")")
                continue
            }
            
            await MainActor.run {
                if anchor.isTracked {
                    // Store T_world^marker in both active tracking and remembered positions
                    let transform = anchor.originFromAnchorTransform
                    arkitTrackedMarkers[markerId] = transform
                    rememberedMarkerPositions[markerId] = transform  // Remember for later!
                    dlog("📐 [ExtrinsicCalibrationManager] Tracking marker \(markerId) - now have \(rememberedMarkerPositions.count) remembered markers")
                } else {
                    // Remove from active tracking but KEEP in remembered positions
                    // Markers are stationary, so their world position doesn't change
                    arkitTrackedMarkers.removeValue(forKey: markerId)
                    dlog("📐 [ExtrinsicCalibrationManager] Lost active tracking of marker \(markerId) (still remembered)")
                }
            }
        }
        
        dlog("📐 [ExtrinsicCalibrationManager] Stopped listening for image anchor updates")
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
    /// Sequential workflow: only collects samples for the CURRENT marker ID
    private func tryCollectSample() {
        // Check if collection is paused (from iPhone motion detection)
        if useMotionDetection && isCollectionPaused {
            return
        }
        
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
        
        // Allow first sample without movement requirement
        guard totalSamples == 0 || movement >= minHeadMovement else {
            // Don't spam movement messages
            return
        }
        
        // Sequential workflow: Only look for the CURRENT marker ID
        let targetMarkerId = currentMarkerId
        
        // Check if ARKit has seen this marker (remembered or actively tracking)
        let arkitHasMarker = rememberedMarkerPositions[targetMarkerId] != nil || arkitTrackedMarkers[targetMarkerId] != nil
        
        // Check if camera sees this marker
        let cameraHasMarker = cameraDetectedMarkers[targetMarkerId] != nil
        
        // Debug logging
        let activeTracking = arkitTrackedMarkers.keys.sorted()
        let remembered = rememberedMarkerPositions.keys.sorted()
        let cameraDetected = cameraDetectedMarkers.keys.sorted()
        
        dlog("📐 [ExtrinsicCalibrationManager] Target: ID \(targetMarkerId) | ARKit active: \(activeTracking), remembered: \(remembered) | Camera: \(cameraDetected)")
        
        // Provide helpful status messages
        if !arkitHasMarker && !cameraHasMarker {
            statusMessage = "Show marker ID \(targetMarkerId) on iPhone"
        } else if !arkitHasMarker {
            statusMessage = "Camera sees ID \(targetMarkerId) - look at it with Vision Pro"
        } else if !cameraHasMarker {
            statusMessage = "ARKit tracking ID \(targetMarkerId) - point camera at iPhone"
        }
        
        // Need BOTH ARKit and camera to see the SAME marker
        guard arkitHasMarker && cameraHasMarker else {
            return
        }
        
        // Get the marker transforms
        guard let markerInWorld = rememberedMarkerPositions[targetMarkerId] ?? arkitTrackedMarkers[targetMarkerId],
              let markerInCamera = cameraDetectedMarkers[targetMarkerId] else {
            return
        }
        
        // Check if we already have enough samples for this marker
        let currentMarkerLeftSamples = samples.filter { 
            $0.markerId == targetMarkerId && ($0.cameraSide == .left || $0.cameraSide == .mono)
        }.count
        
        guard currentMarkerLeftSamples < samplesPerPosition else {
            // Already have 20 samples for this marker, don't collect more
            return
        }
        
        // Collect sample for left/mono camera
        let sample = ExtrinsicCalibrationSample(
            markerId: targetMarkerId,
            cameraSide: isStereoCalibration ? .left : .mono,
            headPoseInWorld: headPose,
            markerPoseInWorld: markerInWorld,
            markerPoseInCamera: markerInCamera
        )
        samples.append(sample)
        
        // Collect sample for right camera (stereo only)
        if isStereoCalibration, let rightMarkerInCamera = rightCameraDetectedMarkers[targetMarkerId] {
            let currentMarkerRightSamples = samples.filter { 
                $0.markerId == targetMarkerId && $0.cameraSide == .right
            }.count
            
            // Only collect right sample if we haven't reached 20 yet
            if currentMarkerRightSamples < samplesPerPosition {
                let rightSample = ExtrinsicCalibrationSample(
                    markerId: targetMarkerId,
                    cameraSide: .right,
                    headPoseInWorld: headPose,
                    markerPoseInWorld: markerInWorld,
                    markerPoseInCamera: rightMarkerInCamera
                )
                samples.append(rightSample)
            }
        }
        
        // Update counts
        let leftSamples = samples.filter { $0.cameraSide == .left || $0.cameraSide == .mono }
        let rightSamples = samples.filter { $0.cameraSide == .right }
        leftSamplesCollected = leftSamples.count
        rightSamplesCollected = rightSamples.count
        samplesCollected = samples.count
        
        // Count samples for current marker
        samplesForCurrentMarker = samples.filter { 
            $0.markerId == targetMarkerId && ($0.cameraSide == .left || $0.cameraSide == .mono)
        }.count
        
        // Count unique markers
        let uniqueMarkers = Set(samples.map { $0.markerId }).count
        
        // Update progress
        calibrationProgress = Float(samplesCollected) / Float(minSamples)
        
        lastCaptureTime = now
        lastHeadPosition = currentPosition
        
        // Stop collecting if we reach exactly 20 samples for this marker
        let samplesForMarker = samples.filter { $0.markerId == targetMarkerId && ($0.cameraSide == .left || $0.cameraSide == .mono) }.count
        
        // Status message
        if samplesForMarker >= samplesPerPosition {
            if currentMarkerIndex < markerIds.count - 1 {
                statusMessage = "✓ Marker \(targetMarkerId) complete (\(samplesPerPosition) samples)! Ready for next marker."
            } else {
                statusMessage = "✓ All \(markerIds.count) markers complete! Ready to finish."
            }
        } else {
            statusMessage = "Marker \(targetMarkerId): \(samplesForMarker)/\(samplesPerPosition) samples"
        }
        
        dlog("📐 [ExtrinsicCalibrationManager] Sample captured for marker \(targetMarkerId) - position \(currentMarkerIndex + 1): \(samplesForCurrentMarker)/\(samplesPerPosition), total: \(samplesCollected)")
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
                dlog("📐 [ExtrinsicCalibrationManager] Enforcing stereo baseline constraint: \(baseline * 100)cm")
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
        
        dlog("✅ [ExtrinsicCalibrationManager] Calibration saved for: \(deviceName)")
        dlog("   Left reproj error: \(leftError) meters (\(leftSamples.count) samples)")
        if let rightError = rightError {
            dlog("   Right reproj error: \(rightError) meters (\(rightSamples.count) samples)")
        }
        dlog("   Markers used: \(uniqueMarkers)")
        
        // Print transform details for verification
        let t = SIMD3<Float>(leftTransform.columns.3.x, leftTransform.columns.3.y, leftTransform.columns.3.z)
        dlog("   Left T_head^camera translation: (\(String(format: "%.3f", t.x)), \(String(format: "%.3f", t.y)), \(String(format: "%.3f", t.z))) m")
        
        // Extract rotation angles (Euler XYZ) for intuition
        let R = simd_float3x3(
            SIMD3<Float>(leftTransform.columns.0.x, leftTransform.columns.0.y, leftTransform.columns.0.z),
            SIMD3<Float>(leftTransform.columns.1.x, leftTransform.columns.1.y, leftTransform.columns.1.z),
            SIMD3<Float>(leftTransform.columns.2.x, leftTransform.columns.2.y, leftTransform.columns.2.z)
        )
        let pitch = asin(-R.columns.2.x) // rotation around Y
        let yaw = atan2(R.columns.2.y, R.columns.2.z) // rotation around X
        let roll = atan2(R.columns.1.x, R.columns.0.x) // rotation around Z
        dlog("   Left rotation (XYZ Euler): X=\(String(format: "%.1f", yaw * 180 / .pi))°, Y=\(String(format: "%.1f", pitch * 180 / .pi))°, Z=\(String(format: "%.1f", roll * 180 / .pi))°")
        
        if t.z > 0 {
            dlog("   ⚠️ WARNING: translation.z > 0 - unusual for front-mounted camera")
        }
        
        return calibrationData
    }
    
    /// Compute T_head^camera from a set of samples using point-based registration (Kabsch algorithm)
    ///
    /// Input data:
    /// - ARKit provides: T_world^marker (marker in world frame) and T_world^head (head in world frame)
    /// - OpenCV provides: T_camera^marker (marker in camera frame, OpenCV convention)
    ///
    /// We compute marker position in head frame: P_head = inv(T_world^head) * T_world^marker.translation
    /// We get marker position in camera frame: P_camera = T_camera^marker.translation
    ///
    /// The Kabsch algorithm finds T such that: P_camera ≈ T * P_head
    /// This gives T_head^camera - the transform from head frame to OpenCV camera frame.
    ///
    /// For a front-mounted forward-looking camera:
    /// - translation.z should be NEGATIVE (head origin is behind camera in OpenCV Z-forward frame)
    /// - Rotation includes ~180° around X axis to flip Y (up→down) and Z (backward→forward)
    private func computeHeadToCamera(from samples: [ExtrinsicCalibrationSample]) -> simd_float4x4 {
        var pointsInHead: [SIMD3<Float>] = []
        var pointsInCamera: [SIMD3<Float>] = []
        
        for sample in samples {
            let headInWorld = sample.headPoseMatrix
            let markerInWorld = sample.markerInWorldMatrix
            let markerInCamera = sample.markerInCameraMatrix
            
            // Get marker position in world frame (from ARKit)
            let markerPosWorld = SIMD3<Float>(
                markerInWorld.columns.3.x,
                markerInWorld.columns.3.y,
                markerInWorld.columns.3.z
            )
            
            // Transform marker position to head frame: P_head = inv(T_world^head) * P_world
            let worldToHead = simd_inverse(headInWorld)
            let markerPosHead4 = worldToHead * SIMD4<Float>(markerPosWorld, 1)
            let markerPosHead = SIMD3<Float>(markerPosHead4.x, markerPosHead4.y, markerPosHead4.z)
            
            // Get marker position in camera frame (from OpenCV solvePnP)
            // OpenCV camera convention: X-right, Y-down, Z-forward
            // We do NOT convert - Kabsch will find the rotation that maps between conventions
            let markerPosCamera = SIMD3<Float>(
                markerInCamera.columns.3.x,
                markerInCamera.columns.3.y,
                markerInCamera.columns.3.z
            )
            
            pointsInHead.append(markerPosHead)
            pointsInCamera.append(markerPosCamera)
        }
        
        // Debug: print sample positions to verify scale/units
        dlog("📐 [Calibration] Total samples: \(pointsInHead.count)")
        
        // Compute statistics
        let headDistances = pointsInHead.map { simd_length($0) }
        let camDistances = pointsInCamera.map { simd_length($0) }
        let avgHeadDist = headDistances.reduce(0, +) / Float(headDistances.count)
        let avgCamDist = camDistances.reduce(0, +) / Float(camDistances.count)
        
        dlog("📐 [Calibration] Average distance to marker:")
        dlog("📐 [Calibration]   - Head frame:   \(String(format: "%.3f", avgHeadDist)) m")
        dlog("📐 [Calibration]   - Camera frame: \(String(format: "%.3f", avgCamDist)) m")
        dlog("📐 [Calibration]   - Ratio (cam/head): \(String(format: "%.3f", avgCamDist / avgHeadDist))")
        
        if abs(avgCamDist / avgHeadDist - 1.0) > 0.5 {
            dlog("⚠️ [Calibration] WARNING: Large distance ratio suggests scale mismatch!")
            dlog("⚠️ [Calibration] Check marker size setting (\(markerSizeMeters)m) matches physical marker")
        }
        
        if let firstHead = pointsInHead.first, let firstCam = pointsInCamera.first,
           let lastHead = pointsInHead.last, let lastCam = pointsInCamera.last {
            dlog("📐 [Calibration] First sample - Head frame: (\(String(format: "%.3f", firstHead.x)), \(String(format: "%.3f", firstHead.y)), \(String(format: "%.3f", firstHead.z))) m")
            dlog("📐 [Calibration] First sample - Camera frame: (\(String(format: "%.3f", firstCam.x)), \(String(format: "%.3f", firstCam.y)), \(String(format: "%.3f", firstCam.z))) m")
            dlog("📐 [Calibration] Last sample - Head frame: (\(String(format: "%.3f", lastHead.x)), \(String(format: "%.3f", lastHead.y)), \(String(format: "%.3f", lastHead.z))) m")
            dlog("📐 [Calibration] Last sample - Camera frame: (\(String(format: "%.3f", lastCam.x)), \(String(format: "%.3f", lastCam.y)), \(String(format: "%.3f", lastCam.z))) m")
        }
        
        // Check point cloud spread (needed for good rotation estimation)
        let headCentroid = pointsInHead.reduce(SIMD3<Float>.zero, +) / Float(pointsInHead.count)
        let camCentroid = pointsInCamera.reduce(SIMD3<Float>.zero, +) / Float(pointsInCamera.count)
        let headSpread = pointsInHead.map { simd_length($0 - headCentroid) }.max() ?? 0
        let camSpread = pointsInCamera.map { simd_length($0 - camCentroid) }.max() ?? 0
        dlog("📐 [Calibration] Point cloud spread (max dist from centroid):")
        dlog("📐 [Calibration]   - Head frame:   \(String(format: "%.3f", headSpread)) m")
        dlog("📐 [Calibration]   - Camera frame: \(String(format: "%.3f", camSpread)) m")
        
        if headSpread < 0.05 {
            dlog("⚠️ [Calibration] WARNING: Head frame points are tightly clustered!")
            dlog("⚠️ [Calibration] Move the marker to more diverse positions for better rotation estimation.")
        }
        
        // Kabsch finds T such that: P_camera_arkit = T * P_head
        // Since we converted camera points to ARKit convention, this gives T_head^camera in ARKit convention
        // Both head frame and camera frame now use: X-right, Y-up, Z-backward
        let T_head_to_camera = kabschAlgorithm(sourcePoints: pointsInHead, targetPoints: pointsInCamera)
        
        // Debug: print the resulting transform
        let translation = SIMD3<Float>(T_head_to_camera.columns.3.x, T_head_to_camera.columns.3.y, T_head_to_camera.columns.3.z)
        dlog("📐 [Calibration] Computed translation: (\(String(format: "%.3f", translation.x)), \(String(format: "%.3f", translation.y)), \(String(format: "%.3f", translation.z))) m")
        dlog("📐 [Calibration] Translation magnitude: \(String(format: "%.3f", simd_length(translation))) m")
        
        // Test transform quality
        var avgError: Float = 0
        for i in 0..<pointsInHead.count {
            let predicted = T_head_to_camera * SIMD4<Float>(pointsInHead[i], 1)
            let predictedPos = SIMD3<Float>(predicted.x, predicted.y, predicted.z)
            avgError += simd_length(predictedPos - pointsInCamera[i])
        }
        avgError /= Float(pointsInHead.count)
        dlog("📐 [Calibration] Average reprojection error: \(String(format: "%.4f", avgError)) m")
        
        // Sanity check: For a front-mounted forward-looking camera in ARKit convention,
        // Z should be POSITIVE (camera is in front of head, which is +Z direction in ARKit where Z points backward)
        // Wait - this needs more thought. In ARKit head frame:
        // - Z points backward (toward back of head)
        // - Camera in front of head would be at NEGATIVE Z
        // But our camera frame is also in ARKit convention now, so the translation represents
        // where the head origin is in camera frame.
        // If camera is in front of head looking forward, head is BEHIND camera, 
         
        // Test: apply transform to first point and compare with expected
        if let firstHead = pointsInHead.first, let firstCam = pointsInCamera.first {
            let predicted = T_head_to_camera * SIMD4<Float>(firstHead, 1)
            let predictedPos = SIMD3<Float>(predicted.x, predicted.y, predicted.z)
            let error = simd_length(predictedPos - firstCam)
            dlog("📐 [Calibration] Test - First point prediction error: \(String(format: "%.4f", error)) m")
        }
        
        return T_head_to_camera
    }
    
    /// Kabsch algorithm: find optimal rigid transform (rotation + translation) 
    /// that minimizes RMSD between two point sets
    /// Returns T such that: target ≈ T * source
    private func kabschAlgorithm(sourcePoints: [SIMD3<Float>], targetPoints: [SIMD3<Float>]) -> simd_float4x4 {
        guard sourcePoints.count == targetPoints.count, sourcePoints.count >= 3 else {
            dlog("⚠️ [ExtrinsicCalibrationManager] Kabsch: need at least 3 point pairs")
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
    
    /// 3x3 SVD using Accelerate's LAPACK (sgesdd)
    /// Returns (U, S, Vt) where A = U * diag(S) * Vt
    private func svd3x3(_ A: simd_float3x3) -> (simd_float3x3, SIMD3<Float>, simd_float3x3) {
        // LAPACK uses column-major (Fortran) ordering, same as simd_float3x3
        // sgesdd computes: A = U * Σ * V^T
        
        // Copy input matrix (LAPACK modifies it)
        var matrix: [Float] = [
            A.columns.0.x, A.columns.0.y, A.columns.0.z,
            A.columns.1.x, A.columns.1.y, A.columns.1.z,
            A.columns.2.x, A.columns.2.y, A.columns.2.z
        ]
        
        var m: Int32 = 3  // rows
        var n: Int32 = 3  // cols
        var lda: Int32 = 3
        var ldu: Int32 = 3
        var ldvt: Int32 = 3
        
        var singularValues: [Float] = [0, 0, 0]
        var uMatrix: [Float] = Array(repeating: 0, count: 9)
        var vtMatrix: [Float] = Array(repeating: 0, count: 9)
        
        // Workspace query
        var jobz = Int8(UInt8(ascii: "A"))  // Compute all of U and V^T
        var lwork: Int32 = -1
        var workQuery: Float = 0
        var iwork: [Int32] = Array(repeating: 0, count: 8 * 3)
        var info: Int32 = 0
        
        // Query optimal workspace size
        sgesdd_(&jobz, &m, &n, &matrix, &lda, &singularValues, &uMatrix, &ldu, &vtMatrix, &ldvt, &workQuery, &lwork, &iwork, &info)
        
        lwork = Int32(workQuery)
        var work: [Float] = Array(repeating: 0, count: Int(lwork))
        
        // Perform SVD
        sgesdd_(&jobz, &m, &n, &matrix, &lda, &singularValues, &uMatrix, &ldu, &vtMatrix, &ldvt, &work, &lwork, &iwork, &info)
        
        if info != 0 {
            dlog("⚠️ [ExtrinsicCalibrationManager] SVD failed with info = \(info)")
            return (simd_float3x3(1), SIMD3<Float>(1, 1, 1), simd_float3x3(1))
        }
        
        // Convert back to simd_float3x3 (column-major)
        let U = simd_float3x3(
            SIMD3<Float>(uMatrix[0], uMatrix[1], uMatrix[2]),
            SIMD3<Float>(uMatrix[3], uMatrix[4], uMatrix[5]),
            SIMD3<Float>(uMatrix[6], uMatrix[7], uMatrix[8])
        )
        
        let Vt = simd_float3x3(
            SIMD3<Float>(vtMatrix[0], vtMatrix[1], vtMatrix[2]),
            SIMD3<Float>(vtMatrix[3], vtMatrix[4], vtMatrix[5]),
            SIMD3<Float>(vtMatrix[6], vtMatrix[7], vtMatrix[8])
        )
        
        return (U, SIMD3<Float>(singularValues[0], singularValues[1], singularValues[2]), Vt)
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
        
        dlog("📐 [Baseline] Original positions - Left: (\(leftPos.x*100), \(leftPos.y*100), \(leftPos.z*100))cm, Right: (\(rightPos.x*100), \(rightPos.y*100), \(rightPos.z*100))cm")
        
        // Compute current baseline
        let currentBaseline = rightPos.x - leftPos.x
        dlog("📐 [Baseline] Current X baseline: \(currentBaseline * 100)cm, Target: \(baseline * 100)cm")
        
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
        dlog("📐 [Baseline] Current midpoint X: \(midpointX * 100)cm (ideally ~0)")
        
        // Enforce baseline with centering at x=0
        // Left camera at -baseline/2, right camera at +baseline/2
        let newLeftX = -baseline / 2
        let newRightX = baseline / 2
        
        let newLeftPos = SIMD3<Float>(newLeftX, avgY, avgZ)
        let newRightPos = SIMD3<Float>(newRightX, avgY, avgZ)
        
        dlog("📐 [Baseline] Refined positions - Left: (\(newLeftPos.x*100), \(newLeftPos.y*100), \(newLeftPos.z*100))cm, Right: (\(newRightPos.x*100), \(newRightPos.y*100), \(newRightPos.z*100))cm")
        
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
        rememberedMarkerPositions = [:]
        cameraDetectedMarkers = [:]
        rightCameraDetectedMarkers = [:]
        
        // Reset sequential workflow state
        currentMarkerIndex = 0
        currentMarkerId = markerIds.first ?? 0
        samplesForCurrentMarker = 0
        
        dlog("📐 [ExtrinsicCalibrationManager] Calibration cancelled")
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
    /// headToCamera transforms points from head frame (ARKit) to camera frame (OpenCV)
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
            
            // Observed marker position in camera frame (from OpenCV solvePnP)
            let observedPosCamera = SIMD3<Float>(
                markerInCamera.columns.3.x,
                markerInCamera.columns.3.y,
                markerInCamera.columns.3.z
            )
            
            // Predicted marker position in camera frame
            // P_camera = T_head^camera * P_head
            let predictedPosCamera4 = headToCamera * SIMD4<Float>(markerPosHead, 1)
            let predictedPosCamera = SIMD3<Float>(predictedPosCamera4.x, predictedPosCamera4.y, predictedPosCamera4.z)
            
            totalError += simd_length(observedPosCamera - predictedPosCamera)
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
    
    // MARK: - iCloud Sync for iPhone App
    
    /// iCloud KVS key for extrinsic calibration results
    private static let iCloudExtrinsicResultsKey = "extrinsicCalibrationResults"
    
    /// Sync all extrinsic calibrations to iCloud KVS for the iPhone app to display
    private func syncToiCloud() {
        // Convert all calibrations to a format compatible with iPhone app
        var results: [[String: Any]] = []
        
        for calibration in allCalibrations.values {
            let result: [String: Any] = [
                "cameraDeviceId": calibration.cameraDeviceId,
                "cameraDeviceName": calibration.cameraDeviceName,
                "isStereo": calibration.isStereo,
                "headToCamera": calibration.leftHeadToCamera,
                "reprojectionError": calibration.leftReprojectionError,
                "sampleCount": calibration.leftSampleCount + (calibration.rightSampleCount ?? 0),
                "calibrationDate": calibration.calibrationDate.timeIntervalSince1970
            ]
            results.append(result)
        }
        
        // Encode and save to iCloud KVS
        do {
            let jsonData = try JSONSerialization.data(withJSONObject: results, options: [])
            NSUbiquitousKeyValueStore.default.set(jsonData, forKey: Self.iCloudExtrinsicResultsKey)
            NSUbiquitousKeyValueStore.default.synchronize()
            dlog("☁️ [ExtrinsicCalibrationManager] Synced \(results.count) calibration(s) to iCloud")
        } catch {
            dlog("❌ [ExtrinsicCalibrationManager] Failed to sync to iCloud: \(error)")
        }
    }
}
