import SwiftUI
import RealityKit
import ARKit
import Foundation
import GRPCCore
import GRPCNIOTransportHTTP2
import GRPCProtobuf

struct Skeleton {
    var joints: [simd_float4x4]

    init() {
        // Initialize the joints array with 27 matrices
        // Ordering: [0-24] standard 25 joints (wrist + fingers), [25-26] forearm joints
        self.joints = Array(repeating: simd_float4x4(1), count: 27)
    }
}

struct HandTrackingData {
    var leftWrist: simd_float4x4 = simd_float4x4(1)
    var rightWrist: simd_float4x4 = simd_float4x4(1)
    var leftSkeleton: Skeleton = Skeleton()
    var rightSkeleton: Skeleton = Skeleton()
    var Head: simd_float4x4 = simd_float4x4(1)
}

struct BenchmarkEvent {
    let sequenceID: UInt32
    let sentTimestampMs: UInt32
    let detectedTimestampMs: UInt32
}

@available(macOS 15.0, iOS 18.0, watchOS 11.0, tvOS 18.0, visionOS 2.0, *)
final class BenchmarkEventDispatcher: @unchecked Sendable {
    static let shared = BenchmarkEventDispatcher()

    private let lock = NSLock()
    private var responseWriter: (any RPCWriterProtocol)?
    private var swiftEpochNanoseconds: UInt64?

    private init() {}

    func register(responseWriter: some RPCWriterProtocol) {
        lock.lock()
        self.responseWriter = responseWriter
        self.swiftEpochNanoseconds = nil
        lock.unlock()
    }

    func clear() {
        lock.lock()
        responseWriter = nil
        swiftEpochNanoseconds = nil
        lock.unlock()
    }

    func emitDetection(sequenceID: UInt32, sentTimestampMs: UInt32, detectedAtNanoseconds: UInt64) {
        lock.lock()
        guard responseWriter != nil else {
            lock.unlock()
            dlog("⚠️ [Benchmark] No active gRPC stream; dropping detection for seq=\(sequenceID)")
            return
        }

        if swiftEpochNanoseconds == nil {
            swiftEpochNanoseconds = detectedAtNanoseconds
        }

        let base = swiftEpochNanoseconds ?? detectedAtNanoseconds
        let relativeNanoseconds = detectedAtNanoseconds &- base
        let detectedMs = UInt32(relativeNanoseconds / 1_000_000)

        let event = BenchmarkEvent(
            sequenceID: sequenceID,
            sentTimestampMs: sentTimestampMs,
            detectedTimestampMs: detectedMs
        )
        lock.unlock()

        var message = Handtracking_HandUpdate()
        message.head.m00 = 777.0
        message.head.m01 = Float(event.sequenceID)
        message.head.m02 = Float(event.sentTimestampMs)
        message.head.m03 = Float(event.detectedTimestampMs)

        Task {
            do {
                if let writer = self.responseWriter as? GRPCCore.RPCWriter<Handtracking_HandUpdate> {
                    try await writer.write(message)
                }
            } catch {
                dlog("⚠️ [Benchmark] Failed to send detection event: \(error)")
            }
        }
    }
}

/// Protocol to allow type erasure for RPCWriter
protocol RPCWriterProtocol: Sendable {}

/// Video source selection
enum VideoSource: String, CaseIterable {
    case network = "Network Stream"
    case uvcCamera = "USB Camera"
    
    var icon: String {
        switch self {
        case .network: return "wifi"
        case .uvcCamera: return "cable.connector"
        }
    }
}

class DataManager: ObservableObject {
    static let shared = DataManager()
    
    // MARK: - Python Library Version Compatibility
    // Minimum required Python library version (encoded as major*10000 + minor*100 + patch)
    // Update this when breaking changes are made that require users to upgrade
    static let minimumPythonVersionCode: Int = 20500  // Minimum: 2.50.0
    static let minimumPythonVersionString: String = "2.50.0"
    
    /// Connected Python library version code (0 if unknown/old library)
    @Published var pythonLibraryVersionCode: Int = 0
    
    /// Human-readable version string of connected Python library
    var pythonLibraryVersionString: String {
        guard pythonLibraryVersionCode > 0 else { return "Unknown (< 2.50.0)" }
        let major = pythonLibraryVersionCode / 10000
        let minor = (pythonLibraryVersionCode % 10000) / 100
        let patch = pythonLibraryVersionCode % 100
        return "\(major).\(minor).\(patch)"
    }
    
    /// Whether the connected Python library is compatible with this visionOS app
    var isPythonVersionCompatible: Bool {
        // Version code 0 means old library that doesn't send version - treat as incompatible
        guard pythonLibraryVersionCode > 0 else { return false }
        return pythonLibraryVersionCode >= DataManager.minimumPythonVersionCode
    }
    
    /// Whether we should show a version warning (only when connected but incompatible)
    var shouldShowVersionWarning: Bool {
        return pythonClientIP != nil && !isPythonVersionCompatible
    }
    
    var latestHandTrackingData: HandTrackingData = HandTrackingData()
    @Published var pythonClientIP: String? = nil  // Store Python client's IP when it connects via gRPC
    @Published var grpcServerReady: Bool = false  // Indicates if gRPC server is ready to accept connections
    @Published var webrtcServerInfo: (host: String, port: Int)? = nil  // WebRTC server info from gRPC
    @Published var webrtcGeneration: Int = 0  // Increments when new WebRTC info is received to trigger reconnects
    @Published var stereoEnabled: Bool = false  // Whether stereo video mode is enabled
    @Published var stereoAudioEnabled: Bool = false  // Whether stereo audio mode is enabled
    @Published var audioEnabled: Bool = false  // Whether audio track is present at all
    @Published var videoEnabled: Bool = false  // Whether video track is present at all
    @Published var simEnabled: Bool = false    // Whether simulation is enabled
    @Published var crossNetworkRoomCode: String? = nil  // Room code for cross-network mode (nil = local mode)
    
    // USDZ scene loaded via WebRTC (cross-network mode)
    @Published var loadedUsdzPath: String? = nil  // Path to the loaded USDZ file
    @Published var loadedUsdzAttachPosition: [Float]? = nil  // Attach position [x, y, z]
    @Published var loadedUsdzAttachRotation: [Float]? = nil  // Attach rotation [x, y, z, w]
    
    // Persistent setting for Cross-Network Mode (vs Local Mode)
    @Published var isCrossNetworkMode: Bool {
        didSet {
            UserDefaults.standard.set(isCrossNetworkMode, forKey: "isCrossNetworkMode")
        }
    }
    
    // Video source selection (persistent via UserDefaults)
    @Published var videoSource: VideoSource {
        didSet {
            UserDefaults.standard.set(videoSource.rawValue, forKey: "videoSource")
        }
    }
    
    // Stream stats
    @Published var videoResolution: String = "Waiting..."
    @Published var videoFPS: Int = 0
    @Published var audioSampleRate: Int = 0
    
    @Published var connectionStatus: String = "Initializing..."
    @Published var webRTCConnectionType: String = "" // Host, STUN, or TURN
    
    // Video plane z-distance setting (persistent via UserDefaults)
    @Published var videoPlaneZDistance: Float {
        didSet {
            UserDefaults.standard.set(videoPlaneZDistance, forKey: "videoPlaneZDistance")
            syncSettingToiCloud("visionos.videoPlaneZDistance", value: Double(videoPlaneZDistance))
        }
    }
    
    // Video plane y-position setting (persistent via UserDefaults)
    @Published var videoPlaneYPosition: Float {
        didSet {
            UserDefaults.standard.set(videoPlaneYPosition, forKey: "videoPlaneYPosition")
            syncSettingToiCloud("visionos.videoPlaneYPosition", value: Double(videoPlaneYPosition))
        }
    }
    
    // Auto-tilt video plane to be perpendicular to view (persistent via UserDefaults)
    @Published var videoPlaneAutoPerpendicular: Bool {
        didSet {
            UserDefaults.standard.set(videoPlaneAutoPerpendicular, forKey: "videoPlaneAutoPerpendicular")
        }
    }
    
    // Lock video plane to world frame (not persistent, always defaults to head-following)
    @Published var videoPlaneFixedToWorld: Bool
    
    // Status window minimized position (persistent via UserDefaults)
    @Published var statusMinimizedXPosition: Float {
        didSet {
            UserDefaults.standard.set(statusMinimizedXPosition, forKey: "statusMinimizedXPosition")
            syncSettingToiCloud("visionos.statusMinimizedXPosition", value: Double(statusMinimizedXPosition))
        }
    }
    
    @Published var statusMinimizedYPosition: Float {
        didSet {
            UserDefaults.standard.set(statusMinimizedYPosition, forKey: "statusMinimizedYPosition")
            syncSettingToiCloud("visionos.statusMinimizedYPosition", value: Double(statusMinimizedYPosition))
        }
    }
    
    // Upper limb (hand) visibility setting (persistent via UserDefaults)
    @Published var upperLimbVisible: Bool {
        didSet {
            UserDefaults.standard.set(upperLimbVisible, forKey: "upperLimbVisible")
            syncSettingToiCloud("visionos.upperLimbVisible", value: upperLimbVisible)
        }
    }
    
    // Head beam (ray cast) visibility setting (persistent via UserDefaults)
    @Published var showHeadBeam: Bool {
        didSet {
            UserDefaults.standard.set(showHeadBeam, forKey: "showHeadBeam")
            syncSettingToiCloud("visionos.showHeadBeam", value: showHeadBeam)
        }
    }
    
    // Hand joint spheres visibility setting (persistent via UserDefaults)
    @Published var showHandJoints: Bool {
        didSet {
            UserDefaults.standard.set(showHandJoints, forKey: "showHandJoints")
            syncSettingToiCloud("visionos.showHandJoints", value: showHandJoints)
        }
    }
    
    // Hand joints opacity setting (persistent via UserDefaults)
    @Published var handJointsOpacity: Float {
        didSet {
            UserDefaults.standard.set(handJointsOpacity, forKey: "handJointsOpacity")
            syncSettingToiCloud("visionos.handJointsOpacity", value: Double(handJointsOpacity))
        }
    }
    
    // Hand tracking prediction offset in seconds (persistent via UserDefaults)
    // A positive value queries predicted future hand poses for lower perceived latency
    // Range: 0 (no prediction) to 0.5 (500ms ahead - maximum)
    @Published var handPredictionOffset: Float {
        didSet {
            UserDefaults.standard.set(handPredictionOffset, forKey: "handPredictionOffset")
            syncSettingToiCloud("visionos.handPredictionOffset", value: Double(handPredictionOffset))
        }
    }
    
    // Video plane scale factor (persistent via UserDefaults)
    @Published var videoPlaneScale: Float {
        didSet {
            UserDefaults.standard.set(videoPlaneScale, forKey: "videoPlaneScale")
            syncSettingToiCloud("visionos.videoPlaneScale", value: Double(videoPlaneScale))
        }
    }
    
    // Stereo baseline offset for IPD adjustment (persistent via UserDefaults)
    // Negative = narrower baseline (crop outer sides), Positive = wider baseline (crop inner sides)
    @Published var stereoBaselineOffset: Float {
        didSet {
            UserDefaults.standard.set(stereoBaselineOffset, forKey: "stereoBaselineOffset")
            syncSettingToiCloud("visionos.stereoBaselineOffset", value: Double(stereoBaselineOffset))
        }
    }
    
    @Published var showExitConfirmation: Bool = false
    
    // Flag to indicate calibration wizard is active (status view should minimize)
    @Published var isCalibrationWizardActive: Bool = false
    
    // MARK: - Python Calibration Mode State
    // These are updated via gRPC when Python calibration server sends status (m00=778.0)
    @Published var pythonCalibrationActive: Bool = false
    @Published var pythonCalibrationStep: Int = 0  // 0, 1, 2 for extrinsic steps
    @Published var pythonCalibrationSamplesCollected: Int = 0
    @Published var pythonCalibrationSamplesNeeded: Int = 20
    @Published var pythonCalibrationTargetMarker: Int = 0  // 0, 2, or 3
    @Published var pythonCalibrationMarkerDetected: Bool = false
    @Published var pythonCalibrationProgress: Float = 0.0
    @Published var pythonCalibrationStepStatus: Int = 0  // 0=collecting, 1=calibrating, 2=complete
    
    // MARK: - iCloud Sync Helper
    
    private func syncSettingToiCloud<T>(_ key: String, value: T) {
        let store = NSUbiquitousKeyValueStore.default
        store.set(value, forKey: key)
        store.set(Date().timeIntervalSince1970, forKey: "visionos.lastSyncTime")
        store.synchronize()
    }
    
    private init() {
        // Load saved video source or default to network
        self.videoSource = VideoSource(rawValue: UserDefaults.standard.string(forKey: "videoSource") ?? "") ?? .network
        
        // Use local vars for checks to avoid accessing self before full init
        let savedZ = UserDefaults.standard.float(forKey: "videoPlaneZDistance")
        self.videoPlaneZDistance = (savedZ == 0) ? 1.6 : savedZ
        
        let savedY = UserDefaults.standard.float(forKey: "videoPlaneYPosition")
        self.videoPlaneYPosition = (savedY == 0) ? 1.5 : savedY
        
        self.videoPlaneAutoPerpendicular = UserDefaults.standard.object(forKey: "videoPlaneAutoPerpendicular") as? Bool ?? true
        
        self.isCrossNetworkMode = UserDefaults.standard.bool(forKey: "isCrossNetworkMode")
        // Always default to head-following (false) on startup, ignoring any saved state
        self.videoPlaneFixedToWorld = false
        // Load saved minimized status position or use defaults
        self.statusMinimizedXPosition = UserDefaults.standard.object(forKey: "statusMinimizedXPosition") as? Float ?? 0.0
        self.statusMinimizedYPosition = UserDefaults.standard.object(forKey: "statusMinimizedYPosition") as? Float ?? -0.3
        // Load saved upper limb visibility or default to true (visible)
        self.upperLimbVisible = UserDefaults.standard.object(forKey: "upperLimbVisible") as? Bool ?? true
        // Load saved head beam visibility or default to false (hidden)
        self.showHeadBeam = UserDefaults.standard.object(forKey: "showHeadBeam") as? Bool ?? false
        
        // Check if this is a new install or update (version-based)
        // Show hand joints by default on fresh install/update until user explicitly changes it
        let currentVersion = Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "1.0"
        let lastVersionForHandJoints = UserDefaults.standard.string(forKey: "lastVersionForHandJointsDefault")
        let isNewOrUpdated = lastVersionForHandJoints != currentVersion
        
        if isNewOrUpdated && UserDefaults.standard.object(forKey: "showHandJoints") == nil {
            // Fresh install or update with no explicit user preference - default to true (visible)
            self.showHandJoints = true
            UserDefaults.standard.set(currentVersion, forKey: "lastVersionForHandJointsDefault")
        } else {
            // User has explicitly set a preference - respect it
            self.showHandJoints = UserDefaults.standard.object(forKey: "showHandJoints") as? Bool ?? true
        }
        
        // Load saved hand joints opacity or default to 0.9 (90%)
        self.handJointsOpacity = UserDefaults.standard.object(forKey: "handJointsOpacity") as? Float ?? 0.9
        // Load saved hand prediction offset or default to 0.033 (33ms)
        self.handPredictionOffset = UserDefaults.standard.object(forKey: "handPredictionOffset") as? Float ?? 0.033
        // Load saved video plane scale or default to 1.0 (100%)
        self.videoPlaneScale = UserDefaults.standard.object(forKey: "videoPlaneScale") as? Float ?? 1.0
        // Load saved stereo baseline offset or default to 0.0 (no adjustment)
        self.stereoBaselineOffset = UserDefaults.standard.object(forKey: "stereoBaselineOffset") as? Float ?? 0.0
    }
}


@MainActor
class 🥽AppModel: ObservableObject {
    @AppStorage("unit") var unit: 📏Unit = .meters
    @Published private(set) var authorizationStatus: ARKitSession.AuthorizationStatus?
    
    private let session = ARKitSession()
    private let handTracking = HandTrackingProvider()
    private let worldTracking = WorldTrackingProvider()
    private let sceneReconstruction = SceneReconstructionProvider()
    
    // Pre-computed joint types array (static to avoid recreation on each update)
    // Ordering: [0-24] standard 25 joints (wrist + 4 fingers), [25-26] forearm joints
    // This ensures indices 0-24 are identical between 25-joint and 27-joint formats
    private static let jointTypes: [HandSkeleton.JointName] = [
        // Standard 25 joints (indices 0-24)
        .wrist,
        .thumbKnuckle, .thumbIntermediateBase, .thumbIntermediateTip, .thumbTip,
        .indexFingerMetacarpal, .indexFingerKnuckle, .indexFingerIntermediateBase, .indexFingerIntermediateTip, .indexFingerTip,
        .middleFingerMetacarpal, .middleFingerKnuckle, .middleFingerIntermediateBase, .middleFingerIntermediateTip, .middleFingerTip,
        .ringFingerMetacarpal, .ringFingerKnuckle, .ringFingerIntermediateBase, .ringFingerIntermediateTip, .ringFingerTip,
        .littleFingerMetacarpal, .littleFingerKnuckle, .littleFingerIntermediateBase, .littleFingerIntermediateTip, .littleFingerTip,
        // Forearm joints (indices 25-26) - appended at end for backward compatibility
        .forearmWrist, .forearmArm,
    ]
}

extension 🥽AppModel {
    
    func run() {
#if targetEnvironment(simulator)
        dlog("Not support handTracking in simulator.")
#else
        
        Task {
            @MainActor in
            do {
                try await self.session.run([self.handTracking, self.worldTracking, self.sceneReconstruction])
                // Use predictive hand tracking with handAnchors(at:) for lower latency
                // This polls at 120Hz and queries predicted poses at a future timestamp
                await self.processHandTrackingPredictive()
            } catch {
                dlog("\(error)")
            }
        }
#endif
    }

    func startserver() {
        dlog("🚀 [DEBUG] startserver() called - initiating gRPC server...")
        Task { startServer() }
    }
    
    
}

extension 🥽AppModel {
    
    @MainActor
    func run_device_tracking(function: () async -> Void, withFrequency hz: UInt64) async {
        while true {
            if Task.isCancelled {
                return
            }
            
            // Sleep for 1 s / hz before calling the function.
            let nanoSecondsToSleep: UInt64 = NSEC_PER_SEC / hz
            do {
                try await Task.sleep(nanoseconds: nanoSecondsToSleep)
            } catch {
                // Sleep fails when the Task is cancelled. Exit the loop.
                return
            }
            
            await function()
        }
    }

    @MainActor
    func processDeviceAnchorUpdates() async {
        await run_device_tracking(function: self.queryAndProcessLatestDeviceAnchor, withFrequency: 120)
    }
    
    func processReconstructionUpdates() async {
        for await update in sceneReconstruction.anchorUpdates {
            // dlog("reconstruction update")
            let meshAnchor = update.anchor
            let mesh_description = meshAnchor.geometry.description
            // dlog(mesh_description)
        }
    }

    
    @MainActor
    private func queryAndProcessLatestDeviceAnchor() async {
        // Device anchors are only available when the provider is running.\
        guard worldTracking.state == .running else { return }
        
        let deviceAnchor = worldTracking.queryDeviceAnchor(atTimestamp: CACurrentMediaTime())
        // dlog(" *** device tracking running ")
//        dlog(deviceAnchor?.originFromAnchorTransform)
        guard let deviceAnchor else { return }
        DataManager.shared.latestHandTrackingData.Head = deviceAnchor.originFromAnchorTransform
            }

    /// Process hand updates using predictive handAnchors(at:) polling instead of anchorUpdates stream.
    /// This allows querying predicted hand poses at future timestamps for lower perceived latency.
    /// The prediction offset is configurable via DataManager.shared.handPredictionOffset (0 to 0.5 seconds).
    private func processHandUpdatesPredictive() async {
        guard handTracking.state == .running else { return }
        
        // Use pre-computed static joint types array for better performance
        let jointTypes = Self.jointTypes
        
        // Query hand anchors at a slightly future timestamp for prediction
        // Use configurable prediction offset from DataManager
        let predictionOffset = TimeInterval(DataManager.shared.handPredictionOffset)
        let targetTimestamp = CACurrentMediaTime() + predictionOffset
        let anchors = handTracking.handAnchors(at: targetTimestamp)
        
        // Process left hand
        if let leftAnchor = anchors.leftHand {
            if leftAnchor.isTracked {
                DataManager.shared.latestHandTrackingData.leftWrist = leftAnchor.originFromAnchorTransform
            }
            
            if let skeleton = leftAnchor.handSkeleton {
                for (index, jointType) in jointTypes.enumerated() {
                    let joint = skeleton.joint(jointType)
                    DataManager.shared.latestHandTrackingData.leftSkeleton.joints[index] = joint.anchorFromJointTransform
                }
            }
        }
        
        // Process right hand
        if let rightAnchor = anchors.rightHand {
            if rightAnchor.isTracked {
                DataManager.shared.latestHandTrackingData.rightWrist = rightAnchor.originFromAnchorTransform
            }
            
            if let skeleton = rightAnchor.handSkeleton {
                for (index, jointType) in jointTypes.enumerated() {
                    let joint = skeleton.joint(jointType)
                    DataManager.shared.latestHandTrackingData.rightSkeleton.joints[index] = joint.anchorFromJointTransform
                }
            }
        }
    }
    
    /// Run predictive hand tracking at high frequency (replaces processHandUpdates)
    @MainActor
    func processHandTrackingPredictive() async {
        await run_device_tracking(function: self.processHandUpdatesPredictive, withFrequency: 120)
    }
    
    /// Legacy: Process hand updates using anchorUpdates stream (event-driven, non-predictive)
    private func processHandUpdates() async {
        for await update in self.handTracking.anchorUpdates {
            let handAnchor = update.anchor
            
            // Use pre-computed static joint types array for better performance
            let jointTypes = Self.jointTypes
            
            switch handAnchor.chirality {
            case .left:
                if handAnchor.isTracked {
                    DataManager.shared.latestHandTrackingData.leftWrist = handAnchor.originFromAnchorTransform
                }
                
                if let skeleton = handAnchor.handSkeleton {
                    for (index, jointType) in jointTypes.enumerated() {
                        let joint = skeleton.joint(jointType)
                        DataManager.shared.latestHandTrackingData.leftSkeleton.joints[index] = joint.anchorFromJointTransform
                    }
                }

            case .right:
                if handAnchor.isTracked {
                    DataManager.shared.latestHandTrackingData.rightWrist = handAnchor.originFromAnchorTransform
                }
                
                if let skeleton = handAnchor.handSkeleton {
                    for (index, jointType) in jointTypes.enumerated() {
                        let joint = skeleton.joint(jointType)
                        DataManager.shared.latestHandTrackingData.rightSkeleton.joints[index] = joint.anchorFromJointTransform
                    }
                }
            }
        }
    }
}


// MARK: - gRPC Server (grpc-swift-2)

/// Extension to make RPCWriter conform to RPCWriterProtocol for type erasure
@available(macOS 15.0, iOS 18.0, watchOS 11.0, tvOS 18.0, visionOS 2.0, *)
extension GRPCCore.RPCWriter: RPCWriterProtocol {}

/// Starts the gRPC server using grpc-swift-2
@available(macOS 15.0, iOS 18.0, watchOS 11.0, tvOS 18.0, visionOS 2.0, *)
func startServer() {
    dlog("📡 [DEBUG] startServer() - Starting gRPC server setup (grpc-swift-2)...")
    
    Task {
        let serverManager = GRPCServerManager()
        await serverManager.startServer(port: 12345)
    }
}

/// Legacy startServer function for backwards compatibility with older iOS versions
func startServerLegacy() {
    dlog("⚠️ [DEBUG] Legacy startServer called - grpc-swift-2 requires iOS 18.0+/visionOS 2.0+")
}

func fill_handUpdate() -> Handtracking_HandUpdate {
    var handUpdate = Handtracking_HandUpdate()
    
    // Assuming DataManager provides an ordered list/array of joints for leftSkeleton and rightSkeleton
    let leftJoints = DataManager.shared.latestHandTrackingData.leftSkeleton.joints
    let rightJoints = DataManager.shared.latestHandTrackingData.rightSkeleton.joints
    let leftWrist = DataManager.shared.latestHandTrackingData.leftWrist
    let rightWrist = DataManager.shared.latestHandTrackingData.rightWrist
    let Head = DataManager.shared.latestHandTrackingData.Head
    
    
    handUpdate.leftHand.wristMatrix = createMatrix4x4(from: leftWrist)
    handUpdate.rightHand.wristMatrix = createMatrix4x4(from: rightWrist)
    handUpdate.head = createMatrix4x4(from: Head)
    
    // Fill left hand joints
    for (index, jointMatrix) in leftJoints.enumerated() {
        let matrix = createMatrix4x4(from: jointMatrix)
        if index < handUpdate.leftHand.skeleton.jointMatrices.count {
            handUpdate.leftHand.skeleton.jointMatrices[index] = matrix
        } else {
            handUpdate.leftHand.skeleton.jointMatrices.append(matrix)
        }
    }

    // Fill right hand joints
    for (index, jointMatrix) in rightJoints.enumerated() {
        let matrix = createMatrix4x4(from: jointMatrix)
        if index < handUpdate.rightHand.skeleton.jointMatrices.count {
            handUpdate.rightHand.skeleton.jointMatrices[index] = matrix
        } else {
            handUpdate.rightHand.skeleton.jointMatrices.append(matrix)
        }
    }
    
    // MARKER DETECTION: Append detected markers as additional matrices in right hand skeleton
    // Format: After normal 27 joints, append:
    //   - Header matrix: m00=666.0 (marker data signal), m01=marker_count
    //   - For each marker: pose matrix with marker_id encoded in m33 (normally 1.0) as 1000+id+dict*100
    if let markerData = getMarkerMatrices() {
        for matrix in markerData {
            handUpdate.rightHand.skeleton.jointMatrices.append(matrix)
        }
    }
    
    // STYLUS TRACKING: Append tracked stylus pose after marker data
    // Format: Header matrix (m00=777.0, m01=stylus_count), then pose matrices
    if #available(visionOS 26.0, *) {
        if let stylusData = getStylusMatrices() {
            for matrix in stylusData {
                handUpdate.rightHand.skeleton.jointMatrices.append(matrix)
            }
        }
    }
    
    return handUpdate
}

/// Get stylus tracking matrices to append to the hand update
/// Returns nil if stylus tracking is disabled or no stylus connected
/// Encodes: pose (4x4), button states, pressures, timestamp
/// Only available on visionOS 26.0+
@available(visionOS 26.0, *)
func getStylusMatrices() -> [Handtracking_Matrix4x4]? {
    let manager = AccessoryTrackingManager.shared
    let snapshot = manager.snapshot
    
    // Check if tracking is active with valid position
    guard snapshot.isTracking,
          let position = snapshot.position,
          let orientation = snapshot.orientation else {
        return nil
    }
    
    var matrices: [Handtracking_Matrix4x4] = []
    
    // Header matrix: signals stylus data presence
    var header = Handtracking_Matrix4x4()
    header.m00 = 777.0  // Stylus data signal
    header.m01 = 1.0    // Number of styluses
    matrices.append(header)
    
    // Build pose matrix from position + orientation
    var transform = simd_float4x4(orientation)
    transform.columns.3 = SIMD4<Float>(position.x, position.y, position.z, 1)
    var matrix = createMatrix4x4(from: transform)
    
    // Encode button data in the last row (m30, m31, m32) and flags in m33
    // m30 = tip_pressure (0.0-1.0)
    // m31 = primary_pressure (0.0-1.0)
    // m32 = secondary_pressure (0.0-1.0)
    // m33 = button_flags encoded: 1000 + (tip?1:0) + (primary?2:0) + (secondary?4:0) + timestamp_fraction*10
    matrix.m30 = snapshot.tipPressure
    matrix.m31 = snapshot.primaryPressure
    matrix.m32 = snapshot.secondaryPressure
    
    // Encode button pressed states as flags + timestamp fraction for uniqueness
    var flags: Float = 1000.0  // Base value to distinguish from normal m33=1.0
    if snapshot.tipPressed { flags += 1.0 }
    if snapshot.primaryPressed { flags += 2.0 }
    if snapshot.secondaryPressed { flags += 4.0 }
    // Add timestamp fraction (0-999) for temporal info
    let timestampFraction = Float(Int(snapshot.timestamp * 1000) % 1000)
    flags += timestampFraction / 1000.0 * 10.0
    matrix.m33 = flags
    
    matrices.append(matrix)
    
    return matrices
}

/// Get marker detection matrices to append to the hand update
/// Returns nil if marker detection is disabled or no images detected
/// Includes both ArUco markers and custom user images
func getMarkerMatrices() -> [Handtracking_Matrix4x4]? {
    let manager = MarkerDetectionManager.shared
    // Use thread-safe snapshot properties for non-MainActor access
    guard manager.snapshotEnabled else { return nil }
    let markers = manager.snapshotMarkers
    let fixedMarkerIds = manager.snapshotFixedMarkers
    let customImages = manager.snapshotCustomImages
    let fixedCustomIds = manager.snapshotFixedCustomImages
    
    let totalCount = markers.count + customImages.count
    guard totalCount > 0 else { return nil }
    
    var matrices: [Handtracking_Matrix4x4] = []
    
    // Header matrix: signals marker data presence
    var header = Handtracking_Matrix4x4()
    header.m00 = 666.0  // Marker data signal
    header.m01 = Float(totalCount)  // Number of images following
    matrices.append(header)
    
    // Append each ArUco marker's pose matrix
    for (_, marker) in markers {
        var matrix = createMatrix4x4(from: marker.poseInWorld)
        // Encode marker info in m33 (normally 1.0 for homogeneous coordinates)
        // Format: 1000 + marker_id + (dict_type * 100)
        // This allows decoding: id = (value-1000) % 100, dict = (value-1000) / 100
        matrix.m33 = 1000.0 + Float(marker.id) + Float(marker.dictionaryType) * 100.0
        // Encode image type in m30: 0.0 = ArUco marker
        matrix.m30 = 0.0
        // Encode isFixed flag in m32 (normally 0.0 in a homogeneous matrix)
        // 1.0 = fixed, 0.0 = not fixed
        matrix.m32 = fixedMarkerIds.contains(marker.id) ? 1.0 : 0.0
        // Encode isTracked flag in m31 (normally 0.0 in a homogeneous matrix)
        // 1.0 = currently tracked by ARKit, 0.0 = tracking lost
        matrix.m31 = marker.isTracked ? 1.0 : 0.0
        matrices.append(matrix)
    }
    
    // Append each custom image's pose matrix
    // Custom images use a sequential index for encoding
    var customIndex = 0
    for (imageId, customImage) in customImages {
        var matrix = createMatrix4x4(from: customImage.poseInWorld)
        // Encode custom image info in m33: 2000 + sequential_index
        // Python will decode via m30 to determine image type
        matrix.m33 = 2000.0 + Float(customIndex)
        // Encode image type in m30: 1.0 = custom image
        matrix.m30 = 1.0
        // Encode isFixed flag in m32
        matrix.m32 = fixedCustomIds.contains(imageId) ? 1.0 : 0.0
        // Encode isTracked flag in m31
        matrix.m31 = customImage.isTracked ? 1.0 : 0.0
        matrices.append(matrix)
        customIndex += 1
    }
    
    return matrices
}



func createMatrix4x4(from jointMatrix: simd_float4x4) -> Handtracking_Matrix4x4 {
    var matrix = Handtracking_Matrix4x4()
    matrix.m00 = Float(jointMatrix.columns.0.x)
    matrix.m01 = Float(jointMatrix.columns.1.x)
    matrix.m02 = Float(jointMatrix.columns.2.x)
    matrix.m03 = Float(jointMatrix.columns.3.x)
    matrix.m10 = Float(jointMatrix.columns.0.y)
    matrix.m11 = Float(jointMatrix.columns.1.y)
    matrix.m12 = Float(jointMatrix.columns.2.y)
    matrix.m13 = Float(jointMatrix.columns.3.y)
    matrix.m20 = Float(jointMatrix.columns.0.z)
    matrix.m21 = Float(jointMatrix.columns.1.z)
    matrix.m22 = Float(jointMatrix.columns.2.z)
    matrix.m23 = Float(jointMatrix.columns.3.z)
    matrix.m30 = Float(jointMatrix.columns.0.w)
    matrix.m31 = Float(jointMatrix.columns.1.w)
    matrix.m32 = Float(jointMatrix.columns.2.w)
    matrix.m33 = Float(jointMatrix.columns.3.w)
    return matrix
}
