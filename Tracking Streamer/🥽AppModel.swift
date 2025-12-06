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
        // Initialize the joints array with 27 matrices (forearmArm, forearmWrist, wrist + 24 finger joints)
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
            print("⚠️ [Benchmark] No active gRPC stream; dropping detection for seq=\(sequenceID)")
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
                print("⚠️ [Benchmark] Failed to send detection event: \(error)")
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
    
    @Published var showExitConfirmation: Bool = false
    
    // MARK: - iCloud Sync Helper
    
    private func syncSettingToiCloud<T>(_ key: String, value: T) {
        let store = NSUbiquitousKeyValueStore.default
        store.set(value, forKey: key)
        store.set(Date().timeIntervalSince1970, forKey: "visionos.lastSyncTime")
        store.synchronize()
    }
    
    private init() {
        // Load saved video source or default to network
        if let savedVideoSource = UserDefaults.standard.string(forKey: "videoSource"),
           let source = VideoSource(rawValue: savedVideoSource) {
            self.videoSource = source
        } else {
            self.videoSource = .network
        }
        // Load saved z-distance or use default of -10.0
        self.videoPlaneZDistance = UserDefaults.standard.object(forKey: "videoPlaneZDistance") as? Float ?? -10.0
        // Load saved y-position or use default of 0.0
        self.videoPlaneYPosition = UserDefaults.standard.object(forKey: "videoPlaneYPosition") as? Float ?? 0.0
        // Load saved auto-perpendicular or use default of false
        self.videoPlaneAutoPerpendicular = UserDefaults.standard.object(forKey: "videoPlaneAutoPerpendicular") as? Bool ?? false
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

}

extension 🥽AppModel {
    
    func run() {
#if targetEnvironment(simulator)
        print("Not support handTracking in simulator.")
#else
        
        Task {
            @MainActor in
            do {
                try await self.session.run([self.handTracking, self.worldTracking, self.sceneReconstruction])
                await self.processHandUpdates();
            } catch {
                print(error)
            }
        }
#endif
    }

    func startserver() {
        print("🚀 [DEBUG] startserver() called - initiating gRPC server...")
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
            // print("reconstruction update")
            let meshAnchor = update.anchor
            let mesh_description = meshAnchor.geometry.description
            // print(mesh_description)
        }
    }

    
    @MainActor
    private func queryAndProcessLatestDeviceAnchor() async {
        // Device anchors are only available when the provider is running.\
        guard worldTracking.state == .running else { return }
        
        let deviceAnchor = worldTracking.queryDeviceAnchor(atTimestamp: CACurrentMediaTime())
        // print(" *** device tracking running ")
//        print(deviceAnchor?.originFromAnchorTransform)
        guard let deviceAnchor else { return }
        DataManager.shared.latestHandTrackingData.Head = deviceAnchor.originFromAnchorTransform
            }

    private func processHandUpdates() async {
        for await update in self.handTracking.anchorUpdates {
            let handAnchor = update.anchor
            // print("processHandUpates is running.")
            switch handAnchor.chirality {
            case .left:
                if handAnchor.isTracked {
                    DataManager.shared.latestHandTrackingData.leftWrist = handAnchor.originFromAnchorTransform
                }
                // print(handAnchor.originFromAnchorTransform)
                

                let jointTypes: [HandSkeleton.JointName] = [
                    .forearmArm, .forearmWrist, .wrist,
                    .thumbKnuckle, .thumbIntermediateBase, .thumbIntermediateTip, .thumbTip,
                    .indexFingerMetacarpal, .indexFingerKnuckle, .indexFingerIntermediateBase, .indexFingerIntermediateTip, .indexFingerTip,
                    .middleFingerMetacarpal, .middleFingerKnuckle, .middleFingerIntermediateBase, .middleFingerIntermediateTip, .middleFingerTip,
                    .ringFingerMetacarpal, .ringFingerKnuckle, .ringFingerIntermediateBase, .ringFingerIntermediateTip, .ringFingerTip,
                    .littleFingerMetacarpal, .littleFingerKnuckle, .littleFingerIntermediateBase, .littleFingerIntermediateTip, .littleFingerTip,
                ]
                
                for (index, jointType) in jointTypes.enumerated() {
                    guard let joint = handAnchor.handSkeleton?.joint(jointType) else {
                        continue
                    }
                    DataManager.shared.latestHandTrackingData.leftSkeleton.joints[index] = joint.anchorFromJointTransform
                }
                
                // print("Updated left hand skeleton")
                // Repeat for right hand and other fingers as needed

            case .right:
            
                if handAnchor.isTracked {
                    DataManager.shared.latestHandTrackingData.rightWrist = handAnchor.originFromAnchorTransform
                }
                // print(handAnchor.originFromAnchorTransform)
                
                let jointTypes: [HandSkeleton.JointName] = [
                    .forearmArm, .forearmWrist, .wrist,
                    .thumbKnuckle, .thumbIntermediateBase, .thumbIntermediateTip, .thumbTip,
                    .indexFingerMetacarpal, .indexFingerKnuckle, .indexFingerIntermediateBase, .indexFingerIntermediateTip, .indexFingerTip,
                    .middleFingerMetacarpal, .middleFingerKnuckle, .middleFingerIntermediateBase, .middleFingerIntermediateTip, .middleFingerTip,
                    .ringFingerMetacarpal, .ringFingerKnuckle, .ringFingerIntermediateBase, .ringFingerIntermediateTip, .ringFingerTip,
                    .littleFingerMetacarpal, .littleFingerKnuckle, .littleFingerIntermediateBase, .littleFingerIntermediateTip, .littleFingerTip,
                ]

                for (index, jointType) in jointTypes.enumerated() {
                    guard let joint = handAnchor.handSkeleton?.joint(jointType) else {
                        continue
                    }
                    // print(index)
                    DataManager.shared.latestHandTrackingData.rightSkeleton.joints[index] = joint.anchorFromJointTransform
                }
                
                // print("Updated right hand skeleton")
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
    print("📡 [DEBUG] startServer() - Starting gRPC server setup (grpc-swift-2)...")
    
    Task {
        let serverManager = GRPCServerManager()
        await serverManager.startServer(port: 12345)
    }
}

/// Legacy startServer function for backwards compatibility with older iOS versions
func startServerLegacy() {
    print("⚠️ [DEBUG] Legacy startServer called - grpc-swift-2 requires iOS 18.0+/visionOS 2.0+")
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
    
    return handUpdate
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
