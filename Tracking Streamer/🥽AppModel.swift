import SwiftUI
import RealityKit
import ARKit
import GRPC
import NIO

struct Skeleton {
    var joints: [simd_float4x4]

    init() {
        // Initialize the joints array with 24 identity matrices
        self.joints = Array(repeating: simd_float4x4(1), count: 25)
    }
}

struct HandTrackingData {
    var leftWrist: simd_float4x4 = simd_float4x4(1)
    var rightWrist: simd_float4x4 = simd_float4x4(1)
    var leftSkeleton: Skeleton = Skeleton()
    var rightSkeleton: Skeleton = Skeleton()
    var Head: simd_float4x4 = simd_float4x4(1)
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
    
    @Published var connectionStatus: String = "Initializing..."
    
    // Video plane z-distance setting (persistent via UserDefaults)
    @Published var videoPlaneZDistance: Float {
        didSet {
            UserDefaults.standard.set(videoPlaneZDistance, forKey: "videoPlaneZDistance")
        }
    }
    
    // Video plane y-position setting (persistent via UserDefaults)
    @Published var videoPlaneYPosition: Float {
        didSet {
            UserDefaults.standard.set(videoPlaneYPosition, forKey: "videoPlaneYPosition")
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
        }
    }
    
    @Published var statusMinimizedYPosition: Float {
        didSet {
            UserDefaults.standard.set(statusMinimizedYPosition, forKey: "statusMinimizedYPosition")
        }
    }
    
    private init() {
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
        await run_device_tracking(function: self.queryAndProcessLatestDeviceAnchor, withFrequency: 90)
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
                DispatchQueue.main.async {
                    DataManager.shared.latestHandTrackingData.leftWrist = handAnchor.originFromAnchorTransform
                    // print(handAnchor.originFromAnchorTransform)
                    

                    let jointTypes: [HandSkeleton.JointName] = [
                        .wrist,
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
                }

            case .right:
                DispatchQueue.main.async {
                    DataManager.shared.latestHandTrackingData.rightWrist = handAnchor.originFromAnchorTransform
                    // print(handAnchor.originFromAnchorTransform)
                    
                    let jointTypes: [HandSkeleton.JointName] = [
                        .wrist,
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
}



class HandTrackingServiceProvider: Handtracking_HandTrackingServiceProvider {

    var interceptors: Handtracking_HandTrackingServiceServerInterceptorFactoryProtocol?

    nonisolated func streamHandUpdates(
        request: Handtracking_HandUpdate,
        context: StreamingResponseCallContext<Handtracking_HandUpdate>
    ) -> EventLoopFuture<GRPCStatus> {
        let eventLoop = context.eventLoop
        print("📥 [DEBUG] streamHandUpdates called - client connected!")
        
        // Check for WebRTC info in metadata (gRPC headers)
        let headers = context.headers
        if let webrtcInfo = headers.first(name: "webrtc-info") {
            print("🎞️ [DEBUG] Found WebRTC info in metadata: \(webrtcInfo)")
            let parts = webrtcInfo.split(separator: "|")
            if parts.count == 5 {
                let ip1 = Int(parts[0]) ?? 0
                let ip2 = Int(parts[1]) ?? 0
                let ip3 = Int(parts[2]) ?? 0
                let ip4 = Int(parts[3]) ?? 0
                let port = Int(parts[4]) ?? 9999
                let host = "\(ip1).\(ip2).\(ip3).\(ip4)"
                print("🎞️ WebRTC server from metadata: \(host):\(port)")
                DataManager.shared.webrtcServerInfo = (host: host, port: port)
            }
        }
        
        print("🔍 [DEBUG] Checking for discovery message (m00=\(request.head.m00))...")
        
        // Track if this is a WebRTC info-only connection (will close immediately)
        let isWebRTCInfoOnly = request.head.m00 == 999.0
        
        // Check if this is a special "discovery" message from Python
        // Python will send a message with Head.m00 = 888.0 to announce its presence
        // and encode its IP in subsequent matrix elements
        if request.head.m00 == 888.0 {
            print("✨ [DEBUG] Discovery message detected!")
            let ip1 = Int(request.head.m01)
            let ip2 = Int(request.head.m02)
            let ip3 = Int(request.head.m03)
            let ip4 = Int(request.head.m10)
            let pythonIP = "\(ip1).\(ip2).\(ip3).\(ip4)"
            print("🔍 Python client discovered at: \(pythonIP)")
            print("💾 [DEBUG] Storing Python IP in DataManager...")
            DataManager.shared.pythonClientIP = pythonIP
        } else if isWebRTCInfoOnly {
            // WebRTC server info message (this connection will close immediately after sending info)
            print("🎞️ [DEBUG] WebRTC server info message detected (info-only connection)!")
            let ip1 = Int(request.head.m01)
            let ip2 = Int(request.head.m02)
            let ip3 = Int(request.head.m03)
            let ip4 = Int(request.head.m10)
            let port = Int(request.head.m11)
            let stereoVideo = request.head.m12 > 0.5  // Stereo video flag
            let stereoAudio = request.head.m13 > 0.5  // Stereo audio flag
            let audioEnabled = request.head.m20 > 0.5  // Audio enabled flag
            let host = "\(ip1).\(ip2).\(ip3).\(ip4)"
            print("🎞️ WebRTC server available at: \(host):\(port) (stereo_video=\(stereoVideo), stereo_audio=\(stereoAudio), audio_enabled=\(audioEnabled))")
            print("💾 [DEBUG] Storing WebRTC info in DataManager...")
            
            DispatchQueue.main.async {
                let hadConnection = DataManager.shared.webrtcServerInfo != nil
                DataManager.shared.webrtcServerInfo = (host: host, port: port)
                DataManager.shared.stereoEnabled = stereoVideo
                DataManager.shared.stereoAudioEnabled = stereoAudio
                DataManager.shared.audioEnabled = audioEnabled
                // Increment generation to trigger reconnect
                // Use positive numbers for valid connections
                if DataManager.shared.webrtcGeneration < 0 || !hadConnection {
                    DataManager.shared.webrtcGeneration = 1
                } else {
                    DataManager.shared.webrtcGeneration += 1
                }
                print("🔄 [DEBUG] Set WebRTC generation to \(DataManager.shared.webrtcGeneration)")
            }
        } else {
            print("⚠️ [DEBUG] Not a special message (expected m00=888.0 or 999.0, got \(request.head.m00))")
        }
        
        print("🔄 [DEBUG] Starting hand tracking data stream...")
        print("hey...")
        // Example task to simulate sending hand tracking data.
        // In a real application, you would replace this with actual data collection and streaming.
        print("⏱️ [DEBUG] Scheduling repeated task for hand tracking updates (10ms interval)...")
        var updateCount = 0
        let task = eventLoop.scheduleRepeatedAsyncTask(initialDelay: .milliseconds(10), delay: .milliseconds(10)) { task -> EventLoopFuture<Void> in
//            var handUpdate = Handtracking_HandUpdate()
            
            let recent_hand = fill_handUpdate()
            updateCount += 1
            if updateCount == 1 || updateCount % 100 == 0 {
                print("📤 [DEBUG] Sending hand update #\(updateCount)...")
            }
            
            // Send the update to the client.
            return context.sendResponse(recent_hand).map { _ in }
        }

        // Ensure the task is cancelled when the client disconnects or the stream is otherwise closed.
        print("🔗 [DEBUG] Setting up disconnect handler...")
        context.statusPromise.futureResult.whenComplete { result in
            print("🔌 [DEBUG] Client disconnected or stream closed. Sent \(updateCount) updates. Result: \(result)")
            task.cancel()
            
            // Only clear connection state if this was NOT a WebRTC info-only connection
            // WebRTC info connections close immediately after sending info, but that's normal
            if !isWebRTCInfoOnly {
                DispatchQueue.main.async {
                    print("🧹 [DEBUG] Cleaning up connection state after main client disconnect")
                    DataManager.shared.pythonClientIP = nil
                    DataManager.shared.webrtcServerInfo = nil
                    // Set generation to negative to signal disconnection
                    DataManager.shared.webrtcGeneration = -1
                }
            } else {
                print("ℹ️ [DEBUG] WebRTC info-only connection closed (expected behavior)")
            }
        }

        // Return a future that will complete when the streaming operation is done.
        // Here, we're indicating that the stream will remain open indefinitely until the client disconnects.
        return eventLoop.makePromise(of: GRPCStatus.self).futureResult
    }
}

func startServer() {
    print("📡 [DEBUG] startServer() - Starting gRPC server setup...")
    DispatchQueue.global().async {
        print("🔧 [DEBUG] Dispatched to background thread")
        
        let port = 12345
        let host = "0.0.0.0"
        print("🔧 [DEBUG] Server configuration: host=\(host), port=\(port)")
        
        print("🔧 [DEBUG] Creating MultiThreadedEventLoopGroup...")
        let group = MultiThreadedEventLoopGroup(numberOfThreads: 2)
        defer {
            print("🔧 [DEBUG] Shutting down EventLoopGroup...")
            try! group.syncShutdownGracefully()
        }
        
        print("🔧 [DEBUG] Creating HandTrackingServiceProvider...")
        let provider = HandTrackingServiceProvider()
        
        print("🔧 [DEBUG] Building gRPC server with provider...")
        
        // Configure keepalive to detect dead connections
        let keepalive = ServerConnectionKeepalive(
            interval: .seconds(5),
            timeout: .seconds(3),
            permitWithoutCalls: true
        )
        
        let server = GRPC.Server.insecure(group: group)
            .withKeepalive(keepalive)
            .withServiceProviders([provider])
            .bind(host: host, port: port)
        
        print("🔧 [DEBUG] Server binding initiated, waiting for result...")
        server.map {
            $0.channel.localAddress
        }.whenSuccess { address in
            print("🟢 gRPC server started on \(address!) port \(address!.port!)")
            print("✅ [DEBUG] Server is now accepting connections")
            DataManager.shared.grpcServerReady = true
        }
        
        server.whenFailure { error in
            print("❌ [DEBUG] Server failed to start: \(error)")
        }
        
        //         Wait on the server's `onClose` future to stop the program from exiting.
        _ = try! server.flatMap {
            $0.onClose
        }.wait()
    }
}

func fill_handUpdate() -> Handtracking_HandUpdate {
    var handUpdate = Handtracking_HandUpdate()
    
    // Assuming DataManager provides an ordered list/array of joints for leftSkeleton and rightSkeleton
    let leftJoints = DataManager.shared.latestHandTrackingData.leftSkeleton.joints // Your actual data structure access method might differ
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
