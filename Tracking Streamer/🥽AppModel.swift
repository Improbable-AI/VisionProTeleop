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

class DataManager {
    static let shared = DataManager()
    
    var latestHandTrackingData: HandTrackingData = HandTrackingData()
    
    private init() {}
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
            print("reconstruction update")
            let meshAnchor = update.anchor
            let mesh_description = meshAnchor.geometry.description
            print(mesh_description)
        }
    }

    
    @MainActor
    private func queryAndProcessLatestDeviceAnchor() async {
        // Device anchors are only available when the provider is running.\
        guard worldTracking.state == .running else { return }
        
        let deviceAnchor = worldTracking.queryDeviceAnchor(atTimestamp: CACurrentMediaTime())
        print(" *** device tracking running ")
//        print(deviceAnchor?.originFromAnchorTransform)
        guard let deviceAnchor, deviceAnchor.isTracked else { return }
        DataManager.shared.latestHandTrackingData.Head = deviceAnchor.originFromAnchorTransform
            }

    private func processHandUpdates() async {
        for await update in self.handTracking.anchorUpdates {
            let handAnchor = update.anchor
            print("processHandUpates is running.")
            switch handAnchor.chirality {
            case .left:
                DispatchQueue.main.async {
                    DataManager.shared.latestHandTrackingData.leftWrist = handAnchor.originFromAnchorTransform
                    print(handAnchor.originFromAnchorTransform)
                    

                    let jointTypes: [HandSkeleton.JointName] = [
                        .wrist,
                        .thumbKnuckle, .thumbIntermediateBase, .thumbIntermediateTip, .thumbTip,
                        .indexFingerMetacarpal, .indexFingerKnuckle, .indexFingerIntermediateBase, .indexFingerIntermediateTip, .indexFingerTip,
                        .middleFingerMetacarpal, .middleFingerKnuckle, .middleFingerIntermediateBase, .middleFingerIntermediateTip, .middleFingerTip,
                        .ringFingerMetacarpal, .ringFingerKnuckle, .ringFingerIntermediateBase, .ringFingerIntermediateTip, .ringFingerTip,
                        .littleFingerMetacarpal, .littleFingerKnuckle, .littleFingerIntermediateBase, .littleFingerIntermediateTip, .littleFingerTip,
                    ]
                    
                    for (index, jointType) in jointTypes.enumerated() {
                        guard let joint = handAnchor.handSkeleton?.joint(jointType), joint.isTracked else {
                            continue
                        }
                        DataManager.shared.latestHandTrackingData.leftSkeleton.joints[index] = joint.anchorFromJointTransform
                    }
                    
                    print("Updated left hand skeleton")
                    // Repeat for right hand and other fingers as needed
                }

            case .right:
                DispatchQueue.main.async {
                    DataManager.shared.latestHandTrackingData.rightWrist = handAnchor.originFromAnchorTransform
                    print(handAnchor.originFromAnchorTransform)
                    
                    let jointTypes: [HandSkeleton.JointName] = [
                        .wrist,
                        .thumbKnuckle, .thumbIntermediateBase, .thumbIntermediateTip, .thumbTip,
                        .indexFingerMetacarpal, .indexFingerKnuckle, .indexFingerIntermediateBase, .indexFingerIntermediateTip, .indexFingerTip,
                        .middleFingerMetacarpal, .middleFingerKnuckle, .middleFingerIntermediateBase, .middleFingerIntermediateTip, .middleFingerTip,
                        .ringFingerMetacarpal, .ringFingerKnuckle, .ringFingerIntermediateBase, .ringFingerIntermediateTip, .ringFingerTip,
                        .littleFingerMetacarpal, .littleFingerKnuckle, .littleFingerIntermediateBase, .littleFingerIntermediateTip, .littleFingerTip,
                    ]
 
                    for (index, jointType) in jointTypes.enumerated() {
                        guard let joint = handAnchor.handSkeleton?.joint(jointType), joint.isTracked else {
                            continue
                        }
                        print(index)
                        DataManager.shared.latestHandTrackingData.rightSkeleton.joints[index] = joint.anchorFromJointTransform
                    }
                    
                    print("Updated right hand skeleton")
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
        print("hey...")
        // Example task to simulate sending hand tracking data.
        // In a real application, you would replace this with actual data collection and streaming.
        let task = eventLoop.scheduleRepeatedAsyncTask(initialDelay: .milliseconds(10), delay: .milliseconds(10)) { task -> EventLoopFuture<Void> in
//            var handUpdate = Handtracking_HandUpdate()
            
            let recent_hand = fill_handUpdate()
            print("sending...")
            
            // Send the update to the client.
            return context.sendResponse(recent_hand).map { _ in }
        }

        // Ensure the task is cancelled when the client disconnects or the stream is otherwise closed.
        context.statusPromise.futureResult.whenComplete { _ in task.cancel() }

        // Return a future that will complete when the streaming operation is done.
        // Here, we're indicating that the stream will remain open indefinitely until the client disconnects.
        return eventLoop.makePromise(of: GRPCStatus.self).futureResult
    }
}

func startServer() {
    DispatchQueue.global().async {
        
        let port = 12345
        let host = "0.0.0.0"
        
        let group = MultiThreadedEventLoopGroup(numberOfThreads: 2)
        defer {
            try! group.syncShutdownGracefully()
        }
        
        let provider = HandTrackingServiceProvider()
        
        let server = GRPC.Server.insecure(group: group)
            .withServiceProviders([provider])
            .bind(host: host, port: port)
        
        server.map {
            $0.channel.localAddress
        }.whenSuccess { address in
            print("server started on \(address!) \(address!.port!) ")
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
