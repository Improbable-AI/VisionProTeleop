import SwiftUI
import RealityKit
import ARKit
import GRPC
import NIO

struct Skeleton {
    var j1: simd_float4x4 = simd_float4x4(1)
    var j2: simd_float4x4 = simd_float4x4(1)
    var j3: simd_float4x4 = simd_float4x4(1)
    var j4: simd_float4x4 = simd_float4x4(1)
    var j5: simd_float4x4 = simd_float4x4(1)
    var j6: simd_float4x4 = simd_float4x4(1)
    var j7: simd_float4x4 = simd_float4x4(1)
    var j8: simd_float4x4 = simd_float4x4(1)
    var j9: simd_float4x4 = simd_float4x4(1)
    var j10: simd_float4x4 = simd_float4x4(1)
    var j11: simd_float4x4 = simd_float4x4(1)
    var j12: simd_float4x4 = simd_float4x4(1)
    var j13: simd_float4x4 = simd_float4x4(1)
    var j14: simd_float4x4 = simd_float4x4(1)
    var j15: simd_float4x4 = simd_float4x4(1)
    var j16: simd_float4x4 = simd_float4x4(1)
    var j17: simd_float4x4 = simd_float4x4(1)
    var j18: simd_float4x4 = simd_float4x4(1)
    var j19: simd_float4x4 = simd_float4x4(1)
    var j20: simd_float4x4 = simd_float4x4(1)
    var j21: simd_float4x4 = simd_float4x4(1)
    var j22: simd_float4x4 = simd_float4x4(1)
    var j23: simd_float4x4 = simd_float4x4(1)
    var j24: simd_float4x4 = simd_float4x4(1)
}

struct HandTrackingData {
    var leftWrist: simd_float4x4 = simd_float4x4(1)
    var rightWrist: simd_float4x4 = simd_float4x4(1)
    var leftSkeleton: Skeleton = Skeleton()
    var rightSkeleton: Skeleton = Skeleton()
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

    let rootEntity = Entity()
//    private let lineEntity = 🧩Entity.line()
    private let fingerEntities: [HandAnchor.Chirality: Entity] = 🧩Entity.fingerTips()
}

extension 🥽AppModel {
//    func setUpChildEntities() {
////        self.rootEntity.addChild(self.lineEntity)
//        self.fingerEntities.values.forEach { self.rootEntity.addChild($0) }
//    }
//        
//    func observeAuthorizationStatus() {
//        Task {
//            self.authorizationStatus = await self.session.queryAuthorization(for: [.handTracking])[.handTracking]
//            
//            for await update in self.session.events {
//                if case .authorizationChanged(let type, let status) = update {
//                    if type == .handTracking { self.authorizationStatus = status }
//                } else {
//                    print("Another session event \(update).")
//                }
//            }
//        }
//    }
    
    func run() {
#if targetEnvironment(simulator)
        print("Not support handTracking in simulator.")
#else
        
        Task {
            @MainActor in
            do {
                try await self.session.run([self.handTracking])
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

fileprivate extension 🥽AppModel {
    private func processHandUpdates() async {
        for await update in self.handTracking.anchorUpdates {
            let handAnchor = update.anchor
            
            print("processHandUpates is running.")
            switch handAnchor.chirality {
            case .left:
                print("left hand")
//              index Finger
                guard handAnchor.isTracked,
                      let j1 = handAnchor.handSkeleton?.joint(.indexFingerTip),
                      j1.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j2 = handAnchor.handSkeleton?.joint(.indexFingerKnuckle),
                      j2.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j3 = handAnchor.handSkeleton?.joint(.indexFingerMetacarpal),
                      j3.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j4 = handAnchor.handSkeleton?.joint(.indexFingerIntermediateTip),
                      j4.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j5 = handAnchor.handSkeleton?.joint(.indexFingerIntermediateBase),
                      j5.isTracked else { continue }
                
                
//               Middle Finger
                guard handAnchor.isTracked,
                      let j6 = handAnchor.handSkeleton?.joint(.middleFingerTip),
                      j6.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j7 = handAnchor.handSkeleton?.joint(.middleFingerKnuckle),
                      j7.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j8 = handAnchor.handSkeleton?.joint(.middleFingerMetacarpal),
                      j8.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j9 = handAnchor.handSkeleton?.joint(.middleFingerIntermediateTip),
                      j9.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j10 = handAnchor.handSkeleton?.joint(.middleFingerIntermediateBase),
                      j10.isTracked else { continue }

//                ringFinger
                guard handAnchor.isTracked,
                      let j11 = handAnchor.handSkeleton?.joint(.ringFingerTip),
                      j11.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j12 = handAnchor.handSkeleton?.joint(.ringFingerKnuckle),
                      j12.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j13 = handAnchor.handSkeleton?.joint(.ringFingerMetacarpal),
                      j13.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j14 = handAnchor.handSkeleton?.joint(.ringFingerIntermediateTip),
                      j14.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j15 = handAnchor.handSkeleton?.joint(.ringFingerIntermediateBase),
                      j15.isTracked else { continue }
                
                
                
//                littleFinger
                guard handAnchor.isTracked,
                      let j16 = handAnchor.handSkeleton?.joint(.littleFingerTip),
                      j16.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j17 = handAnchor.handSkeleton?.joint(.littleFingerKnuckle),
                      j17.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j18 = handAnchor.handSkeleton?.joint(.littleFingerMetacarpal),
                      j18.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j19 = handAnchor.handSkeleton?.joint(.littleFingerIntermediateTip),
                      j19.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j20 = handAnchor.handSkeleton?.joint(.littleFingerIntermediateBase),
                      j20.isTracked else { continue }
                
//                thumbFinger
                guard handAnchor.isTracked,
                      let j21 = handAnchor.handSkeleton?.joint(.thumbTip),
                      j21.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j22 = handAnchor.handSkeleton?.joint(.thumbKnuckle),
                      j22.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j23 = handAnchor.handSkeleton?.joint(.thumbIntermediateTip),
                      j23.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j24 = handAnchor.handSkeleton?.joint(.thumbIntermediateBase),
                      j24.isTracked else { continue }
                
                
                
                DispatchQueue.main.async {
                    DataManager.shared.latestHandTrackingData.leftWrist = handAnchor.originFromAnchorTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j1 = j1.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j2 = j2.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j3 = j3.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j4 = j4.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j5 = j5.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j6 = j6.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j7 = j7.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j8 = j8.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j9 = j9.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j10 = j10.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j11 = j11.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j12 = j12.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j13 = j13.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j14 = j14.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j15 = j15.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j16 = j16.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j17 = j17.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j18 = j18.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j19 = j19.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j20 = j20.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j21 = j21.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j22 = j22.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j23 = j23.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.leftSkeleton.j24 = j24.anchorFromJointTransform
                    print("updated left Hand")
                    // Repeat for right hand and other fingers as needed
                }
                
            case .right:
                print("right hand")
                //              index Finger
                guard handAnchor.isTracked,
                      let j1 = handAnchor.handSkeleton?.joint(.indexFingerTip),
                      j1.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j2 = handAnchor.handSkeleton?.joint(.indexFingerKnuckle),
                      j2.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j3 = handAnchor.handSkeleton?.joint(.indexFingerMetacarpal),
                      j3.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j4 = handAnchor.handSkeleton?.joint(.indexFingerIntermediateTip),
                      j4.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j5 = handAnchor.handSkeleton?.joint(.indexFingerIntermediateBase),
                      j5.isTracked else { continue }
                                
                                
                //               Middle Finger
                guard handAnchor.isTracked,
                      let j6 = handAnchor.handSkeleton?.joint(.middleFingerTip),
                      j6.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j7 = handAnchor.handSkeleton?.joint(.middleFingerKnuckle),
                      j7.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j8 = handAnchor.handSkeleton?.joint(.middleFingerMetacarpal),
                      j8.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j9 = handAnchor.handSkeleton?.joint(.middleFingerIntermediateTip),
                      j9.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j10 = handAnchor.handSkeleton?.joint(.middleFingerIntermediateBase),
                      j10.isTracked else { continue }

                //                ringFinger
                guard handAnchor.isTracked,
                      let j11 = handAnchor.handSkeleton?.joint(.ringFingerTip),
                      j11.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j12 = handAnchor.handSkeleton?.joint(.ringFingerKnuckle),
                      j12.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j13 = handAnchor.handSkeleton?.joint(.ringFingerMetacarpal),
                      j13.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j14 = handAnchor.handSkeleton?.joint(.ringFingerIntermediateTip),
                      j14.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j15 = handAnchor.handSkeleton?.joint(.ringFingerIntermediateBase),
                      j15.isTracked else { continue }
                                
                                
                                
                //                littleFinger
                guard handAnchor.isTracked,
                      let j16 = handAnchor.handSkeleton?.joint(.littleFingerTip),
                      j16.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j17 = handAnchor.handSkeleton?.joint(.littleFingerKnuckle),
                      j17.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j18 = handAnchor.handSkeleton?.joint(.littleFingerMetacarpal),
                      j18.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j19 = handAnchor.handSkeleton?.joint(.littleFingerIntermediateTip),
                      j19.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j20 = handAnchor.handSkeleton?.joint(.littleFingerIntermediateBase),
                      j20.isTracked else { continue }
                                
                //                thumbFinger
                guard handAnchor.isTracked,
                      let j21 = handAnchor.handSkeleton?.joint(.thumbTip),
                      j21.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j22 = handAnchor.handSkeleton?.joint(.thumbKnuckle),
                      j22.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j23 = handAnchor.handSkeleton?.joint(.thumbIntermediateTip),
                      j23.isTracked else { continue }
                guard handAnchor.isTracked,
                      let j24 = handAnchor.handSkeleton?.joint(.thumbIntermediateBase),
                      j24.isTracked else { continue }
                
                
                
                DispatchQueue.main.async {
                    DataManager.shared.latestHandTrackingData.rightWrist = handAnchor.originFromAnchorTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j1 = j1.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j2 = j2.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j3 = j3.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j4 = j4.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j5 = j5.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j6 = j6.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j7 = j7.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j8 = j8.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j9 = j9.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j10 = j10.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j11 = j11.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j12 = j12.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j13 = j13.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j14 = j14.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j15 = j15.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j16 = j16.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j17 = j17.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j18 = j18.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j19 = j19.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j20 = j20.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j21 = j21.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j22 = j22.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j23 = j23.anchorFromJointTransform
                    DataManager.shared.latestHandTrackingData.rightSkeleton.j24 = j24.anchorFromJointTransform
                    print("updated right Hand")
                    // Repeat for right hand and other fingers as needed
                }
            }
            
            //            let originFromWrist = handAnchor.originFromAnchorTransform
            //
            //            let wristFromIndex = indexfingerTip.anchorFromJointTransform
            //            let originFromIndex = originFromWrist * wristFromIndex
            //            self.fingerEntities[handAnchor.chirality]?.setTransformMatrix(originFromIndex,
            //                                                                          relativeTo: nil)
            //                    }
        }
        
        
    }
}
//



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
            var handUpdate = Handtracking_HandUpdate()
            
            print("sending...")
            
            // Simulate hand tracking data update
            handUpdate.leftHand.x = Float(DataManager.shared.latestHandTrackingData.leftWrist.columns.0.x)  // Float(DataManager.shared.latestHandTrackingData.leftHand.x) // (latest?.leftHand.x)!
            handUpdate.leftHand.y = 1.0 //Float(DataManager.shared.latestHandTrackingData.leftHand.y) //(latest?.leftHand.y)!
            handUpdate.leftHand.z = 1.0 //Float(DataManager.shared.latestHandTrackingData.leftHand.z) //(latest?.leftHand.z)!
            handUpdate.rightHand.x = 1.0 //Float(DataManager.shared.latestHandTrackingData.rightHand.x) //(latest?.leftHand.x)!
            handUpdate.rightHand.y = 1.0 //Float(DataManager.shared.latestHandTrackingData.rightHand.y) //(latest?.leftHand.y)!
            handUpdate.rightHand.z = 1.0 //Float(DataManager.shared.latestHandTrackingData.rightHand.z) //(latest?.leftHand.z)!
            
            // Send the update to the client.
            return context.sendResponse(handUpdate).map { _ in }
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


