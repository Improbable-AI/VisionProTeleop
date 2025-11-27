import GRPCCore
import GRPCNIOTransportHTTP2
import GRPCProtobuf
import QuartzCore
import simd
import RealityKit
import Foundation

/// Manages the MuJoCo AR gRPC server
@available(macOS 15.0, iOS 18.0, watchOS 11.0, tvOS 18.0, visionOS 2.0, *)
final class MuJoCoGRPCServerManager: ObservableObject {
    private var grpcServer: GRPCServer<HTTP2ServerTransport.Posix>?
    private var currentPort: Int = 50051
    
    func startServer(streamingView: MuJoCoStreamingView, port: Int = 50051) async {
        self.currentPort = port
        print("🚀 Starting MuJoCo gRPC server...")
        do {
            let mujocoService = MuJoCoARServiceImpl(streamingView: streamingView)
            print("✅ Created MuJoCoARServiceImpl")
            
            let transport = HTTP2ServerTransport.Posix(
                address: .ipv4(host: "0.0.0.0", port: currentPort),
                transportSecurity: .plaintext
            )
            print("✅ Created HTTP2ServerTransport on port \(currentPort)")
            
            let server = GRPCServer(
                transport: transport,
                services: [mujocoService]
            )
            print("✅ Created GRPCServer with MuJoCo AR service")
            
            self.grpcServer = server
            
            print("🎯 Starting server.serve()...")
            try await server.serve()
            print("🚀 MuJoCo gRPC server started successfully on port \(currentPort)")
            
        } catch {
            print("❌ Failed to start MuJoCo gRPC server: \(error)")
            print("🔍 Error details: \(error.localizedDescription)")
        }
    }
    
    func stopServer() async {
        if let server = self.grpcServer {
            server.beginGracefulShutdown()
            print("🛑 MuJoCo gRPC server stopped")
            self.grpcServer = nil
        } else {
            print("ℹ️ MuJoCo gRPC server was not running")
        }
    }
}

/// MuJoCo AR service implementation
@available(macOS 15.0, iOS 18.0, watchOS 11.0, tvOS 18.0, visionOS 2.0, *)
struct MuJoCoARServiceImpl: MujocoAr_MuJoCoARService.SimpleServiceProtocol {
    var streamingView: MuJoCoStreamingView?
    
    private let poseProcessingQueue = DispatchQueue(label: "com.mujocoAR.poseProcessing", qos: .userInteractive)
    private let rateLimiter = RateLimiter(maxFPS: 60)
    
    init(streamingView: MuJoCoStreamingView) {
        self.streamingView = streamingView
        print("🎯 MuJoCoARServiceImpl initialized")
        print("📋 Available methods:")
        print("   - sendUsdzUrl")
        print("   - sendUsdzData")
        print("   - sendUsdzDataChunked")
        print("   - updatePoses")
        print("   - streamPoses")
        print("   - streamHandTracking")
    }
    
    // MARK: - SendUsdzUrl
    func sendUsdzUrl(
        request: MujocoAr_UsdzUrlRequest,
        context: ServerContext
    ) async throws -> MujocoAr_UsdzUrlResponse {
        print("📨 [sendUsdzUrl] Received request")
        print("📨 [sendUsdzUrl] USDZ URL: \(request.usdzURL)")
        print("📨 [sendUsdzUrl] Session ID: \(request.sessionID)")
        
        await MainActor.run {
            print("📨 [sendUsdzUrl] Updating streaming view...")
            streamingView?.updateUsdzURL(request.usdzURL)
            print("📨 [sendUsdzUrl] Streaming view updated")
        }
        
        var response = MujocoAr_UsdzUrlResponse()
        response.success = true
        response.message = "USDZ URL received successfully"
        print("📨 [sendUsdzUrl] Sending success response")
        return response
    }
    
    // MARK: - SendUsdzData
    func sendUsdzData(
        request: MujocoAr_UsdzDataRequest,
        context: ServerContext
    ) async throws -> MujocoAr_UsdzDataResponse {
        print("📨 [sendUsdzData] *** METHOD CALLED ***")
        print("📨 [sendUsdzData] Received USDZ data: \(request.usdzData.count) bytes")
        print("📨 [sendUsdzData] Filename: \(request.filename)")
        print("📨 [sendUsdzData] Session ID: \(request.sessionID)")
        
        var attachToPosition: SIMD3<Float>? = nil
        var attachToRotation: simd_quatf? = nil
        
        if request.hasAttachToPosition && request.hasAttachToRotation {
            attachToPosition = SIMD3<Float>(request.attachToPosition.x, request.attachToPosition.y, request.attachToPosition.z)
            attachToRotation = simd_quatf(ix: request.attachToRotation.x, iy: request.attachToRotation.y, iz: request.attachToRotation.z, r: request.attachToRotation.w)
            print("📨 [sendUsdzData] Attach to position: \(attachToPosition!), rotation: \(attachToRotation!)")
        }
        
        var response = MujocoAr_UsdzDataResponse()
        
        do {
            print("📨 [sendUsdzData] Creating temporary file...")
            let tempDir = FileManager.default.temporaryDirectory
            let fileName = request.filename.isEmpty ? "\(UUID().uuidString).usdz" : request.filename
            let localURL = tempDir.appendingPathComponent(fileName)
            
            print("📨 [sendUsdzData] Writing to: \(localURL.path)")
            try request.usdzData.write(to: localURL)
            
            print("💾 [sendUsdzData] Saved USDZ data to: \(localURL.path)")
            
            await MainActor.run {
                print("📨 [sendUsdzData] Updating streaming view with local file...")
                streamingView?.updateUsdzURL(localURL.absoluteString, attachToPosition: attachToPosition, attachToRotation: attachToRotation)
                print("📨 [sendUsdzData] Streaming view updated")
            }
            
            response.success = true
            response.message = "USDZ data received and saved successfully"
            response.localFilePath = localURL.path
            print("📨 [sendUsdzData] Sending success response")
            
        } catch {
            print("❌ [sendUsdzData] Failed to save USDZ data: \(error)")
            print("❌ [sendUsdzData] Error details: \(error.localizedDescription)")
            response.success = false
            response.message = "Failed to save USDZ data: \(error.localizedDescription)"
        }
        
        print("📨 [sendUsdzData] Returning response (success: \(response.success))")
        return response
    }
    
    // MARK: - SendUsdzDataChunked
    func sendUsdzDataChunked(
        request: RPCAsyncSequence<MujocoAr_UsdzChunkRequest, any Error>,
        context: ServerContext
    ) async throws -> MujocoAr_UsdzDataResponse {
        print("📦 [sendUsdzDataChunked] *** CHUNKED TRANSFER STARTED ***")
        
        var response = MujocoAr_UsdzDataResponse()
        var chunkData = Data()
        var fileName = ""
        var sessionID = ""
        var totalExpectedSize: Int64 = 0
        var receivedChunks = 0
        var totalChunks = 0
        var attachToPosition: SIMD3<Float>? = nil
        var attachToRotation: simd_quatf? = nil
        
        do {
            for try await chunkRequest in request {
                print("📦 [sendUsdzDataChunked] Received chunk \(chunkRequest.chunkIndex + 1)/\(chunkRequest.totalChunks)")
                print("📦 [sendUsdzDataChunked] Chunk size: \(chunkRequest.chunkData.count) bytes")
                
                if receivedChunks == 0 {
                    fileName = chunkRequest.filename
                    sessionID = chunkRequest.sessionID
                    totalExpectedSize = chunkRequest.totalSize
                    totalChunks = Int(chunkRequest.totalChunks)
                    
                    if chunkRequest.hasAttachToPosition && chunkRequest.hasAttachToRotation {
                        attachToPosition = SIMD3<Float>(chunkRequest.attachToPosition.x, chunkRequest.attachToPosition.y, chunkRequest.attachToPosition.z)
                        attachToRotation = simd_quatf(ix: chunkRequest.attachToRotation.x, iy: chunkRequest.attachToRotation.y, iz: chunkRequest.attachToRotation.z, r: chunkRequest.attachToRotation.w)
                        print("📦 [sendUsdzDataChunked] Attach to position: \(attachToPosition!), rotation: \(attachToRotation!)")
                    }
                    
                    print("📦 [sendUsdzDataChunked] Filename: \(fileName)")
                    print("📦 [sendUsdzDataChunked] Total expected size: \(totalExpectedSize) bytes")
                    print("📦 [sendUsdzDataChunked] Total chunks: \(totalChunks)")
                }
                
                chunkData.append(chunkRequest.chunkData)
                receivedChunks += 1
                
                print("📦 [sendUsdzDataChunked] Progress: \(chunkData.count)/\(totalExpectedSize) bytes")
                
                if chunkRequest.isLastChunk {
                    print("📦 [sendUsdzDataChunked] Received last chunk!")
                    break
                }
            }
            
            if chunkData.count != totalExpectedSize {
                print("⚠️ [sendUsdzDataChunked] Warning: Received \(chunkData.count) bytes, expected \(totalExpectedSize)")
            }
            
            print("📦 [sendUsdzDataChunked] All chunks received, assembling file...")
            
            let tempDir = FileManager.default.temporaryDirectory
            let finalFileName = fileName.isEmpty ? "\(UUID().uuidString).usdz" : fileName
            let localURL = tempDir.appendingPathComponent(finalFileName)
            
            print("📦 [sendUsdzDataChunked] Writing assembled file to: \(localURL.path)")
            try chunkData.write(to: localURL)
            
            print("💾 [sendUsdzDataChunked] Successfully saved complete USDZ file (\(chunkData.count) bytes)")
            
            await MainActor.run {
                print("📦 [sendUsdzDataChunked] Updating streaming view with assembled file...")
                streamingView?.updateUsdzURL(localURL.absoluteString, attachToPosition: attachToPosition, attachToRotation: attachToRotation)
                print("📦 [sendUsdzDataChunked] Streaming view updated")
            }
            
            response.success = true
            response.message = "Chunked USDZ data received and assembled successfully (\(receivedChunks) chunks, \(chunkData.count) bytes)"
            response.localFilePath = localURL.path
            print("📦 [sendUsdzDataChunked] Sending success response")
            
        } catch {
            print("❌ [sendUsdzDataChunked] Failed to process chunked data: \(error)")
            response.success = false
            response.message = "Failed to process chunked data: \(error.localizedDescription)"
        }
        
        return response
    }
    
    // MARK: - UpdatePoses
    func updatePoses(
        request: MujocoAr_PoseUpdateRequest,
        context: ServerContext
    ) async throws -> MujocoAr_PoseUpdateResponse {
        print("📨 [updatePoses] Received pose update with \(request.bodyPoses.count) bodies")
        
        var posesMutable: [String: MujocoAr_BodyPose] = [:]
        for bodyPose in request.bodyPoses {
            posesMutable[bodyPose.bodyName] = bodyPose
        }
        let poses = posesMutable
        
        await withCheckedContinuation { continuation in
            poseProcessingQueue.async {
                Task { @MainActor in
                    guard let streamingView = streamingView else {
                        continuation.resume()
                        return
                    }
                    
                    let finalTransforms = streamingView.computeFinalTransforms(poses)
                    streamingView.updatePosesWithTransforms(finalTransforms)
                    
                    continuation.resume()
                }
            }
        }
        
        var response = MujocoAr_PoseUpdateResponse()
        response.success = true
        response.message = "Poses updated successfully"
        response.bodiesUpdated = Int32(request.bodyPoses.count)
        return response
    }
    
    // MARK: - StreamPoses
    func streamPoses(
        request: RPCAsyncSequence<MujocoAr_PoseUpdateRequest, any Error>,
        response: RPCWriter<MujocoAr_PoseUpdateResponse>,
        context: ServerContext
    ) async throws {
        print("🔄 [streamPoses] Starting pose streaming...")
        
        do {
            for try await poseRequest in request {
                var posesMutable: [String: MujocoAr_BodyPose] = [:]
                for bodyPose in poseRequest.bodyPoses {
                    posesMutable[bodyPose.bodyName] = bodyPose
                }
                let poses = posesMutable
                
                await withCheckedContinuation { continuation in
                    poseProcessingQueue.async {
                        Task { @MainActor in
                            guard let streamingView = streamingView else {
                                continuation.resume()
                                return
                            }
                            
                            let finalTransforms = streamingView.computeFinalTransforms(poses)
                            streamingView.updatePosesWithTransforms(finalTransforms)
                            
                            continuation.resume()
                        }
                    }
                }
                
                var responseMsg = MujocoAr_PoseUpdateResponse()
                responseMsg.success = true
                responseMsg.message = "Stream poses updated"
                responseMsg.bodiesUpdated = Int32(poseRequest.bodyPoses.count)
                
                try await response.write(responseMsg)
            }
        } catch {
            print("❌ Error in pose streaming: \(error)")
            throw error
        }
        
        print("🔄 [streamPoses] Pose streaming ended")
    }
    
    // MARK: - StreamHandTracking
    func streamHandTracking(
        request: MujocoAr_HandTrackingRequest,
        response: RPCWriter<MujocoAr_HandTrackingUpdate>,
        context: ServerContext
    ) async throws {
        print("🖐️ [streamHandTracking] Client connected for hand tracking stream")
        print("🖐️ [streamHandTracking] Session ID: \(request.sessionID)")
        
        do {
            while !Task.isCancelled {
                if let handTrackingData = await MainActor.run(body: {
                    return streamingView?.getHandTrackingData()
                }) {
                    try await response.write(handTrackingData)
                }
                
                try await Task.sleep(nanoseconds: 10_000_000) // ~100 Hz
            }
        } catch {
            print("❌ [streamHandTracking] Error in hand tracking streaming: \(error)")
            throw error
        }
        
        print("🖐️ [streamHandTracking] Client disconnected from hand tracking stream")
    }
}

// MARK: - Rate Limiter
private class RateLimiter: @unchecked Sendable {
    private var lastUpdateTime: CFTimeInterval = 0
    private let minInterval: CFTimeInterval
    private let queue = DispatchQueue(label: "com.mujocoAR.rateLimiter")
    
    init(maxFPS: Double) {
        self.minInterval = 1.0 / maxFPS
    }
    
    func shouldAllow() -> Bool {
        return queue.sync {
            let currentTime = CACurrentMediaTime()
            if currentTime - lastUpdateTime >= minInterval {
                lastUpdateTime = currentTime
                return true
            }
            return false
        }
    }
}

// MARK: - Spatial Gen Helper (for loading USDZ)
class SpatialGenHelper {
    func loadEntity(from url: URL) async throws -> Entity {
        // Handle both local file URLs and remote URLs
        if url.isFileURL {
            // Local file - load directly
            return try await Entity(contentsOf: url)
        } else {
            // Remote URL - download first
            let (data, _) = try await URLSession.shared.data(from: url)
            
            // Save to temporary file
            let tempDir = FileManager.default.temporaryDirectory
            let tempFile = tempDir.appendingPathComponent(UUID().uuidString + ".usdz")
            try data.write(to: tempFile)
            
            // Load from temporary file
            return try await Entity(contentsOf: tempFile)
        }
    }
}
