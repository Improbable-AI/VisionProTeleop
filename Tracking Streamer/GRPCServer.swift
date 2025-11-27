import GRPCCore
import GRPCNIOTransportHTTP2
import GRPCProtobuf
import QuartzCore

/// Manages the gRPC server for hand tracking using grpc-swift-2
@available(macOS 15.0, iOS 18.0, watchOS 11.0, tvOS 18.0, visionOS 2.0, *)
final class GRPCServerManager: ObservableObject {
    private var grpcServer: GRPCServer<HTTP2ServerTransport.Posix>?
    private var currentPort: Int = 12345
    
    func startServer(port: Int = 12345) async {
        self.currentPort = port
        print("🚀 Starting gRPC server...")
        do {
            // Create the service implementation
            let handTrackingService = HandTrackingServiceImpl()
            print("✅ Created HandTrackingServiceImpl")
            
            // Create the gRPC server with NIO transport
            let transport = HTTP2ServerTransport.Posix(
                address: .ipv4(host: "0.0.0.0", port: currentPort),
                transportSecurity: .plaintext
            )
            print("✅ Created HTTP2ServerTransport on port \(currentPort)")
            
            let server = GRPCServer(
                transport: transport,
                services: [handTrackingService]
            )
            print("✅ Created GRPCServer with service")
            
            self.grpcServer = server
            
            // Mark server as ready
            await MainActor.run {
                DataManager.shared.grpcServerReady = true
            }
            
            print("🎯 Starting server.serve()...")
            try await server.serve()
            print("🚀 gRPC server started successfully on port \(currentPort)")
            
        } catch {
            print("❌ Failed to start gRPC server: \(error)")
            print("🔍 Error details: \(error.localizedDescription)")
        }
    }
    
    func stopServer() async {
        if let server = self.grpcServer {
            server.beginGracefulShutdown()
            print("🛑 gRPC server stopped")
            self.grpcServer = nil
        } else {
            print("ℹ️ gRPC server was not running")
        }
    }
}

/// Hand tracking service implementation using grpc-swift-2 SimpleServiceProtocol
@available(macOS 15.0, iOS 18.0, watchOS 11.0, tvOS 18.0, visionOS 2.0, *)
struct HandTrackingServiceImpl: Handtracking_HandTrackingService.SimpleServiceProtocol {
    
    init() {
        print("🎯 HandTrackingServiceImpl initialized")
    }
    
    func streamHandUpdates(
        request: Handtracking_HandUpdate,
        response: GRPCCore.RPCWriter<Handtracking_HandUpdate>,
        context: GRPCCore.ServerContext
    ) async throws {
        print("📥 [DEBUG] streamHandUpdates called - client connected!")
        
        // Track if this is a WebRTC info-only connection (will close immediately)
        let isWebRTCInfoOnly = request.head.m00 == 999.0
        
        // Check if this is a special "discovery" message from Python
        // Python will send a message with Head.m00 = 888.0 to announce its presence
        if request.head.m00 == 888.0 {
            print("✨ [DEBUG] Discovery message detected!")
            let ip1 = Int(request.head.m01)
            let ip2 = Int(request.head.m02)
            let ip3 = Int(request.head.m03)
            let ip4 = Int(request.head.m10)
            let pythonIP = "\(ip1).\(ip2).\(ip3).\(ip4)"
            print("🔍 Python client discovered at: \(pythonIP)")
            print("💾 [DEBUG] Storing Python IP in DataManager...")
            await MainActor.run {
                DataManager.shared.pythonClientIP = pythonIP
            }
        } else if isWebRTCInfoOnly {
            // WebRTC server info message
            print("🎞️ [DEBUG] WebRTC server info message detected (info-only connection)!")
            let ip1 = Int(request.head.m01)
            let ip2 = Int(request.head.m02)
            let ip3 = Int(request.head.m03)
            let ip4 = Int(request.head.m10)
            let port = Int(request.head.m11)
            let stereoVideo = request.head.m12 > 0.5
            let stereoAudio = request.head.m13 > 0.5
            let audioEnabled = request.head.m20 > 0.5
            let videoEnabled = request.head.m21 > 0.5
            let simEnabled = request.head.m22 > 0.5
            let host = "\(ip1).\(ip2).\(ip3).\(ip4)"
            print("🎞️ WebRTC server available at: \(host):\(port) (video=\(videoEnabled), audio=\(audioEnabled), sim=\(simEnabled))")
            print("💾 [DEBUG] Storing WebRTC info in DataManager...")
            
            await MainActor.run {
                let hadConnection = DataManager.shared.webrtcServerInfo != nil
                DataManager.shared.webrtcServerInfo = (host: host, port: port)
                DataManager.shared.stereoEnabled = stereoVideo
                DataManager.shared.stereoAudioEnabled = stereoAudio
                DataManager.shared.audioEnabled = audioEnabled
                DataManager.shared.videoEnabled = videoEnabled
                DataManager.shared.simEnabled = simEnabled
                if DataManager.shared.webrtcGeneration < 0 || !hadConnection {
                    DataManager.shared.webrtcGeneration = 1
                } else {
                    DataManager.shared.webrtcGeneration += 1
                }
                print("🔄 [DEBUG] Set WebRTC generation to \(DataManager.shared.webrtcGeneration)")
            }
            
            // Send one response and return for info-only connections
            try await response.write(fill_handUpdate())
            return
        } else {
            print("⚠️ [DEBUG] Not a special message (expected m00=888.0 or 999.0, got \(request.head.m00))")
        }
        
        // Register for benchmark events
        if !isWebRTCInfoOnly {
            BenchmarkEventDispatcher.shared.register(responseWriter: response)
        }
        
        print("🔄 [DEBUG] Starting hand tracking data stream...")
        print("⏱️ [DEBUG] Starting hand tracking updates...")
        
        var updateCount = 0
        
        // Stream hand tracking data until cancelled
        while !Task.isCancelled {
            let handUpdate = fill_handUpdate()
            updateCount += 1
            
            if updateCount == 1 || updateCount % 100 == 0 {
                print("📤 [DEBUG] Sending hand update #\(updateCount)...")
            }
            
            do {
                try await response.write(handUpdate)
            } catch {
                print("🔌 [DEBUG] Client disconnected or error writing: \(error)")
                break
            }
            
            // Stream at approximately 200Hz (5ms delay)
            try await Task.sleep(nanoseconds: 5_000_000)
        }
        
        print("🔌 [DEBUG] Stream ended. Sent \(updateCount) updates.")
        
        // Cleanup on disconnect
        if !isWebRTCInfoOnly {
            BenchmarkEventDispatcher.shared.clear()
            
            await MainActor.run {
                print("🧹 [DEBUG] Cleaning up connection state after main client disconnect")
                DataManager.shared.pythonClientIP = nil
                DataManager.shared.webrtcServerInfo = nil
                DataManager.shared.webrtcGeneration = -1
            }
        }
    }
}
