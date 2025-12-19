import Foundation
import LiveKitWebRTC
import SwiftProtobuf
import UIKit

/// Metadata for USDZ file transfer via WebRTC
struct UsdzTransferMetadata {
    let filename: String
    let totalSize: Int
    let totalChunks: Int
    let attachPosition: [Float]?  // [x, y, z]
    let attachRotation: [Float]?  // [x, y, z, w]
}

/// WebRTC client that connects to the Python server and receives video frames
/// Supports both direct TCP connection (local) and WebSocket signaling (cross-network)
class WebRTCClient: NSObject, LKRTCPeerConnectionDelegate, @unchecked Sendable {
    private var peerConnection: LKRTCPeerConnection?
    private let factory: LKRTCPeerConnectionFactory
    private var videoTrack: LKRTCVideoTrack?
    private var statsTimer: Timer?
    private var audioTrack: LKRTCAudioTrack?
    private var handDataChannel: LKRTCDataChannel?
    private var simPosesDataChannel: LKRTCDataChannel?  // WebRTC data channel for sim pose streaming
    private var simPosesMessageCount: Int = 0  // Counter for debugging message flow
    private var lastProcessedPoseTimestamp: Double = 0  // For dropping stale frames
    private var handStreamTask: Task<Void, Never>?
    private let handStreamIntervalNanoseconds: UInt64 = 2_000_000
    private var pendingVideoRenderer: LKRTCVideoRenderer?  // Renderer waiting for track
    private var pendingAudioRenderer: LKRTCAudioRenderer?  // Renderer waiting for track
    
    // USDZ transfer state (cross-network mode)
    private var usdzDataChannel: LKRTCDataChannel?
    private var usdzTransferMetadata: UsdzTransferMetadata?
    private var usdzChunks: [Int: Data] = [:]
    private var usdzReceivedChunksCount: Int = 0
    
    var onFrameReceived: ((CVPixelBuffer) -> Void)?
    
    /// Callback for receiving simulation pose updates via WebRTC data channel
    /// Format: JSON dictionary {"t": timestamp, "p": {"body_name": [x,y,z,qx,qy,qz,qw], ...}, "q": [...], "c": [...]}
    /// Returns: (timestamp, poses, qpos, ctrl)
    var onSimPosesReceived: ((Double, [String: [Float]], [Float]?, [Float]?) -> Void)?
    
    /// Callback for connection state changes (isConnected)
    var onConnectionStateChanged: ((Bool) -> Void)?
    
    /// Callback when local ICE candidate is generated (for signaling)
    var onLocalICECandidateGenerated: ((LKRTCIceCandidate) -> Void)?
    
    private let stunServer = "stun:stun.l.google.com:19302"
    
    // ICE gathering completion tracking
    private var iceGatheringContinuation: CheckedContinuation<Void, Error>?
    private var iceGatheringResumed = false
    
    override init() {
        // Initialize WebRTC factory
        LKRTCInitializeSSL()
        
        let encoderFactory = LKRTCDefaultVideoEncoderFactory()
        let decoderFactory = LKRTCDefaultVideoDecoderFactory()
        
        self.factory = LKRTCPeerConnectionFactory(
            encoderFactory: encoderFactory,
            decoderFactory: decoderFactory
        )
        
        super.init()
    }
    
    deinit {
        handStreamTask?.cancel()
        LKRTCCleanupSSL()
    }
    
    /// Connect to the WebRTC server at the given address (Local Mode)
    func connect(to serverAddress: String, port: Int) async throws {
        // Create peer connection with STUN server for NAT traversal
        let config = LKRTCConfiguration()
        config.iceServers = [LKRTCIceServer(urlStrings: [stunServer])]
        // Enable ICE candidate pool for faster connection
        config.iceCandidatePoolSize = 10
        // Try to use all network interfaces
        config.continualGatheringPolicy = .gatherContinually
        config.tcpCandidatePolicy = .disabled
        // Set ICE transport policy to prefer local connections
        config.iceTransportPolicy = .all
        
        let constraints = LKRTCMediaConstraints(
            mandatoryConstraints: nil,
            optionalConstraints: ["DtlsSrtpKeyAgreement": "true"]
        )
        
        guard let pc = factory.peerConnection(
            with: config,
            constraints: constraints,
            delegate: self
        ) else {
            throw WebRTCError.failedToCreatePeerConnection
        }
        
        self.peerConnection = pc
        
        // Connect to server via TCP socket
        try await connectToServer(host: serverAddress, port: port)
    }
    
    /// Connect using an existing SignalingClient (Cross-Network Mode)
    /// VisionOS is the answerer - waits for Python's offer
    func connectWithSignaling(_ signalingClient: SignalingClient) async throws {
        dlog("DEBUG: Connecting with signaling client (Cross-Network Mode - Answerer)")
        await MainActor.run {
            DataManager.shared.connectionStatus = "Waiting for Python to connect..."
        }
        
        // Create peer connection with STUN/TURN servers
        let config = LKRTCConfiguration()
        config.iceServers = [LKRTCIceServer(urlStrings: [stunServer])]
        config.iceCandidatePoolSize = 10
        config.continualGatheringPolicy = .gatherContinually
        
        let constraints = LKRTCMediaConstraints(
            mandatoryConstraints: nil,
            optionalConstraints: ["DtlsSrtpKeyAgreement": "true"]
        )
        
        guard let pc = factory.peerConnection(
            with: config,
            constraints: constraints,
            delegate: self
        ) else {
            throw WebRTCError.failedToCreatePeerConnection
        }
        
        self.peerConnection = pc
        
        // Setup Signaling Callbacks
        await MainActor.run {
            // Handle incoming SDP offer from Python (NEW: we are now the answerer)
            signalingClient.onSDPOfferReceived = { [weak self] sdp in
                dlog("📥 [SIGNALING] Received SDP offer from Python")
                Task {
                    await self?.handleOfferAndCreateAnswer(sdp: sdp, signalingClient: signalingClient)
                }
            }
            
            signalingClient.onICECandidateReceived = { [weak self] candidateDict in
                guard let self = self,
                      let sdp = candidateDict["candidate"] as? String,
                      let sdpMid = candidateDict["sdpMid"] as? String,
                      let sdpMLineIndex = candidateDict["sdpMLineIndex"] as? Int32 else {
                    return
                }
                let candidate = LKRTCIceCandidate(sdp: sdp, sdpMLineIndex: sdpMLineIndex, sdpMid: sdpMid)
                self.handleRemoteCandidate(candidate)
            }
            
            // onOfferRequested is no longer needed - Python creates offers
            signalingClient.onOfferRequested = nil
            
            // Handle Peer Left
            signalingClient.onPeerLeft = { [weak self] in
                dlog("⚠️ [SIGNALING] Peer left room - closing WebRTC")
                Task { @MainActor in
                    DataManager.shared.connectionStatus = "Peer disconnected"
                    // Close connection
                    self?.peerConnection?.close()
                    self?.peerConnection = nil
                    self?.stopStatsTimer()
                    self?.onConnectionStateChanged?(false)
                }
            }
        }
        
        // Wire up local ICE candidate generation to SignalingClient
        self.onLocalICECandidateGenerated = { candidate in
            Task { @MainActor in
                signalingClient.sendICECandidate(
                    candidate: candidate.sdp,
                    sdpMid: candidate.sdpMid,
                    sdpMLineIndex: candidate.sdpMLineIndex
                )
            }
        }
        
        // NOTE: Data channels are now created by Python (the offerer)
        // They will be received via the peerConnection:didOpenDataChannel delegate
        
        await MainActor.run {
            DataManager.shared.connectionStatus = "Waiting for Python's offer..."
        }
        dlog("DEBUG: Waiting for Python to send SDP offer...")
    }
    
    /// Handle incoming SDP offer from Python and create answer
    private func handleOfferAndCreateAnswer(sdp: String, signalingClient: SignalingClient) async {
        guard let pc = self.peerConnection else {
            dlog("❌ [WEBRTC] PeerConnection not initialized, cannot handle offer")
            return
        }
        
        do {
            await MainActor.run {
                DataManager.shared.connectionStatus = "Processing Python's offer..."
            }
            
            // Set remote description (Python's offer)
            let offer = LKRTCSessionDescription(type: .offer, sdp: sdp)
            try await pc.setRemoteDescription(offer)
            dlog("DEBUG: Remote description (offer) set successfully")
            
            // Create answer
            let constraints = LKRTCMediaConstraints(mandatoryConstraints: nil, optionalConstraints: nil)
            let answer = try await pc.answer(for: constraints)
            dlog("DEBUG: Created SDP Answer")
            
            // Set local description
            try await pc.setLocalDescription(answer)
            dlog("DEBUG: Local description (answer) set")
            
            // Send answer via signaling
            await MainActor.run {
                signalingClient.sendSDPAnswer(answer.sdp)
                DataManager.shared.connectionStatus = "Sent answer, establishing connection..."
            }
            dlog("DEBUG: Sent SDP Answer to Python")
            
        } catch {
            dlog("❌ [WEBRTC] Error handling offer: \(error)")
            await MainActor.run {
                DataManager.shared.connectionStatus = "Error: \(error.localizedDescription)"
            }
        }
    }
    

    private func handleRemoteSDP(sdp: String, type: LKRTCSdpType) {
        dlog("DEBUG: Handling remote SDP (\(type.rawValue))")
        let remoteDesc = LKRTCSessionDescription(type: type, sdp: sdp)
        
        Task {
            do {
                try await self.peerConnection?.setRemoteDescription(remoteDesc)
                dlog("DEBUG: Remote description set successfully")
            } catch {
                dlog("ERROR: Failed to set remote description: \(error)")
            }
        }
    }
    
    private func handleRemoteCandidate(_ candidate: LKRTCIceCandidate) {
        dlog("DEBUG: Adding remote ICE candidate")
        self.peerConnection?.add(candidate)
    }
    
    private func connectToServer(host: String, port: Int) async throws {
        dlog("DEBUG: Attempting to connect to \(host):\(port)")
        await MainActor.run {
            DataManager.shared.connectionStatus = "Connecting to \(host):\(port)..."
        }
        let (inputStream, outputStream) = try await AsyncSocketConnection.connect(
            host: host,
            port: port
        )
        dlog("DEBUG: Socket connection established to \(host):\(port)")
        await MainActor.run {
            DataManager.shared.connectionStatus = "Socket connected to \(host):\(port)"
        }
        
        // Read offer from server
        guard let offerData = try await inputStream.readLine(),
              let offerJson = try? JSONDecoder().decode(SDPMessage.self, from: offerData) else {
            throw WebRTCError.invalidOffer
        }
        
        dlog("DEBUG: Received offer from server")
        await MainActor.run {
            DataManager.shared.connectionStatus = "Received offer from server"
        }
        
        // Set remote description (offer)
        let remoteDesc = LKRTCSessionDescription(type: .offer, sdp: offerJson.sdp)
        try await peerConnection?.setRemoteDescription(remoteDesc)
        
        // Create answer
        guard let answer = try await peerConnection?.answer(for: LKRTCMediaConstraints(
            mandatoryConstraints: nil,
            optionalConstraints: nil
        )) else {
            throw WebRTCError.failedToCreateAnswer
        }
        
        // Set local description (answer)
        try await peerConnection?.setLocalDescription(answer)
        
        // Wait for ICE gathering to complete
        dlog("DEBUG: Waiting for ICE gathering to complete")
        await MainActor.run {
            DataManager.shared.connectionStatus = "Waiting for ICE gathering..."
        }
        try await waitForICEGatheringComplete()
        
        // Send answer to server
        guard let localSDP = peerConnection?.localDescription else {
            throw WebRTCError.noLocalDescription
        }
        
        let answerMessage = SDPMessage(
            sdp: localSDP.sdp,
            type: localSDP.type.stringValue
        )
        
        let answerData = try JSONEncoder().encode(answerMessage)
        var answerString = String(data: answerData, encoding: .utf8)! + "\n"
        try await outputStream.write(answerString.data(using: .utf8)!)
        
        dlog("DEBUG: Answer sent to server")
        await MainActor.run {
            DataManager.shared.connectionStatus = "Answer sent, waiting for video..."
        }
    }

    private func startHandTrackingStream(on channel: LKRTCDataChannel) {
        handStreamTask?.cancel()

        let interval = handStreamIntervalNanoseconds
        handStreamTask = Task { [weak self] in
            while !Task.isCancelled {
                guard let _ = self else { return }
                do {
                    let update = fill_handUpdate()
                    let payload = try update.serializedData()
                    let buffer = LKRTCDataBuffer(data: payload, isBinary: true)

                    let sendResult = await MainActor.run { () -> Bool in
                        guard let strongChannel = self?.handDataChannel else { return false }
                        if strongChannel.readyState != .open {
                            return false
                        }
                        return strongChannel.sendData(buffer)
                    }

                    if !sendResult {
                        dlog("ERROR: Failed to send hand tracking update over WebRTC (channel not open or backpressure)")
                    }
                } catch {
                    dlog("ERROR: Unable to serialize hand tracking update: \(error)")
                }

                do {
                    try await Task.sleep(nanoseconds: interval)
                } catch {
                    return
                }
            }
        }
    }

    private func stopHandTrackingStream() {
        handStreamTask?.cancel()
        handStreamTask = nil
        handDataChannel = nil
    }
    
    @MainActor
    private func waitForICEGatheringComplete() async throws {
        return try await withCheckedThrowingContinuation { continuation in
            self.iceGatheringContinuation = continuation
            self.iceGatheringResumed = false
            
            // Set up timeout as fallback (500ms should be plenty)
            Task {
                try? await Task.sleep(nanoseconds: 500_000_000)
                if !self.iceGatheringResumed, let cont = self.iceGatheringContinuation {
                    self.iceGatheringResumed = true
                    self.iceGatheringContinuation = nil
                    dlog("DEBUG: ICE gathering timed out after 500ms (proceeding anyway)")
                    Task { @MainActor in
                        DataManager.shared.connectionStatus = "ICE ready"
                    }
                    cont.resume()
                }
            }
            
            // Check if already complete
            if let pc = self.peerConnection, pc.iceGatheringState == .complete {
                self.iceGatheringResumed = true
                self.iceGatheringContinuation = nil
                dlog("DEBUG: ICE gathering already complete")
                Task { @MainActor in
                    DataManager.shared.connectionStatus = "ICE gathering complete"
                }
                continuation.resume()
            }
        }
    }
    
    func addVideoRenderer(_ renderer: LKRTCVideoRenderer) {
        if let track = videoTrack {
            track.add(renderer)
            dlog("DEBUG: Video renderer attached to track - track enabled: \(track.isEnabled)")
        } else {
            // Store renderer to attach when track arrives (cross-network timing)
            dlog("INFO: No video track yet, storing renderer as pending")
            self.pendingVideoRenderer = renderer
        }
    }
    
    func addAudioRenderer(_ renderer: LKRTCAudioRenderer) {
        if let track = audioTrack {
            track.add(renderer)
            dlog("DEBUG: Audio renderer attached to track - track enabled: \(track.isEnabled)")
        } else {
            // Store renderer to attach when track arrives (cross-network timing)
            dlog("INFO: No audio track yet, storing renderer as pending")
            self.pendingAudioRenderer = renderer
        }
    }
    
    func disconnect() {
        peerConnection?.close()
        peerConnection = nil
        self.stopStatsTimer()
        dlog("WebRTC Client disconnected")
        videoTrack = nil
        audioTrack = nil
        stopHandTrackingStream()
    }
    
    // MARK: - Stats Monitoring
    
    private func stopStatsTimer() {
        statsTimer?.invalidate()
        statsTimer = nil
    }
    
    private func startStatsTimer() {
        stopStatsTimer()
        DispatchQueue.main.async {
            self.statsTimer = Timer.scheduledTimer(withTimeInterval: 5.0, repeats: true) { [weak self] _ in
                self?.monitorConnectionStats()
            }
        }
    }
    
    private func monitorConnectionStats() {
        guard let pc = self.peerConnection,
              pc.connectionState == .connected else {
            return
        }
        
        pc.statistics { report in
            var activePairId: String?
            
            // 1. Find the active transport to get the selected candidate pair ID
            for (id, stats) in report.statistics {
                if stats.type == "transport" {
                    if let selectedId = stats.values["selectedCandidatePairId"] as? String {
                        activePairId = selectedId
                        break
                    }
                }
            }
            
            // Fallback: iterate candidate-pair stats to find the one with 'nominated' and 'state'='succeeded' 
            // if selectedCandidatePairId isn't reliable in some versions
            
            guard let pairId = activePairId,
                  let pairStats = report.statistics[pairId] else {
                return
            }
            
            let localId = pairStats.values["localCandidateId"] as? String
            let remoteId = pairStats.values["remoteCandidateId"] as? String
            
            var connectionTypeString = "Unknown"
            
            if let remoteId = remoteId,
               let remoteCandidate = report.statistics[remoteId] {
                
                // Inspect candidate types
                // types: host, srflx, prflx, relay
                if let type = remoteCandidate.values["candidateType"] as? String {
                    if type == "relay" {
                        connectionTypeString = "TURN (Relay)"
                    } else if type == "srflx" || type == "prflx" {
                        connectionTypeString = "STUN (P2P)"
                    } else if type == "host" {
                        connectionTypeString = "Direct (Host)"
                    }
                }
                
                // If local is relay, it's definitely relay
                if let localId = localId,
                   let localCandidate = report.statistics[localId],
                   let localType = localCandidate.values["candidateType"] as? String {
                    if localType == "relay" {
                        connectionTypeString = "TURN (Relay)"
                    }
                }
            }
            
            // Check current status to append/update
            Task { @MainActor in
                let current = DataManager.shared.connectionStatus
                // Only update if it contains "Connected" to avoid overwriting transient states
                // Or if we specifically want to show this info always once connected
                DataManager.shared.webRTCConnectionType = connectionTypeString
            }
        }
    }
    
}

// MARK: - RTCPeerConnectionDelegate
extension WebRTCClient {
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didChange stateChanged: LKRTCSignalingState) {
        dlog("DEBUG: Signaling state changed to: \(stateChanged)")
        Task { @MainActor in
            switch stateChanged.rawValue {
            case 0: DataManager.shared.connectionStatus = "Signaling: Stable"
            case 1: DataManager.shared.connectionStatus = "Signaling: Have local offer"
            case 2: DataManager.shared.connectionStatus = "Signaling: Have remote offer"
            case 3: DataManager.shared.connectionStatus = "Signaling: Have local answer"
            case 4: DataManager.shared.connectionStatus = "Signaling: Have remote answer"
            default: break
            }
        }
    }
    
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didAdd stream: LKRTCMediaStream) {
        dlog("DEBUG: Stream added - id: \(stream.streamId)")
        dlog("DEBUG: Stream has \(stream.videoTracks.count) video tracks, \(stream.audioTracks.count) audio tracks")
        Task { @MainActor in
            DataManager.shared.connectionStatus = "Video stream received (\(stream.videoTracks.count) tracks)"
        }
        if let videoTrack = stream.videoTracks.first {
            self.videoTrack = videoTrack
            dlog("DEBUG: Video track received - id: \(videoTrack.trackId), enabled: \(videoTrack.isEnabled)")
            
            // Attach pending renderer if one was waiting
            if let pendingRenderer = self.pendingVideoRenderer {
                videoTrack.add(pendingRenderer)
                dlog("DEBUG: Attached pending video renderer to newly received track")
                self.pendingVideoRenderer = nil
            }
            
            Task { @MainActor in
                DataManager.shared.connectionStatus = "Video track enabled, waiting for frames..."
                DataManager.shared.videoEnabled = true
            }
        }
        if let audioTrack = stream.audioTracks.first {
            self.audioTrack = audioTrack
            dlog("DEBUG: Audio track received - id: \(audioTrack.trackId), enabled: \(audioTrack.isEnabled)")
            
            // Attach pending renderer if one was waiting
            if let pendingRenderer = self.pendingAudioRenderer {
                audioTrack.add(pendingRenderer)
                dlog("DEBUG: Attached pending audio renderer to newly received track")
                self.pendingAudioRenderer = nil
            }
            
            Task { @MainActor in
                DataManager.shared.connectionStatus = "Audio track enabled"
                DataManager.shared.audioEnabled = true
            }
        }
    }
    
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didRemove stream: LKRTCMediaStream) {
        dlog("DEBUG: Stream removed")
    }
    
    func peerConnectionShouldNegotiate(_ peerConnection: LKRTCPeerConnection) {
        dlog("DEBUG: Should negotiate")
    }
    
    
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didChange newState: LKRTCIceConnectionState) {
        dlog("DEBUG: ICE connection state changed to: \(newState.rawValue) (\(iceStateString(newState)))")
        Task { @MainActor in        
            if newState == .connected {
                 dlog("✅ [WebRTC] PeerConnection connected!")
                 self.onConnectionStateChanged?(true)
                 self.startStatsTimer()
                 Task { @MainActor in
                     DataManager.shared.connectionStatus = "Connected (Negotiating...)"
                 }
            } else if newState == .failed || newState == .disconnected || newState == .closed {
                let status = newState == .failed ? "failed" : (newState == .disconnected ? "disconnected" : "closed")
                dlog("⚠️ [WebRTC] ICE connection \(status)")
                DataManager.shared.connectionStatus = "ICE \(status)"
                self.onConnectionStateChanged?(false)
                
                // Reset flags on disconnect
                if newState == .closed || newState == .disconnected {
                     DataManager.shared.videoEnabled = false
                     DataManager.shared.audioEnabled = false
                     DataManager.shared.simEnabled = false
                }
            } else if newState == .checking {
                DataManager.shared.connectionStatus = "Checking ICE connection..."
            }
        }
    }
    
    private func iceStateString(_ state: LKRTCIceConnectionState) -> String {
        switch state.rawValue {
        case 0: return "new"
        case 1: return "checking"
        case 2: return "connected"
        case 3: return "completed"
        case 4: return "failed"
        case 5: return "disconnected"
        case 6: return "closed"
        default: return "unknown"
        }
    }
    
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didChange newState: LKRTCIceGatheringState) {
        dlog("DEBUG: ICE gathering state changed to: \(newState) (rawValue: \(newState.rawValue))")
        
        // Resume continuation when gathering is complete
        if newState == .complete {
            if !iceGatheringResumed, let continuation = iceGatheringContinuation {
                iceGatheringResumed = true
                iceGatheringContinuation = nil
                dlog("DEBUG: ICE gathering completed via delegate callback")
                Task { @MainActor in
                    DataManager.shared.connectionStatus = "ICE gathering complete"
                }
                continuation.resume()
            }
        }
    }
    
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didGenerate candidate: LKRTCIceCandidate) {
        dlog("DEBUG: Generated ICE candidate: \(candidate.sdp) [\(candidate.sdpMid ?? "no-mid")] type: \(candidate.sdp.contains("host") ? "host" : candidate.sdp.contains("srflx") ? "srflx" : "relay")")
        // Check if we are using SignalingClient (by checking connection status maybe? or just try both)
        // If connected via signaling, send candidate
        Task { @MainActor in
            // This is a bit of a hack to access the signaling client globally or passed in. 
            // Ideally should check active mode.
            // For now, if we are in Remote mode, send via signaling
            // But WebRTCClient doesn't know about AppModel/ContentView state directly.
            // However, we can check if data manager has a room code? 
            // Or better: The caller of connectWithSignaling should have set a property or we pass it
            
            // NOTE: For now, I'm modifying this to just log. 
            // The actual sending needs to happen if we have a callback or reference.
            // Let's add a callback for ICE candidates generated locally
        }
        
        self.onLocalICECandidateGenerated?(candidate)
    }
    
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didRemove candidates: [LKRTCIceCandidate]) {
        dlog("DEBUG: ICE candidates removed")
    }
    
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didOpen dataChannel: LKRTCDataChannel) {
        dlog("🔔 [WebRTC] Data channel opened (label=\(dataChannel.label), state=\(dataChannel.readyState.rawValue))")
        
        if dataChannel.label == "hand-tracking" {
            handDataChannel = dataChannel
            dataChannel.delegate = self
            startHandTrackingStream(on: dataChannel)
            Task { @MainActor in
                DataManager.shared.connectionStatus = "Hand data channel open"
            }
        } else if dataChannel.label == "sim-poses" {
            simPosesDataChannel = dataChannel
            dataChannel.delegate = self
            let hasCallback = onSimPosesReceived != nil
            dlog("🔔 [WebRTC] Sim-poses data channel connected! hasCallback=\(hasCallback)")
            Task { @MainActor in
                DataManager.shared.connectionStatus = "Sim-poses data channel open"
                DataManager.shared.simEnabled = true
            }
        } else if dataChannel.label == "usdz-transfer" {
            usdzDataChannel = dataChannel
            dataChannel.delegate = self
            // Reset transfer state for new transfer
            usdzTransferMetadata = nil
            usdzChunks = [:]
            usdzReceivedChunksCount = 0
            dlog("📦 [WebRTC] USDZ transfer channel connected!")
            Task { @MainActor in
                DataManager.shared.connectionStatus = "USDZ transfer channel open"
            }
        } else {
            dlog("DEBUG: Unknown data channel: \(dataChannel.label)")
        }
    }
}

extension WebRTCClient: LKRTCDataChannelDelegate {
    func dataChannelDidChangeState(_ dataChannel: LKRTCDataChannel) {
        let stateNames = ["connecting", "open", "closing", "closed"]
        let stateName = dataChannel.readyState.rawValue < stateNames.count ? stateNames[Int(dataChannel.readyState.rawValue)] : "unknown"
        dlog("🔄 [WebRTC] Data channel '\(dataChannel.label)' state → \(stateName) (\(dataChannel.readyState.rawValue))")
        
        if dataChannel.readyState == .open {
            if dataChannel.label == "hand-tracking" {
                dlog("🟢 [WebRTC] Hand tracking channel OPEN - Starting stream")
                Task { @MainActor in
                    DataManager.shared.connectionStatus = "Hand data channel open"
                }
                // Ensure we start streaming if this was a locally created channel
                // (didOpen is not called for local channels)
                self.handDataChannel = dataChannel
                startHandTrackingStream(on: dataChannel)
            } else if dataChannel.label == "sim-poses" {
                 dlog("🟢 [WebRTC] Sim poses channel OPEN")
                 self.simPosesDataChannel = dataChannel
                 Task { @MainActor in
                     DataManager.shared.connectionStatus = "Sim-poses data channel open"
                 }
            }
        } else if dataChannel.readyState == .closed || dataChannel.readyState == .closing {
            if dataChannel == handDataChannel {
                stopHandTrackingStream()
                Task { @MainActor in
                    DataManager.shared.connectionStatus = "Hand data channel closed"
                }
            } else if dataChannel == simPosesDataChannel {
                dlog("⚠️ [WebRTC] sim-poses channel is closing/closed! Total messages received: \(simPosesMessageCount)")
                simPosesDataChannel = nil
                Task { @MainActor in
                    DataManager.shared.connectionStatus = "Sim-poses data channel closed"
                }
            }
        }
    }

    func dataChannel(_ dataChannel: LKRTCDataChannel, didReceiveMessageWith buffer: LKRTCDataBuffer) {
        if dataChannel.label == "sim-poses" {
            simPosesMessageCount += 1
            
            // Log only first message and every 100th for debugging
            if simPosesMessageCount == 1 || simPosesMessageCount % 100 == 0 {
                dlog("📥 [WebRTC sim-poses] Message #\(simPosesMessageCount), bytes=\(buffer.data.count)")
            }
            
            // Parse binary pose data
            // Format:
            //   [timestamp: 8 bytes double]
            //   [body_count: 2 bytes uint16]
            //   For each body:
            //     [name_len: 1 byte uint8]
            //     [name: N bytes UTF-8]
            //     [x, y, z, qx, qy, qz, qw: 7 × 4 bytes float32]
            //   [qpos_count: 2 bytes uint16]
            //   [qpos values: N × 4 bytes float32]
            //   [ctrl_count: 2 bytes uint16]
            //   [ctrl values: N × 4 bytes float32]
            
            let data = buffer.data
            var offset = 0
            
            // Need at least timestamp (8) + body_count (2) = 10 bytes
            guard data.count >= 10 else {
                dlog("⚠️ [WebRTC sim-poses] Message too small: \(data.count) bytes")
                return
            }
            
            // Read timestamp (8 bytes, little-endian double)
            let timestamp: Double = data.withUnsafeBytes { ptr in
                ptr.loadUnaligned(fromByteOffset: offset, as: Double.self)
            }
            offset += 8
            
            // Read body count (2 bytes, little-endian uint16)
            let bodyCount: UInt16 = data.withUnsafeBytes { ptr in
                ptr.loadUnaligned(fromByteOffset: offset, as: UInt16.self)
            }
            offset += 2
            
            var floatPoses: [String: [Float]] = [:]
            
            // Read each body
            for _ in 0..<bodyCount {
                // Check bounds
                guard offset + 1 <= data.count else { break }
                
                // Read name length (1 byte)
                let nameLen = Int(data[offset])
                offset += 1
                
                guard offset + nameLen + 28 <= data.count else { break }
                
                // Read name (N bytes UTF-8)
                let nameData = data.subdata(in: offset..<(offset + nameLen))
                let bodyName = String(data: nameData, encoding: .utf8) ?? ""
                offset += nameLen
                
                // Read 7 floats (28 bytes)
                var values: [Float] = []
                for _ in 0..<7 {
                    let value: Float = data.withUnsafeBytes { ptr in
                        ptr.loadUnaligned(fromByteOffset: offset, as: Float.self)
                    }
                    values.append(value)
                    offset += 4
                }
                
                floatPoses[bodyName] = values
            }
            
            // Read qpos
            var qpos: [Float]? = nil
            if offset + 2 <= data.count {
                let qposCount: UInt16 = data.withUnsafeBytes { ptr in
                    ptr.loadUnaligned(fromByteOffset: offset, as: UInt16.self)
                }
                offset += 2
                
                if qposCount > 0 && offset + Int(qposCount) * 4 <= data.count {
                    var qposValues: [Float] = []
                    for _ in 0..<qposCount {
                        let value: Float = data.withUnsafeBytes { ptr in
                            ptr.loadUnaligned(fromByteOffset: offset, as: Float.self)
                        }
                        qposValues.append(value)
                        offset += 4
                    }
                    qpos = qposValues
                }
            }
            
            // Read ctrl
            var ctrl: [Float]? = nil
            if offset + 2 <= data.count {
                let ctrlCount: UInt16 = data.withUnsafeBytes { ptr in
                    ptr.loadUnaligned(fromByteOffset: offset, as: UInt16.self)
                }
                offset += 2
                
                if ctrlCount > 0 && offset + Int(ctrlCount) * 4 <= data.count {
                    var ctrlValues: [Float] = []
                    for _ in 0..<ctrlCount {
                        let value: Float = data.withUnsafeBytes { ptr in
                            ptr.loadUnaligned(fromByteOffset: offset, as: Float.self)
                        }
                        ctrlValues.append(value)
                        offset += 4
                    }
                    ctrl = ctrlValues
                }
            }
            
            // Call callback on main thread
            if let callback = onSimPosesReceived {
                // Debug: log timestamp, body count, and first two body positions every 100th message
                if simPosesMessageCount % 100 == 0 {
                    let sortedBodies = floatPoses.keys.sorted()
                    if sortedBodies.count >= 2 {
                        let body0 = sortedBodies[0]
                        let body1 = sortedBodies[1]
                        let pos0 = floatPoses[body0]!
                        let pos1 = floatPoses[body1]!
                        dlog("🔍 [DEBUG] msg#\(simPosesMessageCount) ts=\(String(format: "%.3f", timestamp)) bodies=\(floatPoses.count)")
                        dlog("   body0='\(body0)' pos=[\(String(format: "%.3f", pos0[0])),\(String(format: "%.3f", pos0[1])),\(String(format: "%.3f", pos0[2]))]")
                        dlog("   body1='\(body1)' pos=[\(String(format: "%.3f", pos1[0])),\(String(format: "%.3f", pos1[1])),\(String(format: "%.3f", pos1[2]))]")
                    }
                }
                
                DispatchQueue.main.async {
                    callback(timestamp, floatPoses, qpos, ctrl)
                }
            } else {
                dlog("⚠️ [WebRTC sim-poses] Callback not set, dropping \(floatPoses.count) poses")
            }
        } else if dataChannel.label == "usdz-transfer" {
            handleUsdzMessage(buffer)
        }
        // Other data channels silently ignored
    }
    
    /// Handle incoming USDZ transfer messages
    private func handleUsdzMessage(_ buffer: LKRTCDataBuffer) {
        let data = buffer.data
        
        // Check if this is JSON metadata (text) or binary chunk data
        if !buffer.isBinary, let jsonString = String(data: data, encoding: .utf8) {
            // Parse metadata JSON
            guard let jsonData = jsonString.data(using: .utf8),
                  let json = try? JSONSerialization.jsonObject(with: jsonData) as? [String: Any],
                  let type = json["type"] as? String else {
                dlog("⚠️ [USDZ] Failed to parse metadata JSON")
                return
            }
            
            if type == "metadata" {
                guard let filename = json["filename"] as? String,
                      let totalSize = json["totalSize"] as? Int,
                      let totalChunks = json["totalChunks"] as? Int else {
                    dlog("⚠️ [USDZ] Invalid metadata format")
                    return
                }
                
                let attachPosition = json["attachPosition"] as? [Double]
                let attachRotation = json["attachRotation"] as? [Double]
                
                usdzTransferMetadata = UsdzTransferMetadata(
                    filename: filename,
                    totalSize: totalSize,
                    totalChunks: totalChunks,
                    attachPosition: attachPosition?.map { Float($0) },
                    attachRotation: attachRotation?.map { Float($0) }
                )
                usdzChunks = [:]
                usdzReceivedChunksCount = 0
                
                dlog("📦 [USDZ] Receiving: \(filename) (\(totalSize / 1024)KB, \(totalChunks) chunks)")
                Task { @MainActor in
                    DataManager.shared.connectionStatus = "Receiving USDZ: \(filename)..."
                }
            }
        } else {
            // Binary chunk data: [4 bytes index][N bytes data]
            guard data.count >= 4 else {
                dlog("⚠️ [USDZ] Chunk too small")
                return
            }
            
            // Extract chunk index (big-endian UInt32)
            let chunkIndex: UInt32 = data.withUnsafeBytes { ptr in
                ptr.load(as: UInt32.self).bigEndian
            }
            let chunkData = data.subdata(in: 4..<data.count)
            
            usdzChunks[Int(chunkIndex)] = chunkData
            usdzReceivedChunksCount += 1
            
            // Log progress every 50 chunks
            if usdzReceivedChunksCount % 50 == 0, let meta = usdzTransferMetadata {
                let progress = Float(usdzReceivedChunksCount) / Float(meta.totalChunks) * 100
                dlog("📥 [USDZ] Progress: \(usdzReceivedChunksCount)/\(meta.totalChunks) (\(String(format: "%.1f", progress))%)")
            }
            
            // Check if transfer is complete
            if let meta = usdzTransferMetadata, usdzReceivedChunksCount == meta.totalChunks {
                dlog("📦 [USDZ] All chunks received, assembling file...")
                assembleAndLoadUsdz()
            }
        }
    }
    
    /// Assemble USDZ chunks and load the scene
    private func assembleAndLoadUsdz() {
        guard let meta = usdzTransferMetadata else {
            dlog("⚠️ [USDZ] No metadata for assembly")
            sendUsdzError("No metadata")
            return
        }
        
        // Assemble chunks in order
        var assembledData = Data()
        for i in 0..<meta.totalChunks {
            guard let chunk = usdzChunks[i] else {
                dlog("❌ [USDZ] Missing chunk \(i)")
                sendUsdzError("Missing chunk \(i)")
                return
            }
            assembledData.append(chunk)
        }
        
        // Verify size
        guard assembledData.count == meta.totalSize else {
            dlog("❌ [USDZ] Size mismatch: expected \(meta.totalSize), got \(assembledData.count)")
            sendUsdzError("Size mismatch")
            return
        }
        
        // Save to Documents directory
        let documentsDir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!
        let usdzURL = documentsDir.appendingPathComponent(meta.filename)
        
        do {
            try assembledData.write(to: usdzURL)
            dlog("✅ [USDZ] Saved to: \(usdzURL.path)")
            
            // Clear transfer state
            usdzChunks = [:]
            usdzReceivedChunksCount = 0
            
            // Send acknowledgment
            sendUsdzComplete(path: usdzURL.path)
            
            // Load the scene (notify the app)
            Task { @MainActor in
                DataManager.shared.connectionStatus = "USDZ loaded: \(meta.filename)"
                // Store the USDZ path so CombinedStreamingView can load it
                DataManager.shared.loadedUsdzPath = usdzURL.path
                DataManager.shared.loadedUsdzAttachPosition = meta.attachPosition
                DataManager.shared.loadedUsdzAttachRotation = meta.attachRotation
            }
            
        } catch {
            dlog("❌ [USDZ] Failed to save: \(error)")
            sendUsdzError("Save failed: \(error.localizedDescription)")
        }
    }
    
    /// Send completion acknowledgment to Python
    private func sendUsdzComplete(path: String) {
        guard let channel = usdzDataChannel, channel.readyState == .open else {
            dlog("⚠️ [USDZ] Cannot send ack - channel not open")
            return
        }
        
        let response: [String: Any] = [
            "type": "complete",
            "path": path
        ]
        
        if let jsonData = try? JSONSerialization.data(withJSONObject: response),
           let jsonString = String(data: jsonData, encoding: .utf8) {
            let buffer = LKRTCDataBuffer(data: jsonString.data(using: .utf8)!, isBinary: false)
            channel.sendData(buffer)
            dlog("📤 [USDZ] Sent completion ack")
        }
    }
    
    /// Send error message to Python
    private func sendUsdzError(_ message: String) {
        guard let channel = usdzDataChannel, channel.readyState == .open else { return }
        
        let response: [String: Any] = [
            "type": "error",
            "message": message
        ]
        
        if let jsonData = try? JSONSerialization.data(withJSONObject: response),
           let jsonString = String(data: jsonData, encoding: .utf8) {
            let buffer = LKRTCDataBuffer(data: jsonString.data(using: .utf8)!, isBinary: false)
            channel.sendData(buffer)
        }
    }
}

// MARK: - Helper Types
struct SDPMessage: Codable {
    let sdp: String
    let type: String
}

enum WebRTCError: Error {
    case failedToCreatePeerConnection
    case failedToCreateOffer
    case failedToCreateAnswer
    case invalidOffer
    case noLocalDescription
    case peerConnectionClosed
    case connectionFailed
}

extension LKRTCSdpType {
    var stringValue: String {
        switch self {
        case .offer: return "offer"
        case .prAnswer: return "pranswer"
        case .answer: return "answer"
        case .rollback: return "rollback"
        @unknown default: return "unknown"
        }
    }
}

// MARK: - Async Socket Helper
actor AsyncSocketConnection {
    @MainActor
    static func connect(host: String, port: Int) async throws -> (InputStream, OutputStream) {
        var inputStream: InputStream?
        var outputStream: OutputStream?
        
        Stream.getStreamsToHost(
            withName: host,
            port: port,
            inputStream: &inputStream,
            outputStream: &outputStream
        )
        
        guard let input = inputStream, let output = outputStream else {
            throw WebRTCError.connectionFailed
        }
        
        input.schedule(in: .main, forMode: .default)
        output.schedule(in: .main, forMode: .default)
        
        input.open()
        output.open()
        
        // Wait for connection
        try await Task.sleep(nanoseconds: 100_000_000) // 100ms
        
        return (input, output)
    }
}

extension InputStream {
    func readLine() async throws -> Data? {
        var buffer = Data()
        let chunkSize = 1024
        var chunk = [UInt8](repeating: 0, count: chunkSize)
        
        while true {
            guard self.hasBytesAvailable else {
                try await Task.sleep(nanoseconds: 10_000_000) // 10ms
                continue
            }
            
            let bytesRead = self.read(&chunk, maxLength: 1)
            if bytesRead <= 0 {
                return buffer.isEmpty ? nil : buffer
            }
            
            if chunk[0] == 0x0A { // newline
                return buffer
            }
            
            buffer.append(chunk[0])
        }
    }
}

extension OutputStream {
    func write(_ data: Data) async throws {
        let bytes = [UInt8](data)
        var totalWritten = 0
        
        while totalWritten < bytes.count {
            guard self.hasSpaceAvailable else {
                try await Task.sleep(nanoseconds: 10_000_000) // 10ms
                continue
            }
            
            let written = bytes.withUnsafeBufferPointer { bufferPointer in
                self.write(bufferPointer.baseAddress! + totalWritten, maxLength: bytes.count - totalWritten)
            }
            
            if written < 0 {
                throw WebRTCError.connectionFailed
            }
            totalWritten += written
        }
    }
}

extension LKRTCDataChannel: @unchecked Sendable {}
