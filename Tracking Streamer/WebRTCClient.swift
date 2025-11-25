import Foundation
import LiveKitWebRTC
import SwiftProtobuf
import UIKit

/// WebRTC client that connects to the Python server and receives video frames
class WebRTCClient: NSObject, LKRTCPeerConnectionDelegate, @unchecked Sendable {
    private var peerConnection: LKRTCPeerConnection?
    private let factory: LKRTCPeerConnectionFactory
    private var videoTrack: LKRTCVideoTrack?
    private var audioTrack: LKRTCAudioTrack?
    private var handDataChannel: LKRTCDataChannel?
    private var handStreamTask: Task<Void, Never>?
    private let handStreamIntervalNanoseconds: UInt64 = 2_000_000
    
    var onFrameReceived: ((CVPixelBuffer) -> Void)?
    
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
    
    /// Connect to the WebRTC server at the given address
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
    
    private func connectToServer(host: String, port: Int) async throws {
        print("DEBUG: Attempting to connect to \(host):\(port)")
        await MainActor.run {
            DataManager.shared.connectionStatus = "Connecting to \(host):\(port)..."
        }
        let (inputStream, outputStream) = try await AsyncSocketConnection.connect(
            host: host,
            port: port
        )
        print("DEBUG: Socket connection established to \(host):\(port)")
        await MainActor.run {
            DataManager.shared.connectionStatus = "Socket connected to \(host):\(port)"
        }
        
        // Read offer from server
        guard let offerData = try await inputStream.readLine(),
              let offerJson = try? JSONDecoder().decode(SDPMessage.self, from: offerData) else {
            throw WebRTCError.invalidOffer
        }
        
        print("DEBUG: Received offer from server")
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
        print("DEBUG: Waiting for ICE gathering to complete")
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
        
        print("DEBUG: Answer sent to server")
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
                        print("ERROR: Failed to send hand tracking update over WebRTC (channel not open or backpressure)")
                    }
                } catch {
                    print("ERROR: Unable to serialize hand tracking update: \(error)")
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
                    print("DEBUG: ICE gathering timed out after 500ms (proceeding anyway)")
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
                print("DEBUG: ICE gathering already complete")
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
            print("DEBUG: Video renderer attached to track - track enabled: \(track.isEnabled)")
        } else {
            print("ERROR: Cannot attach renderer - no video track available")
        }
    }
    
    func addAudioRenderer(_ renderer: LKRTCAudioRenderer) {
        if let track = audioTrack {
            track.add(renderer)
            print("DEBUG: Audio renderer attached to track - track enabled: \(track.isEnabled)")
        } else {
            print("INFO: No audio track available (audio may not be enabled on server)")
        }
    }
    
    func disconnect() {
        peerConnection?.close()
        peerConnection = nil
        videoTrack = nil
        audioTrack = nil
        stopHandTrackingStream()
    }
}

// MARK: - RTCPeerConnectionDelegate
extension WebRTCClient {
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didChange stateChanged: LKRTCSignalingState) {
        print("DEBUG: Signaling state changed to: \(stateChanged)")
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
        print("DEBUG: Stream added - id: \(stream.streamId)")
        print("DEBUG: Stream has \(stream.videoTracks.count) video tracks, \(stream.audioTracks.count) audio tracks")
        Task { @MainActor in
            DataManager.shared.connectionStatus = "Video stream received (\(stream.videoTracks.count) tracks)"
        }
        if let videoTrack = stream.videoTracks.first {
            self.videoTrack = videoTrack
            print("DEBUG: Video track received - id: \(videoTrack.trackId), enabled: \(videoTrack.isEnabled)")
            Task { @MainActor in
                DataManager.shared.connectionStatus = "Video track enabled, waiting for frames..."
            }
        }
        if let audioTrack = stream.audioTracks.first {
            self.audioTrack = audioTrack
            print("DEBUG: Audio track received - id: \(audioTrack.trackId), enabled: \(audioTrack.isEnabled)")
            Task { @MainActor in
                DataManager.shared.connectionStatus = "Audio track enabled"
            }
        }
    }
    
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didRemove stream: LKRTCMediaStream) {
        print("DEBUG: Stream removed")
    }
    
    func peerConnectionShouldNegotiate(_ peerConnection: LKRTCPeerConnection) {
        print("DEBUG: Should negotiate")
    }
    
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didChange newState: LKRTCIceConnectionState) {
        print("DEBUG: ICE connection state changed to: \(newState.rawValue) (\(iceStateString(newState)))")
        Task { @MainActor in
            if newState == .connected {
                print("DEBUG: *** ICE CONNECTION SUCCESSFUL ***")
                DataManager.shared.connectionStatus = "ICE connected successfully!"
            } else if newState == .failed {
                print("ERROR: ICE connection failed")
                DataManager.shared.connectionStatus = "ICE connection failed"
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
        print("DEBUG: ICE gathering state changed to: \(newState) (rawValue: \(newState.rawValue))")
        
        // Resume continuation when gathering is complete
        if newState == .complete {
            if !iceGatheringResumed, let continuation = iceGatheringContinuation {
                iceGatheringResumed = true
                iceGatheringContinuation = nil
                print("DEBUG: ICE gathering completed via delegate callback")
                Task { @MainActor in
                    DataManager.shared.connectionStatus = "ICE gathering complete"
                }
                continuation.resume()
            }
        }
    }
    
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didGenerate candidate: LKRTCIceCandidate) {
        print("DEBUG: ICE candidate generated - \(candidate.sdp) [\(candidate.sdpMid ?? "no-mid")] type: \(candidate.sdp.contains("host") ? "host" : candidate.sdp.contains("srflx") ? "srflx" : "relay")")
    }
    
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didRemove candidates: [LKRTCIceCandidate]) {
        print("DEBUG: ICE candidates removed")
    }
    
    func peerConnection(_ peerConnection: LKRTCPeerConnection, didOpen dataChannel: LKRTCDataChannel) {
        print("DEBUG: Data channel opened (label=\(dataChannel.label), state=\(dataChannel.readyState.rawValue))")
        handDataChannel = dataChannel
        dataChannel.delegate = self
        startHandTrackingStream(on: dataChannel)
        Task { @MainActor in
            DataManager.shared.connectionStatus = "Hand data channel open"
        }
    }
}

extension WebRTCClient: LKRTCDataChannelDelegate {
    func dataChannelDidChangeState(_ dataChannel: LKRTCDataChannel) {
        print("DEBUG: Data channel state changed to: \(dataChannel.readyState.rawValue)")
        if dataChannel.readyState == .closed || dataChannel.readyState == .closing {
            if dataChannel == handDataChannel {
                stopHandTrackingStream()
                Task { @MainActor in
                    DataManager.shared.connectionStatus = "Hand data channel closed"
                }
            }
        }
    }

    func dataChannel(_ dataChannel: LKRTCDataChannel, didReceiveMessageWith buffer: LKRTCDataBuffer) {
        print("DEBUG: Received \(buffer.data.count) bytes on data channel (ignored)")
    }
}

// MARK: - Helper Types
struct SDPMessage: Codable {
    let sdp: String
    let type: String
}

enum WebRTCError: Error {
    case failedToCreatePeerConnection
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
