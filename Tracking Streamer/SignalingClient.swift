import Foundation
import LiveKitWebRTC

/// Signaling client for cross-network WebRTC connections
/// Connects to a Cloudflare Workers signaling server to exchange SDP and ICE candidates
@MainActor
class SignalingClient: ObservableObject {
    
    static let shared = SignalingClient()
    
    // MARK: - Published Properties
    
    @Published var isConnected: Bool = false
    @Published var roomCode: String = ""
    @Published var peerConnected: Bool = false
    @Published var connectionStatus: String = "Disconnected"
    @Published var isRoomCodeLocked: Bool = false  // Whether room code persists across sessions
    
    // MARK: - UserDefaults Keys
    
    private let roomCodeKey = "persistentRoomCode"
    private let roomCodeLockedKey = "roomCodeLocked"
    
    // MARK: - Private Properties
    
    private var webSocketTask: URLSessionWebSocketTask?
    private var session: URLSession?
    private var reconnectTask: Task<Void, Never>?
    
    private let signalingURL = "wss://visionpro-signaling.parkyh9492.workers.dev/ws"
    
    // MARK: - Callbacks
    
    /// Called when an SDP offer is received from Python
    var onSDPOfferReceived: ((String) -> Void)? {
        didSet {
            // If we have a pending offer buffered before callback was registered, deliver it now
            if let pendingOffer = pendingSDPOffer, onSDPOfferReceived != nil {
                print("[SIGNALING] Delivering buffered SDP offer")
                onSDPOfferReceived?(pendingOffer)
                pendingSDPOffer = nil
            }
        }
    }
    
    /// Called when an SDP answer is received from Python
    var onSDPAnswerReceived: ((String) -> Void)?
    
    /// Called when an ICE candidate is received from Python
    var onICECandidateReceived: (([String: Any]) -> Void)? {
        didSet {
            // Deliver any pending ICE candidates
            if onICECandidateReceived != nil && !pendingICECandidates.isEmpty {
                print("[SIGNALING] Delivering \(pendingICECandidates.count) buffered ICE candidates")
                for candidate in pendingICECandidates {
                    onICECandidateReceived?(candidate)
                }
                pendingICECandidates.removeAll()
            }
        }
    }
    
    /// Called when Python peer joins the room
    var onPeerJoined: (() -> Void)?
    
    /// Called when Python peer requests an offer explicitly
    var onOfferRequested: (() -> Void)?
    
    /// Called when ICE servers are received from signaling
    var onIceServersReceived: (([LKRTCIceServer]) -> Void)?
    
    /// Called when Python peer leaves the room
    var onPeerLeft: (() -> Void)?
    
    // MARK: - Pending Message Buffers (for race condition handling)
    
    /// Buffered SDP offer received before callback was registered
    private var pendingSDPOffer: String?
    
    /// Buffered ICE candidates received before callback was registered
    private var pendingICECandidates: [[String: Any]] = []
    
    /// Clear all pending messages (call when peer disconnects to avoid stale data)
    func clearPendingMessages() {
        print("[SIGNALING] Clearing pending messages")
        pendingSDPOffer = nil
        pendingICECandidates.removeAll()
    }
    
    /// Clear WebRTC-related callbacks (call when session ends so new session can re-register)
    /// This ensures that when a new WebRTCClient is created, it can register fresh callbacks
    /// and receive any buffered offers.
    /// NOTE: We keep onSDPOfferReceived and onICECandidateReceived to support reconnection
    /// when the same WebRTCClient handles multiple peer connections.
    func clearWebRTCCallbacks() {
        print("[SIGNALING] Clearing WebRTC callbacks for session reset (keeping offer/ice callbacks for reconnection)")
        // Keep onSDPOfferReceived - needed for handling reconnection
        // Keep onICECandidateReceived - needed for handling reconnection  
        onSDPAnswerReceived = nil
        onPeerJoined = nil
        onOfferRequested = nil
        onIceServersReceived = nil
        // Note: onPeerLeft is kept so VideoStreamManager can still handle future disconnects
    }
    
    // MARK: - Lifecycle
    
    init() {
        loadPersistedRoomCode()
    }
    
    deinit {
        // Can't call MainActor methods from deinit, so just cancel tasks directly
        reconnectTask?.cancel()
        webSocketTask?.cancel(with: .goingAway, reason: nil)
    }
    
    // MARK: - Public Methods
    
    /// Load persisted room code from UserDefaults, or generate a new one
    private func loadPersistedRoomCode() {
        isRoomCodeLocked = UserDefaults.standard.bool(forKey: roomCodeLockedKey)
        
        if isRoomCodeLocked, let savedCode = UserDefaults.standard.string(forKey: roomCodeKey), !savedCode.isEmpty {
            roomCode = savedCode
            print("[SIGNALING] Loaded persisted room code: \(roomCode)")
        } else {
            generateRoomCode()
        }
    }
    
    /// Generate a new room code (format: ABCD-1234)
    func generateRoomCode() {
        let letters = "ABCDEFGHJKLMNPQRSTUVWXYZ"  // Excludes I and O to avoid confusion
        let numbers = "0123456789"
        
        let letterPart = String((0..<4).map { _ in letters.randomElement()! })
        let numberPart = String((0..<4).map { _ in numbers.randomElement()! })
        
        roomCode = "\(letterPart)-\(numberPart)"
        print("[SIGNALING] Generated room code: \(roomCode)")
        
        // If locked, save the new code too
        if isRoomCodeLocked {
            UserDefaults.standard.set(roomCode, forKey: roomCodeKey)
        }
    }
    
    /// Lock/unlock the current room code for persistence across sessions
    func setRoomCodeLocked(_ locked: Bool) {
        isRoomCodeLocked = locked
        UserDefaults.standard.set(locked, forKey: roomCodeLockedKey)
        
        if locked {
            // Save current code
            UserDefaults.standard.set(roomCode, forKey: roomCodeKey)
            print("[SIGNALING] Room code locked: \(roomCode)")
        } else {
            // Clear saved code (will generate new on next launch)
            UserDefaults.standard.removeObject(forKey: roomCodeKey)
            print("[SIGNALING] Room code unlocked (will regenerate on next launch)")
        }
    }
    
    /// Connect to the signaling server and join the room
    func connect() {
        guard !roomCode.isEmpty else {
            print("[SIGNALING] Error: No room code")
            return
        }
        
        guard let url = URL(string: signalingURL) else {
            print("[SIGNALING] Error: Invalid signaling URL")
            return
        }
        
        connectionStatus = "Connecting..."
        print("[SIGNALING] Connecting to \(signalingURL)")
        
        session = URLSession(configuration: .default)
        webSocketTask = session?.webSocketTask(with: url)
        webSocketTask?.resume()
        
        // Start receiving messages
        receiveMessage()
        
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { [weak self] in
            self?.joinRoom()
        }
    }
    
    /// Disconnect from the signaling server
    func disconnect() {
        reconnectTask?.cancel()
        reconnectTask = nil
        
        webSocketTask?.cancel(with: .goingAway, reason: nil)
        webSocketTask = nil
        session = nil
        
        isConnected = false
        peerConnected = false
        connectionStatus = "Disconnected"
    }
    
    /// Send SDP offer to the peer via signaling server
    func sendSDPOffer(_ sdp: String) {
        let message: [String: Any] = [
            "type": "sdp",
            "sdp": sdp,
            "sdpType": "offer"
        ]
        sendMessage(message)
        print("[SIGNALING] Sent SDP offer")
    }
    
    /// Send SDP answer to the peer via signaling server
    func sendSDPAnswer(_ sdp: String) {
        print("[SIGNALING] 📤 Sending SDP answer to Python (length: \(sdp.count) chars)...")
        let message: [String: Any] = [
            "type": "sdp",
            "sdp": sdp,
            "sdpType": "answer"
        ]
        sendMessage(message)
        print("[SIGNALING] ✅ SDP answer sent successfully")
    }
    
    /// Send ICE candidate to the peer via signaling server
    func sendICECandidate(candidate: String, sdpMid: String?, sdpMLineIndex: Int32) {
        var candidateDict: [String: Any] = [
            "candidate": candidate,
            "sdpMLineIndex": sdpMLineIndex
        ]
        if let sdpMid = sdpMid {
            candidateDict["sdpMid"] = sdpMid
        }
        
        let message: [String: Any] = [
            "type": "ice",
            "candidate": candidateDict
        ]
        sendMessage(message)
        print("[SIGNALING] Sent ICE candidate")
    }
    
    /// Request ICE servers (TURN) from the signaling server
    func requestIceServers() {
        let message: [String: Any] = [
            "type": "request-ice"
        ]
        sendMessage(message)
        print("[SIGNALING] Requested ICE servers from signaling")
    }
    
    // MARK: - Private Methods
    
    private func joinRoom() {
        let message: [String: Any] = [
            "type": "join",
            "room": roomCode,
            "role": "visionos"
        ]
        sendMessage(message)
        print("[SIGNALING] Joining room: \(roomCode)")
    }
    
    private func sendMessage(_ dict: [String: Any]) {
        guard let data = try? JSONSerialization.data(withJSONObject: dict),
              let jsonString = String(data: data, encoding: .utf8) else {
            print("[SIGNALING] Error: Failed to serialize message")
            return
        }
        
        let message = URLSessionWebSocketTask.Message.string(jsonString)
        webSocketTask?.send(message) { error in
            if let error = error {
                print("[SIGNALING] Send error: \(error.localizedDescription)")
            }
        }
    }
    
    private func receiveMessage() {
        webSocketTask?.receive { [weak self] result in
            guard let self = self else { return }
            
            switch result {
            case .success(let message):
                switch message {
                case .string(let text):
                    self.handleMessage(text)
                case .data(let data):
                    if let text = String(data: data, encoding: .utf8) {
                        self.handleMessage(text)
                    }
                @unknown default:
                    break
                }
                
                // Continue receiving
                self.receiveMessage()
                
            case .failure(let error):
                print("[SIGNALING] ❌ Receive error: \(error.localizedDescription)")
                Task { @MainActor in
                    self.isConnected = false
                    self.connectionStatus = "Connection lost"
                    self.scheduleReconnect()
                }
            }
        }
    }
    
    private func handleMessage(_ text: String) {
        // Log ALL incoming messages to debug reconnection issues
        let preview = text.prefix(80)
        print("[SIGNALING] 📩 Message received: \(preview)...")
        
        guard let data = text.data(using: .utf8),
              let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              let type = json["type"] as? String else {
            print("[SIGNALING] ❌ Invalid message: \(text)")
            return
        }
        
        print("[SIGNALING] 📩 Parsed message type: \(type)")
        
        Task { @MainActor in
            switch type {
            case "joined":
                let peersInRoom = json["peersInRoom"] as? Int ?? 0
                print("[SIGNALING] Joined room with \(peersInRoom) peer(s)")
                isConnected = true
                connectionStatus = "Room: \(roomCode)"
                
                // If there are already peers in the room (size > 1 means me + others), mark as connected
                if peersInRoom > 1 {
                    print("[SIGNALING] Found existing peer in room")
                    peerConnected = true
                    connectionStatus = "Peer found!"
                    onPeerJoined?()
                }
                
            case "peer-joined":
                let peerRole = json["peerRole"] as? String ?? "unknown"
                print("[SIGNALING] 🔵 Peer joined: \(peerRole)")
                print("[SIGNALING] 🔍 onSDPOfferReceived callback is \(onSDPOfferReceived == nil ? "NIL ❌" : "SET ✅")")
                peerConnected = true
                connectionStatus = "Python connected!"
                onPeerJoined?()
                
            case "peer-left":
                print("[SIGNALING] 🔴 Peer left")
                peerConnected = false
                connectionStatus = "Room: \(roomCode)"
                // Clear any pending messages from previous session
                clearPendingMessages()
                // Clear WebRTC callbacks so new WebRTCClient can register fresh ones
                // This is critical: offers arriving after this will be buffered
                // until the new WebRTCClient registers its callbacks
                clearWebRTCCallbacks()
                onPeerLeft?()
                
            case "sdp":
                if let sdp = json["sdp"] as? String,
                   let sdpType = json["sdpType"] as? String {
                    print("[SIGNALING] 📨 Received SDP \(sdpType) (length: \(sdp.count) chars)")
                    
                    // Extract stereo and enabled flags from offer (sent by Python)
                    if sdpType == "offer" {
                        // Debug: Log raw JSON values for stereo flags
                        let rawStereoVideo = json["stereoVideo"]
                        let rawStereoAudio = json["stereoAudio"]
                        let stereoVideoType = rawStereoVideo == nil ? "nil" : String(describing: Swift.type(of: rawStereoVideo!))
                        let stereoAudioType = rawStereoAudio == nil ? "nil" : String(describing: Swift.type(of: rawStereoAudio!))
                        print("[SIGNALING] 🔍 Raw JSON values: stereoVideo=\(String(describing: rawStereoVideo)) (type: \(stereoVideoType)), stereoAudio=\(String(describing: rawStereoAudio)) (type: \(stereoAudioType))")
                        
                        let stereoVideo = json["stereoVideo"] as? Bool ?? false
                        let stereoAudio = json["stereoAudio"] as? Bool ?? false
                        let videoEnabled = json["videoEnabled"] as? Bool ?? false
                        let audioEnabled = json["audioEnabled"] as? Bool ?? false
                        let simEnabled = json["simEnabled"] as? Bool ?? false
                        
                        print("[SIGNALING] 🎬 Stream config: stereoVideo=\(stereoVideo), stereoAudio=\(stereoAudio), video=\(videoEnabled), audio=\(audioEnabled), sim=\(simEnabled)")
                        
                        // Update DataManager with stream configuration
                        DataManager.shared.stereoEnabled = stereoVideo
                        DataManager.shared.stereoAudioEnabled = stereoAudio
                        DataManager.shared.videoEnabled = videoEnabled
                        DataManager.shared.audioEnabled = audioEnabled
                        DataManager.shared.simEnabled = simEnabled
                        
                        // Verify the flags were set correctly
                        print("[SIGNALING] ✅ DataManager.stereoEnabled is now: \(DataManager.shared.stereoEnabled)")
                        
                        print("[SIGNALING] 🔍 Checking if onSDPOfferReceived callback is set...")
                        if let callback = onSDPOfferReceived {
                            print("[SIGNALING] ✅ Callback IS set, delivering SDP offer now...")
                            callback(sdp)
                            print("[SIGNALING] ✅ Callback invoked successfully")
                        } else {
                            // Buffer the offer until callback is registered
                            print("[SIGNALING] ⚠️ Callback is NIL - Buffering SDP offer")
                            pendingSDPOffer = sdp
                        }
                    } else if sdpType == "answer" {
                        print("[SIGNALING] Delivering SDP answer to callback")
                        onSDPAnswerReceived?(sdp)
                    }
                } else {
                    print("[SIGNALING] ⚠️ Failed to parse SDP message: \(json)")
                }
                
            case "ice":
                if let candidate = json["candidate"] as? [String: Any] {
                    print("[SIGNALING] Received ICE candidate")
                    if let callback = onICECandidateReceived {
                        callback(candidate)
                    } else {
                        // Buffer the ICE candidate until callback is registered
                        print("[SIGNALING] Buffering ICE candidate (callback not yet registered)")
                        pendingICECandidates.append(candidate)
                    }
                }
                
            case "ice-servers":
                if let servers = json["servers"] as? [[String: Any]] {
                     print("[SIGNALING] Received \(servers.count) ICE servers")
                     var iceServers: [LKRTCIceServer] = []
                     
                     for serverDict in servers {
                         if let urls = serverDict["urls"] as? [String] {
                             let username = serverDict["username"] as? String
                             let credential = serverDict["credential"] as? String
                             
                             let iceServer = LKRTCIceServer(
                                 urlStrings: urls,
                                 username: username,
                                 credential: credential
                             )
                             iceServers.append(iceServer)
                         }
                     }
                     
                     onIceServersReceived?(iceServers)
                }
                
            case "request-offer":
                print("[SIGNALING] Received request for SDP offer")
                // Python requested an offer. We should trigger the WebRTC client to generate/send it.
                peerConnected = true
                onOfferRequested?()
                
            case "error":
                let errorMessage = json["message"] as? String ?? "Unknown error"
                print("[SIGNALING] Error: \(errorMessage)")
                connectionStatus = "Error: \(errorMessage)"
                
            case "pong":
                break  // Keepalive response
                
            default:
                print("[SIGNALING] Unknown message type: \(type)")
            }
        }
    }
    
    private func scheduleReconnect() {
        reconnectTask?.cancel()
        reconnectTask = Task { @MainActor in
            connectionStatus = "Reconnecting in 5s..."
            try? await Task.sleep(nanoseconds: 5_000_000_000)
            
            if !Task.isCancelled {
                connect()
            }
        }
    }
}
