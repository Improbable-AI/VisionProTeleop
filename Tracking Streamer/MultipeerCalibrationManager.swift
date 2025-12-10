//
//  MultipeerCalibrationManager.swift
//  Tracking Streamer (visionOS)
//
//  Manages Multipeer Connectivity for real-time calibration sync between Vision Pro and iPhone.
//

import Foundation
import MultipeerConnectivity
import Combine

// MARK: - Calibration Command Protocol

/// Commands sent from Vision Pro to iPhone during calibration
enum CalibrationCommand: Codable {
    // Existing commands
    case startCalibration(markerIds: [Int])
    case switchToMarker(markerId: Int)
    case pauseCollection
    case resumeCollection
    case calibrationComplete
    case requestStatus
    
    // New commands for unified calibration wizard
    case showCheckerboard                    // Display checkerboard for intrinsic calibration
    case showAruco(markerId: Int)            // Display specific ArUco marker
    case hideDisplay                         // Hide current display (return to normal)
    case intrinsicProgress(samples: Int, total: Int)  // Update intrinsic progress
    case extrinsicProgress(marker: Int, samples: Int, total: Int)  // Update extrinsic progress per marker
    case markerDetected                      // ARKit detected the marker
    case readyForNextPosition                // Time to move iPhone to new position
    
    var description: String {
        switch self {
        case .startCalibration(let ids): return "Start calibration with markers \(ids)"
        case .switchToMarker(let id): return "Switch to marker \(id)"
        case .pauseCollection: return "Pause sample collection"
        case .resumeCollection: return "Resume sample collection"
        case .calibrationComplete: return "Calibration complete"
        case .requestStatus: return "Request status"
        case .showCheckerboard: return "Show checkerboard pattern"
        case .showAruco(let id): return "Show ArUco marker \(id)"
        case .hideDisplay: return "Hide calibration display"
        case .intrinsicProgress(let s, let t): return "Intrinsic progress: \(s)/\(t)"
        case .extrinsicProgress(let m, let s, let t): return "Extrinsic marker \(m): \(s)/\(t)"
        case .markerDetected: return "Marker detected by ARKit"
        case .readyForNextPosition: return "Ready for next marker position"
        }
    }
}

/// Status updates sent from iPhone to Vision Pro
struct CalibrationStatus: Codable {
    let currentMarkerId: Int
    let isPhoneStationary: Bool
    let motionMagnitude: Double  // 0-1 scale
    let isDisplayingMarker: Bool
    let timestamp: Date
    
    init(currentMarkerId: Int, isPhoneStationary: Bool, motionMagnitude: Double = 0, isDisplayingMarker: Bool) {
        self.currentMarkerId = currentMarkerId
        self.isPhoneStationary = isPhoneStationary
        self.motionMagnitude = motionMagnitude
        self.isDisplayingMarker = isDisplayingMarker
        self.timestamp = Date()
    }
}

/// Message wrapper for peer communication
struct CalibrationMessage: Codable {
    enum MessageType: String, Codable {
        case command
        case status
    }
    
    let type: MessageType
    let commandData: Data?
    let statusData: Data?
    
    init(command: CalibrationCommand) throws {
        self.type = .command
        self.commandData = try JSONEncoder().encode(command)
        self.statusData = nil
    }
    
    init(status: CalibrationStatus) throws {
        self.type = .status
        self.commandData = nil
        self.statusData = try JSONEncoder().encode(status)
    }
    
    func getCommand() throws -> CalibrationCommand? {
        guard type == .command, let data = commandData else { return nil }
        return try JSONDecoder().decode(CalibrationCommand.self, from: data)
    }
    
    func getStatus() throws -> CalibrationStatus? {
        guard type == .status, let data = statusData else { return nil }
        return try JSONDecoder().decode(CalibrationStatus.self, from: data)
    }
}

// MARK: - Multipeer Calibration Manager (visionOS)

/// Manages peer-to-peer communication with iPhone for calibration
/// This is the visionOS version that BROWSES for and sends commands TO iPhone
@MainActor
class MultipeerCalibrationManager: NSObject, ObservableObject {
    static let shared = MultipeerCalibrationManager()
    
    // MARK: - Published State
    
    /// Whether connected to an iPhone
    @Published var isConnected: Bool = false
    
    /// Name of connected iPhone (if any)
    @Published var connectedDeviceName: String? = nil
    
    /// Latest status from iPhone
    @Published var iPhoneStatus: CalibrationStatus? = nil
    
    /// Whether iPhone is stationary (good for sample collection)
    @Published var isPhoneStationary: Bool = false
    
    /// Motion magnitude from iPhone (0 = still, 1 = moving a lot)
    @Published var phoneMotionMagnitude: Double = 0
    
    /// Connection status message
    @Published var connectionStatus: String = "Not connected"
    
    /// Available peers
    @Published var availablePeers: [MCPeerID] = []
    
    // MARK: - Multipeer Properties
    
    private let serviceType = "vp-calibrate" // Must be 1-15 chars, lowercase + hyphen only
    private var peerID: MCPeerID!
    private var session: MCSession!
    private var browser: MCNearbyServiceBrowser!
    
    // MARK: - Callbacks
    
    /// Called when iPhone status is received
    var onStatusReceived: ((CalibrationStatus) -> Void)?
    
    /// Called when connection state changes
    var onConnectionChanged: ((Bool) -> Void)?
    
    // MARK: - Initialization
    
    private override init() {
        super.init()
        setupMultipeer()
    }
    
    private func setupMultipeer() {
        peerID = MCPeerID(displayName: "Vision Pro")
        session = MCSession(peer: peerID, securityIdentity: nil, encryptionPreference: .required)
        session.delegate = self
        
        browser = MCNearbyServiceBrowser(peer: peerID, serviceType: serviceType)
        browser.delegate = self
        
        dlog("📡 [MultipeerCalibrationManager] Initialized for visionOS")
    }
    
    // MARK: - Connection Management
    
    /// Start browsing for nearby iPhones
    func startBrowsing() {
        browser.startBrowsingForPeers()
        connectionStatus = "Searching for iPhone..."
        dlog("📡 [MultipeerCalibrationManager] Started browsing for peers")
    }
    
    /// Stop browsing
    func stopBrowsing() {
        browser.stopBrowsingForPeers()
        dlog("📡 [MultipeerCalibrationManager] Stopped browsing")
    }
    
    /// Disconnect from current peer
    func disconnect() {
        session.disconnect()
        isConnected = false
        connectedDeviceName = nil
        connectionStatus = "Disconnected"
        dlog("📡 [MultipeerCalibrationManager] Disconnected")
    }
    
    /// Connect to a specific peer
    func connect(to peer: MCPeerID) {
        browser.invitePeer(peer, to: session, withContext: nil, timeout: 30)
        connectionStatus = "Connecting to \(peer.displayName)..."
        dlog("📡 [MultipeerCalibrationManager] Inviting peer: \(peer.displayName)")
    }
    
    // MARK: - Command Sending
    
    /// Send a command to the connected iPhone
    func sendCommand(_ command: CalibrationCommand) {
        guard isConnected, !session.connectedPeers.isEmpty else {
            dlog("⚠️ [MultipeerCalibrationManager] Cannot send command - not connected")
            return
        }
        
        do {
            let message = try CalibrationMessage(command: command)
            let data = try JSONEncoder().encode(message)
            try session.send(data, toPeers: session.connectedPeers, with: .reliable)
            dlog("📡 [MultipeerCalibrationManager] Sent command: \(command.description)")
        } catch {
            dlog("❌ [MultipeerCalibrationManager] Failed to send command: \(error)")
        }
    }
    
    /// Tell iPhone to start calibration with specific markers
    func startCalibration(markerIds: [Int]) {
        sendCommand(.startCalibration(markerIds: markerIds))
    }
    
    /// Tell iPhone to switch to a specific marker
    func switchToMarker(_ markerId: Int) {
        sendCommand(.switchToMarker(markerId: markerId))
    }
    
    /// Tell iPhone to pause (phone is moving)
    func pauseCollection() {
        sendCommand(.pauseCollection)
    }
    
    /// Tell iPhone to resume (phone is still)
    func resumeCollection() {
        sendCommand(.resumeCollection)
    }
    
    /// Tell iPhone calibration is complete
    func calibrationComplete() {
        sendCommand(.calibrationComplete)
    }
    
    /// Request current status from iPhone
    func requestStatus() {
        sendCommand(.requestStatus)
    }
}

// MARK: - MCSessionDelegate

extension MultipeerCalibrationManager: MCSessionDelegate {
    nonisolated func session(_ session: MCSession, peer peerID: MCPeerID, didChange state: MCSessionState) {
        Task { @MainActor in
            switch state {
            case .connected:
                isConnected = true
                connectedDeviceName = peerID.displayName
                connectionStatus = "Connected to \(peerID.displayName)"
                onConnectionChanged?(true)
                dlog("📡 [MultipeerCalibrationManager] Connected to: \(peerID.displayName)")
                
            case .connecting:
                connectionStatus = "Connecting to \(peerID.displayName)..."
                dlog("📡 [MultipeerCalibrationManager] Connecting to: \(peerID.displayName)")
                
            case .notConnected:
                isConnected = session.connectedPeers.count > 0
                if !isConnected {
                    connectedDeviceName = nil
                    connectionStatus = "Disconnected from \(peerID.displayName)"
                    onConnectionChanged?(false)
                }
                dlog("📡 [MultipeerCalibrationManager] Disconnected from: \(peerID.displayName)")
                
            @unknown default:
                break
            }
        }
    }
    
    nonisolated func session(_ session: MCSession, didReceive data: Data, fromPeer peerID: MCPeerID) {
        Task { @MainActor in
            do {
                let message = try JSONDecoder().decode(CalibrationMessage.self, from: data)
                
                if let status = try message.getStatus() {
                    iPhoneStatus = status
                    isPhoneStationary = status.isPhoneStationary
                    phoneMotionMagnitude = status.motionMagnitude
                    onStatusReceived?(status)
                    
                    // Log only significant status changes
                    if !status.isPhoneStationary {
                        dlog("📡 [MultipeerCalibrationManager] iPhone moving (motion: \(String(format: "%.2f", status.motionMagnitude)))")
                    }
                }
            } catch {
                dlog("❌ [MultipeerCalibrationManager] Failed to decode message: \(error)")
            }
        }
    }
    
    nonisolated func session(_ session: MCSession, didReceive stream: InputStream, withName streamName: String, fromPeer peerID: MCPeerID) {}
    
    nonisolated func session(_ session: MCSession, didStartReceivingResourceWithName resourceName: String, fromPeer peerID: MCPeerID, with progress: Progress) {}
    
    nonisolated func session(_ session: MCSession, didFinishReceivingResourceWithName resourceName: String, fromPeer peerID: MCPeerID, at localURL: URL?, withError error: Error?) {}
}

// MARK: - MCNearbyServiceBrowserDelegate

extension MultipeerCalibrationManager: MCNearbyServiceBrowserDelegate {
    nonisolated func browser(_ browser: MCNearbyServiceBrowser, foundPeer peerID: MCPeerID, withDiscoveryInfo info: [String : String]?) {
        Task { @MainActor in
            if !availablePeers.contains(where: { $0.displayName == peerID.displayName }) {
                availablePeers.append(peerID)
                dlog("📡 [MultipeerCalibrationManager] Found peer: \(peerID.displayName)")
                
                // Auto-connect if it's an iPhone (starts with "iPhone")
                if peerID.displayName.contains("iPhone") {
                    connect(to: peerID)
                }
            }
        }
    }
    
    nonisolated func browser(_ browser: MCNearbyServiceBrowser, lostPeer peerID: MCPeerID) {
        Task { @MainActor in
            availablePeers.removeAll { $0.displayName == peerID.displayName }
            dlog("📡 [MultipeerCalibrationManager] Lost peer: \(peerID.displayName)")
        }
    }
    
    nonisolated func browser(_ browser: MCNearbyServiceBrowser, didNotStartBrowsingForPeers error: Error) {
        Task { @MainActor in
            connectionStatus = "Failed to start browsing: \(error.localizedDescription)"
            dlog("❌ [MultipeerCalibrationManager] Browser error: \(error)")
        }
    }
}
