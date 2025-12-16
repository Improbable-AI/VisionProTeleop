//
//  MultipeerCalibrationManager_iOS.swift
//  Tracking Viewer (iOS)
//
//  Manages Multipeer Connectivity for real-time calibration sync with Vision Pro.
//  Includes motion detection for automatic sample collection control.
//

import Foundation
import MultipeerConnectivity
import CoreMotion
import UIKit
import Combine

// MARK: - Calibration Command Protocol (Shared)

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
    case showCharuco                         // Display ChArUco board for intrinsic calibration
    case showAruco(markerId: Int)            // Display specific ArUco marker
    case showVerificationBoard(cols: Int, rows: Int, tagSizeMM: Float, marginMM: Float, tagIds: [Int])  // Display ArUco verification board
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
        case .showCharuco: return "Show ChArUco board"
        case .showAruco(let id): return "Show ArUco marker \(id)"
        case .showVerificationBoard(let cols, let rows, _, _, _): return "Show verification board \(cols)x\(rows)"
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

// MARK: - Motion Detector

/// Detects iPhone motion state for sample collection control
class MotionDetector: ObservableObject {
    
    @Published var isStationary: Bool = true
    @Published var motionMagnitude: Double = 0
    @Published var isStabilizing: Bool = false
    @Published var isRunning: Bool = false
    @Published var lastVariance: Double = 0
    
    private let motionManager = CMMotionManager()
    private var accelerometerData: [CMAcceleration] = []
    private let windowSize = 25  // Number of samples to average (0.5 sec at 50Hz)
    
    /// Threshold for considering the phone stationary (in g-force variance)
    /// Lower value = more sensitive (detects smaller movements)
    /// 0.00005 = extremely sensitive (detects micro-vibrations)
    /// 0.0001 = very sensitive
    /// 0.0005 = sensitive
    var stationaryThreshold: Double = 0.0001
    
    /// Minimum time the phone must be still before considered stationary (seconds)
    var stabilizationTime: TimeInterval = 0.8
    
    /// Time when phone became still
    private var stillStartTime: Date?
    
    /// Timer for status updates
    private var updateTimer: Timer?
    
    /// Callback when motion state changes
    var onMotionStateChanged: ((Bool, Double) -> Void)?
    
    init() {
        dlog("📱 [MotionDetector] Initialized")
    }
    
    deinit {
        stop()
    }
    
    func start() {
        guard !isRunning else {
            dlog("📱 [MotionDetector] Already running")
            return
        }
        
        guard motionManager.isAccelerometerAvailable else {
            dlog("❌ [MotionDetector] Accelerometer not available on this device!")
            return
        }
        
        accelerometerData = []
        isRunning = true
        
        // High frequency for responsive detection
        motionManager.accelerometerUpdateInterval = 0.02  // 50 Hz
        
        motionManager.startAccelerometerUpdates(to: .main) { [weak self] data, error in
            if let error = error {
                dlog("❌ [MotionDetector] Accelerometer error: \(error)")
                return
            }
            guard let self = self, let data = data else { return }
            self.processAccelerometerData(data.acceleration)
        }
        
        // Timer for periodic status updates
        updateTimer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
            self?.updateMotionState()
        }
        
        dlog("📱 [MotionDetector] Started motion detection (threshold: \(stationaryThreshold))")
    }
    
    func stop() {
        guard isRunning else { return }
        motionManager.stopAccelerometerUpdates()
        updateTimer?.invalidate()
        updateTimer = nil
        isRunning = false
        accelerometerData = []
        dlog("📱 [MotionDetector] Stopped motion detection")
    }
    
    private func processAccelerometerData(_ acceleration: CMAcceleration) {
        accelerometerData.append(acceleration)
        
        // Keep only the most recent samples
        while accelerometerData.count > windowSize {
            accelerometerData.removeFirst()
        }
    }
    
    private func updateMotionState() {
        guard accelerometerData.count >= windowSize / 2 else { 
            // Not enough data yet
            return 
        }
        
        // Calculate variance of acceleration magnitude
        let magnitudes = accelerometerData.map { sqrt($0.x * $0.x + $0.y * $0.y + $0.z * $0.z) }
        let mean = magnitudes.reduce(0, +) / Double(magnitudes.count)
        let variance = magnitudes.map { pow($0 - mean, 2) }.reduce(0, +) / Double(magnitudes.count)
        
        lastVariance = variance
        
        // Map variance to 0-1 motion magnitude (clamped)
        // Use stationaryThreshold * 10 as "full motion" reference
        let normalizedMotion = min(1.0, variance / (stationaryThreshold * 10))
        motionMagnitude = normalizedMotion
        
        let isNowStill = variance < stationaryThreshold
        
        if isNowStill {
            if stillStartTime == nil {
                stillStartTime = Date()
                isStabilizing = true
            }
            
            // Check if we've been still long enough
            if let startTime = stillStartTime,
               Date().timeIntervalSince(startTime) >= stabilizationTime {
                if !isStationary {
                    isStationary = true
                    isStabilizing = false
                    onMotionStateChanged?(true, motionMagnitude)
                    dlog("📱 [MotionDetector] Phone is now STATIONARY (variance: \(String(format: "%.6f", variance)), threshold: \(stationaryThreshold))")
                }
            }
        } else {
            stillStartTime = nil
            isStabilizing = false
            
            if isStationary {
                isStationary = false
                onMotionStateChanged?(false, motionMagnitude)
                dlog("📱 [MotionDetector] Phone is now MOVING (variance: \(String(format: "%.6f", variance)), threshold: \(stationaryThreshold))")
            }
        }
    }
}

// MARK: - Multipeer Calibration Manager (iOS)

/// Manages peer-to-peer communication with Vision Pro for calibration
/// This is the iOS version that ADVERTISES and receives commands FROM Vision Pro
@MainActor
class MultipeerCalibrationManager_iOS: NSObject, ObservableObject {
    static let shared = MultipeerCalibrationManager_iOS()
    
    // MARK: - Published State
    
    /// Whether connected to Vision Pro
    @Published var isConnected: Bool = false
    
    /// Name of connected Vision Pro (if any)
    @Published var connectedDeviceName: String? = nil
    
    /// Connection status message
    @Published var connectionStatus: String = "Not connected"
    
    /// Whether we're in calibration mode
    @Published var isInCalibrationMode: Bool = false
    
    /// Current marker ID to display (from Vision Pro command)
    @Published var commandedMarkerId: Int = 0
    
    /// Whether sample collection is paused (phone moving)
    @Published var isCollectionPaused: Bool = false
    
    /// Motion detector for automatic pause/resume
    @Published var motionDetector = MotionDetector()
    
    /// Whether the phone is stationary
    @Published var isPhoneStationary: Bool = true
    
    /// Current motion magnitude (0-1)
    @Published var motionMagnitude: Double = 0
    
    // MARK: - New Calibration Wizard State
    
    /// Current display mode commanded by Vision Pro
    enum CalibrationDisplayMode: Equatable {
        case none
        case checkerboard
        case charuco
        case aruco(markerId: Int)
        case verificationBoard(cols: Int, rows: Int, tagSizeMM: Float, marginMM: Float, tagIds: [Int])
    }
    
    /// What should be displayed on screen
    @Published var displayMode: CalibrationDisplayMode = .none
    
    /// Intrinsic calibration progress (from Vision Pro)
    @Published var intrinsicProgress: (samples: Int, total: Int) = (0, 30)
    
    /// Extrinsic calibration progress for current marker
    @Published var extrinsicProgress: (marker: Int, samples: Int, total: Int) = (0, 0, 30)
    
    /// Whether ARKit has detected the marker on Vision Pro
    @Published var markerDetectedByARKit: Bool = false
    
    /// Whether ready for next marker position
    @Published var readyForNextPosition: Bool = false
    
    // MARK: - Multipeer Properties
    
    private let serviceType = "vp-calibrate" // Must match visionOS version
    private var peerID: MCPeerID!
    private var session: MCSession!
    private var advertiser: MCNearbyServiceAdvertiser!
    
    private var statusTimer: Timer?
    
    // MARK: - Callbacks
    
    /// Called when a command is received from Vision Pro
    var onCommandReceived: ((CalibrationCommand) -> Void)?
    
    /// Called when marker should switch (from Vision Pro command)
    var onMarkerSwitch: ((Int) -> Void)?
    
    /// Called when connection state changes
    var onConnectionChanged: ((Bool) -> Void)?
    
    /// Called when display mode changes (for automatic view transitions)
    var onDisplayModeChanged: ((CalibrationDisplayMode) -> Void)?
    
    // MARK: - Initialization
    
    private override init() {
        super.init()
        setupMultipeer()
        setupMotionDetector()
    }
    
    private func setupMultipeer() {
        peerID = MCPeerID(displayName: UIDevice.current.name)
        session = MCSession(peer: peerID, securityIdentity: nil, encryptionPreference: .required)
        session.delegate = self
        
        advertiser = MCNearbyServiceAdvertiser(peer: peerID, discoveryInfo: nil, serviceType: serviceType)
        advertiser.delegate = self
        
        dlog("📱 [MultipeerCalibrationManager_iOS] Initialized for iOS")
    }
    
    private func setupMotionDetector() {
        motionDetector.onMotionStateChanged = { [weak self] isStationary, magnitude in
            Task { @MainActor in
                self?.isPhoneStationary = isStationary
                self?.motionMagnitude = magnitude
                self?.sendStatusUpdate()
            }
        }
    }
    
    // MARK: - Connection Management
    
    /// Start advertising to nearby Vision Pro devices
    func startAdvertising() {
        advertiser.startAdvertisingPeer()
        connectionStatus = "Waiting for Vision Pro..."
        dlog("📱 [MultipeerCalibrationManager_iOS] Started advertising")
    }
    
    /// Stop advertising
    func stopAdvertising() {
        advertiser.stopAdvertisingPeer()
        dlog("📱 [MultipeerCalibrationManager_iOS] Stopped advertising")
    }
    
    /// Disconnect from current peer
    func disconnect() {
        session.disconnect()
        stopStatusUpdates()
        motionDetector.stop()
        isConnected = false
        connectedDeviceName = nil
        isInCalibrationMode = false
        connectionStatus = "Disconnected"
        dlog("📱 [MultipeerCalibrationManager_iOS] Disconnected")
    }
    
    // MARK: - Status Updates
    
    /// Start sending periodic status updates to Vision Pro
    func startStatusUpdates() {
        motionDetector.start()
        
        // Send status updates at 10 Hz
        statusTimer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
            Task { @MainActor in
                self?.sendStatusUpdate()
            }
        }
        
        dlog("📱 [MultipeerCalibrationManager_iOS] Started status updates")
    }
    
    /// Stop sending status updates
    func stopStatusUpdates() {
        statusTimer?.invalidate()
        statusTimer = nil
        motionDetector.stop()
        dlog("📱 [MultipeerCalibrationManager_iOS] Stopped status updates")
    }
    
    /// Send current status to Vision Pro
    private func sendStatusUpdate() {
        guard isConnected, !session.connectedPeers.isEmpty else { return }
        
        let status = CalibrationStatus(
            currentMarkerId: commandedMarkerId,
            isPhoneStationary: isPhoneStationary,
            motionMagnitude: motionMagnitude,
            isDisplayingMarker: isInCalibrationMode
        )
        
        do {
            let message = try CalibrationMessage(status: status)
            let data = try JSONEncoder().encode(message)
            try session.send(data, toPeers: session.connectedPeers, with: .unreliable)  // Use unreliable for high frequency
        } catch {
            // Don't spam errors for status updates
        }
    }
    
    // MARK: - Command Handling
    
    private func handleCommand(_ command: CalibrationCommand) {
        dlog("📱 [MultipeerCalibrationManager_iOS] Received command: \(command.description)")
        
        switch command {
        case .startCalibration(let markerIds):
            isInCalibrationMode = true
            if let firstId = markerIds.first {
                commandedMarkerId = firstId
                onMarkerSwitch?(firstId)
            }
            startStatusUpdates()
            
        case .switchToMarker(let markerId):
            commandedMarkerId = markerId
            displayMode = .aruco(markerId: markerId)
            onMarkerSwitch?(markerId)
            // Trigger haptic feedback
            let generator = UIImpactFeedbackGenerator(style: .medium)
            generator.impactOccurred()
            
        case .pauseCollection:
            isCollectionPaused = true
            
        case .resumeCollection:
            isCollectionPaused = false
            
        case .calibrationComplete:
            isInCalibrationMode = false
            displayMode = .none
            markerDetectedByARKit = false
            readyForNextPosition = false
            stopStatusUpdates()
            // Trigger success haptic
            let generator = UINotificationFeedbackGenerator()
            generator.notificationOccurred(.success)
            
        case .requestStatus:
            sendStatusUpdate()
            
        // New commands for unified wizard
        case .showCheckerboard:
            isInCalibrationMode = true
            displayMode = .checkerboard
            startStatusUpdates()
            // Trigger haptic to indicate mode change
            let generator = UIImpactFeedbackGenerator(style: .medium)
            generator.impactOccurred()
            
        case .showCharuco:
            isInCalibrationMode = true
            displayMode = .charuco
            startStatusUpdates()
            // Trigger haptic to indicate mode change
            let generator = UIImpactFeedbackGenerator(style: .medium)
            generator.impactOccurred()
            
        case .showAruco(let markerId):
            isInCalibrationMode = true
            displayMode = .aruco(markerId: markerId)
            commandedMarkerId = markerId
            markerDetectedByARKit = false
            readyForNextPosition = false
            onMarkerSwitch?(markerId)
            startStatusUpdates()
            // Trigger haptic
            let generator = UIImpactFeedbackGenerator(style: .medium)
            generator.impactOccurred()
            
        case .showVerificationBoard(let cols, let rows, let tagSizeMM, let marginMM, let tagIds):
            isInCalibrationMode = true
            displayMode = .verificationBoard(cols: cols, rows: rows, tagSizeMM: tagSizeMM, marginMM: marginMM, tagIds: tagIds)
            startStatusUpdates()
            // Trigger haptic
            let generator = UIImpactFeedbackGenerator(style: .medium)
            generator.impactOccurred()
            
        case .hideDisplay:
            displayMode = .none
            isInCalibrationMode = false
            stopStatusUpdates()
            
        case .intrinsicProgress(let samples, let total):
            intrinsicProgress = (samples, total)
            
        case .extrinsicProgress(let marker, let samples, let total):
            extrinsicProgress = (marker, samples, total)
            
        case .markerDetected:
            markerDetectedByARKit = true
            // Play a nice detection sound
            let generator = UINotificationFeedbackGenerator()
            generator.notificationOccurred(.success)
            
        case .readyForNextPosition:
            readyForNextPosition = true
            // Strong haptic to get attention
            let generator = UINotificationFeedbackGenerator()
            generator.notificationOccurred(.warning)
        }
        
        onCommandReceived?(command)
    }
}

// MARK: - MCSessionDelegate

extension MultipeerCalibrationManager_iOS: MCSessionDelegate {
    nonisolated func session(_ session: MCSession, peer peerID: MCPeerID, didChange state: MCSessionState) {
        Task { @MainActor in
            switch state {
            case .connected:
                isConnected = true
                connectedDeviceName = peerID.displayName
                connectionStatus = "Connected to \(peerID.displayName)"
                onConnectionChanged?(true)
                dlog("📱 [MultipeerCalibrationManager_iOS] Connected to: \(peerID.displayName)")
                
            case .connecting:
                connectionStatus = "Connecting to \(peerID.displayName)..."
                dlog("📱 [MultipeerCalibrationManager_iOS] Connecting to: \(peerID.displayName)")
                
            case .notConnected:
                isConnected = session.connectedPeers.count > 0
                if !isConnected {
                    connectedDeviceName = nil
                    connectionStatus = "Disconnected from \(peerID.displayName)"
                    isInCalibrationMode = false
                    stopStatusUpdates()
                    onConnectionChanged?(false)
                }
                dlog("📱 [MultipeerCalibrationManager_iOS] Disconnected from: \(peerID.displayName)")
                
            @unknown default:
                break
            }
        }
    }
    
    nonisolated func session(_ session: MCSession, didReceive data: Data, fromPeer peerID: MCPeerID) {
        Task { @MainActor in
            do {
                let message = try JSONDecoder().decode(CalibrationMessage.self, from: data)
                
                if let command = try message.getCommand() {
                    handleCommand(command)
                }
            } catch {
                dlog("❌ [MultipeerCalibrationManager_iOS] Failed to decode message: \(error)")
            }
        }
    }
    
    nonisolated func session(_ session: MCSession, didReceive stream: InputStream, withName streamName: String, fromPeer peerID: MCPeerID) {}
    
    nonisolated func session(_ session: MCSession, didStartReceivingResourceWithName resourceName: String, fromPeer peerID: MCPeerID, with progress: Progress) {}
    
    nonisolated func session(_ session: MCSession, didFinishReceivingResourceWithName resourceName: String, fromPeer peerID: MCPeerID, at localURL: URL?, withError error: Error?) {}
}

// MARK: - MCNearbyServiceAdvertiserDelegate

extension MultipeerCalibrationManager_iOS: MCNearbyServiceAdvertiserDelegate {
    nonisolated func advertiser(_ advertiser: MCNearbyServiceAdvertiser, didReceiveInvitationFromPeer peerID: MCPeerID, withContext context: Data?, invitationHandler: @escaping (Bool, MCSession?) -> Void) {
        Task { @MainActor in
            // Auto-accept invitations from Vision Pro
            if peerID.displayName.contains("Vision") {
                invitationHandler(true, session)
                dlog("📱 [MultipeerCalibrationManager_iOS] Accepted invitation from: \(peerID.displayName)")
            } else {
                invitationHandler(true, session)  // Accept all for now
                dlog("📱 [MultipeerCalibrationManager_iOS] Accepted invitation from: \(peerID.displayName)")
            }
        }
    }
    
    nonisolated func advertiser(_ advertiser: MCNearbyServiceAdvertiser, didNotStartAdvertisingPeer error: Error) {
        Task { @MainActor in
            connectionStatus = "Failed to advertise: \(error.localizedDescription)"
            dlog("❌ [MultipeerCalibrationManager_iOS] Advertiser error: \(error)")
        }
    }
}
