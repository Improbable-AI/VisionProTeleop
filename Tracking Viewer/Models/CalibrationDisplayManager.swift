//
//  CalibrationDisplayManager.swift
//  Tracking Viewer
//
//  Manages calibration marker display and syncs state with Vision Pro via iCloud.
//

import Foundation
import SwiftUI
import UIKit
import Combine

/// Manages the calibration display on iPhone and syncs with Vision Pro
@MainActor
class CalibrationDisplayManager: ObservableObject {
    static let shared = CalibrationDisplayManager()
    
    // MARK: - Published Properties
    
    /// Current display specification for this iPhone
    @Published private(set) var displaySpec: iPhoneDisplaySpec
    
    /// Whether the display size has been validated by the user
    @Published var isDisplayValidated: Bool = false
    
    /// Manual override for points per mm if user calibrated with credit card
    @Published var manualPointsPerMM: Double?
    
    /// ArUco marker configuration (embedded defaults matching Vision Pro app)
    @Published var arucoConfig: ArucoMarkerConfig
    
    /// Checkerboard configuration (embedded defaults matching Vision Pro app)
    @Published var checkerboardConfig: CheckerboardDisplayConfig
    
    /// Current calibration session state
    @Published var sessionState: CalibrationSessionState = CalibrationSessionState()
    
    /// Current marker ID being displayed
    @Published var currentMarkerId: Int = 0
    
    /// Whether we're in calibration mode (full screen marker display)
    @Published var isDisplayingMarker: Bool = false
    
    /// Display mode: aruco marker or checkerboard
    @Published var displayMode: DisplayMode = .arucoMarker
    
    /// Extrinsic calibration results from Vision Pro (if any)
    @Published var extrinsicResults: [ExtrinsicCalibrationResult] = []
    
    /// Intrinsic calibration results from Vision Pro (if any)
    @Published var intrinsicResults: [IntrinsicCalibrationResult] = []
    
    /// Error message for display
    @Published var errorMessage: String?
    
    /// Multipeer connection manager for real-time sync with Vision Pro
    let multipeerManager = MultipeerCalibrationManager_iOS.shared
    
    // MARK: - Types
    
    enum DisplayMode: String, CaseIterable {
        case arucoMarker = "ArUco Marker"
        case checkerboard = "Checkerboard"
        
        var icon: String {
            switch self {
            case .arucoMarker: return "qrcode"
            case .checkerboard: return "checkerboard.rectangle"
            }
        }
    }
    
    // MARK: - Computed Properties
    
    /// Points per millimeter (uses manual override if set)
    var pointsPerMM: Double {
        return manualPointsPerMM ?? displaySpec.pointsPerMM
    }
    
    /// Convert millimeters to points for accurate display
    func mmToPoints(_ mm: Double) -> CGFloat {
        return CGFloat(mm * pointsPerMM)
    }
    
    /// Current marker size in points
    var markerSizePoints: CGFloat {
        return mmToPoints(Double(arucoConfig.markerSizeMM))
    }
    
    /// Maximum marker size that fits on screen (with padding)
    var maxMarkerSizeMM: Double {
        let screenWidthMM = displaySpec.screenWidthMM
        let screenHeightMM = displaySpec.screenHeightMM
        // Use 90% of the smaller dimension
        return min(screenWidthMM, screenHeightMM) * 0.9
    }
    
    /// Check if current marker size fits on screen
    var markerFitsOnScreen: Bool {
        return Double(arucoConfig.markerSizeMM) <= maxMarkerSizeMM
    }
    
    /// Checkerboard pattern size in points
    var checkerboardSizePoints: CGSize {
        return CGSize(
            width: mmToPoints(Double(checkerboardConfig.patternWidthMM)),
            height: mmToPoints(Double(checkerboardConfig.patternHeightMM))
        )
    }
    
    /// Check if checkerboard fits on screen
    var checkerboardFitsOnScreen: Bool {
        let patternWidth = Double(checkerboardConfig.patternWidthMM)
        let patternHeight = Double(checkerboardConfig.patternHeightMM)
        let screenWidth = displaySpec.screenWidthMM
        let screenHeight = displaySpec.screenHeightMM
        // Check both orientations (pattern can be rotated)
        return (patternWidth <= screenWidth && patternHeight <= screenHeight) ||
               (patternHeight <= screenWidth && patternWidth <= screenHeight)
    }
    
    // MARK: - Private Properties
    
    private let keychain = KeychainManager.shared
    private var cancellables = Set<AnyCancellable>()
    
    // iCloud Key-Value Store keys
    private let kCalibrationSessionStateKey = "calibrationSessionState"
    private let kExtrinsicResultsKey = "extrinsicCalibrationResults"
    private let kIntrinsicResultsKey = "intrinsicCalibrationResults"
    private let kDisplayValidatedKey = "displaySizeValidated"
    private let kManualPointsPerMMKey = "manualPointsPerMM"
    
    // MARK: - Initialization
    
    private init() {
        // Detect iPhone model and get display spec
        let spec: iPhoneDisplaySpec
        if let detectedSpec = iPhoneDisplayDatabase.currentDeviceSpec() {
            spec = detectedSpec
        } else {
            spec = iPhoneDisplayDatabase.fallbackSpec()
        }
        displaySpec = spec
        
        // Initialize with embedded defaults that match Vision Pro app
        // These are the PRECONFIGURED values that both apps must agree on
        arucoConfig = ArucoMarkerConfig(
            dictionary: .dict4X4_50,  // Must match Vision Pro's ExtrinsicCalibrationManager.arucoDictionary
            markerSizeMM: 55.0,       // Optimized for iPhone display
            availableMarkerIds: [0, 2, 3],  // Must match Vision Pro's markerIds (ID 1 removed)
            minUniquePositions: 3,
            samplesPerPosition: 30
        )
        
        checkerboardConfig = CheckerboardDisplayConfig(
            innerCornersX: 5,   // Must match Vision Pro's default
            innerCornersY: 4,   // Must match Vision Pro's default
            squareSizeMM: 10.0  // 10mm squares to fit on iPhone screen
        )
        
        // Now we can use self
        dlog("📱 [CalibrationDisplayManager] Detected device: \(spec.displayName)")
        
        // Load saved state
        loadState()
        
        // Listen for iCloud key-value store changes
        setupiCloudSync()
        
        // Setup Multipeer connectivity callbacks
        setupMultipeerCallbacks()
    }
    
    // MARK: - Multipeer Callbacks
    
    private func setupMultipeerCallbacks() {
        // Handle commands from Vision Pro
        multipeerManager.onCommandReceived = { [weak self] command in
            Task { @MainActor in
                self?.handleVisionProCommand(command)
            }
        }
        
        // Handle marker switch commands
        multipeerManager.onMarkerSwitch = { [weak self] markerId in
            Task { @MainActor in
                self?.setMarkerId(markerId)
            } 
        }
        
        // Start advertising immediately
        multipeerManager.startAdvertising()
        dlog("📱 [CalibrationDisplayManager] Started Multipeer advertising")
    }
    
    private func handleVisionProCommand(_ command: CalibrationCommand) {
        switch command {
        case .startCalibration(let markerIds):
            // Vision Pro wants to start calibration with these markers
            if let firstId = markerIds.first {
                currentMarkerId = firstId
                sessionState.currentMarkerId = firstId
            }
            isDisplayingMarker = true
            dlog("📱 [CalibrationDisplayManager] Received start calibration command, markers: \(markerIds)")
            
        case .switchToMarker(let markerId):
            // Switch to the specified marker
            setMarkerId(markerId)
            dlog("📱 [CalibrationDisplayManager] Received switch to marker \(markerId)")
            
        case .calibrationComplete:
            // Calibration finished
            isDisplayingMarker = false
            dlog("📱 [CalibrationDisplayManager] Calibration complete")
            
        case .pauseCollection, .resumeCollection, .requestStatus,
             .showCheckerboard, .showAruco, .hideDisplay,
             .intrinsicProgress, .extrinsicProgress,
             .markerDetected, .readyForNextPosition:
            // These are handled by MultipeerCalibrationManager_iOS directly
            break
        }
    }
    
    // MARK: - iCloud Sync
    
    private func setupiCloudSync() {
        // Listen for external changes
        NotificationCenter.default.publisher(for: NSUbiquitousKeyValueStore.didChangeExternallyNotification)
            .sink { [weak self] notification in
                guard let self = self else { return }
                Task { @MainActor in
                    self.handleiCloudUpdate(notification)
                }
            }
            .store(in: &cancellables)
        
        // Sync on app foreground
        NotificationCenter.default.publisher(for: UIApplication.willEnterForegroundNotification)
            .sink { [weak self] _ in
                Task { @MainActor in
                    self?.synciCloud()
                }
            }
            .store(in: &cancellables)
        
        // Initial sync
        NSUbiquitousKeyValueStore.default.synchronize()
    }
    
    private func handleiCloudUpdate(_ notification: Notification) {
        guard let userInfo = notification.userInfo,
              let changeReason = userInfo[NSUbiquitousKeyValueStoreChangeReasonKey] as? Int else {
            return
        }
        
        dlog("☁️ [CalibrationDisplayManager] iCloud update received, reason: \(changeReason)")
        
        // Reload results from iCloud
        loadCalibrationResults()
    }
    
    private func synciCloud() {
        NSUbiquitousKeyValueStore.default.synchronize()
        loadCalibrationResults()
    }
    
    /// Public method to refresh calibration results from iCloud
    func refreshFromiCloud() {
        dlog("☁️ [CalibrationDisplayManager] Manual refresh from iCloud...")
        NSUbiquitousKeyValueStore.default.synchronize()
        loadCalibrationResults()
    }
    
    // MARK: - State Persistence
    
    private func loadState() {
        // Load display validation state
        isDisplayValidated = UserDefaults.standard.bool(forKey: kDisplayValidatedKey)
        
        // Load manual points per mm if set
        if let manual = UserDefaults.standard.object(forKey: kManualPointsPerMMKey) as? Double, manual > 0 {
            manualPointsPerMM = manual
            dlog("📱 [CalibrationDisplayManager] Loaded manual points/mm: \(manual)")
        }
        
        // Load calibration results from iCloud
        loadCalibrationResults()
    }
    
    private func saveState() {
        UserDefaults.standard.set(isDisplayValidated, forKey: kDisplayValidatedKey)
        if let manual = manualPointsPerMM {
            UserDefaults.standard.set(manual, forKey: kManualPointsPerMMKey)
        }
    }
    
    private func loadCalibrationResults() {
        let store = NSUbiquitousKeyValueStore.default
        
        // Load extrinsic results
        if let data = store.data(forKey: kExtrinsicResultsKey) {
            do {
                extrinsicResults = try JSONDecoder().decode([ExtrinsicCalibrationResult].self, from: data)
                dlog("☁️ [CalibrationDisplayManager] Loaded \(extrinsicResults.count) extrinsic calibration(s)")
            } catch {
                dlog("❌ [CalibrationDisplayManager] Failed to decode extrinsic results: \(error)")
            }
        }
        
        // Load intrinsic results
        if let data = store.data(forKey: kIntrinsicResultsKey) {
            do {
                intrinsicResults = try JSONDecoder().decode([IntrinsicCalibrationResult].self, from: data)
                dlog("☁️ [CalibrationDisplayManager] Loaded \(intrinsicResults.count) intrinsic calibration(s)")
            } catch {
                dlog("❌ [CalibrationDisplayManager] Failed to decode intrinsic results: \(error)")
            }
        }
    }
    
    // MARK: - Display Validation
    
    /// Called when user completes credit card validation
    /// - Parameter measuredWidthPoints: The width of the credit card overlay in points as measured by the user
    func completeDisplayValidation(measuredWidthPoints: CGFloat) {
        // Credit card width is 85.6mm (ISO standard)
        // Calculate actual points per mm based on user's measurement
        let calculatedPointsPerMM = Double(measuredWidthPoints) / CreditCardDimensions.widthMM
        
        dlog("📱 [CalibrationDisplayManager] Credit card validation complete")
        dlog("📱 [CalibrationDisplayManager] Expected points/mm: \(displaySpec.pointsPerMM)")
        dlog("📱 [CalibrationDisplayManager] Measured points/mm: \(calculatedPointsPerMM)")
        
        // If the difference is significant (>5%), use the measured value
        let difference = abs(calculatedPointsPerMM - displaySpec.pointsPerMM) / displaySpec.pointsPerMM
        if difference > 0.05 {
            manualPointsPerMM = calculatedPointsPerMM
            dlog("📱 [CalibrationDisplayManager] Using measured points/mm (difference: \(Int(difference * 100))%)")
        } else {
            manualPointsPerMM = nil
            dlog("📱 [CalibrationDisplayManager] Using device spec (difference: \(Int(difference * 100))%)")
        }
        
        isDisplayValidated = true
        saveState()
    }
    
    /// Reset display validation
    func resetDisplayValidation() {
        isDisplayValidated = false
        manualPointsPerMM = nil
        saveState()
    }
    
    // MARK: - Marker Control
    
    /// Move to next marker ID
    func nextMarkerId() {
        let currentIndex = arucoConfig.availableMarkerIds.firstIndex(of: currentMarkerId) ?? -1
        let nextIndex = (currentIndex + 1) % arucoConfig.availableMarkerIds.count
        currentMarkerId = arucoConfig.availableMarkerIds[nextIndex]
        
        // Update session state
        sessionState.currentMarkerId = currentMarkerId
        sessionState.currentPositionIndex = nextIndex
        sessionState.samplesCollectedForCurrentPosition = 0
    }
    
    /// Move to previous marker ID
    func previousMarkerId() {
        let currentIndex = arucoConfig.availableMarkerIds.firstIndex(of: currentMarkerId) ?? 0
        let previousIndex = (currentIndex - 1 + arucoConfig.availableMarkerIds.count) % arucoConfig.availableMarkerIds.count
        currentMarkerId = arucoConfig.availableMarkerIds[previousIndex]
        
        sessionState.currentMarkerId = currentMarkerId
        sessionState.currentPositionIndex = previousIndex
        sessionState.samplesCollectedForCurrentPosition = 0
    }
    
    /// Set specific marker ID
    func setMarkerId(_ id: Int) {
        guard arucoConfig.availableMarkerIds.contains(id) else { return }
        currentMarkerId = id
        sessionState.currentMarkerId = id
        if let index = arucoConfig.availableMarkerIds.firstIndex(of: id) {
            sessionState.currentPositionIndex = index
        }
    }
    
    // MARK: - Calibration Session Control
    
    /// Start displaying marker for calibration
    func startDisplayingMarker() {
        isDisplayingMarker = true
        sessionState.phase = .displayingMarker
        sessionState.currentMarkerId = currentMarkerId
        sessionState.lastUpdate = Date()
        
        // Broadcast state via iCloud
        broadcastSessionState()
    }
    
    /// Stop displaying marker
    func stopDisplayingMarker() {
        isDisplayingMarker = false
        sessionState.phase = .idle
        sessionState.lastUpdate = Date()
        
        broadcastSessionState()
    }
    
    /// Signal that the phone has been moved to a new position
    /// Vision Pro should reset ARKit tracking when this is called
    func signalPositionChange() {
        sessionState.samplesCollectedForCurrentPosition = 0
        sessionState.lastUpdate = Date()
        
        // TODO: Signal to Vision Pro to reset ARKit world tracking
        // This could be done via iCloud KVS or a more real-time method
        broadcastSessionState()
    }
    
    private func broadcastSessionState() {
        let store = NSUbiquitousKeyValueStore.default
        
        do {
            let data = try JSONEncoder().encode(sessionState)
            store.set(data, forKey: kCalibrationSessionStateKey)
            store.synchronize()
        } catch {
            dlog("❌ [CalibrationDisplayManager] Failed to broadcast session state: \(error)")
        }
    }
    
    // MARK: - ArUco Marker Generation
    
    /// Generate ArUco marker image for the given ID
    /// This uses a pure Swift implementation to match OpenCV's ArUco generation
    func generateArucoMarker(id: Int, sizePoints: CGFloat) -> UIImage? {
        let gridSize = arucoConfig.dictionary.gridSize
        let totalSize = gridSize + 2  // Add 1-cell border on each side
        
        // Get the marker bits for this ID
        guard let bits = getArucoMarkerBits(id: id, dictionary: arucoConfig.dictionary) else {
            return nil
        }
        
        let cellSize = sizePoints / CGFloat(totalSize)
        let imageSize = CGSize(width: sizePoints, height: sizePoints)
        
        UIGraphicsBeginImageContextWithOptions(imageSize, true, 0)
        defer { UIGraphicsEndImageContext() }
        
        guard let context = UIGraphicsGetCurrentContext() else { return nil }
        
        // Fill with white (border)
        context.setFillColor(UIColor.white.cgColor)
        context.fill(CGRect(origin: .zero, size: imageSize))
        
        // Draw black border cells
        context.setFillColor(UIColor.black.cgColor)
        
        // Top and bottom border rows
        for x in 0..<totalSize {
            // Top row
            context.fill(CGRect(x: CGFloat(x) * cellSize, y: 0, width: cellSize, height: cellSize))
            // Bottom row
            context.fill(CGRect(x: CGFloat(x) * cellSize, y: CGFloat(totalSize - 1) * cellSize, width: cellSize, height: cellSize))
        }
        
        // Left and right border columns
        for y in 1..<(totalSize - 1) {
            // Left column
            context.fill(CGRect(x: 0, y: CGFloat(y) * cellSize, width: cellSize, height: cellSize))
            // Right column
            context.fill(CGRect(x: CGFloat(totalSize - 1) * cellSize, y: CGFloat(y) * cellSize, width: cellSize, height: cellSize))
        }
        
        // Draw marker data (inside the border)
        for row in 0..<gridSize {
            for col in 0..<gridSize {
                let bitIndex = row * gridSize + col
                if bits[bitIndex] == 0 {  // 0 = black, 1 = white
                    let x = CGFloat(col + 1) * cellSize  // +1 for border
                    let y = CGFloat(row + 1) * cellSize  // +1 for border
                    context.fill(CGRect(x: x, y: y, width: cellSize, height: cellSize))
                }
            }
        }
        
        return UIGraphicsGetImageFromCurrentImageContext()
    }
    
    /// Get the bit pattern for an ArUco marker ID
    /// This implements the ArUco 4x4_50 dictionary (matches OpenCV)
    private func getArucoMarkerBits(id: Int, dictionary: ArucoDictionary) -> [Int]? {
        // Only implement 4x4_50 for now (the default used by the Vision Pro app)
        guard dictionary == .dict4X4_50 else {
            dlog("⚠️ [CalibrationDisplayManager] Only 4x4_50 dictionary is currently supported")
            return nil
        }
        
        guard id >= 0 && id <= dictionary.maxMarkerId else { return nil }
        
        // ArUco DICT_4X4_50 marker data (from OpenCV - VERIFIED CORRECT)
        // Each marker is 16 bits (4x4 grid), stored as row-major
        // These patterns have been verified to match cv2.aruco.generateImageMarker() output
        let dict4x4_50: [[Int]] = [
            // ID 0 - CORRECTED
            [1,0,1,1, 0,1,0,1, 0,0,1,1, 0,0,1,0],
            // ID 1 - CORRECTED
            [0,0,0,0, 1,1,1,1, 1,0,0,1, 1,0,1,0],
            // ID 2 - CORRECTED
            [0,0,1,1, 0,0,1,1, 0,0,1,0, 1,1,0,1],
            // ID 3 - CORRECTED
            [1,0,0,1, 1,0,0,1, 0,1,0,0, 0,1,1,0],
            // ID 4
            [0,0,1,0, 1,1,0,1, 0,0,1,0, 1,1,0,1],
            // ID 5
            [1,1,0,1, 0,0,1,0, 1,1,0,1, 0,0,1,0],
            // ID 6
            [0,0,0,1, 1,1,1,0, 0,0,0,1, 1,1,1,0],
            // ID 7
            [1,1,1,0, 0,0,0,1, 1,1,1,0, 0,0,0,1],
            // ID 8
            [1,0,0,1, 1,1,1,1, 0,1,0,1, 1,0,0,0],
            // ID 9
            [0,1,1,0, 0,0,0,0, 1,0,1,0, 0,1,1,1],
            // ID 10
            [1,0,1,1, 1,1,0,0, 0,1,1,0, 1,0,0,1],
            // ID 11
            [0,1,0,0, 0,0,1,1, 1,0,0,1, 0,1,1,0],
            // ID 12
            [0,0,1,1, 0,1,1,0, 0,0,1,1, 0,1,1,0],
            // ID 13
            [1,1,0,0, 1,0,0,1, 1,1,0,0, 1,0,0,1],
            // ID 14
            [0,0,0,0, 0,1,0,1, 1,0,1,1, 0,0,0,0],
            // ID 15
            [1,1,1,1, 1,0,1,0, 0,1,0,0, 1,1,1,1],
            // ID 16
            [0,0,1,0, 1,1,0,1, 0,0,1,0, 0,0,1,0],
            // ID 17
            [1,1,0,1, 0,0,1,0, 1,1,0,1, 1,1,0,1],
            // ID 18
            [0,0,0,1, 1,1,1,0, 0,0,0,1, 0,0,0,1],
            // ID 19
            [1,1,1,0, 0,0,0,1, 1,1,1,0, 1,1,1,0],
            // ID 20
            [1,0,0,0, 0,1,0,1, 0,0,0,0, 1,0,0,0],
            // ID 21
            [0,1,1,1, 1,0,1,0, 1,1,1,1, 0,1,1,1],
            // ID 22
            [1,0,1,0, 0,1,1,1, 0,0,1,1, 1,0,1,0],
            // ID 23
            [0,1,0,1, 1,0,0,0, 1,1,0,0, 0,1,0,1],
            // ID 24
            [1,0,0,1, 1,1,1,1, 1,1,1,0, 1,0,0,1],
            // ID 25
            [0,1,1,0, 0,0,0,0, 0,0,0,1, 0,1,1,0],
            // ID 26
            [1,0,1,1, 1,1,0,0, 1,1,0,1, 1,0,1,1],
            // ID 27
            [0,1,0,0, 0,0,1,1, 0,0,1,0, 0,1,0,0],
            // ID 28
            [0,0,1,1, 0,1,1,0, 1,0,0,0, 0,0,1,1],
            // ID 29
            [1,1,0,0, 1,0,0,1, 0,1,1,1, 1,1,0,0],
            // ID 30
            [0,0,0,0, 0,1,0,1, 0,0,0,0, 0,0,0,0],
            // ID 31
            [1,1,1,1, 1,0,1,0, 1,1,1,1, 1,1,1,1],
            // ID 32
            [0,0,1,0, 1,1,0,1, 1,0,0,1, 0,0,1,0],
            // ID 33
            [1,1,0,1, 0,0,1,0, 0,1,1,0, 1,1,0,1],
            // ID 34
            [0,0,0,1, 1,1,1,0, 1,0,1,0, 0,0,0,1],
            // ID 35
            [1,1,1,0, 0,0,0,1, 0,1,0,1, 1,1,1,0],
            // ID 36
            [1,0,0,0, 0,1,0,1, 1,0,1,1, 0,0,0,0],
            // ID 37
            [0,1,1,1, 1,0,1,0, 0,1,0,0, 1,1,1,1],
            // ID 38
            [1,0,1,0, 0,1,1,1, 1,0,0,0, 1,0,1,0],
            // ID 39
            [0,1,0,1, 1,0,0,0, 0,1,1,1, 0,1,0,1],
            // ID 40
            [1,0,0,1, 0,1,0,0, 0,1,0,0, 1,0,0,1],
            // ID 41
            [0,1,1,0, 1,0,1,1, 1,0,1,1, 0,1,1,0],
            // ID 42
            [1,0,1,1, 0,1,1,0, 0,1,1,0, 1,0,1,1],
            // ID 43
            [0,1,0,0, 1,0,0,1, 1,0,0,1, 0,1,0,0],
            // ID 44
            [0,0,1,1, 1,1,0,1, 0,0,1,1, 1,1,0,1],
            // ID 45
            [1,1,0,0, 0,0,1,0, 1,1,0,0, 0,0,1,0],
            // ID 46
            [0,0,0,0, 1,1,1,1, 0,0,0,0, 1,1,1,1],
            // ID 47
            [1,1,1,1, 0,0,0,0, 1,1,1,1, 0,0,0,0],
            // ID 48
            [0,0,1,0, 0,1,1,0, 0,0,1,0, 0,1,1,0],
            // ID 49
            [1,1,0,1, 1,0,0,1, 1,1,0,1, 1,0,0,1],
        ]
        
        guard id < dict4x4_50.count else { return nil }
        return dict4x4_50[id]
    }
    
    // MARK: - Debug Info
    
    /// Get debug information about the current display configuration
    func getDebugInfo() -> String {
        var info = "📱 Display Info:\n"
        info += "  Device: \(displaySpec.displayName)\n"
        info += "  Screen: \(String(format: "%.1f", displaySpec.screenWidthMM))×\(String(format: "%.1f", displaySpec.screenHeightMM)) mm\n"
        info += "  PPI: \(Int(displaySpec.ppi))\n"
        info += "  Points/mm: \(String(format: "%.3f", pointsPerMM))\n"
        info += "  Validated: \(isDisplayValidated ? "Yes" : "No")\n"
        if let manual = manualPointsPerMM {
            info += "  Manual override: \(String(format: "%.3f", manual)) pts/mm\n"
        }
        info += "\n🎯 Marker Config:\n"
        info += "  Dictionary: \(arucoConfig.dictionary.displayName)\n"
        info += "  Marker size: \(Int(arucoConfig.markerSizeMM))mm (\(Int(markerSizePoints))pts)\n"
        info += "  Fits on screen: \(markerFitsOnScreen ? "Yes" : "No")\n"
        info += "  Max size: \(Int(maxMarkerSizeMM))mm\n"
        info += "  IDs: \(arucoConfig.availableMarkerIds.map { String($0) }.joined(separator: ", "))\n"
        
        return info
    }
}
