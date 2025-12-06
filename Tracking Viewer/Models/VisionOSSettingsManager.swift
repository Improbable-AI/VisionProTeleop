//
//  VisionOSSettingsManager.swift
//  Tracking Viewer
//
//  Manages visionOS settings that can be configured from iOS and synced via iCloud.
//  Uses NSUbiquitousKeyValueStore for cross-device sync (up to 1MB, 1024 keys max).
//

import Foundation
import SwiftUI
import Combine

/// Settings for visionOS that can be configured from iOS
@MainActor
class VisionOSSettingsManager: ObservableObject {
    static let shared = VisionOSSettingsManager()
    
    // MARK: - Setting Keys (must match visionOS)
    
    private enum SettingKey: String {
        // Visualization settings
        case upperLimbVisible = "visionos.upperLimbVisible"
        case showHeadBeam = "visionos.showHeadBeam"
        case showHandJoints = "visionos.showHandJoints"
        case handJointsOpacity = "visionos.handJointsOpacity"
        
        // Video plane settings
        case videoPlaneZDistance = "visionos.videoPlaneZDistance"
        case videoPlaneYPosition = "visionos.videoPlaneYPosition"
        
        // Controller position settings
        case statusMinimizedXPosition = "visionos.statusMinimizedXPosition"
        case statusMinimizedYPosition = "visionos.statusMinimizedYPosition"
        
        // Recording settings
        case autoRecordingEnabled = "visionos.autoRecordingEnabled"
        
        // App mode
        case appMode = "visionos.appMode"
        
        // Last sync timestamp
        case lastSyncTime = "visionos.lastSyncTime"
    }
    
    // MARK: - Published Properties (Visualization)
    
    /// Whether hands are rendered on top of AR content
    @Published var upperLimbVisible: Bool = true {
        didSet { saveToiCloud(.upperLimbVisible, value: upperLimbVisible) }
    }
    
    /// Whether to show head gaze ray
    @Published var showHeadBeam: Bool = false {
        didSet { saveToiCloud(.showHeadBeam, value: showHeadBeam) }
    }
    
    /// Whether to show hand tracking joint spheres
    @Published var showHandJoints: Bool = false {
        didSet { saveToiCloud(.showHandJoints, value: showHandJoints) }
    }
    
    /// Opacity of hand joint spheres (0.0 - 1.0)
    @Published var handJointsOpacity: Float = 0.9 {
        didSet { saveToiCloud(.handJointsOpacity, value: handJointsOpacity) }
    }
    
    // MARK: - Published Properties (Video Plane)
    
    /// Distance of video plane from user (2.0 - 20.0 meters)
    @Published var videoPlaneZDistance: Float = 10.0 {
        didSet { saveToiCloud(.videoPlaneZDistance, value: videoPlaneZDistance) }
    }
    
    /// Vertical offset of video plane (-2.0 - 2.0 meters)
    @Published var videoPlaneYPosition: Float = 0.0 {
        didSet { saveToiCloud(.videoPlaneYPosition, value: videoPlaneYPosition) }
    }
    
    // MARK: - Published Properties (Controller Position)
    
    /// Horizontal position of minimized controller (-0.5 - 0.5)
    @Published var statusMinimizedXPosition: Float = 0.0 {
        didSet { saveToiCloud(.statusMinimizedXPosition, value: statusMinimizedXPosition) }
    }
    
    /// Vertical position of minimized controller (-0.5 - 0.5)
    @Published var statusMinimizedYPosition: Float = -0.3 {
        didSet { saveToiCloud(.statusMinimizedYPosition, value: statusMinimizedYPosition) }
    }
    
    // MARK: - Published Properties (Recording)
    
    /// Whether to auto-start recording when connected
    @Published var autoRecordingEnabled: Bool = true {
        didSet { saveToiCloud(.autoRecordingEnabled, value: autoRecordingEnabled) }
    }
    
    // MARK: - Published Properties (App Mode)
    
    /// App mode: teleop or egorecord
    @Published var appMode: String = "teleop" {
        didSet { saveToiCloud(.appMode, value: appMode) }
    }
    
    // MARK: - Sync Status
    
    @Published private(set) var lastSyncTime: Date?
    @Published private(set) var isSyncing: Bool = false
    
    // MARK: - Private Properties
    
    private let store = NSUbiquitousKeyValueStore.default
    private var cancellables = Set<AnyCancellable>()
    private var isLoadingFromiCloud = false
    
    // MARK: - Initialization
    
    private init() {
        loadFromiCloud()
        setupiCloudObserver()
    }
    
    // MARK: - iCloud Sync
    
    private func setupiCloudObserver() {
        // Listen for external changes from other devices (bidirectional sync with visionOS)
        NotificationCenter.default.publisher(for: NSUbiquitousKeyValueStore.didChangeExternallyNotification)
            .receive(on: DispatchQueue.main)
            .sink { [weak self] notification in
                guard let self = self else { return }
                
                // Check what changed
                if let userInfo = notification.userInfo,
                   let reason = userInfo[NSUbiquitousKeyValueStoreChangeReasonKey] as? Int {
                    switch reason {
                    case NSUbiquitousKeyValueStoreServerChange,
                         NSUbiquitousKeyValueStoreInitialSyncChange:
                        print("☁️ [VisionOSSettings] Received external changes from iCloud (possibly from Vision Pro)")
                        self.loadFromiCloud()
                    case NSUbiquitousKeyValueStoreQuotaViolationChange:
                        print("⚠️ [VisionOSSettings] iCloud quota exceeded")
                    case NSUbiquitousKeyValueStoreAccountChange:
                        print("ℹ️ [VisionOSSettings] iCloud account changed")
                        self.loadFromiCloud()
                    default:
                        break
                    }
                }
            }
            .store(in: &cancellables)
        
        // Also listen for app foreground to refresh
        NotificationCenter.default.publisher(for: UIApplication.willEnterForegroundNotification)
            .sink { [weak self] _ in
                self?.forceRefresh()
            }
            .store(in: &cancellables)
        
        // Start sync
        store.synchronize()
    }
    
    private func loadFromiCloud() {
        isLoadingFromiCloud = true
        defer { isLoadingFromiCloud = false }
        
        print("☁️ [VisionOSSettings] Loading settings from iCloud...")
        
        // Visualization settings
        if let value = store.object(forKey: SettingKey.upperLimbVisible.rawValue) as? Bool {
            upperLimbVisible = value
        }
        if let value = store.object(forKey: SettingKey.showHeadBeam.rawValue) as? Bool {
            showHeadBeam = value
        }
        if let value = store.object(forKey: SettingKey.showHandJoints.rawValue) as? Bool {
            showHandJoints = value
        }
        if let value = store.object(forKey: SettingKey.handJointsOpacity.rawValue) as? Double {
            handJointsOpacity = Float(value)
        }
        
        // Video plane settings
        if let value = store.object(forKey: SettingKey.videoPlaneZDistance.rawValue) as? Double {
            videoPlaneZDistance = Float(value)
        }
        if let value = store.object(forKey: SettingKey.videoPlaneYPosition.rawValue) as? Double {
            videoPlaneYPosition = Float(value)
        }
        
        // Controller position settings
        if let value = store.object(forKey: SettingKey.statusMinimizedXPosition.rawValue) as? Double {
            statusMinimizedXPosition = Float(value)
        }
        if let value = store.object(forKey: SettingKey.statusMinimizedYPosition.rawValue) as? Double {
            statusMinimizedYPosition = Float(value)
        }
        
        // Recording settings
        if let value = store.object(forKey: SettingKey.autoRecordingEnabled.rawValue) as? Bool {
            autoRecordingEnabled = value
        }
        
        // App mode
        if let value = store.object(forKey: SettingKey.appMode.rawValue) as? String {
            appMode = value
        }
        
        // Last sync time
        if let timestamp = store.object(forKey: SettingKey.lastSyncTime.rawValue) as? Double {
            lastSyncTime = Date(timeIntervalSince1970: timestamp)
        }
        
        print("☁️ [VisionOSSettings] Loaded: hands=\(upperLimbVisible), headBeam=\(showHeadBeam), handJoints=\(showHandJoints)")
    }
    
    private func saveToiCloud<T>(_ key: SettingKey, value: T) {
        guard !isLoadingFromiCloud else { return }
        
        store.set(value, forKey: key.rawValue)
        store.set(Date().timeIntervalSince1970, forKey: SettingKey.lastSyncTime.rawValue)
        
        // Trigger sync
        store.synchronize()
        lastSyncTime = Date()
        
        print("☁️ [VisionOSSettings] Saved \(key.rawValue) = \(value)")
    }
    
    // MARK: - Public Methods
    
    /// Force refresh from iCloud
    func forceRefresh() {
        isSyncing = true
        store.synchronize()
        loadFromiCloud()
        isSyncing = false
    }
    
    /// Reset all settings to defaults
    func resetToDefaults() {
        upperLimbVisible = true
        showHeadBeam = false
        showHandJoints = false
        handJointsOpacity = 0.9
        videoPlaneZDistance = 10.0
        videoPlaneYPosition = 0.0
        statusMinimizedXPosition = 0.0
        statusMinimizedYPosition = -0.3
        autoRecordingEnabled = true
        appMode = "teleop"
    }
}

// MARK: - App Mode Enum (for display)

enum VisionOSAppMode: String, CaseIterable, Identifiable {
    case teleop = "teleop"
    case egorecord = "egorecord"
    
    var id: String { rawValue }
    
    var displayName: String {
        switch self {
        case .teleop: return "Teleoperation"
        case .egorecord: return "EgoRecord"
        }
    }
    
    var description: String {
        switch self {
        case .teleop: return "Stream hand/head tracking to a Python client"
        case .egorecord: return "Record egocentric video with USB camera"
        }
    }
    
    var icon: String {
        switch self {
        case .teleop: return "network"
        case .egorecord: return "video.badge.plus"
        }
    }
}
