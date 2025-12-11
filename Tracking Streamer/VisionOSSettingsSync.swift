//
//  VisionOSSettingsSync.swift
//  Tracking Streamer (visionOS)
//
//  Syncs settings from iOS via iCloud NSUbiquitousKeyValueStore.
//  Reads settings set by the iOS companion app.
//

import Foundation
import Combine

/// Manages settings synced from iOS via iCloud
@MainActor
class VisionOSSettingsSync: ObservableObject {
    static let shared = VisionOSSettingsSync()
    
    // MARK: - Setting Keys (must match iOS)
    
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
    
    // MARK: - Published Properties
    
    @Published private(set) var lastSyncTime: Date?
    @Published private(set) var hasRemoteSettings: Bool = false
    
    // MARK: - Private Properties
    
    private let store = NSUbiquitousKeyValueStore.default
    private var cancellables = Set<AnyCancellable>()
    private weak var dataManager: DataManager?
    private weak var recordingManager: RecordingManager?
    
    // MARK: - Initialization
    
    private init() {
        setupiCloudObserver()
    }
    
    // MARK: - Configuration
    
    /// Call this after DataManager and RecordingManager are initialized
    func configure(dataManager: DataManager, recordingManager: RecordingManager) {
        self.dataManager = dataManager
        self.recordingManager = recordingManager
        
        // Initial load
        loadFromiCloud()
    }
    
    // MARK: - iCloud Sync
    
    private func setupiCloudObserver() {
        // Listen for external changes from iOS
        NotificationCenter.default.publisher(for: NSUbiquitousKeyValueStore.didChangeExternallyNotification)
            .receive(on: DispatchQueue.main)
            .sink { [weak self] notification in
                guard let self = self else { return }
                
                if let userInfo = notification.userInfo,
                   let reason = userInfo[NSUbiquitousKeyValueStoreChangeReasonKey] as? Int {
                    switch reason {
                    case NSUbiquitousKeyValueStoreServerChange,
                         NSUbiquitousKeyValueStoreInitialSyncChange:
                        dlog("☁️ [VisionOSSettingsSync] Received settings update from iOS")
                        self.loadFromiCloud()
                    case NSUbiquitousKeyValueStoreAccountChange:
                        dlog("ℹ️ [VisionOSSettingsSync] iCloud account changed")
                        self.loadFromiCloud()
                    default:
                        break
                    }
                }
            }
            .store(in: &cancellables)
        
        // Start sync
        store.synchronize()
    }
    
    private func loadFromiCloud() {
        dlog("☁️ [VisionOSSettingsSync] Loading settings from iCloud...")
        
        guard let dataManager = dataManager else {
            dlog("⚠️ [VisionOSSettingsSync] DataManager not configured yet")
            return
        }
        
        var settingsApplied = false
        
        // Visualization settings
        if let value = store.object(forKey: SettingKey.upperLimbVisible.rawValue) as? Bool {
            dataManager.upperLimbVisible = value
            settingsApplied = true
            dlog("☁️ [VisionOSSettingsSync] Applied upperLimbVisible = \(value)")
        }
        
        if let value = store.object(forKey: SettingKey.showHeadBeam.rawValue) as? Bool {
            dataManager.showHeadBeam = value
            settingsApplied = true
            dlog("☁️ [VisionOSSettingsSync] Applied showHeadBeam = \(value)")
        }
        
        if let value = store.object(forKey: SettingKey.showHandJoints.rawValue) as? Bool {
            dataManager.showHandJoints = value
            settingsApplied = true
            dlog("☁️ [VisionOSSettingsSync] Applied showHandJoints = \(value)")
        }
        
        if let value = store.object(forKey: SettingKey.handJointsOpacity.rawValue) as? Double {
            dataManager.handJointsOpacity = Float(value)
            settingsApplied = true
            dlog("☁️ [VisionOSSettingsSync] Applied handJointsOpacity = \(value)")
        }
        
        // Video plane settings
        if let value = store.object(forKey: SettingKey.videoPlaneZDistance.rawValue) as? Double {
            dataManager.videoPlaneZDistance = Float(value)
            settingsApplied = true
            dlog("☁️ [VisionOSSettingsSync] Applied videoPlaneZDistance = \(value)")
        }
        
        if let value = store.object(forKey: SettingKey.videoPlaneYPosition.rawValue) as? Double {
            dataManager.videoPlaneYPosition = Float(value)
            settingsApplied = true
            dlog("☁️ [VisionOSSettingsSync] Applied videoPlaneYPosition = \(value)")
        }
        
        // Controller position settings
        if let value = store.object(forKey: SettingKey.statusMinimizedXPosition.rawValue) as? Double {
            dataManager.statusMinimizedXPosition = Float(value)
            settingsApplied = true
            dlog("☁️ [VisionOSSettingsSync] Applied statusMinimizedXPosition = \(value)")
        }
        
        if let value = store.object(forKey: SettingKey.statusMinimizedYPosition.rawValue) as? Double {
            dataManager.statusMinimizedYPosition = Float(value)
            settingsApplied = true
            dlog("☁️ [VisionOSSettingsSync] Applied statusMinimizedYPosition = \(value)")
        }
        
        // Recording settings
        if let value = store.object(forKey: SettingKey.autoRecordingEnabled.rawValue) as? Bool {
            recordingManager?.autoRecordingEnabled = value
            settingsApplied = true
            dlog("☁️ [VisionOSSettingsSync] Applied autoRecordingEnabled = \(value)")
        }
        
        // App mode - stored in UserDefaults with @AppStorage
        if let value = store.object(forKey: SettingKey.appMode.rawValue) as? String {
            UserDefaults.standard.set(value, forKey: "appMode")
            settingsApplied = true
            dlog("☁️ [VisionOSSettingsSync] Applied appMode = \(value)")
        }
        
        // Update sync status
        if let timestamp = store.object(forKey: SettingKey.lastSyncTime.rawValue) as? Double {
            lastSyncTime = Date(timeIntervalSince1970: timestamp)
        }
        
        hasRemoteSettings = settingsApplied
        
        if settingsApplied {
            dlog("☁️ [VisionOSSettingsSync] ✅ Settings applied from iOS")
        } else {
            dlog("☁️ [VisionOSSettingsSync] No remote settings found")
        }
    }
    
    // MARK: - Public Methods
    
    /// Force refresh from iCloud
    func forceRefresh() {
        store.synchronize()
        loadFromiCloud()
    }
    
    /// Push current visionOS settings to iCloud (for bidirectional sync)
    func pushCurrentSettings() {
        guard let dataManager = dataManager else { return }
        
        // Visualization settings
        store.set(dataManager.upperLimbVisible, forKey: SettingKey.upperLimbVisible.rawValue)
        store.set(dataManager.showHeadBeam, forKey: SettingKey.showHeadBeam.rawValue)
        store.set(dataManager.showHandJoints, forKey: SettingKey.showHandJoints.rawValue)
        store.set(Double(dataManager.handJointsOpacity), forKey: SettingKey.handJointsOpacity.rawValue)
        
        // Video plane settings
        store.set(Double(dataManager.videoPlaneZDistance), forKey: SettingKey.videoPlaneZDistance.rawValue)
        store.set(Double(dataManager.videoPlaneYPosition), forKey: SettingKey.videoPlaneYPosition.rawValue)
        
        // Controller position settings
        store.set(Double(dataManager.statusMinimizedXPosition), forKey: SettingKey.statusMinimizedXPosition.rawValue)
        store.set(Double(dataManager.statusMinimizedYPosition), forKey: SettingKey.statusMinimizedYPosition.rawValue)
        
        // Recording settings
        if let recordingManager = recordingManager {
            store.set(recordingManager.autoRecordingEnabled, forKey: SettingKey.autoRecordingEnabled.rawValue)
        }
        
        // App mode
        if let appMode = UserDefaults.standard.string(forKey: "appMode") {
            store.set(appMode, forKey: SettingKey.appMode.rawValue)
        }
        
        // Timestamp
        store.set(Date().timeIntervalSince1970, forKey: SettingKey.lastSyncTime.rawValue)
        
        store.synchronize()
        dlog("☁️ [VisionOSSettingsSync] Pushed current settings to iCloud")
    }
}
