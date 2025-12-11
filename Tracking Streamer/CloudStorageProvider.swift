//
//  CloudStorageProvider.swift
//  Tracking Streamer (visionOS)
//
//  Cloud storage provider selection, synced from iOS via iCloud Keychain.
//

import Foundation
import SwiftUI
import Combine

/// Available cloud storage providers for recordings
enum CloudStorageProvider: String, CaseIterable, Codable, Identifiable {
    case iCloudDrive = "icloud"
    case dropbox = "dropbox"
    case googleDrive = "google_drive"
    
    var id: String { rawValue }
    
    var displayName: String {
        switch self {
        case .iCloudDrive: return "iCloud Drive"
        case .dropbox: return "Dropbox"
        case .googleDrive: return "Google Drive"
        }
    }
    
    var icon: String {
        switch self {
        case .iCloudDrive: return "icloud.fill"
        case .dropbox: return "shippingbox.fill"
        case .googleDrive: return "externaldrive.fill"
        }
    }
    
    var color: Color {
        switch self {
        case .iCloudDrive: return .blue
        case .dropbox: return Color(red: 0, green: 0.4, blue: 1)
        case .googleDrive: return Color(red: 0.26, green: 0.52, blue: 0.96)
        }
    }
}

/// Reads cloud storage settings from iCloud Keychain (set by iOS companion app)
@MainActor
class CloudStorageSettings: ObservableObject {
    static let shared = CloudStorageSettings()
    
    // MARK: - Published Properties
    
    @Published private(set) var selectedProvider: CloudStorageProvider = .iCloudDrive
    @Published private(set) var isDropboxAvailable: Bool = false
    @Published private(set) var isGoogleDriveAvailable: Bool = false
    @Published private(set) var lastSyncTime: Date?
    @Published private(set) var debugInfo: String = ""
    
    private let keychain = KeychainManager.shared
    private var cancellables = Set<AnyCancellable>()
    
    private init() {
        loadSettings()
        setupForegroundObserver()
    }
    
    // MARK: - Foreground Observer
    
    private func setupForegroundObserver() {
        // Reload settings when app comes to foreground (keychain may have synced)
        NotificationCenter.default.publisher(for: NSNotification.Name("NSApplicationDidBecomeActiveNotification"))
            .sink { [weak self] _ in
                Task { @MainActor in
                    dlog("☁️ [CloudStorageSettings] App became active, refreshing settings...")
                    self?.loadSettings()
                }
            }
            .store(in: &cancellables)
        
        // Also listen for scene phase changes
        NotificationCenter.default.publisher(for: NSNotification.Name("ScenePhaseDidChange"))
            .sink { [weak self] _ in
                Task { @MainActor in
                    self?.loadSettings()
                }
            }
            .store(in: &cancellables)
    }
    
    // MARK: - Public Methods
    
    /// Reload settings from keychain (call when app comes to foreground)
    func loadSettings() {
        dlog("☁️ [CloudStorageSettings] ==========================================")
        dlog("☁️ [CloudStorageSettings] Loading settings from iCloud Keychain...")
        
        // Debug: Check all keys
        let providerValue = keychain.loadString(forKey: .selectedCloudProvider)
        let hasDropboxAccessToken = keychain.exists(key: .dropboxAccessToken)
        let hasDropboxRefreshToken = keychain.exists(key: .dropboxRefreshToken)
        let hasGoogleDriveAccessToken = keychain.exists(key: .googleDriveAccessToken)
        let hasGoogleDriveRefreshToken = keychain.exists(key: .googleDriveRefreshToken)
        
        dlog("☁️ [CloudStorageSettings] Raw keychain values:")
        dlog("☁️   - selectedCloudProvider: \(providerValue ?? "nil")")
        dlog("☁️   - dropboxAccessToken exists: \(hasDropboxAccessToken)")
        dlog("☁️   - dropboxRefreshToken exists: \(hasDropboxRefreshToken)")
        dlog("☁️   - googleDriveAccessToken exists: \(hasGoogleDriveAccessToken)")
        dlog("☁️   - googleDriveRefreshToken exists: \(hasGoogleDriveRefreshToken)")
        
        // Load selected provider
        if let providerString = providerValue,
           let provider = CloudStorageProvider(rawValue: providerString) {
            selectedProvider = provider
            dlog("☁️ [CloudStorageSettings] ✅ Provider from keychain: \(provider.displayName)")
        } else {
            selectedProvider = .iCloudDrive
            dlog("☁️ [CloudStorageSettings] ⚠️ No provider in keychain, using iCloud Drive")
        }
        
        // Check if Dropbox is configured
        isDropboxAvailable = hasDropboxAccessToken
        
        // Check if Google Drive is configured
        isGoogleDriveAvailable = hasGoogleDriveAccessToken
        
        lastSyncTime = Date()
        
        // Build debug info string
        debugInfo = """
        Provider: \(selectedProvider.displayName)
        Dropbox available: \(isDropboxAvailable)
        Dropbox access token: \(hasDropboxAccessToken ? "✓" : "✗")
        Dropbox refresh token: \(hasDropboxRefreshToken ? "✓" : "✗")
        Google Drive available: \(isGoogleDriveAvailable)
        Google Drive access token: \(hasGoogleDriveAccessToken ? "✓" : "✗")
        Google Drive refresh token: \(hasGoogleDriveRefreshToken ? "✓" : "✗")
        Last sync: \(lastSyncTime?.formatted() ?? "never")
        """
        
        dlog("☁️ [CloudStorageSettings] ==========================================")
        dlog("☁️ [CloudStorageSettings] Final: Provider=\(selectedProvider.displayName), Dropbox=\(isDropboxAvailable), GoogleDrive=\(isGoogleDriveAvailable)")
        
        // Post notification for RecordingManager
        NotificationCenter.default.post(name: .cloudStorageSettingsDidChange, object: nil)
    }
    
    /// Get the currently configured provider, validating Dropbox/Google Drive availability
    func getActiveProvider() -> CloudStorageProvider {
        if selectedProvider == .dropbox && !isDropboxAvailable {
            dlog("⚠️ [CloudStorageSettings] Dropbox selected but not available, falling back to iCloud")
            return .iCloudDrive
        }
        if selectedProvider == .googleDrive && !isGoogleDriveAvailable {
            dlog("⚠️ [CloudStorageSettings] Google Drive selected but not available, falling back to iCloud")
            return .iCloudDrive
        }
        return selectedProvider
    }
    
    /// Force refresh settings (call from UI)
    func forceRefresh() {
        dlog("☁️ [CloudStorageSettings] Force refresh requested")
        loadSettings()
    }
}

// MARK: - Notifications

extension Notification.Name {
    static let cloudStorageSettingsDidChange = Notification.Name("cloudStorageSettingsDidChange")
}
