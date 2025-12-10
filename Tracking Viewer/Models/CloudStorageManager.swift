//
//  CloudStorageManager.swift
//  Tracking Viewer
//
//  Manages cloud storage provider selection and syncs settings via iCloud Keychain.
//  This allows users to configure their preferred cloud storage on iOS, 
//  and the visionOS app will automatically use the same settings.
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
        case .dropbox: return "shippingbox.fill" // Dropbox-like icon
        case .googleDrive: return "externaldrive.fill" // Google Drive-like icon
        }
    }
    
    var description: String {
        switch self {
        case .iCloudDrive: return "Recordings sync across all your Apple devices"
        case .dropbox: return "Recordings upload to your Dropbox account"
        case .googleDrive: return "Recordings upload to your Google Drive"
        }
    }
    
    var color: Color {
        switch self {
        case .iCloudDrive: return .blue
        case .dropbox: return Color(red: 0, green: 0.4, blue: 1) // Dropbox blue
        case .googleDrive: return Color(red: 0.26, green: 0.52, blue: 0.96) // Google blue
        }
    }
}

/// Manages cloud storage provider selection and credentials
@MainActor
class CloudStorageManager: ObservableObject {
    static let shared = CloudStorageManager()
    
    // MARK: - Published Properties
    
    /// Currently selected cloud storage provider
    @Published private(set) var selectedProvider: CloudStorageProvider = .iCloudDrive
    
    /// Whether Dropbox is authenticated
    @Published private(set) var isDropboxAuthenticated: Bool = false
    
    /// Whether Google Drive is authenticated
    @Published private(set) var isGoogleDriveAuthenticated: Bool = false
    
    /// Dropbox account info (email/name) when authenticated
    @Published private(set) var dropboxAccountInfo: String?
    
    /// Google Drive account info (email/name) when authenticated
    @Published private(set) var googleDriveAccountInfo: String?
    
    /// Whether settings are being synced
    @Published private(set) var isSyncing: Bool = false
    
    /// Error message if any operation fails
    @Published var errorMessage: String?
    
    // MARK: - Private Properties
    
    private let keychain = KeychainManager.shared
    private var cancellables = Set<AnyCancellable>()
    
    // MARK: - Initialization
    
    private init() {
        loadSettings()
        
        // Observe changes and sync
        NotificationCenter.default.publisher(for: UIApplication.willEnterForegroundNotification)
            .sink { [weak self] _ in
                Task { @MainActor in
                    self?.loadSettings()
                }
            }
            .store(in: &cancellables)
    }
    
    // MARK: - Public Methods
    
    /// Load settings from keychain (called on launch and when returning from background)
    func loadSettings() {
        // Load selected provider
        if let providerString = keychain.loadString(forKey: .selectedCloudProvider),
           let provider = CloudStorageProvider(rawValue: providerString) {
            selectedProvider = provider
            dlog("☁️ [CloudStorageManager] Loaded provider from keychain: \(provider.displayName)")
        } else {
            // Default to iCloud Drive
            selectedProvider = .iCloudDrive
            dlog("☁️ [CloudStorageManager] No provider saved, defaulting to iCloud Drive")
        }
        
        // Check Dropbox authentication status
        checkDropboxAuthStatus()
        
        // Check Google Drive authentication status
        checkGoogleDriveAuthStatus()
    }
    
    /// Select a cloud storage provider
    /// - Parameter provider: The provider to select
    func selectProvider(_ provider: CloudStorageProvider) {
        // For Dropbox, user must authenticate first
        if provider == .dropbox && !isDropboxAuthenticated {
            errorMessage = "Please sign in to Dropbox first"
            return
        }
        
        // For Google Drive, user must authenticate first
        if provider == .googleDrive && !isGoogleDriveAuthenticated {
            errorMessage = "Please sign in to Google Drive first"
            return
        }
        
        selectedProvider = provider
        
        // Save to keychain (syncs via iCloud Keychain)
        keychain.save(provider.rawValue, forKey: .selectedCloudProvider)
        
        dlog("☁️ [CloudStorageManager] Selected provider: \(provider.displayName)")
        
        // Post notification for other parts of the app
        NotificationCenter.default.post(name: .cloudStorageProviderChanged, object: provider)
    }
    
    /// Check if the user can use the selected provider
    func canUseSelectedProvider() -> Bool {
        switch selectedProvider {
        case .iCloudDrive:
            return true // Always available (with iCloud account)
        case .dropbox:
            return isDropboxAuthenticated
        case .googleDrive:
            return isGoogleDriveAuthenticated
        }
    }
    
    /// Get the storage path description for the selected provider
    func getStoragePathDescription() -> String {
        switch selectedProvider {
        case .iCloudDrive:
            return "iCloud Drive → VisionProTeleop"
        case .dropbox:
            return "Dropbox → Apps → VisionProTeleop"
        case .googleDrive:
            return "Google Drive → VisionProTeleop"
        }
    }
    
    // MARK: - Dropbox Authentication
    
    /// Check Dropbox authentication status from keychain
    private func checkDropboxAuthStatus() {
        if let _ = keychain.loadString(forKey: .dropboxAccessToken) {
            isDropboxAuthenticated = true
            // Load account info if available
            dropboxAccountInfo = keychain.loadString(forKey: .dropboxRefreshToken) != nil ? "Signed in" : nil
            dlog("☁️ [CloudStorageManager] Dropbox: Authenticated")
        } else {
            isDropboxAuthenticated = false
            dropboxAccountInfo = nil
            dlog("☁️ [CloudStorageManager] Dropbox: Not authenticated")
        }
    }
    
    /// Sign in to Dropbox (opens OAuth flow)
    func signInToDropbox() {
        // This will be handled by DropboxManager
        Task {
            await DropboxManager.shared.startOAuthFlow()
        }
    }
    
    /// Sign out of Dropbox
    func signOutOfDropbox() {
        // Clear tokens
        keychain.delete(key: .dropboxAccessToken)
        keychain.delete(key: .dropboxRefreshToken)
        keychain.delete(key: .dropboxTokenExpiry)
        
        isDropboxAuthenticated = false
        dropboxAccountInfo = nil
        
        // If Dropbox was selected, switch back to iCloud Drive
        if selectedProvider == .dropbox {
            selectProvider(.iCloudDrive)
        }
        
        dlog("☁️ [CloudStorageManager] Signed out of Dropbox")
    }
    
    /// Called when Dropbox OAuth completes successfully
    func onDropboxAuthSuccess(accessToken: String, refreshToken: String?, expiresIn: TimeInterval?) {
        keychain.save(accessToken, forKey: .dropboxAccessToken)
        
        if let refreshToken = refreshToken {
            keychain.save(refreshToken, forKey: .dropboxRefreshToken)
        }
        
        if let expiresIn = expiresIn {
            let expiryDate = Date().addingTimeInterval(expiresIn)
            keychain.save(String(expiryDate.timeIntervalSince1970), forKey: .dropboxTokenExpiry)
        }
        
        isDropboxAuthenticated = true
        errorMessage = nil
        
        dlog("☁️ [CloudStorageManager] Dropbox authentication successful")
        
        // Fetch account info
        Task {
            await fetchDropboxAccountInfo()
        }
    }
    
    /// Fetch Dropbox account info (email/name)
    private func fetchDropboxAccountInfo() async {
        guard let accountInfo = await DropboxManager.shared.getAccountInfo() else {
            return
        }
        dropboxAccountInfo = accountInfo
    }
    
    // MARK: - Google Drive Authentication
    
    /// Check Google Drive authentication status from keychain
    private func checkGoogleDriveAuthStatus() {
        if let _ = keychain.loadString(forKey: .googleDriveAccessToken) {
            isGoogleDriveAuthenticated = true
            // Load account info if available
            googleDriveAccountInfo = keychain.loadString(forKey: .googleDriveRefreshToken) != nil ? "Signed in" : nil
            dlog("☁️ [CloudStorageManager] Google Drive: Authenticated")
        } else {
            isGoogleDriveAuthenticated = false
            googleDriveAccountInfo = nil
            dlog("☁️ [CloudStorageManager] Google Drive: Not authenticated")
        }
    }
    
    /// Sign in to Google Drive (opens OAuth flow)
    func signInToGoogleDrive() {
        Task {
            await GoogleDriveManager.shared.startOAuthFlow()
        }
    }
    
    /// Sign out of Google Drive
    func signOutOfGoogleDrive() {
        // Clear tokens
        keychain.delete(key: .googleDriveAccessToken)
        keychain.delete(key: .googleDriveRefreshToken)
        keychain.delete(key: .googleDriveTokenExpiry)
        
        isGoogleDriveAuthenticated = false
        googleDriveAccountInfo = nil
        
        // If Google Drive was selected, switch back to iCloud Drive
        if selectedProvider == .googleDrive {
            selectProvider(.iCloudDrive)
        }
        
        dlog("☁️ [CloudStorageManager] Signed out of Google Drive")
    }
    
    /// Called when Google Drive OAuth completes successfully
    func onGoogleDriveAuthSuccess(accessToken: String, refreshToken: String?, expiresIn: TimeInterval?) {
        keychain.save(accessToken, forKey: .googleDriveAccessToken)
        
        if let refreshToken = refreshToken {
            keychain.save(refreshToken, forKey: .googleDriveRefreshToken)
        }
        
        if let expiresIn = expiresIn {
            let expiryDate = Date().addingTimeInterval(expiresIn)
            keychain.save(String(expiryDate.timeIntervalSince1970), forKey: .googleDriveTokenExpiry)
        }
        
        isGoogleDriveAuthenticated = true
        errorMessage = nil
        
        dlog("☁️ [CloudStorageManager] Google Drive authentication successful")
        
        // Fetch account info
        Task {
            await fetchGoogleDriveAccountInfo()
        }
    }
    
    /// Fetch Google Drive account info (email/name)
    private func fetchGoogleDriveAccountInfo() async {
        guard let accountInfo = await GoogleDriveManager.shared.getAccountInfo() else {
            return
        }
        googleDriveAccountInfo = accountInfo
    }
    
    // MARK: - Google Drive File Ops
    
    /// Delete a folder (recording) from Google Drive by its folder ID
    /// Expects the recording folder to be identified by `GoogleDriveRecordingInfo.id`
    func deleteGoogleDriveFolder(folderId: String) async throws {
        do {
            try await GoogleDriveManager.shared.deleteFolder(folderId: folderId)
        } catch {
            throw error
        }
    }
}

// MARK: - Notification Names

extension Notification.Name {
    static let cloudStorageProviderChanged = Notification.Name("cloudStorageProviderChanged")
}
