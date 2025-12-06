//
//  KeychainManager.swift
//  Tracking Streamer (visionOS)
//
//  Manages secure, iCloud-synced keychain storage for cloud storage credentials.
//  Uses kSecAttrSynchronizable to read tokens synced from iOS.
//

import Foundation
import Security

/// Manages keychain operations with iCloud sync support
class KeychainManager {
    static let shared = KeychainManager()
    
    // MARK: - Constants
    
    /// Service identifier - must match iOS app for iCloud Keychain sync
    private let service = "VisionProTeleop.CloudStorage"
    
    /// Shared keychain access group for cross-app sync (Team ID + group identifier)
    private let accessGroup = "ATTMC2WVK2.com.younghyopark.VisionProTeleop.shared"
    
    // MARK: - Keys
    
    enum KeychainKey: String {
        case dropboxAccessToken = "dropbox_access_token"
        case dropboxRefreshToken = "dropbox_refresh_token"
        case dropboxTokenExpiry = "dropbox_token_expiry"
        case googleDriveAccessToken = "google_drive_access_token"
        case googleDriveRefreshToken = "google_drive_refresh_token"
        case googleDriveTokenExpiry = "google_drive_token_expiry"
        case selectedCloudProvider = "selected_cloud_provider"
    }
    
    private init() {}
    
    // MARK: - Public Methods
    
    /// Save a string value to the keychain with iCloud sync
    @discardableResult
    func save(_ value: String, forKey key: KeychainKey, synchronizable: Bool = true) -> Bool {
        guard let data = value.data(using: .utf8) else {
            print("❌ [KeychainManager] Failed to convert string to data")
            return false
        }
        return save(data, forKey: key, synchronizable: synchronizable)
    }
    
    /// Save data to the keychain with iCloud sync
    @discardableResult
    func save(_ data: Data, forKey key: KeychainKey, synchronizable: Bool = true) -> Bool {
        // Delete any existing item first
        delete(key: key)
        
        let query: [String: Any] = [
            kSecClass as String: kSecClassGenericPassword,
            kSecAttrService as String: service,
            kSecAttrAccount as String: key.rawValue,
            kSecValueData as String: data,
            kSecAttrAccessible as String: kSecAttrAccessibleAfterFirstUnlock,
            kSecAttrSynchronizable as String: synchronizable ? kCFBooleanTrue! : kCFBooleanFalse!,
            kSecAttrAccessGroup as String: accessGroup
        ]
        
        let status = SecItemAdd(query as CFDictionary, nil)
        
        if status == errSecSuccess {
            print("✅ [KeychainManager] Saved \(key.rawValue) to keychain (sync: \(synchronizable))")
            return true
        } else {
            print("❌ [KeychainManager] Failed to save \(key.rawValue): \(status)")
            return false
        }
    }
    
    /// Load a string value from the keychain
    func loadString(forKey key: KeychainKey) -> String? {
        guard let data = loadData(forKey: key) else { return nil }
        return String(data: data, encoding: .utf8)
    }
    
    /// Load data from the keychain
    func loadData(forKey key: KeychainKey) -> Data? {
        let query: [String: Any] = [
            kSecClass as String: kSecClassGenericPassword,
            kSecAttrService as String: service,
            kSecAttrAccount as String: key.rawValue,
            kSecReturnData as String: kCFBooleanTrue!,
            kSecMatchLimit as String: kSecMatchLimitOne,
            kSecAttrSynchronizable as String: kSecAttrSynchronizableAny,
            kSecAttrAccessGroup as String: accessGroup
        ]
        
        var dataTypeRef: AnyObject?
        let status = SecItemCopyMatching(query as CFDictionary, &dataTypeRef)
        
        if status == errSecSuccess, let data = dataTypeRef as? Data {
            print("✅ [KeychainManager] Loaded \(key.rawValue) from keychain")
            return data
        } else if status == errSecItemNotFound {
            print("ℹ️ [KeychainManager] Key \(key.rawValue) not found in keychain")
            return nil
        } else {
            print("❌ [KeychainManager] Failed to load \(key.rawValue): \(status)")
            return nil
        }
    }
    
    /// Delete a value from the keychain
    @discardableResult
    func delete(key: KeychainKey) -> Bool {
        let query: [String: Any] = [
            kSecClass as String: kSecClassGenericPassword,
            kSecAttrService as String: service,
            kSecAttrAccount as String: key.rawValue,
            kSecAttrSynchronizable as String: kSecAttrSynchronizableAny,
            kSecAttrAccessGroup as String: accessGroup
        ]
        
        let status = SecItemDelete(query as CFDictionary)
        return status == errSecSuccess || status == errSecItemNotFound
    }
    
    /// Check if a key exists in the keychain
    func exists(key: KeychainKey) -> Bool {
        return loadData(forKey: key) != nil
    }
}
