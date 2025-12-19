//
//  KeychainManager.swift
//  Tracking Viewer
//
//  Manages secure, iCloud-synced keychain storage for cloud storage credentials.
//  Uses kSecAttrSynchronizable to sync tokens between iOS and visionOS devices.
//

import Foundation
import Security

/// Manages keychain operations with iCloud sync support
class KeychainManager {
    static let shared = KeychainManager()
    
    // MARK: - Constants
    
    /// Service identifier for our app's keychain items
    /// Using a shared service name allows iCloud Keychain to sync between devices
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
    /// - Parameters:
    ///   - value: The string value to save
    ///   - key: The key to store the value under
    ///   - synchronizable: Whether to sync via iCloud Keychain (default: true)
    /// - Returns: Whether the operation succeeded
    @discardableResult
    func save(_ value: String, forKey key: KeychainKey, synchronizable: Bool = true) -> Bool {
        guard let data = value.data(using: .utf8) else {
            dlog("❌ [KeychainManager] Failed to convert string to data")
            return false
        }
        return save(data, forKey: key, synchronizable: synchronizable)
    }
    
    /// Save data to the keychain with iCloud sync
    /// - Parameters:
    ///   - data: The data to save
    ///   - key: The key to store the data under
    ///   - synchronizable: Whether to sync via iCloud Keychain (default: true)
    /// - Returns: Whether the operation succeeded
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
            dlog("✅ [KeychainManager] Saved \(key.rawValue) to keychain (sync: \(synchronizable))")
            return true
        } else {
            dlog("❌ [KeychainManager] Failed to save \(key.rawValue): \(status)")
            return false
        }
    }
    
    /// Load a string value from the keychain
    /// - Parameter key: The key to retrieve
    /// - Returns: The string value if found, nil otherwise
    func loadString(forKey key: KeychainKey) -> String? {
        guard let data = loadData(forKey: key) else { return nil }
        return String(data: data, encoding: .utf8)
    }
    
    /// Load data from the keychain
    /// - Parameter key: The key to retrieve
    /// - Returns: The data if found, nil otherwise
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
            dlog("✅ [KeychainManager] Loaded \(key.rawValue) from keychain")
            return data
        } else if status == errSecItemNotFound {
            // dlog("ℹ️ [KeychainManager] Key \(key.rawValue) not found in keychain")
            return nil
        } else {
            dlog("❌ [KeychainManager] Failed to load \(key.rawValue): \(status)")
            return nil
        }
    }
    
    /// Delete a value from the keychain
    /// - Parameter key: The key to delete
    /// - Returns: Whether the operation succeeded
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
        
        if status == errSecSuccess || status == errSecItemNotFound {
            dlog("✅ [KeychainManager] Deleted \(key.rawValue) from keychain")
            return true
        } else {
            dlog("❌ [KeychainManager] Failed to delete \(key.rawValue): \(status)")
            return false
        }
    }
    
    /// Delete all items for this service
    func deleteAll() {
        for key in [KeychainKey.dropboxAccessToken, .dropboxRefreshToken, .dropboxTokenExpiry, .selectedCloudProvider] {
            delete(key: key)
        }
        dlog("🗑️ [KeychainManager] Deleted all keychain items")
    }
    
    /// Check if a key exists in the keychain
    func exists(key: KeychainKey) -> Bool {
        return loadData(forKey: key) != nil
    }
}
