//
//  DropboxUploader.swift
//  Tracking Streamer (visionOS)
//
//  Handles Dropbox file uploads using tokens synced from iOS.
//  This is a simplified version - no OAuth flow, just API calls with existing tokens.
//

import Foundation

/// Uploads recordings to Dropbox using tokens from iCloud Keychain
class DropboxUploader {
    static let shared = DropboxUploader()
    
    /// Your Dropbox App Key - must match iOS app
    private let appKey = "p4iwmllykm4ytc2"
    
    private let keychain = KeychainManager.shared
    
    private init() {}
    
    // MARK: - Token Management
    
    /// Get valid access token, refreshing if needed
    func getValidAccessToken() async -> String? {
        guard let accessToken = keychain.loadString(forKey: .dropboxAccessToken) else {
            dlog("❌ [DropboxUploader] No access token in keychain")
            return nil
        }
        
        // Check expiry
        if let expiryString = keychain.loadString(forKey: .dropboxTokenExpiry),
           let expiryTimestamp = Double(expiryString) {
            let expiryDate = Date(timeIntervalSince1970: expiryTimestamp)
            
            // Refresh if expiring within 5 minutes
            if expiryDate.timeIntervalSinceNow < 300 {
                dlog("🔄 [DropboxUploader] Token expiring soon, refreshing...")
                return await refreshAccessToken()
            }
        }
        
        return accessToken
    }
    
    /// Refresh access token using refresh token
    private func refreshAccessToken() async -> String? {
        guard let refreshToken = keychain.loadString(forKey: .dropboxRefreshToken) else {
            dlog("❌ [DropboxUploader] No refresh token available")
            return nil
        }
        
        let url = URL(string: "https://api.dropboxapi.com/oauth2/token")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/x-www-form-urlencoded", forHTTPHeaderField: "Content-Type")
        
        let body = [
            "grant_type": "refresh_token",
            "refresh_token": refreshToken,
            "client_id": appKey
        ]
        
        request.httpBody = body
            .map { "\($0.key)=\($0.value.addingPercentEncoding(withAllowedCharacters: .urlQueryAllowed) ?? $0.value)" }
            .joined(separator: "&")
            .data(using: .utf8)
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                dlog("❌ [DropboxUploader] Token refresh failed")
                return nil
            }
            
            struct TokenResponse: Codable {
                let access_token: String
                let expires_in: TimeInterval?
            }
            
            let tokenResponse = try JSONDecoder().decode(TokenResponse.self, from: data)
            
            // Save new tokens
            keychain.save(tokenResponse.access_token, forKey: .dropboxAccessToken)
            if let expiresIn = tokenResponse.expires_in {
                let expiryDate = Date().addingTimeInterval(expiresIn)
                keychain.save(String(expiryDate.timeIntervalSince1970), forKey: .dropboxTokenExpiry)
            }
            
            dlog("✅ [DropboxUploader] Token refreshed")
            return tokenResponse.access_token
            
        } catch {
            dlog("❌ [DropboxUploader] Token refresh error: \(error)")
            return nil
        }
    }
    
    // MARK: - Upload Methods
    
    /// Upload a single file to Dropbox
    /// - Parameters:
    ///   - fileURL: Local file URL
    ///   - dropboxPath: Destination path in Dropbox
    /// - Returns: Success status
    func uploadFile(from fileURL: URL, to dropboxPath: String) async -> Bool {
        guard let accessToken = await getValidAccessToken() else {
            return false
        }
        
        guard let fileData = try? Data(contentsOf: fileURL) else {
            dlog("❌ [DropboxUploader] Cannot read file: \(fileURL)")
            return false
        }
        
        let url = URL(string: "https://content.dropboxapi.com/2/files/upload")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        request.setValue("application/octet-stream", forHTTPHeaderField: "Content-Type")
        
        let apiArg: [String: Any] = [
            "path": dropboxPath,
            "mode": "overwrite",
            "autorename": true,
            "mute": false
        ]
        
        if let apiArgData = try? JSONSerialization.data(withJSONObject: apiArg),
           let apiArgString = String(data: apiArgData, encoding: .utf8) {
            request.setValue(apiArgString, forHTTPHeaderField: "Dropbox-API-Arg")
        }
        
        request.httpBody = fileData
        
        do {
            let (_, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                let statusCode = (response as? HTTPURLResponse)?.statusCode ?? -1
                dlog("❌ [DropboxUploader] Upload failed: HTTP \(statusCode)")
                return false
            }
            
            dlog("✅ [DropboxUploader] Uploaded \(fileURL.lastPathComponent) to \(dropboxPath)")
            return true
            
        } catch {
            dlog("❌ [DropboxUploader] Upload error: \(error)")
            return false
        }
    }
    
    /// Upload an entire recording folder to Dropbox
    /// - Parameters:
    ///   - folderURL: Local recording folder
    ///   - recordingName: Name for the recording in Dropbox
    ///   - progressCallback: Called with (currentFile, totalFiles, currentFileName) for each file
    /// - Returns: Success status
    func uploadRecording(folderURL: URL, recordingName: String, progressCallback: ((Int, Int, String) -> Void)? = nil) async -> Bool {
        let basePath = "/Apps/VisionProTeleop/\(recordingName)"
        
        // Create folder first
        await createFolder(path: basePath)
        
        let fileManager = FileManager.default
        guard let enumerator = fileManager.enumerator(at: folderURL, includingPropertiesForKeys: nil) else {
            return false
        }
        
        // Collect all files first to get total count
        var filesToUpload: [(url: URL, relativePath: String)] = []
        for case let fileURL as URL in enumerator {
            var isDirectory: ObjCBool = false
            guard fileManager.fileExists(atPath: fileURL.path, isDirectory: &isDirectory),
                  !isDirectory.boolValue else {
                continue
            }
            let relativePath = fileURL.path.replacingOccurrences(of: folderURL.path, with: "")
            filesToUpload.append((url: fileURL, relativePath: relativePath))
        }
        
        let totalFiles = filesToUpload.count
        var success = true
        
        for (index, file) in filesToUpload.enumerated() {
            let dropboxPath = basePath + file.relativePath
            let fileName = file.url.lastPathComponent
            
            // Report progress with filename
            progressCallback?(index + 1, totalFiles, fileName)
            
            if await !uploadFile(from: file.url, to: dropboxPath) {
                success = false
            }
        }
        
        return success
    }
    
    /// Create a folder in Dropbox
    private func createFolder(path: String) async {
        guard let accessToken = await getValidAccessToken() else { return }
        
        let url = URL(string: "https://api.dropboxapi.com/2/files/create_folder_v2")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        
        let body: [String: Any] = [
            "path": path,
            "autorename": false
        ]
        
        request.httpBody = try? JSONSerialization.data(withJSONObject: body)
        
        // Ignore errors - folder might already exist
        _ = try? await URLSession.shared.data(for: request)
    }
    
    /// Check if Dropbox is configured and has valid tokens
    func isAvailable() -> Bool {
        return keychain.exists(key: .dropboxAccessToken)
    }
}
