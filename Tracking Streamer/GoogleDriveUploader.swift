//
//  GoogleDriveUploader.swift
//  Tracking Streamer (visionOS)
//
//  Handles Google Drive file uploads using tokens synced from iOS.
//  This is a simplified version - no OAuth flow, just API calls with existing tokens.
//

import Foundation

/// Uploads recordings to Google Drive using tokens from iCloud Keychain
class GoogleDriveUploader {
    static let shared = GoogleDriveUploader()
    
    /// Your Google OAuth Client ID - must match iOS app
    private let clientID = "613801329299-jcqrgumln3s9udtomsll5dahcbjcivej.apps.googleusercontent.com"
    
    private let keychain = KeychainManager.shared
    
    private init() {}
    
    // MARK: - Token Management
    
    /// Get valid access token, refreshing if needed
    func getValidAccessToken() async -> String? {
        guard let accessToken = keychain.loadString(forKey: .googleDriveAccessToken) else {
            print("❌ [GoogleDriveUploader] No access token in keychain")
            return nil
        }
        
        // Check expiry
        if let expiryString = keychain.loadString(forKey: .googleDriveTokenExpiry),
           let expiryTimestamp = Double(expiryString) {
            let expiryDate = Date(timeIntervalSince1970: expiryTimestamp)
            
            // Refresh if expiring within 5 minutes
            if expiryDate.timeIntervalSinceNow < 300 {
                print("🔄 [GoogleDriveUploader] Token expiring soon, refreshing...")
                return await refreshAccessToken()
            }
        }
        
        return accessToken
    }
    
    /// Refresh access token using refresh token
    private func refreshAccessToken() async -> String? {
        guard let refreshToken = keychain.loadString(forKey: .googleDriveRefreshToken) else {
            print("❌ [GoogleDriveUploader] No refresh token available")
            return nil
        }
        
        let url = URL(string: "https://oauth2.googleapis.com/token")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/x-www-form-urlencoded", forHTTPHeaderField: "Content-Type")
        
        let body = [
            "grant_type": "refresh_token",
            "refresh_token": refreshToken,
            "client_id": clientID
        ]
        
        request.httpBody = body
            .map { "\($0.key)=\($0.value.addingPercentEncoding(withAllowedCharacters: .urlQueryAllowed) ?? $0.value)" }
            .joined(separator: "&")
            .data(using: .utf8)
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                print("❌ [GoogleDriveUploader] Token refresh failed")
                return nil
            }
            
            struct TokenResponse: Codable {
                let access_token: String
                let expires_in: TimeInterval?
            }
            
            let tokenResponse = try JSONDecoder().decode(TokenResponse.self, from: data)
            
            // Save new tokens
            keychain.save(tokenResponse.access_token, forKey: .googleDriveAccessToken)
            if let expiresIn = tokenResponse.expires_in {
                let expiryDate = Date().addingTimeInterval(expiresIn)
                keychain.save(String(expiryDate.timeIntervalSince1970), forKey: .googleDriveTokenExpiry)
            }
            
            print("✅ [GoogleDriveUploader] Token refreshed")
            return tokenResponse.access_token
            
        } catch {
            print("❌ [GoogleDriveUploader] Token refresh error: \(error)")
            return nil
        }
    }
    
    // MARK: - Folder Management
    
    /// Get or create the VisionProTeleop folder in Google Drive
    /// Returns the folder ID
    private func getOrCreateAppFolder() async -> String? {
        guard let accessToken = await getValidAccessToken() else {
            return nil
        }
        
        // First, search for existing folder
        let searchURL = URL(string: "https://www.googleapis.com/drive/v3/files")!
        var searchComponents = URLComponents(url: searchURL, resolvingAgainstBaseURL: false)!
        searchComponents.queryItems = [
            URLQueryItem(name: "q", value: "name='VisionProTeleop' and mimeType='application/vnd.google-apps.folder' and trashed=false"),
            URLQueryItem(name: "spaces", value: "drive"),
            URLQueryItem(name: "fields", value: "files(id, name)")
        ]
        
        var searchRequest = URLRequest(url: searchComponents.url!)
        searchRequest.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        
        do {
            let (data, response) = try await URLSession.shared.data(for: searchRequest)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                print("❌ [GoogleDriveUploader] Failed to search for folder")
                return nil
            }
            
            if let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
               let files = json["files"] as? [[String: Any]],
               let firstFolder = files.first,
               let folderId = firstFolder["id"] as? String {
                print("✅ [GoogleDriveUploader] Found existing VisionProTeleop folder: \(folderId)")
                return folderId
            }
            
            // Create folder if it doesn't exist
            return await createFolder(name: "VisionProTeleop", parentId: nil)
            
        } catch {
            print("❌ [GoogleDriveUploader] Error searching for folder: \(error)")
            return nil
        }
    }
    
    /// Create a folder in Google Drive
    private func createFolder(name: String, parentId: String?) async -> String? {
        guard let accessToken = await getValidAccessToken() else {
            return nil
        }
        
        let url = URL(string: "https://www.googleapis.com/drive/v3/files")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        
        var metadata: [String: Any] = [
            "name": name,
            "mimeType": "application/vnd.google-apps.folder"
        ]
        
        if let parentId = parentId {
            metadata["parents"] = [parentId]
        }
        
        request.httpBody = try? JSONSerialization.data(withJSONObject: metadata)
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                print("❌ [GoogleDriveUploader] Failed to create folder")
                return nil
            }
            
            if let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
               let folderId = json["id"] as? String {
                print("✅ [GoogleDriveUploader] Created folder: \(name) with ID: \(folderId)")
                return folderId
            }
            
            return nil
            
        } catch {
            print("❌ [GoogleDriveUploader] Error creating folder: \(error)")
            return nil
        }
    }
    
    // MARK: - Upload Methods
    
    /// Upload a single file to Google Drive
    /// - Parameters:
    ///   - fileURL: Local file URL
    ///   - fileName: Name for the file in Google Drive
    ///   - parentFolderId: Parent folder ID (optional)
    /// - Returns: Success status
    func uploadFile(from fileURL: URL, fileName: String, parentFolderId: String?) async -> Bool {
        guard let accessToken = await getValidAccessToken() else {
            return false
        }
        
        guard let fileData = try? Data(contentsOf: fileURL) else {
            print("❌ [GoogleDriveUploader] Cannot read file: \(fileURL)")
            return false
        }
        
        // Determine MIME type
        let mimeType = getMimeType(for: fileURL.pathExtension)
        
        // Create multipart upload request
        let boundary = UUID().uuidString
        let url = URL(string: "https://www.googleapis.com/upload/drive/v3/files?uploadType=multipart")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        request.setValue("multipart/related; boundary=\(boundary)", forHTTPHeaderField: "Content-Type")
        
        // Build metadata
        var metadata: [String: Any] = ["name": fileName]
        if let parentFolderId = parentFolderId {
            metadata["parents"] = [parentFolderId]
        }
        
        guard let metadataData = try? JSONSerialization.data(withJSONObject: metadata) else {
            return false
        }
        
        // Build multipart body
        var body = Data()
        body.append("--\(boundary)\r\n".data(using: .utf8)!)
        body.append("Content-Type: application/json; charset=UTF-8\r\n\r\n".data(using: .utf8)!)
        body.append(metadataData)
        body.append("\r\n--\(boundary)\r\n".data(using: .utf8)!)
        body.append("Content-Type: \(mimeType)\r\n\r\n".data(using: .utf8)!)
        body.append(fileData)
        body.append("\r\n--\(boundary)--\r\n".data(using: .utf8)!)
        
        request.httpBody = body
        
        do {
            let (_, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                let statusCode = (response as? HTTPURLResponse)?.statusCode ?? -1
                print("❌ [GoogleDriveUploader] Upload failed: HTTP \(statusCode)")
                return false
            }
            
            print("✅ [GoogleDriveUploader] Uploaded \(fileName)")
            return true
            
        } catch {
            print("❌ [GoogleDriveUploader] Upload error: \(error)")
            return false
        }
    }
    
    /// Upload an entire recording folder to Google Drive
    /// - Parameters:
    ///   - folderURL: Local recording folder
    ///   - recordingName: Name for the recording in Google Drive
    ///   - progressCallback: Called with (currentFile, totalFiles, currentFileName) for each file
    /// - Returns: Success status
    func uploadRecording(folderURL: URL, recordingName: String, progressCallback: ((Int, Int, String) -> Void)? = nil) async -> Bool {
        // Get or create app folder
        guard let appFolderId = await getOrCreateAppFolder() else {
            print("❌ [GoogleDriveUploader] Failed to get app folder")
            return false
        }
        
        // Create recording folder
        guard let recordingFolderId = await createFolder(name: recordingName, parentId: appFolderId) else {
            print("❌ [GoogleDriveUploader] Failed to create recording folder")
            return false
        }
        
        let fileManager = FileManager.default
        guard let enumerator = fileManager.enumerator(at: folderURL, includingPropertiesForKeys: nil) else {
            return false
        }
        
        // Collect all files first to get total count
        var filesToUpload: [URL] = []
        for case let fileURL as URL in enumerator {
            var isDirectory: ObjCBool = false
            guard fileManager.fileExists(atPath: fileURL.path, isDirectory: &isDirectory),
                  !isDirectory.boolValue else {
                continue
            }
            filesToUpload.append(fileURL)
        }
        
        let totalFiles = filesToUpload.count
        var success = true
        
        for (index, fileURL) in filesToUpload.enumerated() {
            let fileName = fileURL.lastPathComponent
            
            // Report progress with filename
            progressCallback?(index + 1, totalFiles, fileName)
            
            if await !uploadFile(from: fileURL, fileName: fileName, parentFolderId: recordingFolderId) {
                success = false
            }
        }
        
        return success
    }
    
    // MARK: - Helper Methods
    
    private func getMimeType(for pathExtension: String) -> String {
        switch pathExtension.lowercased() {
        case "mp4": return "video/mp4"
        case "json": return "application/json"
        case "jsonl": return "application/jsonl"
        case "txt": return "text/plain"
        case "jpg", "jpeg": return "image/jpeg"
        case "png": return "image/png"
        case "usdz": return "model/vnd.usdz+zip"
        default: return "application/octet-stream"
        }
    }
    
    /// Check if Google Drive is configured and has valid tokens
    func isAvailable() -> Bool {
        return keychain.exists(key: .googleDriveAccessToken)
    }
}
