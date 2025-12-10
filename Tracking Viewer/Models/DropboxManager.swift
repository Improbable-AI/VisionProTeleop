//
//  DropboxManager.swift
//  Tracking Viewer
//
//  Handles Dropbox OAuth authentication and API operations.
//  Uses OAuth 2.0 with PKCE for secure authentication.
//

import Foundation
import AuthenticationServices
import CryptoKit
import Combine

/// Manages Dropbox OAuth and file uploads
@MainActor
class DropboxManager: NSObject, ObservableObject {
    static let shared = DropboxManager()
    
    // MARK: - Dropbox App Configuration
    
    private let appKey = "p4iwmllykm4ytc2"
    
    /// Redirect URI for mobile OAuth with PKCE
    private var redirectURI: String {
        return "db-\(appKey)://oauth2redirect"
    }
    
    /// URL scheme for ASWebAuthenticationSession callback
    private var callbackURLScheme: String {
        return "db-\(appKey)"
    }
    
    // MARK: - OAuth State
    
    private var codeVerifier: String?
    private var authSession: ASWebAuthenticationSession?
    
    // MARK: - Published Properties
    
    @Published var isAuthenticating: Bool = false
    @Published var authError: String?
    
    // Pagination state
    @Published var isLoadingMore: Bool = false
    @Published var hasMoreRecordings: Bool = true
    private var listFolderCursor: String?
    private let pageSize = 20
    
    /// Check if user is authenticated with Dropbox (has stored tokens)
    var isAuthenticated: Bool {
        return KeychainManager.shared.loadString(forKey: .dropboxAccessToken) != nil
    }
    
    private override init() {
        super.init()
    }
    
    // MARK: - OAuth Flow
    
    /// Start the Dropbox OAuth flow using ASWebAuthenticationSession
    func startOAuthFlow() async {
        guard appKey != "YOUR_DROPBOX_APP_KEY" else {
            authError = "Dropbox App Key not configured. Please set up a Dropbox app."
            dlog("❌ [DropboxManager] App Key not configured")
            return
        }
        
        isAuthenticating = true
        authError = nil
        
        // Generate PKCE code verifier and challenge
        let verifier = generateCodeVerifier()
        codeVerifier = verifier
        let challenge = generateCodeChallenge(from: verifier)
        
        // Build authorization URL
        var components = URLComponents(string: "https://www.dropbox.com/oauth2/authorize")!
        components.queryItems = [
            URLQueryItem(name: "client_id", value: appKey),
            URLQueryItem(name: "response_type", value: "code"),
            URLQueryItem(name: "redirect_uri", value: redirectURI),
            URLQueryItem(name: "code_challenge", value: challenge),
            URLQueryItem(name: "code_challenge_method", value: "S256"),
            URLQueryItem(name: "token_access_type", value: "offline"), // Get refresh token
        ]
        
        guard let authURL = components.url else {
            authError = "Failed to build authorization URL"
            isAuthenticating = false
            return
        }
        
        dlog("🔐 [DropboxManager] Starting OAuth flow...")
        
        // Use ASWebAuthenticationSession for secure OAuth
        await withCheckedContinuation { (continuation: CheckedContinuation<Void, Never>) in
            authSession = ASWebAuthenticationSession(
                url: authURL,
                callbackURLScheme: callbackURLScheme
            ) { [weak self] callbackURL, error in
                Task { @MainActor in
                    defer { continuation.resume() }
                    
                    guard let self = self else { return }
                    self.isAuthenticating = false
                    
                    if let error = error {
                        if case ASWebAuthenticationSessionError.canceledLogin = error {
                            self.authError = "Sign in was cancelled"
                        } else {
                            self.authError = "Authentication failed: \(error.localizedDescription)"
                        }
                        dlog("❌ [DropboxManager] Auth error: \(error)")
                        return
                    }
                    
                    guard let callbackURL = callbackURL,
                          let code = self.extractCode(from: callbackURL) else {
                        self.authError = "Failed to get authorization code"
                        return
                    }
                    
                    // Exchange code for tokens
                    await self.exchangeCodeForTokens(code: code)
                }
            }
            
            // Set presentation context and start
            authSession?.presentationContextProvider = self
            authSession?.prefersEphemeralWebBrowserSession = false
            authSession?.start()
        }
    }
    
    /// Handle OAuth callback URL (for universal links or custom URL scheme)
    func handleCallback(url: URL) async -> Bool {
        guard let code = extractCode(from: url) else {
            dlog("❌ [DropboxManager] No code in callback URL")
            return false
        }
        
        await exchangeCodeForTokens(code: code)
        return true
    }
    
    // MARK: - Token Exchange
    
    /// Exchange authorization code for access and refresh tokens
    private func exchangeCodeForTokens(code: String) async {
        guard let verifier = codeVerifier else {
            authError = "Missing code verifier"
            return
        }
        
        let url = URL(string: "https://api.dropboxapi.com/oauth2/token")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/x-www-form-urlencoded", forHTTPHeaderField: "Content-Type")
        
        let body = [
            "code": code,
            "grant_type": "authorization_code",
            "client_id": appKey,
            "redirect_uri": redirectURI,
            "code_verifier": verifier
        ]
        
        request.httpBody = body
            .map { "\($0.key)=\($0.value.addingPercentEncoding(withAllowedCharacters: .urlQueryAllowed) ?? $0.value)" }
            .joined(separator: "&")
            .data(using: .utf8)
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                let statusCode = (response as? HTTPURLResponse)?.statusCode ?? -1
                authError = "Token exchange failed (HTTP \(statusCode))"
                dlog("❌ [DropboxManager] Token exchange failed: HTTP \(statusCode)")
                if let body = String(data: data, encoding: .utf8) {
                    dlog("   Response: \(body)")
                }
                return
            }
            
            let tokenResponse = try JSONDecoder().decode(DropboxTokenResponse.self, from: data)
            
            // Save tokens via CloudStorageManager (which saves to keychain)
            CloudStorageManager.shared.onDropboxAuthSuccess(
                accessToken: tokenResponse.accessToken,
                refreshToken: tokenResponse.refreshToken,
                expiresIn: tokenResponse.expiresIn
            )
            
            codeVerifier = nil
            dlog("✅ [DropboxManager] Token exchange successful")
            
        } catch {
            authError = "Token exchange failed: \(error.localizedDescription)"
            dlog("❌ [DropboxManager] Token exchange error: \(error)")
        }
    }
    
    /// Refresh the access token using the refresh token
    func refreshAccessToken() async -> String? {
        guard let refreshToken = KeychainManager.shared.loadString(forKey: .dropboxRefreshToken) else {
            dlog("❌ [DropboxManager] No refresh token available")
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
                dlog("❌ [DropboxManager] Token refresh failed")
                return nil
            }
            
            let tokenResponse = try JSONDecoder().decode(DropboxTokenResponse.self, from: data)
            
            // Save new access token
            KeychainManager.shared.save(tokenResponse.accessToken, forKey: .dropboxAccessToken)
            
            if let expiresIn = tokenResponse.expiresIn {
                let expiryDate = Date().addingTimeInterval(expiresIn)
                KeychainManager.shared.save(String(expiryDate.timeIntervalSince1970), forKey: .dropboxTokenExpiry)
            }
            
            dlog("✅ [DropboxManager] Token refreshed")
            return tokenResponse.accessToken
            
        } catch {
            dlog("❌ [DropboxManager] Token refresh error: \(error)")
            return nil
        }
    }
    
    // MARK: - API Operations
    
    /// Get current access token, refreshing if needed
    func getValidAccessToken() async -> String? {
        guard let accessToken = KeychainManager.shared.loadString(forKey: .dropboxAccessToken) else {
            return nil
        }
        
        // Check if token is expired
        if let expiryString = KeychainManager.shared.loadString(forKey: .dropboxTokenExpiry),
           let expiryTimestamp = Double(expiryString) {
            let expiryDate = Date(timeIntervalSince1970: expiryTimestamp)
            
            // Refresh if expiring within 5 minutes
            if expiryDate.timeIntervalSinceNow < 300 {
                dlog("🔄 [DropboxManager] Token expiring soon, refreshing...")
                return await refreshAccessToken()
            }
        }
        
        return accessToken
    }
    
    /// Get account info (email or display name)
    func getAccountInfo() async -> String? {
        guard let accessToken = await getValidAccessToken() else {
            return nil
        }
        
        let url = URL(string: "https://api.dropboxapi.com/2/users/get_current_account")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                return nil
            }
            
            if let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
               let email = json["email"] as? String {
                return email
            }
            
            return nil
            
        } catch {
            dlog("❌ [DropboxManager] Failed to get account info: \(error)")
            return nil
        }
    }
    
    /// Upload a file to Dropbox
    /// - Parameters:
    ///   - fileURL: Local file URL to upload
    ///   - dropboxPath: Destination path in Dropbox (e.g., "/Apps/VisionProTeleop/recording.mp4")
    /// - Returns: Whether the upload succeeded
    func uploadFile(from fileURL: URL, to dropboxPath: String) async -> Bool {
        guard let accessToken = await getValidAccessToken() else {
            dlog("❌ [DropboxManager] No valid access token for upload")
            return false
        }
        
        // Read file data
        guard let fileData = try? Data(contentsOf: fileURL) else {
            dlog("❌ [DropboxManager] Failed to read file: \(fileURL)")
            return false
        }
        
        let url = URL(string: "https://content.dropboxapi.com/2/files/upload")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        request.setValue("application/octet-stream", forHTTPHeaderField: "Content-Type")
        
        // Dropbox API arg header
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
                dlog("❌ [DropboxManager] Upload failed: HTTP \(statusCode)")
                return false
            }
            
            dlog("✅ [DropboxManager] Uploaded \(fileURL.lastPathComponent) to \(dropboxPath)")
            return true
            
        } catch {
            dlog("❌ [DropboxManager] Upload error: \(error)")
            return false
        }
    }
    
    /// Upload a folder (recording) to Dropbox
    func uploadRecording(folderURL: URL, recordingName: String) async -> Bool {
        let basePath = "/Apps/VisionProTeleop/\(recordingName)"
        
        // Create folder first
        await createFolder(path: basePath)
        
        // Upload all files in the folder
        let fileManager = FileManager.default
        guard let enumerator = fileManager.enumerator(at: folderURL, includingPropertiesForKeys: nil) else {
            return false
        }
        
        var success = true
        
        for case let fileURL as URL in enumerator {
            var isDirectory: ObjCBool = false
            guard fileManager.fileExists(atPath: fileURL.path, isDirectory: &isDirectory),
                  !isDirectory.boolValue else {
                continue
            }
            
            let relativePath = fileURL.path.replacingOccurrences(of: folderURL.path, with: "")
            let dropboxPath = basePath + relativePath
            
            if await !uploadFile(from: fileURL, to: dropboxPath) {
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
        
        _ = try? await URLSession.shared.data(for: request)
        // Ignore errors - folder might already exist
    }
    
    // MARK: - List Files
    
    /// List recording folders in Dropbox with full details (initial fetch with pagination)
    /// Returns array of DropboxRecordingInfo
    func listRecordings() async -> [DropboxRecordingInfo] {
        // Reset pagination state
        listFolderCursor = nil
        hasMoreRecordings = true
        
        guard let accessToken = await getValidAccessToken() else {
            dlog("❌ [DropboxManager] No valid access token for listing")
            return []
        }
        
        let url = URL(string: "https://api.dropboxapi.com/2/files/list_folder")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        
        let body: [String: Any] = [
            "path": "/Apps/VisionProTeleop",
            "recursive": true,
            "include_deleted": false,
            "include_has_explicit_shared_members": false,
            "include_mounted_folders": true,
            "include_non_downloadable_files": false,
            "limit": 100  // Dropbox limit per request for recursive
        ]
        
        request.httpBody = try? JSONSerialization.data(withJSONObject: body)
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                let statusCode = (response as? HTTPURLResponse)?.statusCode ?? -1
                dlog("❌ [DropboxManager] List folder failed: HTTP \(statusCode)")
                return []
            }
            
            guard let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
                  let entries = json["entries"] as? [[String: Any]] else {
                return []
            }
            
            // Store cursor for pagination
            if let hasMore = json["has_more"] as? Bool {
                hasMoreRecordings = hasMore
                if hasMore, let cursor = json["cursor"] as? String {
                    listFolderCursor = cursor
                }
            } else {
                hasMoreRecordings = false
            }
            
            let recordings = parseDropboxEntries(entries)
            dlog("✅ [DropboxManager] Found \(recordings.count) recordings in Dropbox (hasMore: \(hasMoreRecordings))")
            return recordings
            
        } catch {
            dlog("❌ [DropboxManager] List folder error: \(error)")
            return []
        }
    }
    
    /// Load more recordings from Dropbox (pagination continue)
    func loadMoreRecordings() async -> [DropboxRecordingInfo] {
        guard !isLoadingMore, hasMoreRecordings, let cursor = listFolderCursor else {
            return []
        }
        
        isLoadingMore = true
        defer { 
            Task { @MainActor in
                self.isLoadingMore = false 
            }
        }
        
        guard let accessToken = await getValidAccessToken() else {
            return []
        }
        
        let url = URL(string: "https://api.dropboxapi.com/2/files/list_folder/continue")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        
        let body: [String: Any] = ["cursor": cursor]
        request.httpBody = try? JSONSerialization.data(withJSONObject: body)
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                return []
            }
            
            guard let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
                  let entries = json["entries"] as? [[String: Any]] else {
                return []
            }
            
            // Update cursor for next page
            if let hasMore = json["has_more"] as? Bool {
                await MainActor.run {
                    self.hasMoreRecordings = hasMore
                }
                if hasMore, let newCursor = json["cursor"] as? String {
                    listFolderCursor = newCursor
                } else {
                    listFolderCursor = nil
                }
            } else {
                await MainActor.run {
                    self.hasMoreRecordings = false
                }
                listFolderCursor = nil
            }
            
            let recordings = parseDropboxEntries(entries)
            dlog("✅ [DropboxManager] Loaded \(recordings.count) more recordings (hasMore: \(hasMoreRecordings))")
            return recordings
            
        } catch {
            dlog("❌ [DropboxManager] Load more error: \(error)")
            return []
        }
    }
    
    /// Parse Dropbox entries into DropboxRecordingInfo array
    private func parseDropboxEntries(_ entries: [[String: Any]]) -> [DropboxRecordingInfo] {
        // Group files by folder to build recording info
        var folderFiles: [String: [(name: String, size: Int64?, modified: Date?)]] = [:]
        var folderDates: [String: Date] = [:]
        
        for entry in entries {
            guard let pathLower = entry["path_lower"] as? String else { continue }
            
            // Skip the root folder
            let components = pathLower.split(separator: "/")
            guard components.count >= 3 else { continue } // /Apps/VisionProTeleop/recording_xxx/...
            
            let recordingFolder = String(components[2]) // recording_xxx
            let recordingPath = "/Apps/VisionProTeleop/\(recordingFolder)"
            
            if let tag = entry[".tag"] as? String {
                if tag == "folder" && components.count == 3 {
                    // This is the recording folder itself
                    if folderFiles[recordingPath] == nil {
                        folderFiles[recordingPath] = []
                    }
                } else if tag == "file" && components.count == 4 {
                    // This is a file inside a recording folder
                    let fileName = String(components[3])
                    let size = entry["size"] as? Int64
                    
                    var modified: Date?
                    if let serverModified = entry["server_modified"] as? String {
                        let formatter = ISO8601DateFormatter()
                        formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
                        modified = formatter.date(from: serverModified)
                        if modified == nil {
                            formatter.formatOptions = [.withInternetDateTime]
                            modified = formatter.date(from: serverModified)
                        }
                    }
                    
                    if folderFiles[recordingPath] == nil {
                        folderFiles[recordingPath] = []
                    }
                    folderFiles[recordingPath]?.append((name: fileName, size: size, modified: modified))
                    
                    // Track latest modified date for sorting
                    if let mod = modified {
                        if folderDates[recordingPath] == nil || mod > folderDates[recordingPath]! {
                            folderDates[recordingPath] = mod
                        }
                    }
                }
            }
        }
        
        var recordings: [DropboxRecordingInfo] = []
        
        for (path, files) in folderFiles {
            let name = (path as NSString).lastPathComponent
            let hasVideo = files.contains { $0.name == "video.mp4" }
            let hasMetadata = files.contains { $0.name == "metadata.json" }
            let hasTracking = files.contains { $0.name == "tracking.jsonl" }
            let hasSimulationData = files.contains { $0.name == "mjdata.jsonl" }
            let hasUSDZ = files.contains { $0.name == "scene.usdz" }
            let videoSize = files.first { $0.name == "video.mp4" }?.size
            let modifiedDate = folderDates[path]
            
            let info = DropboxRecordingInfo(
                id: name,
                name: name,
                path: path,
                hasVideo: hasVideo,
                hasMetadata: hasMetadata,
                hasTracking: hasTracking,
                hasSimulationData: hasSimulationData,
                hasUSDZ: hasUSDZ,
                modifiedDate: modifiedDate,
                videoSize: videoSize
            )
            recordings.append(info)
        }
        
        return recordings
    }
    
    /// Download a file from Dropbox to a local URL
    func downloadFile(path dropboxPath: String, to localURL: URL) async -> Bool {
        guard let accessToken = await getValidAccessToken() else {
            return false
        }
        
        let url = URL(string: "https://content.dropboxapi.com/2/files/download")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        
        let apiArg: [String: Any] = ["path": dropboxPath]
        if let apiArgData = try? JSONSerialization.data(withJSONObject: apiArg),
           let apiArgString = String(data: apiArgData, encoding: .utf8) {
            request.setValue(apiArgString, forHTTPHeaderField: "Dropbox-API-Arg")
        }
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                return false
            }
            
            try data.write(to: localURL)
            return true
            
        } catch {
            dlog("❌ [DropboxManager] Download error: \(error)")
            return false
        }
    }
    
    /// Download recording metadata from Dropbox
    func downloadMetadata(for recordingPath: String) async -> RecordingMetadata? {
        let metadataPath = "\(recordingPath)/metadata.json"
        
        guard let accessToken = await getValidAccessToken() else {
            return nil
        }
        
        let url = URL(string: "https://content.dropboxapi.com/2/files/download")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        
        let apiArg: [String: Any] = ["path": metadataPath]
        if let apiArgData = try? JSONSerialization.data(withJSONObject: apiArg),
           let apiArgString = String(data: apiArgData, encoding: .utf8) {
            request.setValue(apiArgString, forHTTPHeaderField: "Dropbox-API-Arg")
        }
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                return nil
            }
            
            let decoder = JSONDecoder()
            return try decoder.decode(RecordingMetadata.self, from: data)
            
        } catch {
            dlog("⚠️ [DropboxManager] Failed to download metadata: \(error)")
            return nil
        }
    }
    
    /// Get a temporary download link for a file (for video playback)
    func getTemporaryLink(for path: String) async -> String? {
        guard let accessToken = await getValidAccessToken() else {
            return nil
        }
        
        let url = URL(string: "https://api.dropboxapi.com/2/files/get_temporary_link")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        
        let body: [String: Any] = ["path": path]
        request.httpBody = try? JSONSerialization.data(withJSONObject: body)
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                return nil
            }
            
            if let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
               let link = json["link"] as? String {
                return link
            }
            
            return nil
            
        } catch {
            dlog("❌ [DropboxManager] Get temporary link error: \(error)")
            return nil
        }
    }
    
    /// Create a shared link for a file or folder (anyone with link can access)
    /// - Parameter path: Dropbox path to share
    /// - Returns: Shared link URL, or nil if failed
    func createSharedLink(path: String) async -> String? {
        guard let accessToken = await getValidAccessToken() else {
            return nil
        }
        
        // First, try to get existing shared link
        let listUrl = URL(string: "https://api.dropboxapi.com/2/sharing/list_shared_links")!
        var listRequest = URLRequest(url: listUrl)
        listRequest.httpMethod = "POST"
        listRequest.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        listRequest.setValue("application/json", forHTTPHeaderField: "Content-Type")
        
        let listBody: [String: Any] = ["path": path]
        listRequest.httpBody = try? JSONSerialization.data(withJSONObject: listBody)
        
        do {
            let (data, response) = try await URLSession.shared.data(for: listRequest)
            
            if let httpResponse = response as? HTTPURLResponse,
               httpResponse.statusCode == 200,
               let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
               let links = json["links"] as? [[String: Any]],
               let firstLink = links.first,
               let url = firstLink["url"] as? String {
                dlog("✅ [DropboxManager] Found existing shared link for: \(path)")
                return url
            }
        } catch {
            dlog("⚠️ [DropboxManager] No existing shared link found, creating new one")
        }
        
        // Create new shared link
        let createUrl = URL(string: "https://api.dropboxapi.com/2/sharing/create_shared_link_with_settings")!
        var createRequest = URLRequest(url: createUrl)
        createRequest.httpMethod = "POST"
        createRequest.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        createRequest.setValue("application/json", forHTTPHeaderField: "Content-Type")
        
        let createBody: [String: Any] = [
            "path": path,
            "settings": [
                "requested_visibility": "public",
                "audience": "public",
                "access": "viewer"
            ]
        ]
        createRequest.httpBody = try? JSONSerialization.data(withJSONObject: createBody)
        
        do {
            let (data, response) = try await URLSession.shared.data(for: createRequest)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                dlog("❌ [DropboxManager] Failed to create shared link")
                return nil
            }
            
            if let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
               let url = json["url"] as? String {
                dlog("✅ [DropboxManager] Created shared link for: \(path)")
                return url
            }
            
            return nil
            
        } catch {
            dlog("❌ [DropboxManager] Create shared link error: \(error)")
            return nil
        }
    }
    
    /// Delete a folder/file at the specified path
    func deleteFolder(path: String) async throws {
        guard let accessToken = await getValidAccessToken() else {
            throw NSError(domain: "DropboxManager", code: -1, userInfo: [NSLocalizedDescriptionKey: "Not authenticated"])
        }
        
        let url = URL(string: "https://api.dropboxapi.com/2/files/delete_v2")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        
        let body: [String: Any] = ["path": path]
        request.httpBody = try JSONSerialization.data(withJSONObject: body)
        
        let (data, response) = try await URLSession.shared.data(for: request)
        
        guard let httpResponse = response as? HTTPURLResponse else {
            throw NSError(domain: "DropboxManager", code: -1, userInfo: [NSLocalizedDescriptionKey: "Invalid response"])
        }
        
        if httpResponse.statusCode != 200 {
            if let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
               let errorSummary = json["error_summary"] as? String {
                throw NSError(domain: "DropboxManager", code: httpResponse.statusCode, userInfo: [NSLocalizedDescriptionKey: errorSummary])
            }
            throw NSError(domain: "DropboxManager", code: httpResponse.statusCode, userInfo: [NSLocalizedDescriptionKey: "Delete failed with status \(httpResponse.statusCode)"])
        }
        
        dlog("✅ [DropboxManager] Deleted: \(path)")
    }
    
    // MARK: - PKCE Helpers
    
    private func generateCodeVerifier() -> String {
        var bytes = [UInt8](repeating: 0, count: 32)
        _ = SecRandomCopyBytes(kSecRandomDefault, bytes.count, &bytes)
        return Data(bytes).base64EncodedString()
            .replacingOccurrences(of: "+", with: "-")
            .replacingOccurrences(of: "/", with: "_")
            .replacingOccurrences(of: "=", with: "")
            .prefix(128)
            .description
    }
    
    private func generateCodeChallenge(from verifier: String) -> String {
        let data = Data(verifier.utf8)
        let hashed = SHA256.hash(data: data)
        return Data(hashed).base64EncodedString()
            .replacingOccurrences(of: "+", with: "-")
            .replacingOccurrences(of: "/", with: "_")
            .replacingOccurrences(of: "=", with: "")
    }
    
    private func extractCode(from url: URL) -> String? {
        guard let components = URLComponents(url: url, resolvingAgainstBaseURL: true),
              let queryItems = components.queryItems else {
            return nil
        }
        return queryItems.first(where: { $0.name == "code" })?.value
    }
}

// MARK: - ASWebAuthenticationSession Presentation Context

extension DropboxManager: ASWebAuthenticationPresentationContextProviding {
    nonisolated func presentationAnchor(for session: ASWebAuthenticationSession) -> ASPresentationAnchor {
        // Return the first window scene's window
        let scenes = UIApplication.shared.connectedScenes
        let windowScene = scenes.first as? UIWindowScene
        return windowScene?.windows.first ?? ASPresentationAnchor()
    }
}

// MARK: - Token Response Model

struct DropboxTokenResponse: Codable {
    let accessToken: String
    let tokenType: String
    let expiresIn: TimeInterval?
    let refreshToken: String?
    let scope: String?
    let uid: String?
    let accountId: String?
    
    enum CodingKeys: String, CodingKey {
        case accessToken = "access_token"
        case tokenType = "token_type"
        case expiresIn = "expires_in"
        case refreshToken = "refresh_token"
        case scope
        case uid
        case accountId = "account_id"
    }
}

// MARK: - Dropbox Recording Info

struct DropboxRecordingInfo: Identifiable, Hashable {
    let id: String
    let name: String
    let path: String
    let hasVideo: Bool
    let hasMetadata: Bool
    let hasTracking: Bool
    let hasSimulationData: Bool
    let hasUSDZ: Bool
    let modifiedDate: Date?
    let videoSize: Int64?
    
    var displayName: String {
        // Format: "recording_20241125_143000" -> "2024-11-25 14:30:00"
        if name.hasPrefix("recording_") {
            let dateStr = String(name.dropFirst(10))
            let formatter = DateFormatter()
            formatter.dateFormat = "yyyyMMdd_HHmmss"
            if let date = formatter.date(from: dateStr) {
                let displayFormatter = DateFormatter()
                displayFormatter.dateStyle = .medium
                displayFormatter.timeStyle = .medium
                return displayFormatter.string(from: date)
            }
        }
        return name
    }
}
