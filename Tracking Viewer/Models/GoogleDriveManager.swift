//
//  GoogleDriveManager.swift
//  Tracking Viewer
//
//  Handles Google Drive OAuth authentication and API operations.
//  Uses OAuth 2.0 with PKCE for secure authentication.
//

import Foundation
import AuthenticationServices
import CryptoKit
import Combine

/// Manages Google Drive OAuth and file uploads
@MainActor
class GoogleDriveManager: NSObject, ObservableObject {
    static let shared = GoogleDriveManager()
    
    // MARK: - Google Cloud Configuration
    
    /// Your Google OAuth Client ID for iOS
    /// Get this from Google Cloud Console: https://console.cloud.google.com/
    /// Create OAuth 2.0 credentials for iOS app
    private let clientID = "613801329299-jcqrgumln3s9udtomsll5dahcbjcivej.apps.googleusercontent.com"
    
    /// Redirect URI for mobile OAuth with PKCE
    /// Format: com.googleusercontent.apps.{CLIENT_ID}:/oauth2redirect
    private var redirectURI: String {
        let reversedClientID = clientID.components(separatedBy: ".").reversed().joined(separator: ".")
        return "\(reversedClientID):/oauth2redirect"
    }
    
    /// URL scheme for ASWebAuthenticationSession callback
    private var callbackURLScheme: String {
        return clientID.components(separatedBy: ".").reversed().joined(separator: ".")
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
    private var nextPageToken: String?
    private let pageSize = 20
    
    /// Check if user is authenticated with Google Drive (has stored tokens)
    var isAuthenticated: Bool {
        return KeychainManager.shared.loadString(forKey: .googleDriveAccessToken) != nil
    }
    
    private override init() {
        super.init()
    }
    
    // MARK: - OAuth Flow
    
    /// Start the Google Drive OAuth flow using ASWebAuthenticationSession
    func startOAuthFlow() async {
        guard !clientID.contains("YOUR_GOOGLE_CLIENT_ID") else {
            authError = "Google Client ID not configured. Please set up a Google Cloud project."
            print("❌ [GoogleDriveManager] Client ID not configured")
            return
        }
        
        isAuthenticating = true
        authError = nil
        
        // Generate PKCE code verifier and challenge
        let verifier = generateCodeVerifier()
        codeVerifier = verifier
        let challenge = generateCodeChallenge(from: verifier)
        
        // Build authorization URL
        var components = URLComponents(string: "https://accounts.google.com/o/oauth2/v2/auth")!
        components.queryItems = [
            URLQueryItem(name: "client_id", value: clientID),
            URLQueryItem(name: "response_type", value: "code"),
            URLQueryItem(name: "redirect_uri", value: redirectURI),
            URLQueryItem(name: "code_challenge", value: challenge),
            URLQueryItem(name: "code_challenge_method", value: "S256"),
            URLQueryItem(name: "scope", value: "https://www.googleapis.com/auth/drive.file"),
            URLQueryItem(name: "access_type", value: "offline"), // Get refresh token
            URLQueryItem(name: "prompt", value: "consent"), // Always prompt for consent to get refresh token
        ]
        
        guard let authURL = components.url else {
            authError = "Failed to build authorization URL"
            isAuthenticating = false
            return
        }
        
        print("🔐 [GoogleDriveManager] Starting OAuth flow...")
        
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
                        print("❌ [GoogleDriveManager] Auth error: \(error)")
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
            print("❌ [GoogleDriveManager] No code in callback URL")
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
        
        let url = URL(string: "https://oauth2.googleapis.com/token")!
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.setValue("application/x-www-form-urlencoded", forHTTPHeaderField: "Content-Type")
        
        let body = [
            "code": code,
            "grant_type": "authorization_code",
            "client_id": clientID,
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
                print("❌ [GoogleDriveManager] Token exchange failed: HTTP \(statusCode)")
                if let body = String(data: data, encoding: .utf8) {
                    print("   Response: \(body)")
                }
                return
            }
            
            let tokenResponse = try JSONDecoder().decode(GoogleTokenResponse.self, from: data)
            
            // Save tokens via CloudStorageManager (which saves to keychain)
            CloudStorageManager.shared.onGoogleDriveAuthSuccess(
                accessToken: tokenResponse.accessToken,
                refreshToken: tokenResponse.refreshToken,
                expiresIn: tokenResponse.expiresIn
            )
            
            codeVerifier = nil
            print("✅ [GoogleDriveManager] Token exchange successful")
            
        } catch {
            authError = "Token exchange failed: \(error.localizedDescription)"
            print("❌ [GoogleDriveManager] Token exchange error: \(error)")
        }
    }
    
    /// Refresh the access token using the refresh token
    func refreshAccessToken() async -> String? {
        guard let refreshToken = KeychainManager.shared.loadString(forKey: .googleDriveRefreshToken) else {
            print("❌ [GoogleDriveManager] No refresh token available")
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
                print("❌ [GoogleDriveManager] Token refresh failed")
                return nil
            }
            
            let tokenResponse = try JSONDecoder().decode(GoogleTokenResponse.self, from: data)
            
            // Save new access token
            KeychainManager.shared.save(tokenResponse.accessToken, forKey: .googleDriveAccessToken)
            
            if let expiresIn = tokenResponse.expiresIn {
                let expiryDate = Date().addingTimeInterval(expiresIn)
                KeychainManager.shared.save(String(expiryDate.timeIntervalSince1970), forKey: .googleDriveTokenExpiry)
            }
            
            print("✅ [GoogleDriveManager] Token refreshed")
            return tokenResponse.accessToken
            
        } catch {
            print("❌ [GoogleDriveManager] Token refresh error: \(error)")
            return nil
        }
    }
    
    // MARK: - API Operations
    
    /// Get current access token, refreshing if needed
    func getValidAccessToken() async -> String? {
        guard let accessToken = KeychainManager.shared.loadString(forKey: .googleDriveAccessToken) else {
            return nil
        }
        
        // Check if token is expired
        if let expiryString = KeychainManager.shared.loadString(forKey: .googleDriveTokenExpiry),
           let expiryTimestamp = Double(expiryString) {
            let expiryDate = Date(timeIntervalSince1970: expiryTimestamp)
            
            // Refresh if expiring within 5 minutes
            if expiryDate.timeIntervalSinceNow < 300 {
                print("🔄 [GoogleDriveManager] Token expiring soon, refreshing...")
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
        
        let url = URL(string: "https://www.googleapis.com/oauth2/v2/userinfo")!
        var request = URLRequest(url: url)
        request.httpMethod = "GET"
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
            print("❌ [GoogleDriveManager] Failed to get account info: \(error)")
            return nil
        }
    }
    
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
                print("❌ [GoogleDriveManager] Failed to search for folder")
                return nil
            }
            
            if let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
               let files = json["files"] as? [[String: Any]],
               let firstFolder = files.first,
               let folderId = firstFolder["id"] as? String {
                print("✅ [GoogleDriveManager] Found existing VisionProTeleop folder: \(folderId)")
                return folderId
            }
            
            // Create folder if it doesn't exist
            return await createFolder(name: "VisionProTeleop", parentId: nil)
            
        } catch {
            print("❌ [GoogleDriveManager] Error searching for folder: \(error)")
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
                print("❌ [GoogleDriveManager] Failed to create folder")
                return nil
            }
            
            if let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
               let folderId = json["id"] as? String {
                print("✅ [GoogleDriveManager] Created folder: \(name) with ID: \(folderId)")
                return folderId
            }
            
            return nil
            
        } catch {
            print("❌ [GoogleDriveManager] Error creating folder: \(error)")
            return nil
        }
    }
    
    /// Upload a file to Google Drive
    /// - Parameters:
    ///   - fileURL: Local file URL to upload
    ///   - fileName: Name for the file in Google Drive
    ///   - parentFolderId: Parent folder ID (optional)
    /// - Returns: Whether the upload succeeded
    func uploadFile(from fileURL: URL, fileName: String, parentFolderId: String?) async -> Bool {
        guard let accessToken = await getValidAccessToken() else {
            print("❌ [GoogleDriveManager] No valid access token for upload")
            return false
        }
        
        // Read file data
        guard let fileData = try? Data(contentsOf: fileURL) else {
            print("❌ [GoogleDriveManager] Failed to read file: \(fileURL)")
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
                print("❌ [GoogleDriveManager] Upload failed: HTTP \(statusCode)")
                return false
            }
            
            print("✅ [GoogleDriveManager] Uploaded \(fileName)")
            return true
            
        } catch {
            print("❌ [GoogleDriveManager] Upload error: \(error)")
            return false
        }
    }
    
    /// Upload a folder (recording) to Google Drive
    func uploadRecording(folderURL: URL, recordingName: String) async -> Bool {
        // Get or create app folder
        guard let appFolderId = await getOrCreateAppFolder() else {
            print("❌ [GoogleDriveManager] Failed to get app folder")
            return false
        }
        
        // Create recording folder
        guard let recordingFolderId = await createFolder(name: recordingName, parentId: appFolderId) else {
            print("❌ [GoogleDriveManager] Failed to create recording folder")
            return false
        }
        
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
            
            let fileName = fileURL.lastPathComponent
            
            if await !uploadFile(from: fileURL, fileName: fileName, parentFolderId: recordingFolderId) {
                success = false
            }
        }
        
        return success
    }
    
    /// List recording folders in Google Drive (initial fetch with pagination)
    func listRecordings() async -> [GoogleDriveRecordingInfo] {
        // Reset pagination state
        nextPageToken = nil
        await MainActor.run {
            hasMoreRecordings = true
        }
        
        guard let accessToken = await getValidAccessToken() else {
            print("❌ [GoogleDriveManager] No valid access token for listing")
            return []
        }
        
        // First get the app folder
        guard let appFolderId = await getOrCreateAppFolder() else {
            return []
        }
        
        // List folders in app folder with pagination
        let url = URL(string: "https://www.googleapis.com/drive/v3/files")!
        var components = URLComponents(url: url, resolvingAgainstBaseURL: false)!
        components.queryItems = [
            URLQueryItem(name: "q", value: "'\(appFolderId)' in parents and mimeType='application/vnd.google-apps.folder' and trashed=false"),
            URLQueryItem(name: "spaces", value: "drive"),
            URLQueryItem(name: "fields", value: "nextPageToken, files(id, name, modifiedTime)"),
            URLQueryItem(name: "pageSize", value: String(pageSize)),
            URLQueryItem(name: "orderBy", value: "modifiedTime desc")
        ]
        
        var request = URLRequest(url: components.url!)
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                return []
            }
            
            guard let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
                  let files = json["files"] as? [[String: Any]] else {
                return []
            }
            
            // Store pagination token
            if let token = json["nextPageToken"] as? String {
                nextPageToken = token
                await MainActor.run {
                    self.hasMoreRecordings = true
                }
            } else {
                nextPageToken = nil
                await MainActor.run {
                    self.hasMoreRecordings = false
                }
            }
            
            let recordings = await parseGoogleDriveFiles(files)
            print("✅ [GoogleDriveManager] Found \(recordings.count) recordings in Google Drive (hasMore: \(hasMoreRecordings))")
            return recordings
            
        } catch {
            print("❌ [GoogleDriveManager] List recordings error: \(error)")
            return []
        }
    }
    
    /// Load more recordings from Google Drive (pagination)
    func loadMoreRecordings() async -> [GoogleDriveRecordingInfo] {
        guard !isLoadingMore, hasMoreRecordings, let pageToken = nextPageToken else {
            return []
        }
        
        await MainActor.run {
            isLoadingMore = true
        }
        defer {
            Task { @MainActor in
                self.isLoadingMore = false
            }
        }
        
        guard let accessToken = await getValidAccessToken() else {
            return []
        }
        
        guard let appFolderId = await getOrCreateAppFolder() else {
            return []
        }
        
        let url = URL(string: "https://www.googleapis.com/drive/v3/files")!
        var components = URLComponents(url: url, resolvingAgainstBaseURL: false)!
        components.queryItems = [
            URLQueryItem(name: "q", value: "'\(appFolderId)' in parents and mimeType='application/vnd.google-apps.folder' and trashed=false"),
            URLQueryItem(name: "spaces", value: "drive"),
            URLQueryItem(name: "fields", value: "nextPageToken, files(id, name, modifiedTime)"),
            URLQueryItem(name: "pageSize", value: String(pageSize)),
            URLQueryItem(name: "orderBy", value: "modifiedTime desc"),
            URLQueryItem(name: "pageToken", value: pageToken)
        ]
        
        var request = URLRequest(url: components.url!)
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                return []
            }
            
            guard let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
                  let files = json["files"] as? [[String: Any]] else {
                return []
            }
            
            // Update pagination token
            if let token = json["nextPageToken"] as? String {
                nextPageToken = token
                await MainActor.run {
                    self.hasMoreRecordings = true
                }
            } else {
                nextPageToken = nil
                await MainActor.run {
                    self.hasMoreRecordings = false
                }
            }
            
            let recordings = await parseGoogleDriveFiles(files)
            print("✅ [GoogleDriveManager] Loaded \(recordings.count) more recordings (hasMore: \(hasMoreRecordings))")
            return recordings
            
        } catch {
            print("❌ [GoogleDriveManager] Load more error: \(error)")
            return []
        }
    }
    
    /// Parse Google Drive file entries into GoogleDriveRecordingInfo array
    private func parseGoogleDriveFiles(_ files: [[String: Any]]) async -> [GoogleDriveRecordingInfo] {
        var recordings: [GoogleDriveRecordingInfo] = []
        
        for file in files {
            guard let id = file["id"] as? String,
                  let name = file["name"] as? String else {
                continue
            }
            
            var modifiedDate: Date?
            if let modifiedTime = file["modifiedTime"] as? String {
                let formatter = ISO8601DateFormatter()
                formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
                modifiedDate = formatter.date(from: modifiedTime)
                if modifiedDate == nil {
                    formatter.formatOptions = [.withInternetDateTime]
                    modifiedDate = formatter.date(from: modifiedTime)
                }
            }
            
            var info = GoogleDriveRecordingInfo(
                id: id,
                name: name,
                modifiedDate: modifiedDate
            )
            
            // Get files in this recording folder
            let folderContents = await listFilesInFolder(folderId: id)
            for content in folderContents {
                if content.name == "video.mp4" {
                    info.hasVideo = true
                    info.videoFileId = content.id
                } else if content.name == "tracking.jsonl" {
                    info.hasTrackingData = true
                    info.trackingDataFileId = content.id
                } else if content.name == "metadata.json" {
                    info.hasMetadata = true
                    info.metadataFileId = content.id
                } else if content.name == "mjdata.jsonl" {
                    info.hasSimulationData = true
                    info.simulationDataFileId = content.id
                } else if content.name == "scene.usdz" {
                    info.hasUSDZ = true
                    info.usdzFileId = content.id
                }
            }
            
            recordings.append(info)
        }
        
        return recordings
    }
    

    
    /// Get file IDs for video, tracking data, simulation data, and USDZ in a recording folder
    func getRecordingFileIds(folderId: String) async -> (videoFileId: String?, trackingFileId: String?, simulationFileId: String?, usdzFileId: String?) {
        let files = await listFilesInFolder(folderId: folderId)
        
        var videoId: String?
        var trackingId: String?
        var simulationId: String?
        var usdzId: String?
        
        for file in files {
            if file.name == "video.mp4" {
                videoId = file.id
            } else if file.name == "tracking.jsonl" {
                trackingId = file.id
            } else if file.name == "mjdata.jsonl" {
                simulationId = file.id
            } else if file.name == "scene.usdz" {
                usdzId = file.id
            }
        }
        
        return (videoId, trackingId, simulationId, usdzId)
    }
    
    /// List files in a folder
    private func listFilesInFolder(folderId: String) async -> [(id: String, name: String)] {
        guard let accessToken = await getValidAccessToken() else {
            return []
        }
        
        let url = URL(string: "https://www.googleapis.com/drive/v3/files")!
        var components = URLComponents(url: url, resolvingAgainstBaseURL: false)!
        components.queryItems = [
            URLQueryItem(name: "q", value: "'\(folderId)' in parents and trashed=false"),
            URLQueryItem(name: "spaces", value: "drive"),
            URLQueryItem(name: "fields", value: "files(id, name)")
        ]
        
        var request = URLRequest(url: components.url!)
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                return []
            }
            
            guard let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
                  let files = json["files"] as? [[String: Any]] else {
                return []
            }
            
            return files.compactMap { file -> (id: String, name: String)? in
                guard let id = file["id"] as? String,
                      let name = file["name"] as? String else {
                    return nil
                }
                return (id: id, name: name)
            }
            
        } catch {
            return []
        }
    }
    
    /// Download a file from Google Drive
    func downloadFile(fileId: String, to localURL: URL) async -> Bool {
        guard let accessToken = await getValidAccessToken() else {
            return false
        }
        
        let url = URL(string: "https://www.googleapis.com/drive/v3/files/\(fileId)?alt=media")!
        var request = URLRequest(url: url)
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        
        do {
            let (data, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse,
                  httpResponse.statusCode == 200 else {
                return false
            }
            
            try data.write(to: localURL)
            return true
            
        } catch {
            print("❌ [GoogleDriveManager] Download error: \(error)")
            return false
        }
    }
    
    /// Create a shared link for a file or folder (anyone with link can access)
    /// - Parameter fileId: Google Drive file/folder ID to share
    /// - Returns: Shared link URL, or nil if failed
    func createSharedLink(fileId: String) async -> String? {
        guard let accessToken = await getValidAccessToken() else {
            return nil
        }
        
        // Update file permissions to make it publicly accessible
        let permissionsUrl = URL(string: "https://www.googleapis.com/drive/v3/files/\(fileId)/permissions")!
        var permissionsRequest = URLRequest(url: permissionsUrl)
        permissionsRequest.httpMethod = "POST"
        permissionsRequest.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        permissionsRequest.setValue("application/json", forHTTPHeaderField: "Content-Type")
        
        let permissionsBody: [String: Any] = [
            "role": "reader",
            "type": "anyone"
        ]
        permissionsRequest.httpBody = try? JSONSerialization.data(withJSONObject: permissionsBody)
        
        do {
            let (permissionsData, response) = try await URLSession.shared.data(for: permissionsRequest)
            
            if let httpResponse = response as? HTTPURLResponse,
               (httpResponse.statusCode == 200 || httpResponse.statusCode == 201) {
                print("✅ [GoogleDriveManager] Permissions updated successfully")
            } else {
                // Permission update failed, but we'll try to get the link anyway
                // This handles cases where it's already shared or we have partial access issues (like 403 appNotAuthorizedToChild)
                if let errorJson = try? JSONSerialization.jsonObject(with: permissionsData) as? [String: Any] {
                    print("⚠️ [GoogleDriveManager] Warning: Failed to set public permissions. Status: \((response as? HTTPURLResponse)?.statusCode ?? 0). Body: \(errorJson)")
                } else {
                    print("⚠️ [GoogleDriveManager] Warning: Failed to set public permissions. Status: \((response as? HTTPURLResponse)?.statusCode ?? 0)")
                }
            }
            
            // Get the web view link (proceed regardless of permission update success)
            let fileUrl = URL(string: "https://www.googleapis.com/drive/v3/files/\(fileId)?fields=webViewLink")!
            var fileRequest = URLRequest(url: fileUrl)
            fileRequest.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
            
            let (data, fileResponse) = try await URLSession.shared.data(for: fileRequest)
            
            guard let httpFileResponse = fileResponse as? HTTPURLResponse,
                  httpFileResponse.statusCode == 200 else {
                print("❌ [GoogleDriveManager] Failed to get file link. Status: \((fileResponse as? HTTPURLResponse)?.statusCode ?? 0)")
                return nil
            }
            
            if let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
               let webViewLink = json["webViewLink"] as? String {
                print("✅ [GoogleDriveManager] Retrieved shared link for file: \(fileId)")
                return webViewLink
            }
            
            return nil
            
        } catch {
            print("❌ [GoogleDriveManager] Create shared link error: \(error)")
            return nil
        }
    }
    
    /// Delete a folder from Google Drive
    /// - Parameter folderId: The ID of the folder to delete
    /// - Throws: Error if deletion fails
    func deleteFolder(folderId: String) async throws {
        guard let accessToken = await getValidAccessToken() else {
            throw NSError(domain: "GoogleDriveManager", code: -1, userInfo: [NSLocalizedDescriptionKey: "No valid access token"])
        }
        
        let url = URL(string: "https://www.googleapis.com/drive/v3/files/\(folderId)")!
        var request = URLRequest(url: url)
        request.httpMethod = "DELETE"
        request.setValue("Bearer \(accessToken)", forHTTPHeaderField: "Authorization")
        
        do {
            let (_, response) = try await URLSession.shared.data(for: request)
            
            guard let httpResponse = response as? HTTPURLResponse else {
                throw NSError(domain: "GoogleDriveManager", code: -1, userInfo: [NSLocalizedDescriptionKey: "Invalid response"])
            }
            
            // Google Drive API returns 204 No Content on successful deletion
            guard httpResponse.statusCode == 204 else {
                let errorMessage = "Failed to delete folder (HTTP \(httpResponse.statusCode))"
                print("❌ [GoogleDriveManager] \(errorMessage)")
                throw NSError(domain: "GoogleDriveManager", code: httpResponse.statusCode, userInfo: [NSLocalizedDescriptionKey: errorMessage])
            }
            
            print("✅ [GoogleDriveManager] Deleted folder with ID: \(folderId)")
            
        } catch let error as NSError {
            print("❌ [GoogleDriveManager] Delete folder error: \(error)")
            throw error
        }
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

extension GoogleDriveManager: ASWebAuthenticationPresentationContextProviding {
    nonisolated func presentationAnchor(for session: ASWebAuthenticationSession) -> ASPresentationAnchor {
        // Return the first window scene's window
        let scenes = UIApplication.shared.connectedScenes
        let windowScene = scenes.first as? UIWindowScene
        return windowScene?.windows.first ?? ASPresentationAnchor()
    }
}

// MARK: - Token Response Model

struct GoogleTokenResponse: Codable {
    let accessToken: String
    let tokenType: String
    let expiresIn: TimeInterval?
    let refreshToken: String?
    let scope: String?
    
    enum CodingKeys: String, CodingKey {
        case accessToken = "access_token"
        case tokenType = "token_type"
        case expiresIn = "expires_in"
        case refreshToken = "refresh_token"
        case scope
    }
}

// MARK: - Google Drive Recording Info

struct GoogleDriveRecordingInfo: Identifiable, Hashable {
    let id: String
    let name: String
    let modifiedDate: Date?
    var hasVideo: Bool = false
    var videoFileId: String?
    var hasTrackingData: Bool = false
    var trackingDataFileId: String?
    var hasMetadata: Bool = false
    var metadataFileId: String?
    var hasSimulationData: Bool = false
    var simulationDataFileId: String?
    var hasUSDZ: Bool = false
    var usdzFileId: String?
    
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
