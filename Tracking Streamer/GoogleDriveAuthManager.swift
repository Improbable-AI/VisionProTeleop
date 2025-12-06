//
//  GoogleDriveAuthManager.swift
//  Tracking Streamer (visionOS)
//
//  Handles Google Drive OAuth authentication directly on visionOS.
//  Tokens are saved to iCloud Keychain and automatically sync to iOS.
//

import Foundation
import AuthenticationServices
import CryptoKit
import Combine

/// Manages Google Drive OAuth authentication on visionOS
@MainActor
class GoogleDriveAuthManager: NSObject, ObservableObject {
    static let shared = GoogleDriveAuthManager()
    
    // MARK: - Google Cloud Configuration
    
    /// Google OAuth Client ID (same as iOS app for compatibility)
    private let clientID = "613801329299-jcqrgumln3s9udtomsll5dahcbjcivej.apps.googleusercontent.com"
    
    /// Redirect URI for mobile OAuth with PKCE\
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
    private let keychain = KeychainManager.shared
    
    // MARK: - Published Properties
    
    @Published var isAuthenticating: Bool = false
    @Published var authError: String?
    
    /// Check if user is authenticated with Google Drive
    var isAuthenticated: Bool {
        return keychain.exists(key: .googleDriveAccessToken)
    }
    
    private override init() {
        super.init()
    }
    
    // MARK: - OAuth Flow
    
    /// Start the Google Drive OAuth flow using ASWebAuthenticationSession
    func startOAuthFlow() async {
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
        
        print("🔐 [GoogleDriveAuthManager] Starting OAuth flow on visionOS...")
        
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
                        print("❌ [GoogleDriveAuthManager] Auth error: \(error)")
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
                print("❌ [GoogleDriveAuthManager] Token exchange failed: HTTP \(statusCode)")
                if let body = String(data: data, encoding: .utf8) {
                    print("   Response: \(body)")
                }
                return
            }
            
            let tokenResponse = try JSONDecoder().decode(GoogleTokenResponse.self, from: data)
            
            // Save tokens to keychain (will sync to iOS via iCloud Keychain)
            keychain.save(tokenResponse.accessToken, forKey: .googleDriveAccessToken)
            
            if let refreshToken = tokenResponse.refreshToken {
                keychain.save(refreshToken, forKey: .googleDriveRefreshToken)
            }
            
            if let expiresIn = tokenResponse.expiresIn {
                let expiryDate = Date().addingTimeInterval(expiresIn)
                keychain.save(String(expiryDate.timeIntervalSince1970), forKey: .googleDriveTokenExpiry)
            }
            
            // Also save the selected provider
            keychain.save(CloudStorageProvider.googleDrive.rawValue, forKey: .selectedCloudProvider)
            
            codeVerifier = nil
            print("✅ [GoogleDriveAuthManager] Token exchange successful - tokens synced to iCloud Keychain")
            
            // Reload cloud storage settings to reflect the change
            CloudStorageSettings.shared.loadSettings()
            
        } catch {
            authError = "Token exchange failed: \(error.localizedDescription)"
            print("❌ [GoogleDriveAuthManager] Token exchange error: \(error)")
        }
    }
    
    /// Sign out from Google Drive (remove tokens)
    func signOut() {
        keychain.delete(key: .googleDriveAccessToken)
        keychain.delete(key: .googleDriveRefreshToken)
        keychain.delete(key: .googleDriveTokenExpiry)
        
        // Reset to iCloud if Google Drive was selected
        if CloudStorageSettings.shared.selectedProvider == .googleDrive {
            keychain.save(CloudStorageProvider.iCloudDrive.rawValue, forKey: .selectedCloudProvider)
        }
        
        CloudStorageSettings.shared.loadSettings()
        print("✅ [GoogleDriveAuthManager] Signed out from Google Drive")
    }
    
    // MARK: - PKCE Helpers
    
    /// Generate a cryptographically random code verifier
    private func generateCodeVerifier() -> String {
        var buffer = [UInt8](repeating: 0, count: 32)
        _ = SecRandomCopyBytes(kSecRandomDefault, buffer.count, &buffer)
        return Data(buffer).base64EncodedString()
            .replacingOccurrences(of: "+", with: "-")
            .replacingOccurrences(of: "/", with: "_")
            .replacingOccurrences(of: "=", with: "")
    }
    
    /// Generate code challenge from verifier using SHA256
    private func generateCodeChallenge(from verifier: String) -> String {
        let data = Data(verifier.utf8)
        let hash = SHA256.hash(data: data)
        return Data(hash).base64EncodedString()
            .replacingOccurrences(of: "+", with: "-")
            .replacingOccurrences(of: "/", with: "_")
            .replacingOccurrences(of: "=", with: "")
    }
    
    /// Extract authorization code from callback URL
    private func extractCode(from url: URL) -> String? {
        guard let components = URLComponents(url: url, resolvingAgainstBaseURL: false),
              let code = components.queryItems?.first(where: { $0.name == "code" })?.value else {
            return nil
        }
        return code
    }
}

// MARK: - ASWebAuthenticationPresentationContextProviding

extension GoogleDriveAuthManager: ASWebAuthenticationPresentationContextProviding {
    nonisolated func presentationAnchor(for session: ASWebAuthenticationSession) -> ASPresentationAnchor {
        // Return a window for visionOS - use first available window
        #if os(visionOS)
        // On visionOS, we need to get a window from the scene
        if let windowScene = UIApplication.shared.connectedScenes.first as? UIWindowScene,
           let window = windowScene.windows.first {
            return window
        }
        // Fallback: create a new window
        return UIWindow()
        #else
        return UIWindow()
        #endif
    }
}

// MARK: - Token Response Model

/// Google OAuth token response
struct GoogleTokenResponse: Codable {
    let accessToken: String
    let refreshToken: String?
    let expiresIn: TimeInterval?
    let tokenType: String?
    let scope: String?
    
    enum CodingKeys: String, CodingKey {
        case accessToken = "access_token"
        case refreshToken = "refresh_token"
        case expiresIn = "expires_in"
        case tokenType = "token_type"
        case scope
    }
}
