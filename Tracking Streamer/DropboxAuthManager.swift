//
//  DropboxAuthManager.swift
//  Tracking Streamer (visionOS)
//
//  Handles Dropbox OAuth authentication directly on visionOS.
//  Tokens are saved to iCloud Keychain and automatically sync to iOS.
//

import Foundation
import AuthenticationServices
import CryptoKit
import Combine
import UIKit

/// Manages Dropbox OAuth authentication on visionOS
@MainActor
class DropboxAuthManager: NSObject, ObservableObject {
    static let shared = DropboxAuthManager()
    
    // MARK: - Dropbox App Configuration
    
    /// Dropbox App Key (same as iOS app for compatibility)
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
    private let keychain = KeychainManager.shared
    
    // MARK: - Published Properties
    
    @Published var isAuthenticating: Bool = false
    @Published var authError: String?
    
    /// Check if user is authenticated with Dropbox
    var isAuthenticated: Bool {
        return keychain.exists(key: .dropboxAccessToken)
    }
    
    private override init() {
        super.init()
    }
    
    // MARK: - OAuth Flow
    
    /// Start the Dropbox OAuth flow using ASWebAuthenticationSession
    func startOAuthFlow() async {
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
        
        dlog("🔐 [DropboxAuthManager] Starting OAuth flow on visionOS...")
        
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
                        dlog("❌ [DropboxAuthManager] Auth error: \(error)")
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
                dlog("❌ [DropboxAuthManager] Token exchange failed: HTTP \(statusCode)")
                if let body = String(data: data, encoding: .utf8) {
                    dlog("   Response: \(body)")
                }
                return
            }
            
            let tokenResponse = try JSONDecoder().decode(DropboxTokenResponse.self, from: data)
            
            // Save tokens to keychain (will sync to iOS via iCloud Keychain)
            keychain.save(tokenResponse.accessToken, forKey: .dropboxAccessToken)
            
            if let refreshToken = tokenResponse.refreshToken {
                keychain.save(refreshToken, forKey: .dropboxRefreshToken)
            }
            
            if let expiresIn = tokenResponse.expiresIn {
                let expiryDate = Date().addingTimeInterval(expiresIn)
                keychain.save(String(expiryDate.timeIntervalSince1970), forKey: .dropboxTokenExpiry)
            }
            
            // Also save the selected provider
            keychain.save(CloudStorageProvider.dropbox.rawValue, forKey: .selectedCloudProvider)
            
            codeVerifier = nil
            dlog("✅ [DropboxAuthManager] Token exchange successful - tokens synced to iCloud Keychain")
            
            // Reload cloud storage settings to reflect the change
            CloudStorageSettings.shared.loadSettings()
            
        } catch {
            authError = "Token exchange failed: \(error.localizedDescription)"
            dlog("❌ [DropboxAuthManager] Token exchange error: \(error)")
        }
    }
    
    /// Sign out from Dropbox (remove tokens)
    func signOut() {
        keychain.delete(key: .dropboxAccessToken)
        keychain.delete(key: .dropboxRefreshToken)
        keychain.delete(key: .dropboxTokenExpiry)
        
        // Reset to iCloud if Dropbox was selected
        if CloudStorageSettings.shared.selectedProvider == .dropbox {
            keychain.save(CloudStorageProvider.iCloudDrive.rawValue, forKey: .selectedCloudProvider)
        }
        
        CloudStorageSettings.shared.loadSettings()
        dlog("✅ [DropboxAuthManager] Signed out from Dropbox")
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

extension DropboxAuthManager: ASWebAuthenticationPresentationContextProviding {
    nonisolated func presentationAnchor(for session: ASWebAuthenticationSession) -> ASPresentationAnchor {
        // Return a window for visionOS - use first available window
        #if os(visionOS)
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

/// Dropbox OAuth token response
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
