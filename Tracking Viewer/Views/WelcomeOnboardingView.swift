//
//  WelcomeOnboardingView.swift
//  Tracking Viewer
//
//  First-time user onboarding flow with cloud storage setup and Vision Pro promotion.
//

import SwiftUI

// MARK: - Onboarding State Keys
private let hasSeenWelcomeKey = "hasSeenWelcome"
private let dontShowCloudSignInAgainKey = "dontShowCloudSignInAgain"
private let dontShowVisionOSPromoAgainKey = "dontShowVisionOSPromoAgain"
private let hasSignedInOnIOSKey = "hasSignedInOnIOS"

// MARK: - Main Onboarding Container

struct WelcomeOnboardingView: View {
    @Binding var isOnboardingComplete: Bool
    
    @State private var currentPage: OnboardingPage = .welcome
    @State private var hasCheckedCloudStatus = false
    @StateObject private var dropboxManager = DropboxManager.shared
    @StateObject private var googleDriveManager = GoogleDriveManager.shared
    @StateObject private var cloudStorageManager = CloudStorageManager.shared
    
    /// Check if cloud storage is already configured (e.g., from visionOS via iCloud Keychain)
    private var isAlreadyConnected: Bool {
        cloudStorageManager.isDropboxAuthenticated || cloudStorageManager.isGoogleDriveAuthenticated
    }
    
    /// Check if user has seen the welcome page before
    private var hasSeenWelcome: Bool {
        UserDefaults.standard.bool(forKey: hasSeenWelcomeKey)
    }
    
    /// Check if user opted out of sign-in prompts
    private var dontShowCloudSignInAgain: Bool {
        UserDefaults.standard.bool(forKey: dontShowCloudSignInAgainKey)
    }
    
    /// Check if user opted out of visionOS promo
    private var dontShowVisionOSPromoAgain: Bool {
        UserDefaults.standard.bool(forKey: dontShowVisionOSPromoAgainKey)
    }
    
    /// Check if user has signed in on iOS before
    private var hasSignedInOnIOS: Bool {
        UserDefaults.standard.bool(forKey: hasSignedInOnIOSKey)
    }
    
    /// Get the name of the connected provider
    private var connectedProviderName: String {
        if cloudStorageManager.isGoogleDriveAuthenticated && cloudStorageManager.isDropboxAuthenticated {
            return "Google Drive & Dropbox"
        } else if cloudStorageManager.isGoogleDriveAuthenticated {
            return "Google Drive"
        } else if cloudStorageManager.isDropboxAuthenticated {
            return "Dropbox"
        }
        return ""
    }
    
    enum OnboardingPage {
        case welcome
        case cloudStorage
        case alreadyConnected  // For users who already have cloud storage from visionOS
        case visionProPromo
    }
    
    var body: some View {
        ZStack {
            // Background gradient
            LinearGradient(
                colors: [
                    Color(.systemBackground),
                    Color(.systemBackground).opacity(0.95)
                ],
                startPoint: .top,
                endPoint: .bottom
            )
            .ignoresSafeArea()
            
            switch currentPage {
            case .welcome:
                WelcomePageView(onContinue: {
                    // Mark welcome as seen
                    UserDefaults.standard.set(true, forKey: hasSeenWelcomeKey)
                    navigateAfterWelcome()
                })
                .transition(.asymmetric(
                    insertion: .move(edge: .trailing).combined(with: .opacity),
                    removal: .move(edge: .leading).combined(with: .opacity)
                ))
                
            case .cloudStorage:
                CloudStorageOnboardingView(
                    dropboxManager: dropboxManager,
                    googleDriveManager: googleDriveManager,
                    onBack: {
                        withAnimation(.easeInOut(duration: 0.4)) {
                            currentPage = .welcome
                        }
                    },
                    onSkipForNow: {
                        // Skip for now - will show again next time, check visionOS promo
                        navigateToVisionOSPromoOrComplete()
                    },
                    onSkipForever: {
                        // Don't ask again - but still try to show visionOS promo
                        UserDefaults.standard.set(true, forKey: dontShowCloudSignInAgainKey)
                        navigateToVisionOSPromoOrComplete()
                    },
                    onSignInComplete: {
                        // User signed in - mark it, then check visionOS promo
                        UserDefaults.standard.set(true, forKey: hasSignedInOnIOSKey)
                        navigateToVisionOSPromoOrComplete()
                    }
                )
                .transition(.asymmetric(
                    insertion: .move(edge: .trailing).combined(with: .opacity),
                    removal: .move(edge: .leading).combined(with: .opacity)
                ))
                
            case .alreadyConnected:
                AlreadyConnectedView(
                    providerName: connectedProviderName,
                    onBack: {
                        withAnimation(.easeInOut(duration: 0.4)) {
                            currentPage = .welcome
                        }
                    },
                    onContinue: {
                        // Already connected from visionOS - check if we should show visionOS promo
                        navigateToVisionOSPromoOrComplete()
                    }
                )
                .transition(.asymmetric(
                    insertion: .move(edge: .trailing).combined(with: .opacity),
                    removal: .move(edge: .leading).combined(with: .opacity)
                ))
                
            case .visionProPromo:
                VisionProPromoView(
                    onBack: {
                        withAnimation(.easeInOut(duration: 0.4)) {
                            currentPage = .cloudStorage
                        }
                    },
                    onGetStarted: {
                        // Get Started = don't show promo again
                        UserDefaults.standard.set(true, forKey: dontShowVisionOSPromoAgainKey)
                        completeOnboarding()
                    },
                    onRemindLater: {
                        // Remind later = show again next time
                        completeOnboarding()
                    }
                )
                .transition(.asymmetric(
                    insertion: .move(edge: .trailing).combined(with: .opacity),
                    removal: .move(edge: .leading).combined(with: .opacity)
                ))
            }
        }
        .onAppear {
            // Refresh cloud storage status from keychain
            cloudStorageManager.loadSettings()
            
            // Determine starting page based on state
            determineStartingPage()
        }
    }
    
    /// Determine which page to start on based on user's history
    private func determineStartingPage() {
        // If user has never seen welcome, start there
        if !hasSeenWelcome {
            currentPage = .welcome
            return
        }
        
        // User has seen welcome before - check sign-in status
        if isAlreadyConnected {
            // Already connected (from visionOS or previous iOS sign-in)
            completeOnboarding()
            return
        }
        
        // Not connected - should we show sign-in?
        if !dontShowCloudSignInAgain {
            currentPage = .cloudStorage
            return
        }
        
        // User opted out of sign-in - should we show visionOS promo?
        if !dontShowVisionOSPromoAgain {
            currentPage = .visionProPromo
            return
        }
        
        // User opted out of everything - go to app
        completeOnboarding()
    }
    
    /// Navigate after welcome page based on connection status
    private func navigateAfterWelcome() {
        withAnimation(.easeInOut(duration: 0.4)) {
            if isAlreadyConnected {
                currentPage = .alreadyConnected
            } else {
                currentPage = .cloudStorage
            }
        }
    }
    
    /// Navigate to visionOS promo or complete onboarding
    private func navigateToVisionOSPromoOrComplete() {
        if dontShowVisionOSPromoAgain {
            completeOnboarding()
        } else {
            withAnimation(.easeInOut(duration: 0.4)) {
                currentPage = .visionProPromo
            }
        }
    }
    
    private func completeOnboarding() {
        withAnimation(.easeInOut(duration: 0.3)) {
            isOnboardingComplete = true
        }
    }
}

// MARK: - Welcome Page

struct WelcomePageView: View {
    let onContinue: () -> Void
    
    @State private var animateIcon = false
    @State private var animateText = false
    @State private var animateButton = false
    
    var body: some View {
        VStack(spacing: 0) {
            Spacer()
            
            // App icon and title
            VStack(spacing: 24) {
                // Animated app icon
                ZStack {
                    // Outer glow
                    Circle()
                        .fill(
                            RadialGradient(
                                colors: [
                                    Color.blue.opacity(0.3),
                                    Color.clear
                                ],
                                center: .center,
                                startRadius: 40,
                                endRadius: 80
                            )
                        )
                        .frame(width: 160, height: 160)
                        .scaleEffect(animateIcon ? 1.1 : 0.9)
                        .animation(.easeInOut(duration: 2).repeatForever(autoreverses: true), value: animateIcon)
                    
                    // Icon background
                    RoundedRectangle(cornerRadius: 28)
                        .fill(
                            LinearGradient(
                                colors: [Color.blue, Color.purple],
                                startPoint: .topLeading,
                                endPoint: .bottomTrailing
                            )
                        )
                        .frame(width: 100, height: 100)
                        .shadow(color: .blue.opacity(0.4), radius: 20, y: 10)
                    
                    // Icon symbol
                    Image(systemName: "hand.raised.fingers.spread.fill")
                        .font(.system(size: 44))
                        .foregroundColor(.white)
                }
                .scaleEffect(animateIcon ? 1 : 0.8)
                .opacity(animateIcon ? 1 : 0)
                
                // Title
                VStack(spacing: 8) {
                    Text("Tracking Manager")
                        .font(.system(size: 34, weight: .bold, design: .rounded))
                    
                    Text("Review your Vision Pro recordings")
                        .font(.title3)
                        .foregroundColor(.secondary)
                }
                .opacity(animateText ? 1 : 0)
                .offset(y: animateText ? 0 : 20)
            }
            
            Spacer()
            
            // Features list
            VStack(spacing: 20) {
                WelcomeFeatureRow(
                    icon: "play.rectangle.fill",
                    iconColor: .blue,
                    title: "Browse Recordings",
                    description: "Access all your hand tracking sessions"
                )
                
                WelcomeFeatureRow(
                    icon: "icloud.fill",
                    iconColor: .cyan,
                    title: "Cloud Sync",
                    description: "Sync recordings from Google Drive or Dropbox"
                )
                
                WelcomeFeatureRow(
                    icon: "globe",
                    iconColor: .green,
                    title: "Public Library",
                    description: "Explore and share recordings with the community"
                )
            }
            .padding(.horizontal, 32)
            .opacity(animateText ? 1 : 0)
            .offset(y: animateText ? 0 : 30)
            
            Spacer()
            
            // Continue button
            Button(action: onContinue) {
                Text("Get Started")
                    .font(.headline)
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 16)
                    .background(
                        LinearGradient(
                            colors: [Color.blue, Color.purple],
                            startPoint: .leading,
                            endPoint: .trailing
                        )
                    )
                    .cornerRadius(14)
            }
            .padding(.horizontal, 32)
            .padding(.bottom, 48)
            .opacity(animateButton ? 1 : 0)
            .offset(y: animateButton ? 0 : 20)
        }
        .onAppear {
            withAnimation(.easeOut(duration: 0.6)) {
                animateIcon = true
            }
            withAnimation(.easeOut(duration: 0.6).delay(0.2)) {
                animateText = true
            }
            withAnimation(.easeOut(duration: 0.6).delay(0.4)) {
                animateButton = true
            }
        }
    }
}

// MARK: - Welcome Feature Row

struct WelcomeFeatureRow: View {
    let icon: String
    let iconColor: Color
    let title: String
    let description: String
    
    var body: some View {
        HStack(spacing: 16) {
            ZStack {
                Circle()
                    .fill(iconColor.opacity(0.15))
                    .frame(width: 50, height: 50)
                
                Image(systemName: icon)
                    .font(.title2)
                    .foregroundColor(iconColor)
            }
            
            VStack(alignment: .leading, spacing: 2) {
                Text(title)
                    .font(.headline)
                Text(description)
                    .font(.subheadline)
                    .foregroundColor(.secondary)
            }
            
            Spacer()
        }
    }
}

// MARK: - Cloud Storage Onboarding View

struct CloudStorageOnboardingView: View {
    @ObservedObject var dropboxManager: DropboxManager
    @ObservedObject var googleDriveManager: GoogleDriveManager
    @StateObject private var cloudStorageManager = CloudStorageManager.shared
    let onBack: () -> Void
    let onSkipForNow: () -> Void
    let onSkipForever: () -> Void
    let onSignInComplete: () -> Void
    
    private var isAuthenticating: Bool {
        googleDriveManager.isAuthenticating || dropboxManager.isAuthenticating
    }
    
    private var authError: String? {
        googleDriveManager.authError ?? dropboxManager.authError
    }
    
    var body: some View {
        VStack(spacing: 0) {
            // Header with back button
            HStack {
                Button(action: onBack) {
                    HStack(spacing: 4) {
                        Image(systemName: "chevron.left")
                            .font(.body.weight(.semibold))
                        Text("Back")
                    }
                    .foregroundColor(.blue)
                }
                
                Spacer()
                
                // Page indicator
                HStack(spacing: 6) {
                    Circle()
                        .fill(Color.secondary.opacity(0.3))
                        .frame(width: 8, height: 8)
                    Circle()
                        .fill(Color.blue)
                        .frame(width: 8, height: 8)
                    Circle()
                        .fill(Color.secondary.opacity(0.3))
                        .frame(width: 8, height: 8)
                }
                
                Spacer()
                
                // Invisible spacer for balance
                Text("Back")
                    .opacity(0)
            }
            .padding(.horizontal, 20)
            .padding(.top, 16)
            
            Spacer()
            
            // Cloud icon and title
            VStack(spacing: 20) {
                ZStack {
                    Circle()
                        .fill(
                            LinearGradient(
                                colors: [Color.blue.opacity(0.2), Color.purple.opacity(0.2)],
                                startPoint: .topLeading,
                                endPoint: .bottomTrailing
                            )
                        )
                        .frame(width: 100, height: 100)
                    
                    Image(systemName: "icloud.fill")
                        .font(.system(size: 44))
                        .foregroundStyle(
                            LinearGradient(
                                colors: [.blue, .purple],
                                startPoint: .topLeading,
                                endPoint: .bottomTrailing
                            )
                        )
                }
                
                VStack(spacing: 8) {
                    Text("Connect Cloud Storage")
                        .font(.system(size: 28, weight: .bold))
                    
                    Text("Sync recordings from your Vision Pro")
                        .font(.body)
                        .foregroundColor(.secondary)
                        .multilineTextAlignment(.center)
                }
            }
            .padding(.bottom, 40)
            
            // Benefits
            VStack(alignment: .leading, spacing: 18) {
                CloudBenefitRow(
                    icon: "arrow.triangle.2.circlepath",
                    iconColor: .blue,
                    title: "Automatic Sync",
                    description: "Recordings sync from Vision Pro automatically"
                )
                
                CloudBenefitRow(
                    icon: "lock.shield.fill",
                    iconColor: .green,
                    title: "Your Data, Your Control",
                    description: "All recordings stay in your personal cloud storage"
                )
                
                CloudBenefitRow(
                    icon: "visionpro",
                    iconColor: .purple,
                    title: "Seamless Integration",
                    description: "Credentials sync to Vision Pro via iCloud Keychain"
                )
            }
            .padding(.horizontal, 32)
            
            Spacer()
            
            // Error message
            if let error = authError {
                HStack(spacing: 8) {
                    Image(systemName: "exclamationmark.triangle.fill")
                        .foregroundColor(.yellow)
                    Text(error)
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                .padding(.horizontal, 32)
                .padding(.bottom, 12)
            }
            
            // Sign-in buttons
            VStack(spacing: 12) {
                // Google Drive button
                Button {
                    Task {
                        await googleDriveManager.startOAuthFlow()
                        if googleDriveManager.isAuthenticated {
                            // Reload settings to pick up the new auth state
                            cloudStorageManager.loadSettings()
                            // Automatically select Google Drive as the provider
                            cloudStorageManager.selectProvider(.googleDrive)
                            onSignInComplete()
                        }
                    }
                } label: {
                    HStack(spacing: 12) {
                        if googleDriveManager.isAuthenticating {
                            ProgressView()
                                .tint(.white)
                        } else {
                            Image(systemName: "externaldrive.fill")
                                .font(.title3)
                        }
                        
                        Text(googleDriveManager.isAuthenticating ? "Signing in..." : "Continue with Google Drive")
                            .font(.headline)
                    }
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 16)
                    .background(
                        LinearGradient(
                            colors: [
                                Color(red: 0.26, green: 0.52, blue: 0.96),
                                Color(red: 0.22, green: 0.45, blue: 0.88)
                            ],
                            startPoint: .leading,
                            endPoint: .trailing
                        )
                    )
                    .cornerRadius(14)
                }
                .disabled(isAuthenticating)
                
                // Dropbox button
                Button {
                    Task {
                        await dropboxManager.startOAuthFlow()
                        if dropboxManager.isAuthenticated {
                            // Reload settings to pick up the new auth state
                            cloudStorageManager.loadSettings()
                            // Automatically select Dropbox as the provider
                            cloudStorageManager.selectProvider(.dropbox)
                            onSignInComplete()
                        }
                    }
                } label: {
                    HStack(spacing: 12) {
                        if dropboxManager.isAuthenticating {
                            ProgressView()
                                .tint(.white)
                        } else {
                            Image(systemName: "shippingbox.fill")
                                .font(.title3)
                        }
                        
                        Text(dropboxManager.isAuthenticating ? "Signing in..." : "Continue with Dropbox")
                            .font(.headline)
                    }
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 16)
                    .background(
                        LinearGradient(
                            colors: [
                                Color(red: 0, green: 0.4, blue: 1),
                                Color(red: 0, green: 0.35, blue: 0.9)
                            ],
                            startPoint: .leading,
                            endPoint: .trailing
                        )
                    )
                    .cornerRadius(14)
                }
                .disabled(isAuthenticating)
                
                // Skip buttons - side by side with light background
                HStack(spacing: 12) {
                    Button(action: onSkipForNow) {
                        Text("Skip for now")
                            .font(.body)
                            .foregroundColor(.secondary)
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 14)
                            .background(Color(.systemGray5))
                            .cornerRadius(14)
                    }
                    .disabled(isAuthenticating)
                    
                    Button(action: onSkipForever) {
                        Text("Don't ask again")
                            .font(.body)
                            .foregroundColor(.secondary)
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 14)
                            .background(Color(.systemGray5))
                            .cornerRadius(14)
                    }
                    .disabled(isAuthenticating)
                }
            }
            .padding(.horizontal, 32)
            .padding(.bottom, 48)
        }
    }
}

// MARK: - Cloud Benefit Row

struct CloudBenefitRow: View {
    let icon: String
    let iconColor: Color
    let title: String
    let description: String
    
    var body: some View {
        HStack(alignment: .top, spacing: 14) {
            Image(systemName: icon)
                .font(.title3)
                .foregroundColor(iconColor)
                .frame(width: 28)
            
            VStack(alignment: .leading, spacing: 2) {
                Text(title)
                    .font(.subheadline.weight(.semibold))
                Text(description)
                    .font(.subheadline)
                    .foregroundColor(.secondary)
                    .fixedSize(horizontal: false, vertical: true)
            }
        }
    }
}

// MARK: - Already Connected View (for users who set up cloud storage on visionOS first)

struct AlreadyConnectedView: View {
    let providerName: String
    let onBack: () -> Void
    let onContinue: () -> Void
    
    @StateObject private var cloudStorageManager = CloudStorageManager.shared
    @State private var animateCheckmark = false
    @State private var animateContent = false
    
    var body: some View {
        VStack(spacing: 0) {
            // Header with back button
            HStack {
                Button(action: onBack) {
                    HStack(spacing: 4) {
                        Image(systemName: "chevron.left")
                            .font(.body.weight(.semibold))
                        Text("Back")
                    }
                    .foregroundColor(.blue)
                }
                
                Spacer()
                
                // Page indicator (2 pages only for this flow)
                HStack(spacing: 6) {
                    Circle()
                        .fill(Color.secondary.opacity(0.3))
                        .frame(width: 8, height: 8)
                    Circle()
                        .fill(Color.blue)
                        .frame(width: 8, height: 8)
                }
                
                Spacer()
                
                // Invisible spacer for balance
                Text("Back")
                    .opacity(0)
            }
            .padding(.horizontal, 20)
            .padding(.top, 16)
            
            Spacer()
            
            // Success icon and message
            VStack(spacing: 24) {
                ZStack {
                    // Animated success rings
                    ForEach(0..<2) { index in
                        Circle()
                            .stroke(Color.green.opacity(0.2), lineWidth: 2)
                            .frame(width: CGFloat(120 + index * 50), height: CGFloat(120 + index * 50))
                            .scaleEffect(animateCheckmark ? 1.0 : 0.8)
                            .opacity(animateCheckmark ? 0.5 : 0)
                            .animation(
                                .easeOut(duration: 0.6)
                                .delay(Double(index) * 0.15),
                                value: animateCheckmark
                            )
                    }
                    
                    // Success circle
                    ZStack {
                        Circle()
                            .fill(
                                LinearGradient(
                                    colors: [Color.green, Color.green.opacity(0.8)],
                                    startPoint: .topLeading,
                                    endPoint: .bottomTrailing
                                )
                            )
                            .frame(width: 100, height: 100)
                            .shadow(color: .green.opacity(0.4), radius: 20, y: 10)
                        
                        Image(systemName: "checkmark")
                            .font(.system(size: 44, weight: .bold))
                            .foregroundColor(.white)
                            .scaleEffect(animateCheckmark ? 1 : 0)
                            .animation(.spring(response: 0.4, dampingFraction: 0.6).delay(0.2), value: animateCheckmark)
                    }
                    .scaleEffect(animateCheckmark ? 1 : 0.5)
                    .animation(.spring(response: 0.5, dampingFraction: 0.7), value: animateCheckmark)
                }
                .frame(height: 180)
                
                VStack(spacing: 12) {
                    Text("You're All Set!")
                        .font(.system(size: 28, weight: .bold))
                    
                    Text("Connected to \(providerName)")
                        .font(.title3)
                        .foregroundColor(.green)
                    
                    Text("Your Vision Pro is already syncing recordings")
                        .font(.body)
                        .foregroundColor(.secondary)
                        .multilineTextAlignment(.center)
                }
                .opacity(animateContent ? 1 : 0)
                .offset(y: animateContent ? 0 : 20)
            }
            .padding(.bottom, 40)
            
            // Info cards
            VStack(spacing: 16) {
                AlreadyConnectedInfoCard(
                    icon: "visionpro",
                    iconColor: .purple,
                    title: "Synced from Vision Pro",
                    description: "Your cloud storage credentials were automatically shared via iCloud Keychain"
                )
                
                AlreadyConnectedInfoCard(
                    icon: "arrow.triangle.2.circlepath",
                    iconColor: .blue,
                    title: "Ready to Browse",
                    description: "Your recordings will appear in the app automatically"
                )
            }
            .padding(.horizontal, 32)
            .opacity(animateContent ? 1 : 0)
            .offset(y: animateContent ? 0 : 30)
            
            Spacer()
            
            // Continue button
            Button(action: onContinue) {
                Text("Continue")
                    .font(.headline)
                    .foregroundColor(.white)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 16)
                    .background(Color.green)
                    .cornerRadius(14)
            }
            .padding(.horizontal, 32)
            .padding(.bottom, 48)
            .opacity(animateContent ? 1 : 0)
            .offset(y: animateContent ? 0 : 20)
        }
        .onAppear {
            // Set the default storage provider to match what's configured from visionOS
            configureDefaultProvider()
            
            withAnimation {
                animateCheckmark = true
            }
            withAnimation(.easeOut(duration: 0.5).delay(0.3)) {
                animateContent = true
            }
        }
    }
    
    /// Configure the default storage provider based on what's authenticated from visionOS
    private func configureDefaultProvider() {
        // Prioritize Google Drive if authenticated, otherwise use Dropbox
        if cloudStorageManager.isGoogleDriveAuthenticated {
            cloudStorageManager.selectProvider(.googleDrive)
        } else if cloudStorageManager.isDropboxAuthenticated {
            cloudStorageManager.selectProvider(.dropbox)
        }
    }
}

// MARK: - Already Connected Info Card

struct AlreadyConnectedInfoCard: View {
    let icon: String
    let iconColor: Color
    let title: String
    let description: String
    
    var body: some View {
        HStack(alignment: .top, spacing: 14) {
            ZStack {
                Circle()
                    .fill(iconColor.opacity(0.15))
                    .frame(width: 44, height: 44)
                
                Image(systemName: icon)
                    .font(.title3)
                    .foregroundColor(iconColor)
            }
            
            VStack(alignment: .leading, spacing: 4) {
                Text(title)
                    .font(.subheadline.weight(.semibold))
                Text(description)
                    .font(.caption)
                    .foregroundColor(.secondary)
                    .fixedSize(horizontal: false, vertical: true)
            }
            
            Spacer()
        }
        .padding(16)
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
}

// MARK: - Vision Pro Promo View

struct VisionProPromoView: View {
    let onBack: () -> Void
    let onGetStarted: () -> Void
    let onRemindLater: () -> Void
    
    @State private var animateVisionPro = false
    
    var body: some View {
        VStack(spacing: 0) {
            // Header with back button
            HStack {
                Button(action: onBack) {
                    HStack(spacing: 4) {
                        Image(systemName: "chevron.left")
                            .font(.body.weight(.semibold))
                        Text("Back")
                    }
                    .foregroundColor(.blue)
                }
                
                Spacer()
                
                // Page indicator
                HStack(spacing: 6) {
                    Circle()
                        .fill(Color.secondary.opacity(0.3))
                        .frame(width: 8, height: 8)
                    Circle()
                        .fill(Color.secondary.opacity(0.3))
                        .frame(width: 8, height: 8)
                    Circle()
                        .fill(Color.blue)
                        .frame(width: 8, height: 8)
                }
                
                Spacer()
                
                // Invisible spacer for balance
                Text("Back")
                    .opacity(0)
            }
            .padding(.horizontal, 20)
            .padding(.top, 16)
            
            Spacer()
            
            // Vision Pro illustration
            VStack(spacing: 24) {
                ZStack {
                    // Animated glow rings
                    ForEach(0..<3) { index in
                        Circle()
                            .stroke(
                                LinearGradient(
                                    colors: [.green.opacity(0.3), .orange.opacity(0.3)],
                                    startPoint: .topLeading,
                                    endPoint: .bottomTrailing
                                ),
                                lineWidth: 2
                            )
                            .frame(width: CGFloat(120 + index * 40), height: CGFloat(120 + index * 40))
                            .scaleEffect(animateVisionPro ? 1.1 : 0.9)
                            .opacity(animateVisionPro ? 0.3 : 0.6)
                            .animation(
                                .easeInOut(duration: 2)
                                .repeatForever(autoreverses: true)
                                .delay(Double(index) * 0.3),
                                value: animateVisionPro
                            )
                    }
                    
                    // Vision Pro icon
                    ZStack {
                        RoundedRectangle(cornerRadius: 24)
                            .fill(
                                LinearGradient(
                                    colors: [Color.green, Color.orange],
                                    startPoint: .topLeading,
                                    endPoint: .bottomTrailing
                                )
                            )
                            .frame(width: 100, height: 100)
                            .shadow(color: .green.opacity(0.3), radius: 20, y: 10)
                        
                        Image(systemName: "visionpro")
                            .font(.system(size: 44))
                            .foregroundColor(.white)
                    }
                }
                .frame(height: 200)
                
                VStack(spacing: 8) {
                    Text("Tracking Streamer")
                        .font(.system(size: 28, weight: .bold))
                    
                    Text("for Apple Vision Pro")
                        .font(.title3)
                        .foregroundColor(.secondary)
                }
            }
            .padding(.bottom, 32)
            
            // Features
            VStack(alignment: .leading, spacing: 18) {
                VisionProFeatureRow(
                    icon: "hand.raised.fingers.spread.fill",
                    iconColor: .orange,
                    title: "Hand & Head Tracking",
                    description: "Stream real-time tracking data to your robotics setup"
                )
                
                VisionProFeatureRow(
                    icon: "video.fill",
                    iconColor: .green,
                    title: "Video Streaming",
                    description: "Stream video and audio for immersive teleoperation"
                )
                
                VisionProFeatureRow(
                    icon: "cube.transparent",
                    iconColor: .purple,
                    title: "MuJoCo Integration",
                    description: "Visualize physics simulations in mixed reality"
                )
                
                VisionProFeatureRow(
                    icon: "record.circle",
                    iconColor: .red,
                    title: "Session Recording",
                    description: "Automatically save sessions to your cloud storage"
                )
            }
            .padding(.horizontal, 32)
            
            Spacer()
            
            // App Store info
            VStack(spacing: 8) {
                Text("Available on the App Store")
                    .font(.caption)
                    .foregroundColor(.secondary)
                
                HStack(spacing: 4) {
                    Image(systemName: "applelogo")
                    Text("Search \"Tracking Streamer\"")
                }
                .font(.caption.weight(.medium))
                .foregroundColor(.secondary)
            }
            .padding(.bottom, 16)
            
            // Buttons - side by side
            HStack(spacing: 12) {
                // Remind me later button
                Button(action: onRemindLater) {
                    Text("Remind me later")
                        .font(.body)
                        .foregroundColor(.secondary)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(Color(.systemGray5))
                        .cornerRadius(14)
                }
                
                // Get Started button
                Button(action: onGetStarted) {
                    Text("Get Started")
                        .font(.headline)
                        .foregroundColor(.white)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 14)
                        .background(
                            LinearGradient(
                                colors: [Color.green, Color.orange],
                                startPoint: .leading,
                                endPoint: .trailing
                            )
                        )
                        .cornerRadius(14)
                }
            }
            .padding(.horizontal, 32)
            .padding(.bottom, 48)
        }
        .onAppear {
            animateVisionPro = true
        }
    }
}

// MARK: - Vision Pro Feature Row

struct VisionProFeatureRow: View {
    let icon: String
    let iconColor: Color
    let title: String
    let description: String
    
    var body: some View {
        HStack(alignment: .top, spacing: 14) {
            Image(systemName: icon)
                .font(.title3)
                .foregroundColor(iconColor)
                .frame(width: 28)
            
            VStack(alignment: .leading, spacing: 2) {
                Text(title)
                    .font(.subheadline.weight(.semibold))
                Text(description)
                    .font(.subheadline)
                    .foregroundColor(.secondary)
                    .fixedSize(horizontal: false, vertical: true)
            }
        }
    }
}

// MARK: - Helper to check onboarding status

/// Check if user should see any onboarding screens
func shouldShowOnboarding() -> Bool {
    let hasSeenWelcome = UserDefaults.standard.bool(forKey: hasSeenWelcomeKey)
    let dontShowCloudSignIn = UserDefaults.standard.bool(forKey: dontShowCloudSignInAgainKey)
    let dontShowVisionOSPromo = UserDefaults.standard.bool(forKey: dontShowVisionOSPromoAgainKey)
    
    // Check if already connected
    let cloudStorageManager = CloudStorageManager.shared
    let isConnected = cloudStorageManager.isDropboxAuthenticated || cloudStorageManager.isGoogleDriveAuthenticated
    
    // First time user - show welcome
    if !hasSeenWelcome {
        return true
    }
    
    // Already connected - no need to show anything
    if isConnected {
        return false
    }
    
    // Should we show sign-in?
    if !dontShowCloudSignIn {
        return true
    }
    
    // Should we show visionOS promo?
    if !dontShowVisionOSPromo {
        return true
    }
    
    // User opted out of everything
    return false
}

/// Legacy function for backward compatibility
func hasCompletedOnboarding() -> Bool {
    !shouldShowOnboarding()
}

func isCloudStorageConfigured() -> Bool {
    let cloudStorageManager = CloudStorageManager.shared
    return cloudStorageManager.isDropboxAuthenticated || cloudStorageManager.isGoogleDriveAuthenticated
}

#Preview("Welcome") {
    WelcomeOnboardingView(isOnboardingComplete: .constant(false))
}

#Preview("Cloud Storage") {
    CloudStorageOnboardingView(
        dropboxManager: DropboxManager.shared,
        googleDriveManager: GoogleDriveManager.shared,
        onBack: {},
        onSkipForNow: {},
        onSkipForever: {},
        onSignInComplete: {}
    )
}

#Preview("Already Connected") {
    AlreadyConnectedView(
        providerName: "Google Drive",
        onBack: {},
        onContinue: {}
    )
}

#Preview("Vision Pro Promo") {
    VisionProPromoView(onBack: {}, onGetStarted: {}, onRemindLater: {})
}
