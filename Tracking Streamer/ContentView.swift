import SwiftUI
import CoreLocation
import UIKit
import AVKit
import SystemConfiguration.CaptiveNetwork

private let dontShowSignInAgainKey = "dontShowSignInAgain"  // User opted out of sign-in prompt
private let dontShowIOSPromoAgainKey = "dontShowIOSPromoAgain"  // User opted out of iOS promo views
private let hasSignedInOnVisionOSKey = "hasSignedInOnVisionOS"  // Track if sign-in happened on visionOS

func dlog(_ msg: @autoclosure () -> String) {
    #if DEBUG
    print(msg())
    #endif
}

/// Onboarding flow state for first-time users
enum OnboardingState: Equatable {
    case mainView                           // Initial START screen
    case cloudSignInPrompt                  // Cloud storage sign-in prompt
    case iosAppDetectedSignIn               // User signed in via iOS app (Case 1A)
    case iosAppShowcaseAfterSignIn          // iOS app features after signing in on visionOS
    case iosAppShowcaseAfterSkip            // iOS app features after skipping (mentions sign-in via iOS)
}

struct ContentView: View {
    @Environment(\.openImmersiveSpace) var openImmersiveSpace
    @Environment(\.dismissWindow) var dismissWindow
    @State private var showVideoStream = false
    @AppStorage("pythonServerIP") private var pythonServerIP = "10.29.239.70"
    @State private var showSettings = false
    @State private var serverReady = false
    @State private var onboardingState: OnboardingState = .mainView
    @StateObject private var googleAuthManager = GoogleDriveAuthManager.shared
    @StateObject private var dropboxAuthManager = DropboxAuthManager.shared
    @ObservedObject private var cloudStorageSettings = CloudStorageSettings.shared

    @ObservedObject private var signalingClient = SignalingClient.shared
    @ObservedObject private var dataManager = DataManager.shared
    
    /// Check if user opted out of sign-in prompt
    private var dontShowSignInAgain: Bool {
        UserDefaults.standard.bool(forKey: dontShowSignInAgainKey)
    }
    
    /// Check if user opted out of iOS promo views
    private var dontShowIOSPromoAgain: Bool {
        UserDefaults.standard.bool(forKey: dontShowIOSPromoAgainKey)
    }
    
    /// Check if user has ever signed in on visionOS (not via iOS app)
    private var hasSignedInOnVisionOS: Bool {
        UserDefaults.standard.bool(forKey: hasSignedInOnVisionOSKey)
    }
    
    /// Check if any cloud storage is configured (Google Drive or Dropbox)
    private var isCloudStorageConfigured: Bool {
        cloudStorageSettings.isGoogleDriveAvailable || cloudStorageSettings.isDropboxAvailable
    }
    
    /// Check if cloud was configured via iOS app (configured but never signed in on visionOS)
    private var wasConfiguredViaIOSApp: Bool {
        isCloudStorageConfigured && !hasSignedInOnVisionOS
    }
    
    var body: some View {
        Group {
            switch onboardingState {
            case .mainView:
                mainContentView
                    .transition(.opacity)
                
            case .cloudSignInPrompt:
                CloudStorageSignInPromptView(
                    googleAuthManager: googleAuthManager,
                    dropboxAuthManager: dropboxAuthManager,
                    onBack: {
                        withAnimation(.easeInOut(duration: 0.25)) {
                            onboardingState = .mainView
                        }
                    },
                    onSkip: {
                        // User skipped sign-in - show iOS promo unless opted out
                        if !dontShowIOSPromoAgain {
                            withAnimation(.easeInOut(duration: 0.25)) {
                                onboardingState = .iosAppShowcaseAfterSkip
                            }
                        } else {
                            proceedToImmersiveSpace()
                        }
                    },
                    onSkipForever: {
                        // User chose to never see sign-in prompt again
                        UserDefaults.standard.set(true, forKey: dontShowSignInAgainKey)
                        proceedToImmersiveSpace()
                    },
                    onSignInComplete: {
                        // Mark that sign-in happened on visionOS
                        UserDefaults.standard.set(true, forKey: hasSignedInOnVisionOSKey)
                        
                        // Show iOS promo unless opted out
                        if !dontShowIOSPromoAgain {
                            withAnimation(.easeInOut(duration: 0.25)) {
                                onboardingState = .iosAppShowcaseAfterSignIn
                            }
                        } else {
                            proceedToImmersiveSpace()
                        }
                    }
                )
                .transition(.opacity)
                
            case .iosAppDetectedSignIn:
                // Case 1A: User signed in via iOS app first
                IOSAppSignInDetectedView(
                    onGetStarted: {
                        // Get Started = don't show iOS promo again
                        UserDefaults.standard.set(true, forKey: dontShowIOSPromoAgainKey)
                        proceedToImmersiveSpace()
                    },
                    onRemindNextTime: {
                        // Just proceed without setting the flag
                        proceedToImmersiveSpace()
                    }
                )
                .transition(.opacity)
                
            case .iosAppShowcaseAfterSignIn:
                // User signed in on visionOS, show iOS app features
                IOSAppShowcaseView(
                    showcaseMode: .afterVisionOSSignIn,
                    onGetStarted: {
                        // Get Started = don't show iOS promo again
                        UserDefaults.standard.set(true, forKey: dontShowIOSPromoAgainKey)
                        proceedToImmersiveSpace()
                    },
                    onRemindNextTime: {
                        // Just proceed without setting the flag
                        proceedToImmersiveSpace()
                    }
                )
                .transition(.opacity)
                
            case .iosAppShowcaseAfterSkip:
                // User skipped sign-in, show iOS app with sign-in reminder
                IOSAppShowcaseView(
                    showcaseMode: .afterSkip,
                    onGetStarted: {
                        // Get Started = don't show iOS promo again
                        UserDefaults.standard.set(true, forKey: dontShowIOSPromoAgainKey)
                        proceedToImmersiveSpace()
                    },
                    onRemindNextTime: {
                        // Just proceed without setting the flag
                        proceedToImmersiveSpace()
                    }
                )
                .transition(.opacity)
            }
        }
        .animation(.easeInOut(duration: 0.25), value: onboardingState)
    }
    
    private func proceedToImmersiveSpace() {
        Task {
            // If in Remote mode, connect to signaling server first
            if dataManager.isCrossNetworkMode {
                signalingClient.connect()
                // Store the room code in DataManager so it can be used by VideoStreamManager
                DataManager.shared.crossNetworkRoomCode = signalingClient.roomCode
            }
            
            await self.openImmersiveSpace(id: "combinedStreamSpace")
            self.dismissWindow()
        }
    }
    
    /// Handle START button press - determines which onboarding flow to show
    private func handleStartButton() {
        // No cloud storage configured - show sign-in prompt unless user opted out
        if !isCloudStorageConfigured {
            if !dontShowSignInAgain {
                // Show sign-in prompt
                withAnimation(.easeInOut(duration: 0.25)) {
                    onboardingState = .cloudSignInPrompt
                }
            } else {
                // User opted out of sign-in, but still try to show iOS promo
                if !dontShowIOSPromoAgain {
                    withAnimation(.easeInOut(duration: 0.25)) {
                        onboardingState = .iosAppShowcaseAfterSkip
                    }
                } else {
                    proceedToImmersiveSpace()
                }
            }
            return
        }
        
        // Cloud storage IS configured - check if user opted out of iOS promo views
        if dontShowIOSPromoAgain {
            proceedToImmersiveSpace()
            return
        }
        
        // Case 1A: Cloud storage is configured via iOS app - show detected sign-in view
        if wasConfiguredViaIOSApp {
            withAnimation(.easeInOut(duration: 0.25)) {
                onboardingState = .iosAppDetectedSignIn
            }
            return
        }
        
        // Cloud storage is configured on visionOS - show showcase
        withAnimation(.easeInOut(duration: 0.25)) {
            onboardingState = .iosAppShowcaseAfterSignIn
        }
    }
    
    // MARK: - Main Content View
    private var mainContentView: some View {
        VStack(spacing: 32) {
            VStack(spacing: 4) {
                Text("VisionProTeleop")
                    .font(.system(size: 72, weight: .bold))
                    .foregroundColor(.white)
                Text("Tracking Streamer")
                    .font(.largeTitle)
                    .foregroundColor(.white.opacity(0.7))
            }
            .padding(.top, 32)
                
            // Animated data flow visualization + START button
            HStack(spacing: 10) {
                // Video/Audio/Sim label (left side)
                VStack(spacing: 4) {
                    Image(systemName: "video.fill")
                        .font(.title)
                        .foregroundColor(.cyan)
                    Text("Video · Audio")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.7))
                    Text("MuJoCo · IsaacLab")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.7))
                }
                .frame(width: 120)
                
                // Arrows flowing right (from video/audio/sim toward button)
                AnimatedArrows(color: .cyan)
                    
                // Simple START button
                Button {
                    handleStartButton()
                } label: {
                    Text("START")
                        .font(.system(size: 36, weight: .bold))
                        .foregroundColor(.white)
                        .padding(.vertical, 20)
                        .padding(.horizontal, 60)
                        .background(
                            LinearGradient(
                                colors: [Color(hex: "#6366F1"), Color(hex: "#A855F7"), Color(hex: "#EC4899")],
                                startPoint: .leading,
                                endPoint: .trailing
                            )
                        )
                        .cornerRadius(16)
                }
                .buttonStyle(.plain)
                
                // Arrows flowing right (from button toward hand tracking)
                AnimatedArrows(color: .pink)
                
                // Hand/Head tracking label (right side)
                VStack(spacing: 4) {
                    Image(systemName: "hand.raised.fill")
                        .font(.title)
                        .foregroundColor(.pink)
                    Text("Hand / Head")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.7))
                    Text("Tracking")
                        .font(.caption)
                        .foregroundColor(.white.opacity(0.7))
                }
                .frame(width: 100)
            }
            .padding(.top, 16)
            
            // Connection Mode Toggle + Info
            VStack(spacing: 16) {
                // Mode toggle (Local / Remote)
                HStack(spacing: 0) {

                    Button {
                        withAnimation(.easeInOut(duration: 0.2)) {
                            dataManager.isCrossNetworkMode = false
                        }
                    } label: {
                        HStack(spacing: 8) {
                            Image(systemName: "wifi")
                                .font(.headline)
                            Text("Local")
                                .font(.headline)
                        }
                        .foregroundColor(!dataManager.isCrossNetworkMode ? .white : .white.opacity(0.5))
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(
                            RoundedRectangle(cornerRadius: 12)
                                .fill(!dataManager.isCrossNetworkMode ? Color.blue.opacity(0.6) : Color.clear)
                        )
                        .contentShape(Rectangle())
                    }
                    .buttonStyle(.plain)
                    
                    Button {
                        withAnimation(.easeInOut(duration: 0.2)) {
                            dataManager.isCrossNetworkMode = true
                        }
                    } label: {
                        HStack(spacing: 8) {
                            Image(systemName: "globe")
                                .font(.headline)
                            Text("Remote")
                                .font(.headline)
                        }
                        .foregroundColor(dataManager.isCrossNetworkMode ? .white : .white.opacity(0.5))
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                        .background(
                            RoundedRectangle(cornerRadius: 12)
                                .fill(dataManager.isCrossNetworkMode ? Color.cyan.opacity(0.6) : Color.clear)
                        )
                        .contentShape(Rectangle())
                    }
                    .buttonStyle(.plain)
                }
                .frame(height: 54) // Enforce fixed height on the container
                .padding(4)
                .background(Color.white.opacity(0.1))
                .cornerRadius(16)
                .frame(width: 300) // Fixed width for the toggle container
                
                // Connection info based on mode
                if dataManager.isCrossNetworkMode {
                    // Remote mode - show room code
                    VStack(spacing: 8) {
                        HStack(spacing: 12) {
                            Text(signalingClient.roomCode)
                                .font(.system(size: 32, weight: .bold, design: .monospaced))
                                .foregroundColor(.white)
                                .tracking(2)
                            
                            Button {
                                signalingClient.generateRoomCode()
                            } label: {
                                Image(systemName: "arrow.clockwise")
                                    .font(.body)
                                    .foregroundColor(.white.opacity(0.5))
                                    .padding(8)
                                    .background(Color.white.opacity(0.1))
                                    .cornerRadius(8)
                            }
                            .buttonStyle(.plain)
                        }
                        
                        Text("Python: VisionProStreamer(room=\"\(signalingClient.roomCode)\")")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.5))
                    }
                    .frame(height: 80) // Fixed height to prevent layout jump
                } else {
                    // Local mode - show IP addresses
                    // Wrap in specific frame to match remote mode height roughly
                    VStack {
                         IPAddressCard(addresses: getIPAddresses())
                    }
                    .frame(minHeight: 80)
                }
                
                // Server status - ONLY SHOW IN LOCAL MODE
                if !dataManager.isCrossNetworkMode {
                    HStack(spacing: 8) {
                        Circle()
                            .fill(serverReady ? Color.green : Color.orange)
                            .frame(width: 12, height: 12)
                            .shadow(color: serverReady ? Color.green.opacity(0.6) : Color.orange.opacity(0.6), radius: 6)
                        Text(serverReady ? "gRPC Server Ready" : "Starting gRPC Server...")
                            .font(.subheadline.weight(.medium))
                            .foregroundColor(serverReady ? .green : .orange)
                    }
                    .transition(.opacity) // Smooth fade in/out
                }
            }
            .onAppear {
                // Poll for server ready status (keep polling even if hidden, so state is ready when switching back)
                Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { timer in
                    serverReady = DataManager.shared.grpcServerReady
                    if serverReady {
                        timer.invalidate()
                    }
                }
            }
            
            // Exit button
            HStack(spacing: 24) {
                Button {
                    exit(0)
                } label: {
                    ZStack {
                        Circle()
                            .fill(Color.red)
                            .frame(width: 50, height: 50)
                        Text("✕")
                            .font(.title.bold())
                            .foregroundColor(.white)
                    }
                }
                .buttonStyle(.plain)
            }
            
        }
        .padding(32)
        .frame(minWidth: 700, minHeight: 600)
    }
}

// MARK: - IP Address Card Component
struct IPAddressCard: View {
    let addresses: [(name: String, address: String)]
    
    var body: some View {
        VStack(spacing: 12) {
            if addresses.isEmpty {
                Text("No network connection")
                    .font(.title3)
                    .foregroundColor(.white.opacity(0.6))
            } else if addresses.count == 1 {
                // Single address - centered
                ForEach(addresses, id: \.address) { ip in
                    HStack(spacing: 10) {
                        Text(ip.name)
                            .font(.system(size: 14, weight: .medium, design: .monospaced))
                            .foregroundColor(.white.opacity(0.7))
                            .frame(width: 55)
                            .padding(.vertical, 4)
                            .padding(.horizontal, 8)
                            .background(Color.white.opacity(0.15))
                            .cornerRadius(6)
                        Text(ip.address)
                            .font(.system(size: 20, weight: .semibold, design: .monospaced))
                            .foregroundColor(.white)
                            .frame(width: 200, alignment: .trailing)
                    }
                }
            } else {
                // Grid layout for 2+ addresses
                HStack(alignment: .top, spacing: 24) {
                    // Left column
                    VStack(spacing: 8) {
                        ForEach(Array(addresses.enumerated()).filter { $0.offset % 2 == 0 }, id: \.element.address) { _, ip in
                            HStack(spacing: 8) {
                                Text(ip.name)
                                    .font(.system(size: 14, weight: .medium, design: .monospaced))
                                    .foregroundColor(.white.opacity(0.7))
                                    .frame(width: 55)
                                    .padding(.vertical, 4)
                                    .padding(.horizontal, 8)
                                    .background(Color.white.opacity(0.15))
                                    .cornerRadius(6)
                                Text(ip.address)
                                    .font(.system(size: 20, weight: .semibold, design: .monospaced))
                                    .foregroundColor(.white)
                                    .frame(width: 180, alignment: .trailing)
                            }
                        }
                    }
                    // Right column
                    VStack(spacing: 8) {
                        ForEach(Array(addresses.enumerated()).filter { $0.offset % 2 == 1 }, id: \.element.address) { _, ip in
                            HStack(spacing: 8) {
                                Text(ip.name)
                                    .font(.system(size: 14, weight: .medium, design: .monospaced))
                                    .foregroundColor(.white.opacity(0.7))
                                    .frame(width: 55)
                                    .padding(.vertical, 4)
                                    .padding(.horizontal, 8)
                                    .background(Color.white.opacity(0.15))
                                    .cornerRadius(6)
                                Text(ip.address)
                                    .font(.system(size: 20, weight: .semibold, design: .monospaced))
                                    .foregroundColor(.white)
                                    .frame(width: 180, alignment: .trailing)
                            }
                        }
                    }
                }
            }
        }
    }
}

// MARK: - Animated Arrows Component
struct AnimatedArrows: View {
    let color: Color
    
    var body: some View {
        TimelineView(.animation(minimumInterval: 1/60)) { timeline in
            Canvas { context, size in
                let time = timeline.date.timeIntervalSinceReferenceDate
                let dotCount = 8
                let dotSpacing: CGFloat = 8
                let totalWidth = CGFloat(dotCount) * dotSpacing
                let startX = (size.width - totalWidth) / 2
                let centerY = size.height / 2
                
                // Animation phase (0 to 1, cycling every 0.8 seconds)
                let phase = (time.truncatingRemainder(dividingBy: 0.8)) / 0.8
                
                for i in 0..<dotCount {
                    let baseX = startX + CGFloat(i) * dotSpacing
                    
                    // Calculate opacity based on wave position
                    let normalizedPos = Double(i) / Double(dotCount - 1)
                    let wavePos = phase
                    
                    // Create traveling wave effect
                    var diff = normalizedPos - wavePos
                    if diff < 0 { diff += 1.0 }
                    
                    let opacity = diff < 0.4 ? (1.0 - diff / 0.4) * 0.85 + 0.15 : 0.15
                    
                    // Draw dot
                    let dotSize: CGFloat = 4
                    let rect = CGRect(x: baseX - dotSize/2, y: centerY - dotSize/2, width: dotSize, height: dotSize)
                    let path = Path(ellipseIn: rect)
                    context.fill(path, with: .color(color.opacity(opacity)))
                }
            }
        }
        .frame(width: 70, height: 24)
    }
}

// MARK: - Cloud Storage Sign-In Prompt View
struct CloudStorageSignInPromptView: View {
    @ObservedObject var googleAuthManager: GoogleDriveAuthManager
    @ObservedObject var dropboxAuthManager: DropboxAuthManager
    let onBack: () -> Void
    let onSkip: () -> Void
    let onSkipForever: () -> Void
    let onSignInComplete: () -> Void
    
    private var isAuthenticating: Bool {
        googleAuthManager.isAuthenticating || dropboxAuthManager.isAuthenticating
    }
    
    private var authError: String? {
        googleAuthManager.authError ?? dropboxAuthManager.authError
    }
    
    var body: some View {
        VStack(spacing: 0) {
            // Back button at top-left
            HStack {
                Button {
                    onBack()
                } label: {
                    HStack(spacing: 6) {
                        Image(systemName: "chevron.left")
                            .font(.body.bold())
                        Text("Back")
                            .font(.body)
                    }
                    .foregroundColor(.secondary)
                    .padding(.horizontal, 12)
                    .padding(.vertical, 8)
                }
                .buttonStyle(.plain)
                
                Spacer()
            }
            .padding(.top, 16)
            .padding(.horizontal, 24)
            
            // Header
            VStack(spacing: 16) {
                // Cloud Storage Icon
                ZStack {
                    Circle()
                        .fill(
                            LinearGradient(
                                colors: [Color.green, Color.orange],
                                startPoint: .topLeading,
                                endPoint: .bottomTrailing
                            )
                        )
                        .frame(width: 80, height: 80)
                    
                    Image(systemName: "icloud.fill")
                        .font(.system(size: 36))
                        .foregroundColor(.white)
                }
                .padding(.top, 16)
                
                Text("Connect Cloud Storage")
                    .font(.system(size: 32, weight: .bold))
            }
            .padding(.bottom, 32)
            
            // Benefits Section
            VStack(alignment: .leading, spacing: 20) {
                BenefitRow(
                    icon: "record.circle",
                    iconColor: .red,
                    title: "Automatic Session Recording",
                    description: "Video and hand tracking data are automatically saved to your cloud storage"
                )
                
                BenefitRow(
                    icon: "iphone.and.arrow.forward",
                    iconColor: .blue,
                    title: "iOS Companion App",
                    description: "Review and replay your sessions on the accompanying iOS app"
                )
                
                BenefitRow(
                    icon: "lock.shield.fill",
                    iconColor: .green,
                    title: "Your Data, Your Control",
                    description: "All recordings stay in your personal cloud storage! We never access your data, unless you explicitly choose to share it."
                )
            }
            .padding(.horizontal, 40)
            
            Spacer()
            
            // Error message if any
            if let error = authError {
                HStack(spacing: 8) {
                    Image(systemName: "exclamationmark.triangle.fill")
                        .foregroundColor(.yellow)
                    Text(error)
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                }
                .padding(.horizontal, 40)
                .padding(.bottom, 16)
            }
            
            // Action Buttons - Side by side
            VStack(spacing: 16) {
                HStack(spacing: 16) {
                    // Google Drive Button
                    Button {
                        Task {
                            await googleAuthManager.startOAuthFlow()
                            if googleAuthManager.isAuthenticated {
                                onSignInComplete()
                            }
                        }
                    } label: {
                        HStack(spacing: 10) {
                            if googleAuthManager.isAuthenticating {
                                ProgressView()
                                    .progressViewStyle(CircularProgressViewStyle(tint: .white))
                            } else {
                                Image(systemName: "g.circle.fill")
                                    .font(.title2)
                            }
                            
                            Text(googleAuthManager.isAuthenticating ? "Signing in..." : "Google Drive")
                                .font(.title3.bold())
                        }
                        .foregroundColor(.white)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                        .background(
                            LinearGradient(
                                colors: [
                                    Color(red: 0.35, green: 0.55, blue: 0.85),
                                    Color(red: 0.30, green: 0.60, blue: 0.82)
                                ],
                                startPoint: .leading,
                                endPoint: .trailing
                            )
                        )
                        .cornerRadius(12)
                    }
                    .buttonStyle(.plain)
                    .disabled(isAuthenticating)
                    
                    // Dropbox Button
                    Button {
                        Task {
                            await dropboxAuthManager.startOAuthFlow()
                            if dropboxAuthManager.isAuthenticated {
                                onSignInComplete()
                            }
                        }
                    } label: {
                        HStack(spacing: 10) {
                            if dropboxAuthManager.isAuthenticating {
                                ProgressView()
                                    .progressViewStyle(CircularProgressViewStyle(tint: .white))
                            } else {
                                Image(systemName: "shippingbox.fill")
                                    .font(.title2)
                            }
                            
                            Text(dropboxAuthManager.isAuthenticating ? "Signing in..." : "Dropbox")
                                .font(.title3.bold())
                        }
                        .foregroundColor(.white)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                        .background(
                            LinearGradient(
                                colors: [
                                    Color(red: 0.25, green: 0.45, blue: 0.75),
                                    Color(red: 0.30, green: 0.50, blue: 0.78)
                                ],
                                startPoint: .leading,
                                endPoint: .trailing
                            )
                        )
                        .cornerRadius(12)
                    }
                    .buttonStyle(.plain)
                    .disabled(isAuthenticating)
                }
                
                // Skip buttons - side by side with light background
                HStack(spacing: 16) {
                    Button {
                        onSkip()
                    } label: {
                        Text("Skip for now")
                            .font(.body)
                            .foregroundColor(.secondary)
                            .padding(.horizontal, 20)
                            .padding(.vertical, 12)
                            .background(Color.white.opacity(0.1))
                            .cornerRadius(10)
                            .contentShape(Rectangle())
                    }
                    .buttonStyle(.plain)
                    .disabled(isAuthenticating)
                    
                    Button {
                        onSkipForever()
                    } label: {
                        Text("Don't ask again")
                            .font(.body)
                            .foregroundColor(.secondary)
                            .padding(.horizontal, 20)
                            .padding(.vertical, 12)
                            .background(Color.white.opacity(0.1))
                            .cornerRadius(10)
                            .contentShape(Rectangle())
                    }
                    .buttonStyle(.plain)
                    .disabled(isAuthenticating)
                }
                .padding(.top, 8)
            }
            .padding(.horizontal, 40)
            .padding(.bottom, 40)
        }
        .frame(minWidth: 700, minHeight: 600)
    }
}

// MARK: - iOS App Showcase Mode
enum IOSAppShowcaseMode {
    case afterVisionOSSignIn    // User signed in on visionOS
    case afterSkip              // User skipped sign-in (remind about iOS sign-in)
    
    var videoAssetName: String {
        switch self {
        case .afterVisionOSSignIn: return "vpt4"
        case .afterSkip: return "vpt5"
        }
    }
}

// MARK: - iOS App Sign-In Detected View (Case 1A)
/// Shown when user signed in via iOS app first, then opens visionOS app
struct IOSAppSignInDetectedView: View {
    let onGetStarted: () -> Void
    let onRemindNextTime: () -> Void
    
    // Video player for the iPhone mockup
    @State private var player: AVPlayer?
    @ObservedObject private var cloudStorageSettings = CloudStorageSettings.shared
    
    private var connectedServiceName: String {
        if cloudStorageSettings.isGoogleDriveAvailable {
            return "Google Drive"
        } else if cloudStorageSettings.isDropboxAvailable {
            return "Dropbox"
        }
        return "cloud storage"
    }
    
    private var connectedServiceIcon: String {
        if cloudStorageSettings.isGoogleDriveAvailable {
            return "g.circle.fill"
        } else if cloudStorageSettings.isDropboxAvailable {
            return "shippingbox.fill"
        }
        return "icloud.fill"
    }
    
    var body: some View {
        VStack(spacing: 0) {
            // Header with success message
            VStack(spacing: 16) {
                // Success checkmark icon
                ZStack {
                    Circle()
                        .fill(
                            LinearGradient(
                                colors: [Color.green, Color.green.opacity(0.7)],
                                startPoint: .topLeading,
                                endPoint: .bottomTrailing
                            )
                        )
                        .frame(width: 80, height: 80)
                    
                    Image(systemName: "checkmark.circle.fill")
                        .font(.system(size: 44))
                        .foregroundColor(.white)
                }
                .padding(.top, 40)
                
                Text("You're All Set!")
                    .font(.system(size: 36, weight: .bold))
                
                // Detected sign-in message
                HStack(spacing: 8) {
                    Image(systemName: connectedServiceIcon)
                        .font(.title3)
                        .foregroundColor(.blue)
                    Text("Connected via \(connectedServiceName) from our iOS app")
                        .font(.title3)
                        .foregroundColor(.secondary)
                }
            }
            .padding(.bottom, 40)
            
            // Info about what this means
            VStack(alignment: .leading, spacing: 20) {
                BenefitRow(
                    icon: "icloud.and.arrow.up.fill",
                    iconColor: .green,
                    title: "Recordings Auto-Sync",
                    description: "Your sessions will automatically upload to \(connectedServiceName)"
                )
                
                BenefitRow(
                    icon: "iphone",
                    iconColor: .blue,
                    title: "Review on iOS",
                    description: "Browse and replay your recordings using the Tracking Manager iOS app"
                )
                
                BenefitRow(
                    icon: "arrow.triangle.2.circlepath",
                    iconColor: .purple,
                    title: "Always in Sync",
                    description: "Changes made on either device are reflected everywhere"
                )
            }
            .padding(.horizontal, 40)
            
            Spacer()
            
            // Get Started button and Remind me option - side by side
            HStack(spacing: 16) {
                Button {
                    player?.pause()
                    onRemindNextTime()
                } label: {
                    Text("Remind me later")
                        .font(.body)
                        .foregroundColor(.secondary)
                        .padding(.horizontal, 24)
                        .padding(.vertical, 14)
                        .background(Color.white.opacity(0.1))
                        .cornerRadius(10)
                        .contentShape(Rectangle())
                }
                .buttonStyle(.plain)
                
                Button {
                    player?.pause()
                    onGetStarted()
                } label: {
                    Text("Get Started")
                        .font(.body.bold())
                        .foregroundColor(.white)
                        .padding(.horizontal, 32)
                        .padding(.vertical, 14)
                        .background(
                            LinearGradient(
                                colors: [Color.green, Color.orange],
                                startPoint: .leading,
                                endPoint: .trailing
                            )
                        )
                        .cornerRadius(10)
                        .contentShape(Rectangle())
                }
                .buttonStyle(.plain)
            }
            .padding(.bottom, 48)
        }
        .frame(minWidth: 700, minHeight: 600)
    }
}

/// Compact feature chip for horizontal layout
struct FeatureChip: View {
    let icon: String
    let label: String
    let color: Color
    
    var body: some View {
        VStack(spacing: 6) {
            Image(systemName: icon)
                .font(.title2)
                .foregroundColor(color)
            Text(label)
                .font(.caption)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
        }
        .frame(width: 90)
    }
}

// MARK: - iOS App Showcase View
struct IOSAppShowcaseView: View {
    let showcaseMode: IOSAppShowcaseMode
    let onGetStarted: () -> Void
    let onRemindNextTime: () -> Void
    
    // Video player for the iPhone mockup
    @State private var player: AVPlayer?
    
    /// Whether this view is shown after user skipped cloud sign-in
    private var isSkippedMode: Bool {
        showcaseMode == .afterSkip
    }
    
    private var videoAssetName: String {
        showcaseMode.videoAssetName
    }
    
    var body: some View {
        VStack(spacing: 0) {
            // Header
            VStack(spacing: 8) {
                Text("Companion iOS App: Tracking Manager")
                    .font(.system(size: 36, weight: .bold))
                
                Text(isSkippedMode 
                    ? "You can sign in anytime via the iOS app"
                    : "Review and manage your recordings on the go")
                    .font(.title3)
                    .foregroundColor(.secondary)
            }
            .padding(.top, 40)
            .padding(.bottom, 32)
            
            // Main content: Features on left, iPhone mockup on right
            HStack(alignment: .center, spacing: 60) {
                // Left side: Feature list
                VStack(alignment: .leading, spacing: 24) {
                    // Show sign-in reminder first when user skipped
                    if isSkippedMode {
                        ShowcaseFeatureRow(
                            icon: "icloud.and.arrow.up",
                            iconColor: .cyan,
                            title: "Sign In Anytime",
                            description: "Connect Google Drive or Dropbox through the iOS app whenever you're ready"
                        )
                    }
                    
                    ShowcaseFeatureRow(
                        icon: "play.rectangle.fill",
                        iconColor: .blue,
                        title: "Browse All Recordings",
                        description: "Access simulation, real-world, and egocentric recordings in one place"
                    )
                    
                    ShowcaseFeatureRow(
                        icon: "globe",
                        iconColor: .green,
                        title: "Share with Community",
                        description: "Make recordings public with a single tap for others to learn from"
                    )
                    
                    ShowcaseFeatureRow(
                        icon: "person.2.fill",
                        iconColor: .purple,
                        title: "Explore Public Library",
                        description: "Discover and learn from recordings shared by the community"
                    )
                    
                    ShowcaseFeatureRow(
                        icon: "camera.viewfinder",
                        iconColor: .orange,
                        title: "Camera Calibration",
                        description: "One-stop setup for external cameras in egocentric mode"
                    )
                }
                .frame(maxWidth: 340)
                
                // Right side: iPhone mockup with video
                iPhoneMockup {
                    // Video player placeholder - will show video when available
                    if let player = player {
                        VideoPlayer(player: player)
                            .onAppear {
                                player.play()
                            }
                            .onDisappear {
                                player.pause()
                            }
                    } else {
                        // Placeholder when no video is available
                        ZStack {
                            Color.black
                            
                            VStack(spacing: 16) {
                                Image(systemName: "iphone.gen3")
                                    .font(.system(size: 48))
                                    .foregroundColor(.gray.opacity(0.5))
                                
                                Text("Demo Video")
                                    .font(.headline)
                                    .foregroundColor(.gray.opacity(0.7))
                                
                                Text("Coming Soon")
                                    .font(.caption)
                                    .foregroundColor(.gray.opacity(0.5))
                            }
                        }
                    }
                }
                .frame(width: 220, height: 440)
            }
            .padding(.horizontal, 40)
            
            Spacer(minLength: 32)
            
            // Get Started button and Remind me option - side by side
            HStack(spacing: 16) {
                Button {
                    player?.pause()
                    onRemindNextTime()
                } label: {
                    Text("Remind me later")
                        .font(.body)
                        .foregroundColor(.secondary)
                        .padding(.horizontal, 24)
                        .padding(.vertical, 14)
                        .background(Color.white.opacity(0.1))
                        .cornerRadius(10)
                        .contentShape(Rectangle())
                }
                .buttonStyle(.plain)
                
                Button {
                    player?.pause()
                    onGetStarted()
                } label: {
                    Text("Get Started")
                        .font(.body.bold())
                        .foregroundColor(.white)
                        .padding(.horizontal, 32)
                        .padding(.vertical, 14)
                        .background(
                            LinearGradient(
                                colors: [Color.green, Color.orange],
                                startPoint: .leading,
                                endPoint: .trailing
                            )
                        )
                        .cornerRadius(10)
                        .contentShape(Rectangle())
                }
                .buttonStyle(.plain)
            }
            .padding(.bottom, 48)
        }
        .frame(minWidth: 800, minHeight: 700)
        .onAppear {
            loadVideo()
        }
    }
    
    private func loadVideo() {
        // Try to load the demo video from Assets catalog
        if let dataAsset = NSDataAsset(name: videoAssetName) {
            // Write to temporary file since AVPlayer needs a URL
            let tempDir = FileManager.default.temporaryDirectory
            let tempURL = tempDir.appendingPathComponent("\(videoAssetName)_demo.mp4")
            
            do {
                // Remove existing file if present
                try? FileManager.default.removeItem(at: tempURL)
                try dataAsset.data.write(to: tempURL)
                
                player = AVPlayer(url: tempURL)
                player?.isMuted = true
                player?.actionAtItemEnd = .none
                
                // Loop the video
                NotificationCenter.default.addObserver(
                    forName: .AVPlayerItemDidPlayToEndTime,
                    object: player?.currentItem,
                    queue: .main
                ) { _ in
                    player?.seek(to: .zero)
                    player?.play()
                }
            } catch {
                dlog("⚠️ Failed to load demo video: \(error)")
            }
        }
    }
}

// MARK: - iPhone Mockup Component
struct iPhoneMockup<Content: View>: View {
    let content: Content
    
    init(@ViewBuilder content: () -> Content) {
        self.content = content()
    }
    
    var body: some View {
        ZStack {
            // iPhone frame
            RoundedRectangle(cornerRadius: 40)
                .fill(Color.black)
            
            // Inner bezel
            RoundedRectangle(cornerRadius: 36)
                .fill(Color(white: 0.1))
                .padding(4)
            
            // Screen content
            RoundedRectangle(cornerRadius: 32)
                .fill(Color.black)
                .padding(8)
                .overlay(
                    content
                        .clipShape(RoundedRectangle(cornerRadius: 32))
                        .padding(8)
                )
            
            // Dynamic Island
            VStack {
                Capsule()
                    .fill(Color.black)
                    .frame(width: 90, height: 28)
                    .padding(.top, 16)
                Spacer()
            }
            
            // Home indicator
            VStack {
                Spacer()
                Capsule()
                    .fill(Color.white.opacity(0.3))
                    .frame(width: 100, height: 5)
                    .padding(.bottom, 12)
            }
        }
    }
}

// MARK: - Showcase Feature Row
struct ShowcaseFeatureRow: View {
    let icon: String
    let iconColor: Color
    let title: String
    let description: String
    
    var body: some View {
        HStack(alignment: .top, spacing: 14) {
            Image(systemName: icon)
                .font(.title2)
                .foregroundColor(iconColor)
                .frame(width: 28)
            
            VStack(alignment: .leading, spacing: 4) {
                Text(title)
                    .font(.headline)
                Text(description)
                    .font(.subheadline)
                    .foregroundColor(.secondary)
                    .fixedSize(horizontal: false, vertical: true)
            }
        }
    }
}

// MARK: - Benefit Row Component
struct BenefitRow: View {
    let icon: String
    let iconColor: Color
    let title: String
    let description: String
    
    var body: some View {
        HStack(alignment: .top, spacing: 14) {
            Image(systemName: icon)
                .font(.title2)
                .foregroundColor(iconColor)
                .frame(width: 28)
            
            VStack(alignment: .leading, spacing: 3) {
                Text(title)
                    .font(.headline)
                Text(description)
                    .font(.subheadline)
                    .foregroundColor(.secondary)
                    .fixedSize(horizontal: false, vertical: true)
            }
        }
    }
}

func getIPAddresses() -> [(name: String, address: String)] {
    var addresses: [(name: String, address: String)] = []
    var ifaddr: UnsafeMutablePointer<ifaddrs>? = nil
    if getifaddrs(&ifaddr) == 0 {
        var ptr = ifaddr
        while ptr != nil {
            defer { ptr = ptr?.pointee.ifa_next }

            guard let interface = ptr?.pointee else { continue }
            let addrFamily = interface.ifa_addr.pointee.sa_family
            if addrFamily == UInt8(AF_INET) {
                let name: String = String(cString: (interface.ifa_name))
                var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                getnameinfo(interface.ifa_addr, socklen_t((interface.ifa_addr.pointee.sa_len)), &hostname, socklen_t(hostname.count), nil, socklen_t(0), NI_NUMERICHOST)
                let address = String(cString: hostname)
                if address != "127.0.0.1" {
                    addresses.append((name: name, address: address))
                }
            }
        }
        freeifaddrs(ifaddr)
    }
    return addresses
}


func getWiFiName() -> String? {
  // CNCopyCurrentNetworkInfo is unavailable in visionOS
  // WiFi SSID access is restricted on this platform
  return nil
}

// MARK: - Color Extension for Hex Support
extension Color {
    init(hex: String) {
        let hex = hex.trimmingCharacters(in: CharacterSet.alphanumerics.inverted)
        var int: UInt64 = 0
        Scanner(string: hex).scanHexInt64(&int)
        let a, r, g, b: UInt64
        switch hex.count {
        case 3: // RGB (12-bit)
            (a, r, g, b) = (255, (int >> 8) * 17, (int >> 4 & 0xF) * 17, (int & 0xF) * 17)
        case 6: // RGB (24-bit)
            (a, r, g, b) = (255, int >> 16, int >> 8 & 0xFF, int & 0xFF)
        case 8: // ARGB (32-bit)
            (a, r, g, b) = (int >> 24, int >> 16 & 0xFF, int >> 8 & 0xFF, int & 0xFF)
        default:
            (a, r, g, b) = (1, 1, 1, 0)
        }
        self.init(
            .sRGB,
            red: Double(r) / 255,
            green: Double(g) / 255,
            blue: Double(b) / 255,
            opacity: Double(a) / 255
        )
    }
}
