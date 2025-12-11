//
//  ContentView.swift
//  Tracking Viewer
//
//  Created by younghyopark on 11/29/25.
//

import SwiftUI

// MARK: - Main Tab View

struct ContentView: View {
    @State private var selectedTab = 0
    @StateObject private var multipeerManager = MultipeerCalibrationManager_iOS.shared
    @State private var showCalibrationCoordinator = false
    
    var body: some View {
        TabView(selection: $selectedTab) {
            RecordingsView()
                .tabItem {
                    Label("Personal", systemImage: "person.fill")
                }
                .tag(0)
            
            NavigationStack {
                PublicRecordingsView()
            }
            .tabItem {
                Label("Public", systemImage: "globe")
            }
            .tag(1)
            
            NavigationStack {
                PlaylistsListView()
            }
            .tabItem {
                Label("Playlists", systemImage: "folder.fill")
            }
            .tag(2)
            
            CalibrationTabView()
                .tabItem {
                    Label("Calibration", systemImage: "camera.viewfinder")
                }
                .tag(3)
            
            NavigationStack {
                SettingsView()
            }
            .tabItem {
                Label("Settings", systemImage: "gear")
            }
            .tag(4)
        }
        .onAppear {
            // Start advertising for Vision Pro connection immediately
            multipeerManager.startAdvertising()
        }
        .onChange(of: multipeerManager.isInCalibrationMode) { _, isCalibrating in
            // Automatically show calibration coordinator when Vision Pro requests it
            if isCalibrating && !showCalibrationCoordinator {
                showCalibrationCoordinator = true
            }
            // Auto-hide when calibration ends
            if !isCalibrating && showCalibrationCoordinator {
                showCalibrationCoordinator = false
            }
        }
        .fullScreenCover(isPresented: $showCalibrationCoordinator, onDismiss: {
            // Reset calibration mode when user dismisses manually
            if multipeerManager.isInCalibrationMode {
                multipeerManager.isInCalibrationMode = false
                multipeerManager.displayMode = .none
                multipeerManager.stopStatusUpdates()
            }
        }) {
            CalibrationCoordinatorView()
        }
    }
}

// MARK: - Settings View

struct SettingsView: View {
    @StateObject private var storageManager = CloudStorageManager.shared
    @StateObject private var dropboxManager = DropboxManager.shared
    @StateObject private var googleDriveManager = GoogleDriveManager.shared
    @StateObject private var visionOSSettings = VisionOSSettingsManager.shared
    
    var body: some View {
        List {
            // Vision Pro Settings Section (NEW)
            visionProSettingsSection
            
            // Header section explaining sync
            Section {
                VStack(alignment: .leading, spacing: 12) {
                    HStack {
                        Image(systemName: "icloud.and.arrow.up")
                            .font(.title)
                            .foregroundColor(.blue)
                        
                        VStack(alignment: .leading) {
                            Text("Cloud Storage")
                                .font(.headline)
                            Text("Choose where recordings are saved")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                    
                    HStack(spacing: 8) {
                        Image(systemName: "checkmark.circle.fill")
                            .foregroundColor(.green)
                            .font(.caption)
                        Text("Settings sync to Vision Pro automatically")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                }
                .padding(.vertical, 8)
            }
            
            // Storage provider selection
            Section("Storage Provider") {
                ForEach(CloudStorageProvider.allCases) { provider in
                    StorageProviderRow(
                        provider: provider,
                        isSelected: storageManager.selectedProvider == provider,
                        isAvailable: isProviderAvailable(provider)
                    ) {
                        storageManager.selectProvider(provider)
                    }
                }
            }
            
            // Dropbox account section
            Section {
                if storageManager.isDropboxAuthenticated {
                    // Signed in state
                    HStack {
                        VStack(alignment: .leading, spacing: 4) {
                            Text("Dropbox Account")
                                .font(.subheadline)
                            if let accountInfo = storageManager.dropboxAccountInfo {
                                Text(accountInfo)
                                    .font(.caption)
                                    .foregroundColor(.secondary)
                            }
                        }
                        
                        Spacer()
                        
                        Button("Sign Out", role: .destructive) {
                            storageManager.signOutOfDropbox()
                        }
                        .buttonStyle(.bordered)
                        .controlSize(.small)
                    }
                } else {
                    // Sign in button
                    Button {
                        Task {
                            await dropboxManager.startOAuthFlow()
                            // After successful sign-in, automatically switch to Dropbox
                            if dropboxManager.isAuthenticated {
                                storageManager.loadSettings()
                                storageManager.selectProvider(.dropbox)
                            }
                        }
                    } label: {
                        HStack {
                            Image(systemName: "person.badge.plus")
                            Text("Sign in to Dropbox")
                            Spacer()
                            
                            if dropboxManager.isAuthenticating {
                                ProgressView()
                                    .controlSize(.small)
                            } else {
                                Image(systemName: "chevron.right")
                                    .font(.caption)
                                    .foregroundColor(.secondary)
                            }
                        }
                    }
                    .disabled(dropboxManager.isAuthenticating)
                }
            } header: {
                Text("Dropbox")
            } footer: {
                if let error = dropboxManager.authError {
                    Text(error)
                        .foregroundColor(.red)
                } else if !storageManager.isDropboxAuthenticated {
                    Text("Sign in to Dropbox to use it as your storage provider. Your login will be shared with Vision Pro.")
                }
            }
            
            // Google Drive account section
            Section {
                if storageManager.isGoogleDriveAuthenticated {
                    // Signed in state
                    HStack {
                        VStack(alignment: .leading, spacing: 4) {
                            Text("Google Drive Account")
                                .font(.subheadline)
                            if let accountInfo = storageManager.googleDriveAccountInfo {
                                Text(accountInfo)
                                    .font(.caption)
                                    .foregroundColor(.secondary)
                            }
                        }
                        
                        Spacer()
                        
                        Button("Sign Out", role: .destructive) {
                            storageManager.signOutOfGoogleDrive()
                        }
                        .buttonStyle(.bordered)
                        .controlSize(.small)
                    }
                } else {
                    // Sign in button
                    Button {
                        Task {
                            await googleDriveManager.startOAuthFlow()
                            // After successful sign-in, automatically switch to Google Drive
                            if googleDriveManager.isAuthenticated {
                                storageManager.loadSettings()
                                storageManager.selectProvider(.googleDrive)
                            }
                        }
                    } label: {
                        HStack {
                            Image(systemName: "person.badge.plus")
                            Text("Sign in to Google Drive")
                            Spacer()
                            
                            if googleDriveManager.isAuthenticating {
                                ProgressView()
                                    .controlSize(.small)
                            } else {
                                Image(systemName: "chevron.right")
                                    .font(.caption)
                                    .foregroundColor(.secondary)
                            }
                        }
                    }
                    .disabled(googleDriveManager.isAuthenticating)
                }
            } header: {
                Text("Google Drive")
            } footer: {
                if let error = googleDriveManager.authError {
                    Text(error)
                        .foregroundColor(.red)
                } else if !storageManager.isGoogleDriveAuthenticated {
                    Text("Sign in to Google Drive to use it as your storage provider. Your login will be shared with Vision Pro.")
                }
            }
            
            // Help section
            Section {
                NavigationLink {
                    CloudStorageHelpView()
                } label: {
                    Label("How it works", systemImage: "questionmark.circle")
                }
            }
            
            // About Section
            Section("About") {
                HStack {
                    Text("Version")
                    Spacer()
                    Text(Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "1.0")
                        .foregroundColor(.secondary)
                }
                
                HStack {
                    Text("Build")
                    Spacer()
                    Text(Bundle.main.infoDictionary?["CFBundleVersion"] as? String ?? "1")
                        .foregroundColor(.secondary)
                }
            }
            
            // Debug Section
            #if DEBUG
            Section("Debug") {
                Button(action: {
                    // Reset all onboarding keys
                    UserDefaults.standard.removeObject(forKey: "hasSeenWelcome")
                    UserDefaults.standard.removeObject(forKey: "dontShowCloudSignInAgain")
                    UserDefaults.standard.removeObject(forKey: "dontShowVisionOSPromoAgain")
                    UserDefaults.standard.removeObject(forKey: "hasSignedInOnIOS")
                    // Legacy key
                    UserDefaults.standard.removeObject(forKey: "hasCompletedOnboarding")
                }) {
                    Label("Reset Onboarding", systemImage: "arrow.counterclockwise")
                }
                
                Text("Restart app after resetting to see the welcome screen")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            #endif
        }
        .navigationTitle("Settings")
        .alert("Error", isPresented: .init(
            get: { storageManager.errorMessage != nil },
            set: { if !$0 { storageManager.errorMessage = nil } }
        )) {
            Button("OK") {
                storageManager.errorMessage = nil
            }
        } message: {
            if let error = storageManager.errorMessage {
                Text(error)
            }
        }
    }
    
    private func isProviderAvailable(_ provider: CloudStorageProvider) -> Bool {
        switch provider {
        case .iCloudDrive:
            return true
        case .dropbox:
            return storageManager.isDropboxAuthenticated
        case .googleDrive:
            return storageManager.isGoogleDriveAuthenticated
        }
    }
    
    // MARK: - Vision Pro Settings Section
    
    private var visionProSettingsSection: some View {
        Section {
            NavigationLink {
                VisionProSettingsDetailView()
            } label: {
                HStack(spacing: 14) {
                    ZStack {
                        RoundedRectangle(cornerRadius: 8)
                            .fill(LinearGradient(
                                colors: [.purple, .blue],
                                startPoint: .topLeading,
                                endPoint: .bottomTrailing
                            ))
                            .frame(width: 40, height: 40)
                        
                        Image(systemName: "visionpro")
                            .font(.system(size: 18))
                            .foregroundColor(.white)
                    }
                    
                    VStack(alignment: .leading, spacing: 2) {
                        Text("Vision Pro Settings")
                            .font(.body)
                        
                        HStack(spacing: 4) {
                            Image(systemName: "icloud")
                                .font(.caption2)
                            Text("Synced via iCloud")
                                .font(.caption)
                        }
                        .foregroundColor(.secondary)
                    }
                }
                .padding(.vertical, 4)
            }
        } header: {
            Text("Vision Pro")
        } footer: {
            Text("Configure your Vision Pro experience from here. Changes sync automatically.")
        }
    }
}

// MARK: - Vision Pro Settings Detail View

struct VisionProSettingsDetailView: View {
    @StateObject private var settings = VisionOSSettingsManager.shared
    @State private var showResetConfirmation = false
    
    var body: some View {
        List {
            // Sync status header
            Section {
                HStack {
                    Image(systemName: "icloud.fill")
                        .foregroundColor(.blue)
                    
                    VStack(alignment: .leading, spacing: 2) {
                        Text("iCloud Sync")
                            .font(.subheadline)
                            .fontWeight(.medium)
                        
                        if let lastSync = settings.lastSyncTime {
                            Text("Last synced: \(lastSync.formatted(date: .abbreviated, time: .shortened))")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        } else {
                            Text("Waiting for sync...")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                    
                    Spacer()
                    
                    Button {
                        settings.forceRefresh()
                    } label: {
                        Image(systemName: "arrow.clockwise")
                            .font(.body)
                    }
                    .disabled(settings.isSyncing)
                }
                .padding(.vertical, 4)
            }
            
            // App Mode Section
            Section {
                Picker("Mode", selection: Binding(
                    get: { VisionOSAppMode(rawValue: settings.appMode) ?? .teleop },
                    set: { settings.appMode = $0.rawValue }
                )) {
                    ForEach(VisionOSAppMode.allCases) { mode in
                        Label(mode.displayName, systemImage: mode.icon)
                            .tag(mode)
                    }
                }
                .pickerStyle(.menu)
            } header: {
                Text("App Mode")
            } footer: {
                let mode = VisionOSAppMode(rawValue: settings.appMode) ?? .teleop
                Text(mode.description)
            }
            
            // Visualization Settings
            Section {
                Toggle(isOn: $settings.upperLimbVisible) {
                    Label {
                        VStack(alignment: .leading, spacing: 2) {
                            Text("Hands Over AR")
                            Text("Show hands on top of AR content")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    } icon: {
                        Image(systemName: settings.upperLimbVisible ? "hand.raised.fill" : "hand.raised.slash.fill")
                            .foregroundColor(.cyan)
                    }
                }
                
                Toggle(isOn: $settings.showHeadBeam) {
                    Label {
                        VStack(alignment: .leading, spacing: 2) {
                            Text("Head Gaze Ray")
                            Text("Project a ray showing where you're looking")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    } icon: {
                        Image(systemName: settings.showHeadBeam ? "rays" : "circle.dashed")
                            .foregroundColor(.yellow)
                    }
                }
                
                Toggle(isOn: $settings.showHandJoints) {
                    Label {
                        VStack(alignment: .leading, spacing: 2) {
                            Text("Hand Tracking Joints")
                            Text("Show spheres at tracked hand joints")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    } icon: {
                        Image(systemName: settings.showHandJoints ? "circle.grid.3x3.fill" : "circle.grid.3x3")
                            .foregroundColor(.orange)
                    }
                }
                
                if settings.showHandJoints {
                    VStack(alignment: .leading, spacing: 8) {
                        HStack {
                            Text("Joint Opacity")
                            Spacer()
                            Text("\(Int(settings.handJointsOpacity * 100))%")
                                .foregroundColor(.secondary)
                                .monospacedDigit()
                        }
                        
                        Slider(value: $settings.handJointsOpacity, in: 0.1...1.0, step: 0.1)
                            .tint(.orange)
                    }
                    .padding(.leading, 36)
                }
            } header: {
                Text("Visualizations")
            } footer: {
                Text("Control what visual elements are rendered in the immersive space.")
            }
            
            // Video Plane Settings
            Section {
                // Preview visualization
                VideoPlanePreviewView(
                    distance: settings.videoPlaneZDistance,
                    height: settings.videoPlaneYPosition
                )
                .listRowInsets(EdgeInsets())
                .listRowBackground(Color.clear)
                
                VStack(alignment: .leading, spacing: 8) {
                    HStack {
                        Text("Distance")
                        Spacer()
                        Text("\(String(format: "%.1f", settings.videoPlaneZDistance))m")
                            .foregroundColor(.secondary)
                            .monospacedDigit()
                    }
                    
                    Slider(value: $settings.videoPlaneZDistance, in: 2.0...20.0, step: 0.5)
                        .tint(.blue)
                }
                
                VStack(alignment: .leading, spacing: 8) {
                    HStack {
                        Text("Height Offset")
                        Spacer()
                        Text("\(String(format: "%.1f", settings.videoPlaneYPosition))m")
                            .foregroundColor(.secondary)
                            .monospacedDigit()
                    }
                    
                    Slider(value: $settings.videoPlaneYPosition, in: -2.0...2.0, step: 0.1)
                        .tint(.green)
                }
            } header: {
                Text("Video Plane")
            } footer: {
                Text("Position of the video stream in your immersive space.")
            }
            
            // Controller Position Settings
            Section {
                // Preview visualization
                ControllerPositionPreviewView(
                    xPosition: settings.statusMinimizedXPosition,
                    yPosition: settings.statusMinimizedYPosition
                )
                .listRowInsets(EdgeInsets())
                .listRowBackground(Color.clear)
                
                VStack(alignment: .leading, spacing: 8) {
                    HStack {
                        Text("Horizontal Position")
                        Spacer()
                        Text("\(String(format: "%.2f", settings.statusMinimizedXPosition))")
                            .foregroundColor(.secondary)
                            .monospacedDigit()
                    }
                    
                    Slider(value: $settings.statusMinimizedXPosition, in: -0.5...0.5, step: 0.05)
                        .tint(.purple)
                }
                
                VStack(alignment: .leading, spacing: 8) {
                    HStack {
                        Text("Vertical Position")
                        Spacer()
                        Text("\(String(format: "%.2f", settings.statusMinimizedYPosition))")
                            .foregroundColor(.secondary)
                            .monospacedDigit()
                    }
                    
                    Slider(value: $settings.statusMinimizedYPosition, in: -0.5...0.5, step: 0.05)
                        .tint(.purple)
                }
                
                Button {
                    settings.statusMinimizedXPosition = 0.0
                    settings.statusMinimizedYPosition = -0.3
                } label: {
                    Label("Reset Position", systemImage: "arrow.counterclockwise")
                }
            } header: {
                Text("Controller Position")
            } footer: {
                Text("Position of the minimized controller in your field of view.")
            }
            
            // Recording Settings
            Section {
                Toggle(isOn: $settings.autoRecordingEnabled) {
                    Label {
                        VStack(alignment: .leading, spacing: 2) {
                            Text("Auto-Record on Connect")
                            Text("Start recording when client connects")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    } icon: {
                        Image(systemName: "record.circle")
                            .foregroundColor(.red)
                    }
                }
            } header: {
                Text("Recording")
            }
            
            // Reset Section
            Section {
                Button(role: .destructive) {
                    showResetConfirmation = true
                } label: {
                    HStack {
                        Image(systemName: "arrow.counterclockwise")
                        Text("Reset to Defaults")
                    }
                }
            } footer: {
                Text("This will reset all Vision Pro settings to their default values.")
            }
        }
        .navigationTitle("Vision Pro Settings")
        .navigationBarTitleDisplayMode(.inline)
        .alert("Reset Settings?", isPresented: $showResetConfirmation) {
            Button("Cancel", role: .cancel) {}
            Button("Reset", role: .destructive) {
                settings.resetToDefaults()
            }
        } message: {
            Text("This will reset all Vision Pro settings to their default values. This cannot be undone.")
        }
    }
}

// MARK: - Video Plane Preview

struct VideoPlanePreviewView: View {
    let distance: Float
    let height: Float
    
    // Normalize distance (2-20m) to visual position (0-1)
    private var normalizedDistance: CGFloat {
        CGFloat((distance - 2.0) / 18.0)
    }
    
    // Normalize height (-2 to 2m) to visual position (0-1, inverted for display)
    private var normalizedHeight: CGFloat {
        CGFloat(1.0 - (height + 2.0) / 4.0)
    }
    
    var body: some View {
        VStack(spacing: 4) {
            // Side view schematic
            GeometryReader { geometry in
                let width = geometry.size.width
                let viewHeight = geometry.size.height
                
                ZStack {
                    // Background grid
                    Path { path in
                        // Horizontal lines
                        for i in 0..<5 {
                            let y = viewHeight * CGFloat(i) / 4.0
                            path.move(to: CGPoint(x: 0, y: y))
                            path.addLine(to: CGPoint(x: width, y: y))
                        }
                        // Vertical lines
                        for i in 0..<7 {
                            let x = width * CGFloat(i) / 6.0
                            path.move(to: CGPoint(x: x, y: 0))
                            path.addLine(to: CGPoint(x: x, y: viewHeight))
                        }
                    }
                    .stroke(Color.gray.opacity(0.2), lineWidth: 0.5)
                    
                    // Floor line
                    Path { path in
                        let floorY = viewHeight * 0.7
                        path.move(to: CGPoint(x: 0, y: floorY))
                        path.addLine(to: CGPoint(x: width, y: floorY))
                    }
                    .stroke(Color.gray.opacity(0.5), style: StrokeStyle(lineWidth: 1, dash: [4, 4]))
                    
                    // Person/headset icon (left side)
                    let personX: CGFloat = 30
                    let personY = viewHeight * 0.45
                    
                    // Vision Pro headset shape
                    RoundedRectangle(cornerRadius: 4)
                        .fill(Color.blue.opacity(0.6))
                        .frame(width: 24, height: 14)
                        .position(x: personX, y: personY)
                    
                    // Eye direction indicator
                    Path { path in
                        path.move(to: CGPoint(x: personX + 12, y: personY))
                        path.addLine(to: CGPoint(x: width - 20, y: personY))
                    }
                    .stroke(Color.blue.opacity(0.3), style: StrokeStyle(lineWidth: 1, dash: [2, 2]))
                    
                    // Video plane position
                    let planeX = 50 + (width - 80) * normalizedDistance
                    let planeY = viewHeight * 0.45 + (normalizedHeight - 0.5) * viewHeight * 0.4
                    
                    // Video plane rectangle
                    RoundedRectangle(cornerRadius: 2)
                        .fill(
                            LinearGradient(
                                colors: [Color.green.opacity(0.8), Color.blue.opacity(0.8)],
                                startPoint: .topLeading,
                                endPoint: .bottomTrailing
                            )
                        )
                        .frame(width: 8, height: 40)
                        .position(x: planeX, y: planeY)
                    
                    // Connection line from headset to plane
                    Path { path in
                        path.move(to: CGPoint(x: personX + 12, y: personY))
                        path.addLine(to: CGPoint(x: planeX - 4, y: planeY))
                    }
                    .stroke(Color.green.opacity(0.4), lineWidth: 1)
                    
                    // Distance label
                    Text("\(String(format: "%.1f", distance))m")
                        .font(.system(size: 10, weight: .medium, design: .monospaced))
                        .foregroundColor(.blue)
                        .position(x: (personX + planeX) / 2, y: personY + 25)
                    
                    // Height label (if not at eye level)
                    if abs(height) > 0.1 {
                        Text(height > 0 ? "+\(String(format: "%.1f", height))m" : "\(String(format: "%.1f", height))m")
                            .font(.system(size: 10, weight: .medium, design: .monospaced))
                            .foregroundColor(.green)
                            .position(x: planeX + 25, y: planeY)
                    }
                }
            }
            .frame(height: 100)
            .background(Color(UIColor.secondarySystemBackground))
            .cornerRadius(8)
            .padding(.horizontal, 16)
            .padding(.vertical, 8)
            
            // Labels
            HStack {
                Label("Headset", systemImage: "visionpro")
                    .font(.caption2)
                    .foregroundColor(.blue)
                Spacer()
                Label("Video Plane", systemImage: "rectangle.inset.filled")
                    .font(.caption2)
                    .foregroundColor(.green)
            }
            .padding(.horizontal, 20)
        }
    }
}

// MARK: - Controller Position Preview

struct ControllerPositionPreviewView: View {
    let xPosition: Float
    let yPosition: Float
    
    // Normalize positions (-0.5 to 0.5) to visual position (0-1)
    private var normalizedX: CGFloat {
        CGFloat((xPosition + 0.5))
    }
    
    private var normalizedY: CGFloat {
        CGFloat(1.0 - (yPosition + 0.5))
    }
    
    var body: some View {
        VStack(spacing: 4) {
            // Field of view representation
            GeometryReader { geometry in
                let width = geometry.size.width
                let viewHeight = geometry.size.height
                
                ZStack {
                    // FOV outline (rounded rectangle representing view area)
                    RoundedRectangle(cornerRadius: 16)
                        .stroke(Color.purple.opacity(0.3), lineWidth: 2)
                    
                    // Grid overlay
                    Path { path in
                        // Center crosshairs
                        path.move(to: CGPoint(x: width / 2, y: 10))
                        path.addLine(to: CGPoint(x: width / 2, y: viewHeight - 10))
                        path.move(to: CGPoint(x: 10, y: viewHeight / 2))
                        path.addLine(to: CGPoint(x: width - 10, y: viewHeight / 2))
                    }
                    .stroke(Color.gray.opacity(0.2), style: StrokeStyle(lineWidth: 1, dash: [4, 4]))
                    
                    // Center marker
                    Circle()
                        .stroke(Color.gray.opacity(0.3), lineWidth: 1)
                        .frame(width: 8, height: 8)
                        .position(x: width / 2, y: viewHeight / 2)
                    
                    // Controller position indicator
                    let controllerX = 20 + (width - 40) * normalizedX
                    let controllerY = 20 + (viewHeight - 40) * normalizedY
                    
                    // Controller icon representation
                    VStack(spacing: 2) {
                        // Status pill shape
                        RoundedRectangle(cornerRadius: 8)
                            .fill(Color.purple.opacity(0.8))
                            .frame(width: 50, height: 24)
                            .overlay(
                                HStack(spacing: 4) {
                                    Circle()
                                        .fill(Color.green)
                                        .frame(width: 6, height: 6)
                                    Rectangle()
                                        .fill(Color.white.opacity(0.5))
                                        .frame(width: 20, height: 4)
                                        .cornerRadius(2)
                                }
                            )
                            .shadow(color: Color.purple.opacity(0.3), radius: 4, x: 0, y: 2)
                    }
                    .position(x: controllerX, y: controllerY)
                    
                    // Position label
                    Text("(\(String(format: "%.2f", xPosition)), \(String(format: "%.2f", yPosition)))")
                        .font(.system(size: 9, weight: .medium, design: .monospaced))
                        .foregroundColor(.purple)
                        .padding(.horizontal, 6)
                        .padding(.vertical, 2)
                        .background(Color(UIColor.secondarySystemBackground).opacity(0.8))
                        .cornerRadius(4)
                        .position(x: controllerX, y: controllerY + 22)
                    
                    // Corner labels
                    Text("Top-Left")
                        .font(.system(size: 8))
                        .foregroundColor(.gray.opacity(0.5))
                        .position(x: 35, y: 15)
                    
                    Text("Top-Right")
                        .font(.system(size: 8))
                        .foregroundColor(.gray.opacity(0.5))
                        .position(x: width - 40, y: 15)
                    
                    Text("Bottom-Left")
                        .font(.system(size: 8))
                        .foregroundColor(.gray.opacity(0.5))
                        .position(x: 42, y: viewHeight - 15)
                    
                    Text("Bottom-Right")
                        .font(.system(size: 8))
                        .foregroundColor(.gray.opacity(0.5))
                        .position(x: width - 48, y: viewHeight - 15)
                }
            }
            .frame(height: 120)
            .background(Color(UIColor.secondarySystemBackground))
            .cornerRadius(12)
            .padding(.horizontal, 16)
            .padding(.vertical, 8)
            
            // Label
            Text("Field of View")
                .font(.caption2)
                .foregroundColor(.secondary)
        }
    }
}

// MARK: - Search Filter

enum RecordingFilter: String, CaseIterable {
    case all = "All"
    case favorites = "Favorites"
    case hasNotes = "Has Notes"
    case today = "Today"
    case thisWeek = "This Week"
    case thisMonth = "This Month"
    
    var icon: String {
        switch self {
        case .all: return "square.grid.2x2"
        case .favorites: return "star.fill"
        case .hasNotes: return "note.text"
        case .today: return "calendar"
        case .thisWeek: return "calendar.badge.clock"
        case .thisMonth: return "calendar.circle"
        }
    }
}

// MARK: - Bulk Action Type

enum BulkActionType {
    case deleting
    case makingPublic
    case makingPrivate
    
    var title: String {
        switch self {
        case .deleting: return "Deleting"
        case .makingPublic: return "Making Public"
        case .makingPrivate: return "Making Private"
        }
    }
    
    var icon: String {
        switch self {
        case .deleting: return "trash"
        case .makingPublic: return "globe"
        case .makingPrivate: return "lock.fill"
        }
    }
    
    var color: Color {
        switch self {
        case .deleting: return .red
        case .makingPublic: return .green
        case .makingPrivate: return .gray
        }
    }
}

// MARK: - Recordings View

struct RecordingsView: View {
    @StateObject private var recordingsManager = RecordingsManager.shared
    @StateObject private var playlistManager = PlaylistManager.shared
    @StateObject private var annotationsManager = AnnotationsManager.shared
    @StateObject private var httpServer = HTTPFileServer.shared
    @StateObject private var cloudStorageManager = CloudStorageManager.shared
    @StateObject private var dropboxManager = DropboxManager.shared
    @State private var isSelectionMode = false
    @State private var showExportSheet = false
    @State private var exportURLs: [URL] = []
    @State private var showDeleteConfirmation = false
    @State private var recordingToDelete: Recording?
    @State private var showDeleteSelectedConfirmation = false
    @State private var showExportAllSheet = false
    @State private var showAddToPlaylistSheet = false
    @State private var recordingForPlaylist: Recording?
    @State private var showAddSelectedToPlaylistSheet = false
    @State private var searchText = ""
    @State private var selectedFilter: RecordingFilter = .all
    @State private var selectedTagFilter: Tag? = nil
    @State private var showFilterSheet = false
    @State private var showTagPickerForRecording: Recording?
    @State private var showRenameSheet: Recording?
    @State private var showNotesSheet: Recording?
    
    // Unified recording states (for cloud sources)
    @State private var showTagPickerForUnifiedRecording: AnyRecording?
    @State private var showNotesSheetForUnifiedRecording: AnyRecording?
    @State private var unifiedRecordingForPlaylist: AnyRecording?
    @State private var showAddSelectedUnifiedToPlaylistSheet = false
    @State private var showDeleteSelectedUnifiedConfirmation = false
    @State private var unifiedRecordingToDelete: AnyRecording?
    @State private var showDeleteUnifiedConfirmation = false
    @State private var isProcessingBulkAction = false
    
    // Bulk action progress tracking
    @State private var bulkActionProgress: Double = 0
    @State private var bulkActionTotal: Int = 0
    @State private var bulkActionCurrent: Int = 0
    @State private var bulkActionType: BulkActionType?
    
    private let columns = [
        GridItem(.flexible(), spacing: 16)
    ]
    
    private var filteredRecordings: [Recording] {
        var recordings = recordingsManager.recordings
        
        // Apply quick filter
        switch selectedFilter {
        case .all:
            break
        case .favorites:
            recordings = recordings.filter { annotationsManager.isFavorite($0) }
        case .hasNotes:
            recordings = recordings.filter { !annotationsManager.notes(for: $0).isEmpty }
        case .today:
            let today = Calendar.current.startOfDay(for: Date())
            recordings = recordings.filter { $0.createdAt >= today }
        case .thisWeek:
            let weekAgo = Calendar.current.date(byAdding: .day, value: -7, to: Date()) ?? Date()
            recordings = recordings.filter { $0.createdAt >= weekAgo }
        case .thisMonth:
            let monthAgo = Calendar.current.date(byAdding: .month, value: -1, to: Date()) ?? Date()
            recordings = recordings.filter { $0.createdAt >= monthAgo }
        }
        
        // Apply tag filter
        if let tag = selectedTagFilter {
            recordings = recordings.filter { annotationsManager.hasTag($0, tag: tag) }
        }
        
        // Apply search text
        if !searchText.isEmpty {
            recordings = recordings.filter { recording in
                // Search in display name (including custom name)
                let displayName = annotationsManager.displayName(for: recording)
                if displayName.localizedCaseInsensitiveContains(searchText) {
                    return true
                }
                
                // Search in original ID
                if recording.id.localizedCaseInsensitiveContains(searchText) {
                    return true
                }
                
                // Search in notes
                let notes = annotationsManager.notes(for: recording)
                if notes.localizedCaseInsensitiveContains(searchText) {
                    return true
                }
                
                // Search in tags
                let tagNames = annotationsManager.tagNames(for: recording)
                if tagNames.contains(where: { $0.localizedCaseInsensitiveContains(searchText) }) {
                    return true
                }
                
                // Search by duration (e.g., "1:30" or "90s")
                if let duration = recording.metadata?.duration {
                    let durationString = recording.durationString
                    if durationString.contains(searchText) {
                        return true
                    }
                    // Also match seconds like "90s"
                    if searchText.hasSuffix("s"), let seconds = Int(searchText.dropLast()) {
                        if Int(duration) == seconds {
                            return true
                        }
                    }
                }
                
                return false
            }
        }
        
        return recordings
    }
    
    private var hasActiveFilters: Bool {
        selectedFilter != .all || selectedTagFilter != nil || !searchText.isEmpty
    }
    
    /// Get filtered count for the current source
    private var currentFilteredCount: Int {
        switch recordingsManager.currentSource {
        case .iCloudDrive:
            return filteredRecordings.count
        case .dropbox:
            return filteredUnifiedRecordings(recordingsManager.dropboxRecordings.map { AnyRecording($0) }).count
        case .googleDrive:
            return filteredUnifiedRecordings(recordingsManager.googleDriveRecordings.map { AnyRecording($0) }).count
        }
    }
    
    var body: some View {
        NavigationStack {
            VStack(spacing: 0) {
                // Filter bar (show for all sources when there are recordings)
                if recordingsManager.currentRecordingsCount > 0 {
                    filterBar
                }
                
                Group {
                    if recordingsManager.isLoading {
                        ProgressView("Loading recordings...")
                            .frame(maxWidth: .infinity, maxHeight: .infinity)
                    } else if recordingsManager.currentSource == .iCloudDrive {
                        // iCloud Drive content
                        if recordingsManager.recordings.isEmpty {
                            emptyStateView
                        } else if filteredRecordings.isEmpty {
                            noResultsView
                        } else {
                            recordingsGridView
                        }
                    } else if recordingsManager.currentSource == .dropbox {
                        // Dropbox content
                        if !dropboxManager.isAuthenticated {
                            cloudNotSignedInView(provider: .dropbox)
                        } else if recordingsManager.dropboxRecordings.isEmpty {
                            cloudEmptyView(provider: .dropbox)
                        } else {
                            dropboxRecordingsView
                        }
                    } else if recordingsManager.currentSource == .googleDrive {
                        // Google Drive content
                        if !cloudStorageManager.isGoogleDriveAuthenticated {
                            cloudNotSignedInView(provider: .googleDrive)
                        } else if recordingsManager.googleDriveRecordings.isEmpty {
                            cloudEmptyView(provider: .googleDrive)
                        } else {
                            googleDriveRecordingsView
                        }
                    }
                }
            }
            .onDisappear {
                FeedPlayerManager.shared.stopPlayback()
            }
            .navigationTitle("Recordings")
            .navigationBarTitleDisplayMode(.large)
            .searchable(text: $searchText, prompt: "Search name, tags, notes...")
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    HStack {
                        Button(action: {
                            Task { await recordingsManager.loadRecordings() }
                        }) {
                            Image(systemName: "arrow.clockwise")
                        }
                        
                        // HTTP Server button
                        NavigationLink(destination: HTTPServerView()) {
                            Image(systemName: httpServer.isRunning ? "antenna.radiowaves.left.and.right" : "antenna.radiowaves.left.and.right.slash")
                                .foregroundColor(httpServer.isRunning ? .green : .gray)
                        }
                    }
                }
                
                ToolbarItem(placement: .topBarTrailing) {
                    HStack(spacing: 12) {
                        // Show menu and select button for all sources with recordings
                        if recordingsManager.currentRecordingsCount > 0 {
                            if recordingsManager.currentSource == .iCloudDrive {
                                Menu {
                                    Button(action: {
                                        Task {
                                            exportURLs = await recordingsManager.exportRecordings(recordingsManager.recordings)
                                            showExportAllSheet = true
                                        }
                                    }) {
                                        Label("Export All", systemImage: "square.and.arrow.up.on.square")
                                    }
                                } label: {
                                    Image(systemName: "ellipsis.circle")
                                }
                            }
                            
                            Button(isSelectionMode ? "Done" : "Select") {
                                withAnimation {
                                    isSelectionMode.toggle()
                                    if !isSelectionMode {
                                        recordingsManager.deselectAll()
                                    }
                                }
                            }
                        }
                    }
                }
            }
            .overlay(alignment: .bottom) {
                if isSelectionMode && recordingsManager.selectedCount > 0 {
                    selectionToolbar
                }
            }
            .overlay {
                // Bulk action progress overlay
                if isProcessingBulkAction, let actionType = bulkActionType {
                    BulkActionProgressOverlay(
                        actionType: actionType,
                        current: bulkActionCurrent,
                        total: bulkActionTotal,
                        progress: bulkActionProgress
                    )
                }
            }
            .task {
                await recordingsManager.loadRecordings()
                await CloudKitManager.shared.fetchPublicRecordings()
            }
            .sheet(isPresented: $showExportSheet) {
                ShareSheet(items: exportURLs)
            }
            .alert("Delete Recording?", isPresented: $showDeleteConfirmation) {
                Button("Cancel", role: .cancel) {}
                Button("Delete", role: .destructive) {
                    if let recording = recordingToDelete {
                        Task {
                            playlistManager.removeRecordingFromAllPlaylists(recording)
                            annotationsManager.removeAnnotation(for: recording)
                            try? await recordingsManager.deleteRecording(recording)
                        }
                    }
                }
            } message: {
                Text("This will permanently delete the recording and all its data.")
            }
            .alert("Delete \(recordingsManager.selectedRecordings.count) Recordings?", isPresented: $showDeleteSelectedConfirmation) {
                Button("Cancel", role: .cancel) {}
                Button("Delete All", role: .destructive) {
                    Task {
                        await deleteSelectedRecordingsWithProgress()
                    }
                }
            } message: {
                Text("This will permanently delete all selected recordings and their data.")
            }
            .alert("Delete \(recordingsManager.selectedCount) Recordings?", isPresented: $showDeleteSelectedUnifiedConfirmation) {
                Button("Cancel", role: .cancel) {}
                Button("Delete All", role: .destructive) {
                    Task {
                        await deleteSelectedUnifiedWithProgress()
                    }
                }
            } message: {
                Text("This will permanently delete all selected recordings from \(recordingsManager.currentSource.displayName).")
            }
            .alert("Delete Recording?", isPresented: $showDeleteUnifiedConfirmation) {
                Button("Cancel", role: .cancel) {}
                Button("Delete", role: .destructive) {
                    if let recording = unifiedRecordingToDelete {
                        Task {
                            await recordingsManager.deleteUnifiedRecording(recording)
                        }
                    }
                }
            } message: {
                Text("This will permanently delete this recording from \(recordingsManager.currentSource.displayName).")
            }
            .sheet(isPresented: $showExportAllSheet) {
                ShareSheet(items: exportURLs)
            }
            .sheet(item: $recordingForPlaylist) { recording in
                AddRecordingToPlaylistsSheet(recording: recording)
            }
            .sheet(isPresented: $showAddSelectedToPlaylistSheet) {
                AddMultipleRecordingsToPlaylistSheet(recordings: Array(recordingsManager.selectedRecordings))
            }
            .sheet(item: $showTagPickerForRecording) { recording in
                TagPickerSheet(recording: recording)
            }
            .sheet(item: $showRenameSheet) { recording in
                RenameRecordingSheet(recording: recording)
            }
            .sheet(item: $showNotesSheet) { recording in
                NotesEditorSheet(recording: recording)
            }
            // Unified recording sheets (for cloud sources)
            .sheet(item: $showTagPickerForUnifiedRecording) { recording in
                TagPickerView(recordingID: recording.id)
            }
            .sheet(item: $showNotesSheetForUnifiedRecording) { recording in
                NotesEditorSheet(recordingID: recording.id, recordingName: recording.displayName)
            }
            .sheet(item: $unifiedRecordingForPlaylist) { recording in
                AddAnyRecordingToPlaylistsSheet(recording: recording)
            }
            .sheet(isPresented: $showAddSelectedUnifiedToPlaylistSheet) {
                AddMultipleUnifiedRecordingsToPlaylistSheet(recordings: recordingsManager.selectedUnifiedRecordings)
            }
        }
    }
    
    // MARK: - Filter Bar
    
    private var filterBar: some View {
        VStack(spacing: 8) {
            // Quick filters
            ScrollView(.horizontal, showsIndicators: false) {
                HStack(spacing: 8) {
                    ForEach(RecordingFilter.allCases, id: \.self) { filter in
                        FilterChip(
                            title: filter.rawValue,
                            icon: filter.icon,
                            isSelected: selectedFilter == filter
                        ) {
                            withAnimation {
                                selectedFilter = filter
                            }
                        }
                    }
                    
                    Divider()
                        .frame(height: 24)
                    
                    // Tag filters
                    ForEach(annotationsManager.allTags) { tag in
                        TagFilterChip(
                            tag: tag,
                            isSelected: selectedTagFilter?.id == tag.id
                        ) {
                            withAnimation {
                                if selectedTagFilter?.id == tag.id {
                                    selectedTagFilter = nil
                                } else {
                                    selectedTagFilter = tag
                                }
                            }
                        }
                    }
                }
                .padding(.horizontal)
            }
            
            // Source indicator and filter status
            HStack {
                // Source indicator
                Label(recordingsManager.currentSource.displayName, systemImage: recordingsManager.currentSource.icon)
                    .font(.caption)
                    .foregroundColor(recordingsManager.currentSource.color)
                
                if hasActiveFilters {
                    Text("•")
                        .foregroundColor(.secondary)
                    Text("\(currentFilteredCount) of \(recordingsManager.currentRecordingsCount)")
                        .font(.caption)
                        .foregroundColor(.secondary)
                    
                    Spacer()
                    
                    Button("Clear Filters") {
                        withAnimation {
                            selectedFilter = .all
                            selectedTagFilter = nil
                        }
                    }
                    .font(.caption)
                } else {
                    Text("•")
                        .foregroundColor(.secondary)
                    Text("\(recordingsManager.currentRecordingsCount) recordings")
                        .font(.caption)
                        .foregroundColor(.secondary)
                    
                    Spacer()
                }
            }
            .padding(.horizontal)
        }
        .padding(.vertical, 8)
        .background(Color(.secondarySystemBackground))
    }
    
    // MARK: - Cloud Provider Views
    
    private func cloudNotSignedInView(provider: CloudStorageProvider) -> some View {
        VStack(spacing: 16) {
            Image(systemName: provider.icon)
                .font(.system(size: 60))
                .foregroundColor(provider.color)
            
            Text("Not Signed In")
                .font(.title2)
                .fontWeight(.semibold)
            
            Text("Sign in to \(provider.displayName) to view your recordings")
                .font(.subheadline)
                .foregroundColor(.secondary)
            
            NavigationLink(destination: CloudStorageSettingsView()) {
                Label("Sign In", systemImage: "person.fill")
                    .padding(.horizontal, 20)
                    .padding(.vertical, 10)
            }
            .buttonStyle(.borderedProminent)
            .tint(provider.color)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
    
    private func cloudEmptyView(provider: CloudStorageProvider) -> some View {
        VStack(spacing: 16) {
            Image(systemName: provider.icon)
                .font(.system(size: 60))
                .foregroundColor(.gray)
            
            Text("No Recordings")
                .font(.title2)
                .fontWeight(.semibold)
            
            let storagePath = provider == .dropbox ? "Apps/VisionProTeleop" : "VisionProTeleop"
            Text("Recordings saved to \(provider.displayName) will appear here.\nRecordings are stored in \(storagePath)")
                .font(.subheadline)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
                .padding(.horizontal, 40)
            
            Button(action: {
                Task {
                    await recordingsManager.loadRecordings()
                }
            }) {
                Label("Refresh", systemImage: "arrow.clockwise")
                    .padding(.horizontal, 20)
                    .padding(.vertical, 10)
            }
            .buttonStyle(.bordered)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
    
    private var dropboxRecordingsView: some View {
        let filteredDropbox = filteredUnifiedRecordings(recordingsManager.dropboxRecordings.map { AnyRecording($0) })
        
        return ScrollView {
            if filteredDropbox.isEmpty && !searchText.isEmpty {
                noCloudResultsView
            } else {
                LazyVGrid(columns: columns, spacing: 20) {
                    ForEach(filteredDropbox) { recording in
                        if isSelectionMode {
                            Button(action: {
                                recordingsManager.toggleUnifiedSelection(recording.id)
                            }) {
                                UnifiedRecordingThumbnailView(
                                    recording: recording,
                                    isSelected: recordingsManager.isUnifiedSelected(recording.id),
                                    isSelectionMode: true
                                )
                            }
                            .buttonStyle(.plain)
                            .contextMenu {
                                unifiedRecordingContextMenu(for: recording)
                            }
                            .trackVisibility(id: recording.id, in: .named("scroll"))
                            .onAppear {
                                // Load more when reaching the last item
                                if recording.id == filteredDropbox.last?.id {
                                    Task {
                                        await recordingsManager.loadMoreDropboxRecordings()
                                    }
                                }
                            }
                        } else {
                            NavigationLink(destination: UnifiedRecordingDetailView(recording: recording)) {
                                UnifiedRecordingThumbnailView(
                                    recording: recording,
                                    isSelectionMode: false
                                )
                            }
                            .buttonStyle(.plain)
                            .contextMenu {
                                unifiedRecordingContextMenu(for: recording)
                            }
                            .trackVisibility(id: recording.id, in: .named("scroll"))
                            .onAppear {
                                // Load more when reaching the last item
                                if recording.id == filteredDropbox.last?.id {
                                    Task {
                                        await recordingsManager.loadMoreDropboxRecordings()
                                    }
                                }
                            }
                        }
                    }
                    
                    // Loading indicator at bottom
                    if recordingsManager.isLoadingMore && recordingsManager.currentSource == .dropbox {
                        HStack {
                            Spacer()
                            ProgressView()
                                .padding()
                            Text("Loading more...")
                                .font(.caption)
                                .foregroundColor(.secondary)
                            Spacer()
                        }
                        .padding(.vertical, 8)
                    } else if recordingsManager.hasMoreRecordings && recordingsManager.currentSource == .dropbox && !filteredDropbox.isEmpty {
                        HStack {
                            Spacer()
                            Text("Scroll for more")
                                .font(.caption2)
                                .foregroundColor(Color.gray.opacity(0.6))
                            Spacer()
                        }
                        .padding(.vertical, 4)
                    }
                }
                .padding()
            }
        }
        .coordinateSpace(name: "scroll")
        .onPreferenceChange(VisibilityPreferenceKey.self) { visibility in
            FeedPlayerManager.shared.updateVisibleItems(visibility)
        }
    }
    
    private var googleDriveRecordingsView: some View {
        let filteredGDrive = filteredUnifiedRecordings(recordingsManager.googleDriveRecordings.map { AnyRecording($0) })
        
        return ScrollView {
            if filteredGDrive.isEmpty && !searchText.isEmpty {
                noCloudResultsView
            } else {
                LazyVGrid(columns: columns, spacing: 20) {
                    ForEach(filteredGDrive) { recording in
                        if isSelectionMode {
                            Button(action: {
                                recordingsManager.toggleUnifiedSelection(recording.id)
                            }) {
                                UnifiedRecordingThumbnailView(
                                    recording: recording,
                                    isSelected: recordingsManager.isUnifiedSelected(recording.id),
                                    isSelectionMode: true
                                )
                            }
                            .buttonStyle(.plain)
                            .contextMenu {
                                unifiedRecordingContextMenu(for: recording)
                            }
                            .trackVisibility(id: recording.id, in: .named("scroll"))
                            .onAppear {
                                // Load more when reaching the last item
                                if recording.id == filteredGDrive.last?.id {
                                    Task {
                                        await recordingsManager.loadMoreGoogleDriveRecordings()
                                    }
                                }
                            }
                        } else {
                            NavigationLink(destination: UnifiedRecordingDetailView(recording: recording)) {
                                UnifiedRecordingThumbnailView(
                                    recording: recording,
                                    isSelectionMode: false
                                )
                            }
                            .buttonStyle(.plain)
                            .contextMenu {
                                unifiedRecordingContextMenu(for: recording)
                            }
                            .trackVisibility(id: recording.id, in: .named("scroll"))
                            .onAppear {
                                // Load more when reaching the last item
                                if recording.id == filteredGDrive.last?.id {
                                    Task {
                                        await recordingsManager.loadMoreGoogleDriveRecordings()
                                    }
                                }
                            }
                        }
                    }
                    
                    // Loading indicator at bottom
                    if recordingsManager.isLoadingMore && recordingsManager.currentSource == .googleDrive {
                        HStack {
                            Spacer()
                            ProgressView()
                                .padding()
                            Text("Loading more...")
                                .font(.caption)
                                .foregroundColor(.secondary)
                            Spacer()
                        }
                        .padding(.vertical, 8)
                    } else if recordingsManager.hasMoreRecordings && recordingsManager.currentSource == .googleDrive && !filteredGDrive.isEmpty {
                        HStack {
                            Spacer()
                            Text("Scroll for more")
                                .font(.caption2)
                                .foregroundColor(Color.gray.opacity(0.6))
                            Spacer()
                        }
                        .padding(.vertical, 4)
                    }
                }
                .padding()
            }
        }
        .coordinateSpace(name: "scroll")
        .onPreferenceChange(VisibilityPreferenceKey.self) { visibility in
            FeedPlayerManager.shared.updateVisibleItems(visibility)
        }
    }
    
    // MARK: - Unified Filtering
    
    private func filteredUnifiedRecordings(_ recordings: [AnyRecording]) -> [AnyRecording] {
        var result = recordings
        
        // Apply quick filter
        switch selectedFilter {
        case .all:
            break
        case .favorites:
            result = result.filter { annotationsManager.isFavorite($0.id) }
        case .hasNotes:
            result = result.filter { !(annotationsManager.notes(for: $0.id) ?? "").isEmpty }
        case .today:
            let today = Calendar.current.startOfDay(for: Date())
            result = result.filter { ($0.modifiedDate ?? Date.distantPast) >= today }
        case .thisWeek:
            let weekAgo = Calendar.current.date(byAdding: .day, value: -7, to: Date()) ?? Date()
            result = result.filter { ($0.modifiedDate ?? Date.distantPast) >= weekAgo }
        case .thisMonth:
            let monthAgo = Calendar.current.date(byAdding: .month, value: -1, to: Date()) ?? Date()
            result = result.filter { ($0.modifiedDate ?? Date.distantPast) >= monthAgo }
        }
        
        // Apply tag filter
        if let tag = selectedTagFilter {
            result = result.filter { annotationsManager.hasTag($0.id, tag: tag) }
        }
        
        // Apply search text
        if !searchText.isEmpty {
            result = result.filter { recording in
                // Search in display name (including custom name)
                let customName = annotationsManager.annotation(for: recording.id)?.customName
                let displayName = customName ?? recording.displayName
                if displayName.localizedCaseInsensitiveContains(searchText) {
                    return true
                }
                
                // Search in original name/ID
                if recording.name.localizedCaseInsensitiveContains(searchText) ||
                   recording.id.localizedCaseInsensitiveContains(searchText) {
                    return true
                }
                
                // Search in notes
                if let notes = annotationsManager.notes(for: recording.id),
                   notes.localizedCaseInsensitiveContains(searchText) {
                    return true
                }
                
                // Search in tags
                let tags = annotationsManager.tags(for: recording.id)
                if tags.contains(where: { $0.name.localizedCaseInsensitiveContains(searchText) }) {
                    return true
                }
                
                return false
            }
        }
        
        return result
    }
    
    private var noCloudResultsView: some View {
        VStack(spacing: 16) {
            Image(systemName: "magnifyingglass")
                .font(.system(size: 50))
                .foregroundColor(.gray)
            
            Text("No Results")
                .font(.title2)
                .fontWeight(.semibold)
            
            if !searchText.isEmpty {
                Text("No recordings match \"\(searchText)\"")
                    .font(.subheadline)
                    .foregroundColor(.secondary)
            } else {
                Text("No recordings match the selected filters")
                    .font(.subheadline)
                    .foregroundColor(.secondary)
            }
            
            Button("Clear Filters") {
                withAnimation {
                    searchText = ""
                    selectedFilter = .all
                    selectedTagFilter = nil
                }
            }
            .buttonStyle(.bordered)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .padding()
    }
    
    @ViewBuilder
    private func unifiedRecordingContextMenu(for recording: AnyRecording) -> some View {
        // Favorite toggle
        Button(action: {
            annotationsManager.toggleFavorite(recording.id)
        }) {
            Label(
                annotationsManager.isFavorite(recording.id) ? "Remove from Favorites" : "Add to Favorites",
                systemImage: annotationsManager.isFavorite(recording.id) ? "star.slash" : "star"
            )
        }
        
        // Tags
        Button(action: {
            showTagPickerForUnifiedRecording = recording
        }) {
            Label("Edit Tags", systemImage: "tag")
        }
        
        // Notes
        Button(action: {
            showNotesSheetForUnifiedRecording = recording
        }) {
            Label("Edit Notes", systemImage: "note.text")
        }
        
        Divider()
        
        // Add to playlist
        Button(action: {
            unifiedRecordingForPlaylist = recording
        }) {
            Label("Add to Playlist", systemImage: "folder.badge.plus")
        }
        
        // Show playlists containing this recording
        let containingPlaylists = playlistManager.playlists.filter { $0.recordingIDs.contains(recording.id) }
        if !containingPlaylists.isEmpty {
            Menu {
                ForEach(containingPlaylists) { playlist in
                    Text(playlist.name)
                }
            } label: {
                Label("In \(containingPlaylists.count) Playlist\(containingPlaylists.count == 1 ? "" : "s")", systemImage: "folder.fill")
            }
        }
    }
    
    // MARK: - Empty State
    
    private var emptyStateView: some View {
        VStack(spacing: 16) {
            Image(systemName: "video.badge.plus")
                .font(.system(size: 60))
                .foregroundColor(.gray)
            
            Text("No Recordings")
                .font(.title2)
                .fontWeight(.semibold)
            
            Text("Recordings from Vision Pro will appear here.\nCurrently viewing iCloud Drive recordings.")
                .font(.subheadline)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
                .padding(.horizontal, 40)
            
            // Show current cloud storage setting if different from iCloud
            let selectedProvider = CloudStorageManager.shared.selectedProvider
            if selectedProvider != .iCloudDrive {
                HStack(spacing: 8) {
                    Image(systemName: "info.circle.fill")
                        .foregroundColor(selectedProvider.color)
                    Text("Vision Pro is saving to \(selectedProvider.displayName).\nChange storage provider in Settings to sync.")
                        .font(.caption)
                        .foregroundColor(.secondary)
                        .multilineTextAlignment(.center)
                }
                .padding()
                .background(selectedProvider.color.opacity(0.1))
                .cornerRadius(8)
                .padding(.horizontal, 20)
            }
            
            Button(action: {
                Task { await recordingsManager.loadRecordings() }
            }) {
                Label("Refresh", systemImage: "arrow.clockwise")
                    .padding(.horizontal, 20)
                    .padding(.vertical, 10)
            }
            .buttonStyle(.bordered)
            .padding(.top)
            
            if let error = recordingsManager.errorMessage {
                Text(error)
                    .font(.caption)
                    .foregroundColor(.red)
                    .padding(.top)
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
    
    // MARK: - No Results View
    
    private var noResultsView: some View {
        VStack(spacing: 16) {
            Image(systemName: "magnifyingglass")
                .font(.system(size: 50))
                .foregroundColor(.gray)
            
            Text("No Results")
                .font(.title2)
                .fontWeight(.semibold)
            
            if !searchText.isEmpty {
                Text("No recordings match \"\(searchText)\"")
                    .font(.subheadline)
                    .foregroundColor(.secondary)
            } else {
                Text("No recordings match the selected filters")
                    .font(.subheadline)
                    .foregroundColor(.secondary)
            }
            
            Button("Clear Filters") {
                withAnimation {
                    searchText = ""
                    selectedFilter = .all
                    selectedTagFilter = nil
                }
            }
            .buttonStyle(.bordered)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
    
    // MARK: - Recordings Grid
    
    private var recordingsGridView: some View {
        ScrollView {
            LazyVGrid(columns: columns, spacing: 20) {
                ForEach(filteredRecordings) { recording in
                    if isSelectionMode {
                        Button(action: {
                            recordingsManager.toggleSelection(recording)
                        }) {
                            UnifiedRecordingThumbnailView(
                                recording: AnyRecording(recording),
                                isSelected: recordingsManager.selectedRecordings.contains(recording),
                                isSelectionMode: true
                            )
                        }
                        .buttonStyle(.plain)
                        .contextMenu {
                            recordingContextMenu(for: recording)
                        }
                        .trackVisibility(id: recording.id, in: .named("scroll"))
                        .onAppear {
                            // Lazy load metadata for iCloud recordings when they become visible
                            recordingsManager.loadMetadataIfNeeded(for: recording)
                        }
                    } else {
                        NavigationLink(destination: UnifiedRecordingDetailView(recording: AnyRecording(recording))) {
                            UnifiedRecordingThumbnailView(
                                recording: AnyRecording(recording),
                                isSelectionMode: false
                            )
                        }
                        .buttonStyle(.plain)
                        .contextMenu {
                            recordingContextMenu(for: recording)
                        }
                        .trackVisibility(id: recording.id, in: .named("scroll"))
                        .onAppear {
                            // Lazy load metadata for iCloud recordings when they become visible
                            recordingsManager.loadMetadataIfNeeded(for: recording)
                        }
                    }
                }
            }
            .padding()
        }
        .coordinateSpace(name: "scroll")
        .onPreferenceChange(VisibilityPreferenceKey.self) { visibility in
            FeedPlayerManager.shared.updateVisibleItems(visibility)
        }
        .refreshable {
            await recordingsManager.loadRecordings()
        }
    }
    
    // MARK: - Context Menu
    
    @ViewBuilder
    private func recordingContextMenu(for recording: Recording) -> some View {
        // Favorite toggle
        Button(action: {
            annotationsManager.toggleFavorite(recording)
        }) {
            Label(
                annotationsManager.isFavorite(recording) ? "Remove from Favorites" : "Add to Favorites",
                systemImage: annotationsManager.isFavorite(recording) ? "star.slash" : "star"
            )
        }
        
        // Rename
        Button(action: {
            showRenameSheet = recording
        }) {
            Label("Rename", systemImage: "pencil")
        }
        
        // Tags
        Button(action: {
            showTagPickerForRecording = recording
        }) {
            Label("Edit Tags", systemImage: "tag")
        }
        
        // Notes
        Button(action: {
            showNotesSheet = recording
        }) {
            Label("Edit Notes", systemImage: "note.text")
        }
        
        Divider()
        
        // Add to playlist
        Button(action: {
            recordingForPlaylist = recording
        }) {
            Label("Add to Playlist", systemImage: "folder.badge.plus")
        }
        
        // Show playlists containing this recording
        let containingPlaylists = playlistManager.playlists(containing: recording)
        if !containingPlaylists.isEmpty {
            Menu {
                ForEach(containingPlaylists) { playlist in
                    Text(playlist.name)
                }
            } label: {
                Label("In \(containingPlaylists.count) Playlist\(containingPlaylists.count == 1 ? "" : "s")", systemImage: "folder.fill")
            }
        }
        
        Divider()
        
        Button(action: {
            exportURLs = [recording.folderURL]
            showExportSheet = true
        }) {
            Label("Export", systemImage: "square.and.arrow.up")
        }
        
        if httpServer.isRunning, let url = httpServer.getDownloadURL(for: recording) {
            Button(action: {
                UIPasteboard.general.string = url
            }) {
                Label("Copy Download URL", systemImage: "link")
            }
            
            Button(action: {
                UIPasteboard.general.string = "curl -o \(recording.id).zip \"\(url)\""
            }) {
                Label("Copy curl Command", systemImage: "terminal")
            }
        }
        
        Divider()
        
        Button(role: .destructive, action: {
            recordingToDelete = recording
            showDeleteConfirmation = true
        }) {
            Label("Delete", systemImage: "trash")
        }
    }
    
    // MARK: - Selection Toolbar
    
    private var selectionToolbar: some View {
        HStack(spacing: 12) {
            Button(action: {
                if recordingsManager.allSelected {
                    recordingsManager.deselectAll()
                } else {
                    recordingsManager.selectAllUnified()
                }
            }) {
                Text(recordingsManager.allSelected ? "Deselect All" : "Select All")
                    .font(.subheadline)
            }
            
            Spacer()
            
            Text("\(recordingsManager.selectedCount) selected")
                .foregroundColor(.secondary)
                .font(.subheadline)
            
            Spacer()
            
            // Add to playlist button
            Button(action: {
                if recordingsManager.currentSource == .iCloudDrive {
                    showAddSelectedToPlaylistSheet = true
                } else {
                    showAddSelectedUnifiedToPlaylistSheet = true
                }
            }) {
                Image(systemName: "folder.badge.plus")
                    .font(.title3)
            }
            .disabled(recordingsManager.selectedCount == 0 || isProcessingBulkAction)
            
            // Public/Private Buttons (Only for Cloud Sources)
            if recordingsManager.currentSource != .iCloudDrive {
                // Make Public
                Button(action: {
                    Task {
                        await makeSelectedPublicWithProgress()
                    }
                }) {
                    Image(systemName: "globe")
                        .font(.title3)
                        .foregroundColor(.green)
                }
                .disabled(recordingsManager.selectedCount == 0 || isProcessingBulkAction)
                
                // Make Private
                Button(action: {
                    Task {
                        await makeSelectedPrivateWithProgress()
                    }
                }) {
                    Image(systemName: "lock.fill")
                        .font(.title3)
                        .foregroundColor(.gray)
                }
                .disabled(recordingsManager.selectedCount == 0 || isProcessingBulkAction)
            }
            
            // Delete button for all sources
            Button(action: {
                if recordingsManager.currentSource == .iCloudDrive {
                    showDeleteSelectedConfirmation = true
                } else {
                    showDeleteSelectedUnifiedConfirmation = true
                }
            }) {
                Image(systemName: "trash")
                    .font(.title3)
                    .foregroundColor(.red)
            }
            .disabled(recordingsManager.selectedCount == 0 || isProcessingBulkAction)
            
            // Export button (only for iCloud)
            if recordingsManager.currentSource == .iCloudDrive {
                Button(action: {
                    Task {
                        let selected = Array(recordingsManager.selectedRecordings)
                        exportURLs = await recordingsManager.exportRecordings(selected)
                        showExportSheet = true
                    }
                }) {
                    Image(systemName: "square.and.arrow.up")
                        .font(.title3)
                }
                .disabled(recordingsManager.selectedRecordings.isEmpty)
            }
        }
        .padding()
        .background(.regularMaterial)
    }
    
    // MARK: - Bulk Action Helper Functions
    
    /// Delete selected iCloud recordings with progress tracking
    private func deleteSelectedRecordingsWithProgress() async {
        let recordings = Array(recordingsManager.selectedRecordings)
        guard !recordings.isEmpty else { return }
        
        bulkActionType = .deleting
        bulkActionTotal = recordings.count
        bulkActionCurrent = 0
        bulkActionProgress = 0
        isProcessingBulkAction = true
        
        for (index, recording) in recordings.enumerated() {
            bulkActionCurrent = index + 1
            bulkActionProgress = Double(index + 1) / Double(recordings.count)
            
            playlistManager.removeRecordingFromAllPlaylists(recording)
            annotationsManager.removeAnnotation(for: recording)
            try? await recordingsManager.deleteRecording(recording)
        }
        
        // Reset state
        bulkActionType = nil
        isProcessingBulkAction = false
        isSelectionMode = false
    }
    
    /// Delete selected unified (cloud) recordings with progress tracking
    private func deleteSelectedUnifiedWithProgress() async {
        let recordingIDs = Array(recordingsManager.selectedUnifiedRecordingIDs)
        guard !recordingIDs.isEmpty else { return }
        
        bulkActionType = .deleting
        bulkActionTotal = recordingIDs.count
        bulkActionCurrent = 0
        bulkActionProgress = 0
        isProcessingBulkAction = true
        
        for (index, id) in recordingIDs.enumerated() {
            bulkActionCurrent = index + 1
            bulkActionProgress = Double(index + 1) / Double(recordingIDs.count)
            
            switch recordingsManager.currentSource {
            case .dropbox:
                if let rec = recordingsManager.dropboxRecordings.first(where: { $0.id == id }) {
                    do {
                        try await DropboxManager.shared.deleteFolder(path: rec.path)
                        await MainActor.run {
                            recordingsManager.dropboxRecordings.removeAll { $0.id == id }
                        }
                    } catch {
                        dlog("❌ Failed to delete Dropbox recording: \(error)")
                    }
                }
            case .googleDrive:
                if let rec = recordingsManager.googleDriveRecordings.first(where: { $0.id == id }) {
                    do {
                        try await CloudStorageManager.shared.deleteGoogleDriveFolder(folderId: rec.id)
                        await MainActor.run {
                            recordingsManager.googleDriveRecordings.removeAll { $0.id == id }
                        }
                    } catch {
                        dlog("❌ Failed to delete Google Drive recording: \(error)")
                    }
                }
            default:
                break
            }
        }
        
        recordingsManager.selectedUnifiedRecordingIDs.removeAll()
        
        // Reset state
        bulkActionType = nil
        isProcessingBulkAction = false
        isSelectionMode = false
    }
    
    /// Make selected recordings public with progress tracking
    private func makeSelectedPublicWithProgress() async {
        let selected = recordingsManager.selectedUnifiedRecordings
        guard !selected.isEmpty else { return }
        
        bulkActionType = .makingPublic
        bulkActionTotal = selected.count
        bulkActionCurrent = 0
        bulkActionProgress = 0
        isProcessingBulkAction = true
        
        for (index, recording) in selected.enumerated() {
            bulkActionCurrent = index + 1
            bulkActionProgress = Double(index + 1) / Double(selected.count)
            
            // Generate shared link
            var sharedURL: String?
            var provider: CloudStorageProvider?
            
            if recording.source == .dropbox {
                if let rec = recording.dropboxRecording {
                    sharedURL = await DropboxManager.shared.createSharedLink(path: rec.path)
                    provider = .dropbox
                }
            } else if recording.source == .googleDrive {
                if let rec = recording.googleDriveRecording {
                    sharedURL = await GoogleDriveManager.shared.createSharedLink(fileId: rec.id)
                    provider = .googleDrive
                }
            }
            
            if let url = sharedURL, let prov = provider {
                // Generate thumbnail asset
                var thumbnailAssetURL: URL?
                var image: UIImage?
                
                switch recording.source {
                case .dropbox:
                    if let rec = recording.dropboxRecording {
                        image = await recordingsManager.getDropboxThumbnail(for: rec)
                    }
                case .googleDrive:
                    if let rec = recording.googleDriveRecording {
                        image = await recordingsManager.getGoogleDriveThumbnail(for: rec)
                    }
                default: break
                }
                
                if let thumbnailImage = image,
                   let data = thumbnailImage.jpegData(compressionQuality: 0.7) {
                    let tempDir = FileManager.default.temporaryDirectory
                    let fileName = "thumb_\(UUID().uuidString).jpg"
                    let fileURL = tempDir.appendingPathComponent(fileName)
                    try? data.write(to: fileURL)
                    thumbnailAssetURL = fileURL
                }
                
                let customName = annotationsManager.annotation(for: recording.id)?.customName
                let notes = annotationsManager.notes(for: recording.id)
                
                _ = await CloudKitManager.shared.makeRecordingPublic(
                    recordingId: recording.id,
                    title: customName ?? recording.displayName,
                    description: notes,
                    cloudURL: url,
                    thumbnailURL: nil,
                    thumbnailAssetURL: thumbnailAssetURL,
                    provider: prov
                )
                
                // Clean up
                if let assetURL = thumbnailAssetURL {
                    try? FileManager.default.removeItem(at: assetURL)
                }
            }
        }
        
        // Reset state
        bulkActionType = nil
        isProcessingBulkAction = false
        withAnimation {
            isSelectionMode = false
            recordingsManager.deselectAll()
        }
    }
    
    /// Make selected recordings private with progress tracking
    private func makeSelectedPrivateWithProgress() async {
        let selected = recordingsManager.selectedUnifiedRecordings
        guard !selected.isEmpty else { return }
        
        bulkActionType = .makingPrivate
        bulkActionTotal = selected.count
        bulkActionCurrent = 0
        bulkActionProgress = 0
        isProcessingBulkAction = true
        
        for (index, recording) in selected.enumerated() {
            bulkActionCurrent = index + 1
            bulkActionProgress = Double(index + 1) / Double(selected.count)
            
            _ = await CloudKitManager.shared.makeRecordingPrivate(recordingId: recording.id)
        }
        
        // Reset state
        bulkActionType = nil
        isProcessingBulkAction = false
        withAnimation {
            isSelectionMode = false
            recordingsManager.deselectAll()
        }
    }
}

// MARK: - Bulk Action Progress Overlay

struct BulkActionProgressOverlay: View {
    let actionType: BulkActionType
    let current: Int
    let total: Int
    let progress: Double
    
    var body: some View {
        ZStack {
            // Dimmed background
            Color.black.opacity(0.4)
                .ignoresSafeArea()
            
            // Progress card
            VStack(spacing: 20) {
                // Icon
                Image(systemName: actionType.icon)
                    .font(.system(size: 44))
                    .foregroundColor(actionType.color)
                
                // Title
                Text(actionType.title)
                    .font(.headline)
                
                // Progress text
                Text("\(current) of \(total)")
                    .font(.subheadline)
                    .foregroundColor(.secondary)
                
                // Progress bar
                ProgressView(value: progress)
                    .progressViewStyle(.linear)
                    .tint(actionType.color)
                    .frame(width: 200)
                
                // Percentage
                Text("\(Int(progress * 100))%")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
            .padding(32)
            .background(.regularMaterial)
            .cornerRadius(20)
            .shadow(radius: 20)
        }
    }
}

// MARK: - Filter Chips

struct FilterChip: View {
    let title: String
    let icon: String
    let isSelected: Bool
    let action: () -> Void
    
    var body: some View {
        Button(action: action) {
            HStack(spacing: 4) {
                Image(systemName: icon)
                    .font(.caption)
                Text(title)
                    .font(.caption)
            }
            .padding(.horizontal, 12)
            .padding(.vertical, 6)
            .background(isSelected ? Color.blue : Color.gray.opacity(0.2))
            .foregroundColor(isSelected ? .white : .primary)
            .cornerRadius(16)
        }
    }
}

struct TagFilterChip: View {
    let tag: Tag
    let isSelected: Bool
    let action: () -> Void
    
    var body: some View {
        Button(action: action) {
            HStack(spacing: 4) {
                Circle()
                    .fill(tag.color.color)
                    .frame(width: 8, height: 8)
                Text(tag.name)
                    .font(.caption)
            }
            .padding(.horizontal, 12)
            .padding(.vertical, 6)
            .background(isSelected ? tag.color.color : tag.color.color.opacity(0.2))
            .foregroundColor(isSelected ? .white : .primary)
            .cornerRadius(16)
        }
    }
}

// MARK: - Add Multiple Recordings To Playlist Sheet

struct AddMultipleRecordingsToPlaylistSheet: View {
    let recordings: [Recording]
    
    @StateObject private var playlistManager = PlaylistManager.shared
    @Environment(\.dismiss) private var dismiss
    
    @State private var showCreateSheet = false
    
    var body: some View {
        NavigationStack {
            List {
                Section {
                    HStack {
                        Image(systemName: "video.fill")
                            .font(.title2)
                            .foregroundColor(.blue)
                        
                        VStack(alignment: .leading) {
                            Text("\(recordings.count) Recording\(recordings.count == 1 ? "" : "s") Selected")
                                .font(.headline)
                            Text("Choose a playlist to add them to")
                                .font(.caption)
                                .foregroundColor(.secondary)
                        }
                    }
                }
                
                Section("Playlists") {
                    if playlistManager.playlists.isEmpty {
                        VStack(spacing: 12) {
                            Text("No playlists yet")
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                            
                            Button {
                                showCreateSheet = true
                            } label: {
                                Label("Create Playlist", systemImage: "plus")
                            }
                            .buttonStyle(.bordered)
                        }
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 12)
                    } else {
                        ForEach(playlistManager.playlists) { playlist in
                            Button {
                                playlistManager.addRecordings(recordings, to: playlist)
                                dismiss()
                            } label: {
                                HStack(spacing: 12) {
                                    ZStack {
                                        RoundedRectangle(cornerRadius: 6)
                                            .fill(playlist.color.color.opacity(0.2))
                                            .frame(width: 40, height: 40)
                                        
                                        Image(systemName: playlist.icon.systemName)
                                            .foregroundColor(playlist.color.color)
                                    }
                                    
                                    VStack(alignment: .leading) {
                                        Text(playlist.name)
                                            .font(.headline)
                                            .foregroundColor(.primary)
                                        
                                        Text("\(playlist.recordingCount) recordings")
                                            .font(.caption)
                                            .foregroundColor(.secondary)
                                    }
                                    
                                    Spacer()
                                    
                                    Image(systemName: "plus.circle")
                                        .font(.title2)
                                        .foregroundColor(.blue)
                                }
                            }
                        }
                    }
                }
            }
            .navigationTitle("Add to Playlist")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Cancel") {
                        dismiss()
                    }
                }
                
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        showCreateSheet = true
                    } label: {
                        Image(systemName: "plus")
                    }
                }
            }
            .sheet(isPresented: $showCreateSheet) {
                PlaylistEditSheet(mode: .create)
            }
        }
    }
}

#Preview {
    ContentView()
}
