//
//  CloudStorageSettingsView.swift
//  Tracking Viewer
//
//  UI for configuring cloud storage provider.
//  Users can switch between iCloud Drive and Dropbox here,
//  and settings will sync to Vision Pro via iCloud Keychain.
//

import SwiftUI

struct CloudStorageSettingsView: View {
    @StateObject private var storageManager = CloudStorageManager.shared
    @StateObject private var dropboxManager = DropboxManager.shared
    @StateObject private var googleDriveManager = GoogleDriveManager.shared
    @Environment(\.dismiss) private var dismiss
    
    var body: some View {
        List {
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
            
            // Current storage info
            Section("Current Settings") {
                VStack(alignment: .leading, spacing: 8) {
                    HStack {
                        Text("Active Provider")
                            .foregroundColor(.secondary)
                        Spacer()
                        HStack(spacing: 4) {
                            Image(systemName: storageManager.selectedProvider.icon)
                            Text(storageManager.selectedProvider.displayName)
                        }
                        .foregroundColor(storageManager.selectedProvider.color)
                    }
                    
                    HStack {
                        Text("Save Location")
                            .foregroundColor(.secondary)
                        Spacer()
                        Text(storageManager.getStoragePathDescription())
                            .font(.caption)
                    }
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
        }
        .navigationTitle("Storage Settings")
        .navigationBarTitleDisplayMode(.inline)
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
    
    // MARK: - Helper Methods
    
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
}

// MARK: - Storage Provider Row

struct StorageProviderRow: View {
    let provider: CloudStorageProvider
    let isSelected: Bool
    let isAvailable: Bool
    let action: () -> Void
    
    var body: some View {
        Button(action: action) {
            HStack(spacing: 12) {
                // Provider icon
                ZStack {
                    Circle()
                        .fill(provider.color.opacity(0.2))
                        .frame(width: 44, height: 44)
                    
                    Image(systemName: provider.icon)
                        .font(.title2)
                        .foregroundColor(provider.color)
                }
                
                // Provider info
                VStack(alignment: .leading, spacing: 2) {
                    Text(provider.displayName)
                        .font(.headline)
                        .foregroundColor(.primary)
                    
                    Text(provider.description)
                        .font(.caption)
                        .foregroundColor(.secondary)
                        .lineLimit(2)
                }
                
                Spacer()
                
                // Selection indicator or unavailable badge
                if isSelected {
                    Image(systemName: "checkmark.circle.fill")
                        .font(.title2)
                        .foregroundColor(.blue)
                } else if !isAvailable {
                    Text("Sign in required")
                        .font(.caption)
                        .foregroundColor(.orange)
                        .padding(.horizontal, 8)
                        .padding(.vertical, 4)
                        .background(Color.orange.opacity(0.1))
                        .cornerRadius(8)
                }
            }
            .padding(.vertical, 4)
        }
        .disabled(!isAvailable && !isSelected)
    }
}

// MARK: - Help View

struct CloudStorageHelpView: View {
    var body: some View {
        List {
            Section {
                VStack(alignment: .leading, spacing: 16) {
                    HelpItem(
                        icon: "iphone.and.arrow.right.inward",
                        title: "1. Configure on iPhone",
                        description: "Choose your preferred cloud storage and sign in if needed."
                    )
                    
                    HelpItem(
                        icon: "icloud.and.arrow.up",
                        title: "2. Automatic Sync",
                        description: "Your settings sync to Vision Pro via iCloud Keychain."
                    )
                    
                    HelpItem(
                        icon: "visionpro",
                        title: "3. Record on Vision Pro",
                        description: "Recordings automatically save to your chosen cloud storage."
                    )
                }
                .padding(.vertical)
            } header: {
                Text("How Cloud Sync Works")
            }
            
            Section("Requirements") {
                Label("Same Apple ID on both devices", systemImage: "person.crop.circle")
                Label("iCloud Keychain enabled", systemImage: "key.icloud")
                Label("Internet connection for sync", systemImage: "wifi")
            }
            
            Section("Supported Providers") {
                VStack(alignment: .leading, spacing: 12) {
                    ProviderInfoRow(
                        provider: .iCloudDrive,
                        details: "Built-in, no sign-in required. Files sync across all Apple devices."
                    )
                    
                    Divider()
                    
                    ProviderInfoRow(
                        provider: .dropbox,
                        details: "Requires Dropbox account. Files accessible on any device with Dropbox."
                    )
                    
                    Divider()
                    
                    ProviderInfoRow(
                        provider: .googleDrive,
                        details: "Requires Google account. Files accessible on any device with Google Drive."
                    )
                }
                .padding(.vertical, 4)
            }
            
            Section("Troubleshooting") {
                NavigationLink {
                    TroubleshootingView()
                } label: {
                    Label("Settings not syncing?", systemImage: "wrench.and.screwdriver")
                }
            }
        }
        .navigationTitle("How it works")
        .navigationBarTitleDisplayMode(.inline)
    }
}

struct HelpItem: View {
    let icon: String
    let title: String
    let description: String
    
    var body: some View {
        HStack(alignment: .top, spacing: 12) {
            Image(systemName: icon)
                .font(.title2)
                .foregroundColor(.blue)
                .frame(width: 32)
            
            VStack(alignment: .leading, spacing: 4) {
                Text(title)
                    .font(.headline)
                Text(description)
                    .font(.subheadline)
                    .foregroundColor(.secondary)
            }
        }
    }
}

struct ProviderInfoRow: View {
    let provider: CloudStorageProvider
    let details: String
    
    var body: some View {
        HStack(alignment: .top, spacing: 12) {
            Image(systemName: provider.icon)
                .font(.title2)
                .foregroundColor(provider.color)
                .frame(width: 32)
            
            VStack(alignment: .leading, spacing: 4) {
                Text(provider.displayName)
                    .font(.headline)
                Text(details)
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
        }
    }
}

struct TroubleshootingView: View {
    var body: some View {
        List {
            Section("Common Issues") {
                TroubleshootingItem(
                    problem: "Settings not appearing on Vision Pro",
                    solutions: [
                        "Ensure both devices use the same Apple ID",
                        "Enable iCloud Keychain in Settings → Apple ID → iCloud → Passwords and Keychain",
                        "Wait a few minutes for sync to complete",
                        "Try opening the app on Vision Pro after configuring iPhone"
                    ]
                )
                
                TroubleshootingItem(
                    problem: "Dropbox sign-in fails",
                    solutions: [
                        "Check your internet connection",
                        "Try signing out and back in",
                        "Ensure Dropbox app permissions are correct"
                    ]
                )
                
                TroubleshootingItem(
                    problem: "Google Drive sign-in fails",
                    solutions: [
                        "Check your internet connection",
                        "Try signing out and back in",
                        "Ensure Google account has Drive enabled",
                        "Check if third-party app access is enabled in Google account"
                    ]
                )
                
                TroubleshootingItem(
                    problem: "Recordings not uploading to Dropbox",
                    solutions: [
                        "Check Vision Pro has internet connection",
                        "Verify Dropbox storage isn't full",
                        "Re-authenticate Dropbox on iPhone"
                    ]
                )
                
                TroubleshootingItem(
                    problem: "Recordings not uploading to Google Drive",
                    solutions: [
                        "Check Vision Pro has internet connection",
                        "Verify Google Drive storage isn't full",
                        "Re-authenticate Google Drive on iPhone"
                    ]
                )
            }
        }
        .navigationTitle("Troubleshooting")
        .navigationBarTitleDisplayMode(.inline)
    }
}

struct TroubleshootingItem: View {
    let problem: String
    let solutions: [String]
    
    @State private var isExpanded = false
    
    var body: some View {
        DisclosureGroup(isExpanded: $isExpanded) {
            VStack(alignment: .leading, spacing: 8) {
                ForEach(solutions, id: \.self) { solution in
                    HStack(alignment: .top, spacing: 8) {
                        Image(systemName: "checkmark.circle")
                            .foregroundColor(.green)
                            .font(.caption)
                        Text(solution)
                            .font(.caption)
                    }
                }
            }
            .padding(.vertical, 4)
        } label: {
            Text(problem)
                .font(.subheadline)
        }
    }
}

#Preview {
    NavigationStack {
        CloudStorageSettingsView()
    }
}
