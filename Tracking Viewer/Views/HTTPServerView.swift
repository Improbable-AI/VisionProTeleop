//
//  HTTPServerView.swift
//  Tracking Viewer
//
//  UI controls for the HTTP file server
//

import SwiftUI

/// View showing HTTP server status and controls
struct HTTPServerView: View {
    @StateObject private var server = HTTPFileServer.shared
    @StateObject private var recordingsManager = RecordingsManager.shared
    @StateObject private var playlistManager = PlaylistManager.shared
    @State private var showCopiedToast = false
    @State private var copiedText = ""
    
    var body: some View {
        ScrollView {
            VStack(spacing: 16) {
                // Server Status Header
                serverStatusSection
                
                if server.isRunning {
                    // Active downloads indicator
                    if server.activeDownloads > 0 {
                        HStack {
                            ProgressView()
                                .scaleEffect(0.8)
                            Text("\(server.activeDownloads) active download(s)")
                                .font(.caption)
                                .foregroundColor(.secondary)
                            Spacer()
                        }
                        .padding(.horizontal)
                    }
                    
                    // ZIP creation progress
                    if server.isCreatingZip {
                        VStack(spacing: 4) {
                            ProgressView(value: server.exportProgress)
                            Text(server.statusMessage.isEmpty ? "Creating ZIP..." : server.statusMessage)
                                .font(.caption)
                                .foregroundColor(.secondary)
                                .lineLimit(1)
                                .truncationMode(.middle)
                        }
                        .padding(.horizontal)
                    }
                    
                    // Download All URL
                    downloadAllSection
                    
                    // Playlists section
                    if !playlistManager.playlists.isEmpty {
                        playlistsSection
                    }
                    
                    // Individual recording URLs
                    if !recordingsManager.recordings.isEmpty {
                        recordingsSection
                    }
                    
                    // Help text
                    Text("Open the URL in a browser or use curl to download")
                        .font(.caption2)
                        .foregroundColor(.secondary)
                        .multilineTextAlignment(.center)
                        .padding(.top, 8)
                } else {
                    // Instructions when server is off
                    serverOffInstructions
                }
            }
            .padding()
        }
        .navigationTitle("Download Server")
        .navigationBarTitleDisplayMode(.inline)
        .overlay(alignment: .bottom) {
            if showCopiedToast {
                ToastView(message: "Copied to clipboard")
                    .transition(.move(edge: .bottom).combined(with: .opacity))
            }
        }
    }
    
    // MARK: - Server Status Section
    
    private var serverStatusSection: some View {
        HStack {
            Image(systemName: server.isRunning ? "antenna.radiowaves.left.and.right" : "antenna.radiowaves.left.and.right.slash")
                .font(.title2)
                .foregroundColor(server.isRunning ? .green : .gray)
            
            VStack(alignment: .leading, spacing: 2) {
                Text(server.isRunning ? "Server Running" : "Server Stopped")
                    .font(.headline)
                
                if server.isRunning, let url = server.serverURL {
                    Text(url)
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            
            Spacer()
            
            Toggle("", isOn: Binding(
                get: { server.isRunning },
                set: { enabled in
                    if enabled {
                        server.startServer()
                    } else {
                        server.stopServer()
                    }
                }
            ))
            .labelsHidden()
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
    
    // MARK: - Download All Section
    
    private var downloadAllSection: some View {
        VStack(alignment: .leading, spacing: 8) {
            Text("Download All Recordings")
                .font(.subheadline)
                .fontWeight(.semibold)
            
            if let url = server.getDownloadAllURL() {
                HStack {
                    Text(url)
                        .font(.system(.caption, design: .monospaced))
                        .foregroundColor(.secondary)
                        .lineLimit(1)
                        .truncationMode(.middle)
                    
                    Spacer()
                    
                    Button(action: {
                        copyToClipboard(url)
                    }) {
                        Image(systemName: "doc.on.doc")
                            .font(.caption)
                    }
                }
                
                // curl command
                HStack {
                    Text("curl -o recordings.zip \"\(url)\"")
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundColor(.orange)
                        .lineLimit(1)
                        .truncationMode(.middle)
                    
                    Spacer()
                    
                    Button(action: {
                        copyToClipboard("curl -o recordings.zip \"\(url)\"")
                    }) {
                        Image(systemName: "doc.on.doc")
                            .font(.caption2)
                    }
                }
                .padding(8)
                .background(Color.black.opacity(0.3))
                .cornerRadius(6)
            }
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
    
    // MARK: - Playlists Section
    
    private var playlistsSection: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Image(systemName: "folder.fill")
                    .foregroundColor(.purple)
                Text("Playlists")
                    .font(.subheadline)
                    .fontWeight(.semibold)
            }
            
            ForEach(playlistManager.playlists) { playlist in
                PlaylistURLRow(playlist: playlist, server: server, onCopy: copyToClipboard)
            }
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
    
    // MARK: - Recordings Section
    
    private var recordingsSection: some View {
        VStack(alignment: .leading, spacing: 8) {
            Text("Individual Recordings")
                .font(.subheadline)
                .fontWeight(.semibold)
            
            LazyVStack(spacing: 8) {
                ForEach(recordingsManager.recordings) { recording in
                    RecordingURLRow(recording: recording, server: server, onCopy: copyToClipboard)
                }
            }
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
    
    // MARK: - Server Off Instructions
    
    private var serverOffInstructions: some View {
        VStack(spacing: 12) {
            Image(systemName: "info.circle")
                .font(.title)
                .foregroundColor(.blue)
            
            Text("Enable the server to download recordings via HTTP")
                .font(.subheadline)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
            
            Text("Use curl or wget from any device on the same network")
                .font(.caption)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
        }
        .padding()
    }
    
    private func copyToClipboard(_ text: String) {
        UIPasteboard.general.string = text
        copiedText = text
        
        withAnimation {
            showCopiedToast = true
        }
        
        DispatchQueue.main.asyncAfter(deadline: .now() + 2) {
            withAnimation {
                showCopiedToast = false
            }
        }
    }
}

// MARK: - Playlist URL Row

struct PlaylistURLRow: View {
    let playlist: Playlist
    let server: HTTPFileServer
    let onCopy: (String) -> Void
    
    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack {
                ZStack {
                    RoundedRectangle(cornerRadius: 4)
                        .fill(playlist.color.color.opacity(0.2))
                        .frame(width: 24, height: 24)
                    
                    Image(systemName: playlist.icon.systemName)
                        .font(.caption)
                        .foregroundColor(playlist.color.color)
                }
                
                VStack(alignment: .leading, spacing: 1) {
                    Text(playlist.name)
                        .font(.caption)
                        .fontWeight(.medium)
                    
                    Text("\(playlist.recordingCount) recording\(playlist.recordingCount == 1 ? "" : "s")")
                        .font(.caption2)
                        .foregroundColor(.secondary)
                }
                
                Spacer()
                
                if playlist.recordingCount > 0, let url = server.getPlaylistDownloadURL(for: playlist) {
                    Button(action: {
                        onCopy(url)
                    }) {
                        Image(systemName: "doc.on.doc")
                            .font(.caption)
                    }
                }
            }
            
            if playlist.recordingCount > 0, let url = server.getPlaylistDownloadURL(for: playlist) {
                // curl command
                HStack {
                    let sanitizedName = playlist.name.replacingOccurrences(of: " ", with: "_")
                    Text("curl -o \"\(sanitizedName).zip\" \"\(url)\"")
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundColor(.purple)
                        .lineLimit(1)
                        .truncationMode(.middle)
                    
                    Spacer()
                    
                    Button(action: {
                        let sanitizedName = playlist.name.replacingOccurrences(of: " ", with: "_")
                        onCopy("curl -o \"\(sanitizedName).zip\" \"\(url)\"")
                    }) {
                        Image(systemName: "doc.on.doc")
                            .font(.caption2)
                    }
                }
                .padding(6)
                .background(Color.purple.opacity(0.1))
                .cornerRadius(4)
            } else if playlist.recordingCount == 0 {
                Text("Empty playlist")
                    .font(.caption2)
                    .foregroundColor(.secondary)
                    .italic()
            }
        }
        .padding(.vertical, 4)
    }
}

struct RecordingURLRow: View {
    let recording: Recording
    let server: HTTPFileServer
    let onCopy: (String) -> Void
    
    var body: some View {
        HStack {
            VStack(alignment: .leading, spacing: 2) {
                Text(recording.displayName)
                    .font(.caption)
                    .fontWeight(.medium)
                
                if let url = server.getDownloadURL(for: recording) {
                    Text(url)
                        .font(.system(.caption2, design: .monospaced))
                        .foregroundColor(.secondary)
                        .lineLimit(1)
                        .truncationMode(.middle)
                }
            }
            
            Spacer()
            
            if let url = server.getDownloadURL(for: recording) {
                Button(action: {
                    onCopy(url)
                }) {
                    Image(systemName: "doc.on.doc")
                        .font(.caption)
                }
            }
        }
        .padding(.vertical, 4)
    }
}

struct ToastView: View {
    let message: String
    
    var body: some View {
        Text(message)
            .font(.subheadline)
            .fontWeight(.medium)
            .foregroundColor(.white)
            .padding(.horizontal, 20)
            .padding(.vertical, 12)
            .background(Color.black.opacity(0.8))
            .cornerRadius(25)
            .padding(.bottom, 30)
    }
}

#Preview {
    NavigationStack {
        HTTPServerView()
    }
}
