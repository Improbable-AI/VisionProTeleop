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
    @State private var showCopiedToast = false
    @State private var copiedText = ""
    
    var body: some View {
        VStack(spacing: 16) {
            // Server Status Header
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
                
                Divider()
                
                // Download All URL
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
                
                // Individual recording URLs
                if !recordingsManager.recordings.isEmpty {
                    VStack(alignment: .leading, spacing: 8) {
                        Text("Individual Recordings")
                            .font(.subheadline)
                            .fontWeight(.semibold)
                        
                        ScrollView {
                            LazyVStack(spacing: 8) {
                                ForEach(recordingsManager.recordings) { recording in
                                    RecordingURLRow(recording: recording, server: server, onCopy: copyToClipboard)
                                }
                            }
                        }
                        .frame(maxHeight: 200)
                    }
                    .padding()
                    .background(Color(.secondarySystemBackground))
                    .cornerRadius(12)
                }
                
                // Help text
                Text("Open the URL in a browser or use curl to download")
                    .font(.caption2)
                    .foregroundColor(.secondary)
                    .multilineTextAlignment(.center)
            } else {
                // Instructions when server is off
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
            
            Spacer()
        }
        .padding()
        .navigationTitle("Download Server")
        .navigationBarTitleDisplayMode(.inline)
        .overlay(alignment: .bottom) {
            if showCopiedToast {
                ToastView(message: "Copied to clipboard")
                    .transition(.move(edge: .bottom).combined(with: .opacity))
            }
        }
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
