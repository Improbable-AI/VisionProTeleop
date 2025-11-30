//
//  RecordingDetailView.swift
//  Tracking Viewer
//
//  Created on 11/29/25.
//

import SwiftUI
import AVKit

/// Detailed view for a single recording with video playback
struct RecordingDetailView: View {
    let recording: Recording
    
    @State private var player: AVPlayer?
    @State private var showShareSheet = false
    @State private var isDownloading = false
    @State private var downloadError: String?
    @Environment(\.dismiss) private var dismiss
    
    var body: some View {
        ScrollView {
            VStack(spacing: 20) {
                // Video Player
                if let videoURL = recording.videoURL {
                    ZStack {
                        if let player = player {
                            VideoPlayer(player: player)
                                .frame(height: 250)
                                .cornerRadius(12)
                        } else if isDownloading {
                            RoundedRectangle(cornerRadius: 12)
                                .fill(Color.gray.opacity(0.3))
                                .frame(height: 250)
                                .overlay {
                                    VStack {
                                        ProgressView()
                                            .scaleEffect(1.5)
                                        Text("Downloading from iCloud...")
                                            .font(.headline)
                                            .padding(.top)
                                    }
                                    .foregroundColor(.gray)
                                }
                        } else if let error = downloadError {
                            RoundedRectangle(cornerRadius: 12)
                                .fill(Color.gray.opacity(0.3))
                                .frame(height: 250)
                                .overlay {
                                    VStack {
                                        Image(systemName: "exclamationmark.icloud")
                                            .font(.system(size: 50))
                                        Text(error)
                                            .font(.headline)
                                    }
                                    .foregroundColor(.gray)
                                }
                        } else {
                            RoundedRectangle(cornerRadius: 12)
                                .fill(Color.gray.opacity(0.3))
                                .frame(height: 250)
                                .overlay {
                                    ProgressView()
                                }
                        }
                    }
                    .task {
                        await loadVideo(from: videoURL)
                    }
                    .onDisappear {
                        player?.pause()
                    }
                } else {
                    RoundedRectangle(cornerRadius: 12)
                        .fill(Color.gray.opacity(0.3))
                        .frame(height: 250)
                        .overlay {
                            VStack {
                                Image(systemName: "video.slash")
                                    .font(.system(size: 50))
                                Text("Video not available")
                                    .font(.headline)
                            }
                            .foregroundColor(.gray)
                        }
                }
                
                // Metadata
                GroupBox("Recording Info") {
                    VStack(spacing: 12) {
                        InfoRow(label: "Date", value: recording.displayName)
                        InfoRow(label: "Duration", value: recording.durationString)
                        InfoRow(label: "Frames", value: recording.frameCountString)
                        InfoRow(label: "FPS", value: recording.fpsString)
                        InfoRow(label: "Size", value: recording.fileSizeString)
                        
                        if let metadata = recording.metadata {
                            Divider()
                            InfoRow(label: "Video Source", value: metadata.videoSource)
                            InfoRow(label: "Left Hand", value: metadata.hasLeftHand ? "Yes" : "No")
                            InfoRow(label: "Right Hand", value: metadata.hasRightHand ? "Yes" : "No")
                            InfoRow(label: "Head Tracking", value: metadata.hasHead ? "Yes" : "No")
                            
                            if let device = metadata.deviceInfo {
                                Divider()
                                InfoRow(label: "Device", value: device.model)
                                InfoRow(label: "OS Version", value: device.systemVersion)
                            }
                        }
                    }
                    .padding(.vertical, 8)
                }
                
                // Action Buttons
                VStack(spacing: 12) {
                    // 3D Viewer Button
                    if recording.trackingURL != nil {
                        NavigationLink(destination: SyncedPlaybackView(recording: recording)) {
                            Label("View 3D Skeleton", systemImage: "figure.arms.open")
                                .font(.headline)
                                .frame(maxWidth: .infinity)
                                .padding()
                                .background(Color.purple)
                                .foregroundColor(.white)
                                .cornerRadius(12)
                        }
                    }
                    
                    // Export Button
                    Button(action: { showShareSheet = true }) {
                        Label("Export Recording", systemImage: "square.and.arrow.up")
                            .font(.headline)
                            .frame(maxWidth: .infinity)
                            .padding()
                            .background(Color.blue)
                            .foregroundColor(.white)
                            .cornerRadius(12)
                    }
                }
                .padding(.top)
            }
            .padding()
        }
        .navigationTitle("Recording")
        .navigationBarTitleDisplayMode(.inline)
        .sheet(isPresented: $showShareSheet) {
            ShareSheet(items: [recording.folderURL])
        }
    }
    
    private func loadVideo(from url: URL) async {
        isDownloading = true
        downloadError = nil
        
        // Download from iCloud if needed
        let downloaded = await RecordingsManager.shared.downloadFromiCloud(url)
        
        isDownloading = false
        
        if downloaded {
            let newPlayer = AVPlayer(url: url)
            player = newPlayer
            // Auto-play
            newPlayer.play()
        } else {
            downloadError = "Failed to download"
        }
    }
}

struct InfoRow: View {
    let label: String
    let value: String
    
    var body: some View {
        HStack {
            Text(label)
                .foregroundColor(.secondary)
            Spacer()
            Text(value)
                .fontWeight(.medium)
        }
    }
}

/// UIKit Share Sheet wrapper
struct ShareSheet: UIViewControllerRepresentable {
    let items: [Any]
    
    func makeUIViewController(context: Context) -> UIActivityViewController {
        let controller = UIActivityViewController(activityItems: items, applicationActivities: nil)
        return controller
    }
    
    func updateUIViewController(_ uiViewController: UIActivityViewController, context: Context) {}
}

#Preview {
    NavigationStack {
        RecordingDetailView(
            recording: Recording(
                id: "test",
                folderURL: URL(fileURLWithPath: "/test"),
                metadata: nil,
                videoURL: nil,
                trackingURL: nil
            )
        )
    }
}
