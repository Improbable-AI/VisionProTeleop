//
//  RecordingDetailView.swift
//  Tracking Viewer
//
//  Created on 11/29/25.
//

import SwiftUI
import AVKit

/// Detailed view for a single recording with video playback and 3D skeleton
struct RecordingDetailView: View {
    let recording: Recording
    
    @StateObject private var playbackController = PlaybackController()
    @StateObject private var playlistManager = PlaylistManager.shared
    @StateObject private var annotationsManager = AnnotationsManager.shared
    @State private var isLoading = true
    @State private var loadError: String?
    @State private var followHead = true
    @State private var showSimulation = false
    @State private var showShareSheet = false
    @State private var showCopiedToast = false
    @State private var showAddToPlaylistSheet = false
    @State private var showTagPicker = false
    @State private var showNotesEditor = false
    @State private var showRenameSheet = false
    @State private var customNameInput = ""
    @StateObject private var httpServer = HTTPFileServer.shared
    @Environment(\.dismiss) private var dismiss
    
    private var containingPlaylists: [Playlist] {
        playlistManager.playlists(containing: recording)
    }
    
    private var isFavorite: Bool {
        annotationsManager.isFavorite(recording.id)
    }
    
    private var recordingTags: [Tag] {
        annotationsManager.tags(for: recording.id)
    }
    
    private var recordingNotes: String? {
        annotationsManager.notes(for: recording.id)
    }
    
    private var customName: String? {
        annotationsManager.annotation(for: recording.id)?.customName
    }
    
    var body: some View {
        ScrollView {
            VStack(spacing: 16) {
                // MARK: - 2D Video Player
                videoSection
                
                // MARK: - 3D Skeleton View
                if recording.trackingURL != nil {
                    skeletonSection
                }
                
                // MARK: - Playback Controls
                if recording.trackingURL != nil && !isLoading {
                    playbackControlsSection
                }
                
                // MARK: - Annotations Section (Tags, Notes)
                annotationsSection
                
                // MARK: - Playlists Section
                playlistsSection
                
                // MARK: - Recording Info
                recordingInfoSection
                
                // MARK: - Export & Download
                exportSection
            }
            .padding()
        }
        .navigationTitle(customName ?? recording.displayName)
        .navigationBarTitleDisplayMode(.inline)
        .toolbar {
            ToolbarItem(placement: .topBarTrailing) {
                HStack(spacing: 12) {
                    // Favorite button
                    Button {
                        annotationsManager.toggleFavorite(recording.id)
                    } label: {
                        Image(systemName: isFavorite ? "star.fill" : "star")
                            .foregroundColor(isFavorite ? .yellow : .primary)
                    }
                    
                    // Add to playlist
                    Button {
                        showAddToPlaylistSheet = true
                    } label: {
                        Image(systemName: "folder.badge.plus")
                    }
                }
            }
        }
        .task {
            await loadData()
        }
        .onDisappear {
            playbackController.cleanup()
        }
        .sheet(isPresented: $showShareSheet) {
            ShareSheet(items: [recording.folderURL])
        }
        .sheet(isPresented: $showAddToPlaylistSheet) {
            AddRecordingToPlaylistsSheet(recording: recording)
        }
        .sheet(isPresented: $showTagPicker) {
            TagPickerView(recordingID: recording.id)
        }
        .sheet(isPresented: $showNotesEditor) {
            NotesEditorSheet(recordingID: recording.id, recordingName: recording.displayName)
        }
        .sheet(isPresented: $showRenameSheet) {
            renameSheet
        }
        .overlay(alignment: .bottom) {
            if showCopiedToast {
                Text("Copied to clipboard")
                    .font(.subheadline)
                    .fontWeight(.medium)
                    .foregroundColor(.white)
                    .padding(.horizontal, 20)
                    .padding(.vertical, 12)
                    .background(Color.black.opacity(0.8))
                    .cornerRadius(25)
                    .padding(.bottom, 30)
                    .transition(.move(edge: .bottom).combined(with: .opacity))
            }
        }
        .animation(.easeInOut, value: showCopiedToast)
    }
    
    // MARK: - Rename Sheet
    
    private var renameSheet: some View {
        NavigationStack {
            Form {
                Section("Custom Name") {
                    TextField("Enter a custom name", text: $customNameInput)
                    
                    if !customNameInput.isEmpty {
                        Button("Clear Custom Name", role: .destructive) {
                            customNameInput = ""
                        }
                    }
                }
                
                Section {
                    Text("Original: \(recording.displayName)")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
            .navigationTitle("Rename Recording")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Cancel") {
                        showRenameSheet = false
                    }
                }
                ToolbarItem(placement: .confirmationAction) {
                    Button("Save") {
                        let name = customNameInput.trimmingCharacters(in: .whitespacesAndNewlines)
                        annotationsManager.setCustomName(name.isEmpty ? nil : name, for: recording.id)
                        showRenameSheet = false
                    }
                }
            }
            .onAppear {
                customNameInput = customName ?? ""
            }
        }
    }
    
    // MARK: - Video Section
    
    private var videoSection: some View {
        Group {
            if let player = playbackController.player {
                VideoPlayer(player: player)
                    .frame(height: 220)
                    .cornerRadius(12)
            } else if isLoading {
                RoundedRectangle(cornerRadius: 12)
                    .fill(Color.gray.opacity(0.3))
                    .frame(height: 220)
                    .overlay {
                        VStack {
                            ProgressView()
                                .scaleEffect(1.5)
                            Text("Loading...")
                                .font(.headline)
                                .padding(.top)
                        }
                        .foregroundColor(.gray)
                    }
            } else if recording.videoURL == nil {
                RoundedRectangle(cornerRadius: 12)
                    .fill(Color.gray.opacity(0.3))
                    .frame(height: 220)
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
        }
    }
    
    // MARK: - 3D Skeleton Section
    
    private var skeletonSection: some View {
        ZStack {
            if isLoading {
                RoundedRectangle(cornerRadius: 12)
                    .fill(Color.gray.opacity(0.3))
                    .frame(height: 300)
                    .overlay {
                        ProgressView("Loading tracking data...")
                    }
            } else if let error = loadError {
                RoundedRectangle(cornerRadius: 12)
                    .fill(Color.gray.opacity(0.3))
                    .frame(height: 300)
                    .overlay {
                        VStack {
                            Image(systemName: "exclamationmark.triangle")
                                .font(.largeTitle)
                                .foregroundColor(.orange)
                            Text(error)
                                .font(.caption)
                                .multilineTextAlignment(.center)
                                .padding()
                        }
                    }
            } else {
                ZStack {
                    if showSimulation, let usdzURL = recording.usdzURL {
                        SimulationViewer3D(
                            usdzURL: usdzURL,
                            currentFrame: playbackController.currentSimulationFrame
                        )
                    } else {
                        SkeletonViewer3D(
                            leftHand: playbackController.currentFrame?.leftHand,
                            rightHand: playbackController.currentFrame?.rightHand,
                            headMatrix: playbackController.currentFrame?.headMatrix,
                            followHead: followHead
                        )
                    }
                }
                .frame(height: 300)
                .cornerRadius(12)
                .overlay(alignment: .topLeading) {
                    HStack {
                        // Follow head toggle
                        if !showSimulation {
                            Button(action: { followHead.toggle() }) {
                                HStack(spacing: 4) {
                                    Image(systemName: followHead ? "person.fill" : "person")
                                    Text(followHead ? "Following" : "Free")
                                }
                                .font(.caption)
                                .padding(8)
                                .background(.ultraThinMaterial)
                                .cornerRadius(8)
                            }
                        }
                        
                        // Simulation toggle
                        if recording.metadata?.hasSimulationData == true {
                            Button(action: { showSimulation.toggle() }) {
                                HStack(spacing: 4) {
                                    Image(systemName: showSimulation ? "cube.fill" : "hand.point.up.left.fill")
                                    Text(showSimulation ? "Simulation" : "Skeleton")
                                }
                                .font(.caption)
                                .padding(8)
                                .background(.ultraThinMaterial)
                                .cornerRadius(8)
                            }
                        }
                    }
                    .padding(8)
                }
                .overlay(alignment: .topTrailing) {
                    // Frame info
                    VStack(alignment: .trailing, spacing: 2) {
                        Text("Frame: \(playbackController.currentFrameIndex + 1)/\(playbackController.totalFrames)")
                        Text(String(format: "Time: %.2fs", playbackController.currentTime))
                    }
                    .font(.caption2)
                    .padding(8)
                    .background(.ultraThinMaterial)
                    .cornerRadius(8)
                    .padding(8)
                }
            }
        }
    }
    
    // MARK: - Playback Controls Section
    
    private var playbackControlsSection: some View {
        VStack(spacing: 12) {
            // Timeline slider
            Slider(
                value: Binding(
                    get: { playbackController.currentTime },
                    set: { playbackController.seek(to: $0) }
                ),
                in: 0...max(playbackController.duration, 0.1)
            )
            
            // Control buttons
            HStack(spacing: 24) {
                // Step backward
                Button(action: { playbackController.stepBackward() }) {
                    Image(systemName: "backward.frame.fill")
                        .font(.title3)
                }
                
                // Play/Pause
                Button(action: { playbackController.togglePlayPause() }) {
                    Image(systemName: playbackController.isPlaying ? "pause.circle.fill" : "play.circle.fill")
                        .font(.system(size: 44))
                }
                
                // Step forward
                Button(action: { playbackController.stepForward() }) {
                    Image(systemName: "forward.frame.fill")
                        .font(.title3)
                }
                
                Spacer()
                
                // Loop indicator
                HStack(spacing: 4) {
                    Image(systemName: "repeat")
                        .font(.caption)
                    Text("Loop")
                        .font(.caption)
                }
                .foregroundColor(.green)
                
                // Speed control
                Menu {
                    ForEach([0.25, 0.5, 1.0, 1.5, 2.0], id: \.self) { speed in
                        Button("\(speed, specifier: "%.2f")x") {
                            playbackController.setSpeed(Float(speed))
                        }
                    }
                } label: {
                    Text("\(playbackController.playbackSpeed, specifier: "%.2f")x")
                        .font(.caption)
                        .fontWeight(.medium)
                        .padding(.horizontal, 10)
                        .padding(.vertical, 6)
                        .background(Color.secondary.opacity(0.2))
                        .cornerRadius(6)
                }
            }
        }
        .padding()
        .background(Color(.secondarySystemBackground))
        .cornerRadius(12)
    }
    
    // MARK: - Recording Info Section
    
    private var recordingInfoSection: some View {
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
                    InfoRow(label: "Left Hand", value: metadata.hasLeftHand ? "✓ Yes" : "✗ No")
                    InfoRow(label: "Right Hand", value: metadata.hasRightHand ? "✓ Yes" : "✗ No")
                    InfoRow(label: "Head Tracking", value: metadata.hasHead ? "✓ Yes" : "✗ No")
                    
                    if let device = metadata.deviceInfo {
                        Divider()
                        InfoRow(label: "Device", value: device.model)
                        InfoRow(label: "OS Version", value: device.systemVersion)
                    }
                }
            }
            .padding(.vertical, 8)
        }
    }
    
    // MARK: - Annotations Section
    
    private var annotationsSection: some View {
        GroupBox {
            VStack(alignment: .leading, spacing: 16) {
                // Header with favorite
                HStack {
                    Image(systemName: "tag.fill")
                        .foregroundColor(.blue)
                    Text("Annotations")
                        .font(.headline)
                    
                    Spacer()
                    
                    // Rename button
                    Button {
                        showRenameSheet = true
                    } label: {
                        Image(systemName: "pencil.circle")
                            .foregroundColor(.secondary)
                    }
                    
                    // Favorite toggle
                    Button {
                        annotationsManager.toggleFavorite(recording.id)
                    } label: {
                        Image(systemName: isFavorite ? "star.fill" : "star")
                            .foregroundColor(isFavorite ? .yellow : .secondary)
                    }
                }
                
                // Custom name (if set)
                if let name = customName {
                    HStack {
                        Text("Custom Name:")
                            .font(.caption)
                            .foregroundColor(.secondary)
                        Text(name)
                            .font(.subheadline)
                            .fontWeight(.medium)
                    }
                }
                
                Divider()
                
                // Tags section
                VStack(alignment: .leading, spacing: 8) {
                    HStack {
                        Text("Tags")
                            .font(.subheadline)
                            .fontWeight(.medium)
                        
                        Spacer()
                        
                        Button {
                            showTagPicker = true
                        } label: {
                            Image(systemName: "plus.circle")
                                .font(.caption)
                                .foregroundColor(.blue)
                        }
                    }
                    
                    if recordingTags.isEmpty {
                        Text("No tags")
                            .font(.caption)
                            .foregroundColor(.secondary)
                            .italic()
                    } else {
                        FlowLayout(spacing: 6) {
                            ForEach(recordingTags) { tag in
                                HStack(spacing: 4) {
                                    Text(tag.name)
                                        .font(.caption)
                                    
                                    Button {
                                        annotationsManager.removeTag(tag.id, from: recording.id)
                                    } label: {
                                        Image(systemName: "xmark.circle.fill")
                                            .font(.system(size: 10))
                                    }
                                }
                                .foregroundColor(.white)
                                .padding(.horizontal, 8)
                                .padding(.vertical, 4)
                                .background(tag.color.color)
                                .cornerRadius(12)
                            }
                        }
                    }
                }
                
                Divider()
                
                // Notes section
                VStack(alignment: .leading, spacing: 8) {
                    HStack {
                        Text("Notes")
                            .font(.subheadline)
                            .fontWeight(.medium)
                        
                        Spacer()
                        
                        Button {
                            showNotesEditor = true
                        } label: {
                            Image(systemName: recordingNotes?.isEmpty == false ? "pencil.circle" : "plus.circle")
                                .font(.caption)
                                .foregroundColor(.blue)
                        }
                    }
                    
                    if let notes = recordingNotes, !notes.isEmpty {
                        Text(notes)
                            .font(.caption)
                            .foregroundColor(.secondary)
                            .lineLimit(5)
                            .onTapGesture {
                                showNotesEditor = true
                            }
                    } else {
                        Text("No notes")
                            .font(.caption)
                            .foregroundColor(.secondary)
                            .italic()
                    }
                }
            }
            .padding(.vertical, 4)
        }
    }
    
    // MARK: - Playlists Section
    
    private var playlistsSection: some View {
        GroupBox {
            VStack(alignment: .leading, spacing: 12) {
                HStack {
                    Image(systemName: "folder.fill")
                        .foregroundColor(.purple)
                    Text("Playlists")
                        .font(.headline)
                    
                    Spacer()
                    
                    Button {
                        showAddToPlaylistSheet = true
                    } label: {
                        Image(systemName: "plus.circle")
                            .foregroundColor(.blue)
                    }
                }
                
                if containingPlaylists.isEmpty {
                    HStack {
                        Spacer()
                        VStack(spacing: 8) {
                            Text("Not in any playlist")
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                            
                            Button {
                                showAddToPlaylistSheet = true
                            } label: {
                                Text("Add to Playlist")
                                    .font(.caption)
                            }
                            .buttonStyle(.bordered)
                        }
                        Spacer()
                    }
                    .padding(.vertical, 8)
                } else {
                    ForEach(containingPlaylists) { playlist in
                        HStack(spacing: 10) {
                            ZStack {
                                RoundedRectangle(cornerRadius: 6)
                                    .fill(playlist.color.color.opacity(0.2))
                                    .frame(width: 32, height: 32)
                                
                                Image(systemName: playlist.icon.systemName)
                                    .font(.caption)
                                    .foregroundColor(playlist.color.color)
                            }
                            
                            Text(playlist.name)
                                .font(.subheadline)
                            
                            Spacer()
                            
                            Button {
                                playlistManager.removeRecording(recording, from: playlist)
                            } label: {
                                Image(systemName: "xmark.circle.fill")
                                    .foregroundColor(.secondary)
                            }
                        }
                    }
                }
            }
            .padding(.vertical, 4)
        }
    }
    
    // MARK: - Export Section
    
    private var exportSection: some View {
        VStack(spacing: 12) {
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
            
            // HTTP Download URL (if server is running)
            if httpServer.isRunning, let url = httpServer.getDownloadURL(for: recording) {
                VStack(spacing: 8) {
                    Text("Download URL")
                        .font(.caption)
                        .foregroundColor(.secondary)
                    
                    HStack {
                        Text(url)
                            .font(.system(.caption, design: .monospaced))
                            .foregroundColor(.secondary)
                            .lineLimit(1)
                            .truncationMode(.middle)
                        
                        Button(action: {
                            UIPasteboard.general.string = url
                            showCopiedToast = true
                            DispatchQueue.main.asyncAfter(deadline: .now() + 2) {
                                showCopiedToast = false
                            }
                        }) {
                            Image(systemName: "doc.on.doc")
                        }
                    }
                    .padding(10)
                    .background(Color(.secondarySystemBackground))
                    .cornerRadius(8)
                    
                    Button(action: {
                        UIPasteboard.general.string = "curl -o \(recording.id).zip \"\(url)\""
                        showCopiedToast = true
                        DispatchQueue.main.asyncAfter(deadline: .now() + 2) {
                            showCopiedToast = false
                        }
                    }) {
                        Label("Copy curl Command", systemImage: "terminal")
                            .font(.subheadline)
                            .frame(maxWidth: .infinity)
                            .padding(10)
                            .background(Color.orange.opacity(0.2))
                            .foregroundColor(.orange)
                            .cornerRadius(8)
                    }
                }
            }
        }
    }
    
    // MARK: - Data Loading
    
    private func loadData() async {
        isLoading = true
        loadError = nil
        
        // Load video
        if let videoURL = recording.videoURL {
            let downloaded = await RecordingsManager.shared.downloadFromiCloud(videoURL)
            if downloaded {
                await playbackController.setupPlayer(with: videoURL, loop: true)
            }
        }
        
        // Load tracking data
        if let trackingURL = recording.trackingURL {
            do {
                let frames = try await TrackingDataLoader.loadTrackingData(from: trackingURL)
                await playbackController.setTrackingData(frames)
                
                // Auto-play after loading
                playbackController.play()
            } catch {
                loadError = "Failed to load tracking data: \(error.localizedDescription)"
            }
        }
        
        // Load simulation data
        if let simulationURL = recording.simulationDataURL {
            do {
                let frames = try await SimulationDataLoader.loadSimulationData(from: simulationURL)
                await playbackController.setSimulationData(frames)
                
                // Download USDZ if available
                if let usdzURL = recording.usdzURL {
                    _ = await RecordingsManager.shared.downloadFromiCloud(usdzURL)
                }
                
                showSimulation = true // Default to simulation view if available
            } catch {
                dlog("⚠️ Failed to load simulation data: \(error)")
            }
        }
        
        isLoading = false
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
