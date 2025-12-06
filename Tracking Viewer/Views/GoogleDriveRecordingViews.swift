//
//  GoogleDriveRecordingViews.swift
//  Tracking Viewer
//
//  Created on 12/2/25.
//

import SwiftUI
import AVKit

// MARK: - Google Drive Recording Thumbnail View

struct GoogleDriveRecordingThumbnailView: View {
    let recording: GoogleDriveRecordingInfo
    
    @State private var thumbnail: UIImage?
    @State private var isLoadingThumbnail = false
    
    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            // Thumbnail
            ZStack {
                RoundedRectangle(cornerRadius: 12)
                    .fill(Color.gray.opacity(0.2))
                    .aspectRatio(16/9, contentMode: .fit)
                
                if let thumbnail = thumbnail {
                    Image(uiImage: thumbnail)
                        .resizable()
                        .aspectRatio(contentMode: .fill)
                        .frame(minWidth: 0, maxWidth: .infinity)
                        .aspectRatio(16/9, contentMode: .fit)
                        .clipShape(RoundedRectangle(cornerRadius: 12))
                } else if isLoadingThumbnail {
                    ProgressView()
                } else if recording.hasVideo {
                    Image(systemName: "video.fill")
                        .font(.largeTitle)
                        .foregroundColor(.gray)
                } else {
                    Image(systemName: "doc.text.fill")
                        .font(.largeTitle)
                        .foregroundColor(.gray)
                }
                
                // Google Drive badge
                VStack {
                    HStack {
                        Spacer()
                        Image(systemName: "externaldrive.fill")
                            .font(.caption)
                            .foregroundColor(.white)
                            .padding(4)
                            .background(Color(red: 0.26, green: 0.52, blue: 0.96))
                            .clipShape(Circle())
                            .padding(6)
                    }
                    Spacer()
                }
            }
            
            // Info
            VStack(alignment: .leading, spacing: 2) {
                Text(recording.displayName)
                    .font(.subheadline)
                    .fontWeight(.medium)
                    .lineLimit(1)
                
                HStack(spacing: 4) {
                    // File indicators
                    if recording.hasVideo {
                        Image(systemName: "video.fill")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                    if recording.hasTrackingData {
                        Image(systemName: "hand.raised.fill")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                    if recording.hasMetadata {
                        Image(systemName: "info.circle.fill")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                    
                    Spacer()
                }
                
                // Modified date
                if let date = recording.modifiedDate {
                    Text(date, style: .relative)
                        .font(.caption2)
                        .foregroundColor(.secondary)
                }
            }
        }
        .task {
            await loadThumbnail()
        }
    }
    
    private func loadThumbnail() async {
        guard recording.hasVideo, thumbnail == nil else { return }
        
        isLoadingThumbnail = true
        defer { isLoadingThumbnail = false }
        
        thumbnail = await RecordingsManager.shared.getGoogleDriveThumbnail(for: recording)
    }
}

// MARK: - Google Drive Recording Detail View

struct GoogleDriveRecordingDetailView: View {
    let recording: GoogleDriveRecordingInfo
    
    @StateObject private var playbackController = PlaybackController()
    @StateObject private var playlistManager = PlaylistManager.shared
    @StateObject private var annotationsManager = AnnotationsManager.shared
    @State private var isLoading = true
    @State private var loadError: String?
    @State private var followHead = true
    @State private var localVideoURL: URL?
    @State private var localTrackingURL: URL?
    @State private var showAddToPlaylistSheet = false
    @State private var showTagPicker = false
    @State private var showNotesEditor = false
    @State private var showRenameSheet = false
    @State private var customNameInput = ""
    @State private var isPublic = false
    @State private var isCheckingPublicStatus = false
    @State private var isTogglingPublic = false
    
    @StateObject private var cloudKitManager = CloudKitManager.shared
    
    private let googleDriveColor = Color(red: 0.26, green: 0.52, blue: 0.96)
    
    private var containingPlaylists: [Playlist] {
        // Use recording ID for playlist tracking
        playlistManager.playlists.filter { playlist in
            playlist.recordingIDs.contains(recording.id)
        }
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
                if recording.hasTrackingData || playbackController.totalFrames > 0 {
                    skeletonSection
                }
                
                // MARK: - Playback Controls
                if !isLoading && playbackController.totalFrames > 0 {
                    playbackControlsSection
                }
                
                // MARK: - Annotations Section (Tags, Notes)
                annotationsSection
                
                // MARK: - Playlists Section
                playlistsSection
                
                // MARK: - Recording Info
                recordingInfoSection
            }
            .padding()
        }
        .navigationTitle(customName ?? recording.displayName)
        .navigationBarTitleDisplayMode(.inline)
        .toolbar {
            ToolbarItem(placement: .topBarTrailing) {
                HStack(spacing: 12) {
                    // Public/Private toggle
                    Button {
                        Task {
                            await togglePublicStatus()
                        }
                    } label: {
                        if isTogglingPublic {
                            ProgressView()
                        } else {
                            Image(systemName: isPublic ? "globe.badge.chevron.backward" : "globe")
                                .foregroundColor(isPublic ? .green : .secondary)
                        }
                    }
                    .disabled(isTogglingPublic)
                    
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
            await checkPublicStatus()
        }
        .onDisappear {
            playbackController.cleanup()
        }
        .sheet(isPresented: $showAddToPlaylistSheet) {
            AddRecordingToPlaylistsSheet(recordingID: recording.id, recordingName: customName ?? recording.displayName)
        }
        .sheet(isPresented: $showTagPicker) {
            TagPickerView(recordingID: recording.id)
        }
        .sheet(isPresented: $showNotesEditor) {
            NotesEditorSheet(recordingID: recording.id, recordingName: customName ?? recording.displayName)
        }
        .sheet(isPresented: $showRenameSheet) {
            renameSheet
        }
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
            } else if isLoading && recording.hasVideo {
                RoundedRectangle(cornerRadius: 12)
                    .fill(Color.gray.opacity(0.3))
                    .frame(height: 220)
                    .overlay {
                        VStack {
                            ProgressView()
                                .scaleEffect(1.5)
                            Text("Downloading video...")
                                .font(.headline)
                                .padding(.top)
                        }
                        .foregroundColor(.gray)
                    }
            } else if !recording.hasVideo {
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
                        VStack {
                            ProgressView()
                            Text("Loading tracking data...")
                                .font(.caption)
                                .padding(.top, 8)
                        }
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
                SkeletonViewer3D(
                    leftHand: playbackController.currentFrame?.leftHand,
                    rightHand: playbackController.currentFrame?.rightHand,
                    headMatrix: playbackController.currentFrame?.headMatrix,
                    followHead: followHead
                )
                .frame(height: 300)
                .cornerRadius(12)
                .overlay(alignment: .topLeading) {
                    // Follow head toggle
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
            .tint(googleDriveColor)
            
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
                .foregroundColor(googleDriveColor)
                
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
                                playlistManager.removeRecordingByID(recording.id, from: playlist)
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
    
    // MARK: - Recording Info Section
    
    private var recordingInfoSection: some View {
        GroupBox {
            VStack(alignment: .leading, spacing: 12) {
                HStack {
                    Image(systemName: "externaldrive.fill")
                        .foregroundColor(googleDriveColor)
                    Text("Recording Info")
                        .font(.headline)
                }
                
                VStack(spacing: 8) {
                    InfoRow(label: "Name", value: recording.name)
                    
                    if let date = recording.modifiedDate {
                        InfoRow(label: "Modified", value: date.formatted(date: .long, time: .shortened))
                    }
                    
                    HStack {
                        Text("Contents")
                            .foregroundColor(.secondary)
                        Spacer()
                        HStack(spacing: 8) {
                            if recording.hasVideo {
                                Label("Video", systemImage: "video.fill")
                                    .font(.caption)
                                    .foregroundColor(.blue)
                            }
                            if recording.hasTrackingData {
                                Label("Tracking", systemImage: "hand.raised.fill")
                                    .font(.caption)
                                    .foregroundColor(.green)
                            }
                            if recording.hasMetadata {
                                Label("Metadata", systemImage: "info.circle.fill")
                                    .font(.caption)
                                    .foregroundColor(.orange)
                            }
                        }
                    }
                    
                    if playbackController.totalFrames > 0 {
                        Divider()
                        InfoRow(label: "Frames", value: "\(playbackController.totalFrames)")
                        InfoRow(label: "Duration", value: String(format: "%.2fs", playbackController.duration))
                    }
                }
            }
            .padding(.vertical, 4)
        }
    }
    
    // MARK: - Data Loading
    
    private func loadData() async {
        isLoading = true
        loadError = nil
        
        print("📂 Google Drive: Loading recording: \(recording.name)")
        print("📂 hasVideo: \(recording.hasVideo), videoFileId: \(recording.videoFileId ?? "nil")")
        print("📂 hasTrackingData: \(recording.hasTrackingData), trackingFileId: \(recording.trackingDataFileId ?? "nil")")
        
        // Download and load video
        if recording.hasVideo {
            print("📂 Downloading video...")
            if let url = await RecordingsManager.shared.downloadGoogleDriveVideo(for: recording) {
                print("✅ Video downloaded to: \(url)")
                localVideoURL = url
                await playbackController.setupPlayer(with: url, loop: true)
            } else {
                print("❌ Failed to download video")
            }
        }
        
        // Download and load tracking data
        if recording.hasTrackingData {
            print("📂 Downloading tracking data...")
            if let url = await RecordingsManager.shared.downloadGoogleDriveTrackingData(for: recording) {
                print("✅ Tracking data downloaded to: \(url)")
                localTrackingURL = url
                do {
                    let frames = try await TrackingDataLoader.loadTrackingData(from: url)
                    print("✅ Loaded \(frames.count) frames")
                    await playbackController.setTrackingData(frames)
                } catch {
                    print("❌ Failed to parse tracking data: \(error)")
                    loadError = "Failed to load tracking data: \(error.localizedDescription)"
                }
            } else {
                print("❌ Failed to download tracking data")
                loadError = "Failed to download tracking data"
            }
        }
        
        isLoading = false
        print("📂 Loading complete. totalFrames: \(playbackController.totalFrames)")
        
        // Auto-play after everything is loaded
        if localVideoURL != nil || playbackController.totalFrames > 0 {
            playbackController.play()
        }
    }
    
    // MARK: - Public/Private Sharing
    
    private func checkPublicStatus() async {
        isCheckingPublicStatus = true
        defer { isCheckingPublicStatus = false }
        
        isPublic = await cloudKitManager.isRecordingPublic(recordingId: recording.id)
    }
    
    private func togglePublicStatus() async {
        isTogglingPublic = true
        defer { isTogglingPublic = false }
        
        if isPublic {
            // Make private
            let success = await cloudKitManager.makeRecordingPrivate(recordingId: recording.id)
            if success {
                isPublic = false
                print("✅ Recording made private")
            }
        } else {
            // Make public - need to create shared link first
            guard let sharedURL = await GoogleDriveManager.shared.createSharedLink(fileId: recording.id) else {
                print("❌ Failed to create shared link")
                return
            }
            
            let success = await cloudKitManager.makeRecordingPublic(
                recordingId: recording.id,
                title: customName ?? recording.displayName,
                description: recordingNotes,
                cloudURL: sharedURL,
                thumbnailURL: nil,
                provider: .googleDrive
            )
            
            if success {
                isPublic = true
                print("✅ Recording made public")
            }
        }
    }
}


// MARK: - Preview

#Preview {
    NavigationStack {
        GoogleDriveRecordingDetailView(recording: GoogleDriveRecordingInfo(
            id: "test123",
            name: "recording_20241125_143000",
            modifiedDate: Date(),
            hasVideo: true,
            videoFileId: "video123",
            hasTrackingData: true,
            trackingDataFileId: "tracking123",
            hasMetadata: true,
            metadataFileId: "metadata123"
        ))
    }
}
