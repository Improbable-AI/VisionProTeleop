//
//  UnifiedRecordingViews.swift
//  Tracking Viewer
//
//  Unified views for displaying recordings from any cloud source
//
//  Created on 12/2/25.
//

import SwiftUI
import AVKit

// MARK: - Unified Thumbnail View

struct UnifiedRecordingThumbnailView: View {
    let recording: AnyRecording
    var isSelected: Bool = false
    var isSelectionMode: Bool = false
    
    @State private var thumbnail: UIImage?
    @State private var isLoadingThumbnail = false
    @StateObject private var annotationsManager = AnnotationsManager.shared
    @StateObject private var feedPlayer = FeedPlayerManager.shared
    @ObservedObject private var cloudKitManager = CloudKitManager.shared
    @State private var player: AVPlayer?
    @State private var showMakePublicAlert = false
    @State private var showMakePrivateAlert = false
    @State private var isMakingPublic = false
    
    private var isFavorite: Bool {
        annotationsManager.isFavorite(recording.id)
    }
    
    private var recordingTags: [Tag] {
        annotationsManager.tags(for: recording.id)
    }
    
    private var sourceColor: Color {
        recording.source.color
    }
    
    private var isPlaying: Bool {
        feedPlayer.currentPlayingID == recording.id
    }
    
    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            // Thumbnail / Video Player
            ZStack {
                RoundedRectangle(cornerRadius: 12)
                    .fill(Color.gray.opacity(0.2))
                    .aspectRatio(16/9, contentMode: .fit)
                
                // Content Layer
                Group {
                    if isPlaying, let player = player {
                        CustomVideoPlayer(player: player)
                            .aspectRatio(16/9, contentMode: .fit)
                            .clipShape(RoundedRectangle(cornerRadius: 12))
                            .allowsHitTesting(false)
                    } else {
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
                        } else if recording.hasUSDZ {
                            // Simulation dataset with USDZ but no video
                            Image(systemName: "cube.fill")
                                .font(.largeTitle)
                                .foregroundColor(.gray)
                        } else {
                            Image(systemName: "doc.text.fill")
                                .font(.largeTitle)
                                .foregroundColor(.gray)
                        }
                    }
                }
                
                // Overlays (Always visible)
                VStack {
                    HStack {
                        // Source badge
                        Image(systemName: recording.source.icon)
                            .font(.caption)
                            .foregroundColor(.white)
                            .padding(4)
                            .background(sourceColor)
                            .clipShape(Circle())
                            .padding(6)
                        
                        Spacer()
                        
                        // Favorite indicator
                        if isFavorite {
                            Image(systemName: "star.fill")
                                .font(.caption)
                                .foregroundColor(.yellow)
                                .padding(6)
                        }
                        
                        // Make Public Button (only for cloud sources)
                        if recording.source != .iCloudDrive {
                            let isPublic = cloudKitManager.publicRecordings.contains { 
                                let match = $0.recordingId == recording.id
                                if match { print("✅ Found public match: \($0.recordingId)") }
                                return match
                            }
                            
                            Button {
                                if isPublic {
                                    showMakePrivateAlert = true
                                } else {
                                    showMakePublicAlert = true
                                }
                            } label: {
                                Image(systemName: "globe")
                                    .font(.headline) // Reduced from title2
                                    .foregroundColor(.white)
                                    .padding(8) // Reduced from 12
                                    .background(isPublic ? Color.green : Color.black.opacity(0.5))
                                    .clipShape(Circle())
                            }
                            .padding(.trailing, 4)
                        }
                    }
                    Spacer()
                }

                // Selection overlay (always on top if selecting)
                if isSelectionMode {
                    if isSelected {
                        // Selected: Clear view with checkmark
                        ZStack {
                            RoundedRectangle(cornerRadius: 12)
                                .stroke(Color.blue, lineWidth: 3)
                            
                            VStack {
                                HStack {
                                    Spacer()
                                    Image(systemName: "checkmark.circle.fill")
                                        .font(.title)
                                        .foregroundColor(.blue)
                                        .background(Circle().fill(Color.white))
                                        .padding(8)
                                }
                                Spacer()
                            }
                        }
                    } else {
                        // Unselected: Dark transparent layer
                        RoundedRectangle(cornerRadius: 12)
                            .fill(Color.black.opacity(0.4))
                    }
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
                
                // Tags preview
                if !recordingTags.isEmpty {
                    HStack(spacing: 4) {
                        ForEach(recordingTags.prefix(3)) { tag in
                            Text(tag.name)
                                .font(.caption2)
                                .padding(.horizontal, 6)
                                .padding(.vertical, 2)
                                .background(tag.color.color.opacity(0.2))
                                .foregroundColor(tag.color.color)
                                .cornerRadius(4)
                        }
                        if recordingTags.count > 3 {
                            Text("+\(recordingTags.count - 3)")
                                .font(.caption2)
                                .foregroundColor(.secondary)
                        }
                    }
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
        .onChange(of: isPlaying) { playing in
            if playing {
                Task {
                    await loadPlayer()
                }
            } else {
                player?.pause()
                player = nil
            }
        }
        .alert("Make Public?", isPresented: $showMakePublicAlert) {
            Button("Cancel", role: .cancel) {}
            Button("Make Public", role: .destructive) {
                Task {
                    isMakingPublic = true
                    let success = await RecordingsManager.shared.shareRecordingPublicly(recording)
                    isMakingPublic = false
                }
            }
        } message: {
            Text("This will make the recording publicly available to anyone with the link. Please ensure there are no privacy or confidentiality issues.\n\nYou can retract this at any time from the Public tab.")
        }
        .alert("Make Private?", isPresented: $showMakePrivateAlert) {
            Button("Cancel", role: .cancel) {}
            Button("Make Private", role: .destructive) {
                Task {
                    isMakingPublic = true
                    let success = await RecordingsManager.shared.unpublishRecording(recording)
                    isMakingPublic = false
                }
            }
        } message: {
            Text("This will remove the recording from the public list. The shared link will no longer work for new users.")
        }
        .overlay {
            if isMakingPublic {
                ZStack {
                    Color.black.opacity(0.4)
                        .clipShape(RoundedRectangle(cornerRadius: 12))
                    ProgressView()
                        .tint(.white)
                }
            }
        }
    }
    
    private func loadThumbnail() async {
        // Try to load thumbnail if we have video OR USDZ (for simulation datasets)
        guard (recording.hasVideo || recording.hasUSDZ), thumbnail == nil else { return }
        
        isLoadingThumbnail = true
        defer { isLoadingThumbnail = false }
        
        switch recording.source {
        case .iCloudDrive:
            if let rec = recording.iCloudRecording {
                thumbnail = await RecordingsManager.shared.getThumbnail(for: rec)
            }
        case .dropbox:
            if let rec = recording.dropboxRecording {
                thumbnail = await RecordingsManager.shared.getDropboxThumbnail(for: rec)
            }
        case .googleDrive:
            if let rec = recording.googleDriveRecording {
                thumbnail = await RecordingsManager.shared.getGoogleDriveThumbnail(for: rec)
            }
        }
    }
    
    private func loadPlayer() async {
        // Only load if we are supposed to be playing
        guard isPlaying else { return }
        
        var videoURL: URL?
        
        switch recording.source {
        case .iCloudDrive:
            if let rec = recording.iCloudRecording {
                videoURL = rec.videoURL
            }
        case .dropbox:
            if let rec = recording.dropboxRecording {
                // For Dropbox, we might need to download or get a temp link
                // For feed auto-play, we prefer local cache if available
                videoURL = await RecordingsManager.shared.downloadDropboxVideo(for: rec)
            }
        case .googleDrive:
            if let rec = recording.googleDriveRecording {
                videoURL = await RecordingsManager.shared.downloadGoogleDriveVideo(for: rec)
            }
        }
        
        if let url = videoURL {
            let newPlayer = AVPlayer(url: url)
            newPlayer.isMuted = true
            newPlayer.actionAtItemEnd = .none
            
            // Loop logic
            NotificationCenter.default.addObserver(forName: .AVPlayerItemDidPlayToEndTime, object: newPlayer.currentItem, queue: .main) { _ in
                newPlayer.seek(to: .zero)
                newPlayer.play()
            }
            
            self.player = newPlayer
            feedPlayer.registerPlayer(newPlayer, for: recording.id)
        }
    }
}

// MARK: - Unified Detail View

struct UnifiedRecordingDetailView: View {
    let recording: AnyRecording
    
    @StateObject private var playbackController = PlaybackController()
    @StateObject private var annotationsManager = AnnotationsManager.shared
    @StateObject private var playlistManager = PlaylistManager.shared
    @State private var isLoading = true
    @State private var loadError: String?
    @State private var followHead = true
    @State private var localVideoURL: URL?
    @State private var localTrackingURL: URL?

    @State private var localSimulationURL: URL?
    @State private var localUSDZURL: URL?
    @State private var loadedMetadata: RecordingMetadata?
    @State private var showSimulation = false
    
    // Sheets
    @State private var showAddToPlaylistSheet = false
    @State private var showTagPicker = false
    @State private var showNotesEditor = false
    @State private var showRenameSheet = false
    @State private var customNameInput = ""
    
    // Public Sharing
    @StateObject private var cloudKitManager = CloudKitManager.shared
    @State private var isPublic = false
    @State private var isCheckingPublicStatus = false
    @State private var isTogglingPublic = false
    
    private var sourceColor: Color {
        recording.source.color
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
    
    private var containingPlaylists: [Playlist] {
        playlistManager.playlists.filter { playlist in
            playlist.recordingIDs.contains(recording.id)
        }
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
                    // Public/Private toggle (only for Dropbox/Google Drive)
                    if recording.source == .dropbox || recording.source == .googleDrive {
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
                    }
                    
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
            // Check public status if supported
            if recording.source == .dropbox || recording.source == .googleDrive {
                await checkPublicStatus()
            }
        }
        .onDisappear {
            playbackController.cleanup()
        }
        .sheet(isPresented: $showAddToPlaylistSheet) {
            AddAnyRecordingToPlaylistsSheet(recording: recording)
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
    
    @ViewBuilder
    private var videoSection: some View {
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
        }
        // No else case - don't show anything when video is unavailable
    }
    
    // MARK: - 3D Skeleton Section
    
    private var skeletonSection: some View {
        VStack(spacing: 12) {
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
                // Simulation View (if available)
                if let usdzURL = localUSDZURL, playbackController.currentSimulationFrame != nil {
                    simulationViewerSection(usdzURL: usdzURL)
                }
                
                // Hand Tracking View (if available)
                if playbackController.currentFrame != nil {
                    handTrackingSection
                }
            }
        }
    }
    
    private func simulationViewerSection(usdzURL: URL) -> some View {
        ZStack {
            SimulationViewer3D(
                usdzURL: usdzURL,
                currentFrame: playbackController.currentSimulationFrame
            )
            .frame(height: 300)
            .cornerRadius(12)
        }
        .overlay(alignment: .topLeading) {
            HStack(spacing: 4) {
                Image(systemName: "cube.fill")
                Text("Simulation")
            }
            .font(.caption)
            .padding(8)
            .background(.ultraThinMaterial)
            .cornerRadius(8)
            .padding(8)
        }
        .overlay(alignment: .topTrailing) {
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
    
    private var handTrackingSection: some View {
        ZStack {
            SkeletonViewer3D(
                leftHand: playbackController.currentFrame?.leftHand,
                rightHand: playbackController.currentFrame?.rightHand,
                headMatrix: playbackController.currentFrame?.headMatrix,
                followHead: followHead
            )
            .frame(height: 250)
            .cornerRadius(12)
        }
        .overlay(alignment: .topLeading) {
            HStack {
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
                
                HStack(spacing: 4) {
                    Image(systemName: "hand.point.up.left.fill")
                    Text("Hand Tracking")
                }
                .font(.caption)
                .padding(8)
                .background(.ultraThinMaterial)
                .cornerRadius(8)
            }
            .padding(8)
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
            .tint(sourceColor)
            
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
                .foregroundColor(sourceColor)
                
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
                    Image(systemName: recording.source.icon)
                        .foregroundColor(sourceColor)
                    Text("Recording Info")
                        .font(.headline)
                }
                
                VStack(spacing: 8) {
                    // Basic info for all sources
                    InfoRow(label: "Source", value: recording.source.displayName)
                    InfoRow(label: "Date", value: recording.displayName)
                    
                    if let date = recording.modifiedDate {
                        InfoRow(label: "Modified", value: date.formatted(date: .long, time: .shortened))
                    }
                    
                    // Show playback info if available (works for all sources after loading)
                    if playbackController.totalFrames > 0 {
                        InfoRow(label: "Frames", value: "\(playbackController.totalFrames)")
                        InfoRow(label: "Duration", value: String(format: "%.2fs", playbackController.duration))
                        
                        if playbackController.duration > 0 {
                            let fps = Double(playbackController.totalFrames) / playbackController.duration
                            InfoRow(label: "FPS", value: String(format: "%.1f", fps))
                        }
                    }
                    
                    // For iCloud Drive, we can show file size
                    if let iCloudRec = recording.iCloudRecording {
                        InfoRow(label: "Size", value: iCloudRec.fileSizeString)
                    }
                    
                    // Show metadata if available (for ALL sources)
                    let metadata = loadedMetadata ?? recording.iCloudRecording?.metadata
                    if let metadata = metadata {
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
                    
                    Divider()
                    
                    // Contents (for all sources)
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
                }
            }
            .padding(.vertical, 4)
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
            
            guard let url = sharedURL, let prov = provider else {
                print("❌ Failed to create shared link")
                return
            }
            
            // Generate thumbnail asset
            let thumbnailAssetURL = await generateThumbnailAsset()
            
            let customName = annotationsManager.annotation(for: recording.id)?.customName
            let notes = annotationsManager.notes(for: recording.id)
            
            let success = await cloudKitManager.makeRecordingPublic(
                recordingId: recording.id,
                title: customName ?? recording.displayName,
                description: notes,
                cloudURL: url,
                thumbnailURL: nil,
                thumbnailAssetURL: thumbnailAssetURL,
                provider: prov
            )
            
            // Clean up temp file
            if let assetURL = thumbnailAssetURL {
                try? FileManager.default.removeItem(at: assetURL)
            }
            
            if success {
                isPublic = true
                print("✅ Recording made public")
            }
        }
    }
    
    private func generateThumbnailAsset() async -> URL? {
        var image: UIImage?
        
        // Get thumbnail image
        switch recording.source {
        case .iCloudDrive:
            if let rec = recording.iCloudRecording {
                image = await RecordingsManager.shared.getThumbnail(for: rec)
            }
        case .dropbox:
            if let rec = recording.dropboxRecording {
                image = await RecordingsManager.shared.getDropboxThumbnail(for: rec)
            }
        case .googleDrive:
            if let rec = recording.googleDriveRecording {
                image = await RecordingsManager.shared.getGoogleDriveThumbnail(for: rec)
            }
        }
        
        guard let thumbnailImage = image,
              let data = thumbnailImage.jpegData(compressionQuality: 0.7) else {
            return nil
        }
        
        // Save to temp file
        let tempDir = FileManager.default.temporaryDirectory
        let fileName = "thumb_\(UUID().uuidString).jpg"
        let fileURL = tempDir.appendingPathComponent(fileName)
        
        do {
            try data.write(to: fileURL)
            return fileURL
        } catch {
            print("❌ Failed to save temp thumbnail: \(error)")
            return nil
        }
    }
    
    // MARK: - Data Loading
    
    private func loadData() async {
        isLoading = true
        loadError = nil
        
        print("🎬 Loading recording: \(recording.name) from \(recording.source.displayName)")
        
        switch recording.source {
        case .iCloudDrive:
            await loadiCloudData()
        case .dropbox:
            await loadDropboxData()
        case .googleDrive:
            await loadGoogleDriveData()
        }
        
        isLoading = false
        
        // Auto-play after everything is loaded
        if localVideoURL != nil || playbackController.totalFrames > 0 {
            playbackController.play()
        }
        
        print("🎬 Loading complete. totalFrames: \(playbackController.totalFrames)")
    }
    
    private func loadiCloudData() async {
        guard let rec = recording.iCloudRecording else { return }
        
        // Load video
        if let videoURL = rec.videoURL {
            let downloaded = await RecordingsManager.shared.downloadFromiCloud(videoURL)
            if downloaded {
                localVideoURL = videoURL
                await playbackController.setupPlayer(with: videoURL, loop: true)
            }
        }
        
        // Load tracking data
        if let trackingURL = rec.trackingURL {
            do {
                let frames = try await TrackingDataLoader.loadTrackingData(from: trackingURL)
                await playbackController.setTrackingData(frames)
            } catch {
                loadError = "Failed to load tracking data: \(error.localizedDescription)"
            }
        }
        
        // Load simulation data
        if let simulationURL = rec.simulationDataURL {
            do {
                let frames = try await SimulationDataLoader.loadSimulationData(from: simulationURL)
                await playbackController.setSimulationData(frames)
                
                // Download USDZ if available
                if let usdzURL = rec.usdzURL {
                    if await RecordingsManager.shared.downloadFromiCloud(usdzURL) {
                        localUSDZURL = usdzURL
                        showSimulation = true
                    }
                }
            } catch {
                print("⚠️ Failed to load simulation data: \(error)")
            }
        }
    }
    
    private func loadDropboxData() async {
        guard let rec = recording.dropboxRecording else { return }
        
        // Download and load video
        if rec.hasVideo {
            if let url = await RecordingsManager.shared.downloadDropboxVideo(for: rec) {
                localVideoURL = url
                await playbackController.setupPlayer(with: url, loop: true)
            }
        }
        
        // Download and load tracking data
        if rec.hasTracking {
            if let url = await RecordingsManager.shared.downloadDropboxTrackingData(for: rec) {
                localTrackingURL = url
                do {
                    let frames = try await TrackingDataLoader.loadTrackingData(from: url)
                    await playbackController.setTrackingData(frames)
                } catch {
                    loadError = "Failed to load tracking data: \(error.localizedDescription)"
                }
            } else {
                loadError = "Failed to download tracking data"
            }
        }
        
        // Download and load metadata
        if rec.hasMetadata {
            if let url = await RecordingsManager.shared.downloadDropboxMetadata(for: rec) {
                do {
                    let data = try Data(contentsOf: url)
                    let decoder = JSONDecoder()
                    decoder.dateDecodingStrategy = .iso8601
                    loadedMetadata = try decoder.decode(RecordingMetadata.self, from: data)
                } catch {
                    print("⚠️ Failed to parse metadata: \(error)")
                }
            }
        }
        
        // Download and load simulation data
        if let url = await RecordingsManager.shared.downloadDropboxSimulationData(for: rec) {
            localSimulationURL = url
            do {
                let frames = try await SimulationDataLoader.loadSimulationData(from: url)
                await playbackController.setSimulationData(frames)
                
                // Download USDZ
                if let usdzUrl = await RecordingsManager.shared.downloadDropboxUSDZ(for: rec) {
                    localUSDZURL = usdzUrl
                    showSimulation = true
                }
            } catch {
                print("⚠️ Failed to load simulation data: \(error)")
            }
        }
    }
    
    private func loadGoogleDriveData() async {
        guard let rec = recording.googleDriveRecording else { return }
        
        // Download and load video
        if rec.hasVideo {
            if let url = await RecordingsManager.shared.downloadGoogleDriveVideo(for: rec) {
                localVideoURL = url
                await playbackController.setupPlayer(with: url, loop: true)
            }
        }
        
        // Download and load tracking data
        if rec.hasTrackingData {
            if let url = await RecordingsManager.shared.downloadGoogleDriveTrackingData(for: rec) {
                localTrackingURL = url
                do {
                    let frames = try await TrackingDataLoader.loadTrackingData(from: url)
                    await playbackController.setTrackingData(frames)
                } catch {
                    loadError = "Failed to load tracking data: \(error.localizedDescription)"
                }
            } else {
                loadError = "Failed to download tracking data"
            }
        }
        
        // Download and load metadata
        if rec.hasMetadata {
            if let url = await RecordingsManager.shared.downloadGoogleDriveMetadata(for: rec) {
                do {
                    let data = try Data(contentsOf: url)
                    let decoder = JSONDecoder()
                    decoder.dateDecodingStrategy = .iso8601
                    loadedMetadata = try decoder.decode(RecordingMetadata.self, from: data)
                } catch {
                    print("⚠️ Failed to parse metadata: \(error)")
                }
            }
        }
        
        // Download and load simulation data
        if let url = await RecordingsManager.shared.downloadGoogleDriveSimulationData(for: rec) {
            localSimulationURL = url
            do {
                let frames = try await SimulationDataLoader.loadSimulationData(from: url)
                await playbackController.setSimulationData(frames)
                
                // Download USDZ
                if let usdzUrl = await RecordingsManager.shared.downloadGoogleDriveUSDZ(for: rec) {
                    localUSDZURL = usdzUrl
                    showSimulation = true
                }
            } catch {
                print("⚠️ Failed to load simulation data: \(error)")
            }
        }
    }
}

// MARK: - Add to Playlist Sheet for AnyRecording

struct AddAnyRecordingToPlaylistsSheet: View {
    let recording: AnyRecording
    @StateObject private var playlistManager = PlaylistManager.shared
    @Environment(\.dismiss) private var dismiss
    @State private var showCreateSheet = false
    
    var body: some View {
        NavigationStack {
            List {
                if playlistManager.playlists.isEmpty {
                    VStack(spacing: 16) {
                        Image(systemName: "folder.badge.plus")
                            .font(.system(size: 50))
                            .foregroundColor(.gray)
                        
                        Text("No Playlists")
                            .font(.headline)
                        
                        Text("Create a playlist first to organize this recording")
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                            .multilineTextAlignment(.center)
                        
                        Button {
                            showCreateSheet = true
                        } label: {
                            Label("Create Playlist", systemImage: "plus")
                        }
                        .buttonStyle(.bordered)
                    }
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 20)
                    .listRowBackground(Color.clear)
                } else {
                    ForEach(playlistManager.playlists) { playlist in
                        UnifiedPlaylistSelectionRow(playlist: playlist, recordingID: recording.id)
                    }
                }
            }
            .navigationTitle("Add to Playlist")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Done") {
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

struct UnifiedPlaylistSelectionRow: View {
    let playlist: Playlist
    let recordingID: String
    
    @StateObject private var playlistManager = PlaylistManager.shared
    
    private var isInPlaylist: Bool {
        playlist.recordingIDs.contains(recordingID)
    }
    
    var body: some View {
        Button {
            if isInPlaylist {
                playlistManager.removeRecording(recordingID, from: playlist)
            } else {
                playlistManager.addRecording(recordingID, to: playlist)
            }
        } label: {
            HStack {
                Image(systemName: playlist.icon.systemName)
                    .foregroundColor(playlist.color.color)
                    .frame(width: 30)
                
                VStack(alignment: .leading) {
                    Text(playlist.name)
                        .foregroundColor(.primary)
                    Text("\(playlist.recordingIDs.count) recordings")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                
                Spacer()
                
                if isInPlaylist {
                    Image(systemName: "checkmark.circle.fill")
                        .foregroundColor(.green)
                } else {
                    Image(systemName: "circle")
                        .foregroundColor(.gray)
                }
            }
        }
    }
}

// MARK: - Add Multiple Unified Recordings to Playlist Sheet

struct AddMultipleUnifiedRecordingsToPlaylistSheet: View {
    let recordings: [AnyRecording]
    
    @Environment(\.dismiss) private var dismiss
    @StateObject private var playlistManager = PlaylistManager.shared
    @State private var selectedPlaylists: Set<UUID> = []
    @State private var showCreatePlaylist = false
    
    var body: some View {
        NavigationStack {
            VStack {
                if playlistManager.playlists.isEmpty {
                    ContentUnavailableView(
                        "No Playlists",
                        systemImage: "folder",
                        description: Text("Create a playlist to add recordings to")
                    )
                } else {
                    List {
                        ForEach(playlistManager.playlists) { playlist in
                            MultiUnifiedPlaylistRow(
                                playlist: playlist,
                                recordingIDs: recordings.map { $0.id },
                                isSelected: selectedPlaylists.contains(playlist.id)
                            ) {
                                if selectedPlaylists.contains(playlist.id) {
                                    selectedPlaylists.remove(playlist.id)
                                } else {
                                    selectedPlaylists.insert(playlist.id)
                                }
                            }
                        }
                    }
                }
            }
            .navigationTitle("Add \(recordings.count) to Playlists")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Cancel") {
                        dismiss()
                    }
                }
                
                ToolbarItem(placement: .primaryAction) {
                    Button("Add") {
                        for playlistID in selectedPlaylists {
                            if let playlist = playlistManager.playlists.first(where: { $0.id == playlistID }) {
                                for recording in recordings {
                                    playlistManager.addRecording(recording.id, to: playlist)
                                }
                            }
                        }
                        dismiss()
                    }
                    .disabled(selectedPlaylists.isEmpty)
                }
                
                ToolbarItem(placement: .bottomBar) {
                    Button(action: { showCreatePlaylist = true }) {
                        Label("New Playlist", systemImage: "plus")
                    }
                }
            }
            .sheet(isPresented: $showCreatePlaylist) {
                PlaylistEditSheet(mode: .create)
            }
        }
    }
}

// MARK: - Multiple Unified Playlist Row

private struct MultiUnifiedPlaylistRow: View {
    let playlist: Playlist
    let recordingIDs: [String]
    let isSelected: Bool
    let action: () -> Void
    
    private var containedCount: Int {
        recordingIDs.filter { playlist.recordingIDs.contains($0) }.count
    }
    
    var body: some View {
        Button(action: action) {
            HStack {
                Image(systemName: playlist.icon.systemName)
                    .foregroundColor(playlist.color.color)
                    .frame(width: 30)
                
                VStack(alignment: .leading) {
                    Text(playlist.name)
                        .foregroundColor(.primary)
                    
                    if containedCount > 0 {
                        Text("\(containedCount) of \(recordingIDs.count) already in playlist")
                            .font(.caption)
                            .foregroundColor(.orange)
                    } else {
                        Text("\(playlist.recordingIDs.count) recordings")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                }
                
                Spacer()
                
                if isSelected {
                    Image(systemName: "checkmark.circle.fill")
                        .foregroundColor(.blue)
                } else {
                    Image(systemName: "circle")
                        .foregroundColor(.gray)
                }
            }
        }
    }
}

// MARK: - Preview

#Preview {
    NavigationStack {
        UnifiedRecordingDetailView(recording: AnyRecording(GoogleDriveRecordingInfo(
            id: "test123",
            name: "recording_20241125_143000",
            modifiedDate: Date(),
            hasVideo: true,
            videoFileId: "video123",
            hasTrackingData: true,
            trackingDataFileId: "tracking123",
            hasMetadata: true,
            metadataFileId: "metadata123"
        )))
    }
}
