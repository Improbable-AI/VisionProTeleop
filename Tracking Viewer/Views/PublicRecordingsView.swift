//
//  PublicRecordingsView.swift
//  Tracking Viewer
//
//  Created on 12/2/25.
//

import SwiftUI
import AVKit
import CloudKit

/// View showing all public recordings shared via CloudKit
struct PublicRecordingsView: View {
    @StateObject private var cloudKitManager = CloudKitManager.shared
    @State private var searchText = ""
    @State private var selectedFilter: PublicRecordingFilter = .all
    
    private let columns = [
        GridItem(.flexible(), spacing: 16)
    ]
    
    private var filteredRecordings: [PublicRecording] {
        var recordings = cloudKitManager.publicRecordings
        
        // Apply quick filter
        switch selectedFilter {
        case .all:
            break
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
        
        // Apply search text
        if !searchText.isEmpty {
            recordings = recordings.filter { recording in
                if recording.title.localizedCaseInsensitiveContains(searchText) {
                    return true
                }
                if let description = recording.description, description.localizedCaseInsensitiveContains(searchText) {
                    return true
                }
                return false
            }
        }
        
        return recordings
    }
    
    var body: some View {
        NavigationStack {
            VStack(spacing: 0) {
                // Filter Bar
                if !cloudKitManager.publicRecordings.isEmpty {
                    ScrollView(.horizontal, showsIndicators: false) {
                        HStack(spacing: 8) {
                            ForEach(PublicRecordingFilter.allCases, id: \.self) { filter in
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
                        }
                        .padding(.horizontal)
                    }
                    .padding(.vertical, 8)
                    .background(Color(.secondarySystemBackground))
                }
                
                Group {
                    if cloudKitManager.isLoading {
                        ProgressView("Loading public recordings...")
                            .frame(maxWidth: .infinity, maxHeight: .infinity)
                    } else if let error = cloudKitManager.errorMessage {
                        ContentUnavailableView {
                            Label("Failed to Load", systemImage: "exclamationmark.triangle")
                        } description: {
                            Text(error)
                        } actions: {
                            Button("Retry") {
                                Task {
                                    await cloudKitManager.fetchPublicRecordings()
                                }
                            }
                            .buttonStyle(.borderedProminent)
                        }
                    } else if cloudKitManager.publicRecordings.isEmpty {
                        ContentUnavailableView {
                            Label("No Public Recordings", systemImage: "globe.badge.chevron.backward")
                        } description: {
                            Text("No recordings have been shared publicly yet")
                        } actions: {
                            Button("Refresh") {
                                Task {
                                    await cloudKitManager.fetchPublicRecordings()
                                }
                            }
                            .buttonStyle(.bordered)
                        }
                    } else if filteredRecordings.isEmpty {
                        ContentUnavailableView.search(text: searchText)
                    } else {
                        ScrollView {
                            LazyVGrid(columns: columns, spacing: 20) {
                                ForEach(filteredRecordings) { recording in
                                    NavigationLink {
                                        PublicRecordingDetailView(recording: recording)
                                    } label: {
                                        PublicRecordingThumbnailView(recording: recording)
                                    }
                                    .buttonStyle(.plain)
                                    .trackVisibility(id: recording.id, in: .named("scroll"))
                                    .onAppear {
                                        // Load more when reaching the last item
                                        if recording.id == filteredRecordings.last?.id {
                                            Task {
                                                await cloudKitManager.loadMorePublicRecordings()
                                            }
                                        }
                                    }
                                }
                                
                                // Loading indicator at bottom
                                if cloudKitManager.isLoadingMore {
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
                                } else if cloudKitManager.hasMoreRecordings && !filteredRecordings.isEmpty {
                                    // Subtle indicator that more can be loaded
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
                        .coordinateSpace(name: "scroll")
                        .onPreferenceChange(VisibilityPreferenceKey.self) { visibility in
                            FeedPlayerManager.shared.updateVisibleItems(visibility)
                        }
                        .refreshable {
                            await cloudKitManager.fetchPublicRecordings()
                        }
                    }
                }
            }
            .onDisappear {
                FeedPlayerManager.shared.stopPlayback()
            }
            .navigationTitle("Public Recordings")
            .navigationBarTitleDisplayMode(.large)
            .searchable(text: $searchText, prompt: "Search public recordings...")
            .task {
                if cloudKitManager.publicRecordings.isEmpty {
                    await cloudKitManager.fetchPublicRecordings()
                }
            }
            .toolbar {
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        Task {
                            await cloudKitManager.fetchPublicRecordings()
                        }
                    } label: {
                        Image(systemName: "arrow.clockwise")
                    }
                }
            }
            .task {
                if cloudKitManager.publicRecordings.isEmpty {
                    await cloudKitManager.fetchPublicRecordings()
                }
            }
        }
    }
}

// MARK: - Filter Enum

enum PublicRecordingFilter: String, CaseIterable {
    case all = "All"
    case today = "Today"
    case thisWeek = "This Week"
    case thisMonth = "This Month"
    
    var icon: String {
        switch self {
        case .all: return "square.grid.2x2"
        case .today: return "calendar"
        case .thisWeek: return "calendar.badge.clock"
        case .thisMonth: return "calendar.circle"
        }
    }
}

// MARK: - Public Recording Thumbnail

struct PublicRecordingThumbnailView: View {
    let recording: PublicRecording
    
    @StateObject private var feedPlayer = FeedPlayerManager.shared
    @State private var player: AVPlayer?
    
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
                        if let asset = recording.thumbnailAsset,
                           let fileURL = asset.fileURL,
                           let data = try? Data(contentsOf: fileURL),
                           let image = UIImage(data: data) {
                            Image(uiImage: image)
                                .resizable()
                                .aspectRatio(contentMode: .fill)
                                .frame(minWidth: 0, maxWidth: .infinity)
                                .aspectRatio(16/9, contentMode: .fit)
                                .clipShape(RoundedRectangle(cornerRadius: 12))
                        } else if let urlString = recording.thumbnailURL, let url = URL(string: urlString) {
                            AsyncImage(url: url) { phase in
                                switch phase {
                                case .empty:
                                    ProgressView()
                                case .success(let image):
                                    image
                                        .resizable()
                                        .aspectRatio(contentMode: .fill)
                                        .frame(minWidth: 0, maxWidth: .infinity)
                                        .aspectRatio(16/9, contentMode: .fit)
                                        .clipShape(RoundedRectangle(cornerRadius: 12))
                                case .failure:
                                    Image(systemName: "video.fill")
                                        .font(.largeTitle)
                                        .foregroundColor(.gray)
                                @unknown default:
                                    EmptyView()
                                }
                            }
                        } else {
                            Image(systemName: "video.fill")
                                .font(.largeTitle)
                                .foregroundColor(.gray)
                        }
                    }
                }
                
                // Overlays (Always visible)
                VStack {
                    HStack {
                        // Provider badge
                        Image(systemName: recording.providerIcon)
                            .font(.caption)
                            .foregroundColor(.white)
                            .padding(4)
                            .background(recording.providerColor)
                            .clipShape(Circle())
                            .padding(6)
                        
                        Spacer()
                        
                        // Public badge
                        Image(systemName: "globe")
                            .font(.caption)
                            .foregroundColor(.white)
                            .padding(4)
                            .background(Color.green)
                            .clipShape(Circle())
                            .padding(6)
                    }
                    Spacer()
                }
            }
            
            // Info
            VStack(alignment: .leading, spacing: 2) {
                Text(recording.title)
                    .font(.subheadline)
                    .fontWeight(.medium)
                    .lineLimit(1)
                    .foregroundColor(.primary)
                
                HStack(spacing: 4) {
                    Text(recording.createdAt, style: .date)
                    if let description = recording.description {
                        Text("•")
                        Text(description)
                            .lineLimit(1)
                    }
                }
                .font(.caption)
                .foregroundColor(.secondary)
            }
            .padding(.horizontal, 4)
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
    }
    
    private func loadPlayer() async {
        guard isPlaying else { return }
        
        var videoURL: URL?
        
        // Resolve URL based on provider
        if recording.provider == .dropbox {
            // Dropbox direct link logic
            let baseURL = recording.cloudURL
                .replacingOccurrences(of: "www.dropbox.com", with: "dl.dropboxusercontent.com")
                .replacingOccurrences(of: "?dl=0", with: "")
            
            if let url = URL(string: baseURL + "/video.mp4") {
                videoURL = url
            }
        } else if recording.provider == .googleDrive {
            // Google Drive logic (requires download/cache)
            if GoogleDriveManager.shared.isAuthenticated {
                let (videoId, trackingId, _, _) = await GoogleDriveManager.shared.getRecordingFileIds(folderId: recording.recordingId)
                if let videoId = videoId {
                    let tempInfo = GoogleDriveRecordingInfo(
                        id: recording.recordingId,
                        name: recording.title,
                        modifiedDate: recording.createdAt,
                        hasVideo: true,
                        videoFileId: videoId,
                        hasTrackingData: false,
                        trackingDataFileId: nil,
                        hasMetadata: false,
                        metadataFileId: nil
                    )
                    videoURL = await RecordingsManager.shared.downloadGoogleDriveVideo(for: tempInfo)
                }
            }
        }
        
        if let url = videoURL {
            let newPlayer = AVPlayer(url: url)
            newPlayer.isMuted = true
            newPlayer.actionAtItemEnd = .none
            
            NotificationCenter.default.addObserver(forName: .AVPlayerItemDidPlayToEndTime, object: newPlayer.currentItem, queue: .main) { _ in
                newPlayer.seek(to: .zero)
                newPlayer.play()
            }
            
            self.player = newPlayer
            feedPlayer.registerPlayer(newPlayer, for: recording.id)
        }
    }
}

// MARK: - Public Recording Detail View

struct PublicRecordingDetailView: View {
    let recording: PublicRecording
    
    @StateObject private var playbackController = PlaybackController()
    @State private var isLoading = false
    @State private var loadError: String?
    @State private var localVideoURL: URL?
    @State private var followHead = true
    
    var body: some View {
        ScrollView {
            VStack(spacing: 16) {
                // MARK: - Video Section
                videoSection
                
                // MARK: - 3D Skeleton Section
                if playbackController.totalFrames > 0 {
                    skeletonSection
                }
                
                // MARK: - Playback Controls
                if !isLoading && playbackController.totalFrames > 0 {
                    playbackControlsSection
                }
                
                // MARK: - Info Section
                infoSection
                
                // MARK: - Actions
                Button {
                    if let url = URL(string: recording.cloudURL) {
                        UIApplication.shared.open(url)
                    }
                } label: {
                    Label("Open in \(recording.provider.rawValue.capitalized)", systemImage: "arrow.up.forward.app")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .tint(recording.providerColor)
                .controlSize(.large)
            }
            .padding()
        }
        .navigationTitle(recording.title)
        .navigationBarTitleDisplayMode(.inline)
        .task {
            await loadPreview()
        }
        .onDisappear {
            playbackController.cleanup()
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
                            Text("Loading preview...")
                                .font(.headline)
                                .padding(.top)
                        }
                        .foregroundColor(.gray)
                    }
            } else {
                // Thumbnail / Placeholder
                ZStack {
                    RoundedRectangle(cornerRadius: 12)
                        .fill(Color.gray.opacity(0.2))
                        .aspectRatio(16/9, contentMode: .fit)
                    
                    if let asset = recording.thumbnailAsset,
                       let fileURL = asset.fileURL,
                       let data = try? Data(contentsOf: fileURL),
                       let image = UIImage(data: data) {
                        Image(uiImage: image)
                            .resizable()
                            .aspectRatio(contentMode: .fill)
                            .frame(minWidth: 0, maxWidth: .infinity)
                            .aspectRatio(16/9, contentMode: .fit)
                            .clipShape(RoundedRectangle(cornerRadius: 12))
                            .overlay(Color.black.opacity(0.3))
                    } else {
                        Image(systemName: "video.fill")
                            .font(.system(size: 50))
                            .foregroundColor(.gray)
                    }
                    
                    if let error = loadError {
                        VStack {
                            Image(systemName: "exclamationmark.triangle")
                                .font(.largeTitle)
                                .foregroundColor(.orange)
                            Text(error)
                                .font(.caption)
                                .multilineTextAlignment(.center)
                                .padding()
                            
                            Button("Retry") {
                                Task {
                                    await loadPreview()
                                }
                            }
                            .buttonStyle(.bordered)
                            .padding(.top, 4)
                        }
                    } else {
                        ProgressView()
                    }
                }
            }
        }
    }
    
    // MARK: - Skeleton Section
    
    private var skeletonSection: some View {
        ZStack {
            SkeletonViewer3D(
                leftHand: playbackController.currentFrame?.leftHand,
                rightHand: playbackController.currentFrame?.rightHand,
                headMatrix: playbackController.currentFrame?.headMatrix,
                followHead: followHead
            )
            .frame(height: 300)
            .cornerRadius(12)
            .overlay(alignment: .topLeading) {
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
    
    // MARK: - Playback Controls
    
    private var playbackControlsSection: some View {
        VStack(spacing: 12) {
            Slider(
                value: Binding(
                    get: { playbackController.currentTime },
                    set: { playbackController.seek(to: $0) }
                ),
                in: 0...max(playbackController.duration, 0.1)
            )
            .tint(recording.providerColor)
            
            HStack(spacing: 24) {
                Button(action: { playbackController.stepBackward() }) {
                    Image(systemName: "backward.frame.fill")
                        .font(.title3)
                }
                
                Button(action: { playbackController.togglePlayPause() }) {
                    Image(systemName: playbackController.isPlaying ? "pause.circle.fill" : "play.circle.fill")
                        .font(.system(size: 44))
                }
                .foregroundColor(recording.providerColor)
                
                Button(action: { playbackController.stepForward() }) {
                    Image(systemName: "forward.frame.fill")
                        .font(.title3)
                }
                
                Spacer()
                
                HStack(spacing: 4) {
                    Image(systemName: "repeat")
                        .font(.caption)
                    Text("Loop")
                        .font(.caption)
                }
                .foregroundColor(.green)
                
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
    
    // MARK: - Info Section
    
    private var infoSection: some View {
        GroupBox {
            VStack(alignment: .leading, spacing: 12) {
                HStack {
                    Image(systemName: recording.providerIcon)
                        .foregroundColor(recording.providerColor)
                    Text("Recording Info")
                        .font(.headline)
                }
                
                VStack(spacing: 8) {
                    RecordingInfoRow(label: "Title", value: recording.title)
                    
                    if let description = recording.description {
                        RecordingInfoRow(label: "Description", value: description)
                    }
                    
                    RecordingInfoRow(label: "Provider", value: recording.provider.rawValue.capitalized)
                    RecordingInfoRow(label: "Shared", value: recording.createdAt.formatted(date: .long, time: .shortened))
                    
                    if playbackController.totalFrames > 0 {
                        RecordingInfoRow(label: "Frames", value: "\(playbackController.totalFrames)")
                        RecordingInfoRow(label: "Duration", value: String(format: "%.2fs", playbackController.duration))
                    }
                }
            }
            .padding(.vertical, 4)
        }
    }
    
    // MARK: - Data Loading
    
    private func loadPreview() async {
        isLoading = true
        loadError = nil
        
        defer { isLoading = false }
        
        // 1. Dropbox Handling
        if recording.provider == .dropbox {
            // Dropbox shared link is usually for a folder: https://www.dropbox.com/sh/xyz/folder?dl=0
            // We need to construct direct links for files inside
            
            // Base URL for direct access (replace www.dropbox.com with dl.dropboxusercontent.com)
            let baseURL = recording.cloudURL
                .replacingOccurrences(of: "www.dropbox.com", with: "dl.dropboxusercontent.com")
                .replacingOccurrences(of: "?dl=0", with: "")
            
            // Construct video URL
            // Note: If the shared link is ALREADY to a file, this might fail, but we assume folder structure based on app design
            let videoURLString = baseURL + "/video.mp4"
            let trackingURLString = baseURL + "/tracking.jsonl"
            
            if let videoURL = URL(string: videoURLString) {
                print("🎬 Playing from Dropbox direct link: \(videoURL)")
                await playbackController.setupPlayer(with: videoURL, loop: true)
                playbackController.play()
                
                // Try to load tracking data
                if let trackingURL = URL(string: trackingURLString) {
                    print("📊 Loading tracking data from: \(trackingURL)")
                    do {
                        let frames = try await TrackingDataLoader.loadTrackingData(from: trackingURL)
                        await playbackController.setTrackingData(frames)
                    } catch {
                        print("⚠️ Failed to load tracking data: \(error)")
                        // Don't show error to user if video works, just log it
                    }
                }
            } else {
                loadError = "Invalid Dropbox URL"
            }
            
        } else if recording.provider == .googleDrive {
            // 2. Google Drive Handling
            if GoogleDriveManager.shared.isAuthenticated {
                print("🎬 Authenticated with Google Drive, attempting download...")
                
                // recording.recordingId is the FOLDER ID
                let (videoId, trackingId, _, _) = await GoogleDriveManager.shared.getRecordingFileIds(folderId: recording.recordingId)
                
                guard let videoFileId = videoId else {
                    loadError = "Could not find video.mp4 in the recording folder."
                    return
                }
                
                // Download Video
                // We construct a temp info with the correct video file ID
                let tempInfo = GoogleDriveRecordingInfo(
                    id: recording.recordingId, // Folder ID
                    name: recording.title,
                    modifiedDate: recording.createdAt,
                    hasVideo: true,
                    videoFileId: videoFileId,
                    hasTrackingData: trackingId != nil,
                    trackingDataFileId: trackingId,
                    hasMetadata: false,
                    metadataFileId: nil
                )
                
                if let url = await RecordingsManager.shared.downloadGoogleDriveVideo(for: tempInfo) {
                    print("✅ Downloaded Google Drive video to: \(url)")
                    localVideoURL = url
                    await playbackController.setupPlayer(with: url, loop: true)
                    playbackController.play()
                    
                    // Download Tracking Data
                    if let trackingFileId = trackingId {
                        // We need to manually trigger tracking download since RecordingsManager helpers might not fit perfectly
                        // But wait, downloadGoogleDriveTrackingData uses trackingDataFileId from info
                        if let trackingUrl = await RecordingsManager.shared.downloadGoogleDriveTrackingData(for: tempInfo) {
                            do {
                                let frames = try await TrackingDataLoader.loadTrackingData(from: trackingUrl)
                                await playbackController.setTrackingData(frames)
                            } catch {
                                print("⚠️ Failed to load tracking data: \(error)")
                            }
                        }
                    }
                } else {
                    loadError = "Failed to download video from Google Drive."
                }
            } else {
                loadError = "Sign in to Google Drive in settings to preview this recording."
            }
        }
    }
}

// MARK: - Helper View

struct RecordingInfoRow: View {
    let label: String
    let value: String
    
    var body: some View {
        HStack(alignment: .top) {
            Text(label)
                .foregroundColor(.secondary)
            Spacer()
            Text(value)
                .multilineTextAlignment(.trailing)
        }
        .font(.subheadline)
    }
}

// MARK: - Preview

#Preview {
    NavigationStack {
        PublicRecordingsView()
    }
}
