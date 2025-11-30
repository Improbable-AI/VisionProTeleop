//
//  SyncedPlaybackView.swift
//  Tracking Viewer
//
//  Created on 11/29/25.
//

import SwiftUI
import AVKit
import Combine

/// Synchronized video and 3D skeleton playback view
struct SyncedPlaybackView: View {
    let recording: Recording
    
    @StateObject private var playbackController = PlaybackController()
    @State private var isLoading = true
    @State private var loadError: String?
    @State private var followHead = true
    @Environment(\.dismiss) private var dismiss
    
    var body: some View {
        GeometryReader { geometry in
            VStack(spacing: 0) {
                if isLoading {
                    ProgressView("Loading tracking data...")
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                } else if let error = loadError {
                    VStack {
                        Image(systemName: "exclamationmark.triangle")
                            .font(.largeTitle)
                            .foregroundColor(.orange)
                        Text(error)
                            .padding()
                    }
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
                } else {
                    // Main content
                    if geometry.size.width > geometry.size.height {
                        // Landscape: side by side
                        HStack(spacing: 0) {
                            videoPlayerView
                                .frame(width: geometry.size.width * 0.5)
                            
                            skeletonView
                                .frame(width: geometry.size.width * 0.5)
                        }
                    } else {
                        // Portrait: stacked
                        VStack(spacing: 0) {
                            videoPlayerView
                                .frame(height: geometry.size.height * 0.4)
                            
                            skeletonView
                                .frame(height: geometry.size.height * 0.45)
                            
                            playbackControls
                                .frame(height: geometry.size.height * 0.15)
                        }
                    }
                }
            }
        }
        .navigationTitle("3D Playback")
        .navigationBarTitleDisplayMode(.inline)
        .task {
            await loadData()
        }
        .onDisappear {
            playbackController.cleanup()
        }
    }
    
    // MARK: - Views
    
    private var videoPlayerView: some View {
        Group {
            if let player = playbackController.player {
                VideoPlayer(player: player)
                    .cornerRadius(8)
                    .padding(4)
            } else {
                RoundedRectangle(cornerRadius: 8)
                    .fill(Color.gray.opacity(0.3))
                    .overlay {
                        Text("No video")
                            .foregroundColor(.gray)
                    }
                    .padding(4)
            }
        }
    }
    
    private var skeletonView: some View {
        ZStack {
            SkeletonViewer3D(
                leftHand: playbackController.currentFrame?.leftHand,
                rightHand: playbackController.currentFrame?.rightHand,
                headMatrix: playbackController.currentFrame?.headMatrix,
                followHead: followHead
            )
            .cornerRadius(8)
            .padding(4)
            
            // Frame info overlay
            VStack {
                HStack {
                    // Follow head toggle
                    Button(action: { followHead.toggle() }) {
                        HStack {
                            Image(systemName: followHead ? "person.fill" : "person")
                            Text(followHead ? "Following" : "Free")
                        }
                        .font(.caption)
                        .padding(8)
                        .background(.ultraThinMaterial)
                        .cornerRadius(8)
                    }
                    .padding(8)
                    
                    Spacer()
                    
                    VStack(alignment: .trailing) {
                        Text("Frame: \(playbackController.currentFrameIndex + 1)/\(playbackController.totalFrames)")
                        Text(String(format: "Time: %.2fs", playbackController.currentTime))
                    }
                    .font(.caption)
                    .padding(8)
                    .background(.ultraThinMaterial)
                    .cornerRadius(8)
                    .padding(8)
                }
                Spacer()
            }
        }
    }
    
    private var playbackControls: some View {
        VStack(spacing: 8) {
            // Timeline slider
            Slider(
                value: Binding(
                    get: { playbackController.currentTime },
                    set: { playbackController.seek(to: $0) }
                ),
                in: 0...max(playbackController.duration, 0.1)
            )
            .padding(.horizontal)
            
            // Control buttons
            HStack(spacing: 30) {
                // Step backward
                Button(action: { playbackController.stepBackward() }) {
                    Image(systemName: "backward.frame.fill")
                        .font(.title2)
                }
                
                // Play/Pause
                Button(action: { playbackController.togglePlayPause() }) {
                    Image(systemName: playbackController.isPlaying ? "pause.fill" : "play.fill")
                        .font(.title)
                }
                
                // Step forward
                Button(action: { playbackController.stepForward() }) {
                    Image(systemName: "forward.frame.fill")
                        .font(.title2)
                }
                
                Spacer()
                
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
                        .padding(.horizontal, 8)
                        .padding(.vertical, 4)
                        .background(.regularMaterial)
                        .cornerRadius(4)
                }
            }
            .padding(.horizontal)
        }
        .padding(.vertical, 8)
        .background(.regularMaterial)
    }
    
    // MARK: - Data Loading
    
    private func loadData() async {
        isLoading = true
        loadError = nil
        
        // Load video
        if let videoURL = recording.videoURL {
            let downloaded = await RecordingsManager.shared.downloadFromiCloud(videoURL)
            if downloaded {
                await playbackController.setupPlayer(with: videoURL)
            }
        }
        
        // Load tracking data
        if let trackingURL = recording.trackingURL {
            do {
                let frames = try await TrackingDataLoader.loadTrackingData(from: trackingURL)
                await playbackController.setTrackingData(frames)
            } catch {
                loadError = "Failed to load tracking data: \(error.localizedDescription)"
            }
        } else {
            loadError = "No tracking data available"
        }
        
        isLoading = false
    }
}

/// Controller for synchronized playback
@MainActor
class PlaybackController: ObservableObject {
    @Published var player: AVPlayer?
    @Published var isPlaying = false
    @Published var currentTime: Double = 0
    @Published var duration: Double = 0
    @Published var currentFrameIndex: Int = 0
    @Published var currentFrame: RecordedFrame?
    @Published var playbackSpeed: Float = 1.0
    
    private var frames: [RecordedFrame] = []
    private var timeObserver: Any?
    private var cancellables = Set<AnyCancellable>()
    
    var totalFrames: Int { frames.count }
    
    func setupPlayer(with url: URL) {
        let playerItem = AVPlayerItem(url: url)
        player = AVPlayer(playerItem: playerItem)
        
        // Get duration
        Task {
            if let duration = try? await playerItem.asset.load(.duration) {
                self.duration = CMTimeGetSeconds(duration)
            }
        }
        
        // Add time observer
        let interval = CMTime(seconds: 1/30, preferredTimescale: 600)
        timeObserver = player?.addPeriodicTimeObserver(forInterval: interval, queue: .main) { [weak self] time in
            Task { @MainActor in
                self?.updateCurrentTime(CMTimeGetSeconds(time))
            }
        }
        
        // Observe playback status
        player?.publisher(for: \.timeControlStatus)
            .receive(on: DispatchQueue.main)
            .sink { [weak self] status in
                self?.isPlaying = status == .playing
            }
            .store(in: &cancellables)
    }
    
    func setTrackingData(_ data: [RecordedFrame]) {
        frames = data.sorted { $0.timestamp < $1.timestamp }
        if !frames.isEmpty {
            currentFrame = frames[0]
            if duration == 0, let lastFrame = frames.last {
                duration = lastFrame.timestamp
            }
        }
    }
    
    private func updateCurrentTime(_ time: Double) {
        currentTime = time
        
        // Find matching frame
        if let index = frames.lastIndex(where: { $0.timestamp <= time }) {
            currentFrameIndex = index
            currentFrame = frames[index]
        }
    }
    
    func togglePlayPause() {
        if isPlaying {
            pause()
        } else {
            play()
        }
    }
    
    func play() {
        player?.play()
        player?.rate = playbackSpeed
    }
    
    func pause() {
        player?.pause()
    }
    
    func seek(to time: Double) {
        let cmTime = CMTime(seconds: time, preferredTimescale: 600)
        player?.seek(to: cmTime, toleranceBefore: .zero, toleranceAfter: .zero)
        updateCurrentTime(time)
    }
    
    func stepForward() {
        let newIndex = min(currentFrameIndex + 1, frames.count - 1)
        if newIndex < frames.count {
            seek(to: frames[newIndex].timestamp)
        }
    }
    
    func stepBackward() {
        let newIndex = max(currentFrameIndex - 1, 0)
        if newIndex >= 0 && !frames.isEmpty {
            seek(to: frames[newIndex].timestamp)
        }
    }
    
    func setSpeed(_ speed: Float) {
        playbackSpeed = speed
        if isPlaying {
            player?.rate = speed
        }
    }
    
    func cleanup() {
        if let observer = timeObserver, let player = player {
            player.removeTimeObserver(observer)
            timeObserver = nil
        }
        player?.pause()
        cancellables.removeAll()
    }
}

#Preview {
    NavigationStack {
        SyncedPlaybackView(
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
