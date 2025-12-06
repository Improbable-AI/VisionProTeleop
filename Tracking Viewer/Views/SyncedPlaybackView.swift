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
    @Published var currentSimulationFrame: SimulationFrame?
    @Published var playbackSpeed: Float = 1.0
    
    private var frames: [RecordedFrame] = []
    private var simulationFrames: [SimulationFrame] = []
    private var timeObserver: Any?
    private var cancellables = Set<AnyCancellable>()
    private var loopObserver: NSObjectProtocol?
    
    // Timer-based playback for simulation-only recordings (no video)
    private var simulationTimer: Timer?
    private var simulationStartTime: Date?
    
    var totalFrames: Int { max(frames.count, simulationFrames.count) }
    var totalSimulationFrames: Int { simulationFrames.count }
    
    func setupPlayer(with url: URL, loop: Bool = false) {
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
        
        // Loop playback
        if loop {
            loopObserver = NotificationCenter.default.addObserver(
                forName: .AVPlayerItemDidPlayToEndTime,
                object: playerItem,
                queue: .main
            ) { [weak self] _ in
                self?.player?.seek(to: .zero)
                self?.player?.play()
                if let speed = self?.playbackSpeed {
                    self?.player?.rate = speed
                }
            }
        }
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
    
    func setSimulationData(_ data: [SimulationFrame]) {
        simulationFrames = data.sorted { $0.timestamp < $1.timestamp }
        if !simulationFrames.isEmpty {
            currentSimulationFrame = simulationFrames[0]
            // If we only have simulation data, use its duration
            if frames.isEmpty, duration == 0, let lastFrame = simulationFrames.last {
                duration = lastFrame.timestamp
            }
            
            // Auto-start timer-based playback if no video player
            if player == nil {
                startSimulationPlayback()
            }
        }
    }
    
    /// Start timer-based playback for simulation-only recordings
    private func startSimulationPlayback() {
        guard player == nil && !simulationFrames.isEmpty else { return }
        
        isPlaying = true
        simulationStartTime = Date().addingTimeInterval(-currentTime)
        
        simulationTimer = Timer.scheduledTimer(withTimeInterval: 1.0/60.0, repeats: true) { [weak self] _ in
            Task { @MainActor in
                self?.updateSimulationTime()
            }
        }
    }
    
    /// Update time based on simulation timer
    private func updateSimulationTime() {
        // Don't update if paused
        guard isPlaying, let startTime = simulationStartTime else { return }
        
        var elapsed = Date().timeIntervalSince(startTime) * Double(playbackSpeed)
        
        // Loop when reaching the end
        if elapsed >= duration && duration > 0 {
            elapsed = elapsed.truncatingRemainder(dividingBy: duration)
            simulationStartTime = Date().addingTimeInterval(-elapsed / Double(playbackSpeed))
        }
        
        updateCurrentTime(elapsed)
    }
    
    /// Stop simulation timer
    private func stopSimulationTimer() {
        simulationTimer?.invalidate()
        simulationTimer = nil
    }
    
    private func updateCurrentTime(_ time: Double) {
        currentTime = time
        
        // Find matching frame
        if let index = frames.lastIndex(where: { $0.timestamp <= time }) {
            currentFrameIndex = index
            currentFrame = frames[index]
        }
        
        // Find matching simulation frame
        if let index = simulationFrames.lastIndex(where: { $0.timestamp <= time }) {
            currentSimulationFrame = simulationFrames[index]
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
        if player != nil {
            player?.play()
            player?.rate = playbackSpeed
        } else if !simulationFrames.isEmpty {
            startSimulationPlayback()
        }
    }
    
    func pause() {
        player?.pause()
        stopSimulationTimer()
        isPlaying = false
    }
    
    func seek(to time: Double) {
        // Pause playback when seeking manually
        let wasPlaying = isPlaying
        if wasPlaying {
            pause()
        }
        
        let cmTime = CMTime(seconds: time, preferredTimescale: 600)
        player?.seek(to: cmTime, toleranceBefore: .zero, toleranceAfter: .zero)
        
        // Update simulation start time for when we resume
        simulationStartTime = Date().addingTimeInterval(-time / Double(playbackSpeed))
        
        updateCurrentTime(time)
    }
    
    func stepForward() {
        // Use simulation frames if available, otherwise tracking frames
        let frameList: [(timestamp: Double, index: Int)] = 
            simulationFrames.isEmpty 
                ? frames.enumerated().map { ($0.element.timestamp, $0.offset) }
                : simulationFrames.enumerated().map { ($0.element.timestamp, $0.offset) }
        
        guard !frameList.isEmpty else { return }
        let newIndex = min(currentFrameIndex + 1, frameList.count - 1)
        seek(to: frameList[newIndex].timestamp)
    }
    
    func stepBackward() {
        // Use simulation frames if available, otherwise tracking frames
        let frameList: [(timestamp: Double, index: Int)] = 
            simulationFrames.isEmpty 
                ? frames.enumerated().map { ($0.element.timestamp, $0.offset) }
                : simulationFrames.enumerated().map { ($0.element.timestamp, $0.offset) }
        
        guard !frameList.isEmpty else { return }
        let newIndex = max(currentFrameIndex - 1, 0)
        seek(to: frameList[newIndex].timestamp)
    }
    
    func setSpeed(_ speed: Float) {
        playbackSpeed = speed
        if isPlaying {
            player?.rate = speed
        }
    }
    
    func cleanup() {
        stopSimulationTimer()
        if let observer = timeObserver, let player = player {
            player.removeTimeObserver(observer)
            timeObserver = nil
        }
        if let loopObserver = loopObserver {
            NotificationCenter.default.removeObserver(loopObserver)
            self.loopObserver = nil
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
