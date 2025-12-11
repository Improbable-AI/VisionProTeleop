//
//  FeedPlayerManager.swift
//  Tracking Viewer
//
//  Created on 12/2/25.
//

import SwiftUI
import AVKit
import Combine

/// Manages auto-play logic for the feed view
@MainActor
class FeedPlayerManager: ObservableObject {
    static let shared = FeedPlayerManager()
    
    @Published var currentPlayingID: String?
    @Published var player: AVPlayer?
    
    private var players: [String: AVPlayer] = [:]
    private var timeObservers: [String: Any] = [:]
    private var cancellables = Set<AnyCancellable>()
    
    // Configuration
    private let visibleThreshold: CGFloat = 0.5 // Item must be 50% visible to be considered
    private let topOffset: CGFloat = 100 // "Sweet spot" offset from top
    private var updateTask: Task<Void, Never>?
    
    private init() {
        // Setup audio session for playback
        try? AVAudioSession.sharedInstance().setCategory(.playback, mode: .default, options: [.mixWithOthers])
        try? AVAudioSession.sharedInstance().setActive(true)
    }
    
    /// Update the currently playing item based on visibility frames
    /// - Parameter visibleFrames: Map of ID to Frame (in scroll coordinate space)
    func updateVisibleItems(_ visibleFrames: [String: CGRect]) {
        // Debounce updates to prevent flickering
        updateTask?.cancel()
        updateTask = Task {
            // Wait for 150ms of stability
            try? await Task.sleep(nanoseconds: 150_000_000)
            if Task.isCancelled { return }
            
            await processVisibility(visibleFrames)
        }
    }
    
    private func processVisibility(_ visibleFrames: [String: CGRect]) {
        let screenHeight = UIScreen.main.bounds.height
        let safeAreaTop: CGFloat = 100 // Approximate safe area + nav bar
        
        // Find the item that is:
        // 1. Sufficiently visible
        // 2. Closest to the "sweet spot" (near the top, but not cut off)
        
        let bestItem = visibleFrames
            .filter { id, frame in
                // Calculate visibility ratio
                let intersection = frame.intersection(CGRect(x: 0, y: 0, width: UIScreen.main.bounds.width, height: screenHeight))
                let visibleRatio = (intersection.width * intersection.height) / (frame.width * frame.height)
                return visibleRatio >= visibleThreshold
            }
            .min { a, b in
                // Compare distance from top "sweet spot"
                let distA = abs(a.value.minY - safeAreaTop)
                let distB = abs(b.value.minY - safeAreaTop)
                return distA < distB
            }
        
        guard let best = bestItem else {
            // No valid item found
            return
        }
        
        // If the best item is already playing, do nothing
        if currentPlayingID == best.key {
            return
        }
        
        // Switch to the new item
        play(id: best.key)
    }
    
    /// Start playing a specific item
    func play(id: String) {
        // Stop current
        stopPlayback()
        
        currentPlayingID = id
        
        // In a real implementation, we would look up the URL for this ID
        // For now, the View will observe `currentPlayingID` and trigger the actual player setup
        // This manager acts as the coordinator
        
        dlog("▶️ FeedPlayerManager: Switched to \(id)")
    }
    
    /// Stop all playback
    func stopPlayback() {
        if let current = currentPlayingID {
            dlog("⏹️ FeedPlayerManager: Stopping \(current)")
        }
        currentPlayingID = nil
        player?.pause()
        player = nil
    }
    
    /// Register a player for an ID (called by the View when it starts playing)
    func registerPlayer(_ player: AVPlayer, for id: String) {
        // If we switched away while this was loading, pause it immediately
        guard currentPlayingID == id else {
            player.pause()
            return
        }
        
        self.player = player
        player.play()
        player.isMuted = true // Mute by default for feed
    }
}
