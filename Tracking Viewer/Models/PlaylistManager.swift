//
//  PlaylistManager.swift
//  Tracking Viewer
//
//  Created on 12/02/25.
//

import Foundation
import SwiftUI
import Combine

/// Manages playlists with persistent storage
@MainActor
class PlaylistManager: ObservableObject {
    static let shared = PlaylistManager()
    
    @Published var playlists: [Playlist] = []
    @Published var isLoading: Bool = false
    
    private let fileManager = FileManager.default
    private var playlistsURL: URL? {
        // Store playlists in the same iCloud container as recordings
        if let containerURL = fileManager.url(forUbiquityContainerIdentifier: nil) {
            return containerURL
                .appendingPathComponent("Documents")
                .appendingPathComponent("VisionProTeleop")
                .appendingPathComponent("playlists.json")
        }
        
        // Fallback to local documents
        if let documentsURL = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first {
            return documentsURL.appendingPathComponent("playlists.json")
        }
        
        return nil
    }
    
    private init() {
        loadPlaylists()
    }
    
    // MARK: - CRUD Operations
    
    /// Create a new playlist
    func createPlaylist(name: String, description: String = "", color: PlaylistColor = .blue, icon: PlaylistIcon = .folder) -> Playlist {
        let playlist = Playlist(
            name: name,
            description: description,
            color: color,
            icon: icon
        )
        playlists.append(playlist)
        savePlaylists()
        return playlist
    }
    
    /// Update an existing playlist
    func updatePlaylist(_ playlist: Playlist) {
        if let index = playlists.firstIndex(where: { $0.id == playlist.id }) {
            var updated = playlist
            updated.modifiedAt = Date()
            playlists[index] = updated
            savePlaylists()
        }
    }
    
    /// Delete a playlist
    func deletePlaylist(_ playlist: Playlist) {
        playlists.removeAll { $0.id == playlist.id }
        savePlaylists()
    }
    
    /// Rename a playlist
    func renamePlaylist(_ playlist: Playlist, to newName: String) {
        if let index = playlists.firstIndex(where: { $0.id == playlist.id }) {
            playlists[index].name = newName
            playlists[index].modifiedAt = Date()
            savePlaylists()
        }
    }
    
    // MARK: - Recording Management
    
    /// Add a recording to a playlist
    func addRecording(_ recording: Recording, to playlist: Playlist) {
        addRecording(recording.id, to: playlist)
    }
    
    /// Add a recording by ID to a playlist
    func addRecording(_ recordingID: String, to playlist: Playlist) {
        guard let index = playlists.firstIndex(where: { $0.id == playlist.id }) else { return }
        
        if !playlists[index].recordingIDs.contains(recordingID) {
            playlists[index].recordingIDs.append(recordingID)
            playlists[index].modifiedAt = Date()
            savePlaylists()
        }
    }
    
    /// Add multiple recordings to a playlist
    func addRecordings(_ recordings: [Recording], to playlist: Playlist) {
        guard let index = playlists.firstIndex(where: { $0.id == playlist.id }) else { return }
        
        var modified = false
        for recording in recordings {
            if !playlists[index].recordingIDs.contains(recording.id) {
                playlists[index].recordingIDs.append(recording.id)
                modified = true
            }
        }
        
        if modified {
            playlists[index].modifiedAt = Date()
            savePlaylists()
        }
    }
    
    /// Remove a recording from a playlist
    func removeRecording(_ recording: Recording, from playlist: Playlist) {
        removeRecording(recording.id, from: playlist)
    }
    
    /// Remove a recording by ID from a playlist
    func removeRecording(_ recordingID: String, from playlist: Playlist) {
        guard let index = playlists.firstIndex(where: { $0.id == playlist.id }) else { return }
        
        if let recordingIndex = playlists[index].recordingIDs.firstIndex(of: recordingID) {
            playlists[index].recordingIDs.remove(at: recordingIndex)
            playlists[index].modifiedAt = Date()
            savePlaylists()
        }
    }
    
    // Convenience methods for cloud storage recordings
    
    /// Add a recording by ID to a playlist (convenience method for cloud storage)
    func addRecordingByID(_ recordingID: String, to playlist: Playlist) {
        addRecording(recordingID, to: playlist)
    }
    
    /// Remove a recording by ID from a playlist (convenience method for cloud storage)
    func removeRecordingByID(_ recordingID: String, from playlist: Playlist) {
        removeRecording(recordingID, from: playlist)
    }
    
    /// Remove a recording from all playlists (used when deleting a recording)
    func removeRecordingFromAllPlaylists(_ recording: Recording) {
        var modified = false
        for index in playlists.indices {
            if let recordingIndex = playlists[index].recordingIDs.firstIndex(of: recording.id) {
                playlists[index].recordingIDs.remove(at: recordingIndex)
                playlists[index].modifiedAt = Date()
                modified = true
            }
        }
        if modified {
            savePlaylists()
        }
    }
    
    /// Get all playlists that contain a specific recording
    func playlists(containing recording: Recording) -> [Playlist] {
        playlists.filter { $0.contains(recording) }
    }
    
    /// Get recordings for a playlist (resolved from RecordingsManager)
    func recordings(for playlist: Playlist) -> [Recording] {
        let allRecordings = RecordingsManager.shared.recordings
        return playlist.recordingIDs.compactMap { id in
            allRecordings.first { $0.id == id }
        }
    }
    
    /// Reorder recordings within a playlist
    func moveRecording(in playlist: Playlist, from source: IndexSet, to destination: Int) {
        guard let index = playlists.firstIndex(where: { $0.id == playlist.id }) else { return }
        
        playlists[index].recordingIDs.move(fromOffsets: source, toOffset: destination)
        playlists[index].modifiedAt = Date()
        savePlaylists()
    }
    
    // MARK: - Persistence
    
    /// Load playlists from disk
    func loadPlaylists() {
        guard let url = playlistsURL else {
            dlog("⚠️ [PlaylistManager] No playlists URL available")
            return
        }
        
        // Try to download from iCloud if needed
        try? fileManager.startDownloadingUbiquitousItem(at: url)
        
        guard fileManager.fileExists(atPath: url.path) else {
            dlog("📂 [PlaylistManager] No playlists file found, starting fresh")
            return
        }
        
        do {
            let data = try Data(contentsOf: url)
            let decoder = JSONDecoder()
            decoder.dateDecodingStrategy = .iso8601
            playlists = try decoder.decode([Playlist].self, from: data)
            dlog("✅ [PlaylistManager] Loaded \(playlists.count) playlists")
        } catch {
            dlog("❌ [PlaylistManager] Failed to load playlists: \(error)")
        }
    }
    
    /// Save playlists to disk
    func savePlaylists() {
        guard let url = playlistsURL else {
            dlog("⚠️ [PlaylistManager] No playlists URL available")
            return
        }
        
        // Ensure directory exists
        let directory = url.deletingLastPathComponent()
        if !fileManager.fileExists(atPath: directory.path) {
            try? fileManager.createDirectory(at: directory, withIntermediateDirectories: true)
        }
        
        do {
            let encoder = JSONEncoder()
            encoder.dateEncodingStrategy = .iso8601
            encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
            let data = try encoder.encode(playlists)
            try data.write(to: url)
            dlog("✅ [PlaylistManager] Saved \(playlists.count) playlists")
        } catch {
            dlog("❌ [PlaylistManager] Failed to save playlists: \(error)")
        }
    }
    
    // MARK: - Export
    
    /// Get recording IDs for export
    func getRecordingIDs(for playlist: Playlist) -> [String] {
        playlist.recordingIDs
    }
}
