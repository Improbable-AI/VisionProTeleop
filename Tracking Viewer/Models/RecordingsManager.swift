//
//  RecordingsManager.swift
//  Tracking Viewer
//
//  Created on 11/29/25.
//

import Foundation
import SwiftUI
import AVFoundation
import Combine

/// Manages browsing and exporting recordings from iCloud Drive
@MainActor
class RecordingsManager: ObservableObject {
    static let shared = RecordingsManager()
    
    @Published var recordings: [Recording] = []
    @Published var isLoading: Bool = false
    @Published var errorMessage: String?
    @Published var selectedRecordings: Set<Recording> = []
    @Published var isExporting: Bool = false
    @Published var exportProgress: Double = 0
    
    private var thumbnailCache: [String: UIImage] = [:]
    
    private init() {}
    
    // MARK: - iCloud Drive Access
    
    /// Get the iCloud Drive container URL for VisionProTeleop recordings
    func getRecordingsURL() -> URL? {
        let fileManager = FileManager.default
        
        // Try iCloud container first
        if let containerURL = fileManager.url(forUbiquityContainerIdentifier: nil) {
            let recordingsURL = containerURL
                .appendingPathComponent("Documents")
                .appendingPathComponent("VisionProTeleop")
            return recordingsURL
        }
        
        // Fallback to local documents
        if let documentsURL = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first {
            return documentsURL.appendingPathComponent("Recordings")
        }
        
        return nil
    }
    
    /// Scan for recordings in the iCloud Drive folder
    func loadRecordings() async {
        isLoading = true
        errorMessage = nil
        
        defer { isLoading = false }
        
        guard let recordingsURL = getRecordingsURL() else {
            errorMessage = "Cannot access recordings folder"
            return
        }
        
        print("📂 Scanning for recordings at: \(recordingsURL.path)")
        
        // Ensure the directory exists
        let fileManager = FileManager.default
        if !fileManager.fileExists(atPath: recordingsURL.path) {
            do {
                try fileManager.createDirectory(at: recordingsURL, withIntermediateDirectories: true)
                print("📁 Created recordings directory")
            } catch {
                errorMessage = "Failed to create recordings folder: \(error.localizedDescription)"
                return
            }
        }
        
        // Start iCloud download if needed (for directories)
        do {
            try fileManager.startDownloadingUbiquitousItem(at: recordingsURL)
        } catch {
            print("⚠️ Could not start iCloud download: \(error)")
        }
        
        // Enumerate recording folders
        do {
            let contents = try fileManager.contentsOfDirectory(
                at: recordingsURL,
                includingPropertiesForKeys: [.isDirectoryKey, .creationDateKey],
                options: [.skipsHiddenFiles]
            )
            
            var loadedRecordings: [Recording] = []
            
            for folderURL in contents {
                // Check if it's a directory
                var isDirectory: ObjCBool = false
                guard fileManager.fileExists(atPath: folderURL.path, isDirectory: &isDirectory),
                      isDirectory.boolValue else {
                    continue
                }
                
                // Check for video.mp4 or metadata.json to validate it's a recording
                let videoURL = folderURL.appendingPathComponent("video.mp4")
                let metadataURL = folderURL.appendingPathComponent("metadata.json")
                let trackingURL = folderURL.appendingPathComponent("tracking.jsonl")
                
                let hasVideo = fileManager.fileExists(atPath: videoURL.path)
                let hasMetadata = fileManager.fileExists(atPath: metadataURL.path)
                
                guard hasVideo || hasMetadata else { continue }
                
                // Try to download files from iCloud if needed
                if hasVideo {
                    try? fileManager.startDownloadingUbiquitousItem(at: videoURL)
                }
                if hasMetadata {
                    try? fileManager.startDownloadingUbiquitousItem(at: metadataURL)
                }
                
                // Load metadata if available
                var metadata: RecordingMetadata?
                if hasMetadata {
                    // First try to download the metadata from iCloud
                    do {
                        try fileManager.startDownloadingUbiquitousItem(at: metadataURL)
                        // Wait briefly for small metadata file
                        try? await Task.sleep(nanoseconds: 100_000_000) // 0.1 sec
                    } catch {
                        // Not an iCloud file, continue
                    }
                    
                    do {
                        let data = try Data(contentsOf: metadataURL)
                        let decoder = JSONDecoder()
                        metadata = try decoder.decode(RecordingMetadata.self, from: data)
                    } catch {
                        print("⚠️ Failed to load metadata for \(folderURL.lastPathComponent): \(error)")
                    }
                }
                
                let recording = Recording(
                    id: folderURL.lastPathComponent,
                    folderURL: folderURL,
                    metadata: metadata,
                    videoURL: hasVideo ? videoURL : nil,
                    trackingURL: fileManager.fileExists(atPath: trackingURL.path) ? trackingURL : nil
                )
                
                loadedRecordings.append(recording)
            }
            
            // Sort by creation date, newest first
            recordings = loadedRecordings.sorted { $0.createdAt > $1.createdAt }
            
            print("✅ Found \(recordings.count) recordings")
            
        } catch {
            errorMessage = "Failed to scan recordings: \(error.localizedDescription)"
            print("❌ Error scanning recordings: \(error)")
        }
    }
    
    // MARK: - Thumbnails
    
    /// Generate a thumbnail for a recording's video
    func getThumbnail(for recording: Recording) async -> UIImage? {
        // Check cache first
        if let cached = thumbnailCache[recording.id] {
            return cached
        }
        
        guard let videoURL = recording.videoURL else {
            return nil
        }
        
        // Generate thumbnail from video
        let thumbnail = await generateThumbnail(from: videoURL)
        
        if let thumbnail = thumbnail {
            thumbnailCache[recording.id] = thumbnail
        }
        
        return thumbnail
    }
    
    private func generateThumbnail(from videoURL: URL) async -> UIImage? {
        // First, ensure the file is downloaded from iCloud
        let downloaded = await downloadFromiCloud(videoURL)
        guard downloaded else {
            print("⚠️ Video not yet downloaded from iCloud: \(videoURL.lastPathComponent)")
            return nil
        }
        
        return await withCheckedContinuation { continuation in
            DispatchQueue.global(qos: .userInitiated).async {
                let asset = AVAsset(url: videoURL)
                let imageGenerator = AVAssetImageGenerator(asset: asset)
                imageGenerator.appliesPreferredTrackTransform = true
                imageGenerator.maximumSize = CGSize(width: 300, height: 200)
                
                // Get thumbnail from 0.5 second into the video (or beginning)
                let time = CMTime(seconds: 0.5, preferredTimescale: 600)
                
                do {
                    let cgImage = try imageGenerator.copyCGImage(at: time, actualTime: nil)
                    let thumbnail = UIImage(cgImage: cgImage)
                    continuation.resume(returning: thumbnail)
                } catch {
                    print("⚠️ Failed to generate thumbnail: \(error)")
                    continuation.resume(returning: nil)
                }
            }
        }
    }
    
    /// Download a file from iCloud if needed
    func downloadFromiCloud(_ url: URL) async -> Bool {
        let fileManager = FileManager.default
        
        // Check if file exists locally
        if fileManager.fileExists(atPath: url.path) {
            // Check if it's a placeholder (not downloaded)
            do {
                let resourceValues = try url.resourceValues(forKeys: [.ubiquitousItemDownloadingStatusKey])
                if let status = resourceValues.ubiquitousItemDownloadingStatus {
                    if status == .current {
                        return true // Already downloaded
                    }
                }
            } catch {
                // Not an iCloud file, assume it's local
                return true
            }
        }
        
        // Start download
        do {
            try fileManager.startDownloadingUbiquitousItem(at: url)
        } catch {
            print("⚠️ Failed to start iCloud download: \(error)")
            return false
        }
        
        // Wait for download to complete (with timeout)
        let maxWait = 30.0 // 30 seconds timeout
        let startTime = Date()
        
        while Date().timeIntervalSince(startTime) < maxWait {
            do {
                let resourceValues = try url.resourceValues(forKeys: [.ubiquitousItemDownloadingStatusKey])
                if let status = resourceValues.ubiquitousItemDownloadingStatus, status == .current {
                    return true
                }
            } catch {
                // File might not be an iCloud file
                if fileManager.fileExists(atPath: url.path) {
                    return true
                }
            }
            
            // Wait a bit before checking again
            try? await Task.sleep(nanoseconds: 500_000_000) // 0.5 seconds
        }
        
        print("⚠️ iCloud download timed out for: \(url.lastPathComponent)")
        return false
    }
    
    // MARK: - Export
    
    /// Export selected recordings - opens share sheet for AirDrop/Files
    func exportRecordings(_ recordings: [Recording]) async -> [URL] {
        isExporting = true
        exportProgress = 0
        defer { isExporting = false }
        
        var exportURLs: [URL] = []
        let total = Double(recordings.count)
        
        for (index, recording) in recordings.enumerated() {
            exportProgress = Double(index) / total
            
            // Add the entire recording folder to export
            exportURLs.append(recording.folderURL)
        }
        
        exportProgress = 1.0
        return exportURLs
    }
    
    /// Delete a recording
    func deleteRecording(_ recording: Recording) async throws {
        let fileManager = FileManager.default
        try fileManager.removeItem(at: recording.folderURL)
        
        // Remove from list
        recordings.removeAll { $0.id == recording.id }
        selectedRecordings.remove(recording)
        thumbnailCache.removeValue(forKey: recording.id)
        
        print("🗑️ Deleted recording: \(recording.id)")
    }
    
    /// Toggle selection for a recording
    func toggleSelection(_ recording: Recording) {
        if selectedRecordings.contains(recording) {
            selectedRecordings.remove(recording)
        } else {
            selectedRecordings.insert(recording)
        }
    }
    
    /// Select all recordings
    func selectAll() {
        selectedRecordings = Set(recordings)
    }
    
    /// Deselect all recordings
    func deselectAll() {
        selectedRecordings.removeAll()
    }
}
