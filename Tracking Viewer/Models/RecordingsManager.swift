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
import SceneKit

// MARK: - Recording Source

enum RecordingSource: String, CaseIterable {
    case iCloudDrive = "iCloud Drive"
    case dropbox = "Dropbox"
    case googleDrive = "Google Drive"
    
    var displayName: String {
        rawValue
    }
    
    var icon: String {
        switch self {
        case .iCloudDrive: return "icloud"
        case .dropbox: return "shippingbox"
        case .googleDrive: return "externaldrive.fill"
        }
    }
    
    var color: Color {
        switch self {
        case .iCloudDrive: return .blue
        case .dropbox: return Color(red: 0, green: 0.4, blue: 1)
        case .googleDrive: return Color(red: 0.26, green: 0.52, blue: 0.96)
        }
    }
    
    /// Convert from CloudStorageProvider
    init(from provider: CloudStorageProvider) {
        switch provider {
        case .iCloudDrive: self = .iCloudDrive
        case .dropbox: self = .dropbox
        case .googleDrive: self = .googleDrive
        }
    }
    
    /// Convert to CloudStorageProvider
    var cloudStorageProvider: CloudStorageProvider {
        switch self {
        case .iCloudDrive: return .iCloudDrive
        case .dropbox: return .dropbox
        case .googleDrive: return .googleDrive
        }
    }
}

/// Manages browsing and exporting recordings from iCloud Drive or Dropbox
@MainActor
class RecordingsManager: ObservableObject {
    static let shared = RecordingsManager()
    
    @Published var recordings: [Recording] = []
    @Published var dropboxRecordings: [DropboxRecordingInfo] = []
    @Published var isLoading: Bool = false
    @Published var isLoadingMore: Bool = false
    @Published var hasMoreRecordings: Bool = true
    @Published var errorMessage: String?
    @Published var selectedRecordings: Set<Recording> = []
    @Published var selectedUnifiedRecordingIDs: Set<String> = []  // For cloud sources (by ID)
    @Published var isExporting: Bool = false
    @Published var exportProgress: Double = 0
    @Published var currentSource: RecordingSource = .iCloudDrive
    @Published var googleDriveRecordings: [GoogleDriveRecordingInfo] = []
    
    private var thumbnailCache: [String: UIImage] = [:]
    private var dropboxThumbnailCache: [String: UIImage] = [:]
    private var googleDriveThumbnailCache: [String: UIImage] = [:]
    private var cancellables = Set<AnyCancellable>()
    
    // Lazy metadata loading for iCloud
    private var metadataLoadingTasks: [String: Task<Void, Never>] = [:]
    @Published var loadedMetadata: [String: RecordingMetadata] = [:]
    
    /// Get unified recordings for the current source
    var unifiedRecordings: [AnyRecording] {
        switch currentSource {
        case .iCloudDrive:
            return recordings.map { AnyRecording($0) }
        case .dropbox:
            return dropboxRecordings.map { AnyRecording($0) }
        case .googleDrive:
            return googleDriveRecordings.map { AnyRecording($0) }
        }
    }
    
    /// Get count of recordings for current source
    var currentRecordingsCount: Int {
        switch currentSource {
        case .iCloudDrive: return recordings.count
        case .dropbox: return dropboxRecordings.count
        case .googleDrive: return googleDriveRecordings.count
        }
    }
    
    private init() {
        // Sync with CloudStorageManager's selected provider
        syncWithCloudStorageManager()
        
        // Listen for provider changes
        NotificationCenter.default.publisher(for: .cloudStorageProviderChanged)
            .receive(on: DispatchQueue.main)
            .sink { [weak self] notification in
                if let provider = notification.object as? CloudStorageProvider {
                    Task { @MainActor in
                        await self?.switchSource(to: RecordingSource(from: provider))
                    }
                }
            }
            .store(in: &cancellables)
        
        // Reload when app comes to foreground
        NotificationCenter.default.publisher(for: UIApplication.willEnterForegroundNotification)
            .receive(on: DispatchQueue.main)
            .sink { [weak self] _ in
                Task { @MainActor in
                    self?.syncWithCloudStorageManager()
                }
            }
            .store(in: &cancellables)
    }
    
    /// Sync current source with CloudStorageManager's selected provider
    func syncWithCloudStorageManager() {
        let provider = CloudStorageManager.shared.selectedProvider
        currentSource = RecordingSource(from: provider)
    }
    
    // MARK: - Source Switching
    
    /// Switch recording source and reload
    func switchSource(to source: RecordingSource) async {
        currentSource = source
        await loadRecordings()
    }
    
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
    
    /// Scan for recordings based on current source
    func loadRecordings() async {
        isLoading = true
        errorMessage = nil
        
        defer { isLoading = false }
        
        switch currentSource {
        case .iCloudDrive:
            await loadiCloudRecordings()
        case .dropbox:
            await loadDropboxRecordings()
        case .googleDrive:
            await loadGoogleDriveRecordings()
        }
    }
    
    // MARK: - Google Drive Recordings
    
    private func loadGoogleDriveRecordings() async {
        guard GoogleDriveManager.shared.isAuthenticated else {
            errorMessage = "Not signed in to Google Drive"
            googleDriveRecordings = []
            return
        }
        
        dlog("📂 Loading recordings from Google Drive...")
        
        let fetchedRecordings = await GoogleDriveManager.shared.listRecordings()
        googleDriveRecordings = fetchedRecordings.sorted { ($0.modifiedDate ?? .distantPast) > ($1.modifiedDate ?? .distantPast) }
        hasMoreRecordings = GoogleDriveManager.shared.hasMoreRecordings
        
        if googleDriveRecordings.isEmpty {
            dlog("📬 No recordings found in Google Drive")
        } else {
            dlog("✅ Found \(googleDriveRecordings.count) recordings in Google Drive")
        }
    }
    
    /// Load more Google Drive recordings (pagination)
    func loadMoreGoogleDriveRecordings() async {
        guard !isLoadingMore, GoogleDriveManager.shared.hasMoreRecordings else { return }
        
        isLoadingMore = true
        defer { isLoadingMore = false }
        
        let moreRecordings = await GoogleDriveManager.shared.loadMoreRecordings()
        let sortedMore = moreRecordings.sorted { ($0.modifiedDate ?? .distantPast) > ($1.modifiedDate ?? .distantPast) }
        googleDriveRecordings.append(contentsOf: sortedMore)
        hasMoreRecordings = GoogleDriveManager.shared.hasMoreRecordings
        
        dlog("✅ Loaded \(moreRecordings.count) more Google Drive recordings (total: \(googleDriveRecordings.count))")
    }
    
    /// Download a Google Drive recording video to local cache for viewing
    func downloadGoogleDriveVideo(for recording: GoogleDriveRecordingInfo) async -> URL? {
        guard let videoFileId = recording.videoFileId else {
            return nil
        }
        
        let cacheDir = FileManager.default.temporaryDirectory.appendingPathComponent("GoogleDriveCache")
        
        // Create cache directory if needed
        try? FileManager.default.createDirectory(at: cacheDir, withIntermediateDirectories: true)
        
        let localURL = cacheDir.appendingPathComponent("\(recording.id)_video.mp4")
        
        // Check if already cached
        if FileManager.default.fileExists(atPath: localURL.path) {
            return localURL
        }
        
        // Download from Google Drive
        let success = await GoogleDriveManager.shared.downloadFile(fileId: videoFileId, to: localURL)
        return success ? localURL : nil
    }
    
    /// Download a Google Drive recording tracking data to local cache
    func downloadGoogleDriveTrackingData(for recording: GoogleDriveRecordingInfo) async -> URL? {
        guard let trackingFileId = recording.trackingDataFileId else {
            return nil
        }
        
        let cacheDir = FileManager.default.temporaryDirectory.appendingPathComponent("GoogleDriveCache")
        
        // Create cache directory if needed
        try? FileManager.default.createDirectory(at: cacheDir, withIntermediateDirectories: true)
        
        let localURL = cacheDir.appendingPathComponent("\(recording.id)_tracking.jsonl")
        
        // Check if already cached
        if FileManager.default.fileExists(atPath: localURL.path) {
            return localURL
        }
        
        // Download from Google Drive
        let success = await GoogleDriveManager.shared.downloadFile(fileId: trackingFileId, to: localURL)
        return success ? localURL : nil
    }
    
    /// Download a Google Drive recording metadata to local cache
    func downloadGoogleDriveMetadata(for recording: GoogleDriveRecordingInfo) async -> URL? {
        guard let metadataFileId = recording.metadataFileId else {
            return nil
        }
        
        let cacheDir = FileManager.default.temporaryDirectory.appendingPathComponent("GoogleDriveCache")
        
        // Create cache directory if needed
        try? FileManager.default.createDirectory(at: cacheDir, withIntermediateDirectories: true)
        
        let localURL = cacheDir.appendingPathComponent("\(recording.id)_metadata.json")
        
        // Check if already cached
        if FileManager.default.fileExists(atPath: localURL.path) {
            return localURL
        }
        
        // Download from Google Drive
        let success = await GoogleDriveManager.shared.downloadFile(fileId: metadataFileId, to: localURL)
        return success ? localURL : nil
    }
    
    /// Download a Google Drive recording simulation data to local cache
    func downloadGoogleDriveSimulationData(for recording: GoogleDriveRecordingInfo) async -> URL? {
        guard let simulationFileId = recording.simulationDataFileId else {
            return nil
        }
        
        let cacheDir = FileManager.default.temporaryDirectory.appendingPathComponent("GoogleDriveCache")
        try? FileManager.default.createDirectory(at: cacheDir, withIntermediateDirectories: true)
        
        let localURL = cacheDir.appendingPathComponent("\(recording.id)_mjdata.jsonl")
        
        if FileManager.default.fileExists(atPath: localURL.path) {
            return localURL
        }
        
        let success = await GoogleDriveManager.shared.downloadFile(fileId: simulationFileId, to: localURL)
        return success ? localURL : nil
    }
    
    /// Download a Google Drive recording USDZ to local cache
    func downloadGoogleDriveUSDZ(for recording: GoogleDriveRecordingInfo) async -> URL? {
        guard let usdzFileId = recording.usdzFileId else {
            return nil
        }
        
        let cacheDir = FileManager.default.temporaryDirectory.appendingPathComponent("GoogleDriveCache")
        try? FileManager.default.createDirectory(at: cacheDir, withIntermediateDirectories: true)
        
        let localURL = cacheDir.appendingPathComponent("\(recording.id)_scene.usdz")
        
        if FileManager.default.fileExists(atPath: localURL.path) {
            return localURL
        }
        
        let success = await GoogleDriveManager.shared.downloadFile(fileId: usdzFileId, to: localURL)
        return success ? localURL : nil
    }
    
    /// Get thumbnail for Google Drive recording
    func getGoogleDriveThumbnail(for recording: GoogleDriveRecordingInfo) async -> UIImage? {
        // Check cache first
        if let cached = googleDriveThumbnailCache[recording.id] {
            return cached
        }
        
        // Try video thumbnail first
        if recording.hasVideo {
            if let videoURL = await downloadGoogleDriveVideo(for: recording) {
                // Generate thumbnail from video
                let asset = AVAsset(url: videoURL)
                let imageGenerator = AVAssetImageGenerator(asset: asset)
                imageGenerator.appliesPreferredTrackTransform = true
                imageGenerator.maximumSize = CGSize(width: 320, height: 180)
                
                do {
                    let cgImage = try imageGenerator.copyCGImage(at: .zero, actualTime: nil)
                    let thumbnail = UIImage(cgImage: cgImage)
                    googleDriveThumbnailCache[recording.id] = thumbnail
                    return thumbnail
                } catch {
                    dlog("Failed to generate video thumbnail: \(error)")
                }
            }
        }
        
        // Fall back to USDZ thumbnail for simulation datasets
        if recording.hasUSDZ {
            if let usdzURL = await downloadGoogleDriveUSDZ(for: recording) {
                if let thumbnail = await generateUsdzThumbnail(from: usdzURL) {
                    googleDriveThumbnailCache[recording.id] = thumbnail
                    return thumbnail
                }
            }
        }
        
        return nil
    }
    
    // MARK: - Dropbox Recordings
    
    private func loadDropboxRecordings() async {
        guard DropboxManager.shared.isAuthenticated else {
            errorMessage = "Not signed in to Dropbox"
            dropboxRecordings = []
            return
        }
        
        dlog("📂 Loading recordings from Dropbox...")
        
        let fetchedRecordings = await DropboxManager.shared.listRecordings()
        dropboxRecordings = fetchedRecordings.sorted { ($0.modifiedDate ?? .distantPast) > ($1.modifiedDate ?? .distantPast) }
        hasMoreRecordings = DropboxManager.shared.hasMoreRecordings
        
        if dropboxRecordings.isEmpty {
            dlog("📬 No recordings found in Dropbox")
        } else {
            dlog("✅ Found \(dropboxRecordings.count) recordings in Dropbox")
        }
    }
    
    /// Load more Dropbox recordings (pagination)
    func loadMoreDropboxRecordings() async {
        guard !isLoadingMore, DropboxManager.shared.hasMoreRecordings else { return }
        
        isLoadingMore = true
        defer { isLoadingMore = false }
        
        let moreRecordings = await DropboxManager.shared.loadMoreRecordings()
        let sortedMore = moreRecordings.sorted { ($0.modifiedDate ?? .distantPast) > ($1.modifiedDate ?? .distantPast) }
        dropboxRecordings.append(contentsOf: sortedMore)
        hasMoreRecordings = DropboxManager.shared.hasMoreRecordings
        
        dlog("✅ Loaded \(moreRecordings.count) more Dropbox recordings (total: \(dropboxRecordings.count))")
    }
    
    /// Download a Dropbox recording video to local cache for viewing
    func downloadDropboxVideo(for recording: DropboxRecordingInfo) async -> URL? {
        let videoPath = "\(recording.path)/video.mp4"
        let cacheDir = FileManager.default.temporaryDirectory.appendingPathComponent("DropboxCache")
        
        // Create cache directory if needed
        try? FileManager.default.createDirectory(at: cacheDir, withIntermediateDirectories: true)
        
        let localURL = cacheDir.appendingPathComponent("\(recording.id)_video.mp4")
        
        // Check if already cached
        if FileManager.default.fileExists(atPath: localURL.path) {
            return localURL
        }
        
        // Download from Dropbox
        let success = await DropboxManager.shared.downloadFile(path: videoPath, to: localURL)
        return success ? localURL : nil
    }
    
    /// Download Dropbox tracking data to local cache for viewing
    func downloadDropboxTrackingData(for recording: DropboxRecordingInfo) async -> URL? {
        let trackingPath = "\(recording.path)/tracking.jsonl"
        let cacheDir = FileManager.default.temporaryDirectory.appendingPathComponent("DropboxCache")
        
        // Create cache directory if needed
        try? FileManager.default.createDirectory(at: cacheDir, withIntermediateDirectories: true)
        
        let localURL = cacheDir.appendingPathComponent("\(recording.id)_tracking.jsonl")
        
        // Check if already cached
        if FileManager.default.fileExists(atPath: localURL.path) {
            return localURL
        }
        
        // Download from Dropbox
        let success = await DropboxManager.shared.downloadFile(path: trackingPath, to: localURL)
        return success ? localURL : nil
    }
    
    /// Download Dropbox metadata to local cache
    func downloadDropboxMetadata(for recording: DropboxRecordingInfo) async -> URL? {
        let metadataPath = "\(recording.path)/metadata.json"
        let cacheDir = FileManager.default.temporaryDirectory.appendingPathComponent("DropboxCache")
        
        // Create cache directory if needed
        try? FileManager.default.createDirectory(at: cacheDir, withIntermediateDirectories: true)
        
        let localURL = cacheDir.appendingPathComponent("\(recording.id)_metadata.json")
        
        // Check if already cached
        if FileManager.default.fileExists(atPath: localURL.path) {
            return localURL
        }
        
        // Download from Dropbox
        let success = await DropboxManager.shared.downloadFile(path: metadataPath, to: localURL)
        return success ? localURL : nil
    }
    
    /// Download Dropbox simulation data to local cache
    func downloadDropboxSimulationData(for recording: DropboxRecordingInfo) async -> URL? {
        let simulationPath = "\(recording.path)/mjdata.jsonl"
        let cacheDir = FileManager.default.temporaryDirectory.appendingPathComponent("DropboxCache")
        try? FileManager.default.createDirectory(at: cacheDir, withIntermediateDirectories: true)
        
        let localURL = cacheDir.appendingPathComponent("\(recording.id)_mjdata.jsonl")
        
        if FileManager.default.fileExists(atPath: localURL.path) {
            return localURL
        }
        
        let success = await DropboxManager.shared.downloadFile(path: simulationPath, to: localURL)
        return success ? localURL : nil
    }
    
    /// Download Dropbox USDZ to local cache
    func downloadDropboxUSDZ(for recording: DropboxRecordingInfo) async -> URL? {
        let usdzPath = "\(recording.path)/scene.usdz"
        let cacheDir = FileManager.default.temporaryDirectory.appendingPathComponent("DropboxCache")
        try? FileManager.default.createDirectory(at: cacheDir, withIntermediateDirectories: true)
        
        let localURL = cacheDir.appendingPathComponent("\(recording.id)_scene.usdz")
        
        if FileManager.default.fileExists(atPath: localURL.path) {
            return localURL
        }
        
        let success = await DropboxManager.shared.downloadFile(path: usdzPath, to: localURL)
        return success ? localURL : nil
    }
    
    /// Get thumbnail for Dropbox recording
    func getDropboxThumbnail(for recording: DropboxRecordingInfo) async -> UIImage? {
        // Check cache first
        if let cached = dropboxThumbnailCache[recording.id] {
            return cached
        }
        
        // Try video thumbnail first
        if recording.hasVideo {
            if let videoURL = await downloadDropboxVideo(for: recording) {
                if let thumbnail = await generateThumbnail(from: videoURL) {
                    dropboxThumbnailCache[recording.id] = thumbnail
                    return thumbnail
                }
            }
        }
        
        // Fall back to USDZ thumbnail for simulation datasets
        if recording.hasUSDZ {
            if let usdzURL = await downloadDropboxUSDZ(for: recording) {
                if let thumbnail = await generateUsdzThumbnail(from: usdzURL) {
                    dropboxThumbnailCache[recording.id] = thumbnail
                    return thumbnail
                }
            }
        }
        
        return nil
    }
    
    // MARK: - iCloud Drive Recordings
    
    private func loadiCloudRecordings() async {
        
        guard let recordingsURL = getRecordingsURL() else {
            errorMessage = "Cannot access recordings folder"
            return
        }
        
        dlog("📂 Scanning for recordings at: \(recordingsURL.path)")
        
        // Ensure the directory exists
        let fileManager = FileManager.default
        if !fileManager.fileExists(atPath: recordingsURL.path) {
            do {
                try fileManager.createDirectory(at: recordingsURL, withIntermediateDirectories: true)
                dlog("📁 Created recordings directory")
            } catch {
                errorMessage = "Failed to create recordings folder: \(error.localizedDescription)"
                return
            }
        }
        
        // Start iCloud download if needed (for directories)
        do {
            try fileManager.startDownloadingUbiquitousItem(at: recordingsURL)
        } catch {
            dlog("⚠️ Could not start iCloud download: \(error)")
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
                
                // Try to download files from iCloud if needed (but don't wait)
                if hasVideo {
                    try? fileManager.startDownloadingUbiquitousItem(at: videoURL)
                }
                if hasMetadata {
                    try? fileManager.startDownloadingUbiquitousItem(at: metadataURL)
                }
                
                // Create recording WITHOUT metadata - will be loaded lazily when visible
                let recording = Recording(
                    id: folderURL.lastPathComponent,
                    folderURL: folderURL,
                    metadata: nil,  // Lazy load metadata
                    videoURL: hasVideo ? videoURL : nil,
                    trackingURL: fileManager.fileExists(atPath: trackingURL.path) ? trackingURL : nil
                )
                
                loadedRecordings.append(recording)
            }
            
            // Sort by creation date (from folder attributes), newest first
            recordings = loadedRecordings.sorted { $0.createdAt > $1.createdAt }
            
            dlog("✅ Found \(recordings.count) recordings (metadata will load lazily)")
            
        } catch {
            errorMessage = "Failed to scan recordings: \(error.localizedDescription)"
            dlog("❌ Error scanning recordings: \(error)")
        }
    }
    
    // MARK: - Lazy Metadata Loading for iCloud
    
    /// Load metadata for a recording when it becomes visible
    func loadMetadataIfNeeded(for recording: Recording) {
        let recordingId = recording.id
        
        // Already loaded or loading
        if loadedMetadata[recordingId] != nil || metadataLoadingTasks[recordingId] != nil {
            return
        }
        
        let metadataURL = recording.folderURL.appendingPathComponent("metadata.json")
        let fileManager = FileManager.default
        
        guard fileManager.fileExists(atPath: metadataURL.path) else {
            return
        }
        
        // Start async load
        metadataLoadingTasks[recordingId] = Task {
            do {
                // Try to download from iCloud
                try? fileManager.startDownloadingUbiquitousItem(at: metadataURL)
                
                // Wait briefly for small metadata file
                try await Task.sleep(nanoseconds: 100_000_000) // 0.1 sec
                
                let data = try Data(contentsOf: metadataURL)
                let decoder = JSONDecoder()
                let metadata = try decoder.decode(RecordingMetadata.self, from: data)
                
                await MainActor.run {
                    self.loadedMetadata[recordingId] = metadata
                    
                    // Update recording in array with loaded metadata
                    if let index = self.recordings.firstIndex(where: { $0.id == recordingId }) {
                        self.recordings[index] = Recording(
                            id: recordingId,
                            folderURL: recording.folderURL,
                            metadata: metadata,
                            videoURL: recording.videoURL,
                            trackingURL: recording.trackingURL
                        )
                    }
                }
            } catch {
                dlog("⚠️ Failed to load metadata for \(recordingId): \(error)")
            }
            
            await MainActor.run {
                self.metadataLoadingTasks.removeValue(forKey: recordingId)
            }
        }
    }
    
    /// Get metadata for a recording (from cache or recording itself)
    func getMetadata(for recording: Recording) -> RecordingMetadata? {
        return loadedMetadata[recording.id] ?? recording.metadata
    }
    
    /// Preload metadata for visible recordings
    func preloadMetadata(for recordings: [Recording]) {
        for recording in recordings {
            loadMetadataIfNeeded(for: recording)
        }
    }
    
    // MARK: - Thumbnails
    
    /// Generate a thumbnail for a recording's video
    func getThumbnail(for recording: Recording) async -> UIImage? {
        // Check cache first
        if let cached = thumbnailCache[recording.id] {
            return cached
        }
        
        // Try video thumbnail first
        if let videoURL = recording.videoURL {
            if let thumbnail = await generateThumbnail(from: videoURL) {
                thumbnailCache[recording.id] = thumbnail
                return thumbnail
            }
        }
        
        // Fall back to USDZ thumbnail for simulation datasets
        if let usdzURL = recording.usdzURL {
            let downloaded = await downloadFromiCloud(usdzURL)
            if downloaded {
                if let thumbnail = await generateUsdzThumbnail(from: usdzURL) {
                    thumbnailCache[recording.id] = thumbnail
                    return thumbnail
                }
            }
        }
        
        return nil
    }
    
    /// Get thumbnail for any recording type
    func getThumbnail(for recording: AnyRecording) async -> UIImage? {
        switch recording.source {
        case .iCloudDrive:
            if let rec = recording.iCloudRecording {
                return await getThumbnail(for: rec)
            }
        case .dropbox:
            if let rec = recording.dropboxRecording {
                return await getDropboxThumbnail(for: rec)
            }
        case .googleDrive:
            if let rec = recording.googleDriveRecording {
                return await getGoogleDriveThumbnail(for: rec)
            }
        }
        return nil
    }
    
    private func generateThumbnail(from videoURL: URL) async -> UIImage? {
        // First, ensure the file is downloaded from iCloud
        let downloaded = await downloadFromiCloud(videoURL)
        guard downloaded else {
            dlog("⚠️ Video not yet downloaded from iCloud: \(videoURL.lastPathComponent)")
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
                    dlog("⚠️ Failed to generate thumbnail: \(error)")
                    continuation.resume(returning: nil)
                }
            }
        }
    }
    
    /// Generate a thumbnail from a USDZ file using SceneKit
    func generateUsdzThumbnail(from usdzURL: URL) async -> UIImage? {
        // SceneKit rendering needs to happen on the main thread
        return await MainActor.run {
            do {
                // Load the USDZ scene
                let scene = try SCNScene(url: usdzURL, options: [
                    .checkConsistency: false
                ])
                
                // Create a SceneKit view for rendering
                let sceneView = SCNView(frame: CGRect(x: 0, y: 0, width: 320, height: 180))
                sceneView.scene = scene
                sceneView.backgroundColor = UIColor.systemGray6
                sceneView.antialiasingMode = .multisampling4X
                
                // Auto-position camera to fit the scene
                sceneView.autoenablesDefaultLighting = true
                sceneView.allowsCameraControl = false
                
                // Calculate bounding box and position camera
                let (minVec, maxVec) = scene.rootNode.boundingBox
                let center = SCNVector3(
                    (minVec.x + maxVec.x) / 2,
                    (minVec.y + maxVec.y) / 2,
                    (minVec.z + maxVec.z) / 2
                )
                
                let size = SCNVector3(
                    maxVec.x - minVec.x,
                    maxVec.y - minVec.y,
                    maxVec.z - minVec.z
                )
                
                let maxDimension = max(size.x, max(size.y, size.z))
                let cameraDistance = maxDimension * 2.0
                
                // Create and position camera
                let cameraNode = SCNNode()
                cameraNode.camera = SCNCamera()
                cameraNode.camera?.automaticallyAdjustsZRange = true
                cameraNode.position = SCNVector3(
                    center.x + cameraDistance * 0.5,
                    center.y + cameraDistance * 0.3,
                    center.z + cameraDistance
                )
                cameraNode.look(at: center)
                scene.rootNode.addChildNode(cameraNode)
                sceneView.pointOfView = cameraNode
                
                // Add ambient light for better visibility
                let ambientLight = SCNNode()
                ambientLight.light = SCNLight()
                ambientLight.light?.type = .ambient
                ambientLight.light?.intensity = 500
                scene.rootNode.addChildNode(ambientLight)
                
                // Render the scene to an image
                let snapshot = sceneView.snapshot()
                return snapshot
                
            } catch {
                dlog("⚠️ Failed to generate USDZ thumbnail: \(error)")
                return nil
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
            dlog("⚠️ Failed to start iCloud download: \(error)")
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
        
        dlog("⚠️ iCloud download timed out for: \(url.lastPathComponent)")
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
        
        dlog("🗑️ Deleted recording: \(recording.id)")
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
        selectedUnifiedRecordingIDs.removeAll()
    }
    
    // MARK: - Unified Selection (for cloud sources)
    
    /// Toggle selection for a unified recording by ID
    func toggleUnifiedSelection(_ recordingID: String) {
        if selectedUnifiedRecordingIDs.contains(recordingID) {
            selectedUnifiedRecordingIDs.remove(recordingID)
        } else {
            selectedUnifiedRecordingIDs.insert(recordingID)
        }
    }
    
    /// Check if a unified recording is selected
    func isUnifiedSelected(_ recordingID: String) -> Bool {
        selectedUnifiedRecordingIDs.contains(recordingID)
    }
    
    /// Select all unified recordings for current source
    func selectAllUnified() {
        switch currentSource {
        case .iCloudDrive:
            selectedRecordings = Set(recordings)
        case .dropbox:
            selectedUnifiedRecordingIDs = Set(dropboxRecordings.map { $0.id })
        case .googleDrive:
            selectedUnifiedRecordingIDs = Set(googleDriveRecordings.map { $0.id })
        }
    }
    
    /// Get selected unified recordings for current source
    var selectedUnifiedRecordings: [AnyRecording] {
        switch currentSource {
        case .iCloudDrive:
            return selectedRecordings.map { AnyRecording($0) }
        case .dropbox:
            return dropboxRecordings
                .filter { selectedUnifiedRecordingIDs.contains($0.id) }
                .map { AnyRecording($0) }
        case .googleDrive:
            return googleDriveRecordings
                .filter { selectedUnifiedRecordingIDs.contains($0.id) }
                .map { AnyRecording($0) }
        }
    }
    
    /// Get count of selected recordings for current source
    var selectedCount: Int {
        switch currentSource {
        case .iCloudDrive:
            return selectedRecordings.count
        case .dropbox, .googleDrive:
            return selectedUnifiedRecordingIDs.count
        }
    }
    
    /// Check if all recordings are selected for current source
    var allSelected: Bool {
        switch currentSource {
        case .iCloudDrive:
            return selectedRecordings.count == recordings.count && !recordings.isEmpty
        case .dropbox:
            return selectedUnifiedRecordingIDs.count == dropboxRecordings.count && !dropboxRecordings.isEmpty
        case .googleDrive:
            return selectedUnifiedRecordingIDs.count == googleDriveRecordings.count && !googleDriveRecordings.isEmpty
        }
    }
    
    // MARK: - Cloud Deletion
    
    /// Delete a unified recording from cloud storage
    func deleteUnifiedRecording(_ recording: AnyRecording) async {
        switch recording.source {
        case .iCloudDrive:
            if let rec = recording.iCloudRecording {
                try? await deleteRecording(rec)
            }
        case .dropbox:
            if let rec = recording.dropboxRecording {
                await deleteDropboxRecording(rec)
            }
        case .googleDrive:
            if let rec = recording.googleDriveRecording {
                await deleteGoogleDriveRecording(rec)
            }
        }
    }
    
    /// Delete selected unified recordings
    func deleteSelectedUnifiedRecordings() async {
        switch currentSource {
        case .iCloudDrive:
            for recording in selectedRecordings {
                try? await deleteRecording(recording)
            }
            selectedRecordings.removeAll()
        case .dropbox:
            for id in selectedUnifiedRecordingIDs {
                if let rec = dropboxRecordings.first(where: { $0.id == id }) {
                    await deleteDropboxRecording(rec)
                }
            }
            selectedUnifiedRecordingIDs.removeAll()
        case .googleDrive:
            for id in selectedUnifiedRecordingIDs {
                if let rec = googleDriveRecordings.first(where: { $0.id == id }) {
                    await deleteGoogleDriveRecording(rec)
                }
            }
            selectedUnifiedRecordingIDs.removeAll()
        }
    }
    
    /// Delete a Dropbox recording
    private func deleteDropboxRecording(_ recording: DropboxRecordingInfo) async {
        do {
            try await DropboxManager.shared.deleteFolder(path: recording.path)
            dropboxRecordings.removeAll { $0.id == recording.id }
            dlog("🗑️ Deleted Dropbox recording: \(recording.name)")
        } catch {
            dlog("❌ Failed to delete Dropbox recording: \(error)")
            errorMessage = "Failed to delete: \(error.localizedDescription)"
        }
    }
    
    /// Delete a Google Drive recording
    private func deleteGoogleDriveRecording(_ recording: GoogleDriveRecordingInfo) async {
        do {
            try await CloudStorageManager.shared.deleteGoogleDriveFolder(folderId: recording.id)
            googleDriveRecordings.removeAll { $0.id == recording.id }
            dlog("🗑️ Deleted Google Drive recording: \(recording.name)")
        } catch {
            dlog("❌ Failed to delete Google Drive recording: \(error)")
            errorMessage = "Failed to delete: \(error.localizedDescription)"
        }
    }
    // MARK: - Public Sharing
    
    /// Share a recording publicly via CloudKit
    /// - Parameter recording: The recording to share
    /// - Returns: Success status
    func shareRecordingPublicly(_ recording: AnyRecording) async -> Bool {
        var cloudURL: String?
        var provider: CloudStorageProvider?
        
        // 1. Get or create shared link based on source
        switch recording.source {
        case .iCloudDrive:
            // Not supported yet for iCloud Drive (requires upload)
            errorMessage = "Sharing from iCloud Drive is not supported yet. Please move to Dropbox or Google Drive."
            return false
            
        case .dropbox:
            if let dropboxRec = recording.dropboxRecording {
                provider = .dropbox
                // Create shared link for the folder
                cloudURL = await DropboxManager.shared.createSharedLink(path: dropboxRec.path)
            }
            
        case .googleDrive:
            if let googleRec = recording.googleDriveRecording {
                provider = .googleDrive
                // Create shared link for the folder (recording ID is the folder ID)
                cloudURL = await GoogleDriveManager.shared.createSharedLink(fileId: googleRec.id)
            }
        }
        
        guard let url = cloudURL, let storageProvider = provider else {
            if errorMessage == nil {
                errorMessage = "Failed to generate shared link."
            }
            return false
        }
        
        // 2. Generate and save thumbnail to temp file
        var thumbnailAssetURL: URL?
        if let thumbnail = await getThumbnail(for: recording),
           let data = thumbnail.jpegData(compressionQuality: 0.8) {
            let tempURL = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString + ".jpg")
            do {
                try data.write(to: tempURL)
                thumbnailAssetURL = tempURL
            } catch {
                dlog("⚠️ Failed to save temp thumbnail: \(error)")
            }
        }
        
        // 3. Register with CloudKit
        let success = await CloudKitManager.shared.makeRecordingPublic(
            recordingId: recording.id,
            title: recording.displayName,
            description: nil, // Could add a prompt for description later
            cloudURL: url,
            thumbnailAssetURL: thumbnailAssetURL,
            provider: storageProvider
        )
        
        // Cleanup temp file
        if let tempURL = thumbnailAssetURL {
            try? FileManager.default.removeItem(at: tempURL)
        }
        
        return success
    }
    
    /// Unpublish a recording (remove from public access)
    func unpublishRecording(_ recording: AnyRecording) async -> Bool {
        return await CloudKitManager.shared.makeRecordingPrivate(recordingId: recording.id)
    }
}
