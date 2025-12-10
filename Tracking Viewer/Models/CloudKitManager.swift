//
//  CloudKitManager.swift
//  Tracking Viewer
//
//  Created on 12/2/25.
//

import Foundation
import CloudKit
import SwiftUI
import Combine

/// Manages CloudKit public database for sharing recordings
@MainActor
class CloudKitManager: ObservableObject {
    static let shared = CloudKitManager()
    
    // CloudKit container and database
    private let container: CKContainer
    private let publicDatabase: CKDatabase
    
    // Record type for public recordings
    // Matches "publicRecording" from CloudKit Dashboard
    private let recordType = "publicRecording"
    
    // Pagination
    private let pageSize = 20
    private var queryCursor: CKQueryOperation.Cursor?
    
    // Published properties
    @Published var publicRecordings: [PublicRecording] = []
    @Published var isLoading = false
    @Published var isLoadingMore = false
    @Published var hasMoreRecordings = true
    @Published var errorMessage: String?
    
    private init() {
        // Use the container that matches your entitlements
        // This should match the iCloud container in your entitlements file
        container = CKContainer(identifier: "iCloud.com.younghyopark.VisionProTeleop")
        publicDatabase = container.publicCloudDatabase
    }
    
    // MARK: - Public Recording Operations
    
    /// Make a recording public by creating a CloudKit record
    /// - Parameters:
    ///   - recordingId: Unique ID of the recording
    ///   - title: Display name of the recording
    ///   - description: Optional description
    ///   - cloudURL: Shared link to the recording (Dropbox or Google Drive)
    ///   - thumbnailURL: Optional thumbnail URL
    ///   - provider: Cloud storage provider (dropbox or googleDrive)
    /// - Returns: Success status
    /// Make a recording public by creating a CloudKit record
    /// - Parameters:
    ///   - recordingId: Unique ID of the recording
    ///   - title: Display name of the recording
    ///   - description: Optional description
    ///   - cloudURL: Shared link to the recording (Dropbox or Google Drive)
    ///   - thumbnailURL: Optional thumbnail URL (legacy)
    ///   - thumbnailAssetURL: Local URL to thumbnail image to upload as asset
    ///   - provider: Cloud storage provider (dropbox or googleDrive)
    /// - Returns: Success status
    func makeRecordingPublic(
        recordingId: String,
        title: String,
        description: String? = nil,
        cloudURL: String,
        thumbnailURL: String? = nil,
        thumbnailAssetURL: URL? = nil,
        provider: CloudStorageProvider
    ) async -> Bool {
        // Create a record with the recording ID as the record name for easy lookup
        let recordID = CKRecord.ID(recordName: recordingId)
        let record = CKRecord(recordType: recordType, recordID: recordID)
        
        // Set fields - MUST MATCH CLOUDKIT DASHBOARD EXACTLY
        record["recordingID"] = recordingId as CKRecordValue
        record["title"] = title as CKRecordValue
        record["cloudURL"] = cloudURL as CKRecordValue
        record["provider"] = provider.rawValue as CKRecordValue
        record["createdAt"] = Date() as CKRecordValue
        
        if let description = description {
            record["recodringDescription"] = description as CKRecordValue
        }
        
        if let thumbnailURL = thumbnailURL {
            record["thumbnailURL"] = thumbnailURL as CKRecordValue
        }
        
        if let assetURL = thumbnailAssetURL {
            record["thumbnailAsset"] = CKAsset(fileURL: assetURL)
        }
        
        do {
            let savedRecord = try await publicDatabase.save(record)
            dlog("✅ [CloudKit] Made recording public: \(title)")
            
            // Optimistically update local list
            if let newRecording = PublicRecording(from: savedRecord) {
                await MainActor.run {
                    self.publicRecordings.append(newRecording)
                }
            }
            
            return true
        } catch {
            dlog("❌ [CloudKit] Failed to make recording public: \(error)")
            errorMessage = "Failed to share recording: \(error.localizedDescription)"
            return false
        }
    }
    
    /// Make a recording private by deleting its CloudKit record
    func makeRecordingPrivate(recordingId: String) async -> Bool {
        let recordID = CKRecord.ID(recordName: recordingId)
        
        do {
            _ = try await publicDatabase.deleteRecord(withID: recordID)
            dlog("✅ [CloudKit] Made recording private: \(recordingId)")
            
            // Remove from local cache
            publicRecordings.removeAll { $0.id == recordingId }
            
            return true
        } catch let error as CKError {
            // If record doesn't exist, that's fine (already private)
            if error.code == .unknownItem {
                dlog("⚠️ [CloudKit] Recording already private: \(recordingId)")
                return true
            }
            
            dlog("❌ [CloudKit] Failed to make recording private: \(error)")
            errorMessage = "Failed to unshare recording: \(error.localizedDescription)"
            return false
        } catch {
            dlog("❌ [CloudKit] Failed to make recording private: \(error)")
            errorMessage = "Failed to unshare recording: \(error.localizedDescription)"
            return false
        }
    }
    
    /// Check if a recording is currently public
    func isRecordingPublic(recordingId: String) async -> Bool {
        let recordID = CKRecord.ID(recordName: recordingId)
        
        do {
            _ = try await publicDatabase.record(for: recordID)
            return true
        } catch let error as CKError {
            if error.code == .unknownItem {
                return false
            }
            dlog("❌ [CloudKit] Error checking public status: \(error)")
            return false
        } catch {
            dlog("❌ [CloudKit] Error checking public status: \(error)")
            return false
        }
    }
    
    /// Fetch public recordings with pagination (initial fetch)
    func fetchPublicRecordings() async {
        isLoading = true
        errorMessage = nil
        queryCursor = nil
        hasMoreRecordings = true
        
        defer { isLoading = false }
        
        // Create query to fetch latest recordings
        let query = CKQuery(
            recordType: recordType,
            predicate: NSPredicate(format: "recordingID != %@", "")
        )
        
        // Sort by creation date, newest first
        query.sortDescriptors = [NSSortDescriptor(key: "createdAt", ascending: false)]
        
        do {
            let (matchResults, cursor) = try await publicDatabase.records(matching: query, resultsLimit: pageSize)
            
            var recordings: [PublicRecording] = []
            
            for (recordID, result) in matchResults {
                switch result {
                case .success(let record):
                    if let publicRecording = PublicRecording(from: record) {
                        recordings.append(publicRecording)
                    }
                case .failure(let error):
                    dlog("❌ [CloudKit] Failed to fetch record \(recordID): \(error)")
                }
            }
            
            publicRecordings = recordings
            queryCursor = cursor
            hasMoreRecordings = cursor != nil
            
            dlog("✅ [CloudKit] Fetched \(recordings.count) public recordings (hasMore: \(hasMoreRecordings))")
            
        } catch {
            dlog("❌ [CloudKit] Failed to fetch public recordings: \(error)")
            errorMessage = "Failed to load public recordings: \(error.localizedDescription)"
        }
    }
    
    /// Load more public recordings (pagination)
    func loadMorePublicRecordings() async {
        guard !isLoadingMore, hasMoreRecordings, let cursor = queryCursor else { return }
        
        isLoadingMore = true
        defer { isLoadingMore = false }
        
        do {
            let (matchResults, newCursor) = try await publicDatabase.records(continuingMatchFrom: cursor, resultsLimit: pageSize)
            
            var newRecordings: [PublicRecording] = []
            
            for (recordID, result) in matchResults {
                switch result {
                case .success(let record):
                    if let publicRecording = PublicRecording(from: record) {
                        newRecordings.append(publicRecording)
                    }
                case .failure(let error):
                    dlog("❌ [CloudKit] Failed to fetch record \(recordID): \(error)")
                }
            }
            
            publicRecordings.append(contentsOf: newRecordings)
            queryCursor = newCursor
            hasMoreRecordings = newCursor != nil
            
            dlog("✅ [CloudKit] Loaded \(newRecordings.count) more recordings (total: \(publicRecordings.count), hasMore: \(hasMoreRecordings))")
            
        } catch {
            dlog("❌ [CloudKit] Failed to load more recordings: \(error)")
        }
    }
    
    /// Get a specific public recording by ID
    func getPublicRecording(recordingId: String) async -> PublicRecording? {
        let recordID = CKRecord.ID(recordName: recordingId)
        
        do {
            let record = try await publicDatabase.record(for: recordID)
            return PublicRecording(from: record)
        } catch {
            dlog("❌ [CloudKit] Failed to fetch recording: \(error)")
            return nil
        }
    }
    
    // MARK: - Shared Link Generation
    
    /// Generate a shared link for a Dropbox file/folder
    func generateDropboxSharedLink(path: String) async -> String? {
        return await DropboxManager.shared.createSharedLink(path: path)
    }
    
    /// Generate a shared link for a Google Drive file/folder
    func generateGoogleDriveSharedLink(fileId: String) async -> String? {
        return await GoogleDriveManager.shared.createSharedLink(fileId: fileId)
    }
}

// MARK: - Public Recording Model

/// Represents a public recording shared via CloudKit
struct PublicRecording: Identifiable, Hashable {
    let id: String
    let recordingId: String
    let title: String
    let description: String?
    let cloudURL: String
    let thumbnailURL: String?
    let thumbnailAsset: CKAsset?
    let provider: CloudStorageProvider
    let createdAt: Date
    
    /// Initialize from CloudKit record
    init?(from record: CKRecord) {
        // Match fields from CloudKit Dashboard
        guard let recordingId = record["recordingID"] as? String,
              let title = record["title"] as? String,
              let cloudURL = record["cloudURL"] as? String,
              let providerString = record["provider"] as? String,
              let provider = CloudStorageProvider(rawValue: providerString),
              let createdAt = record["createdAt"] as? Date else {
            return nil
        }
        
        self.id = record.recordID.recordName
        self.recordingId = recordingId
        self.title = title
        self.description = record["recodringDescription"] as? String
        self.cloudURL = cloudURL
        self.thumbnailURL = record["thumbnailURL"] as? String
        self.thumbnailAsset = record["thumbnailAsset"] as? CKAsset
        self.provider = provider
        self.createdAt = createdAt
    }
    
    // Manual init for testing/previews
    init(
        id: String,
        recordingId: String,
        title: String,
        description: String?,
        cloudURL: String,
        thumbnailURL: String?,
        thumbnailAsset: CKAsset? = nil,
        provider: CloudStorageProvider,
        createdAt: Date
    ) {
        self.id = id
        self.recordingId = recordingId
        self.title = title
        self.description = description
        self.cloudURL = cloudURL
        self.thumbnailURL = thumbnailURL
        self.thumbnailAsset = thumbnailAsset
        self.provider = provider
        self.createdAt = createdAt
    }
    
    var displayName: String {
        title
    }
    
    var providerIcon: String {
        switch provider {
        case .dropbox:
            return "externaldrive.fill.badge.checkmark"
        case .googleDrive:
            return "externaldrive.fill"
        case .iCloudDrive:
            return "icloud.fill"
        }
    }
    
    var providerColor: Color {
        switch provider {
        case .dropbox:
            return Color(red: 0.0, green: 0.47, blue: 0.95)
        case .googleDrive:
            return Color(red: 0.26, green: 0.52, blue: 0.96)
        case .iCloudDrive:
            return .blue
        }
    }
}
