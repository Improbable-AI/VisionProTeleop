//
//  UnifiedRecording.swift
//  Tracking Viewer
//
//  Created on 12/2/25.
//

import Foundation
import UIKit
import SwiftUI

/// Protocol that all recording types conform to for unified handling
protocol UnifiedRecording: Identifiable, Hashable {
    var id: String { get }
    var name: String { get }
    var displayName: String { get }
    var modifiedDate: Date? { get }
    var hasVideo: Bool { get }
    var hasTrackingData: Bool { get }
    var hasMetadata: Bool { get }
    var hasUSDZ: Bool { get }
    var recordingSource: RecordingSource { get }
}

// MARK: - Recording Conformance

extension Recording: UnifiedRecording {
    var name: String { folderURL.lastPathComponent }
    var modifiedDate: Date? { createdAt }
    var hasVideo: Bool { videoURL != nil }
    var hasTrackingData: Bool { trackingURL != nil }
    var hasMetadata: Bool { metadata != nil }
    var hasUSDZ: Bool { usdzURL != nil }
    var recordingSource: RecordingSource { .iCloudDrive }
}

// MARK: - DropboxRecordingInfo Conformance

extension DropboxRecordingInfo: UnifiedRecording {
    var hasTrackingData: Bool { hasTracking }
    var recordingSource: RecordingSource { .dropbox }
}

// MARK: - GoogleDriveRecordingInfo Conformance

extension GoogleDriveRecordingInfo: UnifiedRecording {
    var recordingSource: RecordingSource { .googleDrive }
}

// MARK: - Wrapper for type-erased unified recordings

/// Type-erased wrapper for unified recordings
struct AnyRecording: Identifiable, Hashable {
    let id: String
    let name: String
    let displayName: String
    let modifiedDate: Date?
    let hasVideo: Bool
    let hasTrackingData: Bool
    let hasMetadata: Bool
    let hasUSDZ: Bool
    let source: RecordingSource
    
    // Store the underlying recording for navigation
    private let _iCloudRecording: Recording?
    private let _dropboxRecording: DropboxRecordingInfo?
    private let _googleDriveRecording: GoogleDriveRecordingInfo?
    
    var iCloudRecording: Recording? { _iCloudRecording }
    var dropboxRecording: DropboxRecordingInfo? { _dropboxRecording }
    var googleDriveRecording: GoogleDriveRecordingInfo? { _googleDriveRecording }
    
    init(_ recording: Recording) {
        self.id = recording.id
        self.name = recording.name
        self.displayName = recording.displayName
        self.modifiedDate = recording.modifiedDate
        self.hasVideo = recording.hasVideo
        self.hasTrackingData = recording.hasTrackingData
        self.hasMetadata = recording.hasMetadata
        self.hasUSDZ = recording.hasUSDZ
        self.source = recording.recordingSource
        self._iCloudRecording = recording
        self._dropboxRecording = nil
        self._googleDriveRecording = nil
    }
    
    init(_ recording: DropboxRecordingInfo) {
        self.id = recording.id
        self.name = recording.name
        self.displayName = recording.displayName
        self.modifiedDate = recording.modifiedDate
        self.hasVideo = recording.hasVideo
        self.hasTrackingData = recording.hasTrackingData
        self.hasMetadata = recording.hasMetadata
        self.hasUSDZ = recording.hasUSDZ
        self.source = recording.recordingSource
        self._iCloudRecording = nil
        self._dropboxRecording = recording
        self._googleDriveRecording = nil
    }
    
    init(_ recording: GoogleDriveRecordingInfo) {
        self.id = recording.id
        self.name = recording.name
        self.displayName = recording.displayName
        self.modifiedDate = recording.modifiedDate
        self.hasVideo = recording.hasVideo
        self.hasTrackingData = recording.hasTrackingData
        self.hasMetadata = recording.hasMetadata
        self.hasUSDZ = recording.hasUSDZ
        self.source = recording.recordingSource
        self._iCloudRecording = nil
        self._dropboxRecording = nil
        self._googleDriveRecording = recording
    }
    
    static func == (lhs: AnyRecording, rhs: AnyRecording) -> Bool {
        lhs.id == rhs.id && lhs.source == rhs.source
    }
    
    func hash(into hasher: inout Hasher) {
        hasher.combine(id)
        hasher.combine(source)
    }
}
