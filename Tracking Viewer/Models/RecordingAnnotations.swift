//
//  RecordingAnnotations.swift
//  Tracking Viewer
//
//  Created on 12/02/25.
//

import Foundation
import SwiftUI
import Combine

// MARK: - Recording Annotation

/// User-added annotations for a recording (tags, notes, favorites)
struct RecordingAnnotation: Codable, Equatable {
    var isFavorite: Bool = false
    var tags: [String] = []
    var notes: String = ""
    var customName: String? = nil  // Optional custom name override
    var modifiedAt: Date = Date()
    
    var isEmpty: Bool {
        !isFavorite && tags.isEmpty && notes.isEmpty && customName == nil
    }
}

// MARK: - Tag

/// Represents a tag with optional color
struct Tag: Identifiable, Codable, Hashable {
    let id: UUID
    var name: String
    var color: TagColor
    
    init(id: UUID = UUID(), name: String, color: TagColor = .gray) {
        self.id = id
        self.name = name
        self.color = color
    }
    
    static func == (lhs: Tag, rhs: Tag) -> Bool {
        lhs.id == rhs.id
    }
    
    func hash(into hasher: inout Hasher) {
        hasher.combine(id)
    }
}

enum TagColor: String, Codable, CaseIterable {
    case red, orange, yellow, green, mint, teal, cyan, blue, indigo, purple, pink, brown, gray
    
    var color: Color {
        switch self {
        case .red: return .red
        case .orange: return .orange
        case .yellow: return .yellow
        case .green: return .green
        case .mint: return .mint
        case .teal: return .teal
        case .cyan: return .cyan
        case .blue: return .blue
        case .indigo: return .indigo
        case .purple: return .purple
        case .pink: return .pink
        case .brown: return .brown
        case .gray: return .gray
        }
    }
}

// MARK: - Annotations Manager

/// Manages recording annotations with persistent storage
@MainActor
class AnnotationsManager: ObservableObject {
    static let shared = AnnotationsManager()
    
    /// Annotations keyed by recording ID
    @Published var annotations: [String: RecordingAnnotation] = [:]
    
    /// All available tags
    @Published var allTags: [Tag] = []
    
    private let fileManager = FileManager.default
    
    private var annotationsURL: URL? {
        if let containerURL = fileManager.url(forUbiquityContainerIdentifier: nil) {
            return containerURL
                .appendingPathComponent("Documents")
                .appendingPathComponent("VisionProTeleop")
                .appendingPathComponent("annotations.json")
        }
        if let documentsURL = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first {
            return documentsURL.appendingPathComponent("annotations.json")
        }
        return nil
    }
    
    private var tagsURL: URL? {
        if let containerURL = fileManager.url(forUbiquityContainerIdentifier: nil) {
            return containerURL
                .appendingPathComponent("Documents")
                .appendingPathComponent("VisionProTeleop")
                .appendingPathComponent("tags.json")
        }
        if let documentsURL = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first {
            return documentsURL.appendingPathComponent("tags.json")
        }
        return nil
    }
    
    private init() {
        loadAnnotations()
        loadTags()
    }
    
    // MARK: - Annotation Operations
    
    /// Get annotation for a recording
    func annotation(for recording: Recording) -> RecordingAnnotation {
        annotations[recording.id] ?? RecordingAnnotation()
    }
    
    /// Get annotation for a recording ID
    func annotation(for recordingID: String) -> RecordingAnnotation? {
        annotations[recordingID]
    }
    
    /// Update annotation for a recording
    func updateAnnotation(for recording: Recording, _ update: (inout RecordingAnnotation) -> Void) {
        updateAnnotation(forID: recording.id, update)
    }
    
    /// Update annotation for a recording ID
    func updateAnnotation(forID recordingID: String, _ update: (inout RecordingAnnotation) -> Void) {
        var annotation = annotations[recordingID] ?? RecordingAnnotation()
        update(&annotation)
        annotation.modifiedAt = Date()
        
        if annotation.isEmpty {
            annotations.removeValue(forKey: recordingID)
        } else {
            annotations[recordingID] = annotation
        }
        saveAnnotations()
    }
    
    // MARK: - Favorites
    
    func isFavorite(_ recording: Recording) -> Bool {
        annotations[recording.id]?.isFavorite ?? false
    }
    
    func isFavorite(_ recordingID: String) -> Bool {
        annotations[recordingID]?.isFavorite ?? false
    }
    
    func toggleFavorite(_ recording: Recording) {
        updateAnnotation(forID: recording.id) { annotation in
            annotation.isFavorite.toggle()
        }
    }
    
    func toggleFavorite(_ recordingID: String) {
        updateAnnotation(forID: recordingID) { annotation in
            annotation.isFavorite.toggle()
        }
    }
    
    func setFavorite(_ recording: Recording, _ value: Bool) {
        updateAnnotation(forID: recording.id) { annotation in
            annotation.isFavorite = value
        }
    }
    
    // MARK: - Notes
    
    func notes(for recording: Recording) -> String {
        annotations[recording.id]?.notes ?? ""
    }
    
    func notes(for recordingID: String) -> String? {
        let notes = annotations[recordingID]?.notes
        return notes?.isEmpty == true ? nil : notes
    }
    
    func setNotes(for recording: Recording, _ notes: String) {
        updateAnnotation(forID: recording.id) { annotation in
            annotation.notes = notes
        }
    }
    
    func setNotes(_ notes: String, for recordingID: String) {
        updateAnnotation(forID: recordingID) { annotation in
            annotation.notes = notes
        }
    }
    
    // MARK: - Custom Name
    
    func customName(for recording: Recording) -> String? {
        annotations[recording.id]?.customName
    }
    
    func setCustomName(for recording: Recording, _ name: String?) {
        updateAnnotation(forID: recording.id) { annotation in
            annotation.customName = name?.isEmpty == true ? nil : name
        }
    }
    
    func setCustomName(_ name: String?, for recordingID: String) {
        updateAnnotation(forID: recordingID) { annotation in
            annotation.customName = name?.isEmpty == true ? nil : name
        }
    }
    
    func displayName(for recording: Recording) -> String {
        customName(for: recording) ?? recording.displayName
    }
    
    // MARK: - Tags
    
    func tags(for recording: Recording) -> [Tag] {
        let tagIDs = annotations[recording.id]?.tags ?? []
        return tagIDs.compactMap { tagID in
            allTags.first { $0.id.uuidString == tagID }
        }
    }
    
    func tags(for recordingID: String) -> [Tag] {
        let tagIDs = annotations[recordingID]?.tags ?? []
        return tagIDs.compactMap { tagID in
            allTags.first { $0.id.uuidString == tagID }
        }
    }
    
    func tagNames(for recording: Recording) -> [String] {
        tags(for: recording).map { $0.name }
    }
    
    func hasTag(_ recording: Recording, tag: Tag) -> Bool {
        annotations[recording.id]?.tags.contains(tag.id.uuidString) ?? false
    }
    
    func hasTag(_ recordingID: String, tag: Tag) -> Bool {
        annotations[recordingID]?.tags.contains(tag.id.uuidString) ?? false
    }
    
    func addTag(_ tag: Tag, to recording: Recording) {
        updateAnnotation(forID: recording.id) { annotation in
            if !annotation.tags.contains(tag.id.uuidString) {
                annotation.tags.append(tag.id.uuidString)
            }
        }
    }
    
    func addTag(_ tagID: UUID, to recordingID: String) {
        updateAnnotation(forID: recordingID) { annotation in
            if !annotation.tags.contains(tagID.uuidString) {
                annotation.tags.append(tagID.uuidString)
            }
        }
    }
    
    func removeTag(_ tag: Tag, from recording: Recording) {
        updateAnnotation(forID: recording.id) { annotation in
            annotation.tags.removeAll { $0 == tag.id.uuidString }
        }
    }
    
    func removeTag(_ tagID: UUID, from recordingID: String) {
        updateAnnotation(forID: recordingID) { annotation in
            annotation.tags.removeAll { $0 == tagID.uuidString }
        }
    }
    
    func toggleTag(_ tag: Tag, for recording: Recording) {
        if hasTag(recording, tag: tag) {
            removeTag(tag, from: recording)
        } else {
            addTag(tag, to: recording)
        }
    }
    
    func toggleTag(_ tag: Tag, for recordingID: String) {
        if hasTag(recordingID, tag: tag) {
            removeTag(tag.id, from: recordingID)
        } else {
            addTag(tag.id, to: recordingID)
        }
    }
    
    // MARK: - Tag Management
    
    func createTag(name: String, color: TagColor = .gray) -> Tag {
        let tag = Tag(name: name, color: color)
        allTags.append(tag)
        saveTags()
        return tag
    }
    
    func updateTag(_ tag: Tag, name: String? = nil, color: TagColor? = nil) {
        guard let index = allTags.firstIndex(where: { $0.id == tag.id }) else { return }
        if let name = name {
            allTags[index].name = name
        }
        if let color = color {
            allTags[index].color = color
        }
        saveTags()
    }
    
    func deleteTag(_ tag: Tag) {
        // Remove tag from all recordings
        for (recordingID, var annotation) in annotations {
            if annotation.tags.contains(tag.id.uuidString) {
                annotation.tags.removeAll { $0 == tag.id.uuidString }
                annotations[recordingID] = annotation
            }
        }
        
        // Remove from all tags
        allTags.removeAll { $0.id == tag.id }
        
        saveTags()
        saveAnnotations()
    }
    
    /// Get recordings count for a tag
    func recordingsCount(for tag: Tag) -> Int {
        annotations.values.filter { $0.tags.contains(tag.id.uuidString) }.count
    }
    
    // MARK: - Cleanup
    
    /// Remove annotations for a deleted recording
    func removeAnnotation(for recording: Recording) {
        annotations.removeValue(forKey: recording.id)
        saveAnnotations()
    }
    
    // MARK: - Persistence
    
    private func loadAnnotations() {
        guard let url = annotationsURL else { return }
        try? fileManager.startDownloadingUbiquitousItem(at: url)
        
        guard fileManager.fileExists(atPath: url.path) else { return }
        
        do {
            let data = try Data(contentsOf: url)
            let decoder = JSONDecoder()
            decoder.dateDecodingStrategy = .iso8601
            annotations = try decoder.decode([String: RecordingAnnotation].self, from: data)
            print("✅ [AnnotationsManager] Loaded \(annotations.count) annotations")
        } catch {
            print("❌ [AnnotationsManager] Failed to load annotations: \(error)")
        }
    }
    
    private func saveAnnotations() {
        guard let url = annotationsURL else { return }
        
        let directory = url.deletingLastPathComponent()
        if !fileManager.fileExists(atPath: directory.path) {
            try? fileManager.createDirectory(at: directory, withIntermediateDirectories: true)
        }
        
        do {
            let encoder = JSONEncoder()
            encoder.dateEncodingStrategy = .iso8601
            encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
            let data = try encoder.encode(annotations)
            try data.write(to: url)
        } catch {
            print("❌ [AnnotationsManager] Failed to save annotations: \(error)")
        }
    }
    
    private func loadTags() {
        guard let url = tagsURL else { return }
        try? fileManager.startDownloadingUbiquitousItem(at: url)
        
        guard fileManager.fileExists(atPath: url.path) else {
            // Create some default tags
            createDefaultTags()
            return
        }
        
        do {
            let data = try Data(contentsOf: url)
            allTags = try JSONDecoder().decode([Tag].self, from: data)
            print("✅ [AnnotationsManager] Loaded \(allTags.count) tags")
        } catch {
            print("❌ [AnnotationsManager] Failed to load tags: \(error)")
            createDefaultTags()
        }
    }
    
    private func saveTags() {
        guard let url = tagsURL else { return }
        
        let directory = url.deletingLastPathComponent()
        if !fileManager.fileExists(atPath: directory.path) {
            try? fileManager.createDirectory(at: directory, withIntermediateDirectories: true)
        }
        
        do {
            let encoder = JSONEncoder()
            encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
            let data = try encoder.encode(allTags)
            try data.write(to: url)
        } catch {
            print("❌ [AnnotationsManager] Failed to save tags: \(error)")
        }
    }
    
    private func createDefaultTags() {
        allTags = [
            Tag(name: "Important", color: .red),
            Tag(name: "Good Quality", color: .green),
            Tag(name: "Demo", color: .blue),
            Tag(name: "Test", color: .orange),
            Tag(name: "Kitchen", color: .purple),
            Tag(name: "Assembly", color: .teal)
        ]
        saveTags()
    }
}
