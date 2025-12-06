//
//  Playlist.swift
//  Tracking Viewer
//
//  Created on 12/02/25.
//

import Foundation

/// Represents a playlist/collection of recordings
/// A single recording can belong to multiple playlists
struct Playlist: Identifiable, Codable, Hashable {
    let id: UUID
    var name: String
    var description: String
    var recordingIDs: [String]  // Recording folder names
    var createdAt: Date
    var modifiedAt: Date
    var color: PlaylistColor
    var icon: PlaylistIcon
    
    init(
        id: UUID = UUID(),
        name: String,
        description: String = "",
        recordingIDs: [String] = [],
        createdAt: Date = Date(),
        modifiedAt: Date = Date(),
        color: PlaylistColor = .blue,
        icon: PlaylistIcon = .folder
    ) {
        self.id = id
        self.name = name
        self.description = description
        self.recordingIDs = recordingIDs
        self.createdAt = createdAt
        self.modifiedAt = modifiedAt
        self.color = color
        self.icon = icon
    }
    
    var recordingCount: Int {
        recordingIDs.count
    }
    
    /// Check if a recording is in this playlist
    func contains(_ recording: Recording) -> Bool {
        recordingIDs.contains(recording.id)
    }
    
    /// Check if a recording ID is in this playlist
    func contains(recordingID: String) -> Bool {
        recordingIDs.contains(recordingID)
    }
    
    // MARK: - Hashable
    
    static func == (lhs: Playlist, rhs: Playlist) -> Bool {
        lhs.id == rhs.id
    }
    
    func hash(into hasher: inout Hasher) {
        hasher.combine(id)
    }
}

// MARK: - Playlist Color

enum PlaylistColor: String, Codable, CaseIterable {
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

// MARK: - Playlist Icon

enum PlaylistIcon: String, Codable, CaseIterable {
    case folder = "folder.fill"
    case star = "star.fill"
    case heart = "heart.fill"
    case bookmark = "bookmark.fill"
    case tag = "tag.fill"
    case flag = "flag.fill"
    case bolt = "bolt.fill"
    case flame = "flame.fill"
    case hand = "hand.raised.fill"
    case person = "person.fill"
    case cube = "cube.fill"
    case gearshape = "gearshape.fill"
    case wrench = "wrench.and.screwdriver.fill"
    case house = "house.fill"
    case briefcase = "briefcase.fill"
    case graduationcap = "graduationcap.fill"
    case lightbulb = "lightbulb.fill"
    case leaf = "leaf.fill"
    
    var systemName: String {
        rawValue
    }
}

import SwiftUI
