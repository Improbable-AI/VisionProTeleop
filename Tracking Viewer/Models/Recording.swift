//
//  Recording.swift
//  Tracking Viewer
//
//  Created on 11/29/25.
//

import Foundation
import UIKit
import AVFoundation

/// Represents a single recording session with its metadata and video
struct Recording: Identifiable, Hashable {
    let id: String
    let folderURL: URL
    let metadata: RecordingMetadata?
    let videoURL: URL?
    let trackingURL: URL?
    
    var simulationDataURL: URL? {
        guard metadata?.hasSimulationData == true else { return nil }
        return folderURL.appendingPathComponent("mjdata.jsonl")
    }
    
    var usdzURL: URL? {
        guard metadata?.hasUSDZ == true else { return nil }
        return folderURL.appendingPathComponent("scene.usdz")
    }
    
    var displayName: String {
        // Extract date from folder name (e.g., "recording_20251129_143000")
        let name = folderURL.lastPathComponent
        if name.hasPrefix("recording_") {
            let dateString = String(name.dropFirst("recording_".count))
            if let date = Recording.dateFormatter.date(from: dateString) {
                return Recording.displayFormatter.string(from: date)
            }
        }
        return name
    }
    
    var createdAt: Date {
        metadata?.createdAt ?? (try? FileManager.default.attributesOfItem(atPath: folderURL.path)[.creationDate] as? Date) ?? Date.distantPast
    }
    
    var durationString: String {
        guard let duration = metadata?.duration else { return "--:--" }
        let minutes = Int(duration) / 60
        let seconds = Int(duration) % 60
        return String(format: "%d:%02d", minutes, seconds)
    }
    
    var frameCountString: String {
        guard let count = metadata?.frameCount else { return "--" }
        return "\(count) frames"
    }
    
    var fpsString: String {
        guard let fps = metadata?.averageFPS else { return "--" }
        return String(format: "%.0f fps", fps)
    }
    
    var fileSizeString: String {
        guard let videoURL = videoURL,
              let attributes = try? FileManager.default.attributesOfItem(atPath: videoURL.path),
              let size = attributes[.size] as? Int64 else {
            return "--"
        }
        return ByteCountFormatter.string(fromByteCount: size, countStyle: .file)
    }
    
    // MARK: - Date Formatters
    
    private static let dateFormatter: DateFormatter = {
        let f = DateFormatter()
        f.dateFormat = "yyyyMMdd_HHmmss"
        return f
    }()
    
    private static let displayFormatter: DateFormatter = {
        let f = DateFormatter()
        f.dateStyle = .medium
        f.timeStyle = .short
        return f
    }()
    
    // MARK: - Hashable
    
    static func == (lhs: Recording, rhs: Recording) -> Bool {
        lhs.id == rhs.id
    }
    
    func hash(into hasher: inout Hasher) {
        hasher.combine(id)
    }
}

/// Metadata structure matching RecordingManager's format
struct RecordingMetadata: Codable {
    let version: String?
    let createdAt: Date
    let duration: Double
    let frameCount: Int
    let hasVideo: Bool
    let hasLeftHand: Bool
    let hasRightHand: Bool
    let hasHead: Bool
    let hasSimulationData: Bool
    let hasUSDZ: Bool
    let videoSource: String
    let averageFPS: Double
    let deviceInfo: DeviceInfo?
    
    enum CodingKeys: String, CodingKey {
        case version, createdAt, duration, frameCount, hasVideo
        case hasLeftHand, hasRightHand, hasHead, hasSimulationData, hasUSDZ, videoSource, averageFPS, deviceInfo
    }
    
    init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        
        version = try container.decodeIfPresent(String.self, forKey: .version)
        duration = try container.decode(Double.self, forKey: .duration)
        frameCount = try container.decode(Int.self, forKey: .frameCount)
        hasVideo = try container.decode(Bool.self, forKey: .hasVideo)
        hasLeftHand = try container.decode(Bool.self, forKey: .hasLeftHand)
        hasRightHand = try container.decode(Bool.self, forKey: .hasRightHand)
        hasHead = try container.decode(Bool.self, forKey: .hasHead)
        hasSimulationData = try container.decodeIfPresent(Bool.self, forKey: .hasSimulationData) ?? false
        hasUSDZ = try container.decodeIfPresent(Bool.self, forKey: .hasUSDZ) ?? false
        videoSource = try container.decode(String.self, forKey: .videoSource)
        averageFPS = try container.decode(Double.self, forKey: .averageFPS)
        deviceInfo = try container.decodeIfPresent(DeviceInfo.self, forKey: .deviceInfo)
        
        // Handle createdAt as a Double (timeIntervalSinceReferenceDate from default JSONEncoder)
        let timestamp = try container.decode(Double.self, forKey: .createdAt)
        createdAt = Date(timeIntervalSinceReferenceDate: timestamp)
    }
}

struct DeviceInfo: Codable {
    let model: String
    let systemVersion: String
    let appVersion: String
}
