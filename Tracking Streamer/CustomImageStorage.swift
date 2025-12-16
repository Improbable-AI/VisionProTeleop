import Foundation
import SwiftUI
import UIKit

// MARK: - Data Models

/// Type of tracked image
enum TrackedImageType: String, Codable {
    case aruco   // Generated ArUco marker
    case custom  // User-uploaded image
}

/// Registration data for a custom image (persisted)
struct CustomImageRegistration: Identifiable, Codable {
    let id: String              // UUID
    var name: String            // User-editable display name
    var physicalWidthMeters: Float  // Real-world width (default: 0.1 = 10cm)
    let imageFileName: String   // Stored in Documents/TrackedImages/
    let createdAt: Date
    
    init(name: String, physicalWidthMeters: Float = 0.1, imageFileName: String) {
        self.id = UUID().uuidString
        self.name = name
        self.physicalWidthMeters = physicalWidthMeters
        self.imageFileName = imageFileName
        self.createdAt = Date()
    }
}

// MARK: - Storage Manager

/// Manages persistence of custom tracked images
/// Images are stored in Documents/TrackedImages/
/// Metadata is stored in Documents/custom_images.json and synced via iCloud
@MainActor
class CustomImageStorage: ObservableObject {
    static let shared = CustomImageStorage()
    
    /// All registered custom images
    @Published private(set) var registrations: [CustomImageRegistration] = []
    
    /// Directory for storing tracked images
    private let imagesDirectory: URL
    
    /// JSON file for metadata
    private let metadataFile: URL
    
    /// iCloud key for syncing
    private let iCloudKey = "customImageRegistrations"
    
    private init() {
        // Setup directories
        let documentsDir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!
        imagesDirectory = documentsDir.appendingPathComponent("TrackedImages", isDirectory: true)
        metadataFile = documentsDir.appendingPathComponent("custom_images.json")
        
        // Create images directory if needed
        try? FileManager.default.createDirectory(at: imagesDirectory, withIntermediateDirectories: true)
        
        // Load existing registrations
        loadRegistrations()
        
        // Listen for iCloud changes
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(iCloudDidChange),
            name: NSUbiquitousKeyValueStore.didChangeExternallyNotification,
            object: NSUbiquitousKeyValueStore.default
        )
        NSUbiquitousKeyValueStore.default.synchronize()
        
        dlog("📷 [CustomImageStorage] Initialized with \(registrations.count) registered images")
    }
    
    // MARK: - Public API
    
    /// Register a new custom image for tracking
    /// - Parameters:
    ///   - image: The UIImage to track
    ///   - name: Display name for the image
    ///   - physicalWidth: Real-world width in meters (default: 0.1 = 10cm)
    /// - Returns: The created registration, or nil if failed
    func registerImage(_ image: UIImage, name: String, physicalWidth: Float = 0.1) -> CustomImageRegistration? {
        // Generate unique filename
        let fileName = "\(UUID().uuidString).png"
        let fileURL = imagesDirectory.appendingPathComponent(fileName)
        
        // Save image as PNG
        guard let data = image.pngData() else {
            dlog("❌ [CustomImageStorage] Failed to convert image to PNG")
            return nil
        }
        
        do {
            try data.write(to: fileURL)
        } catch {
            dlog("❌ [CustomImageStorage] Failed to save image: \(error)")
            return nil
        }
        
        // Create registration
        let registration = CustomImageRegistration(
            name: name,
            physicalWidthMeters: physicalWidth,
            imageFileName: fileName
        )
        
        registrations.append(registration)
        saveRegistrations()
        
        dlog("✅ [CustomImageStorage] Registered image '\(name)' (ID: \(registration.id), \(physicalWidth * 100)cm)")
        
        // Notify marker detection to restart with new images
        NotificationCenter.default.post(name: .customImagesDidChange, object: nil)
        
        return registration
    }
    
    /// Unregister a custom image
    func unregisterImage(id: String) {
        guard let index = registrations.firstIndex(where: { $0.id == id }) else {
            dlog("⚠️ [CustomImageStorage] Image not found: \(id)")
            return
        }
        
        let registration = registrations[index]
        
        // Delete image file
        let fileURL = imagesDirectory.appendingPathComponent(registration.imageFileName)
        try? FileManager.default.removeItem(at: fileURL)
        
        // Remove registration
        registrations.remove(at: index)
        saveRegistrations()
        
        dlog("🗑 [CustomImageStorage] Unregistered image '\(registration.name)' (ID: \(id))")
        
        // Notify marker detection to restart
        NotificationCenter.default.post(name: .customImagesDidChange, object: nil)
    }
    
    /// Update a registration's properties
    func updateRegistration(id: String, name: String? = nil, physicalWidth: Float? = nil) {
        guard let index = registrations.firstIndex(where: { $0.id == id }) else {
            return
        }
        
        if let name = name {
            registrations[index].name = name
        }
        if let width = physicalWidth {
            registrations[index].physicalWidthMeters = width
        }
        
        saveRegistrations()
        
        // Notify if size changed (requires restart)
        if physicalWidth != nil {
            NotificationCenter.default.post(name: .customImagesDidChange, object: nil)
        }
        
        dlog("📝 [CustomImageStorage] Updated image ID \(id)")
    }
    
    /// Load the UIImage for a registration
    func loadImage(for registration: CustomImageRegistration) -> UIImage? {
        let fileURL = imagesDirectory.appendingPathComponent(registration.imageFileName)
        guard let data = try? Data(contentsOf: fileURL) else {
            return nil
        }
        return UIImage(data: data)
    }
    
    /// Load thumbnail for a registration (scaled down)
    func loadThumbnail(for registration: CustomImageRegistration, size: CGSize = CGSize(width: 60, height: 60)) -> UIImage? {
        guard let image = loadImage(for: registration) else {
            return nil
        }
        
        let renderer = UIGraphicsImageRenderer(size: size)
        return renderer.image { _ in
            image.draw(in: CGRect(origin: .zero, size: size))
        }
    }
    
    // MARK: - Private Methods
    
    private func loadRegistrations() {
        // Try loading from local file first
        if let data = try? Data(contentsOf: metadataFile),
           let decoded = try? JSONDecoder().decode([CustomImageRegistration].self, from: data) {
            registrations = decoded
            dlog("📷 [CustomImageStorage] Loaded \(decoded.count) registrations from disk")
        }
        
        // Check iCloud for newer data
        if let iCloudData = NSUbiquitousKeyValueStore.default.data(forKey: iCloudKey),
           let iCloudRegistrations = try? JSONDecoder().decode([CustomImageRegistration].self, from: iCloudData) {
            // Use iCloud data if it has more registrations (simple merge strategy)
            if iCloudRegistrations.count > registrations.count {
                registrations = iCloudRegistrations
                dlog("☁️ [CustomImageStorage] Using iCloud registrations (\(iCloudRegistrations.count) items)")
            }
        }
    }
    
    private func saveRegistrations() {
        // Save to local file
        if let data = try? JSONEncoder().encode(registrations) {
            try? data.write(to: metadataFile)
            
            // Sync to iCloud
            NSUbiquitousKeyValueStore.default.set(data, forKey: iCloudKey)
            NSUbiquitousKeyValueStore.default.synchronize()
        }
    }
    
    @objc private func iCloudDidChange(_ notification: Notification) {
        guard let userInfo = notification.userInfo,
              let changedKeys = userInfo[NSUbiquitousKeyValueStoreChangedKeysKey] as? [String],
              changedKeys.contains(iCloudKey) else {
            return
        }
        
        dlog("☁️ [CustomImageStorage] iCloud data changed, reloading...")
        loadRegistrations()
        
        // Notify marker detection
        NotificationCenter.default.post(name: .customImagesDidChange, object: nil)
    }
}

// MARK: - Notifications

extension Notification.Name {
    /// Posted when custom image registrations change (add/remove/update)
    static let customImagesDidChange = Notification.Name("customImagesDidChange")
}
