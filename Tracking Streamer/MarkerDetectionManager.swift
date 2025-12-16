import Foundation
import SwiftUI
import simd
import ARKit
import AVFoundation
import AudioToolbox

/// Detected marker information
struct DetectedMarker: Identifiable {
    let id: Int  // Marker ID
    let dictionaryType: Int  // ArUco dictionary type (raw value)
    let poseInWorld: simd_float4x4  // Marker pose in world frame
    let estimatedSizeMeters: Float  // Automatically estimated physical size
    var isTracked: Bool  // Whether ARKit is actively tracking this marker
    let timestamp: Date
    
    /// Extract position from pose matrix
    var position: SIMD3<Float> {
        SIMD3<Float>(poseInWorld.columns.3.x, poseInWorld.columns.3.y, poseInWorld.columns.3.z)
    }
    
    /// Extract quaternion (w, x, y, z) from rotation matrix
    var quaternion: SIMD4<Float> {
        let m = poseInWorld
        let trace = m.columns.0.x + m.columns.1.y + m.columns.2.z
        
        var qw: Float, qx: Float, qy: Float, qz: Float
        
        if trace > 0 {
            let s = 0.5 / sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (m.columns.2.y - m.columns.1.z) * s
            qy = (m.columns.0.z - m.columns.2.x) * s
            qz = (m.columns.1.x - m.columns.0.y) * s
        } else if m.columns.0.x > m.columns.1.y && m.columns.0.x > m.columns.2.z {
            let s = 2.0 * sqrt(1.0 + m.columns.0.x - m.columns.1.y - m.columns.2.z)
            qw = (m.columns.2.y - m.columns.1.z) / s
            qx = 0.25 * s
            qy = (m.columns.0.y + m.columns.1.x) / s
            qz = (m.columns.0.z + m.columns.2.x) / s
        } else if m.columns.1.y > m.columns.2.z {
            let s = 2.0 * sqrt(1.0 + m.columns.1.y - m.columns.0.x - m.columns.2.z)
            qw = (m.columns.0.z - m.columns.2.x) / s
            qx = (m.columns.0.y + m.columns.1.x) / s
            qy = 0.25 * s
            qz = (m.columns.1.z + m.columns.2.y) / s
        } else {
            let s = 2.0 * sqrt(1.0 + m.columns.2.z - m.columns.0.x - m.columns.1.y)
            qw = (m.columns.1.x - m.columns.0.y) / s
            qx = (m.columns.0.z + m.columns.2.x) / s
            qy = (m.columns.1.z + m.columns.2.y) / s
            qz = 0.25 * s
        }
        
        return SIMD4<Float>(qw, qx, qy, qz)
    }
}

/// Tracked custom image information
struct TrackedCustomImage: Identifiable {
    let id: String  // Registration UUID
    let name: String  // Display name
    let poseInWorld: simd_float4x4  // Image pose in world frame
    let physicalWidthMeters: Float  // Real-world width
    var isTracked: Bool  // Whether ARKit is actively tracking this image
    let timestamp: Date
    
    /// Extract position from pose matrix
    var position: SIMD3<Float> {
        SIMD3<Float>(poseInWorld.columns.3.x, poseInWorld.columns.3.y, poseInWorld.columns.3.z)
    }
    
    /// Extract quaternion (w, x, y, z) from rotation matrix
    var quaternion: SIMD4<Float> {
        let m = poseInWorld
        let trace = m.columns.0.x + m.columns.1.y + m.columns.2.z
        
        var qw: Float, qx: Float, qy: Float, qz: Float
        
        if trace > 0 {
            let s = 0.5 / sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (m.columns.2.y - m.columns.1.z) * s
            qy = (m.columns.0.z - m.columns.2.x) * s
            qz = (m.columns.1.x - m.columns.0.y) * s
        } else if m.columns.0.x > m.columns.1.y && m.columns.0.x > m.columns.2.z {
            let s = 2.0 * sqrt(1.0 + m.columns.0.x - m.columns.1.y - m.columns.2.z)
            qw = (m.columns.2.y - m.columns.1.z) / s
            qx = 0.25 * s
            qy = (m.columns.0.y + m.columns.1.x) / s
            qz = (m.columns.0.z + m.columns.2.x) / s
        } else if m.columns.1.y > m.columns.2.z {
            let s = 2.0 * sqrt(1.0 + m.columns.1.y - m.columns.0.x - m.columns.2.z)
            qw = (m.columns.0.z - m.columns.2.x) / s
            qx = (m.columns.0.y + m.columns.1.x) / s
            qy = 0.25 * s
            qz = (m.columns.1.z + m.columns.2.y) / s
        } else {
            let s = 2.0 * sqrt(1.0 + m.columns.2.z - m.columns.0.x - m.columns.1.y)
            qw = (m.columns.1.x - m.columns.0.y) / s
            qx = (m.columns.0.z + m.columns.2.x) / s
            qy = (m.columns.1.z + m.columns.2.y) / s
            qz = 0.25 * s
        }
        
        return SIMD4<Float>(qw, qx, qy, qz)
    }
}

/// Manager for ArUco marker detection and custom image tracking
/// Streams detected image poses to Python via gRPC
@MainActor
class MarkerDetectionManager: ObservableObject {
    static let shared = MarkerDetectionManager()
    
    // MARK: - Published Properties
    
    /// Whether marker detection is enabled
    @Published var isEnabled: Bool = true {
        didSet {
            updateSnapshot()
            if isEnabled && !isRunning {
                Task { await startDetection() }
            } else if !isEnabled && isRunning {
                stopDetection()
            }
        }
    }

    
    /// Whether pose updates are frozen (visualization stays at last known positions) - DEPRECATED, use per-marker isFixed
    @Published var isFrozen: Bool = false
    
    /// Markers that have been "fixed" at their last known pose (marker ID -> frozen DetectedMarker)
    /// Fixed markers persist their pose even when not actively tracked
    @Published var fixedMarkers: [Int: DetectedMarker] = [:]
    
    /// Physical marker size in meters (user-selected reference size for ARKit tracking)
    @Published var markerSizeMeters: Float = 0.055 {
        didSet {
            UserDefaults.standard.set(markerSizeMeters, forKey: "markerSizeMeters")
            // Restart if running to apply new size
            if isRunning {
                stopDetection()
                Task { await startDetection() }
            }
        }
    }
    
    /// Which ArUco dictionary types to track
    /// Default: 4x4_50 (0) only
    @Published var enabledDictionaries: Set<ArucoDictionaryType> = [
        ArucoDictionaryType(rawValue: 0)!  // 4x4_50 only
    ] {
        didSet {
            // Restart if running to apply new dictionaries
            if isRunning {
                stopDetection()
                Task { await startDetection() }
            }
        }
    }
    
    /// Currently detected markers (marker ID -> DetectedMarker)
    @Published var detectedMarkers: [Int: DetectedMarker] = [:]
    
    /// Currently tracked custom images (registration ID -> TrackedCustomImage)
    @Published var trackedCustomImages: [String: TrackedCustomImage] = [:]
    
    /// Fixed custom images (ID -> frozen TrackedCustomImage)
    @Published var fixedCustomImages: [String: TrackedCustomImage] = [:]
    
    /// Whether detection is actively running
    @Published private(set) var isRunning: Bool = false
    
    /// Status message for UI
    @Published var statusMessage: String = "Idle"
    
    // MARK: - Private Properties
    
    private var arkitSession: ARKitSession?
    private var imageTrackingProvider: ImageTrackingProvider?
    
    /// Marker IDs to track (0-49 for 4x4_50 dictionary)
    private let markerIdsToTrack: [Int] = Array(0...19)  // Track first 20 markers
    
    /// Audio player for detection feedback
    private var audioPlayer: AVAudioPlayer?
    
    /// Track which markers have been announced (to avoid repeated sounds)
    private var announcedMarkers: Set<Int> = []
    
    /// Track which custom images have been announced
    private var announcedCustomImages: Set<String> = []
    
    // Thread-safe snapshot for gRPC streaming (accessed from non-MainActor context)
    // Using nonisolated(unsafe) to allow non-MainActor access with manual synchronization
    private let snapshotLock = NSLock()
    nonisolated(unsafe) private var _snapshotEnabled: Bool = false
    nonisolated(unsafe) private var _snapshotMarkers: [Int: DetectedMarker] = [:]
    nonisolated(unsafe) private var _snapshotFixedMarkers: Set<Int> = []
    nonisolated(unsafe) private var _snapshotCustomImages: [String: TrackedCustomImage] = [:]
    nonisolated(unsafe) private var _snapshotFixedCustomImages: Set<String> = []
    
    /// Thread-safe access to enabled state (for gRPC streaming)
    nonisolated var snapshotEnabled: Bool {
        snapshotLock.lock()
        defer { snapshotLock.unlock() }
        return _snapshotEnabled
    }
    
    /// Thread-safe access to detected markers (for gRPC streaming)
    nonisolated var snapshotMarkers: [Int: DetectedMarker] {
        snapshotLock.lock()
        defer { snapshotLock.unlock() }
        return _snapshotMarkers
    }
    
    /// Thread-safe access to fixed marker IDs (for gRPC streaming)
    nonisolated var snapshotFixedMarkers: Set<Int> {
        snapshotLock.lock()
        defer { snapshotLock.unlock() }
        return _snapshotFixedMarkers
    }
    
    /// Thread-safe access to tracked custom images (for gRPC streaming)
    nonisolated var snapshotCustomImages: [String: TrackedCustomImage] {
        snapshotLock.lock()
        defer { snapshotLock.unlock() }
        return _snapshotCustomImages
    }
    
    /// Thread-safe access to fixed custom image IDs (for gRPC streaming)
    nonisolated var snapshotFixedCustomImages: Set<String> {
        snapshotLock.lock()
        defer { snapshotLock.unlock() }
        return _snapshotFixedCustomImages
    }
    
    /// Update the thread-safe snapshot (called from MainActor)
    private func updateSnapshot() {
        snapshotLock.lock()
        _snapshotEnabled = isEnabled
        // Merge detected and fixed markers - fixed markers override detected ones
        var allMarkers = detectedMarkers
        for (id, marker) in fixedMarkers {
            allMarkers[id] = marker
        }
        _snapshotMarkers = allMarkers
        _snapshotFixedMarkers = Set(fixedMarkers.keys)
        // Merge custom images
        var allCustom = trackedCustomImages
        for (id, img) in fixedCustomImages {
            allCustom[id] = img
        }
        _snapshotCustomImages = allCustom
        _snapshotFixedCustomImages = Set(fixedCustomImages.keys)
        snapshotLock.unlock()
    }
    
    // MARK: - Initialization
    
    private init() {
        // Load persisted settings
        if let savedSize = UserDefaults.standard.object(forKey: "markerSizeMeters") as? Float {
            self.markerSizeMeters = savedSize
        }
        dlog("🎯 [MarkerDetection] Manager initialized, isEnabled=\(isEnabled)")
        
        // Listen for custom image changes
        NotificationCenter.default.addObserver(
            forName: .customImagesDidChange,
            object: nil,
            queue: .main
        ) { [weak self] _ in
            Task { @MainActor in
                guard let self = self, self.isRunning else { return }
                dlog("📷 [MarkerDetection] Custom images changed, restarting detection...")
                self.stopDetection()
                await self.startDetection()
            }
        }
        
        // Auto-start if enabled by default
        if isEnabled {
            Task { await startDetection() }
        }
    }
    
    // MARK: - Detection Control
    
    /// Start marker detection
    func startDetection() async {
        guard !isRunning else {
            dlog("🎯 [MarkerDetection] Already running")
            return
        }
        
        dlog("🎯 [MarkerDetection] Starting detection...")
        dlog("🎯 [MarkerDetection] Reference size: 10cm (auto-scale), Dictionaries: \(enabledDictionaries.map { $0.rawValue })")
        statusMessage = "Starting..."
        
        // Generate reference images for all enabled dictionaries (ArUco markers)
        var referenceImages: [ReferenceImage] = []
        
        for dict in enabledDictionaries {
            dlog("🎯 [MarkerDetection] Generating markers for dictionary rawValue: \(dict.rawValue)")
            for markerId in markerIdsToTrack {
                if let refImage = generateReferenceImage(markerId: markerId, dictionary: dict) {
                    referenceImages.append(refImage)
                    dlog("🎯 [MarkerDetection] Generated marker ID \(markerId) with name: \(refImage.name ?? "nil")")
                }
            }
        }
        
        // Add custom user images
        let customStorage = CustomImageStorage.shared
        for registration in customStorage.registrations {
            if let refImage = generateCustomReferenceImage(registration: registration) {
                referenceImages.append(refImage)
                dlog("📷 [MarkerDetection] Added custom image '\(registration.name)' (ID: \(registration.id))")
            }
        }
        
        guard !referenceImages.isEmpty else {
            statusMessage = "No images to track"
            dlog("⚠️ [MarkerDetection] No reference images to track")
            return
        }
        
        let markerCount = referenceImages.count - customStorage.registrations.count
        let customCount = customStorage.registrations.count
        dlog("🎯 [MarkerDetection] Total reference images: \(referenceImages.count) (\(markerCount) markers, \(customCount) custom)")
        
        // Check if image tracking is supported
        guard ImageTrackingProvider.isSupported else {
            statusMessage = "Image tracking not supported"
            dlog("❌ [MarkerDetection] Image tracking not supported on this device")
            return
        }
        
        dlog("🎯 [MarkerDetection] Image tracking is supported, creating session...")
        
        do {
            arkitSession = ARKitSession()
            imageTrackingProvider = ImageTrackingProvider(referenceImages: referenceImages)
            
            // Check authorization status
            let authStatus = await arkitSession!.queryAuthorization(for: [.worldSensing])
            dlog("🎯 [MarkerDetection] Authorization status: \(authStatus)")
            
            try await arkitSession?.run([imageTrackingProvider!])
            
            isRunning = true
            statusMessage = "Detecting..."
            dlog("✅ [MarkerDetection] Detection started with \(referenceImages.count) markers")
            dlog("✅ [MarkerDetection] ImageTrackingProvider state: \(imageTrackingProvider?.state ?? .stopped)")
            
            // Start processing anchor updates
            Task {
                await processAnchorUpdates()
            }
            
            // NOTE: Cleanup loop removed - markers now persist until manually deleted
            
        } catch {
            statusMessage = "Failed to start: \(error.localizedDescription)"
            dlog("❌ [MarkerDetection] Failed to start: \(error)")
            dlog("❌ [MarkerDetection] Error details: \(String(describing: error))")
        }
    }
    
    /// Stop marker detection
    func stopDetection() {
        arkitSession = nil
        imageTrackingProvider = nil
        detectedMarkers = [:]
        // Keep fixedMarkers - they persist across detection restarts
        announcedMarkers = []  // Reset so sounds play again next time
        isRunning = false
        statusMessage = "Stopped"
        updateSnapshot()
        dlog("🎯 [MarkerDetection] Detection stopped")
    }
    
    // MARK: - Marker Management
    
    /// Fix or unfix a marker's pose
    /// When fixed, the marker's current pose is preserved and won't update from tracking
    func setMarkerFixed(_ markerId: Int, fixed: Bool) {
        if fixed {
            // Take the marker from detectedMarkers and add to fixedMarkers
            if let marker = detectedMarkers[markerId] {
                fixedMarkers[markerId] = marker
                dlog("🎯 [MarkerDetection] Fixed marker \(markerId) at position \(marker.position)")
            }
        } else {
            // Remove from fixedMarkers - it will update from tracking again
            fixedMarkers.removeValue(forKey: markerId)
            dlog("🎯 [MarkerDetection] Unfixed marker \(markerId)")
        }
        updateSnapshot()
    }
    
    /// Check if a marker is fixed
    func isMarkerFixed(_ markerId: Int) -> Bool {
        return fixedMarkers[markerId] != nil
    }
    
    /// Delete a marker completely (both detected and fixed)
    func deleteMarker(_ markerId: Int) {
        detectedMarkers.removeValue(forKey: markerId)
        fixedMarkers.removeValue(forKey: markerId)
        announcedMarkers.remove(markerId)
        updateSnapshot()
        statusMessage = "Tracking \(detectedMarkers.count + fixedMarkers.count) marker(s)"
        dlog("🎯 [MarkerDetection] Deleted marker \(markerId)")
    }
    
    // MARK: - Reference Image Generation
    
    private func generateReferenceImage(markerId: Int, dictionary: ArucoDictionaryType) -> ReferenceImage? {
        guard let imageData = OpenCVArucoDetector.generateMarkerImage(
            id: Int32(markerId),
            sizePixels: 1000,
            dictionary: dictionary
        ) else {
            dlog("❌ [MarkerDetection] Failed to generate marker \(markerId)")
            return nil
        }
        
        guard let cgImageSource = CGImageSourceCreateWithData(imageData as CFData, nil),
              let cgImage = CGImageSourceCreateImageAtIndex(cgImageSource, 0, nil) else {
            dlog("❌ [MarkerDetection] Failed to create CGImage for marker \(markerId)")
            return nil
        }
        
        // Use user-selected marker size for reference image
        var refImage = ReferenceImage(
            cgimage: cgImage,
            physicalSize: CGSize(width: CGFloat(markerSizeMeters), height: CGFloat(markerSizeMeters)),
            orientation: .up
        )
        // Encode both dictionary and marker ID in the name
        refImage.name = "marker_\(dictionary.rawValue)_\(markerId)"
        
        return refImage
    }
    
    /// Generate a ReferenceImage from a custom user-uploaded image
    private func generateCustomReferenceImage(registration: CustomImageRegistration) -> ReferenceImage? {
        guard let uiImage = CustomImageStorage.shared.loadImage(for: registration),
              let cgImage = uiImage.cgImage else {
            dlog("❌ [MarkerDetection] Failed to load custom image '\(registration.name)'")
            return nil
        }
        
        // Use the user-specified physical width (assume square for now)
        let physicalSize = CGFloat(registration.physicalWidthMeters)
        var refImage = ReferenceImage(
            cgimage: cgImage,
            physicalSize: CGSize(width: physicalSize, height: physicalSize),
            orientation: .up
        )
        // Encode custom image ID in the name with "custom_" prefix
        refImage.name = "custom_\(registration.id)"
        
        return refImage
    }
    
    // MARK: - Anchor Processing
    
    private func processAnchorUpdates() async {
        guard let provider = imageTrackingProvider else { return }
        
        dlog("🎯 [MarkerDetection] Started listening for image tracking updates")
        
        for await update in provider.anchorUpdates {
            let anchor = update.anchor
            
            guard let name = anchor.referenceImage.name else { continue }
            
            // Handle ArUco markers: "marker_<dict>_<id>"
            if name.hasPrefix("marker_") {
                let parts = name.dropFirst(7).split(separator: "_")
                guard parts.count == 2,
                      let dictValue = Int(parts[0]),
                      let markerId = Int(parts[1]) else { continue }
                
                await MainActor.run {
                    if anchor.isTracked {
                        let isNewMarker = detectedMarkers[markerId] == nil
                        
                        // Calculate estimated marker size using estimatedScaleFactor
                        let estimatedSize = markerSizeMeters * Float(anchor.estimatedScaleFactor)
                        
                        let marker = DetectedMarker(
                            id: markerId,
                            dictionaryType: dictValue,
                            poseInWorld: anchor.originFromAnchorTransform,
                            estimatedSizeMeters: estimatedSize,
                            isTracked: true,
                            timestamp: Date()
                        )
                        
                        // Only update if marker is NOT fixed (fixed markers keep their frozen pose)
                        if fixedMarkers[markerId] == nil {
                            detectedMarkers[markerId] = marker
                        }
                        
                        let totalMarkers = Set(detectedMarkers.keys).union(fixedMarkers.keys).count
                        let totalCustom = Set(trackedCustomImages.keys).union(fixedCustomImages.keys).count
                        statusMessage = "Tracking \(totalMarkers) marker(s), \(totalCustom) image(s)"
                        
                        // Play sound for newly detected markers
                        if isNewMarker && !announcedMarkers.contains(markerId) {
                            announcedMarkers.insert(markerId)
                            playDetectionSound()
                            dlog("🎯 [MarkerDetection] 🔊 New marker ID \(markerId), estimated size: \(String(format: "%.1f", estimatedSize * 100))cm")
                        }
                    } else {
                        // Tracking lost - update isTracked to false but keep the marker (if not fixed)
                        if fixedMarkers[markerId] == nil, var existingMarker = detectedMarkers[markerId] {
                            existingMarker.isTracked = false
                            detectedMarkers[markerId] = existingMarker
                        }
                    }
                    
                    updateSnapshot()
                }
            }
            // Handle custom images: "custom_<uuid>"
            else if name.hasPrefix("custom_") {
                let imageId = String(name.dropFirst(7))
                
                // Find the registration to get name and physical width
                guard let registration = CustomImageStorage.shared.registrations.first(where: { $0.id == imageId }) else {
                    continue
                }
                
                await MainActor.run {
                    if anchor.isTracked {
                        let isNewImage = trackedCustomImages[imageId] == nil
                        
                        let physicalWidth = registration.physicalWidthMeters * Float(anchor.estimatedScaleFactor)
                        
                        let trackedImage = TrackedCustomImage(
                            id: imageId,
                            name: registration.name,
                            poseInWorld: anchor.originFromAnchorTransform,
                            physicalWidthMeters: physicalWidth,
                            isTracked: true,
                            timestamp: Date()
                        )
                        
                        // Only update if image is NOT fixed
                        if fixedCustomImages[imageId] == nil {
                            trackedCustomImages[imageId] = trackedImage
                        }
                        
                        let totalMarkers = Set(detectedMarkers.keys).union(fixedMarkers.keys).count
                        let totalCustom = Set(trackedCustomImages.keys).union(fixedCustomImages.keys).count
                        statusMessage = "Tracking \(totalMarkers) marker(s), \(totalCustom) image(s)"
                        
                        // Play sound for newly detected custom images
                        if isNewImage && !announcedCustomImages.contains(imageId) {
                            announcedCustomImages.insert(imageId)
                            playDetectionSound()
                            dlog("📷 [MarkerDetection] 🔊 New custom image '\(registration.name)'")
                        }
                    } else {
                        // Tracking lost
                        if fixedCustomImages[imageId] == nil, var existingImage = trackedCustomImages[imageId] {
                            existingImage.isTracked = false
                            trackedCustomImages[imageId] = existingImage
                        }
                    }
                    
                    updateSnapshot()
                }
            }
        }
        
        dlog("🎯 [MarkerDetection] Stopped listening for image tracking updates")
    }
    
    // MARK: - Audio Feedback
    
    /// Play a sound when a new marker is detected
    private func playDetectionSound() {
        // Use system sound for a subtle "found" feedback
        // Sound ID 1057 = "Tock" (short, subtle)
        // Sound ID 1103 = "Camera shutter" (more noticeable)
        // Sound ID 1113 = "PaymentSuccess" (pleasant)
        AudioServicesPlaySystemSound(1057)
    }
    
    // MARK: - Data Streaming
    
    /// Get marker data formatted for streaming via gRPC
    /// Returns nil if no markers detected or detection disabled
    func getMarkerDataForStream() -> Data? {
        // Merge detected and fixed markers for streaming
        var allMarkers: [Int: DetectedMarker] = detectedMarkers
        for (id, marker) in fixedMarkers {
            allMarkers[id] = marker  // Fixed markers override detected ones
        }
        
        guard isEnabled, !allMarkers.isEmpty else { return nil }
        
        // Build compact JSON: {"m": [[id, dict, x, y, z, qx, qy, qz, qw, isFixed], ...]}
        var markers: [[Any]] = []
        
        for (_, marker) in allMarkers {
            let pos = marker.position
            let quat = marker.quaternion
            let isFixed = fixedMarkers[marker.id] != nil
            markers.append([
                marker.id,
                marker.dictionaryType,
                Double(pos.x), Double(pos.y), Double(pos.z),
                Double(quat.x), Double(quat.y), Double(quat.z), Double(quat.w),  // xyzw order for Python
                isFixed ? 1.0 : 0.0  // is_fixed flag
            ])
        }
        
        let json: [String: Any] = ["m": markers]
        
        do {
            let jsonData = try JSONSerialization.data(withJSONObject: json, options: [])
            return jsonData
        } catch {
            dlog("❌ [MarkerDetection] Failed to serialize marker data: \(error)")
            return nil
        }
    }
}
