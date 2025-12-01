import SwiftUI
import Foundation
import Combine
import simd
import UniformTypeIdentifiers
import AVFoundation
import VideoToolbox

// MARK: - Recording Data Structures

/// A single frame of recorded data containing tracking and video
struct RecordedFrame: Codable {
    let timestamp: Double  // Seconds since recording started
    let systemTime: Double  // Unix timestamp for synchronization
    
    // Head tracking (4x4 matrix flattened)
    let headMatrix: [Float]?
    
    // Hand tracking data
    let leftHand: HandJointData?
    let rightHand: HandJointData?
    
    // Video frame metadata
    let videoFrameIndex: Int  // Index into video frames
    let videoWidth: Int
    let videoHeight: Int
}

/// Hand joint positions for all 26 joints
struct HandJointData: Codable {
    let wrist: [Float]  // 4x4 matrix flattened
    let thumbCMC: [Float]
    let thumbMP: [Float]
    let thumbIP: [Float]
    let thumbTip: [Float]
    let indexMCP: [Float]
    let indexPIP: [Float]
    let indexDIP: [Float]
    let indexTip: [Float]
    let middleMCP: [Float]
    let middlePIP: [Float]
    let middleDIP: [Float]
    let middleTip: [Float]
    let ringMCP: [Float]
    let ringPIP: [Float]
    let ringDIP: [Float]
    let ringTip: [Float]
    let littleMCP: [Float]
    let littlePIP: [Float]
    let littleDIP: [Float]
    let littleTip: [Float]
    let forearmWrist: [Float]?
    let forearmArm: [Float]?
}

/// Metadata for the entire recording session
struct RecordingMetadata: Codable {
    let version: String = "1.0"
    let createdAt: Date
    let duration: Double  // Total duration in seconds
    let frameCount: Int
    let hasVideo: Bool
    let hasLeftHand: Bool
    let hasRightHand: Bool
    let hasHead: Bool
    let videoSource: String  // "network" or "uvc"
    let averageFPS: Double
    let deviceInfo: DeviceInfo
}

struct DeviceInfo: Codable {
    let model: String
    let systemVersion: String
    let appVersion: String
}

/// Storage location options
enum RecordingStorageLocation: String, CaseIterable {
    case local = "Local Storage"
    case iCloudDrive = "iCloud Drive"
    case documentsFolder = "Documents"
    
    var icon: String {
        switch self {
        case .local: return "internaldrive"
        case .iCloudDrive: return "icloud"
        case .documentsFolder: return "folder"
        }
    }
}

// MARK: - Video Frame for storage
struct VideoFrameData {
    let index: Int
    let timestamp: Double
    let image: UIImage
    let presentationTime: CMTime
}

// MARK: - Recording Manager

/// Manages synchronized recording of tracking data and video frames.
/// Recording is VIDEO-DRIVEN: each new video frame triggers a recording of the latest tracking data.
/// Auto-recording: Recording starts automatically when video frames arrive and stops when disconnected.
@MainActor
class RecordingManager: ObservableObject {
    static let shared = RecordingManager()
    
    // MARK: - Published Properties
    @Published var isRecording: Bool = false
    @Published var recordingDuration: TimeInterval = 0
    @Published var frameCount: Int = 0
    @Published var storageLocation: RecordingStorageLocation {
        didSet {
            UserDefaults.standard.set(storageLocation.rawValue, forKey: "recordingStorageLocation")
        }
    }
    @Published var lastRecordingURL: URL? = nil
    @Published var recordingError: String? = nil
    @Published var isSaving: Bool = false
    
    // Auto-recording state
    @Published var autoRecordingEnabled: Bool {
        didSet {
            UserDefaults.standard.set(autoRecordingEnabled, forKey: "autoRecordingEnabled")
        }
    }
    @Published var isAutoRecording: Bool = false  // True if current recording was auto-started
    
    // MARK: - Private Properties
    private var recordingStartTime: Date?
    private var durationTimer: Timer?
    private var sessionID: String = ""
    private let recordingQueue = DispatchQueue(label: "com.visionproteleop.recording", qos: .userInitiated)
    private let videoWriterQueue = DispatchQueue(label: "com.visionproteleop.videowriter", qos: .userInitiated)
    
    // Frame data (accessed from recordingQueue - use nonisolated(unsafe) for background access)
    nonisolated(unsafe) private var recordedFrames: [RecordedFrame] = []
    nonisolated(unsafe) private var videoFrames: [VideoFrameData] = []
    nonisolated(unsafe) private var pendingFrameCount: Int = 0
    
    // Recording settings
    var videoQuality: CGFloat = 0.7  // JPEG compression quality (0.0-1.0)
    nonisolated(unsafe) var videoBitRate: Int = 10_000_000  // 10 Mbps for H.264 encoding
    
    // Video writer components
    nonisolated(unsafe) private var assetWriter: AVAssetWriter?
    nonisolated(unsafe) private var videoWriterInput: AVAssetWriterInput?
    nonisolated(unsafe) private var pixelBufferAdaptor: AVAssetWriterInputPixelBufferAdaptor?
    nonisolated(unsafe) private var videoSize: CGSize = .zero
    nonisolated(unsafe) private var isWriterSessionStarted: Bool = false
    nonisolated(unsafe) private var pixelBufferPool: CVPixelBufferPool?
    nonisolated(unsafe) private var recordingFolderURL: URL?
    
    // MARK: - Initialization
    
    private init() {
        // Load saved storage location
        if let savedLocation = UserDefaults.standard.string(forKey: "recordingStorageLocation"),
           let location = RecordingStorageLocation(rawValue: savedLocation) {
            self.storageLocation = location
        } else {
            self.storageLocation = .documentsFolder
        }
        
        // Load auto-recording preference (default to true for auto-record by default)
        self.autoRecordingEnabled = UserDefaults.standard.object(forKey: "autoRecordingEnabled") as? Bool ?? true
    }
    
    // MARK: - Auto-Recording Control
    
    /// Called when first video frame is received. Starts recording if auto-recording is enabled.
    /// Video source can be UVC camera or network stream.
    func onFirstVideoFrame() {
        guard autoRecordingEnabled && !isRecording else { return }
        
        print("🔴 [RecordingManager] Auto-starting recording on first video frame")
        isAutoRecording = true
        startRecording()
    }
    
    /// Called when video source is disconnected (UVC camera unplugged, Python client disconnected, or WebRTC disconnected).
    /// Stops recording if it was auto-started.
    func onVideoSourceDisconnected(reason: String) {
        guard isRecording else { return }
        
        print("🔴 [RecordingManager] Stopping recording due to: \(reason)")
        stopRecording()
        isAutoRecording = false
    }
    
    /// Explicitly stop recording (user action). This also clears auto-recording state.
    func stopRecordingManually() {
        guard isRecording else { return }
        
        print("🔴 [RecordingManager] User manually stopped recording")
        stopRecording()
        isAutoRecording = false
    }
    
    // MARK: - Recording Control
    
    func startRecording() {
        guard !isRecording else { return }
        
        print("🔴 [RecordingManager] Starting recording (video-driven mode)...")
        
        // Reset state on background queue to avoid blocking
        recordingQueue.async { [weak self] in
            self?.recordedFrames.removeAll()
            self?.videoFrames.removeAll()
            self?.pendingFrameCount = 0
            self?.isWriterSessionStarted = false
            self?.videoSize = .zero
        }
        
        recordingStartTime = Date()
        frameCount = 0
        recordingDuration = 0
        recordingError = nil
        
        // Generate session ID
        let formatter = DateFormatter()
        formatter.dateFormat = "yyyyMMdd_HHmmss"
        sessionID = "recording_\(formatter.string(from: Date()))"
        
        isRecording = true
        
        // Start duration timer - also updates frame count periodically
        durationTimer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
            Task { @MainActor in
                guard let self = self, let startTime = self.recordingStartTime else { return }
                self.recordingDuration = Date().timeIntervalSince(startTime)
                self.frameCount = self.pendingFrameCount
            }
        }
        
        print("🔴 [RecordingManager] Recording started with session ID: \(sessionID)")
    }
    
    /// Set up the video writer for MP4 output
    nonisolated private func setupVideoWriter(at url: URL, size: CGSize) throws {
        // Clean up any existing file
        try? FileManager.default.removeItem(at: url)
        
        let writer = try AVAssetWriter(outputURL: url, fileType: .mp4)
        
        // Use slightly lower settings for better real-time performance
        // especially when running with debugger attached
        let videoSettings: [String: Any] = [
            AVVideoCodecKey: AVVideoCodecType.h264,
            AVVideoWidthKey: Int(size.width),
            AVVideoHeightKey: Int(size.height),
            AVVideoCompressionPropertiesKey: [
                AVVideoAverageBitRateKey: videoBitRate,
                AVVideoProfileLevelKey: AVVideoProfileLevelH264BaselineAutoLevel,  // Baseline is faster to encode
                AVVideoExpectedSourceFrameRateKey: 30,
                AVVideoAllowFrameReorderingKey: false,  // No B-frames = lower latency
                AVVideoMaxKeyFrameIntervalKey: 30  // Keyframe every second
            ]
        ]
        
        let writerInput = AVAssetWriterInput(mediaType: .video, outputSettings: videoSettings)
        writerInput.expectsMediaDataInRealTime = true
        
        // Pixel buffer attributes with pool for efficiency
        let sourcePixelBufferAttributes: [String: Any] = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA,
            kCVPixelBufferWidthKey as String: Int(size.width),
            kCVPixelBufferHeightKey as String: Int(size.height),
            kCVPixelBufferIOSurfacePropertiesKey as String: [:] as [String: Any]
        ]
        
        let adaptor = AVAssetWriterInputPixelBufferAdaptor(
            assetWriterInput: writerInput,
            sourcePixelBufferAttributes: sourcePixelBufferAttributes
        )
        
        if writer.canAdd(writerInput) {
            writer.add(writerInput)
        } else {
            throw RecordingError.videoWriterSetupFailed
        }
        
        guard writer.startWriting() else {
            if let error = writer.error {
                print("❌ [RecordingManager] Writer failed to start: \(error)")
            }
            throw RecordingError.videoWriterSetupFailed
        }
        
        self.assetWriter = writer
        self.videoWriterInput = writerInput
        self.pixelBufferAdaptor = adaptor
        self.videoSize = size
        
        // Get the pixel buffer pool from the adaptor (will be available after starting session)
        
        print("🎬 [RecordingManager] Video writer set up for \(Int(size.width))x\(Int(size.height))")
    }
    
    /// Create a pixel buffer from a UIImage using pool if available
    nonisolated private func createPixelBuffer(from image: UIImage) -> CVPixelBuffer? {
        guard let cgImage = image.cgImage else { return nil }
        
        let width = cgImage.width
        let height = cgImage.height
        
        var pixelBuffer: CVPixelBuffer?
        
        // Try to use the adaptor's pool first (more efficient)
        if let pool = pixelBufferAdaptor?.pixelBufferPool {
            let status = CVPixelBufferPoolCreatePixelBuffer(kCFAllocatorDefault, pool, &pixelBuffer)
            if status != kCVReturnSuccess {
                pixelBuffer = nil
            }
        }
        
        // Fall back to creating a new buffer if pool isn't available
        if pixelBuffer == nil {
            let attributes: [String: Any] = [
                kCVPixelBufferCGImageCompatibilityKey as String: true,
                kCVPixelBufferCGBitmapContextCompatibilityKey as String: true,
                kCVPixelBufferWidthKey as String: width,
                kCVPixelBufferHeightKey as String: height,
                kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA
            ]
            
            let status = CVPixelBufferCreate(
                kCFAllocatorDefault,
                width,
                height,
                kCVPixelFormatType_32BGRA,
                attributes as CFDictionary,
                &pixelBuffer
            )
            
            guard status == kCVReturnSuccess else { return nil }
        }
        
        guard let buffer = pixelBuffer else { return nil }
        
        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        
        guard let context = CGContext(
            data: CVPixelBufferGetBaseAddress(buffer),
            width: width,
            height: height,
            bitsPerComponent: 8,
            bytesPerRow: CVPixelBufferGetBytesPerRow(buffer),
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue
        ) else {
            return nil
        }
        
        context.draw(cgImage, in: CGRect(x: 0, y: 0, width: width, height: height))
        
        return buffer
    }
    
    func stopRecording() {
        guard isRecording else { return }
        
        print("🔴 [RecordingManager] Stopping recording...")
        
        isRecording = false
        durationTimer?.invalidate()
        durationTimer = nil
        
        // Sync final count
        frameCount = pendingFrameCount
        
        // Calculate final duration
        if let startTime = recordingStartTime {
            recordingDuration = Date().timeIntervalSince(startTime)
        }
        
        print("🔴 [RecordingManager] Recording stopped.")
        print("   Duration: \(String(format: "%.1f", recordingDuration))s")
        print("   Frames: \(frameCount) (~\(String(format: "%.0f", Double(frameCount) / max(recordingDuration, 0.1))) fps)")
        
        // Save the recording
        Task {
            await saveRecording()
        }
    }
    
    // MARK: - Frame Recording (Video-Driven)
    
    /// Record a video frame with the current tracking data.
    /// This is VIDEO-DRIVEN: call this whenever a new video frame arrives.
    /// The latest tracking data is captured and paired with this video frame.
    nonisolated func recordVideoFrame(_ videoFrame: UIImage) {
        // Capture time immediately
        let captureTime = Date()
        
        // Dispatch to background immediately to avoid blocking
        Task.detached(priority: .userInitiated) { [weak self] in
            await self?.processVideoFrame(videoFrame, captureTime: captureTime)
        }
    }
    
    /// Process a video frame on the background queue
    private func processVideoFrame(_ videoFrame: UIImage, captureTime: Date) async {
        guard isRecording, let startTime = recordingStartTime else { return }
        
        let timestamp = captureTime.timeIntervalSince(startTime)
        let systemTime = captureTime.timeIntervalSince1970
        
        // Capture the LATEST tracking data at this moment
        let trackingData = DataManager.shared.latestHandTrackingData
        
        // Get image dimensions
        let width = Int(videoFrame.size.width)
        let height = Int(videoFrame.size.height)
        
        // Do the rest on recording queue
        recordingQueue.async { [weak self] in
            guard let self = self else { return }
            
            let frameIndex = self.pendingFrameCount
            
            // Set up video writer on first frame
            if !self.isWriterSessionStarted {
                do {
                    let baseURL = try self.getStorageURLSync()
                    let recordingFolder = baseURL.appendingPathComponent(self.sessionID)
                    try FileManager.default.createDirectory(at: recordingFolder, withIntermediateDirectories: true)
                    self.recordingFolderURL = recordingFolder
                    
                    let videoURL = recordingFolder.appendingPathComponent("video.mp4")
                    try self.setupVideoWriter(at: videoURL, size: videoFrame.size)
                    self.assetWriter?.startSession(atSourceTime: .zero)
                    self.isWriterSessionStarted = true
                    print("🎬 [RecordingManager] Video writer session started")
                } catch {
                    print("❌ [RecordingManager] Failed to set up video writer: \(error)")
                    return
                }
            }
            
            // Create presentation time based on actual timestamp for accurate timing
            let presentationTime = CMTime(seconds: timestamp, preferredTimescale: 600)
            
            // Write video frame - skip if writer isn't ready (don't block!)
            if let writer = self.assetWriter,
               let writerInput = self.videoWriterInput,
               let adaptor = self.pixelBufferAdaptor,
               writer.status == .writing,
               writerInput.isReadyForMoreMediaData {
                
                if let pixelBuffer = self.createPixelBuffer(from: videoFrame) {
                    if !adaptor.append(pixelBuffer, withPresentationTime: presentationTime) {
                        print("⚠️ [RecordingManager] Failed to append frame \(frameIndex)")
                    }
                }
            } else if self.assetWriter?.status == .writing {
                // Writer not ready, skip this frame but log occasionally
                if frameIndex % 30 == 0 {
                    print("⚠️ [RecordingManager] Writer busy, skipping frame \(frameIndex)")
                }
            }
            
            // Convert tracking data
            let headMatrixArray: [Float] = self.matrixToArray(trackingData.Head)
            let leftHand = self.extractHandJointData(wrist: trackingData.leftWrist, skeleton: trackingData.leftSkeleton)
            let rightHand = self.extractHandJointData(wrist: trackingData.rightWrist, skeleton: trackingData.rightSkeleton)
            
            // Create recorded frame (without video data - that's in the MP4)
            let recordedFrame = RecordedFrame(
                timestamp: timestamp,
                systemTime: systemTime,
                headMatrix: headMatrixArray,
                leftHand: leftHand,
                rightHand: rightHand,
                videoFrameIndex: frameIndex,
                videoWidth: width,
                videoHeight: height
            )
            
            // Append to tracking data array
            self.recordedFrames.append(recordedFrame)
            self.pendingFrameCount += 1
        }
    }
    
    // MARK: - Hand Data Extraction
    
    nonisolated private func matrixToArray(_ m: simd_float4x4) -> [Float] {
        [
            m.columns.0.x, m.columns.0.y, m.columns.0.z, m.columns.0.w,
            m.columns.1.x, m.columns.1.y, m.columns.1.z, m.columns.1.w,
            m.columns.2.x, m.columns.2.y, m.columns.2.z, m.columns.2.w,
            m.columns.3.x, m.columns.3.y, m.columns.3.z, m.columns.3.w
        ]
    }
    
    nonisolated private func extractHandJointData(wrist: simd_float4x4, skeleton: Skeleton) -> HandJointData? {
        let joints = skeleton.joints
        guard joints.count >= 21 else { return nil }
        
        return HandJointData(
            wrist: matrixToArray(wrist),
            thumbCMC: matrixToArray(joints[1]),
            thumbMP: matrixToArray(joints[2]),
            thumbIP: matrixToArray(joints[3]),
            thumbTip: matrixToArray(joints[4]),
            indexMCP: matrixToArray(joints[5]),
            indexPIP: matrixToArray(joints[6]),
            indexDIP: matrixToArray(joints[7]),
            indexTip: matrixToArray(joints[8]),
            middleMCP: matrixToArray(joints[9]),
            middlePIP: matrixToArray(joints[10]),
            middleDIP: matrixToArray(joints[11]),
            middleTip: matrixToArray(joints[12]),
            ringMCP: matrixToArray(joints[13]),
            ringPIP: matrixToArray(joints[14]),
            ringDIP: matrixToArray(joints[15]),
            ringTip: matrixToArray(joints[16]),
            littleMCP: matrixToArray(joints[17]),
            littlePIP: matrixToArray(joints[18]),
            littleDIP: matrixToArray(joints[19]),
            littleTip: matrixToArray(joints[20]),
            forearmWrist: joints.count > 21 ? matrixToArray(joints[21]) : nil,
            forearmArm: joints.count > 22 ? matrixToArray(joints[22]) : nil
        )
    }
    
    // MARK: - Saving
    
    private func saveRecording() async {
        // Wait for recording queue to finish processing
        await withCheckedContinuation { (continuation: CheckedContinuation<Void, Never>) in
            recordingQueue.async {
                continuation.resume()
            }
        }
        
        guard !recordedFrames.isEmpty else {
            recordingError = "No frames recorded"
            return
        }
        
        isSaving = true
        defer { isSaving = false }
        
        print("💾 [RecordingManager] Saving recording to \(storageLocation.rawValue)...")
        print("   Frames to save: \(recordedFrames.count)")
        
        do {
            // Use the folder we already created during recording
            let recordingFolder: URL
            if let existingFolder = recordingFolderURL {
                recordingFolder = existingFolder
            } else {
                let baseURL = try getStorageURL()
                recordingFolder = baseURL.appendingPathComponent(sessionID)
                try FileManager.default.createDirectory(at: recordingFolder, withIntermediateDirectories: true)
            }
            
            // Finalize video file
            if let writer = assetWriter, writer.status == .writing {
                videoWriterInput?.markAsFinished()
                
                await withCheckedContinuation { (continuation: CheckedContinuation<Void, Never>) in
                    writer.finishWriting {
                        continuation.resume()
                    }
                }
                
                if writer.status == .completed {
                    let videoURL = recordingFolder.appendingPathComponent("video.mp4")
                    if let attributes = try? FileManager.default.attributesOfItem(atPath: videoURL.path),
                       let fileSize = attributes[.size] as? Int64 {
                        let sizeMB = Double(fileSize) / (1024 * 1024)
                        print("🎬 [RecordingManager] Video saved: \(String(format: "%.1f", sizeMB)) MB")
                    }
                } else if let error = writer.error {
                    print("❌ [RecordingManager] Video writer error: \(error)")
                }
            }
            
            // Clean up writer references
            assetWriter = nil
            videoWriterInput = nil
            pixelBufferAdaptor = nil
            pixelBufferPool = nil
            isWriterSessionStarted = false
            recordingFolderURL = nil
            
            // Save metadata
            let metadata = RecordingMetadata(
                createdAt: recordingStartTime ?? Date(),
                duration: recordingDuration,
                frameCount: frameCount,
                hasVideo: true,
                hasLeftHand: recordedFrames.contains { $0.leftHand != nil },
                hasRightHand: recordedFrames.contains { $0.rightHand != nil },
                hasHead: recordedFrames.contains { $0.headMatrix != nil },
                videoSource: DataManager.shared.videoSource.rawValue,
                averageFPS: recordingDuration > 0 ? Double(frameCount) / recordingDuration : 0,
                deviceInfo: DeviceInfo(
                    model: "Apple Vision Pro",
                    systemVersion: UIDevice.current.systemVersion,
                    appVersion: Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "1.0"
                )
            )
            
            let metadataURL = recordingFolder.appendingPathComponent("metadata.json")
            let encoder = JSONEncoder()
            encoder.outputFormatting = .prettyPrinted
            let metadataData = try encoder.encode(metadata)
            try metadataData.write(to: metadataURL)
            
            // Save tracking data (JSON Lines format)
            let trackingURL = recordingFolder.appendingPathComponent("tracking.jsonl")
            var trackingContent = ""
            let lineEncoder = JSONEncoder()
            for frame in recordedFrames {
                let frameData = try lineEncoder.encode(frame)
                if let jsonString = String(data: frameData, encoding: .utf8) {
                    trackingContent += jsonString + "\n"
                }
            }
            try trackingContent.write(to: trackingURL, atomically: true, encoding: .utf8)
            
            lastRecordingURL = recordingFolder
            recordingError = nil
            
            print("✅ [RecordingManager] Recording saved successfully to: \(recordingFolder.path)")
            
            // Clear memory
            recordedFrames.removeAll()
            videoFrames.removeAll()
            
        } catch {
            recordingError = "Failed to save: \(error.localizedDescription)"
            print("❌ [RecordingManager] Failed to save recording: \(error)")
            
            // Clean up writer on error
            assetWriter = nil
            videoWriterInput = nil
            pixelBufferAdaptor = nil
            pixelBufferPool = nil
            isWriterSessionStarted = false
            recordingFolderURL = nil
        }
    }
    
    /// Synchronous version of getStorageURL for use on background queue
    nonisolated private func getStorageURLSync() throws -> URL {
        let fileManager = FileManager.default
        
        // Read storage location from UserDefaults directly since we're nonisolated
        let savedLocation = UserDefaults.standard.string(forKey: "recordingStorageLocation") ?? RecordingStorageLocation.documentsFolder.rawValue
        let location = RecordingStorageLocation(rawValue: savedLocation) ?? .documentsFolder
        
        switch location {
        case .local:
            guard let url = fileManager.urls(for: .cachesDirectory, in: .userDomainMask).first else {
                throw RecordingError.storageNotAvailable
            }
            return url.appendingPathComponent("Recordings")
            
        case .documentsFolder:
            guard let url = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first else {
                throw RecordingError.storageNotAvailable
            }
            return url.appendingPathComponent("Recordings")
            
        case .iCloudDrive:
            guard let containerURL = fileManager.url(forUbiquityContainerIdentifier: nil) else {
                guard let url = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first else {
                    throw RecordingError.storageNotAvailable
                }
                return url.appendingPathComponent("Recordings")
            }
            return containerURL.appendingPathComponent("Documents").appendingPathComponent("VisionProTeleop")
        }
    }
    
    private func getStorageURL() throws -> URL {
        let fileManager = FileManager.default
        
        switch storageLocation {
        case .local:
            guard let url = fileManager.urls(for: .cachesDirectory, in: .userDomainMask).first else {
                throw RecordingError.storageNotAvailable
            }
            return url.appendingPathComponent("Recordings")
            
        case .documentsFolder:
            guard let url = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first else {
                throw RecordingError.storageNotAvailable
            }
            return url.appendingPathComponent("Recordings")
            
        case .iCloudDrive:
            // Use the app's iCloud container - syncs to Mac but in ~/Library/Mobile Documents/
            // On Mac, access via: ~/Library/Mobile Documents/iCloud~com~younghyopark~VisionProTeleop/
            // Or create a symlink to make it visible in iCloud Drive
            guard let containerURL = fileManager.url(forUbiquityContainerIdentifier: nil) else {
                print("⚠️ [RecordingManager] iCloud not available, falling back to Documents...")
                guard let url = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first else {
                    throw RecordingError.storageNotAvailable
                }
                return url.appendingPathComponent("Recordings")
            }
            return containerURL.appendingPathComponent("Documents").appendingPathComponent("VisionProTeleop")
        }
    }
    
    // MARK: - Utility
    
    func formatDuration(_ duration: TimeInterval) -> String {
        let hours = Int(duration) / 3600
        let minutes = (Int(duration) % 3600) / 60
        let seconds = Int(duration) % 60
        let tenths = Int((duration.truncatingRemainder(dividingBy: 1)) * 10)
        
        if hours > 0 {
            return String(format: "%d:%02d:%02d", hours, minutes, seconds)
        } else {
            return String(format: "%d:%02d.%d", minutes, seconds, tenths)
        }
    }
    
    /// Get the current storage directory URL (for display purposes)
    func getCurrentStorageURL() -> URL? {
        return try? getStorageURL()
    }
    
    /// Open the Files app at the recordings folder
    func openRecordingsInFilesApp() {
        let urlToOpen: URL
        
        if let lastURL = lastRecordingURL {
            urlToOpen = lastURL
        } else if let storageURL = try? getStorageURL() {
            urlToOpen = storageURL
        } else {
            print("❌ [RecordingManager] Cannot determine storage location")
            return
        }
        
        let path = urlToOpen.path
        
        if let filesURL = URL(string: "shareddocuments://\(path)") {
            Task { @MainActor in
                if await UIApplication.shared.canOpenURL(filesURL) {
                    await UIApplication.shared.open(filesURL)
                    print("📂 [RecordingManager] Opened Files app at: \(path)")
                } else {
                    print("⚠️ [RecordingManager] Cannot open shareddocuments URL, path: \(path)")
                    UIPasteboard.general.string = path
                    print("📋 [RecordingManager] Path copied to clipboard: \(path)")
                }
            }
        }
    }
    
    /// Get a user-friendly description of where recordings are stored
    func getStorageDescription() -> String {
        switch storageLocation {
        case .local:
            return "App Cache (internal)"
        case .documentsFolder:
            return "On My Vision Pro → Tracking Streamer → Recordings"
        case .iCloudDrive:
            return "iCloud Drive → VisionProTeleop"
        }
    }
}

// MARK: - Errors

enum RecordingError: Error, LocalizedError {
    case storageNotAvailable
    case encodingFailed
    case writeFailed
    case videoWriterSetupFailed
    
    var errorDescription: String? {
        switch self {
        case .storageNotAvailable:
            return "Storage location is not available"
        case .encodingFailed:
            return "Failed to encode recording data"
        case .writeFailed:
            return "Failed to write recording to disk"
        case .videoWriterSetupFailed:
            return "Failed to set up video writer"
        }
    }
}
