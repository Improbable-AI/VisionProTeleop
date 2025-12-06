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

/// A single frame of simulation data
struct SimulationFrame: Codable {
    let timestamp: Double  // Seconds since recording started
    let poses: [String: [Float]]
    let qpos: [Float]?
    let ctrl: [Float]?
}

/// Hand joint positions for all 27 joints tracked by ARKit HandSkeleton
/// Joint order matches 🥽AppModel.swift jointTypes array:
///   0: forearmArm
///   1: forearmWrist
///   2: wrist
///   3-6: thumb (knuckle, intermediateBase, intermediateTip, tip)
///   7-11: index (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   12-16: middle (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   17-21: ring (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
///   22-26: little (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
struct HandJointData: Codable {
    // Forearm joints (indices 0-1)
    let forearmArm: [Float]?    // 4x4 matrix flattened (index 0) - optional, may not be tracked
    let forearmWrist: [Float]?  // 4x4 matrix flattened (index 1) - optional, may not be tracked
    
    let wrist: [Float]  // 4x4 matrix flattened (index 2)
    
    // Thumb (4 joints, indices 3-6) - no metacarpal for thumb
    let thumbKnuckle: [Float]          // thumbCMC equivalent
    let thumbIntermediateBase: [Float] // thumbMP equivalent  
    let thumbIntermediateTip: [Float]  // thumbIP equivalent
    let thumbTip: [Float]
    
    // Index finger (5 joints, indices 7-11)
    let indexMetacarpal: [Float]
    let indexKnuckle: [Float]          // indexMCP equivalent
    let indexIntermediateBase: [Float] // indexPIP equivalent
    let indexIntermediateTip: [Float]  // indexDIP equivalent
    let indexTip: [Float]
    
    // Middle finger (5 joints, indices 12-16)
    let middleMetacarpal: [Float]
    let middleKnuckle: [Float]
    let middleIntermediateBase: [Float]
    let middleIntermediateTip: [Float]
    let middleTip: [Float]
    
    // Ring finger (5 joints, indices 17-21)
    let ringMetacarpal: [Float]
    let ringKnuckle: [Float]
    let ringIntermediateBase: [Float]
    let ringIntermediateTip: [Float]
    let ringTip: [Float]
    
    // Little finger (5 joints, indices 22-26)
    let littleMetacarpal: [Float]
    let littleKnuckle: [Float]
    let littleIntermediateBase: [Float]
    let littleIntermediateTip: [Float]
    let littleTip: [Float]
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
    let hasSimulationData: Bool
    let hasUSDZ: Bool
    let videoSource: String  // "network" or "uvc"
    let averageFPS: Double
    let deviceInfo: DeviceInfo
    // Calibration data
    let intrinsicCalibration: [String: Any]?
    let extrinsicCalibration: [String: Any]?
    
    enum CodingKeys: String, CodingKey {
        case version, createdAt, duration, frameCount, hasVideo
        case hasLeftHand, hasRightHand, hasHead, hasSimulationData, hasUSDZ, videoSource, averageFPS, deviceInfo
        case intrinsicCalibration, extrinsicCalibration
    }
    
    func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        try container.encode(version, forKey: .version)
        try container.encode(createdAt, forKey: .createdAt)
        try container.encode(duration, forKey: .duration)
        try container.encode(frameCount, forKey: .frameCount)
        try container.encode(hasVideo, forKey: .hasVideo)
        try container.encode(hasLeftHand, forKey: .hasLeftHand)
        try container.encode(hasRightHand, forKey: .hasRightHand)
        try container.encode(hasHead, forKey: .hasHead)
        try container.encode(hasSimulationData, forKey: .hasSimulationData)
        try container.encode(hasUSDZ, forKey: .hasUSDZ)
        try container.encode(videoSource, forKey: .videoSource)
        try container.encode(averageFPS, forKey: .averageFPS)
        try container.encode(deviceInfo, forKey: .deviceInfo)
        // Encode intrinsic calibration as JSON data
        if let intrinsic = intrinsicCalibration {
            let jsonData = try JSONSerialization.data(withJSONObject: intrinsic, options: [])
            let jsonString = String(data: jsonData, encoding: .utf8) ?? "{}"
            try container.encode(jsonString, forKey: .intrinsicCalibration)
        }
        // Encode extrinsic calibration as JSON data
        if let extrinsic = extrinsicCalibration {
            let jsonData = try JSONSerialization.data(withJSONObject: extrinsic, options: [])
            let jsonString = String(data: jsonData, encoding: .utf8) ?? "{}"
            try container.encode(jsonString, forKey: .extrinsicCalibration)
        }
    }
    
    init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        // version has default value
        createdAt = try container.decode(Date.self, forKey: .createdAt)
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
        deviceInfo = try container.decode(DeviceInfo.self, forKey: .deviceInfo)
        // Decode intrinsic calibration from JSON string
        if let jsonString = try container.decodeIfPresent(String.self, forKey: .intrinsicCalibration),
           let jsonData = jsonString.data(using: .utf8),
           let dict = try JSONSerialization.jsonObject(with: jsonData, options: []) as? [String: Any] {
            intrinsicCalibration = dict
        } else {
            intrinsicCalibration = nil
        }
        // Decode extrinsic calibration from JSON string
        if let jsonString = try container.decodeIfPresent(String.self, forKey: .extrinsicCalibration),
           let jsonData = jsonString.data(using: .utf8),
           let dict = try JSONSerialization.jsonObject(with: jsonData, options: []) as? [String: Any] {
            extrinsicCalibration = dict
        } else {
            extrinsicCalibration = nil
        }
    }
    
    init(createdAt: Date, duration: Double, frameCount: Int, hasVideo: Bool, hasLeftHand: Bool,
         hasRightHand: Bool, hasHead: Bool, hasSimulationData: Bool, hasUSDZ: Bool, videoSource: String, averageFPS: Double,
         deviceInfo: DeviceInfo, intrinsicCalibration: [String: Any]? = nil, extrinsicCalibration: [String: Any]? = nil) {
        self.createdAt = createdAt
        self.duration = duration
        self.frameCount = frameCount
        self.hasVideo = hasVideo
        self.hasLeftHand = hasLeftHand
        self.hasRightHand = hasRightHand
        self.hasHead = hasHead
        self.hasSimulationData = hasSimulationData
        self.hasUSDZ = hasUSDZ
        self.videoSource = videoSource
        self.averageFPS = averageFPS
        self.deviceInfo = deviceInfo
        self.intrinsicCalibration = intrinsicCalibration
        self.extrinsicCalibration = extrinsicCalibration
    }
}

struct DeviceInfo: Codable {
    let model: String
    let systemVersion: String
    let appVersion: String
}

/// Storage location options - simplified to Local vs Cloud
enum RecordingStorageLocation: String, CaseIterable {
    case local = "Local"
    case cloud = "Cloud"
    
    var icon: String {
        switch self {
        case .local: return "internaldrive"
        case .cloud: return "icloud"
        }
    }
    
    var description: String {
        switch self {
        case .local: return "Documents folder (Files app)"
        case .cloud: return "Synced via cloud provider"
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
    
    // Cloud storage (synced from iOS companion app)
    @Published var cloudProvider: CloudStorageProvider = .iCloudDrive
    @Published var isUploadingToCloud: Bool = false
    @Published var cloudUploadProgress: String = ""
    @Published var cloudUploadCurrentFile: Int = 0
    @Published var cloudUploadTotalFiles: Int = 0
    @Published var cloudUploadCurrentFileName: String = ""  // Current file being uploaded
    
    // Auto-recording state
    @Published var autoRecordingEnabled: Bool {
        didSet {
            UserDefaults.standard.set(autoRecordingEnabled, forKey: "autoRecordingEnabled")
        }
    }
    @Published var isAutoRecording: Bool = false  // True if current recording was auto-started
    private var userManuallyStopped: Bool = false // Prevent auto-restart after manual stop
    
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
    nonisolated(unsafe) private var simulationFrames: [SimulationFrame] = []
    nonisolated(unsafe) private var simulationFrameCount: Int = 0
    private var usdzURL: URL?
    
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
    nonisolated(unsafe) private var lastPresentationTime: CMTime?
    
    // Notification observers
    private var cancellables = Set<AnyCancellable>()
    
    // MARK: - Initialization
    
    private init() {
        // Load saved storage location
        if let savedLocation = UserDefaults.standard.string(forKey: "recordingStorageLocation"),
           let location = RecordingStorageLocation(rawValue: savedLocation) {
            self.storageLocation = location
        } else {
            self.storageLocation = .local  // Default to local storage
        }
        
        // Load auto-recording preference (default to true for auto-record by default)
        self.autoRecordingEnabled = UserDefaults.standard.object(forKey: "autoRecordingEnabled") as? Bool ?? true
        
        // Load cloud provider from keychain (synced from iOS)
        loadCloudSettings()
        
        // Observe cloud settings changes
        setupCloudSettingsObserver()
    }
    
    /// Setup observer for cloud settings changes (from iCloud Keychain sync)
    private func setupCloudSettingsObserver() {
        NotificationCenter.default.publisher(for: .cloudStorageSettingsDidChange)
            .receive(on: DispatchQueue.main)
            .sink { [weak self] _ in
                Task { @MainActor in
                    self?.loadCloudSettings()
                }
            }
            .store(in: &cancellables)
    }
    
    /// Load cloud storage settings from iCloud Keychain (set by iOS companion app)
    func loadCloudSettings() {
        CloudStorageSettings.shared.loadSettings()
        cloudProvider = CloudStorageSettings.shared.getActiveProvider()
        print("☁️ [RecordingManager] Cloud provider: \(cloudProvider.displayName)")
    }
    
    // MARK: - Auto-Recording Control
    
    /// Called when first video frame is received. Starts recording if auto-recording is enabled.
    /// Video source can be UVC camera or network stream.
    func onFirstVideoFrame() {
        guard autoRecordingEnabled && !isRecording && !userManuallyStopped else { return }
        
        print("🔴 [RecordingManager] Auto-starting recording on first video frame")
        isAutoRecording = true
        startRecording()
    }
    
    /// Called when video source is disconnected (UVC camera unplugged, Python client disconnected, or WebRTC disconnected).
    /// Stops recording if it was auto-started.
    func onVideoSourceDisconnected(reason: String) {
        // Reset manual stop flag so auto-recording can work on next connection
        userManuallyStopped = false
        
        guard isRecording else { return }
        
        print("🔴 [RecordingManager] Stopping recording due to: \(reason)")
        stopRecording()
        isAutoRecording = false
    }
    
    /// Explicitly stop recording (user action). This also clears auto-recording state.
    func stopRecordingManually() {
        guard isRecording else { return }
        
        print("🔴 [RecordingManager] User manually stopped recording")
        userManuallyStopped = true // Prevent immediate auto-restart
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
            self?.simulationFrames.removeAll()
            self?.pendingFrameCount = 0
            self?.simulationFrameCount = 0
            self?.isWriterSessionStarted = false
            self?.videoSize = .zero
            self?.lastPresentationTime = nil
        }
        
        recordingStartTime = Date()
        frameCount = 0
        recordingDuration = 0
        recordingError = nil
        
        // Generate session ID with UUID to ensure uniqueness even within same second
        let formatter = DateFormatter()
        formatter.dateFormat = "yyyyMMdd_HHmmss"
        let uuidShort = UUID().uuidString.prefix(4)
        sessionID = "recording_\(formatter.string(from: Date()))_\(uuidShort)"
        
        isRecording = true
        
        // Start duration timer - also updates frame count periodically
        durationTimer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
            Task { @MainActor in
                guard let self = self, let startTime = self.recordingStartTime else { return }
                self.recordingDuration = Date().timeIntervalSince(startTime)
                self.frameCount = max(self.pendingFrameCount, self.simulationFrameCount)
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
        
        let timestamp = max(0, captureTime.timeIntervalSince(startTime))
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
            var presentationTime = CMTime(seconds: timestamp, preferredTimescale: 600)
            
            // Ensure strictly increasing timestamps (AVAssetWriter requirement)
            if let lastTime = self.lastPresentationTime {
                if presentationTime <= lastTime {
                    // If timestamp is not increasing, bump it slightly
                    presentationTime = CMTimeAdd(lastTime, CMTime(value: 1, timescale: 600))
                    // print("⚠️ [RecordingManager] Adjusted timestamp for frame \(frameIndex) to maintain order")
                }
            }
            self.lastPresentationTime = presentationTime
            
            // Write video frame - skip if writer isn't ready (don't block!)
            if let writer = self.assetWriter,
               let writerInput = self.videoWriterInput,
               let adaptor = self.pixelBufferAdaptor,
               writer.status == .writing,
               writerInput.isReadyForMoreMediaData {
                
                if let pixelBuffer = self.createPixelBuffer(from: videoFrame) {
                    if adaptor.append(pixelBuffer, withPresentationTime: presentationTime) {
                        // Success
                        if frameIndex % 60 == 0 {
                            print("DEBUG: Appended frame \(frameIndex) at \(presentationTime.seconds)s")
                        }
                    } else {
                        print("⚠️ [RecordingManager] Failed to append frame \(frameIndex). Writer status: \(writer.status.rawValue), Error: \(String(describing: writer.error))")
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
    
    // MARK: - Simulation Data Recording
    
    /// Record simulation data (poses, qpos, ctrl) along with tracking data (hands, head)
    /// This ensures hand tracking is recorded even when video is not streaming
    nonisolated func recordSimulationData(timestamp: Double, poses: [String: [Float]], qpos: [Float]?, ctrl: [Float]?, trackingData: HandTrackingData?) {
        Task { @MainActor [weak self] in
            guard let self = self else { return }
            
            // Trigger auto-recording if needed (just like video)
            if !self.isRecording && self.autoRecordingEnabled && !self.userManuallyStopped {
                self.onFirstSimulationFrame()
            }
            
            self.processSimulationData(timestamp: timestamp, poses: poses, qpos: qpos, ctrl: ctrl, trackingData: trackingData)
        }
    }
    
    /// Called when first simulation frame is received. Starts recording if auto-recording is enabled.
    func onFirstSimulationFrame() {
        guard autoRecordingEnabled && !isRecording && !userManuallyStopped else { return }
        
        print("🔴 [RecordingManager] Auto-starting recording on first simulation frame")
        isAutoRecording = true
        startRecording()
    }
    
    /// Process simulation data on the main thread (to access recordingStartTime) then dispatch to recording queue
    private func processSimulationData(timestamp: Double, poses: [String: [Float]], qpos: [Float]?, ctrl: [Float]?, trackingData: HandTrackingData?) {
        guard isRecording, let startTime = recordingStartTime else { return }
        
        let relativeTimestamp = timestamp - startTime.timeIntervalSince1970
        
        // Prepare tracking data if available
        var headMatrixArray: [Float]? = nil
        var leftHand: HandJointData? = nil
        var rightHand: HandJointData? = nil
        
        if let trackingData = trackingData {
            headMatrixArray = matrixToArray(trackingData.Head)
            leftHand = extractHandJointData(wrist: trackingData.leftWrist, skeleton: trackingData.leftSkeleton)
            rightHand = extractHandJointData(wrist: trackingData.rightWrist, skeleton: trackingData.rightSkeleton)
        }
        
        recordingQueue.async { [weak self] in
            guard let self = self else { return }
            
            // Append simulation frame
            self.simulationFrames.append(SimulationFrame(
                timestamp: relativeTimestamp,
                poses: poses,
                qpos: qpos,
                ctrl: ctrl
            ))
            self.simulationFrameCount += 1
            
            // Also append tracking frame if we have tracking data
            if let head = headMatrixArray {
                let recordedFrame = RecordedFrame(
                    timestamp: relativeTimestamp,
                    systemTime: Date().timeIntervalSince1970,
                    headMatrix: head,
                    leftHand: leftHand,
                    rightHand: rightHand,
                    videoFrameIndex: -1,  // No video frame for simulation-only recordings
                    videoWidth: 0,
                    videoHeight: 0
                )
                self.recordedFrames.append(recordedFrame)
            }
        }
    }
    
    /// Set the URL of the USDZ file to be saved with the recording
    func setUsdzUrl(_ url: URL) {
        self.usdzURL = url
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
        guard joints.count >= 27 else { return nil }
        
        // Joint order from 🥽AppModel.swift:
        //   0: forearmArm
        //   1: forearmWrist
        //   2: wrist
        //   3-6: thumb (knuckle, intermediateBase, intermediateTip, tip)
        //   7-11: index (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
        //   12-16: middle (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
        //   17-21: ring (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
        //   22-26: little (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
        
        // Check if forearm joints are valid (not identity matrix = tracked)
        // Forearm joints may not always be tracked, so we make them optional
        let forearmArmMatrix = joints[0]
        let forearmWristMatrix = joints[1]
        
        // Check if forearmArm is at origin (identity matrix indicates not tracked)
        let isForearmmArmTracked = forearmArmMatrix.columns.3.x != 0 || forearmArmMatrix.columns.3.y != 0 || forearmArmMatrix.columns.3.z != 0
        let isForearmWristTracked = forearmWristMatrix.columns.3.x != 0 || forearmWristMatrix.columns.3.y != 0 || forearmWristMatrix.columns.3.z != 0
        
        return HandJointData(
            // Forearm joints (optional - may not be tracked)
            forearmArm: isForearmmArmTracked ? matrixToArray(forearmArmMatrix) : nil,
            forearmWrist: isForearmWristTracked ? matrixToArray(forearmWristMatrix) : nil,
            
            wrist: matrixToArray(wrist),
            
            // Thumb (indices 3-6)
            thumbKnuckle: matrixToArray(joints[3]),
            thumbIntermediateBase: matrixToArray(joints[4]),
            thumbIntermediateTip: matrixToArray(joints[5]),
            thumbTip: matrixToArray(joints[6]),
            
            // Index (indices 7-11)
            indexMetacarpal: matrixToArray(joints[7]),
            indexKnuckle: matrixToArray(joints[8]),
            indexIntermediateBase: matrixToArray(joints[9]),
            indexIntermediateTip: matrixToArray(joints[10]),
            indexTip: matrixToArray(joints[11]),
            
            // Middle (indices 12-16)
            middleMetacarpal: matrixToArray(joints[12]),
            middleKnuckle: matrixToArray(joints[13]),
            middleIntermediateBase: matrixToArray(joints[14]),
            middleIntermediateTip: matrixToArray(joints[15]),
            middleTip: matrixToArray(joints[16]),
            
            // Ring (indices 17-21)
            ringMetacarpal: matrixToArray(joints[17]),
            ringKnuckle: matrixToArray(joints[18]),
            ringIntermediateBase: matrixToArray(joints[19]),
            ringIntermediateTip: matrixToArray(joints[20]),
            ringTip: matrixToArray(joints[21]),
            
            // Little (indices 22-26)
            littleMetacarpal: matrixToArray(joints[22]),
            littleKnuckle: matrixToArray(joints[23]),
            littleIntermediateBase: matrixToArray(joints[24]),
            littleIntermediateTip: matrixToArray(joints[25]),
            littleTip: matrixToArray(joints[26])
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
        
        guard !recordedFrames.isEmpty || !simulationFrames.isEmpty else {
            recordingError = "No frames recorded"
            return
        }
        
        isSaving = true
        defer { isSaving = false }
        
        print("💾 [RecordingManager] Saving recording to \(storageLocation.rawValue)...")
        print("   Frames to save: \(recordedFrames.count) (Video), \(simulationFrames.count) (Sim)")
        
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
            } else {
                if let writer = assetWriter {
                    print("⚠️ [RecordingManager] Video writer not writing (Status: \(writer.status.rawValue)). Error: \(String(describing: writer.error))")
                } else {
                    print("⚠️ [RecordingManager] No asset writer active")
                }
            }
            
            // Clean up writer references
            assetWriter = nil
            videoWriterInput = nil
            pixelBufferAdaptor = nil
            pixelBufferPool = nil
            isWriterSessionStarted = false
            recordingFolderURL = nil
            
            // Get extrinsic calibration if available
            let extrinsicCalibrationDict: [String: Any]?
            if let currentCalibration = ExtrinsicCalibrationManager.shared.currentCalibration {
                extrinsicCalibrationDict = currentCalibration.toMetadataDictionary()
            } else {
                extrinsicCalibrationDict = nil
            }
            
            // Get intrinsic calibration if available
            let intrinsicCalibrationDict: [String: Any]?
            if let currentIntrinsic = CameraCalibrationManager.shared.currentCalibration {
                intrinsicCalibrationDict = currentIntrinsic.toMetadataDictionary()
            } else {
                intrinsicCalibrationDict = nil
            }
            
            // Save metadata
            let metadata = RecordingMetadata(
                createdAt: recordingStartTime ?? Date(),
                duration: recordingDuration,
                frameCount: frameCount,
                hasVideo: !recordedFrames.isEmpty,
                hasLeftHand: recordedFrames.contains { $0.leftHand != nil },
                hasRightHand: recordedFrames.contains { $0.rightHand != nil },
                hasHead: recordedFrames.contains { $0.headMatrix != nil },
                hasSimulationData: !simulationFrames.isEmpty,
                hasUSDZ: usdzURL != nil,
                videoSource: DataManager.shared.videoSource.rawValue,
                averageFPS: recordingDuration > 0 ? Double(frameCount) / recordingDuration : 0,
                deviceInfo: DeviceInfo(
                    model: "Apple Vision Pro",
                    systemVersion: UIDevice.current.systemVersion,
                    appVersion: Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "1.0"
                ),
                intrinsicCalibration: intrinsicCalibrationDict,
                extrinsicCalibration: extrinsicCalibrationDict
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
            lineEncoder.outputFormatting = .sortedKeys
            
            for frame in recordedFrames {
                if let data = try? lineEncoder.encode(frame),
                   let string = String(data: data, encoding: .utf8) {
                    trackingContent += string + "\n"
                }
            }
            
            try trackingContent.write(to: trackingURL, atomically: true, encoding: .utf8)
            print("💾 [RecordingManager] Tracking data saved")
            
            // Save simulation data if available
            if !simulationFrames.isEmpty {
                let simURL = recordingFolder.appendingPathComponent("mjdata.jsonl")
                var simContent = ""
                
                for frame in simulationFrames {
                    if let data = try? lineEncoder.encode(frame),
                       let string = String(data: data, encoding: .utf8) {
                        simContent += string + "\n"
                    }
                }
                
                try simContent.write(to: simURL, atomically: true, encoding: .utf8)
                print("💾 [RecordingManager] Simulation data saved: \(simulationFrames.count) frames")
            }
            
            // Copy USDZ file if available
            if let usdzURL = usdzURL {
                let destURL = recordingFolder.appendingPathComponent("scene.usdz")
                do {
                    if FileManager.default.fileExists(atPath: destURL.path) {
                        try FileManager.default.removeItem(at: destURL)
                    }
                    try FileManager.default.copyItem(at: usdzURL, to: destURL)
                    print("💾 [RecordingManager] USDZ file copied")
                } catch {
                    print("⚠️ [RecordingManager] Failed to copy USDZ file: \(error)")
                    // Don't fail the whole save for this
                }
            }
            
            // Update metadata with actual file existence
            let finalMetadata = RecordingMetadata(
                createdAt: metadata.createdAt,
                duration: metadata.duration,
                frameCount: metadata.frameCount,
                hasVideo: metadata.hasVideo,
                hasLeftHand: metadata.hasLeftHand,
                hasRightHand: metadata.hasRightHand,
                hasHead: metadata.hasHead,
                hasSimulationData: !simulationFrames.isEmpty,
                hasUSDZ: usdzURL != nil,
                videoSource: metadata.videoSource,
                averageFPS: metadata.averageFPS,
                deviceInfo: metadata.deviceInfo,
                intrinsicCalibration: metadata.intrinsicCalibration,
                extrinsicCalibration: metadata.extrinsicCalibration
            )
            
            // Re-save metadata with updated flags
            let finalMetadataData = try encoder.encode(finalMetadata)
            try finalMetadataData.write(to: metadataURL)
            lastRecordingURL = recordingFolder
            recordingError = nil
            
            print("✅ [RecordingManager] Recording saved successfully to: \(recordingFolder.path)")
            
            print("☁️ [RecordingManager] Calling uploadToCloudIfNeeded...")
            // Upload to cloud storage if configured
            await uploadToCloudIfNeeded(recordingFolder: recordingFolder)
            
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
        let savedLocation = UserDefaults.standard.string(forKey: "recordingStorageLocation") ?? RecordingStorageLocation.local.rawValue
        let location = RecordingStorageLocation(rawValue: savedLocation) ?? .local
        
        switch location {
        case .local:
            // Save to Documents folder - accessible via Files app
            guard let url = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first else {
                throw RecordingError.storageNotAvailable
            }
            return url.appendingPathComponent("Recordings")
            
        case .cloud:
            // Use iCloud Drive container for cloud sync
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
    
    private func getStorageURL() throws -> URL {
        let fileManager = FileManager.default
        
        switch storageLocation {
        case .local:
            // Save to Documents folder - accessible via Files app
            guard let url = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first else {
                throw RecordingError.storageNotAvailable
            }
            return url.appendingPathComponent("Recordings")
            
        case .cloud:
            // Use the app's iCloud container - syncs to cloud
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
            return "On My Vision Pro → Tracking Streamer → Recordings"
        case .cloud:
            return "\(cloudProvider.displayName) → VisionProTeleop"
        }
    }
    
    /// Get a description of the cloud provider setting
    func getCloudProviderDescription() -> String {
        // Refresh cloud settings
        loadCloudSettings()
        
        switch cloudProvider {
        case .iCloudDrive:
            return "iCloud Drive (default)"
        case .dropbox:
            if DropboxUploader.shared.isAvailable() {
                return "Dropbox (configured via iOS)"
            } else {
                return "Dropbox (sign in on iOS app)"
            }
        case .googleDrive:
            if GoogleDriveUploader.shared.isAvailable() {
                return "Google Drive (configured via iOS)"
            } else {
                return "Google Drive (sign in on iOS app)"
            }
        }
    }
    
    // MARK: - Cloud Upload
    
    /// Upload recording to cloud storage if configured (Dropbox or Google Drive)
    /// iCloud Drive uploads happen automatically via the file system
    private func uploadToCloudIfNeeded(recordingFolder: URL) async {
        // Refresh cloud settings in case they changed
        loadCloudSettings()
        
        print("☁️ [RecordingManager] Upload requested. Provider: \(cloudProvider.displayName)")
        
        // iCloud Drive doesn't need manual upload - files are in the iCloud container
        guard cloudProvider == .dropbox || cloudProvider == .googleDrive else {
            print("☁️ [RecordingManager] Using iCloud Drive (or none) - no manual upload needed")
            return
        }
        
        let recordingName = recordingFolder.lastPathComponent
        
        // Progress callback to update UI (now includes current filename)
        let progressCallback: (Int, Int, String) -> Void = { [weak self] current, total, currentFileName in
            Task { @MainActor in
                self?.cloudUploadCurrentFile = current
                self?.cloudUploadTotalFiles = total
                self?.cloudUploadCurrentFileName = currentFileName
                if let provider = self?.cloudProvider {
                    self?.cloudUploadProgress = "Uploading to \(provider.displayName)... (\(current)/\(total))"
                }
            }
        }
        
        if cloudProvider == .dropbox {
            // Check if Dropbox is available
            guard DropboxUploader.shared.isAvailable() else {
                print("⚠️ [RecordingManager] Dropbox selected but not configured - sign in via iOS app")
                return
            }
            
            isUploadingToCloud = true
            cloudUploadCurrentFile = 0
            cloudUploadTotalFiles = 0
            cloudUploadCurrentFileName = ""
            cloudUploadProgress = "Preparing upload to Dropbox..."
            
            print("☁️ [RecordingManager] Uploading to Dropbox...")
            
            let success = await DropboxUploader.shared.uploadRecording(
                folderURL: recordingFolder,
                recordingName: recordingName,
                progressCallback: progressCallback
            )
            
            isUploadingToCloud = false
            cloudUploadCurrentFileName = ""
            
            if success {
                cloudUploadProgress = "Uploaded to Dropbox ✓"
                print("✅ [RecordingManager] Successfully uploaded to Dropbox")
            } else {
                cloudUploadProgress = "Dropbox upload failed"
                print("❌ [RecordingManager] Failed to upload to Dropbox")
            }
        } else if cloudProvider == .googleDrive {
            // Check if Google Drive is available
            guard GoogleDriveUploader.shared.isAvailable() else {
                print("⚠️ [RecordingManager] Google Drive selected but not configured - sign in via iOS app")
                return
            }
            
            isUploadingToCloud = true
            cloudUploadCurrentFile = 0
            cloudUploadTotalFiles = 0
            cloudUploadCurrentFileName = ""
            cloudUploadProgress = "Preparing upload to Google Drive..."
            
            print("☁️ [RecordingManager] Uploading to Google Drive...")
            
            let success = await GoogleDriveUploader.shared.uploadRecording(
                folderURL: recordingFolder,
                recordingName: recordingName,
                progressCallback: progressCallback
            )
            
            isUploadingToCloud = false
            cloudUploadCurrentFileName = ""
            
            if success {
                cloudUploadProgress = "Uploaded to Google Drive ✓"
                print("✅ [RecordingManager] Successfully uploaded to Google Drive")
            } else {
                cloudUploadProgress = "Google Drive upload failed"
                print("❌ [RecordingManager] Failed to upload to Google Drive")
            }
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
