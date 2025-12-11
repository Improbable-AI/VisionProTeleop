import SwiftUI
import AVFoundation
import Combine

/// Represents a connected UVC camera device
struct UVCDevice: Identifiable, Hashable {
    let id: String
    let name: String
    
    var captureDevice: AVCaptureDevice? {
        AVCaptureDevice(uniqueID: id)
    }
}

/// Manages UVC camera discovery, capture, and frame delivery
@MainActor
class UVCCameraManager: NSObject, ObservableObject {
    static let shared = UVCCameraManager()
    
    // MARK: - Published Properties
    @Published var availableDevices: [UVCDevice] = []
    @Published var selectedDevice: UVCDevice? = nil
    @Published var isCapturing: Bool = false
    @Published var authorizationStatus: AVAuthorizationStatus = .authorized
    @Published var currentFrame: UIImage? = nil
    @Published var frameWidth: Int = 0
    @Published var frameHeight: Int = 0
    @Published var fps: Int = 0
    @Published var stereoEnabled: Bool = false  // Whether current camera feed should be treated as stereo
    
    // MARK: - Stereo Preference Persistence
    
    /// UserDefaults key prefix for per-camera stereo preference
    private static let stereoPreferenceKeyPrefix = "uvcStereo_"
    
    /// Set stereo mode for current camera with persistence
    func setStereoMode(_ isStereo: Bool) {
        stereoEnabled = isStereo
        
        // Persist for this specific camera
        if let deviceID = selectedDevice?.id {
            let key = Self.stereoPreferenceKeyPrefix + deviceID
            UserDefaults.standard.set(isStereo, forKey: key)
            dlog("📷 [UVCCameraManager] Saved stereo preference for camera \(selectedDevice?.name ?? "unknown"): \(isStereo)")
        }
    }
    
    /// Load stereo preference for a specific camera
    private func loadStereoPreference(for device: UVCDevice) {
        let key = Self.stereoPreferenceKeyPrefix + device.id
        let savedValue = UserDefaults.standard.object(forKey: key) as? Bool ?? false
        stereoEnabled = savedValue
        dlog("📷 [UVCCameraManager] Loaded stereo preference for \(device.name): \(savedValue)")
    }
    
    // MARK: - Private Properties
    private let captureSession = AVCaptureSession()
    private let videoDataOutput = AVCaptureVideoDataOutput()
    private let sessionQueue = DispatchQueue(label: "com.visionproteleop.uvccamera.session")
    private let processingQueue = DispatchQueue(label: "com.visionproteleop.uvccamera.processing", qos: .userInteractive)
    
    private let discoverySession = AVCaptureDevice.DiscoverySession(
        deviceTypes: [.external],
        mediaType: .video,
        position: .unspecified
    )
    
    // Frame rate calculation
    private var frameCount: Int = 0
    private var lastFPSUpdate: Date = Date()
    
    // Frame callback for external consumers
    var onFrameReceived: ((UIImage) -> Void)?
    
    // Raw pixel buffer callback for calibration and processing
    var onPixelBufferReceived: ((CVPixelBuffer) -> Void)?
    
    // MARK: - Initialization
    
    private override init() {
        super.init()
        dlog("📷 [UVCCameraManager] Initializing...")
        setupSession()
        checkAuthorizationStatus()
        observeDeviceConnectionStates()
        
        // Initial device discovery and auto-start if camera already connected
        Task { @MainActor in
            self.updateDeviceList()
            
            // If we found devices on init and we're authorized, auto-start
            if !self.availableDevices.isEmpty && self.authorizationStatus == .authorized {
                if let device = self.selectedDevice, !self.isCapturing {
                    dlog("📷 [UVCCameraManager] Camera already connected at launch, auto-starting capture")
                    // Short delay for session setup
                    try? await Task.sleep(nanoseconds: 500_000_000) // 500ms
                    self.startCapture()
                }
            }
        }
    }
    
    // MARK: - Authorization
    
    private func checkAuthorizationStatus() {
        authorizationStatus = AVCaptureDevice.authorizationStatus(for: .video)
        dlog("📷 [UVCCameraManager] Authorization status: \(authorizationStatus.rawValue)")
    }
    
    func requestCameraAccess() async -> Bool {
        let status = AVCaptureDevice.authorizationStatus(for: .video)
        
        switch status {
        case .authorized:
            await MainActor.run { authorizationStatus = .authorized }
            return true
        case .notDetermined:
            let granted = await AVCaptureDevice.requestAccess(for: .video)
            await MainActor.run { authorizationStatus = granted ? .authorized : .denied }
            return granted
        case .denied, .restricted:
            await MainActor.run { authorizationStatus = status }
            return false
        @unknown default:
            return false
        }
    }
    
    // MARK: - Device Discovery
    
    private func updateDeviceList() {
        let devices = discoverySession.devices.map { 
            UVCDevice(id: $0.uniqueID, name: $0.localizedName)
        }
        
        dlog("📷 [UVCCameraManager] Found \(devices.count) UVC device(s)")
        for device in devices {
            dlog("   - \(device.name) (ID: \(device.id))")
        }
        
        availableDevices = devices
        
        // Auto-select first device if none selected
        if selectedDevice == nil && !devices.isEmpty {
            let firstDevice = devices.first!
            dlog("📷 [UVCCameraManager] Auto-selected: \(firstDevice.name)")
            
            // Auto-start capture when a new device is connected
            selectDevice(firstDevice)
            
            // Wait for device setup then start capture if authorized
            Task {
                // Request access if needed
                let granted = await requestCameraAccess()
                if granted {
                    try? await Task.sleep(nanoseconds: 300_000_000) // 300ms for device setup
                    await MainActor.run {
                        if !self.isCapturing && self.selectedDevice != nil {
                            dlog("📷 [UVCCameraManager] Auto-starting capture for newly connected device")
                            self.startCapture()
                        }
                    }
                }
            }
        }
        
        // Clear selection if selected device was disconnected
        if let selected = selectedDevice, !devices.contains(where: { $0.id == selected.id }) {
            dlog("📷 [UVCCameraManager] Selected device disconnected")
            selectedDevice = nil
            stopCapture()
            
            // Notify recording manager about UVC disconnect for auto-stop
            Task { @MainActor in
                RecordingManager.shared.onVideoSourceDisconnected(reason: "UVC camera disconnected")
            }
        }
    }
    
    private func observeDeviceConnectionStates() {
        Task {
            for await _ in NotificationCenter.default.notifications(named: AVCaptureDevice.wasConnectedNotification) {
                dlog("📷 [UVCCameraManager] Device connected notification")
                await MainActor.run { updateDeviceList() }
            }
        }
        
        Task {
            for await _ in NotificationCenter.default.notifications(named: AVCaptureDevice.wasDisconnectedNotification) {
                dlog("📷 [UVCCameraManager] Device disconnected notification")
                await MainActor.run { updateDeviceList() }
            }
        }
    }
    
    // MARK: - Capture Session Setup
    
    private func setupSession() {
        sessionQueue.async { [weak self] in
            guard let self = self else { return }
            
            self.captureSession.beginConfiguration()
            defer { self.captureSession.commitConfiguration() }
            
            // Configure video output
            self.videoDataOutput.alwaysDiscardsLateVideoFrames = true
            self.videoDataOutput.videoSettings = [
                kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA
            ]
            self.videoDataOutput.setSampleBufferDelegate(self, queue: self.processingQueue)
            
            if self.captureSession.canAddOutput(self.videoDataOutput) {
                self.captureSession.addOutput(self.videoDataOutput)
                dlog("📷 [UVCCameraManager] Added video data output")
            } else {
                dlog("❌ [UVCCameraManager] Unable to add video data output")
            }
        }
    }
    
    // MARK: - Device Selection and Capture Control
    
    func selectDevice(_ device: UVCDevice?) {
        dlog("📷 [UVCCameraManager] Selecting device: \(device?.name ?? "none")")
        
        sessionQueue.async { [weak self] in
            guard let self = self else { return }
            
            self.captureSession.beginConfiguration()
            defer { self.captureSession.commitConfiguration() }
            
            // Remove all existing inputs
            for input in self.captureSession.inputs {
                self.captureSession.removeInput(input)
            }
            
            Task { @MainActor in
                self.currentFrame = nil
                self.frameWidth = 0
                self.frameHeight = 0
                self.fps = 0
            }
            
            // Return if no device selected
            guard let device = device, let captureDevice = device.captureDevice else {
                Task { @MainActor in
                    self.selectedDevice = nil
                    self.isCapturing = false
                }
                return
            }
            
            // Check authorization
            guard AVCaptureDevice.authorizationStatus(for: .video) == .authorized else {
                dlog("❌ [UVCCameraManager] Camera access not authorized")
                return
            }
            
            do {
                let input = try AVCaptureDeviceInput(device: captureDevice)
                
                if self.captureSession.canAddInput(input) {
                    self.captureSession.addInput(input)
                    dlog("📷 [UVCCameraManager] Added input for \(device.name)")
                    
                    Task { @MainActor in
                        self.selectedDevice = device
                        self.loadStereoPreference(for: device)
                    }
                } else {
                    dlog("❌ [UVCCameraManager] Unable to add input for \(device.name)")
                }
            } catch {
                dlog("❌ [UVCCameraManager] Failed to create input: \(error)")
            }
        }
    }
    
    func startCapture() {
        guard selectedDevice != nil else {
            dlog("📷 [UVCCameraManager] No device selected, cannot start capture")
            return
        }
        
        guard authorizationStatus == .authorized else {
            dlog("📷 [UVCCameraManager] Camera not authorized, cannot start capture")
            Task {
                let granted = await requestCameraAccess()
                if granted {
                    startCapture()
                }
            }
            return
        }
        
        sessionQueue.async { [weak self] in
            guard let self = self else { return }
            
            if !self.captureSession.isRunning {
                self.captureSession.startRunning()
                dlog("📷 [UVCCameraManager] Capture session started")
                
                Task { @MainActor in
                    self.isCapturing = true
                    self.frameCount = 0
                    self.lastFPSUpdate = Date()
                    
                    // Automatically switch video source to UVC camera when capture starts
                    if DataManager.shared.videoSource != .uvcCamera {
                        dlog("📷 [UVCCameraManager] Auto-switching video source to USB Camera")
                        DataManager.shared.videoSource = .uvcCamera
                    }
                }
            }
        }
    }
    
    func stopCapture() {
        sessionQueue.async { [weak self] in
            guard let self = self else { return }
            
            if self.captureSession.isRunning {
                self.captureSession.stopRunning()
                dlog("📷 [UVCCameraManager] Capture session stopped")
                
                Task { @MainActor in
                    self.isCapturing = false
                    self.currentFrame = nil
                    self.fps = 0
                }
            }
        }
    }
}

// MARK: - AVCaptureVideoDataOutputSampleBufferDelegate

extension UVCCameraManager: AVCaptureVideoDataOutputSampleBufferDelegate {
    nonisolated func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        
        // Notify raw pixel buffer consumers (for calibration, etc.)
        // This runs on the capture queue, not main actor
        Task { @MainActor [pixelBuffer] in
            self.onPixelBufferReceived?(pixelBuffer)
        }
        
        // Convert to UIImage
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let context = CIContext()
        guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else { return }
        let uiImage = UIImage(cgImage: cgImage)
        
        Task { @MainActor in
            self.currentFrame = uiImage
            self.frameWidth = width
            self.frameHeight = height
            
            // Calculate FPS
            self.frameCount += 1
            let now = Date()
            let elapsed = now.timeIntervalSince(self.lastFPSUpdate)
            if elapsed >= 1.0 {
                self.fps = Int(Double(self.frameCount) / elapsed)
                self.frameCount = 0
                self.lastFPSUpdate = now
            }
            
            // Notify external consumers
            self.onFrameReceived?(uiImage)
        }
    }
    
    nonisolated func captureOutput(_ output: AVCaptureOutput, didDrop sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        // Frame was dropped, could log for debugging
    }
}
