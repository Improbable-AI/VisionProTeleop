//
//  PersonaCaptureController.swift
//  Tracking Streamer
//
//  Streams frames from the front camera (user-facing) for the user's persona.
//  Provides start/stop control and a live frame stream.
//

import Foundation
import AVFoundation
import SwiftUI
import Combine
import CoreImage

/// Streams frames from the front camera (user-facing) for the user's Spatial Persona.
/// Provides start/stop control and publishes the latest frame for display.
@MainActor
final class PersonaCaptureController: NSObject, ObservableObject {
    enum CaptureError: Error, LocalizedError {
        case cameraUnavailable
        case cameraNotAuthorized
        case configurationFailed

        var errorDescription: String? {
            switch self {
            case .cameraUnavailable: return "No front camera available on this device."
            case .cameraNotAuthorized: return "Camera access is not authorized. Please enable in Settings."
            case .configurationFailed: return "Failed to configure capture session."
            }
        }
    }

    // Singleton for easy access
    static let shared = PersonaCaptureController()

    // Public state
    @Published var isRunning: Bool = false
    @Published var latestImage: CGImage?
    @Published var errorMessage: String?
    @Published var authorizationStatus: AVAuthorizationStatus = .notDetermined
    
    // Frame rate tracking
    @Published var fps: Double = 0.0
    private var frameCount: Int = 0
    private var fpsTimer: Timer?
    
    // Resolution info
    @Published var frameWidth: Int = 0
    @Published var frameHeight: Int = 0

    // Optional per-frame callback
    var onFrame: ((CVPixelBuffer, CMTime) -> Void)?

    // Private capture members
    private let session = AVCaptureSession()
    private var videoInput: AVCaptureDeviceInput?
    private let videoOutput = AVCaptureVideoDataOutput()
    private let videoQueue = DispatchQueue(label: "PersonaCaptureController.VideoOutput")
    private let sessionQueue = DispatchQueue(label: "PersonaCaptureController.Session")
    
    private override init() {
        super.init()
        // Check initial authorization status
        authorizationStatus = AVCaptureDevice.authorizationStatus(for: .video)
    }

    /// Requests camera permission and starts capture from the front camera.
    func start() async throws {
        errorMessage = nil
        
        // Check/request camera permission
        let status = AVCaptureDevice.authorizationStatus(for: .video)
        authorizationStatus = status
        
        switch status {
        case .notDetermined:
            let granted = await AVCaptureDevice.requestAccess(for: .video)
            authorizationStatus = granted ? .authorized : .denied
            guard granted else {
                errorMessage = CaptureError.cameraNotAuthorized.errorDescription
                throw CaptureError.cameraNotAuthorized
            }
        case .authorized:
            break
        case .denied, .restricted:
            errorMessage = CaptureError.cameraNotAuthorized.errorDescription
            throw CaptureError.cameraNotAuthorized
        @unknown default:
            errorMessage = CaptureError.cameraNotAuthorized.errorDescription
            throw CaptureError.cameraNotAuthorized
        }

        // Configure session
        do {
            try configureSession()
        } catch {
            errorMessage = error.localizedDescription
            throw error
        }
        
        // Start session on background queue
        let captureSession = session
        sessionQueue.async { [weak self] in
            captureSession.startRunning()
            let isSessionRunning = captureSession.isRunning
            Task { @MainActor [weak self] in
                self?.isRunning = isSessionRunning
                self?.startFPSTimer()
            }
        }
    }

    /// Stops capture and tears down the session.
    func stop() {
        guard session.isRunning else { return }
        
        let captureSession = session
        sessionQueue.async { [weak self] in
            captureSession.stopRunning()
            Task { @MainActor [weak self] in
                self?.isRunning = false
                self?.stopFPSTimer()
                self?.latestImage = nil
                self?.fps = 0
            }
        }
    }
    
    private func startFPSTimer() {
        frameCount = 0
        fpsTimer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in
                guard let self = self else { return }
                self.fps = Double(self.frameCount)
                self.frameCount = 0
            }
        }
    }
    
    private func stopFPSTimer() {
        fpsTimer?.invalidate()
        fpsTimer = nil
    }

    private func configureSession() throws {
        session.beginConfiguration()
        defer { session.commitConfiguration() }
        
        // Remove existing input/output
        if let input = videoInput {
            session.removeInput(input)
            videoInput = nil
        }
        if session.outputs.contains(videoOutput) {
            session.removeOutput(videoOutput)
        }

        // Note: sessionPreset is not available on visionOS, so we skip it

        // Find a front camera
        guard let device = frontCameraDevice() else {
            throw CaptureError.cameraUnavailable
        }

        do {
            let input = try AVCaptureDeviceInput(device: device)
            if session.canAddInput(input) {
                session.addInput(input)
                videoInput = input
            } else {
                throw CaptureError.configurationFailed
            }
        } catch let error as CaptureError {
            throw error
        } catch {
            throw CaptureError.configurationFailed
        }

        // Configure output
        videoOutput.alwaysDiscardsLateVideoFrames = true
        videoOutput.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA]
        
        if session.canAddOutput(videoOutput) {
            session.addOutput(videoOutput)
        } else {
            throw CaptureError.configurationFailed
        }
        
        videoOutput.setSampleBufferDelegate(self, queue: videoQueue)
        
        // Mirror the video for front camera (so it looks natural like a mirror)
        if let connection = videoOutput.connection(with: .video) {
            if connection.isVideoMirroringSupported {
                connection.isVideoMirrored = true
            }
        }
    }

    private func frontCameraDevice() -> AVCaptureDevice? {
        // Try to find front-facing camera
        let discoverySession = AVCaptureDevice.DiscoverySession(
            deviceTypes: [.builtInWideAngleCamera],
            mediaType: .video,
            position: .front
        )
        return discoverySession.devices.first
    }
}

// MARK: - AVCaptureVideoDataOutputSampleBufferDelegate
extension PersonaCaptureController: AVCaptureVideoDataOutputSampleBufferDelegate {
    nonisolated func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        guard let pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        let timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)
        
        // Convert to CGImage for display
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let context = CIContext()
        guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else { return }
        
        Task { @MainActor in
            self.latestImage = cgImage
            self.frameWidth = width
            self.frameHeight = height
            self.frameCount += 1
            self.onFrame?(pixelBuffer, timestamp)
        }
    }
}

// MARK: - SwiftUI Preview View for Persona

/// A view that displays the persona camera feed
struct PersonaPreviewView: View {
    @ObservedObject private var controller = PersonaCaptureController.shared
    
    var body: some View {
        VStack(spacing: 12) {
            // Preview area
            ZStack {
                RoundedRectangle(cornerRadius: 12)
                    .fill(Color.black.opacity(0.5))
                
                if controller.isRunning, let cgImage = controller.latestImage {
                    Image(decorative: cgImage, scale: 1.0)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .cornerRadius(12)
                } else if controller.isRunning {
                    VStack(spacing: 8) {
                        ProgressView()
                            .tint(.white)
                        Text("Starting camera...")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.7))
                    }
                } else {
                    VStack(spacing: 8) {
                        Image(systemName: "person.crop.circle")
                            .font(.system(size: 40))
                            .foregroundColor(.white.opacity(0.4))
                        Text("Persona Preview")
                            .font(.caption)
                            .foregroundColor(.white.opacity(0.5))
                    }
                }
            }
            .frame(height: 200)
            
            // Status info
            if controller.isRunning {
                HStack(spacing: 16) {
                    // Resolution
                    HStack(spacing: 4) {
                        Image(systemName: "rectangle.on.rectangle")
                            .font(.caption2)
                        Text("\(controller.frameWidth)×\(controller.frameHeight)")
                            .font(.caption2.monospacedDigit())
                    }
                    .foregroundColor(.white.opacity(0.6))
                    
                    // FPS
                    HStack(spacing: 4) {
                        Image(systemName: "speedometer")
                            .font(.caption2)
                        Text("\(Int(controller.fps)) fps")
                            .font(.caption2.monospacedDigit())
                    }
                    .foregroundColor(.white.opacity(0.6))
                }
            }
            
            // Error message
            if let error = controller.errorMessage {
                Text(error)
                    .font(.caption)
                    .foregroundColor(.red)
                    .multilineTextAlignment(.center)
                    .padding(.horizontal)
            }
            
            // Start/Stop button
            Button {
                Task {
                    if controller.isRunning {
                        controller.stop()
                    } else {
                        do {
                            try await controller.start()
                        } catch {
                            dlog("❌ [PersonaCapture] Failed to start: \(error)")
                        }
                    }
                }
            } label: {
                HStack(spacing: 8) {
                    Image(systemName: controller.isRunning ? "stop.circle.fill" : "play.circle.fill")
                        .font(.system(size: 16))
                    Text(controller.isRunning ? "Stop Preview" : "Start Preview")
                        .fontWeight(.medium)
                }
                .foregroundColor(.white)
                .frame(maxWidth: .infinity)
                .padding(.vertical, 12)
                .background(controller.isRunning ? Color.red.opacity(0.3) : Color.green.opacity(0.3))
                .cornerRadius(10)
            }
            .buttonStyle(.plain)
        }
    }
}
