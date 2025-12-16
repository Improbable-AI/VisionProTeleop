//
//  CalibrationDebugRecorder.swift
//  Tracking Streamer
//
//  Records calibration debug data for offline Python analysis.
//

import Foundation
import UIKit
import simd

/// Records calibration debug data including camera frames and pose matrices
@MainActor
class CalibrationDebugRecorder: ObservableObject {
    static let shared = CalibrationDebugRecorder()
    
    // MARK: - Published Properties
    
    @Published var isRecording: Bool = false
    @Published var sessionPath: URL?
    @Published var intrinsicFrameCount: Int = 0
    @Published var extrinsicSampleCount: Int = 0
    
    // MARK: - Private Properties
    
    private var currentSessionURL: URL?
    private var intrinsicFolderURL: URL?
    private var extrinsicFolderURL: URL?
    private var sessionInfo: [String: Any] = [:]
    
    private let fileManager = FileManager.default
    private let jsonEncoder = JSONEncoder()
    
    private init() {
        jsonEncoder.outputFormatting = [.prettyPrinted, .sortedKeys]
    }
    
    // MARK: - Session Management
    
    /// Start a new recording session
    func startSession(deviceId: String, deviceName: String, isStereo: Bool, markerSizeMeters: Float) {
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyyMMdd_HHmmss"
        let timestamp = dateFormatter.string(from: Date())
        let sessionName = "calibration_debug_\(timestamp)"
        
        // Get Documents directory
        guard let documentsURL = fileManager.urls(for: .documentDirectory, in: .userDomainMask).first else {
            dlog("❌ [CalibrationDebugRecorder] Cannot find Documents directory")
            return
        }
        
        let sessionURL = documentsURL.appendingPathComponent("CalibrationDebug").appendingPathComponent(sessionName)
        
        do {
            // Create session folder structure
            try fileManager.createDirectory(at: sessionURL, withIntermediateDirectories: true)
            
            let intrinsicURL = sessionURL.appendingPathComponent("intrinsic")
            try fileManager.createDirectory(at: intrinsicURL, withIntermediateDirectories: true)
            
            let extrinsicURL = sessionURL.appendingPathComponent("extrinsic")
            try fileManager.createDirectory(at: extrinsicURL, withIntermediateDirectories: true)
            
            currentSessionURL = sessionURL
            intrinsicFolderURL = intrinsicURL
            extrinsicFolderURL = extrinsicURL
            sessionPath = sessionURL
            
            // Store session info
            sessionInfo = [
                "device_id": deviceId,
                "device_name": deviceName,
                "is_stereo": isStereo,
                "marker_size_meters": markerSizeMeters,
                "start_time": ISO8601DateFormatter().string(from: Date())
            ]
            
            intrinsicFrameCount = 0
            extrinsicSampleCount = 0
            isRecording = true
            
            dlog("📹 [CalibrationDebugRecorder] Started session: \(sessionName)")
            
        } catch {
            dlog("❌ [CalibrationDebugRecorder] Failed to create session: \(error)")
        }
    }
    
    /// End the current recording session
    func endSession() {
        guard let sessionURL = currentSessionURL else { return }
        
        // Save session info
        sessionInfo["end_time"] = ISO8601DateFormatter().string(from: Date())
        sessionInfo["intrinsic_frame_count"] = intrinsicFrameCount
        sessionInfo["extrinsic_sample_count"] = extrinsicSampleCount
        
        let infoURL = sessionURL.appendingPathComponent("session_info.json")
        if let jsonData = try? JSONSerialization.data(withJSONObject: sessionInfo, options: [.prettyPrinted, .sortedKeys]) {
            try? jsonData.write(to: infoURL)
        }
        
        isRecording = false
        dlog("📹 [CalibrationDebugRecorder] Ended session with \(intrinsicFrameCount) intrinsic frames, \(extrinsicSampleCount) extrinsic samples")
    }
    
    // MARK: - Intrinsic Recording
    
    /// Record a raw camera frame during intrinsic calibration
    func recordIntrinsicFrame(_ pixelBuffer: CVPixelBuffer) {
        guard isRecording, let folderURL = intrinsicFolderURL else { return }
        
        intrinsicFrameCount += 1
        let filename = String(format: "frame_%04d.jpg", intrinsicFrameCount)
        let fileURL = folderURL.appendingPathComponent(filename)
        
        // Convert pixel buffer to JPEG
        if let image = pixelBufferToUIImage(pixelBuffer),
           let jpegData = image.jpegData(compressionQuality: 0.9) {
            do {
                try jpegData.write(to: fileURL)
            } catch {
                dlog("❌ [CalibrationDebugRecorder] Failed to save intrinsic frame: \(error)")
            }
        }
    }
    
    // MARK: - Extrinsic Recording
    
    /// Record an extrinsic calibration sample - frame, head pose, and ARKit marker pose (synchronized)
    func recordExtrinsicSample(
        pixelBuffer: CVPixelBuffer,
        headPose: simd_float4x4,
        arkitMarkerPose: simd_float4x4?,
        markerId: Int,
        isStereo: Bool
    ) {
        guard isRecording, let folderURL = extrinsicFolderURL else { return }
        
        extrinsicSampleCount += 1
        let sampleName = String(format: "sample_%04d", extrinsicSampleCount)
        let sampleURL = folderURL.appendingPathComponent(sampleName)
        
        do {
            try fileManager.createDirectory(at: sampleURL, withIntermediateDirectories: true)
            
            // Save frame
            if let image = pixelBufferToUIImage(pixelBuffer),
               let jpegData = image.jpegData(compressionQuality: 0.9) {
                let frameURL = sampleURL.appendingPathComponent(isStereo ? "frame_stereo.jpg" : "frame.jpg")
                try jpegData.write(to: frameURL)
            }
            
            // Build data dictionary
            var data: [String: Any] = [
                "timestamp": ISO8601DateFormatter().string(from: Date()),
                "marker_id": markerId,
                "is_stereo": isStereo,
                "head_pose": matrixToArray(headPose)
            ]
            
            if let markerPose = arkitMarkerPose {
                data["arkit_marker_pose"] = matrixToArray(markerPose)
            }
            
            // Save data.json
            let dataURL = sampleURL.appendingPathComponent("data.json")
            if let jsonData = try? JSONSerialization.data(withJSONObject: data, options: [.prettyPrinted, .sortedKeys]) {
                try jsonData.write(to: dataURL)
            }
            
        } catch {
            dlog("❌ [CalibrationDebugRecorder] Failed to save extrinsic sample: \(error)")
        }
    }
    
    // MARK: - Intrinsic Calibration Result
    
    /// Save intrinsic calibration result
    func saveIntrinsicCalibration(
        leftIntrinsics: CameraIntrinsics,
        rightIntrinsics: CameraIntrinsics?,
        imageWidth: Int,
        imageHeight: Int
    ) {
        guard let sessionURL = currentSessionURL else { return }
        
        var data: [String: Any] = [
            "image_width": imageWidth,
            "image_height": imageHeight,
            "left": [
                "fx": leftIntrinsics.fx,
                "fy": leftIntrinsics.fy,
                "cx": leftIntrinsics.cx,
                "cy": leftIntrinsics.cy,
                "distortion_coeffs": leftIntrinsics.distortionCoeffs
            ]
        ]
        
        if let right = rightIntrinsics {
            data["right"] = [
                "fx": right.fx,
                "fy": right.fy,
                "cx": right.cx,
                "cy": right.cy,
                "distortion_coeffs": right.distortionCoeffs
            ]
        }
        
        let fileURL = sessionURL.appendingPathComponent("intrinsic_calibration.json")
        if let jsonData = try? JSONSerialization.data(withJSONObject: data, options: [.prettyPrinted, .sortedKeys]) {
            try? jsonData.write(to: fileURL)
        }
    }
    
    // MARK: - Export
    
    /// Get the current session URL for sharing
    func getSessionURL() -> URL? {
        return currentSessionURL
    }
    
    /// Create a zip file of the session for sharing
    func createZipForSharing() -> URL? {
        guard let sessionURL = currentSessionURL else { return nil }
        
        let zipName = sessionURL.lastPathComponent + ".zip"
        let zipURL = sessionURL.deletingLastPathComponent().appendingPathComponent(zipName)
        
        // Remove existing zip if present
        try? fileManager.removeItem(at: zipURL)
        
        // Create zip using FileManager's built-in compression
        let coordinator = NSFileCoordinator()
        var error: NSError?
        var resultURL: URL?
        
        coordinator.coordinate(readingItemAt: sessionURL, options: .forUploading, error: &error) { url in
            do {
                try fileManager.copyItem(at: url, to: zipURL)
                resultURL = zipURL
                dlog("📦 [CalibrationDebugRecorder] Created zip at: \(zipURL.path)")
            } catch {
                dlog("❌ [CalibrationDebugRecorder] Failed to create zip: \(error)")
            }
        }
        
        return resultURL
    }
    
    // MARK: - Helpers
    
    private func pixelBufferToUIImage(_ pixelBuffer: CVPixelBuffer) -> UIImage? {
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let context = CIContext()
        guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else {
            return nil
        }
        return UIImage(cgImage: cgImage)
    }
    
    private func matrixToArray(_ m: simd_float4x4) -> [[Double]] {
        // Return as row-major 4x4 array for easy Python numpy loading
        return [
            [Double(m.columns.0.x), Double(m.columns.1.x), Double(m.columns.2.x), Double(m.columns.3.x)],
            [Double(m.columns.0.y), Double(m.columns.1.y), Double(m.columns.2.y), Double(m.columns.3.y)],
            [Double(m.columns.0.z), Double(m.columns.1.z), Double(m.columns.2.z), Double(m.columns.3.z)],
            [Double(m.columns.0.w), Double(m.columns.1.w), Double(m.columns.2.w), Double(m.columns.3.w)]
        ]
    }
}
