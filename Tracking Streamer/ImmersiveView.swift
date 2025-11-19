import SwiftUI
import RealityKit
import LiveKitWebRTC
import CoreImage
import Accelerate

struct ImmersiveView: View {
    @EnvironmentObject var imageData: ImageData
    @StateObject private var videoStreamManager = VideoStreamManager()
    @StateObject private var appModel = 🥽AppModel()
    @State private var updateTrigger = false
    @State private var hasFrames = false
    @State private var isMinimized = false
    
    var body: some View {
        RealityView { content, attachments in
            print("🟢 [ImmersiveView] RealityView content block called")
            
            // Create the video display anchor
            let videoAnchor = AnchorEntity(.head)
            videoAnchor.name = "videoAnchor"
            content.add(videoAnchor)
            
            // Create the video display plane (initially hidden)
            let skyBox = createSkyBox()
            skyBox.isEnabled = false  // Hide until we have frames
            skyBox.setParent(videoAnchor)
            
            // Position the video plane
            skyBox.transform.translation.z = -6.0
            skyBox.transform.translation.y = 0.0
            
            // Create status display anchor
            let statusAnchor = AnchorEntity(.head)
            statusAnchor.name = "statusHeadAnchor"
            content.add(statusAnchor)
            
            // Create status container entity
            let statusContainer = Entity()
            statusContainer.name = "statusContainer"
            statusContainer.setParent(statusAnchor)
            
            // Position status in top area (relative to head)
            // Y position will be updated in the update block based on minimized state
            statusContainer.transform.translation = SIMD3<Float>(0.0, 0.0, -1.0)
            
            // Attach the status UI to the container
            if let statusAttachment = attachments.entity(for: "status") {
                print("🟢 [ImmersiveView] Status attachment found and attached")
                statusAttachment.setParent(statusContainer)
            } else {
                print("🔴 [ImmersiveView] Status attachment NOT found!")
            }
        } update: { updateContent, attachments in
            // This will be triggered when updateTrigger changes (i.e., when new frames arrive)
            let _ = updateTrigger  // Explicitly depend on updateTrigger
            
            // Find the video anchor
            guard let anchor = updateContent.entities.first(where: { 
                $0.name == "videoAnchor" 
            }) as? AnchorEntity else {
                return
            }
            
            let skyBoxEntity = anchor.children.first(where: { $0.components[ModelComponent.self] != nil })
            
            print("DEBUG: Update block called, left=\(imageData.left != nil), right=\(imageData.right != nil)")
            
            // Update the video texture when new frames arrive
            guard let imageLeft = imageData.left,
                  let imageRight = imageData.right else {
                // No images yet - keep video plane hidden
                print("DEBUG: No images yet, keeping video hidden")
                skyBoxEntity?.isEnabled = false
                hasFrames = false
                return
            }
            
            print("DEBUG: Images available, showing video")
            
            guard let skyBox = skyBoxEntity else {
                return
            }
            
            // Show video
            skyBox.isEnabled = true
            
            // Auto-minimize when frames first arrive (only once, don't override user preference)
            if !hasFrames {
                hasFrames = true
                // Only auto-minimize if user hasn't manually changed the state
                if !isMinimized {
                    // Delay minimization slightly for smooth transition
                    Task { @MainActor in
                        try? await Task.sleep(nanoseconds: 1_000_000_000) // 1 second
                        // Only minimize if user hasn't already interacted with it
                        if !isMinimized {
                            withAnimation(.spring(response: 0.6, dampingFraction: 0.8)) {
                                isMinimized = true
                            }
                        }
                    }
                }
            }
            
            // Update status container position based on minimized state
            if let statusAnchor = updateContent.entities.first(where: { $0.name == "statusHeadAnchor" }) as? AnchorEntity,
               let statusContainer = statusAnchor.children.first(where: { $0.name == "statusContainer" }) {
                // Move higher when minimized
                let yPosition: Float = isMinimized ? 0.5 : 0.0
                let targetTranslation = SIMD3<Float>(0.0, yPosition, -1.0)
                
                // Animate the position change
                var transform = statusContainer.transform
                transform.translation = targetTranslation
                statusContainer.move(to: transform, relativeTo: statusContainer.parent, duration: 0.5, timingFunction: .easeInOut)
            }

            // Check if stereo mode is enabled
            let isStereo = DataManager.shared.stereoEnabled
            print("DEBUG: Stereo mode: \(isStereo)")
            
            if isStereo {
                // Stereo mode: Split side-by-side image and render to separate eyes
                do {
                    // For stereo, we need to split the side-by-side image
                    // Left half goes to left eye, right half to right eye
                    let leftTexture = try TextureResource.generate(
                        from: imageLeft.cgImage!,
                        options: TextureResource.CreateOptions.init(semantic: nil)
                    )
                    let rightTexture = try TextureResource.generate(
                        from: imageRight.cgImage!,
                        options: TextureResource.CreateOptions.init(semantic: nil)
                    )
                    
                    // Create a shader graph material for stereo rendering
                    // Note: This requires a custom shader material created in Reality Composer Pro
                    // For now, we'll attempt to use it if available, otherwise fall back to mono
                    if var stereoMaterial = skyBox.components[ModelComponent.self]?.materials.first as? ShaderGraphMaterial {
                        try stereoMaterial.setParameter(name: "left", value: .textureResource(leftTexture))
                        try stereoMaterial.setParameter(name: "right", value: .textureResource(rightTexture))
                        skyBox.components[ModelComponent.self]?.materials = [stereoMaterial]
                        print("DEBUG: Updated stereo textures successfully")
                    } else {
                        // Fallback: Create stereo material programmatically or use UnlitMaterial
                        print("WARNING: ShaderGraphMaterial not found, falling back to mono display")
                        var skyBoxMaterial = UnlitMaterial()
                        skyBoxMaterial.color = .init(texture: .init(rightTexture))
                        skyBox.components[ModelComponent.self]?.materials = [skyBoxMaterial]
                    }
                } catch {
                    print("ERROR: Failed to load stereo textures: \(error)")
                }
            } else {
                // Mono mode: Use simple unlit material
                var skyBoxMaterial = UnlitMaterial()
                do {
                    // Use right image for mono display
                    let texture = try TextureResource.generate(
                        from: imageRight.cgImage!,
                        options: TextureResource.CreateOptions.init(semantic: nil)
                    )
                    skyBoxMaterial.color = .init(texture: .init(texture))
                    skyBox.components[ModelComponent.self]?.materials = [skyBoxMaterial]
                    print("DEBUG: Updated mono video texture successfully")
                } catch {
                    print("ERROR: Failed to load mono texture: \(error)")
                }
            }
        } attachments: {
            Attachment(id: "status") {
                print("🟡 [ImmersiveView] Status attachment builder called")
                return StatusOverlay(hasFrames: hasFrames, isMinimized: $isMinimized)
                    .frame(maxWidth: 300)
            }
        }
        .onReceive(imageData.$left) { newImage in
            print("DEBUG: onReceive triggered, new image: \(newImage != nil)")
            updateTrigger.toggle()
        }
        .task { appModel.run() }
        .task { await appModel.processDeviceAnchorUpdates() }
        .task(priority: .low) { await appModel.processReconstructionUpdates() }
        .onAppear {
            print("DEBUG: ImmersiveView appeared, starting video stream")
            videoStreamManager.start(imageData: imageData)
        }
        .onDisappear {
            print("DEBUG: ImmersiveView disappeared, stopping video stream")
            videoStreamManager.stop()
        }
    }
}

private func createSkyBox() -> Entity {
    let skyBoxEntity = Entity()
    let largePlane = MeshResource.generatePlane(width: 12.8, height: 9.6)
    var skyBoxMaterial = UnlitMaterial()
    skyBoxMaterial.color = .init(tint: .clear)
    skyBoxEntity.components.set(
        ModelComponent(mesh: largePlane, materials: [skyBoxMaterial])
    )
    return skyBoxEntity
}

/// Manages the WebRTC video stream connection and frame processing
@MainActor
class VideoStreamManager: ObservableObject {
    private var webrtcClient: WebRTCClient?
    private var videoRenderer: VideoFrameRenderer?
    private var isRunning = false
    
    func start(imageData: ImageData) {
        guard !isRunning else { return }
        isRunning = true
        
        Task {
            do {
                print("🎬 [DEBUG] VideoStreamManager.start() called")
                print("⏳ [DEBUG] Waiting for Python client to connect via gRPC...")
                print("💡 [DEBUG] Run your Python script now if you haven't already!")
                
                // Wait for Python client to connect via gRPC
                var pythonIP: String?
                var attempts = 0
                let maxAttempts = 600  // Wait up to 60 seconds (600 * 100ms)
                
                for i in 0..<maxAttempts {
                    if let ip = DataManager.shared.pythonClientIP {
                        pythonIP = ip
                        print("✅ [DEBUG] Python client found after \(i * 100)ms")
                        break
                    }
                    
                    // Print status every 5 seconds
                    if i > 0 && i % 50 == 0 {
                        let secondsWaited = i / 10
                        print("⏳ [DEBUG] Still waiting for Python client... (\(secondsWaited)s elapsed)")
                    }
                    
                    try await Task.sleep(nanoseconds: 100_000_000)  // 100ms
                    attempts += 1
                }
                
                guard let pythonIP = pythonIP else {
                    print("❌ [DEBUG] Timeout: Python client not connected after \(attempts * 100)ms")
                    print("💡 [DEBUG] Make sure you run the Python script with VisionProStreamer")
                    return
                }
                
                print("🔍 [DEBUG] Found Python client at: \(pythonIP)")
                print("⏳ [DEBUG] Waiting for WebRTC server info via gRPC...")
                
                // Wait for WebRTC server info to arrive via gRPC
                var webrtcInfo: (host: String, port: Int)?
                for attempt in 0..<60 {  // Try for 60 seconds
                    if let info = DataManager.shared.webrtcServerInfo {
                        webrtcInfo = info
                        print("✅ [DEBUG] WebRTC info received via gRPC: \(info.host):\(info.port)")
                        break
                    }
                    
                    if attempt % 10 == 0 && attempt > 0 {
                        print("⏳ [DEBUG] Still waiting for WebRTC server info... (\(attempt)s elapsed)")
                        print("💡 [DEBUG] Make sure start_video_streaming() was called in Python")
                    }
                    
                    try await Task.sleep(nanoseconds: 1_000_000_000)  // 1 second
                }
                
                guard let info = webrtcInfo else {
                    print("❌ [DEBUG] Timeout: WebRTC server info not received")
                    print("💡 [DEBUG] Make sure start_video_streaming() was called in Python")
                    return
                }
                
                print("🔗 [DEBUG] Connecting to WebRTC server at \(info.host):\(info.port)...")
                
                // Connect to WebRTC server
                let client = WebRTCClient()
                self.webrtcClient = client
                
                let renderer = VideoFrameRenderer(imageData: imageData)
                self.videoRenderer = renderer
                
                try await client.connect(to: info.host, port: info.port)
                print("✅ [DEBUG] WebRTC connection established!")
                client.addVideoRenderer(renderer)
            } catch {
                print("❌ [DEBUG] Failed to connect to WebRTC server: \(error)")
            }
        }
    }
    
    private func queryWebRTCInfo(pythonIP: String) async throws -> (host: String, port: Int) {
        let urlString = "http://\(pythonIP):8888/webrtc_info"
        print("🌐 [DEBUG] Querying URL: \(urlString)")
        
        guard let url = URL(string: urlString) else {
            print("❌ [DEBUG] Invalid URL: \(urlString)")
            throw NSError(domain: "WebRTC", code: -1, userInfo: [NSLocalizedDescriptionKey: "Invalid URL"])
        }
        
        do {
            let (data, response) = try await URLSession.shared.data(from: url)
            
            guard let httpResponse = response as? HTTPURLResponse else {
                print("❌ [DEBUG] Response is not HTTPURLResponse")
                throw NSError(domain: "WebRTC", code: -1, userInfo: [NSLocalizedDescriptionKey: "Invalid response type"])
            }
            
            print("📊 [DEBUG] HTTP Status: \(httpResponse.statusCode)")
            
            if httpResponse.statusCode != 200 {
                let responseString = String(data: data, encoding: .utf8) ?? "(no data)"
                print("❌ [DEBUG] HTTP error \(httpResponse.statusCode): \(responseString)")
                throw NSError(domain: "WebRTC", code: -1, userInfo: [NSLocalizedDescriptionKey: "HTTP \(httpResponse.statusCode)"])
            }
            
            let json = try JSONSerialization.jsonObject(with: data) as! [String: Any]
            print("📦 [DEBUG] Received JSON: \(json)")
            
            let host = json["host"] as! String
            let port = json["port"] as! Int
            
            return (host, port)
        } catch let error as NSError {
            print("❌ [DEBUG] URL request failed: \(error.localizedDescription)")
            print("💡 [DEBUG] Error domain: \(error.domain), code: \(error.code)")
            throw error
        }
    }
    
    func stop() {
        isRunning = false
        webrtcClient?.disconnect()
        webrtcClient = nil
        videoRenderer = nil
    }
}

/// Renders video frames from WebRTC to UIImage
class VideoFrameRenderer: NSObject, LKRTCVideoRenderer {
    weak var imageData: ImageData?
    
    init(imageData: ImageData) {
        self.imageData = imageData
        super.init()
    }
    
    func setSize(_ size: CGSize) {
        // Optional: log or adjust layout
    }
    
    func renderFrame(_ frame: LKRTCVideoFrame?) {
        guard let frame = frame else { return }
        
        // Extract or convert to CVPixelBuffer
        let pixelBuffer = extractPixelBuffer(from: frame)
        guard let cvPixelBuffer = pixelBuffer else { return }
        
        // Convert CVPixelBuffer to UIImage
        let ciImage = CIImage(cvPixelBuffer: cvPixelBuffer)
        let context = CIContext()
        guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else { return }
        let uiImage = UIImage(cgImage: cgImage)
        
        DispatchQueue.main.async { [weak self] in
            // Check if stereo mode is enabled
            let isStereo = DataManager.shared.stereoEnabled
            
            if isStereo {
                // Split side-by-side image into left and right
                let (leftImage, rightImage) = self?.splitSideBySideImage(uiImage) ?? (uiImage, uiImage)
                self?.imageData?.left = leftImage
                self?.imageData?.right = rightImage
            } else {
                // Mono mode: use same image for both
                self?.imageData?.left = uiImage
                self?.imageData?.right = uiImage
            }
        }
    }
    
    private func splitSideBySideImage(_ image: UIImage) -> (UIImage, UIImage) {
        guard let cgImage = image.cgImage else {
            return (image, image)
        }
        
        let width = cgImage.width
        let height = cgImage.height
        let halfWidth = width / 2
        
        // Crop left half
        let leftRect = CGRect(x: 0, y: 0, width: halfWidth, height: height)
        guard let leftCGImage = cgImage.cropping(to: leftRect) else {
            return (image, image)
        }
        let leftImage = UIImage(cgImage: leftCGImage)
        
        // Crop right half
        let rightRect = CGRect(x: halfWidth, y: 0, width: halfWidth, height: height)
        guard let rightCGImage = cgImage.cropping(to: rightRect) else {
            return (image, image)
        }
        let rightImage = UIImage(cgImage: rightCGImage)
        
        return (leftImage, rightImage)
    }
    
    private func extractPixelBuffer(from frame: LKRTCVideoFrame) -> CVPixelBuffer? {
        let conversionStart = CFAbsoluteTimeGetCurrent()
        let buffer = frame.buffer
        if let cv = buffer as? LKRTCCVPixelBuffer {
            let elapsed = (CFAbsoluteTimeGetCurrent() - conversionStart) * 1000
            print("DEBUG: CVPixelBuffer direct conversion took \(String(format: "%.2f", elapsed))ms")
            return cv.pixelBuffer
        }
        // Convert I420 to BGRA into a new CVPixelBuffer
        guard let i420 = buffer.toI420() as? LKRTCI420Buffer else { return nil }
        let width = Int(buffer.width)
        let height = Int(buffer.height)
        print("DEBUG: Starting I420->BGRA conversion for \(width)x\(height) frame")
        
        var output: CVPixelBuffer?
        let options = [
            kCVPixelBufferCGImageCompatibilityKey as String: true,
            kCVPixelBufferCGBitmapContextCompatibilityKey as String: true
        ] as CFDictionary
        
        let status = CVPixelBufferCreate(
            kCFAllocatorDefault,
            width,
            height,
            kCVPixelFormatType_32ARGB,  // Use ARGB directly
            options,
            &output
        )
        guard status == kCVReturnSuccess, let out = output else { return nil }
        
        CVPixelBufferLockBaseAddress(out, [])
        defer { CVPixelBufferUnlockBaseAddress(out, []) }
        
        guard let base = CVPixelBufferGetBaseAddress(out) else { return nil }
        let bytesPerRow = CVPixelBufferGetBytesPerRow(out)
        
        let yPlane = i420.dataY
        let uPlane = i420.dataU
        let vPlane = i420.dataV
        let yStride = Int(i420.strideY)
        let uStride = Int(i420.strideU)
        let vStride = Int(i420.strideV)
        
        // Use Accelerate framework for hardware-accelerated YUV to ARGB conversion
        let loopStart = CFAbsoluteTimeGetCurrent()
        
        // Create vImage buffers for the planar YUV data
        var srcYPlane = vImage_Buffer(
            data: UnsafeMutableRawPointer(mutating: yPlane),
            height: vImagePixelCount(height),
            width: vImagePixelCount(width),
            rowBytes: yStride
        )
        
        var srcCbPlane = vImage_Buffer(
            data: UnsafeMutableRawPointer(mutating: uPlane),
            height: vImagePixelCount(height / 2),
            width: vImagePixelCount(width / 2),
            rowBytes: uStride
        )
        
        var srcCrPlane = vImage_Buffer(
            data: UnsafeMutableRawPointer(mutating: vPlane),
            height: vImagePixelCount(height / 2),
            width: vImagePixelCount(width / 2),
            rowBytes: vStride
        )
        
        var destARGB = vImage_Buffer(
            data: base,
            height: vImagePixelCount(height),
            width: vImagePixelCount(width),
            rowBytes: bytesPerRow
        )
        
        // YUV to ARGB conversion info (ITU-R BT.601)
        var pixelRange = vImage_YpCbCrPixelRange(Yp_bias: 0, CbCr_bias: 128, YpRangeMax: 255, CbCrRangeMax: 255, YpMax: 255, YpMin: 0, CbCrMax: 255, CbCrMin: 0)
        var infoYpCbCrToARGB = vImage_YpCbCrToARGB()
        vImageConvert_YpCbCrToARGB_GenerateConversion(
            kvImage_YpCbCrToARGBMatrix_ITU_R_601_4!,
            &pixelRange,
            &infoYpCbCrToARGB,
            kvImage420Yp8_Cb8_Cr8,
            kvImageARGB8888,
            vImage_Flags(kvImageNoFlags)
        )
        
        // Perform the conversion to ARGB
        let error = vImageConvert_420Yp8_Cb8_Cr8ToARGB8888(
            &srcYPlane,
            &srcCbPlane,
            &srcCrPlane,
            &destARGB,
            &infoYpCbCrToARGB,
            nil,
            255, // alpha value
            vImage_Flags(kvImageNoFlags)
        )
        
        if error != kvImageNoError {
            print("DEBUG: vImage conversion error: \(error)")
        }
        
        let loopElapsed = (CFAbsoluteTimeGetCurrent() - loopStart) * 1000
        let totalElapsed = (CFAbsoluteTimeGetCurrent() - conversionStart) * 1000
        print("DEBUG: vImage I420->BGRA conversion took \(String(format: "%.2f", loopElapsed))ms, total: \(String(format: "%.2f", totalElapsed))ms")
        return out
    }
}
