import SwiftUI
import RealityKit
import LiveKitWebRTC
import CoreImage
import Accelerate
import AVFAudio
import RealityKitContent

struct ImmersiveView: View {
    @EnvironmentObject var imageData: ImageData
    @StateObject private var videoStreamManager = VideoStreamManager()
    @StateObject private var appModel = 🥽AppModel()
    @ObservedObject private var dataManager = DataManager.shared
    @State private var updateTrigger = false
    @State private var hasFrames = false
    @State private var isMinimized = false
    @State private var showViewControls = false
    @State private var previewZDistance: Float? = nil
    @State private var previewActive = false  // Track if preview should be shown
    @State private var hasAutoMinimized = false  // Track if we've already auto-minimized
    @State private var userInteracted = false  // Track if user has manually changed minimized state
    @State private var currentAspectRatio: Float? = nil  // Track current video aspect ratio
    @State private var videoMinimized = false  // Track if video view is minimized
    @State private var previewStatusPosition: (x: Float, y: Float)? = nil  // Track preview status position
    @State private var previewStatusActive = false  // Track if status preview should be shown
    @State private var stereoMaterialEntity: Entity? = nil  // Store reference to RealityKit stereo material entity
    @State private var fixedWorldTransform: Transform? = nil  // Preserve world transform when locked
    
    var body: some View {
        RealityView { content, attachments in
            print("🟢 [ImmersiveView] RealityView content block called")
            
            // Create the video display anchor (still used for head pose reference)
            let videoAnchor = AnchorEntity(.head)
            videoAnchor.name = "videoAnchor"
            content.add(videoAnchor)
            
            // Create a world anchor that always owns the video root
            let worldAnchor = AnchorEntity(world: .zero)
            worldAnchor.name = "videoWorldAnchor"
            content.add(worldAnchor)
            
            // Container entity that holds the actual video planes
            let videoRoot = Entity()
            videoRoot.name = "videoRoot"
            videoRoot.setParent(worldAnchor)
            
            // Create the video display plane (initially hidden)
            let skyBox = createSkyBox()
            skyBox.isEnabled = false  // Hide until we have frames
            skyBox.name = "videoPlane"
            skyBox.setParent(videoRoot)
            
            // Create preview plane (gray, initially hidden)
            let previewPlane = createPreviewPlane()
            previewPlane.name = "previewPlane"
            previewPlane.isEnabled = false
            previewPlane.setParent(videoRoot)
            
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
            
            // Create preview status container entity (initially hidden)
            let statusPreviewContainer = Entity()
            statusPreviewContainer.name = "statusPreviewContainer"
            statusPreviewContainer.setParent(statusAnchor)
            
            // Initialize at the correct Z position to avoid flying in from far away
            statusPreviewContainer.transform.translation = SIMD3<Float>(
                dataManager.statusMinimizedXPosition,
                dataManager.statusMinimizedYPosition,
                -1.0
            )
            
            // Attach the status preview UI to the preview container
            if let statusPreviewAttachment = attachments.entity(for: "statusPreview") {
                print("🟢 [ImmersiveView] Status preview attachment found and attached")
                statusPreviewAttachment.setParent(statusPreviewContainer)
                statusPreviewContainer.isEnabled = false
            } else {
                print("🔴 [ImmersiveView] Status preview attachment NOT found!")
            }
        } update: { updateContent, attachments in
            // This will be triggered when updateTrigger changes (i.e., when new frames arrive)
            let _ = updateTrigger  // Explicitly depend on updateTrigger
            let _ = dataManager.videoPlaneZDistance  // React to z-distance changes
            let _ = dataManager.videoPlaneYPosition  // React to y-position changes
            let _ = dataManager.videoPlaneAutoPerpendicular  // React to tilt setting changes
            let _ = dataManager.videoPlaneFixedToWorld  // React to anchor mode changes
            let _ = previewZDistance  // React to preview changes
            let _ = currentAspectRatio  // React to aspect ratio changes
            let _ = dataManager.statusMinimizedXPosition  // React to status position changes
            let _ = dataManager.statusMinimizedYPosition  // React to status position changes
            let _ = previewStatusPosition  // React to status preview changes
            let _ = previewStatusActive  // React to status preview active state
            let _ = isMinimized  // React to minimized state changes
            
            func findEntity(named name: String, in collection: RealityViewEntityCollection) -> Entity? {
                for entity in collection {
                    if entity.name == name {
                        return entity
                    }
                    if let nested = entity.findEntity(named: name) {
                        return nested
                    }
                }
                return nil
            }

            guard let videoRoot = findEntity(named: "videoRoot", in: updateContent.entities) else {
                return
            }

            let skyBoxEntity = videoRoot.findEntity(named: "videoPlane")
            let previewEntity = videoRoot.findEntity(named: "previewPlane")
            let headAnchor = findEntity(named: "videoAnchor", in: updateContent.entities) as? AnchorEntity
            let worldAnchor = findEntity(named: "videoWorldAnchor", in: updateContent.entities) as? AnchorEntity

            // Check if we have actual frames available RIGHT NOW
            let framesAvailable = imageData.left != nil && imageData.right != nil
            print("DEBUG: Update block called, left=\(imageData.left != nil), right=\(imageData.right != nil), framesAvailable=\(framesAvailable)")
            print("🎬 [ImmersiveView] Preview z-distance: \(String(describing: previewZDistance))")

            let isFixed = dataManager.videoPlaneFixedToWorld
            let shouldShowPreview = previewZDistance != nil || previewActive
            let targetZ = previewZDistance ?? dataManager.videoPlaneZDistance
            let targetY = dataManager.videoPlaneYPosition

            if isFixed {
                guard let worldAnchor else { return }
                
                // Ensure the panel lives under the world anchor so it stays put
                if videoRoot.parent !== worldAnchor {
                    videoRoot.setParent(worldAnchor, preservingWorldTransform: false)
                    print("🔒 [ImmersiveView] Reparented videoRoot to worldAnchor")
                }
                
                if let lockedTransform = fixedWorldTransform {
                    videoRoot.move(to: lockedTransform, relativeTo: worldAnchor, duration: 0.1, timingFunction: .linear)
                }
            } else {
                guard let headAnchor else { return }
                // Follow the user's head by parenting to the head anchor
                if videoRoot.parent !== headAnchor {
                    videoRoot.setParent(headAnchor, preservingWorldTransform: true)
                }
                fixedWorldTransform = nil
                var offsetTransform = Transform()
                offsetTransform.translation = SIMD3<Float>(0.0, targetY, targetZ)
                if dataManager.videoPlaneAutoPerpendicular {
                    let distance = abs(targetZ)
                    let angle = atan2(targetY, distance)
                    offsetTransform.rotation = simd_quatf(angle: -angle, axis: SIMD3<Float>(1, 0, 0))
                }
                let duration: TimeInterval = shouldShowPreview ? 0.1 : 0.3
                let timing: AnimationTimingFunction = shouldShowPreview ? .linear : .easeInOut
                videoRoot.move(to: offsetTransform, relativeTo: headAnchor, duration: duration, timingFunction: timing)
            }

            if shouldShowPreview && !framesAvailable {
                previewEntity?.isEnabled = true
            } else {
                previewEntity?.isEnabled = false
            }
            
            // Update status container position based on minimized state (do this BEFORE early return)
            if let statusAnchor = updateContent.entities.first(where: { $0.name == "statusHeadAnchor" }) as? AnchorEntity {
                if let statusContainer = statusAnchor.children.first(where: { $0.name == "statusContainer" }) {
                    // When minimized, use custom position; when maximized, use (0, 0, -1.0)
                    let targetTranslation: SIMD3<Float>
                    if isMinimized {
                        targetTranslation = SIMD3<Float>(
                            dataManager.statusMinimizedXPosition,
                            dataManager.statusMinimizedYPosition,
                            -1.0
                        )
                    } else {
                        // Maximized stays at (0, 0, -1.0)
                        targetTranslation = SIMD3<Float>(0.0, 0.0, -1.0)
                    }
                    
                    // Animate the position change
                    var transform = statusContainer.transform
                    transform.translation = targetTranslation
                    statusContainer.move(to: transform, relativeTo: statusContainer.parent, duration: 0.5, timingFunction: .easeInOut)
                }
                
                // Handle status preview
                if let statusPreviewContainer = statusAnchor.children.first(where: { $0.name == "statusPreviewContainer" }) {
                    let shouldShowPreview = previewStatusPosition != nil || previewStatusActive
                    
                    if shouldShowPreview {
                        let xPos = previewStatusPosition?.x ?? dataManager.statusMinimizedXPosition
                        let yPos = previewStatusPosition?.y ?? dataManager.statusMinimizedYPosition
                        
                        print("🎭 [ImmersiveView] Showing status preview at x=\(xPos), y=\(yPos)")
                        statusPreviewContainer.isEnabled = true
                        var previewTransform = statusPreviewContainer.transform
                        previewTransform.translation = SIMD3<Float>(xPos, yPos, -1.0)
                        statusPreviewContainer.move(to: previewTransform, relativeTo: statusPreviewContainer.parent, duration: 0.1, timingFunction: .linear)
                    } else {
                        statusPreviewContainer.isEnabled = false
                    }
                }
            }
            
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
            
            // Calculate aspect ratio from incoming image
            let imageWidth = imageRight.size.width
            let imageHeight = imageRight.size.height
            let aspectRatio = Float(imageWidth / imageHeight)
            
            // Update plane geometry if aspect ratio changed
            if currentAspectRatio == nil || abs(currentAspectRatio! - aspectRatio) > 0.01 {
                print("DEBUG: Updating plane geometry for aspect ratio: \(aspectRatio) (was: \(currentAspectRatio ?? 0))")
                currentAspectRatio = aspectRatio
                
                // Use a fixed height and calculate width based on aspect ratio
                // This preserves the original aspect ratio of the video
                let planeHeight: Float = 9.6
                let planeWidth = planeHeight * aspectRatio
                
                let newMesh = MeshResource.generatePlane(width: planeWidth, height: planeHeight)
                skyBox.components[ModelComponent.self]?.mesh = newMesh
                
                // Also update preview plane if it exists
                if let preview = previewEntity {
                    preview.components[ModelComponent.self]?.mesh = newMesh
                }
            }
            
            // Show video only if not minimized
            skyBox.isEnabled = !videoMinimized
            
            // Auto-minimize when frames first arrive (only once, and never if user has interacted)
            if !hasFrames {
                hasFrames = true
                // Only auto-minimize if user hasn't manually changed the state
                if !hasAutoMinimized && !userInteracted {
                    // Delay minimization slightly for smooth transition
                    Task { @MainActor in
                        try? await Task.sleep(nanoseconds: 1_000_000_000) // 1 second
                        // Only minimize if user hasn't already interacted with it
                        if !hasAutoMinimized && !userInteracted {
                            withAnimation(.spring(response: 0.6, dampingFraction: 0.8)) {
                                isMinimized = true
                                hasAutoMinimized = true
                            }
                        }
                    }
                }
            }
            
            // Check if stereo mode is enabled
            let isStereo = DataManager.shared.stereoEnabled
            print("DEBUG: Stereo mode: \(isStereo)")
            
            if isStereo {
                // Stereo mode: Use the loaded ShaderGraphMaterial from RealityKitContent
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
                    
                    // Get the stereo material from the loaded RealityKit entity
                    if let sphereEntity = stereoMaterialEntity,
                       var stereoMaterial = sphereEntity.components[ModelComponent.self]?.materials.first as? ShaderGraphMaterial {
                        // Update the material parameters with new textures
                        try stereoMaterial.setParameter(name: "left", value: .textureResource(leftTexture))
                        try stereoMaterial.setParameter(name: "right", value: .textureResource(rightTexture))
                        
                        // Apply the stereo material to the video plane
                        skyBox.components[ModelComponent.self]?.materials = [stereoMaterial]
                        print("✅ DEBUG: Updated stereo textures successfully (left + right)")
                    } else {
                        // Fallback if stereo material isn't loaded yet
                        print("⚠️ WARNING: StereoMaterial not loaded yet, falling back to mono display")
                        var skyBoxMaterial = UnlitMaterial()
                        skyBoxMaterial.color = .init(texture: .init(rightTexture))
                        skyBox.components[ModelComponent.self]?.materials = [skyBoxMaterial]
                    }
                } catch {
                    print("❌ ERROR: Failed to load stereo textures: \(error)")
                }
            } else {
                // Mono mode: Use simple unlit material with alpha support
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
                    print("❌ ERROR: Failed to load mono texture: \(error)")
                }
            }
        } attachments: {
            Attachment(id: "status") {
                StatusOverlay(
                    hasFrames: $hasFrames, 
                    isMinimized: $isMinimized,
                    showViewControls: $showViewControls,
                    previewZDistance: $previewZDistance,
                    previewActive: $previewActive,
                    userInteracted: $userInteracted,
                    videoMinimized: $videoMinimized,
                    videoFixed: Binding(
                        get: { dataManager.videoPlaneFixedToWorld },
                        set: { newValue in dataManager.videoPlaneFixedToWorld = newValue }
                    ),
                    previewStatusPosition: $previewStatusPosition,
                    previewStatusActive: $previewStatusActive
                )
                .frame(maxWidth: 300)
            }
            
            Attachment(id: "statusPreview") {
                StatusPreviewView(
                    showVideoStatus: true,
                    videoFixed: dataManager.videoPlaneFixedToWorld
                )
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
            
            // Load the stereo material from RealityKitContent
            Task {
                if let scene = try? await Entity(named: "Immersive", in: realityKitContentBundle) {
                    // Find the sphere with stereo material
                    if let sphereEntity = scene.findEntity(named: "Sphere") {
                        sphereEntity.isEnabled = false  // Hide the sphere, we just need its material
                        await MainActor.run {
                            self.stereoMaterialEntity = sphereEntity
                        }
                        print("✅ [ImmersiveView] Loaded stereo material from RealityKitContent")
                    } else {
                        print("⚠️ [ImmersiveView] Could not find Sphere entity in Immersive scene")
                    }
                } else {
                    print("⚠️ [ImmersiveView] Could not load Immersive scene from RealityKitContent")
                }
            }
        }
        .onChange(of: dataManager.pythonClientIP) { oldValue, newValue in
            if newValue == nil {
                print("🔌 [ImmersiveView] Python client disconnected - clearing state")
                // Clear the video frames
                imageData.left = nil
                imageData.right = nil
                hasFrames = false
                videoMinimized = false
                hasAutoMinimized = false
                fixedWorldTransform = nil
                // Stop the video stream
                videoStreamManager.stop()
            }
        }
        .onChange(of: dataManager.webrtcGeneration) { oldValue, newValue in
            if newValue < 0 {
                // Disconnection detected
                print("🔌 [ImmersiveView] WebRTC disconnected (generation: \(newValue)) - clearing state")
                imageData.left = nil
                imageData.right = nil
                hasFrames = false
                videoMinimized = false
                fixedWorldTransform = nil
                videoStreamManager.stop()
            } else if newValue > 0 && oldValue != newValue {
                // New connection or reconnection
                print("🔄 [ImmersiveView] WebRTC generation changed to \(newValue), restarting stream...")
                videoStreamManager.stop()
                // Give it a moment to cleanup
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                    videoStreamManager.start(imageData: imageData)
                }
            }
        }
        .onChange(of: dataManager.videoPlaneFixedToWorld) { oldValue, isFixed in
            if isFixed {
                print("🔒 [ImmersiveView] Fixed Mode ENABLED - Capturing Transform")
                // Capture head transform immediately when toggled
                let headWorldMatrix = DataManager.shared.latestHandTrackingData.Head
                
                let targetY = dataManager.videoPlaneYPosition
                let targetZ = dataManager.videoPlaneZDistance
                
                var offsetTransform = Transform()
                offsetTransform.translation = SIMD3<Float>(0.0, targetY, targetZ)
                if dataManager.videoPlaneAutoPerpendicular {
                    let distance = abs(targetZ)
                    let angle = atan2(targetY, distance)
                    offsetTransform.rotation = simd_quatf(angle: -angle, axis: SIMD3<Float>(1, 0, 0))
                }
                
                // World = Head * Offset
                let worldMatrix = simd_mul(headWorldMatrix, offsetTransform.matrix)
                fixedWorldTransform = Transform(matrix: worldMatrix)
                print("🔒 [ImmersiveView] Captured Fixed Transform: \(fixedWorldTransform?.translation ?? .zero)")
            } else {
                print("🔓 [ImmersiveView] Fixed Mode DISABLED")
                fixedWorldTransform = nil
            }
        }
        .onDisappear {
            print("DEBUG: ImmersiveView disappeared, stopping video stream")
            videoStreamManager.stop()
            fixedWorldTransform = nil
        }
    }
}

private func createSkyBox() -> Entity {
    let skyBoxEntity = Entity()
    // Start with a default 16:9 aspect ratio plane
    // This will be updated dynamically based on the incoming video's actual aspect ratio
    let defaultHeight: Float = 9.6
    let defaultWidth: Float = defaultHeight * (16.0 / 9.0)  // 16:9 aspect ratio
    let largePlane = MeshResource.generatePlane(width: defaultWidth, height: defaultHeight)
    var skyBoxMaterial = UnlitMaterial()
    skyBoxMaterial.color = .init(tint: .clear)
    skyBoxEntity.components.set(
        ModelComponent(mesh: largePlane, materials: [skyBoxMaterial])
    )
    return skyBoxEntity
}

private func createPreviewPlane() -> Entity {
    let previewEntity = Entity()
    // Start with a default 16:9 aspect ratio plane
    // This will be updated dynamically based on the incoming video's actual aspect ratio
    let defaultHeight: Float = 9.6
    let defaultWidth: Float = defaultHeight * (16.0 / 9.0)  // 16:9 aspect ratio
    let largePlane = MeshResource.generatePlane(width: defaultWidth, height: defaultHeight)
    var previewMaterial = UnlitMaterial()
    // Gray semi-transparent material for preview
    previewMaterial.color = .init(tint: .init(white: 0.5, alpha: 0.6))
    previewEntity.components.set(
        ModelComponent(mesh: largePlane, materials: [previewMaterial])
    )
    return previewEntity
}

/// Manages the WebRTC video stream connection and frame processing
@MainActor
class VideoStreamManager: ObservableObject {
    private var webrtcClient: WebRTCClient?
    private var videoRenderer: VideoFrameRenderer?
    private var audioRenderer: AudioFrameRenderer?
    private var isRunning = false
    
    func start(imageData: ImageData) {
        // If already running, we might want to restart if called explicitly, 
        // but for now let's respect the flag unless stop() was called.
        if isRunning {
            print("⚠️ [DEBUG] VideoStreamManager.start() called but already running. Ignoring.")
            return 
        }
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
                
                let videoRenderer = VideoFrameRenderer(imageData: imageData)
                self.videoRenderer = videoRenderer
                
                let audioRenderer = AudioFrameRenderer()
                self.audioRenderer = audioRenderer
                
                try await client.connect(to: info.host, port: info.port)
                print("✅ [DEBUG] WebRTC connection established!")
                let stereoVideo = DataManager.shared.stereoEnabled
                let stereoAudio = DataManager.shared.stereoAudioEnabled
                print("📊 [DEBUG] Stereo modes - Video: \(stereoVideo), Audio: \(stereoAudio)")
                client.addVideoRenderer(videoRenderer)
                client.addAudioRenderer(audioRenderer)
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
        audioRenderer = nil
    }
}

/// Renders video frames from WebRTC to UIImage
class VideoFrameRenderer: NSObject, LKRTCVideoRenderer {
    weak var imageData: ImageData?
    // Reuse CIContext for performance (creating it every frame is expensive)
    private let context = CIContext()
    
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
        // Use the reused context
        guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else { return }
        let uiImage = UIImage(cgImage: cgImage)
        
        // Update image data directly
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            let isStereo = DataManager.shared.stereoEnabled
            
            if isStereo {
                let (leftImage, rightImage) = self.splitSideBySideImage(uiImage)
                self.imageData?.left = leftImage
                self.imageData?.right = rightImage
            } else {
                self.imageData?.left = uiImage
                self.imageData?.right = uiImage
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
}

extension VideoFrameRenderer {
    private func extractPixelBuffer(from frame: LKRTCVideoFrame) -> CVPixelBuffer? {
        let buffer = frame.buffer
        if let cv = buffer as? LKRTCCVPixelBuffer {
            return cv.pixelBuffer
        }
        // Convert I420 to BGRA into a new CVPixelBuffer
        guard let i420 = buffer.toI420() as? LKRTCI420Buffer else { return nil }
        let width = Int(buffer.width)
        let height = Int(buffer.height)
        
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
        // Use video range (limited range) which is standard for WebRTC/H.264
        // Y: 16-235, CbCr: 16-240
        var pixelRange = vImage_YpCbCrPixelRange(
            Yp_bias: 16,
            CbCr_bias: 128,
            YpRangeMax: 235,
            CbCrRangeMax: 240,
            YpMax: 255,
            YpMin: 0,
            CbCrMax: 255,
            CbCrMin: 0
        )
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
        
        // Silent error handling - don't spam logs
        guard error == kvImageNoError else { return nil }
        
        return out
    }
}

/// Renders audio frames from WebRTC to device speakers
class AudioFrameRenderer: NSObject, LKRTCAudioRenderer {
    private var audioEngine: AVAudioEngine?
    private var playerNode: AVAudioPlayerNode?
    private var audioFormat: AVAudioFormat?
    private var bufferCount = 0
    private let audioQueue = DispatchQueue(label: "com.visionpro.audio", qos: .userInteractive)
    
    override init() {
        super.init()
        print("🔊 AudioFrameRenderer initialized")
    }
    
    private func setupAudioEngine(format: AVAudioFormat) {
        guard audioEngine == nil else { return }
        
        let channelMode = format.channelCount == 2 ? "STEREO" : "MONO"
        print("🔊 Setting up audio engine with format:")
        print("   - Sample rate: \(format.sampleRate) Hz")
        print("   - Channels: \(format.channelCount) [\(channelMode)]")
        
        audioEngine = AVAudioEngine()
        playerNode = AVAudioPlayerNode()
        
        guard let engine = audioEngine, let player = playerNode else {
            print("❌ Failed to create audio engine or player node")
            return
        }
        
        engine.attach(player)
        
        // Connect player to mixer with the incoming audio format
        engine.connect(player, to: engine.mainMixerNode, format: format)
        
        // Prepare engine (pre-allocates resources)
        engine.prepare()
        
        do {
            try engine.start()
            player.play()
            print("✅ Audio engine started successfully!")
        } catch {
            print("❌ Failed to start audio engine: \(error)")
        }
        
        self.audioFormat = format
    }
    
    func render(pcmBuffer: AVAudioPCMBuffer) {
        // Fast early return checks (avoid logging overhead)
        guard let player = playerNode, let engine = audioEngine else {
            if audioEngine == nil {
                setupAudioEngine(format: pcmBuffer.format)
                return
            }
            return
        }
        
        guard engine.isRunning else { return }
        
        // Quick format validation without expensive logging
        guard let expectedFormat = audioFormat, 
              pcmBuffer.format.sampleRate == expectedFormat.sampleRate,
              pcmBuffer.format.channelCount == expectedFormat.channelCount else {
            return
        }
        
        // Schedule buffer on background queue to avoid blocking WebRTC thread
        audioQueue.async { [weak player] in
            guard let player = player else { return }
            player.scheduleBuffer(pcmBuffer, completionHandler: nil)
            
            if !player.isPlaying {
                player.play()
            }
        }
        
        // Log only first buffer
        if bufferCount == 0 {
            print("🔊 Audio streaming started")
        }
        bufferCount += 1
    }
}
