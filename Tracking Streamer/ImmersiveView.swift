import SwiftUI
import RealityKit
import LiveKitWebRTC
import CoreImage
import Accelerate
import CoreGraphics
import RealityKit
import RealityKitContent
// import Draco // Not needed, using Bridging Header



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
    @StateObject private var pointCloudRenderer = PointCloudRenderer()

    
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

            // Create point cloud root
            let pcRoot = Entity()
            pcRoot.name = "pcRoot"
            pcRoot.setParent(worldAnchor)
            pointCloudRenderer.setup(root: pcRoot)

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
                    // Get the stereo material from the loaded RealityKit entity
                    guard let sphereEntity = stereoMaterialEntity,
                          var stereoMaterial = sphereEntity.components[ModelComponent.self]?.materials.first as? ShaderGraphMaterial else {
                        // Fallback if stereo material isn't loaded yet
                        print("⚠️ WARNING: StereoMaterial not loaded yet, falling back to mono display")
                        var skyBoxMaterial = UnlitMaterial()
                        var textureOptions = TextureResource.CreateOptions(semantic: .hdrColor)
                        textureOptions.mipmapsMode = .none
                        let texture = try TextureResource.generate(from: imageRight.cgImage!, options: textureOptions)
                        skyBoxMaterial.color = .init(texture: .init(texture))
                        skyBox.components[ModelComponent.self]?.materials = [skyBoxMaterial]
                        return
                    }
                    
                    // Use .hdrColor for linear color space without conversions
                    var textureOptions = TextureResource.CreateOptions(semantic: .hdrColor)
                    textureOptions.mipmapsMode = .none  // No mipmaps to avoid filtering artifacts
                    
                    // Generate textures directly from CGImages (skip UIImage wrapper overhead)
                    let leftTexture = try TextureResource.generate(
                        from: imageLeft.cgImage!,
                        options: textureOptions
                    )
                    let rightTexture = try TextureResource.generate(
                        from: imageRight.cgImage!,
                        options: textureOptions
                    )
                    
                    // Update the material parameters with new textures
                    try stereoMaterial.setParameter(name: "left", value: .textureResource(leftTexture))
                    try stereoMaterial.setParameter(name: "right", value: .textureResource(rightTexture))
                    
                    // Apply the stereo material to the video plane
                    skyBox.components[ModelComponent.self]?.materials = [stereoMaterial]
                    print("✅ DEBUG: Updated stereo textures successfully (left + right)")
                } catch {
                    print("❌ ERROR: Failed to load stereo textures: \(error)")
                }
            } else {
                // Mono mode: Use simple unlit material with alpha support
                var skyBoxMaterial = UnlitMaterial()
                do {
                    // Use right image for mono display
                    // Use .hdrColor for linear color space without conversions
                    var textureOptions = TextureResource.CreateOptions(semantic: .hdrColor)
                    textureOptions.mipmapsMode = .none  // No mipmaps to avoid filtering artifacts
                    
                    let texture = try TextureResource.generate(
                        from: imageRight.cgImage!,
                        options: textureOptions
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
            videoStreamManager.start(imageData: imageData, pointCloudRenderer: pointCloudRenderer)
            
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
                    videoStreamManager.start(imageData: imageData, pointCloudRenderer: pointCloudRenderer)
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
    
    func start(imageData: ImageData, pointCloudRenderer: PointCloudRenderer? = nil) {
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
                        print("💡 [DEBUG] Make sure start_streaming() was called in Python")
                    }
                    
                    try await Task.sleep(nanoseconds: 1_000_000_000)  // 1 second
                }
                
                guard let info = webrtcInfo else {
                    print("❌ [DEBUG] Timeout: WebRTC server info not received")
                    print("💡 [DEBUG] Make sure start_streaming() was called in Python")
                    return
                }
                
                print("🔗 [DEBUG] Connecting to WebRTC server at \(info.host):\(info.port)...")
                
                // Connect to WebRTC server
                let client = WebRTCClient()
                self.webrtcClient = client
                
                // Inject client into PointCloudRenderer
                pointCloudRenderer?.webRTCClient = client
                
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

                // Hook up point cloud callback
                client.onPointCloudReceived = { [weak pointCloudRenderer] data in
                    // print("DEBUG: [ImmersiveView] Callback triggered with \(data.count) bytes")
                    Task { @MainActor in
                        if pointCloudRenderer == nil {
                            print("⚠️ [ImmersiveView] pointCloudRenderer is NIL!")
                        }
                        pointCloudRenderer?.processData(data)
                    }
                }



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
    private let benchmarkMagic: UInt8 = 0x5A
    private let benchmarkRows = 8
    private let benchmarkCols = 9
    private let benchmarkBlockSize = 16
    private let benchmarkMargin = 8
    private var lastBenchmarkSequence: UInt32?
    private let benchmarkThreshold = 127
    
    // Stats tracking
    private var frameCount: Int = 0
    private var lastFpsUpdateTime: TimeInterval = 0
    
    init(imageData: ImageData) {
        self.imageData = imageData
        super.init()
    }
    
    func setSize(_ size: CGSize) {
        // Optional: log or adjust layout
    }
    
    func renderFrame(_ frame: LKRTCVideoFrame?) {
        guard let frame = frame else { return }
        
        // Update stats (FPS and Resolution)
        let currentTime = CACurrentMediaTime()
        frameCount += 1
        
        if currentTime - lastFpsUpdateTime >= 1.0 {
            let fps = frameCount
            let width = Int(frame.width)
            let height = Int(frame.height)
            let resolutionString = "\(width)x\(height)"
            
            DispatchQueue.main.async {
                DataManager.shared.videoFPS = fps
                if DataManager.shared.videoResolution != resolutionString {
                    DataManager.shared.videoResolution = resolutionString
                }
            }
            
            frameCount = 0
            lastFpsUpdateTime = currentTime
        }
        
        // Extract or convert to CVPixelBuffer
        let pixelBuffer = extractPixelBuffer(from: frame)
        guard let cvPixelBuffer = pixelBuffer else { return }
        
        // Convert CVPixelBuffer to UIImage
        if let payload = detectBenchmarkPayload(pixelBuffer: cvPixelBuffer) {
            handleBenchmarkPayload(payload)
        }
        
        let ciImage = CIImage(cvPixelBuffer: cvPixelBuffer)
        // Use the reused context
        guard let cgImage = context.createCGImage(ciImage, from: ciImage.extent) else { return }
        let uiImage = UIImage(cgImage: cgImage)

        // Update image data directly
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            let isStereo = DataManager.shared.stereoEnabled
            
            if isStereo {
                // Optimize: Return CGImages directly instead of wrapping in UIImage
                if let (leftCG, rightCG) = self.splitSideBySideImage(uiImage) {
                    self.imageData?.left = UIImage(cgImage: leftCG)
                    self.imageData?.right = UIImage(cgImage: rightCG)
                }
            } else {
                self.imageData?.left = uiImage
                self.imageData?.right = uiImage
            }
        }
    }

    
    private func splitSideBySideImage(_ image: UIImage) -> (CGImage, CGImage)? {
        guard let cgImage = image.cgImage else {
            return nil
        }
        
        let width = cgImage.width
        let height = cgImage.height
        let halfWidth = width / 2
        
        // Crop left half
        let leftRect = CGRect(x: 0, y: 0, width: halfWidth, height: height)
        guard let leftCGImage = cgImage.cropping(to: leftRect) else {
            return nil
        }
        
        // Crop right half
        let rightRect = CGRect(x: halfWidth, y: 0, width: halfWidth, height: height)
        guard let rightCGImage = cgImage.cropping(to: rightRect) else {
            return nil
        }
        
        return (leftCGImage, rightCGImage)
    }

    private func handleBenchmarkPayload(_ payload: (sequence: UInt32, sentTimestampMs: UInt32)) {
        if lastBenchmarkSequence == payload.sequence {
            return
        }
        lastBenchmarkSequence = payload.sequence
        let nowNanoseconds = DispatchTime.now().uptimeNanoseconds
        BenchmarkEventDispatcher.shared.emitDetection(
            sequenceID: payload.sequence,
            sentTimestampMs: payload.sentTimestampMs,
            detectedAtNanoseconds: nowNanoseconds
        )
        print("🧪 [Benchmark] Detected sequence \(payload.sequence) (sent_ms=\(payload.sentTimestampMs))")
    }

    private func detectBenchmarkPayload(pixelBuffer: CVPixelBuffer) -> (sequence: UInt32, sentTimestampMs: UInt32)? {
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        let requiredWidth = benchmarkMargin * 2 + benchmarkCols * benchmarkBlockSize
        let requiredHeight = benchmarkMargin * 2 + benchmarkRows * benchmarkBlockSize
        guard width >= requiredWidth, height >= requiredHeight else { return nil }

        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly) }

        let pixelFormat = CVPixelBufferGetPixelFormatType(pixelBuffer)
        guard pixelFormat == kCVPixelFormatType_32ARGB else { return nil }

        guard let baseAddress = CVPixelBufferGetBaseAddress(pixelBuffer) else { return nil }
        let rowBytes = CVPixelBufferGetBytesPerRow(pixelBuffer)
        let bytesPerPixel = 4

        let roiWidth = benchmarkCols * benchmarkBlockSize
        let roiHeight = benchmarkRows * benchmarkBlockSize
        let roiX = benchmarkMargin
        let roiY = benchmarkMargin
        let roiOffset = roiY * rowBytes + roiX * bytesPerPixel
        let roiPointer = baseAddress.advanced(by: roiOffset)

        var roiSource = vImage_Buffer(
            data: roiPointer,
            height: vImagePixelCount(roiHeight),
            width: vImagePixelCount(roiWidth),
            rowBytes: rowBytes
        )

        var roiLuma = vImage_Buffer()
        guard vImageBuffer_Init(&roiLuma, vImagePixelCount(roiHeight), vImagePixelCount(roiWidth), 8, vImage_Flags(kvImageNoFlags)) == kvImageNoError else {
            return nil
        }
        defer { free(roiLuma.data) }

        var coefficients: [Int16] = [0, 77, 150, 29]
        let divisor: Int32 = 256
        guard vImageMatrixMultiply_ARGB8888ToPlanar8(
            &roiSource,
            &roiLuma,
            &coefficients,
            divisor,
            nil,
            0,
            vImage_Flags(kvImageNoFlags)
        ) == kvImageNoError else {
            return nil
        }

        let downsampleCount = benchmarkRows * benchmarkCols
        let destPointer = UnsafeMutablePointer<UInt8>.allocate(capacity: downsampleCount)
        destPointer.initialize(repeating: 0, count: downsampleCount)
        defer { destPointer.deallocate() }

        var downsampled = vImage_Buffer(
            data: UnsafeMutableRawPointer(destPointer),
            height: vImagePixelCount(benchmarkRows),
            width: vImagePixelCount(benchmarkCols),
            rowBytes: benchmarkCols
        )

        guard vImageScale_Planar8(
            &roiLuma,
            &downsampled,
            nil,
            vImage_Flags(kvImageHighQualityResampling)
        ) == kvImageNoError else {
            return nil
        }

        var bits = [UInt8](repeating: 0, count: downsampleCount)
        for row in 0..<benchmarkRows {
            let rowPtr = destPointer.advanced(by: row * benchmarkCols)
            for col in 0..<benchmarkCols {
                let value = rowPtr[col]
                bits[row * benchmarkCols + col] = value > benchmarkThreshold ? 1 : 0
            }
        }

        guard bits.count >= 72 else { return nil }

        var magic: UInt8 = 0
        for bit in bits[0..<8] {
            magic = (magic << 1) | bit
        }
        guard magic == benchmarkMagic else { return nil }

        var sequence: UInt32 = 0
        for bit in bits[8..<(8 + 32)] {
            sequence = (sequence << 1) | UInt32(bit)
        }

        var sentTimestamp: UInt32 = 0
        for bit in bits[(8 + 32)..<(8 + 64)] {
            sentTimestamp = (sentTimestamp << 1) | UInt32(bit)
        }

        return (sequence, sentTimestamp)
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
        
        // Update DataManager with sample rate
        DispatchQueue.main.async {
            DataManager.shared.audioSampleRate = Int(format.sampleRate)
        }
        
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

class PointCloudBuffers {
    var positions: [SIMD3<Float>] = []
    var indices: [UInt32] = []
    var uvs: [SIMD2<Float>] = []
    var colors: [UInt8] = [] // RGBA bytes
    
    func reserve(numPoints: Int) {
        let numVertices = numPoints * 3
        if positions.capacity < numVertices {
            positions.reserveCapacity(numVertices)
        }
        if indices.capacity < numVertices {
            indices.reserveCapacity(numVertices)
        }
        if uvs.capacity < numVertices {
            uvs.reserveCapacity(numVertices)
        }
        if colors.capacity < numPoints * 4 {
            colors.reserveCapacity(numPoints * 4)
        }
    }
    
    func clear() {
        positions.removeAll(keepingCapacity: true)
        indices.removeAll(keepingCapacity: true)
        uvs.removeAll(keepingCapacity: true)
        colors.removeAll(keepingCapacity: true)
    }
}

class PointCloudRenderer: ObservableObject {
    private var root: Entity?
    private var meshEntityA: ModelEntity?
    private var meshEntityB: ModelEntity?
    private var activeBufferIndex = 0 // 0 for A, 1 for B
    private var localTransform: Transform = .identity
    var webRTCClient: WebRTCClient?
    private let buffers = PointCloudBuffers()
    
    func setup(root: Entity) {
        self.root = root
        
        // Create two entities for double buffering
        let entityA = ModelEntity()
        entityA.name = "pointCloudMeshA"
        entityA.setParent(root)
        
        let entityB = ModelEntity()
        entityB.name = "pointCloudMeshB"
        entityB.setParent(root)
        
        self.meshEntityA = entityA
        self.meshEntityB = entityB
        
        // Default material
        var material = UnlitMaterial()
        material.color = .init(tint: .cyan)
        
        let placeholderMesh = MeshResource.generateBox(size: 0.1)
        
        entityA.model = ModelComponent(mesh: placeholderMesh, materials: [material])
        entityA.isEnabled = false
        
        entityB.model = ModelComponent(mesh: placeholderMesh, materials: [material])
        entityB.isEnabled = false
    }
    
    func processData(_ data: Data) {
        let startTime = Date() // Capture start time
        // print("DEBUG: [PointCloudRenderer] processData called with \(data.count) bytes") 
        if data.count < 1 { return }
        
        var header = data[0]
        var payload = data.dropFirst()
        var benchmarkEchoData: Data? = nil
        
        if header == 0x04 || header == 0x07 || header == 0x09 {
            // Benchmark: [SeqID (4)] + [Timestamp (8)] + [Point Cloud Data]
            if payload.count >= 12 {
                let seqIdData = payload.prefix(4)
                let tsData = payload.dropFirst(4).prefix(8)
                
                var echoData = Data()
                echoData.append(seqIdData)
                echoData.append(tsData)
                
                if self.webRTCClient == nil {
                    print("⚠️ [PointCloud] webRTCClient is NIL! Cannot echo benchmark.")
                }
                
                // Defer echo until after mesh update
                benchmarkEchoData = echoData
                
                // Treat the rest as point cloud data
                payload = payload.dropFirst(12)
                
                // Map benchmark headers to data headers
                if header == 0x04 { header = 0x03 } // Float32 + Colors (assumed)
                else if header == 0x07 { header = 0x06 } // Float16 + Colors (assumed)
                else if header == 0x09 { header = 0x08 } // Draco
            }
        }
        
        if header == 0x01 {
            // ...
        } else if header == 0x08 {
            // Draco Compressed Data
            print("DEBUG: [PointCloud] Received Draco header (0x08), payload size: \(payload.count)")
            // Use DracoWrapper to decode
            if let decoded = DracoWrapper.decode(payload) {
                let numPoints = decoded.pointCount
                print("DEBUG: [PointCloud] Decoded \(numPoints) points from Draco")
                if numPoints == 0 { 
                    print("⚠️ [PointCloud] Decoded 0 points!")
                    return 
                }
                
                // 1. Prepare Buffers
                self.buffers.clear()
                self.buffers.reserve(numPoints: numPoints)
                
                let positionsData = decoded.positions
                let colorsData = decoded.colors
                let hasColors = (colorsData.count > 0)
                
                // 1. Prepare Buffers
                self.buffers.clear()
                self.buffers.reserve(numPoints: numPoints)
                
                // 2. Expand Data
                // Resize arrays first
                self.buffers.positions.append(contentsOf: repeatElement(.zero, count: numPoints * 3))
                self.buffers.indices.append(contentsOf: repeatElement(0, count: numPoints * 3))
                self.buffers.uvs.append(contentsOf: repeatElement(.zero, count: numPoints * 3))
                
                // OPTIMIZATION: Direct Color Copy
                // DracoWrapper now outputs RGBA (4 bytes per point).
                // We need to populate self.buffers.colors which is [UInt8] (4 bytes per point).
                // Since we are using 1 texture pixel per point, we just need to copy the buffer.
                if hasColors {
                    self.buffers.colors = Array(colorsData)
                } else {
                    self.buffers.colors = Array(repeating: 255, count: numPoints * 4)
                }
                
                let size: Float = 0.005 // 5mm points
                let halfSize = size * 0.5
                let height = size * 0.866
                
                positionsData.withUnsafeBytes { (pointsRaw: UnsafeRawBufferPointer) in
                    guard let inputPoints = pointsRaw.bindMemory(to: Float.self).baseAddress else { return }
                    
                    self.buffers.positions.withUnsafeMutableBufferPointer { positionsPtr in
                        self.buffers.indices.withUnsafeMutableBufferPointer { indicesPtr in
                            self.buffers.uvs.withUnsafeMutableBufferPointer { uvsPtr in
                                // Note: We don't need to touch colorsPtr in the loop anymore!
                                
                                // Parallel Loop
                                DispatchQueue.concurrentPerform(iterations: numPoints) { i in
                                    // 1. Vertices
                                    let x = inputPoints[i * 3]
                                    let y = inputPoints[i * 3 + 1]
                                    let z = inputPoints[i * 3 + 2]
                                    let center = SIMD3<Float>(x, y, z)
                                    
                                    let v0 = center + SIMD3<Float>(0, size, 0)
                                    let v1 = center + SIMD3<Float>(-height, -halfSize, 0)
                                    let v2 = center + SIMD3<Float>(height, -halfSize, 0)
                                    
                                    let baseIdx = i * 3
                                    positionsPtr[baseIdx] = v0
                                    positionsPtr[baseIdx + 1] = v1
                                    positionsPtr[baseIdx + 2] = v2
                                    
                                    let vertexIdx = UInt32(baseIdx)
                                    indicesPtr[baseIdx] = vertexIdx
                                    indicesPtr[baseIdx + 1] = vertexIdx + 1
                                    indicesPtr[baseIdx + 2] = vertexIdx + 2
                                    
                                    // 2. UVs
                                    // Map each triangle to a single pixel in the texture
                                    // U = (i + 0.5) / numPoints
                                    let u = (Float(i) + 0.5) / Float(numPoints)
                                    let uv = SIMD2<Float>(u, 0.5)
                                    uvsPtr[baseIdx] = uv
                                    uvsPtr[baseIdx + 1] = uv
                                    uvsPtr[baseIdx + 2] = uv
                                }
                            }
                        }
                    }
                }
                
                // 4. Create Mesh Descriptor
                var descriptor = MeshDescriptor(name: "pointCloudDraco")
                descriptor.positions = MeshBuffers.Positions(self.buffers.positions)
                descriptor.primitives = .triangles(self.buffers.indices)
                descriptor.textureCoordinates = MeshBuffers.TextureCoordinates(self.buffers.uvs)
                
                // DOUBLE BUFFERING LOGIC
                let backEntity = (activeBufferIndex == 0) ? meshEntityB : meshEntityA
                let frontEntity = (activeBufferIndex == 0) ? meshEntityA : meshEntityB
                
                guard let targetEntity = backEntity else { 
                    print("❌ [PointCloud] targetEntity is NIL")
                    return 
                }
                
                // 5. Update Mesh
                if let existingMesh = targetEntity.model?.mesh {
                    do {
                        let tempMesh = try MeshResource.generate(from: [descriptor])
                        try existingMesh.replace(with: tempMesh.contents)
                    } catch {
                        print("⚠️ [PointCloud] Failed to replace mesh: \(error)")
                        if let meshResource = try? MeshResource.generate(from: [descriptor]) {
                            targetEntity.model?.mesh = meshResource
                        }
                    }
                } else {
                    if let meshResource = try? MeshResource.generate(from: [descriptor]) {
                        targetEntity.model?.mesh = meshResource
                    }
                }
                
                // 6. Update Material/Texture
                var mat = UnlitMaterial()
                if hasColors {
                    if let texture = createTextureFromBuffer(width: numPoints) {
                         mat.color = .init(texture: .init(texture))
                    } else {
                         mat.color = .init(tint: .cyan)
                    }
                } else {
                    mat.color = .init(tint: .cyan)
                }
                
                targetEntity.model?.materials = [mat]
                
                // 7. Swap visibility
                targetEntity.isEnabled = true
                frontEntity?.isEnabled = false
                
                // 8. Toggle index
                activeBufferIndex = (activeBufferIndex == 0) ? 1 : 0
                
                // 9. Echo benchmark if needed
                if var echo = benchmarkEchoData {
                    let endTime = Date()
                    let processingTime = endTime.timeIntervalSince(startTime)
                    var procTime = processingTime
                    let procTimeData = Data(bytes: &procTime, count: MemoryLayout<Double>.size)
                    echo.append(procTimeData)
                    self.webRTCClient?.sendPointCloudData(echo)
                }
                
                // Force position if needed
                if self.root?.position.z == 0 && self.root?.position.y == 0 {
                     self.root?.position = SIMD3<Float>(0, 1.5, -1.0) 
                     print("⚠️ [PointCloud] Forced root position to (0, 1.5, -1.0)")
                }
            } else {
                print("❌ [PointCloud] DracoWrapper.decode returned NIL")
            }
            
        } else if header == 0x02 || header == 0x03 || header == 0x05 || header == 0x06 {
            // Point Cloud Data
            // 0x02: Float32 Points
            // 0x03: Float32 Points + Colors
            // 0x05: Float16 Points
            // 0x06: Float16 Points + Colors
            
            let isFloat16 = (header == 0x05 || header == 0x06)
            let hasColors = (header == 0x03 || header == 0x06)
            
            var numPoints = 0
            
            if isFloat16 {
                // Float16 = 2 bytes per coord -> 6 bytes per point
                if hasColors {
                    // 6 bytes + 3 bytes (RGB) = 9 bytes
                    numPoints = payload.count / 9
                } else {
                    numPoints = payload.count / 6
                }
            } else {
                // Float32 = 4 bytes per coord -> 12 bytes per point
                if hasColors {
                    // 12 bytes + 3 bytes (RGB) = 15 bytes
                    numPoints = payload.count / 15
                } else {
                    numPoints = payload.count / 12
                }
            }
            
            if numPoints == 0 { return }
            
            // 1. Prepare Buffers
            self.buffers.clear()
            self.buffers.reserve(numPoints: numPoints)
            
            // 2. Parse Data & Parallel Expansion
            let pointsByteSize = numPoints * (isFloat16 ? 6 : 12)
            let pointsData = payload.prefix(pointsByteSize)
            let colorsData = hasColors ? payload.suffix(from: pointsByteSize) : Data()
            
            // Resize arrays first
            self.buffers.positions.append(contentsOf: repeatElement(.zero, count: numPoints * 3))
            self.buffers.indices.append(contentsOf: repeatElement(0, count: numPoints * 3))
            if hasColors {
                self.buffers.uvs.append(contentsOf: repeatElement(.zero, count: numPoints * 3))
                self.buffers.colors.append(contentsOf: repeatElement(255, count: numPoints * 4)) // Alpha=255
            }
            
            let size: Float = 0.005 // 5mm points
            let halfSize = size * 0.5
            let height = size * 0.866
            
            pointsData.withUnsafeBytes { (pointsRaw: UnsafeRawBufferPointer) in
                // For Float16, we bind to Float16. For Float32, we bind to Float.
                // But UnsafeRawBufferPointer doesn't support generic binding easily in one block.
                // We'll handle it inside.
                
                let float32Base = !isFloat16 ? pointsRaw.bindMemory(to: Float.self).baseAddress : nil
                // Swift doesn't have a built-in Float16 type in standard library before recent versions,
                // but it is available in SwiftUI/Accelerate context usually.
                // Assuming Float16 is available (iOS 14+ / macOS 11+).
                let float16Base = isFloat16 ? pointsRaw.bindMemory(to: Float16.self).baseAddress : nil
                
                // Use unsafeBytes for colors to avoid copying
                colorsData.withUnsafeBytes { (colorsRaw: UnsafeRawBufferPointer) in
                    let inputColors = colorsRaw.bindMemory(to: UInt8.self).baseAddress
                    
                    self.buffers.positions.withUnsafeMutableBufferPointer { positionsPtr in
                        self.buffers.indices.withUnsafeMutableBufferPointer { indicesPtr in
                            self.buffers.uvs.withUnsafeMutableBufferPointer { uvsPtr in
                                self.buffers.colors.withUnsafeMutableBufferPointer { colorsPtr in
                                    
                                    // Parallel Loop
                                    DispatchQueue.concurrentPerform(iterations: numPoints) { i in
                                        // 1. Vertices
                                        var x: Float = 0
                                        var y: Float = 0
                                        var z: Float = 0
                                        
                                        if isFloat16 {
                                            if let base = float16Base {
                                                x = Float(base[i * 3])
                                                y = Float(base[i * 3 + 1])
                                                z = Float(base[i * 3 + 2])
                                            }
                                        } else {
                                            if let base = float32Base {
                                                x = base[i * 3]
                                                y = base[i * 3 + 1]
                                                z = base[i * 3 + 2]
                                            }
                                        }
                                        
                                        let center = SIMD3<Float>(x, y, z)
                                        
                                        let v0 = center + SIMD3<Float>(0, size, 0)
                                        let v1 = center + SIMD3<Float>(-height, -halfSize, 0)
                                        let v2 = center + SIMD3<Float>(height, -halfSize, 0)
                                        
                                        let baseIdx = i * 3
                                        positionsPtr[baseIdx] = v0
                                        positionsPtr[baseIdx + 1] = v1
                                        positionsPtr[baseIdx + 2] = v2
                                        
                                        let vertexIdx = UInt32(baseIdx)
                                        indicesPtr[baseIdx] = vertexIdx
                                        indicesPtr[baseIdx + 1] = vertexIdx + 1
                                        indicesPtr[baseIdx + 2] = vertexIdx + 2
                                        
                                        if hasColors {
                                            // 2. UVs
                                            let u = (Float(i) + 0.5) / Float(numPoints)
                                            let uv = SIMD2<Float>(u, 0.5)
                                            uvsPtr[baseIdx] = uv
                                            uvsPtr[baseIdx + 1] = uv
                                            uvsPtr[baseIdx + 2] = uv
                                            
                                            // 3. Colors (RGB -> RGBA)
                                            if let inputColors = inputColors {
                                                let r = inputColors[i * 3]
                                                let g = inputColors[i * 3 + 1]
                                                let b = inputColors[i * 3 + 2]
                                                
                                                let colorIdx = i * 4
                                                colorsPtr[colorIdx] = r
                                                colorsPtr[colorIdx + 1] = g
                                                colorsPtr[colorIdx + 2] = b
                                                // Alpha is already 255 from initialization
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            
            // 4. Create Mesh Descriptor
            var descriptor = MeshDescriptor(name: "pointCloudRaw")
            descriptor.positions = MeshBuffers.Positions(self.buffers.positions)
            descriptor.primitives = .triangles(self.buffers.indices)
            
            if hasColors {
                descriptor.textureCoordinates = MeshBuffers.TextureCoordinates(self.buffers.uvs)
            }
            
            // DOUBLE BUFFERING LOGIC
            let backEntity = (activeBufferIndex == 0) ? meshEntityB : meshEntityA
            let frontEntity = (activeBufferIndex == 0) ? meshEntityA : meshEntityB
            
            guard let targetEntity = backEntity else { return }
            
            // 5. Update Mesh
            if let existingMesh = targetEntity.model?.mesh {
                do {
                    let tempMesh = try MeshResource.generate(from: [descriptor])
                    try existingMesh.replace(with: tempMesh.contents)
                } catch {
                    print("⚠️ [PointCloud] Failed to replace mesh: \(error)")
                    if let meshResource = try? MeshResource.generate(from: [descriptor]) {
                        targetEntity.model?.mesh = meshResource
                    }
                }
            } else {
                if let meshResource = try? MeshResource.generate(from: [descriptor]) {
                    targetEntity.model?.mesh = meshResource
                }
            }
            
            // 6. Update Material/Texture
            var mat = UnlitMaterial()
            
            if hasColors {
                if let texture = createTextureFromBuffer(width: numPoints) {
                     mat.color = .init(texture: .init(texture))
                } else {
                     mat.color = .init(tint: .cyan)
                }
            } else {
                mat.color = .init(tint: .cyan)
            }
            
            targetEntity.model?.materials = [mat]
            
            // 7. Swap visibility
            targetEntity.isEnabled = true
            frontEntity?.isEnabled = false
            
            // 8. Toggle index
            activeBufferIndex = (activeBufferIndex == 0) ? 1 : 0
            
            // 9. Echo benchmark if needed (now that mesh is updated)
            if var echo = benchmarkEchoData {
                let endTime = Date()
                let processingTime = endTime.timeIntervalSince(startTime)
                
                // Append processing time (Double = 8 bytes)
                var procTime = processingTime
                let procTimeData = Data(bytes: &procTime, count: MemoryLayout<Double>.size)
                echo.append(procTimeData)
                
                self.webRTCClient?.sendPointCloudData(echo)
            }
            
            // Force position if needed
            if self.root?.position.z == 0 && self.root?.position.y == 0 {
                 self.root?.position = SIMD3<Float>(0, 1.5, -1.0) 
                 print("⚠️ [PointCloud] Forced root position to (0, 1.5, -1.0)")
            }
        }
    }

    
    private func createTextureFromBuffer(width: Int) -> TextureResource? {
        let height = 1
        let bytesPerRow = width * 4
        
        // Create Data provider from our reusable buffer
        // Note: We are creating a copy here because CGDataProvider takes ownership or needs a copy.
        // To avoid copy, we'd need to use a release callback, but keeping it simple for now.
        // Actually, Data(bytes:count:) copies.
        // Optimization: Use `Data(bytesNoCopy:...)` if we can guarantee buffer validity.
        // But `buffers.colors` might change next frame.
        // However, the texture creation consumes the data immediately to create the CGImage?
        // Let's stick to copy for safety for now, it's a linear copy of bytes, much faster than the loop we had.
        
        let data = Data(self.buffers.colors)
        guard let dataProvider = CGDataProvider(data: data as CFData) else { return nil }
        
        guard let cgImage = CGImage(
            width: width,
            height: height,
            bitsPerComponent: 8,
            bitsPerPixel: 32,
            bytesPerRow: bytesPerRow,
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedLast.rawValue),
            provider: dataProvider,
            decode: nil,
            shouldInterpolate: false,
            intent: .defaultIntent
        ) else { return nil }
        
        do {
            let texture = try TextureResource.generate(from: cgImage, options: .init(semantic: .color))
            return texture
        } catch {
            print("⚠️ [PointCloud] Failed to create texture: \(error)")
            return nil
        }
    }
}
