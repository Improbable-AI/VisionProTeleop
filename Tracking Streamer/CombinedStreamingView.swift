import SwiftUI
import RealityKit
import LiveKitWebRTC
import CoreImage
import Accelerate
import AVFoundation
import AVFAudio
import RealityKitContent
import GRPCCore
import GRPCNIOTransportHTTP2
import GRPCProtobuf
import simd

/// Combined Streaming View that supports:
/// - Video/Audio streaming via WebRTC (from ImmersiveView)
/// - MuJoCo simulation streaming via gRPC (from MuJoCoStreamingView)
/// - UVC camera streaming (USB cameras via Developer Strap)
struct CombinedStreamingView: View {
    @EnvironmentObject var imageData: ImageData
    @StateObject private var videoStreamManager = VideoStreamManager()
    @StateObject private var appModel = 🥽AppModel()
    @StateObject private var mujocoManager = CombinedMuJoCoManager()
    @StateObject private var uvcCameraManager = UVCCameraManager.shared
    @StateObject private var recordingManager = RecordingManager.shared
    @ObservedObject private var dataManager = DataManager.shared
    
    // Video state
    @State private var updateTrigger = false
    @State private var hasFrames = false
    @State private var hasAudio = false      // Audio is streaming
    @State private var hasSimPoses = false   // Sim poses are streaming
    @State private var isMinimized = false
    @State private var showViewControls = false
    @State private var previewZDistance: Float? = nil
    @State private var previewActive = false
    @State private var hasAutoMinimized = false
    @State private var userInteracted = false
    @State private var currentAspectRatio: Float? = nil
    @State private var videoMinimized = false
    @State private var previewStatusPosition: (x: Float, y: Float)? = nil
    @State private var previewStatusActive = false
    @State private var stereoMaterialEntity: Entity? = nil
    @State private var fixedWorldTransform: Transform? = nil
    @State private var uvcFrame: UIImage? = nil  // UVC camera frame
    @State private var currentVideoFrame: UIImage? = nil  // Current frame for recording
    
    // MuJoCo state
    @State private var mujocoEntity: Entity? = nil
    @State private var mujocoBodyEntities: [String: ModelEntity] = [:]
    @State private var initialLocalTransforms: [String: Transform] = [:]
    @State private var pythonToSwiftNameMap: [String: String] = [:]
    @State private var pythonToSwiftTargets: [String: [String]] = [:]  // 1:N mapping for leaf entities
    @State private var entityPathByObjectID: [ObjectIdentifier: String] = [:]
    @State private var nameMappingInitialized: Bool = false
    @State private var attachToPosition: SIMD3<Float>? = nil
    @State private var attachToRotation: simd_quatf? = nil
    @State private var mujocoFinalTransforms: [String: simd_float4x4] = [:]
    @State private var mujocoPoseUpdateTrigger: UUID = UUID()
    @State private var mujocoUsdzURL: String? = nil
    
    private let applyInWorldSpace: Bool = true
    
    var body: some View {
        RealityView { content, attachments in
            print("🟢 [CombinedStreamingView] RealityView content block called")
            
            // === VIDEO SETUP (from ImmersiveView) ===
            let videoAnchor = AnchorEntity(.head)
            videoAnchor.name = "videoAnchor"
            content.add(videoAnchor)
            
            let worldAnchor = AnchorEntity(world: .zero)
            worldAnchor.name = "videoWorldAnchor"
            content.add(worldAnchor)
            
            let videoRoot = Entity()
            videoRoot.name = "videoRoot"
            videoRoot.setParent(worldAnchor)
            
            let skyBox = createSkyBox()
            skyBox.isEnabled = false
            skyBox.name = "videoPlane"
            skyBox.setParent(videoRoot)
            
            let previewPlane = createPreviewPlane()
            previewPlane.name = "previewPlane"
            previewPlane.isEnabled = false
            previewPlane.setParent(videoRoot)
            
            // === MUJOCO SETUP ===
            let mujocoRoot = Entity()
            mujocoRoot.name = "mujocoRoot"
            content.add(mujocoRoot)
            
            // === STATUS DISPLAY ===
            let statusAnchor = AnchorEntity(.head)
            statusAnchor.name = "statusHeadAnchor"
            content.add(statusAnchor)
            
            let statusContainer = Entity()
            statusContainer.name = "statusContainer"
            statusContainer.setParent(statusAnchor)
            statusContainer.transform.translation = SIMD3<Float>(0.0, 0.0, -1.0)
            
            if let statusAttachment = attachments.entity(for: "status") {
                print("🟢 [CombinedStreamingView] Status attachment found and attached")
                statusAttachment.setParent(statusContainer)
            }
            
            let statusPreviewContainer = Entity()
            statusPreviewContainer.name = "statusPreviewContainer"
            statusPreviewContainer.setParent(statusAnchor)
            statusPreviewContainer.transform.translation = SIMD3<Float>(
                dataManager.statusMinimizedXPosition,
                dataManager.statusMinimizedYPosition,
                -1.0
            )
            
            if let statusPreviewAttachment = attachments.entity(for: "statusPreview") {
                statusPreviewAttachment.setParent(statusPreviewContainer)
                statusPreviewContainer.isEnabled = false
            }
        } update: { updateContent, attachments in
            let _ = updateTrigger
            let _ = dataManager.videoPlaneZDistance
            let _ = dataManager.videoPlaneYPosition
            let _ = dataManager.videoPlaneAutoPerpendicular
            let _ = dataManager.videoPlaneFixedToWorld
            let _ = previewZDistance
            let _ = currentAspectRatio
            let _ = dataManager.statusMinimizedXPosition
            let _ = dataManager.statusMinimizedYPosition
            let _ = previewStatusPosition
            let _ = previewStatusActive
            let _ = isMinimized
            let _ = mujocoPoseUpdateTrigger
            let _ = mujocoEntity  // Trigger update when model is loaded
            
            func findEntity(named name: String, in collection: RealityViewEntityCollection) -> Entity? {
                for entity in collection {
                    if entity.name == name { return entity }
                    if let nested = entity.findEntity(named: name) { return nested }
                }
                return nil
            }
            
            // === VIDEO UPDATE (from ImmersiveView) ===
            if let videoRoot = findEntity(named: "videoRoot", in: updateContent.entities) {
                let skyBoxEntity = videoRoot.findEntity(named: "videoPlane")
                let previewEntity = videoRoot.findEntity(named: "previewPlane")
                let headAnchor = findEntity(named: "videoAnchor", in: updateContent.entities) as? AnchorEntity
                let worldAnchor = findEntity(named: "videoWorldAnchor", in: updateContent.entities) as? AnchorEntity
                
                let framesAvailable = imageData.left != nil && imageData.right != nil
                let isFixed = dataManager.videoPlaneFixedToWorld
                let shouldShowPreview = previewZDistance != nil || previewActive
                let targetZ = previewZDistance ?? dataManager.videoPlaneZDistance
                let targetY = dataManager.videoPlaneYPosition
                
                if isFixed {
                    if let worldAnchor = worldAnchor {
                        if videoRoot.parent !== worldAnchor {
                            videoRoot.setParent(worldAnchor, preservingWorldTransform: false)
                        }
                        if let lockedTransform = fixedWorldTransform {
                            videoRoot.move(to: lockedTransform, relativeTo: worldAnchor, duration: 0.1, timingFunction: .linear)
                        }
                    }
                } else {
                    if let headAnchor = headAnchor {
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
                }
                
                if shouldShowPreview && !framesAvailable {
                    previewEntity?.isEnabled = true
                } else {
                    previewEntity?.isEnabled = false
                }
                
                // Determine video source and get appropriate frame
                let isUVCMode = dataManager.videoSource == .uvcCamera
                let networkFrameAvailable = imageData.left != nil && imageData.right != nil
                let uvcFrameAvailable = uvcFrame != nil && isUVCMode
                let hasVideoFrame = isUVCMode ? uvcFrameAvailable : networkFrameAvailable
                
                // Update video texture based on source
                if hasVideoFrame, let skyBox = skyBoxEntity {
                    let displayImage: UIImage
                    let imageWidth: CGFloat
                    let imageHeight: CGFloat
                    
                    if isUVCMode, let uvc = uvcFrame {
                        // Use UVC camera frame
                        displayImage = uvc
                        imageWidth = displayImage.size.width
                        imageHeight = displayImage.size.height
                    } else if let imageRight = imageData.right {
                        // Use network stream frame
                        displayImage = imageRight
                        imageWidth = imageRight.size.width
                        imageHeight = imageRight.size.height
                    } else {
                        skyBoxEntity?.isEnabled = false
                        return
                    }
                    
                    // Store current frame for recording
                    currentVideoFrame = displayImage
                    
                    // Record frame if recording is active (video-driven recording)
                    // Each new video frame captures the latest tracking data
                    if recordingManager.isRecording {
                        recordingManager.recordVideoFrame(displayImage)
                    }
                    
                    let aspectRatio = Float(imageWidth / imageHeight)
                    
                    if currentAspectRatio == nil || abs(currentAspectRatio! - aspectRatio) > 0.01 {
                        currentAspectRatio = aspectRatio
                        let planeHeight: Float = 9.6
                        let planeWidth = planeHeight * aspectRatio
                        let newMesh = MeshResource.generatePlane(width: planeWidth, height: planeHeight)
                        skyBox.components[ModelComponent.self]?.mesh = newMesh
                        previewEntity?.components[ModelComponent.self]?.mesh = newMesh
                    }
                    
                    skyBox.isEnabled = !videoMinimized
                    
                    if !hasFrames {
                        // Mark that we received the first frame
                        // Use Task to ensure state updates are properly committed
                        Task { @MainActor in
                            if !hasFrames {  // Double-check to avoid duplicate triggers
                                hasFrames = true
                                // Audio comes with the same WebRTC connection, so mark it as ready too
                                if dataManager.audioEnabled && !isUVCMode {
                                    hasAudio = true
                                }
                                print("🎬 [CombinedStreamingView] First video frame received (source: \(isUVCMode ? "UVC" : "Network")), hasFrames=\(hasFrames)")
                                // Small delay to ensure state is fully committed
                                try? await Task.sleep(nanoseconds: 100_000_000)  // 100ms
                                print("🎬 [CombinedStreamingView] Checking auto-minimize after delay")
                                tryAutoMinimize()
                            }
                        }
                    }
                    
                    // UVC camera is always mono, network can be stereo
                    let isStereo = !isUVCMode && DataManager.shared.stereoEnabled
                    
                    if isStereo, let imageLeft = imageData.left, let imageRight = imageData.right {
                        do {
                            guard let sphereEntity = stereoMaterialEntity,
                                  var stereoMaterial = sphereEntity.components[ModelComponent.self]?.materials.first as? ShaderGraphMaterial else {
                                var skyBoxMaterial = UnlitMaterial()
                                var textureOptions = TextureResource.CreateOptions(semantic: .hdrColor)
                                textureOptions.mipmapsMode = .none
                                let texture = try TextureResource.generate(from: imageRight.cgImage!, options: textureOptions)
                                skyBoxMaterial.color = .init(texture: .init(texture))
                                skyBox.components[ModelComponent.self]?.materials = [skyBoxMaterial]
                                return
                            }
                            
                            var textureOptions = TextureResource.CreateOptions(semantic: .hdrColor)
                            textureOptions.mipmapsMode = .none
                            let leftTexture = try TextureResource.generate(from: imageLeft.cgImage!, options: textureOptions)
                            let rightTexture = try TextureResource.generate(from: imageRight.cgImage!, options: textureOptions)
                            try stereoMaterial.setParameter(name: "left", value: .textureResource(leftTexture))
                            try stereoMaterial.setParameter(name: "right", value: .textureResource(rightTexture))
                            skyBox.components[ModelComponent.self]?.materials = [stereoMaterial]
                        } catch {
                            print("❌ ERROR: Failed to load stereo textures: \(error)")
                        }
                    } else {
                        // Mono mode (either UVC or network mono)
                        var skyBoxMaterial = UnlitMaterial()
                        do {
                            var textureOptions = TextureResource.CreateOptions(semantic: .hdrColor)
                            textureOptions.mipmapsMode = .none
                            let texture = try TextureResource.generate(from: displayImage.cgImage!, options: textureOptions)
                            skyBoxMaterial.color = .init(texture: .init(texture))
                            skyBox.components[ModelComponent.self]?.materials = [skyBoxMaterial]
                        } catch {
                            print("❌ ERROR: Failed to load mono texture: \(error)")
                        }
                    }
                } else {
                    skyBoxEntity?.isEnabled = false
                    // Only reset hasFrames if we were showing frames before
                    if hasFrames && !isUVCMode {
                        hasFrames = false
                    }
                }
            }
            
            // === STATUS UPDATE ===
            if let statusAnchor = updateContent.entities.first(where: { $0.name == "statusHeadAnchor" }) as? AnchorEntity {
                if let statusContainer = statusAnchor.children.first(where: { $0.name == "statusContainer" }) {
                    let targetTranslation: SIMD3<Float>
                    if isMinimized {
                        targetTranslation = SIMD3<Float>(
                            dataManager.statusMinimizedXPosition,
                            dataManager.statusMinimizedYPosition,
                            -1.0
                        )
                    } else {
                        targetTranslation = SIMD3<Float>(0.0, 0.0, -1.0)
                    }
                    var transform = statusContainer.transform
                    transform.translation = targetTranslation
                    statusContainer.move(to: transform, relativeTo: statusContainer.parent, duration: 0.5, timingFunction: .easeInOut)
                }
                
                if let statusPreviewContainer = statusAnchor.children.first(where: { $0.name == "statusPreviewContainer" }) {
                    let shouldShowPreview = previewStatusPosition != nil || previewStatusActive
                    if shouldShowPreview {
                        let xPos = previewStatusPosition?.x ?? dataManager.statusMinimizedXPosition
                        let yPos = previewStatusPosition?.y ?? dataManager.statusMinimizedYPosition
                        statusPreviewContainer.isEnabled = true
                        var previewTransform = statusPreviewContainer.transform
                        previewTransform.translation = SIMD3<Float>(xPos, yPos, -1.0)
                        statusPreviewContainer.move(to: previewTransform, relativeTo: statusPreviewContainer.parent, duration: 0.1, timingFunction: .linear)
                    } else {
                        statusPreviewContainer.isEnabled = false
                    }
                }
            }
            
            // === MUJOCO MODEL UPDATE ===
            // Add loaded MuJoCo model to the scene if not already added
            if let mujocoRoot = findEntity(named: "mujocoRoot", in: updateContent.entities),
               let entity = mujocoEntity {
                if entity.parent == nil {
                    print("🔗 [CombinedStreamingView] Adding MuJoCo model to scene")
                    mujocoRoot.addChild(entity)
                }
            }
            
            // === MUJOCO POSE UPDATE ===
            if !mujocoFinalTransforms.isEmpty {
                applyMuJoCoTransforms(mujocoFinalTransforms)
            }
            
        } attachments: {
            Attachment(id: "status") {
                StatusOverlay(
                    hasFrames: $hasFrames,
                    showVideoStatus: true,
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
                    previewStatusActive: $previewStatusActive,
                    mujocoManager: mujocoManager
                )
            }
            
            Attachment(id: "statusPreview") {
                StatusPreviewView(
                    showVideoStatus: true,
                    videoFixed: dataManager.videoPlaneFixedToWorld
                )
            }
        }
        .onReceive(imageData.$left) { _ in
            // Only trigger update for network source
            if dataManager.videoSource == .network {
                updateTrigger.toggle()
            }
        }
        .onReceive(uvcCameraManager.$currentFrame) { frame in
            // Update UVC frame and trigger view update
            if dataManager.videoSource == .uvcCamera {
                uvcFrame = frame
                updateTrigger.toggle()
            }
        }
        .onChange(of: dataManager.videoSource) { oldValue, newValue in
            print("📹 [CombinedStreamingView] Video source changed from \(oldValue.rawValue) to \(newValue.rawValue)")
            
            // Reset frame state when switching sources
            hasFrames = false
            currentAspectRatio = nil
            
            if newValue == .uvcCamera {
                // Start UVC capture if a device is selected
                if uvcCameraManager.selectedDevice != nil {
                    print("📹 [CombinedStreamingView] Starting UVC capture with selected device: \(uvcCameraManager.selectedDevice?.name ?? "unknown")")
                    uvcCameraManager.startCapture()
                } else if let firstDevice = uvcCameraManager.availableDevices.first {
                    print("📹 [CombinedStreamingView] Selecting first available device: \(firstDevice.name)")
                    uvcCameraManager.selectDevice(firstDevice)
                    // Wait a moment for device to be configured, then start capture
                    Task {
                        try? await Task.sleep(nanoseconds: 200_000_000) // 200ms
                        await MainActor.run {
                            uvcCameraManager.startCapture()
                        }
                    }
                } else {
                    print("📹 [CombinedStreamingView] No UVC devices available")
                }
            } else {
                // Stop UVC capture when switching to network
                uvcCameraManager.stopCapture()
                uvcFrame = nil
            }
            
            updateTrigger.toggle()
        }
        // Auto-start UVC capture when a device becomes available while in UVC mode
        .onChange(of: uvcCameraManager.availableDevices) { oldDevices, newDevices in
            if dataManager.videoSource == .uvcCamera && !newDevices.isEmpty && oldDevices.isEmpty {
                print("📹 [CombinedStreamingView] UVC device connected while in UVC mode, auto-starting capture")
                if let firstDevice = newDevices.first {
                    uvcCameraManager.selectDevice(firstDevice)
                    Task {
                        try? await Task.sleep(nanoseconds: 200_000_000) // 200ms
                        await MainActor.run {
                            uvcCameraManager.startCapture()
                        }
                    }
                }
            }
        }
        // Auto-start capture when device selection changes
        .onChange(of: uvcCameraManager.selectedDevice) { oldDevice, newDevice in
            if dataManager.videoSource == .uvcCamera && newDevice != nil && !uvcCameraManager.isCapturing {
                print("📹 [CombinedStreamingView] UVC device selected, starting capture")
                Task {
                    try? await Task.sleep(nanoseconds: 100_000_000) // 100ms
                    await MainActor.run {
                        uvcCameraManager.startCapture()
                    }
                }
            }
        }
        .task { appModel.run() }
        .task { await appModel.processDeviceAnchorUpdates() }
        .task(priority: .low) { await appModel.processReconstructionUpdates() }
        .onAppear {
            print("🚀 [CombinedStreamingView] View appeared, starting services")
            
            // Reset auto-minimize state on view appear (fresh start)
            hasAutoMinimized = false
            userInteracted = false
            hasFrames = false
            hasAudio = false
            hasSimPoses = false
            print("🔄 [CombinedStreamingView] Reset auto-minimize state: hasAutoMinimized=\(hasAutoMinimized), userInteracted=\(userInteracted)")
            
            // Setup WebRTC sim-poses callback BEFORE starting video stream
            videoStreamManager.onSimPosesReceived = { [self] poses in
                // Convert WebRTC JSON format [x,y,z,qx,qy,qz,qw] to MuJoCo poses
                Task { @MainActor in
                    let transforms = self.computeMuJoCoFinalTransformsFromWebRTC(poses)
                    self.mujocoFinalTransforms = transforms
                    self.mujocoPoseUpdateTrigger = UUID()
                    
                    // Update stats
                    mujocoManager.recordPoseUpdate(bodyCount: poses.count)
                    
                    // Update status if we're receiving poses via WebRTC
                    if !mujocoManager.poseStreamingViaWebRTC {
                        mujocoManager.poseStreamingViaWebRTC = true
                        mujocoManager.simEnabled = true  // Mark simulation as enabled
                        mujocoManager.updateConnectionStatus("Streaming via WebRTC")
                    }
                    
                    // Track sim poses for auto-minimize
                    if !hasSimPoses {
                        hasSimPoses = true
                        tryAutoMinimize()
                    }
                }
            }
            
            videoStreamManager.start(imageData: imageData)
            
            // Auto-start MuJoCo gRPC server for USDZ transfer
            Task {
                await mujocoManager.startServer()
            }
            
            // Initialize UVC camera if it's the selected source
            if dataManager.videoSource == .uvcCamera {
                print("📹 [CombinedStreamingView] UVC mode active on appear, initializing camera")
                Task {
                    let granted = await uvcCameraManager.requestCameraAccess()
                    if granted {
                        await MainActor.run {
                            if uvcCameraManager.selectedDevice != nil {
                                print("📹 [CombinedStreamingView] Starting capture with existing device")
                                uvcCameraManager.startCapture()
                            } else if let firstDevice = uvcCameraManager.availableDevices.first {
                                print("📹 [CombinedStreamingView] Selecting and starting first device: \(firstDevice.name)")
                                uvcCameraManager.selectDevice(firstDevice)
                            }
                        }
                        // Wait for device setup then start capture
                        try? await Task.sleep(nanoseconds: 300_000_000) // 300ms
                        await MainActor.run {
                            if !uvcCameraManager.isCapturing && uvcCameraManager.selectedDevice != nil {
                                uvcCameraManager.startCapture()
                            }
                        }
                    }
                }
            }
            
            // Load stereo material
            Task {
                if let scene = try? await Entity(named: "Immersive", in: realityKitContentBundle) {
                    if let sphereEntity = scene.findEntity(named: "Sphere") {
                        sphereEntity.isEnabled = false
                        await MainActor.run {
                            self.stereoMaterialEntity = sphereEntity
                        }
                        print("✅ [CombinedStreamingView] Loaded stereo material")
                    }
                }
            }
            
            // Setup MuJoCo callbacks (for gRPC USDZ transfer, backup poses)
            mujocoManager.onUsdzReceived = { url, position, rotation in
                Task { @MainActor in
                    self.attachToPosition = position
                    self.attachToRotation = rotation
                    self.mujocoUsdzURL = url
                    mujocoManager.simEnabled = true  // Mark simulation as enabled when USDZ is received
                }
            }
            
            mujocoManager.onPosesReceived = { poses in
                // Only use gRPC poses if WebRTC is not streaming
                if !mujocoManager.poseStreamingViaWebRTC {
                    Task { @MainActor in
                        let transforms = self.computeMuJoCoFinalTransforms(poses)
                        self.mujocoFinalTransforms = transforms
                        self.mujocoPoseUpdateTrigger = UUID()
                    }
                }
            }
        }
        .onChange(of: mujocoUsdzURL) { _, newValue in
            if let urlStr = newValue, let url = URL(string: urlStr) {
                Task {
                    await loadMuJoCoModel(from: url)
                }
            }
        }
        .onChange(of: mujocoPoseUpdateTrigger) { _, _ in
            Task {
                try? await Task.sleep(nanoseconds: 1_000_000)
                mujocoFinalTransforms = [:]
            }
        }
        .onChange(of: dataManager.pythonClientIP) { oldValue, newValue in
            if newValue == nil {
                print("🔌 [CombinedStreamingView] Python client disconnected")
                
                // Stop auto-recording when Python client disconnects
                recordingManager.onVideoSourceDisconnected(reason: "Python client disconnected")
                
                imageData.left = nil
                imageData.right = nil
                hasFrames = false
                hasAudio = false
                hasSimPoses = false
                videoMinimized = false
                hasAutoMinimized = false
                fixedWorldTransform = nil
                mujocoManager.poseStreamingViaWebRTC = false
                mujocoManager.simEnabled = false
                videoStreamManager.stop()
                
                // Remove MuJoCo assets
                mujocoEntity?.removeFromParent()
                mujocoEntity = nil
                mujocoBodyEntities.removeAll()
                initialLocalTransforms.removeAll()
                pythonToSwiftNameMap.removeAll()
                pythonToSwiftTargets.removeAll()
                entityPathByObjectID.removeAll()
                nameMappingInitialized = false
                mujocoFinalTransforms.removeAll()
                mujocoUsdzURL = nil
                attachToPosition = nil
                attachToRotation = nil
            }
        }
        .onChange(of: dataManager.webrtcGeneration) { oldValue, newValue in
            if newValue < 0 {
                // Stop auto-recording when WebRTC disconnects
                recordingManager.onVideoSourceDisconnected(reason: "WebRTC disconnected")
                
                imageData.left = nil
                imageData.right = nil
                hasFrames = false
                hasAudio = false
                hasSimPoses = false
                videoMinimized = false
                fixedWorldTransform = nil
                mujocoManager.poseStreamingViaWebRTC = false
                mujocoManager.simEnabled = false
                videoStreamManager.stop()
                
                // Remove MuJoCo assets on WebRTC disconnect
                mujocoEntity?.removeFromParent()
                mujocoEntity = nil
                mujocoBodyEntities.removeAll()
                initialLocalTransforms.removeAll()
                pythonToSwiftNameMap.removeAll()
                pythonToSwiftTargets.removeAll()
                entityPathByObjectID.removeAll()
                nameMappingInitialized = false
                mujocoFinalTransforms.removeAll()
                mujocoUsdzURL = nil
                attachToPosition = nil
                attachToRotation = nil
            } else if newValue > 0 && oldValue != newValue {
                mujocoManager.poseStreamingViaWebRTC = false
                hasSimPoses = false  // Reset sim poses on new connection
                videoStreamManager.stop()
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                    videoStreamManager.start(imageData: imageData)
                }
            }
        }
        .onChange(of: hasFrames) { oldValue, newValue in
            if newValue && !oldValue {
                print("🎬 [CombinedStreamingView] Video frames arrived, attempting auto-minimize")
                tryAutoMinimize()
            }
        }
        .onChange(of: dataManager.videoEnabled) { _, newValue in
            print("🔧 [CombinedStreamingView] videoEnabled changed to \(newValue)")
            // Re-check auto-minimize in case frames arrived before config
            if newValue {
                tryAutoMinimize()
            }
        }
        .onChange(of: dataManager.audioEnabled) { _, newValue in
            print("🔧 [CombinedStreamingView] audioEnabled changed to \(newValue)")
            // If we have video frames and audio is now enabled, mark audio as ready
            if newValue && hasFrames {
                hasAudio = true
                tryAutoMinimize()
            }
        }
        .onChange(of: dataManager.simEnabled) { _, newValue in
            print("🔧 [CombinedStreamingView] simEnabled changed to \(newValue)")
            // Re-check auto-minimize in case sim poses arrived before config
            if newValue {
                tryAutoMinimize()
            }
        }
        .onChange(of: dataManager.videoPlaneFixedToWorld) { oldValue, isFixed in
            if isFixed {
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
                
                let worldMatrix = simd_mul(headWorldMatrix, offsetTransform.matrix)
                fixedWorldTransform = Transform(matrix: worldMatrix)
            } else {
                fixedWorldTransform = nil
            }
        }
        .onChange(of: userInteracted) { oldValue, newValue in
            print("⚠️ [CombinedStreamingView] userInteracted changed from \(oldValue) to \(newValue)")
            // Print stack trace to see who changed it
            if newValue {
                Thread.callStackSymbols.prefix(10).forEach { print("  \($0)") }
            }
        }
        .onDisappear {
            print("🛑 [CombinedStreamingView] View disappeared, stopping services")
            videoStreamManager.stop()
            uvcCameraManager.stopCapture()
            fixedWorldTransform = nil
            Task {
                await mujocoManager.stopServer()
            }
        }
    }
    
    // MARK: - Auto-Minimize Logic
    
    /// Check if all required streams are ready and auto-minimize if conditions are met
    private func tryAutoMinimize() {
        print("🔍 [AutoMinimize] Checking... hasAutoMinimized=\(hasAutoMinimized), userInteracted=\(userInteracted)")
        guard !hasAutoMinimized && !userInteracted else { 
            print("🔍 [AutoMinimize] Early exit: already minimized or user interacted")
            return 
        }
        
        let dm = DataManager.shared
        
        // Determine what's required based on what Python configured
        let videoRequired = dm.videoEnabled
        let audioRequired = dm.audioEnabled
        let simRequired = dm.simEnabled
        
        print("🔍 [AutoMinimize] Config: video=\(videoRequired), audio=\(audioRequired), sim=\(simRequired)")
        print("🔍 [AutoMinimize] State: hasFrames=\(hasFrames), hasAudio=\(hasAudio), hasSimPoses=\(hasSimPoses)")
        
        // Check if all required streams are ready
        let videoReady = !videoRequired || hasFrames
        let audioReady = !audioRequired || hasAudio
        let simReady = !simRequired || hasSimPoses
        
        // Special case: if audio is required but video is also enabled, audio comes with video
        let audioEffectivelyReady = !audioRequired || (audioRequired && videoRequired && hasFrames) || hasAudio
        
        let allReady = videoReady && audioEffectivelyReady && simReady
        
        // Also need at least one thing to be configured and ready
        let somethingConfigured = videoRequired || audioRequired || simRequired
        let somethingReady = hasFrames || hasAudio || hasSimPoses
        
        print("🔍 [AutoMinimize] Ready: video=\(videoReady), audio=\(audioEffectivelyReady), sim=\(simReady), all=\(allReady)")
        print("🔍 [AutoMinimize] somethingConfigured=\(somethingConfigured), somethingReady=\(somethingReady)")
        
        if allReady && somethingConfigured && somethingReady {
            print("✅ [AutoMinimize] All required streams ready (video=\(videoRequired)/\(hasFrames), audio=\(audioRequired)/\(hasAudio), sim=\(simRequired)/\(hasSimPoses))")
            Task { @MainActor in
                try? await Task.sleep(nanoseconds: 1_000_000_000)  // 1 second delay
                print("✅ [AutoMinimize] After delay: hasAutoMinimized=\(hasAutoMinimized), userInteracted=\(userInteracted)")
                if !hasAutoMinimized && !userInteracted {
                    print("✅ [AutoMinimize] Minimizing status view now!")
                    withAnimation(.spring(response: 0.6, dampingFraction: 0.8)) {
                        isMinimized = true
                        hasAutoMinimized = true
                    }
                }
            }
        } else {
            print("⏳ [AutoMinimize] Waiting for streams (video=\(videoRequired)/\(hasFrames), audio=\(audioRequired)/\(hasAudio), sim=\(simRequired)/\(hasSimPoses))")
        }
    }
    
    // MARK: - MuJoCo Model Loading
    
    private func loadMuJoCoModel(from url: URL) async {
        do {
            print("📦 [MuJoCo] Loading USDZ from \(url.absoluteString)")
            
            // Clear previous state
            mujocoEntity = nil
            mujocoBodyEntities.removeAll()
            initialLocalTransforms.removeAll()
            pythonToSwiftNameMap.removeAll()
            nameMappingInitialized = false
            
            let loadedEntity = try await Entity(contentsOf: url)
            
            let newEntity: ModelEntity
            if let model = loadedEntity as? ModelEntity {
                newEntity = model
            } else {
                let wrapper = ModelEntity()
                wrapper.addChild(loadedEntity)
                newEntity = wrapper
            }
            
            await MainActor.run {
                mujocoEntity = newEntity
            }
            
            indexMuJoCoBodyEntities(newEntity)
            print("✅ [MuJoCo] Model loaded with \(mujocoBodyEntities.count) bodies")
            
        } catch {
            print("❌ [MuJoCo] Failed to load USDZ: \(error)")
        }
    }
    
    private func indexMuJoCoBodyEntities(_ rootEntity: Entity) {
        mujocoBodyEntities.removeAll()
        initialLocalTransforms.removeAll()
        entityPathByObjectID.removeAll()
        pythonToSwiftTargets.removeAll()
        
        print("🔍 [indexMuJoCoBodyEntities] Starting recursive indexing...")
        
        func indexRec(_ entity: Entity, parentPath: String) {
            if entity.name.isEmpty {
                for child in entity.children {
                    indexRec(child, parentPath: parentPath)
                }
                return
            }
            
            if let modelEntity = entity as? ModelEntity {
                let pathKey = parentPath.isEmpty ? modelEntity.name : "\(parentPath)/\(modelEntity.name)"
                mujocoBodyEntities[pathKey] = modelEntity
                initialLocalTransforms[pathKey] = modelEntity.transform
                entityPathByObjectID[ObjectIdentifier(modelEntity)] = pathKey
                
                print("📎 Indexed ModelEntity: '\(pathKey)'")
                
                for child in modelEntity.children {
                    indexRec(child, parentPath: pathKey)
                }
            } else {
                let wrapper = ModelEntity()
                wrapper.name = entity.name
                wrapper.transform = entity.transform
                let preservedScale = entity.scale
                let originalChildren = Array(entity.children)
                
                if let parent = entity.parent {
                    parent.addChild(wrapper)
                    entity.removeFromParent()
                    wrapper.addChild(entity)
                } else {
                    wrapper.scale = preservedScale
                }
                
                let pathKey = parentPath.isEmpty ? wrapper.name : "\(parentPath)/\(wrapper.name)"
                mujocoBodyEntities[pathKey] = wrapper
                initialLocalTransforms[pathKey] = wrapper.transform
                entityPathByObjectID[ObjectIdentifier(wrapper)] = pathKey
                
                print("📎 Indexed Wrapper: '\(pathKey)'")
                
                for child in originalChildren {
                    indexRec(child, parentPath: pathKey)
                }
            }
        }
        
        indexRec(rootEntity, parentPath: "")
        print("📝 Indexed \(mujocoBodyEntities.count) entities")
    }
    
    // MARK: - MuJoCo Pose Transform
    
    private func computeMuJoCoFinalTransforms(_ poses: [String: MujocoAr_BodyPose]) -> [String: simd_float4x4] {
        guard !mujocoBodyEntities.isEmpty, !initialLocalTransforms.isEmpty else {
            return [:]
        }
        
        if !nameMappingInitialized {
            initializeMuJoCoNameMapping(pythonNames: Array(poses.keys))
            let validMatches = poses.keys.reduce(0) { acc, py in
                if let swift = pythonToSwiftNameMap[py], mujocoBodyEntities[swift] != nil { return acc + 1 }
                return acc
            }
            if validMatches > 0 { nameMappingInitialized = true }
        }
        
        let axisCorrection = simd_quatf(angle: -.pi / 2, axis: SIMD3<Float>(1, 0, 0))
        var finalTransforms: [String: simd_float4x4] = [:]
        
        for (pyName, pose) in poses {
            let bodyName = pythonToSwiftNameMap[pyName] ?? pyName
            
            guard initialLocalTransforms[bodyName] != nil else { continue }
            
            let mjPos = SIMD3<Float>(pose.position.x, pose.position.y, pose.position.z)
            let mjRot = simd_quatf(ix: pose.rotation.x, iy: pose.rotation.y, iz: pose.rotation.z, r: pose.rotation.w)
            
            var mjWorldTransform = matrix_identity_float4x4
            mjWorldTransform = simd_mul(matrix_float4x4(mjRot), mjWorldTransform)
            mjWorldTransform.columns.3 = SIMD4<Float>(mjPos, 1.0)
            
            if let attachPos = attachToPosition, let attachRot = attachToRotation {
                var attachTransform = matrix_identity_float4x4
                attachTransform = simd_mul(matrix_float4x4(attachRot), attachTransform)
                attachTransform.columns.3 = SIMD4<Float>(attachPos, 1.0)
                mjWorldTransform = simd_mul(attachTransform, mjWorldTransform)
            }
            
            let rkPos = axisCorrection.act(SIMD3<Float>(mjWorldTransform.columns.3.x, mjWorldTransform.columns.3.y, mjWorldTransform.columns.3.z))
            let mjRotFromMatrix = simd_quatf(mjWorldTransform)
            let rkRot = axisCorrection * mjRotFromMatrix
            
            var rkWorldTransform = matrix_identity_float4x4
            rkWorldTransform = simd_mul(matrix_float4x4(rkRot), rkWorldTransform)
            rkWorldTransform.columns.3 = SIMD4<Float>(rkPos, 1.0)
            
            let yLocal = initialLocalTransforms[bodyName]?.matrix ?? matrix_identity_float4x4
            let finalTransform = rkWorldTransform * yLocal
            
            finalTransforms[bodyName] = finalTransform
        }
        
        return finalTransforms
    }
    
    /// Find leaf target entities for a Python body name
    /// USDZ importer creates leaf ModelEntities under parent nodes - we need to move those
    private func targets(for pyName: String) -> [String] {
        // Return cached if available
        if let cached = pythonToSwiftTargets[pyName], !cached.isEmpty {
            return cached
        }
        
        // Find base entity from 1:1 mapping
        var baseName: String? = pythonToSwiftNameMap[pyName]
        if baseName == nil || baseName!.isEmpty || mujocoBodyEntities[baseName!] == nil {
            if let best = findBestSwiftMatch(for: pyName, sanitizedSwiftPairs: Array(mujocoBodyEntities.keys).map { ($0, sanitizeName($0)) }, excluding: []) {
                pythonToSwiftNameMap[pyName] = best
                baseName = best
            }
        }
        
        guard let base = baseName, let baseEntity = mujocoBodyEntities[base] else {
            return []
        }
        
        // Find leaf ModelEntities under this base
        var result: [String] = []
        for child in baseEntity.children {
            if let model = child as? ModelEntity {
                if model.children.isEmpty, let key = entityPathByObjectID[ObjectIdentifier(model)] {
                    result.append(key)
                }
            } else {
                // Common pattern: importer creates a Node with same name; look one level deeper
                if child.name == baseEntity.name {
                    for grand in child.children {
                        if let leaf = grand as? ModelEntity, leaf.children.isEmpty,
                           let key = entityPathByObjectID[ObjectIdentifier(leaf)] {
                            result.append(key)
                        }
                    }
                }
            }
        }
        
        // If no leaf targets found, use the base entity itself
        if result.isEmpty {
            result = [base]
        }
        
        pythonToSwiftTargets[pyName] = result
        if result.count > 1 || result.first != base {
            print("🎯 [targets] '\(pyName)' → [\(result.joined(separator: ", "))]")
        }
        return result
    }
    
    /// Compute final transform for a specific target entity
    private func computeFinalTransformForTarget(bodyName: String, values: [Float], axisCorrection: simd_quatf) -> simd_float4x4 {
        // Parse [x, y, z, qx, qy, qz, qw]
        let mjPos = SIMD3<Float>(values[0], values[1], values[2])
        let mjRot = simd_quatf(ix: values[3], iy: values[4], iz: values[5], r: values[6])
        
        // Build MuJoCo world-space transform (ZUP)
        var mjWorldTransform = matrix_identity_float4x4
        mjWorldTransform = simd_mul(matrix_float4x4(mjRot), mjWorldTransform)
        mjWorldTransform.columns.3 = SIMD4<Float>(mjPos, 1.0)
        
        // Apply attach_to offset BEFORE axis correction (in ZUP space)
        if let attachPos = attachToPosition, let attachRot = attachToRotation {
            var attachTransform = matrix_identity_float4x4
            attachTransform = simd_mul(matrix_float4x4(attachRot), attachTransform)
            attachTransform.columns.3 = SIMD4<Float>(attachPos, 1.0)
            mjWorldTransform = simd_mul(attachTransform, mjWorldTransform)
        }
        
        // Convert ZUP → YUP
        let rkPos = axisCorrection.act(SIMD3<Float>(mjWorldTransform.columns.3.x, mjWorldTransform.columns.3.y, mjWorldTransform.columns.3.z))
        let mjRotFromMatrix = simd_quatf(mjWorldTransform)
        let rkRot = axisCorrection * mjRotFromMatrix
        
        // Build RealityKit world transform
        var rkWorldTransform = matrix_identity_float4x4
        rkWorldTransform = simd_mul(matrix_float4x4(rkRot), rkWorldTransform)
        rkWorldTransform.columns.3 = SIMD4<Float>(rkPos, 1.0)
        
        // Apply initial local transform from USDZ import
        let yLocal = initialLocalTransforms[bodyName]?.matrix ?? matrix_identity_float4x4
        return rkWorldTransform * yLocal
    }
    
    /// Compute final transforms from WebRTC JSON format
    /// Input format: {"body_name": [x, y, z, qx, qy, qz, qw], ...}
    private func computeMuJoCoFinalTransformsFromWebRTC(_ poses: [String: [Float]]) -> [String: simd_float4x4] {
        guard !mujocoBodyEntities.isEmpty, !initialLocalTransforms.isEmpty else {
            return [:]
        }
        
        if !nameMappingInitialized {
            print("🔤 [WebRTC poses] Initializing name mapping...")
            print("   Python names: \(Array(poses.keys).sorted())")
            print("   Swift names: \(Array(mujocoBodyEntities.keys).sorted())")
            initializeMuJoCoNameMapping(pythonNames: Array(poses.keys))
            let validMatches = poses.keys.reduce(0) { acc, py in
                if let swift = pythonToSwiftNameMap[py], mujocoBodyEntities[swift] != nil { return acc + 1 }
                return acc
            }
            if validMatches > 0 { 
                nameMappingInitialized = true 
                print("✅ Name mapping initialized with \(validMatches) valid matches")
            }
        }
        
        let axisCorrection = simd_quatf(angle: -.pi / 2, axis: SIMD3<Float>(1, 0, 0))
        var finalTransforms: [String: simd_float4x4] = [:]
        
        for (pyName, values) in poses {
            guard values.count >= 7 else { continue }
            
            // Get all target entities for this Python body
            let targetNames = targets(for: pyName)
            
            for bodyName in targetNames {
                guard initialLocalTransforms[bodyName] != nil else { continue }
                
                let transform = computeFinalTransformForTarget(bodyName: bodyName, values: values, axisCorrection: axisCorrection)
                finalTransforms[bodyName] = transform
            }
        }
        
        return finalTransforms
    }
    
    private func applyMuJoCoTransforms(_ transforms: [String: simd_float4x4]) {
        var depthCache: [String: Int] = [:]
        
        func depth(for name: String) -> Int {
            if let d = depthCache[name] { return d }
            guard let entity = mujocoBodyEntities[name] else { depthCache[name] = 0; return 0 }
            var d = 0
            var p = entity.parent
            while let parent = p {
                d += 1
                p = parent.parent
            }
            depthCache[name] = d
            return d
        }
        
        let sortedNames = transforms.keys.sorted { lhs, rhs in depth(for: lhs) < depth(for: rhs) }
        
        for name in sortedNames {
            guard let entity = mujocoBodyEntities[name], let desiredWorld = transforms[name] else { continue }
            
            if applyInWorldSpace {
                let prevLocalScale = entity.scale
                entity.setTransformMatrix(desiredWorld, relativeTo: nil)
                if let parent = entity.parent {
                    entity.setScale(prevLocalScale, relativeTo: parent)
                } else {
                    entity.setScale(prevLocalScale, relativeTo: nil)
                }
            }
        }
    }
    
    private func initializeMuJoCoNameMapping(pythonNames: [String]) {
        let swiftNames = Array(mujocoBodyEntities.keys)
        let sanitizedSwiftPairs: [(original: String, sanitized: String)] = swiftNames.map { ($0, sanitizeName($0)) }
        var usedSwift: Set<String> = []
        
        var newMap: [String: String] = [:]
        for py in pythonNames {
            if let match = findBestSwiftMatch(for: py, sanitizedSwiftPairs: sanitizedSwiftPairs, excluding: usedSwift) {
                newMap[py] = match
                usedSwift.insert(match)
            } else {
                newMap[py] = py
            }
        }
        
        pythonToSwiftNameMap = newMap
    }
    
    private func sanitizeName(_ s: String) -> String {
        let lowered = s.lowercased()
        let filtered = lowered.unicodeScalars.filter { CharacterSet.alphanumerics.contains($0) }
        return String(String.UnicodeScalarView(filtered))
    }
    
    private func findBestSwiftMatch(for pyName: String,
                                    sanitizedSwiftPairs: [(original: String, sanitized: String)],
                                    excluding used: Set<String>) -> String? {
        let pySan = sanitizeName(pyName)
        var best: (swiftOriginal: String, delta: Int)? = nil
        for (orig, san) in sanitizedSwiftPairs where !used.contains(orig) {
            if san.contains(pySan) {
                let delta = max(0, san.count - pySan.count)
                if let current = best {
                    if delta < current.delta || (delta == current.delta && orig.count < current.swiftOriginal.count) {
                        best = (orig, delta)
                    }
                } else {
                    best = (orig, delta)
                }
            }
        }
        return best?.swiftOriginal
    }
}

// MARK: - Helper Functions

private func createSkyBox() -> Entity {
    let skyBoxEntity = Entity()
    let defaultHeight: Float = 9.6
    let defaultWidth: Float = defaultHeight * (16.0 / 9.0)
    let largePlane = MeshResource.generatePlane(width: defaultWidth, height: defaultHeight)
    var skyBoxMaterial = UnlitMaterial()
    skyBoxMaterial.color = .init(tint: .clear)
    skyBoxEntity.components.set(ModelComponent(mesh: largePlane, materials: [skyBoxMaterial]))
    return skyBoxEntity
}

private func createPreviewPlane() -> Entity {
    let previewEntity = Entity()
    let defaultHeight: Float = 9.6
    let defaultWidth: Float = defaultHeight * (16.0 / 9.0)
    let largePlane = MeshResource.generatePlane(width: defaultWidth, height: defaultHeight)
    var previewMaterial = UnlitMaterial()
    previewMaterial.color = .init(tint: .init(white: 0.5, alpha: 0.6))
    previewEntity.components.set(ModelComponent(mesh: largePlane, materials: [previewMaterial]))
    return previewEntity
}

// MARK: - Combined MuJoCo Manager

@MainActor
final class CombinedMuJoCoManager: ObservableObject, MuJoCoManager {
    @Published var ipAddress: String = "Getting IP..."
    @Published var connectionStatus: String = "Server Stopped"
    @Published var grpcPort: Int = 50051
    @Published var isServerRunning: Bool = false
    @Published var simEnabled: Bool = false  // True if simulation data has been received (USDZ loaded or poses streaming)
    @Published var poseStreamingViaWebRTC: Bool = false  // Tracks if poses are coming via WebRTC
    @Published var bodyCount: Int = 0
    @Published var updateFrequency: Double = 0.0  // Hz
    
    // For calculating update frequency
    private var lastUpdateTime: Date?
    private var updateTimes: [TimeInterval] = []
    private let maxUpdateSamples = 30  // Average over last 30 updates
    
    var onUsdzReceived: ((String, SIMD3<Float>?, simd_quatf?) -> Void)?
    var onPosesReceived: (([String: MujocoAr_BodyPose]) -> Void)?
    
    /// Alternative callback for WebRTC pose updates (JSON format)
    /// Format: {"body_name": [x,y,z,qx,qy,qz,qw], ...}
    var onWebRTCPosesReceived: (([String: [Float]]) -> Void)?
    
    private var grpcServer: GRPCServer<HTTP2ServerTransport.Posix>?
    
    init() {
        updateNetworkInfo()
    }
    
    /// Record a pose update to calculate frequency and body count
    func recordPoseUpdate(bodyCount: Int) {
        let now = Date()
        self.bodyCount = bodyCount
        
        if let lastTime = lastUpdateTime {
            let interval = now.timeIntervalSince(lastTime)
            updateTimes.append(interval)
            
            // Keep only last N samples
            if updateTimes.count > maxUpdateSamples {
                updateTimes.removeFirst()
            }
            
            // Calculate average frequency
            if updateTimes.count > 1 {
                let avgInterval = updateTimes.reduce(0, +) / Double(updateTimes.count)
                if avgInterval > 0 {
                    updateFrequency = 1.0 / avgInterval
                }
            }
        }
        lastUpdateTime = now
    }
    
    func updateNetworkInfo() {
        DispatchQueue.global(qos: .background).async {
            let ip = self.getWiFiAddress() ?? "No IP Found"
            DispatchQueue.main.async {
                self.ipAddress = ip
            }
        }
    }
    
    func updateConnectionStatus(_ status: String) {
        DispatchQueue.main.async {
            self.connectionStatus = status
        }
    }
    
    func startServer() async {
        print("🚀 [CombinedMuJoCoManager] Starting gRPC server on port \(grpcPort)...")
        updateConnectionStatus("Starting Server...")
        isServerRunning = true
        
        do {
            let service = CombinedMuJoCoARServiceImpl(manager: self)
            let transport = HTTP2ServerTransport.Posix(
                address: .ipv4(host: "0.0.0.0", port: grpcPort),
                transportSecurity: .plaintext
            )
            
            let server = GRPCServer(transport: transport, services: [service])
            self.grpcServer = server
            
            updateConnectionStatus("Server Running")
            print("✅ [CombinedMuJoCoManager] gRPC server started on port \(grpcPort)")
            
            try await server.serve()
            
        } catch {
            print("❌ [CombinedMuJoCoManager] Failed to start server: \(error)")
            updateConnectionStatus("Server Error")
            isServerRunning = false
        }
    }
    
    func stopServer() async {
        if let server = grpcServer {
            server.beginGracefulShutdown()
            grpcServer = nil
            isServerRunning = false
            updateConnectionStatus("Server Stopped")
            print("🛑 [CombinedMuJoCoManager] gRPC server stopped")
        }
    }
    
    private func getWiFiAddress() -> String? {
        var address: String?
        var ifaddr: UnsafeMutablePointer<ifaddrs>?
        
        guard getifaddrs(&ifaddr) == 0 else { return nil }
        guard let firstAddr = ifaddr else { return nil }
        
        for ifptr in sequence(first: firstAddr, next: { $0.pointee.ifa_next }) {
            let interface = ifptr.pointee
            let addrFamily = interface.ifa_addr.pointee.sa_family
            
            if addrFamily == UInt8(AF_INET) {
                let name = String(cString: interface.ifa_name)
                if name == "en0" {
                    var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                    getnameinfo(interface.ifa_addr, socklen_t(interface.ifa_addr.pointee.sa_len),
                               &hostname, socklen_t(hostname.count),
                               nil, socklen_t(0), NI_NUMERICHOST)
                    address = String(cString: hostname)
                }
            }
        }
        
        freeifaddrs(ifaddr)
        return address
    }
}

// MARK: - Combined MuJoCo AR Service Implementation

@available(macOS 15.0, iOS 18.0, watchOS 11.0, tvOS 18.0, visionOS 2.0, *)
struct CombinedMuJoCoARServiceImpl: MujocoAr_MuJoCoARService.SimpleServiceProtocol {
    weak var manager: CombinedMuJoCoManager?
    
    init(manager: CombinedMuJoCoManager) {
        self.manager = manager
        print("🎯 [CombinedMuJoCoARServiceImpl] Initialized")
    }
    
    func sendUsdzUrl(
        request: MujocoAr_UsdzUrlRequest,
        context: ServerContext
    ) async throws -> MujocoAr_UsdzUrlResponse {
        print("📨 [sendUsdzUrl] URL: \(request.usdzURL)")
        
        await MainActor.run {
            manager?.onUsdzReceived?(request.usdzURL, nil, nil)
            manager?.updateConnectionStatus("Client Connected - USDZ URL Received")
        }
        
        var response = MujocoAr_UsdzUrlResponse()
        response.success = true
        response.message = "USDZ URL received"
        return response
    }
    
    func sendUsdzData(
        request: MujocoAr_UsdzDataRequest,
        context: ServerContext
    ) async throws -> MujocoAr_UsdzDataResponse {
        print("📨 [sendUsdzData] Received \(request.usdzData.count) bytes")
        
        var response = MujocoAr_UsdzDataResponse()
        
        do {
            let tempDir = FileManager.default.temporaryDirectory
            let fileName = request.filename.isEmpty ? "\(UUID().uuidString).usdz" : request.filename
            let localURL = tempDir.appendingPathComponent(fileName)
            
            try request.usdzData.write(to: localURL)
            print("💾 [sendUsdzData] Saved to: \(localURL.path)")
            
            var attachToPosition: SIMD3<Float>? = nil
            var attachToRotation: simd_quatf? = nil
            
            if request.hasAttachToPosition && request.hasAttachToRotation {
                attachToPosition = SIMD3<Float>(request.attachToPosition.x, request.attachToPosition.y, request.attachToPosition.z)
                attachToRotation = simd_quatf(ix: request.attachToRotation.x, iy: request.attachToRotation.y, iz: request.attachToRotation.z, r: request.attachToRotation.w)
            }
            
            await MainActor.run {
                manager?.onUsdzReceived?(localURL.absoluteString, attachToPosition, attachToRotation)
                manager?.updateConnectionStatus("Client Connected - USDZ Received")
            }
            
            response.success = true
            response.message = "USDZ data received"
            response.localFilePath = localURL.path
            
        } catch {
            print("❌ [sendUsdzData] Error: \(error)")
            response.success = false
            response.message = "Failed: \(error.localizedDescription)"
        }
        
        return response
    }
    
    func sendUsdzDataChunked(
        request: RPCAsyncSequence<MujocoAr_UsdzChunkRequest, any Error>,
        context: ServerContext
    ) async throws -> MujocoAr_UsdzDataResponse {
        print("📦 [sendUsdzDataChunked] Starting chunked transfer")
        
        var response = MujocoAr_UsdzDataResponse()
        var chunkData = Data()
        var fileName = ""
        var attachToPosition: SIMD3<Float>? = nil
        var attachToRotation: simd_quatf? = nil
        var receivedChunks = 0
        
        do {
            for try await chunk in request {
                if receivedChunks == 0 {
                    fileName = chunk.filename
                    if chunk.hasAttachToPosition && chunk.hasAttachToRotation {
                        attachToPosition = SIMD3<Float>(chunk.attachToPosition.x, chunk.attachToPosition.y, chunk.attachToPosition.z)
                        attachToRotation = simd_quatf(ix: chunk.attachToRotation.x, iy: chunk.attachToRotation.y, iz: chunk.attachToRotation.z, r: chunk.attachToRotation.w)
                    }
                }
                
                chunkData.append(chunk.chunkData)
                receivedChunks += 1
                
                if chunk.isLastChunk { break }
            }
            
            let tempDir = FileManager.default.temporaryDirectory
            let finalFileName = fileName.isEmpty ? "\(UUID().uuidString).usdz" : fileName
            let localURL = tempDir.appendingPathComponent(finalFileName)
            
            try chunkData.write(to: localURL)
            print("💾 [sendUsdzDataChunked] Saved \(chunkData.count) bytes to: \(localURL.path)")
            
            await MainActor.run {
                manager?.onUsdzReceived?(localURL.absoluteString, attachToPosition, attachToRotation)
                manager?.updateConnectionStatus("Client Connected - USDZ Received (\(receivedChunks) chunks)")
            }
            
            response.success = true
            response.message = "Chunked USDZ received (\(receivedChunks) chunks)"
            response.localFilePath = localURL.path
            
        } catch {
            print("❌ [sendUsdzDataChunked] Error: \(error)")
            response.success = false
            response.message = "Failed: \(error.localizedDescription)"
        }
        
        return response
    }
    
    func updatePoses(
        request: MujocoAr_PoseUpdateRequest,
        context: ServerContext
    ) async throws -> MujocoAr_PoseUpdateResponse {
        var poses: [String: MujocoAr_BodyPose] = [:]
        for bodyPose in request.bodyPoses {
            poses[bodyPose.bodyName] = bodyPose
        }
        
        await MainActor.run {
            manager?.onPosesReceived?(poses)
            manager?.updateConnectionStatus("Streaming - \(poses.count) Bodies")
        }
        
        var response = MujocoAr_PoseUpdateResponse()
        response.success = true
        response.bodiesUpdated = Int32(poses.count)
        return response
    }
    
    func streamPoses(
        request: RPCAsyncSequence<MujocoAr_PoseUpdateRequest, any Error>,
        response: RPCWriter<MujocoAr_PoseUpdateResponse>,
        context: ServerContext
    ) async throws {
        print("🔄 [streamPoses] Starting pose stream")
        
        do {
            for try await poseRequest in request {
                var poses: [String: MujocoAr_BodyPose] = [:]
                for bodyPose in poseRequest.bodyPoses {
                    poses[bodyPose.bodyName] = bodyPose
                }
                
                await MainActor.run {
                    manager?.onPosesReceived?(poses)
                    manager?.updateConnectionStatus("Streaming - \(poses.count) Bodies")
                }
                
                var responseMsg = MujocoAr_PoseUpdateResponse()
                responseMsg.success = true
                responseMsg.bodiesUpdated = Int32(poses.count)
                try await response.write(responseMsg)
            }
        } catch {
            print("❌ [streamPoses] Error: \(error)")
            throw error
        }
        
        print("🔄 [streamPoses] Stream ended")
    }
    
    func streamHandTracking(
        request: MujocoAr_HandTrackingRequest,
        response: RPCWriter<MujocoAr_HandTrackingUpdate>,
        context: ServerContext
    ) async throws {
        print("🖐️ [streamHandTracking] Client connected")
        
        do {
            while !Task.isCancelled {
                let data = DataManager.shared.latestHandTrackingData
                
                var update = MujocoAr_HandTrackingUpdate()
                update.timestamp = Date().timeIntervalSince1970
                update.leftHand.wristMatrix = convertToMatrix4x4(from: data.leftWrist)
                update.rightHand.wristMatrix = convertToMatrix4x4(from: data.rightWrist)
                update.leftHand.skeleton.jointMatrices = data.leftSkeleton.joints.map { convertToMatrix4x4(from: $0) }
                update.rightHand.skeleton.jointMatrices = data.rightSkeleton.joints.map { convertToMatrix4x4(from: $0) }
                update.head = convertToMatrix4x4(from: data.Head)
                
                try await response.write(update)
                try await Task.sleep(nanoseconds: 10_000_000) // ~100 Hz
            }
        } catch {
            print("❌ [streamHandTracking] Error: \(error)")
            throw error
        }
        
        print("🖐️ [streamHandTracking] Client disconnected")
    }
    
    private func convertToMatrix4x4(from m: simd_float4x4) -> MujocoAr_Matrix4x4 {
        var matrix = MujocoAr_Matrix4x4()
        matrix.m00 = m.columns.0.x
        matrix.m01 = m.columns.1.x
        matrix.m02 = m.columns.2.x
        matrix.m03 = m.columns.3.x
        matrix.m10 = m.columns.0.y
        matrix.m11 = m.columns.1.y
        matrix.m12 = m.columns.2.y
        matrix.m13 = m.columns.3.y
        matrix.m20 = m.columns.0.z
        matrix.m21 = m.columns.1.z
        matrix.m22 = m.columns.2.z
        matrix.m23 = m.columns.3.z
        matrix.m30 = m.columns.0.w
        matrix.m31 = m.columns.1.w
        matrix.m32 = m.columns.2.w
        matrix.m33 = m.columns.3.w
        return matrix
    }
}
