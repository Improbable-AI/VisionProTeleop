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

// MARK: - Marker Label View (SwiftUI attachment for real-time text updates)

struct MarkerLabelView: View {
    @ObservedObject var markerManager: MarkerDetectionManager
    let index: Int
    
    /// Convert dictionary raw value to human-readable name
    private func dictionaryName(_ rawValue: Int) -> String {
        switch rawValue {
        case 0...3: return "4x4"
        case 4...7: return "5x5"
        case 8...11: return "6x6"
        case 12...15: return "7x7"
        default: return "?"
        }
    }
    
    /// Get all visible markers (detected + fixed, deduplicated)
    private var allMarkerIds: [Int] {
        let allIds = Set(markerManager.detectedMarkers.keys).union(markerManager.fixedMarkers.keys)
        return allIds.sorted()
    }
    
    /// Get marker data for display (prefer fixed if exists, otherwise detected)
    private func getMarker(_ markerId: Int) -> DetectedMarker? {
        return markerManager.fixedMarkers[markerId] ?? markerManager.detectedMarkers[markerId]
    }
    
    var body: some View {
        let sortedMarkers = allMarkerIds
        
        if index < sortedMarkers.count {
            let markerId = sortedMarkers[index]
            if let marker = getMarker(markerId) {
                let isFixed = markerManager.isMarkerFixed(markerId)
                
                VStack(spacing: 6) {
                    // Per-marker controls: Fix toggle + Delete button (1.5x larger for easier interaction)
                    HStack(spacing: 12) {
                        // Fix toggle
                        Button {
                            markerManager.setMarkerFixed(markerId, fixed: !isFixed)
                        } label: {
                            HStack(spacing: 6) {
                                Image(systemName: isFixed ? "pin.fill" : "pin")
                                    .font(.system(size: 15, weight: .bold))
                                Text(isFixed ? "Fixed" : "Fix")
                                    .font(.system(size: 15, weight: .medium))
                            }
                            .foregroundColor(isFixed ? .yellow : .white.opacity(0.8))
                            .padding(.horizontal, 12)
                            .padding(.vertical, 6)
                            .background(
                                RoundedRectangle(cornerRadius: 9)
                                    .fill(isFixed ? Color.yellow.opacity(0.3) : Color.black.opacity(0.5))
                            )
                        }
                        .buttonStyle(.plain)
                        
                        // Delete button
                        Button {
                            markerManager.deleteMarker(markerId)
                        } label: {
                            Image(systemName: "xmark.circle.fill")
                                .font(.system(size: 21))
                                .foregroundColor(.red.opacity(0.8))
                        }
                        .buttonStyle(.plain)
                    }
                    .transaction { $0.animation = nil }
                    
                    // Marker info
                    HStack(spacing: 6) {
                        Circle()
                            .fill(isFixed ? Color.yellow : (marker.isTracked ? Color.green : Color.red))
                            .frame(width: 8, height: 8)
                        
                        Text("\(dictionaryName(marker.dictionaryType)) #\(markerId)")
                            .font(.system(size: 14, weight: .bold))
                        Text("~\(String(format: "%.0f", marker.estimatedSizeMeters * 100))cm")
                            .font(.system(size: 12, weight: .medium))
                    }
                    .padding(.horizontal, 10)
                    .padding(.vertical, 6)
                    .background(
                        RoundedRectangle(cornerRadius: 8)
                            .fill(Color.black.opacity(0.7))
                    )
                    .foregroundColor(isFixed ? .yellow : .orange)
                }
            }
        }
    }
}


// MARK: - Calibration Marker Label View (SwiftUI attachment for extrinsic calibration)

struct CalibrationMarkerLabelView: View {
    @ObservedObject var calibrationManager: ExtrinsicCalibrationManager
    let index: Int
    
    var body: some View {
        let sortedMarkers = calibrationManager.arkitTrackedMarkers.keys.sorted()
        
        if index < sortedMarkers.count {
            let markerId = sortedMarkers[index]
            VStack(spacing: 4) {
                // Freeze toggle (only show for first marker)
                if index == 0 {
                    Toggle(isOn: $calibrationManager.isMarkerVisualizationFrozen) {
                        Text("Freeze")
                            .font(.system(size: 10, weight: .medium))
                    }
                    .toggleStyle(.switch)
                    .scaleEffect(0.7)
                    .fixedSize()
                    .padding(.horizontal, 8)
                    .padding(.vertical, 2)
                    .background(
                        RoundedRectangle(cornerRadius: 6)
                            .fill(Color.black.opacity(0.5))
                    )
                    .foregroundColor(.white)
                    .transaction { $0.animation = nil }
                }
                
                // Marker info
                HStack(spacing: 6) {
                    Circle()
                        .fill(Color.green)
                        .frame(width: 8, height: 8)
                    
                    Text("CAL #\(markerId)")
                        .font(.system(size: 14, weight: .bold))
                }
                .padding(.horizontal, 10)
                .padding(.vertical, 6)
                .background(
                    RoundedRectangle(cornerRadius: 8)
                        .fill(Color.black.opacity(0.7))
                )
                .foregroundColor(.purple)
            }
        }
    }
}

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
    @ObservedObject private var markerDetectionManager = MarkerDetectionManager.shared
    @ObservedObject private var extrinsicCalibrationManager = ExtrinsicCalibrationManager.shared
    
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
    @State private var fixedMarkerTransforms: [Int: Transform] = [:]  // Per-marker fixed world transforms
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
    @State private var cachedSortedBodyNames: [String] = []  // Cached sorted body names for consistent iteration
    
    // Hand joint visualization state
    @State private var handJointUpdateTrigger: UInt64 = 0
    
    // Marker visualization state
    @State private var cachedMarkerEntities: [Int: ModelEntity] = [:]  // markerId -> entity
    
    // Cached hand joint entities for performance (avoid repeated findEntity calls)
    @State private var cachedLeftJointEntities: [ModelEntity] = []
    @State private var cachedRightJointEntities: [ModelEntity] = []
    @State private var cachedLeftBoneEntities: [ModelEntity] = []
    @State private var cachedRightBoneEntities: [ModelEntity] = []
    @State private var cachedHandJointsOpacity: Float = -1.0  // Track last opacity to avoid unnecessary material updates
    @State private var cachedLeftJointMaterial: RealityKit.Material? = nil
    @State private var cachedRightJointMaterial: RealityKit.Material? = nil
    @State private var cachedLeftBoneMaterial: RealityKit.Material? = nil
    @State private var cachedRightBoneMaterial: RealityKit.Material? = nil
    
    private let applyInWorldSpace: Bool = true
    
    // Pre-computed bone connections (constant - no need to recreate each frame)
    // NEW joint ordering (matches 🥽AppModel.swift):
    //   0: wrist
    //   1-4: thumb (knuckle, intermediateBase, intermediateTip, tip)
    //   5-9: index (metacarpal, knuckle, intermediateBase, intermediateTip, tip)
    //   10-14: middle
    //   15-19: ring
    //   20-24: little
    //   25-26: forearm (forearmWrist, forearmArm)
    private static let fingerConnections: [(Int, Int)] = [
        // Thumb chain: wrist(0) -> thumbKnuckle(1) -> ... -> thumbTip(4)
        (0, 1), (1, 2), (2, 3), (3, 4),
        // Index chain: wrist(0) -> metacarpal(5) -> knuckle(6) -> ... -> tip(9)
        (0, 5), (5, 6), (6, 7), (7, 8), (8, 9),
        // Middle chain: wrist(0) -> metacarpal(10) -> knuckle(11) -> ... -> tip(14)
        (0, 10), (10, 11), (11, 12), (12, 13), (13, 14),
        // Ring chain: wrist(0) -> metacarpal(15) -> knuckle(16) -> ... -> tip(19)
        (0, 15), (15, 16), (16, 17), (17, 18), (18, 19),
        // Little chain: wrist(0) -> metacarpal(20) -> knuckle(21) -> ... -> tip(24)
        (0, 20), (20, 21), (21, 22), (22, 23), (23, 24),
    ]
    // Forearm connections: forearmArm(26) -> forearmWrist(25) -> wrist(0)
    private static let forearmConnections: [(Int, Int)] = [(26, 25), (25, 0)]
    // Palm connections: connect metacarpals across the palm
    private static let palmConnections: [(Int, Int)] = [(5, 10), (10, 15), (15, 20)]
    private static let allBoneConnections: [(Int, Int)] = fingerConnections + forearmConnections + palmConnections
    
    var body: some View {
        realityViewContent
    }
    
    // MARK: - View Components
    
    @ViewBuilder
    private var realityViewContent: some View {
        baseRealityView
            .modifier(VideoSourceModifiers(
                dataManager: dataManager,
                uvcCameraManager: uvcCameraManager,
                imageData: imageData,
                uvcFrame: $uvcFrame,
                updateTrigger: $updateTrigger,
                hasFrames: $hasFrames,
                currentAspectRatio: $currentAspectRatio
            ))
            .modifier(LifecycleModifiers(
                dataManager: dataManager,
                appModel: appModel,
                videoStreamManager: videoStreamManager,
                uvcCameraManager: uvcCameraManager,
                mujocoManager: mujocoManager,
                recordingManager: recordingManager,
                imageData: imageData,
                hasAutoMinimized: $hasAutoMinimized,
                userInteracted: $userInteracted,
                hasFrames: $hasFrames,
                hasAudio: $hasAudio,
                hasSimPoses: $hasSimPoses,
                stereoMaterialEntity: $stereoMaterialEntity,
                attachToPosition: $attachToPosition,
                attachToRotation: $attachToRotation,
                mujocoUsdzURL: $mujocoUsdzURL,
                mujocoFinalTransforms: $mujocoFinalTransforms,
                mujocoPoseUpdateTrigger: $mujocoPoseUpdateTrigger,
                mujocoBodyEntities: $mujocoBodyEntities,
                computeMuJoCoFinalTransformsFromWebRTC: computeMuJoCoFinalTransformsFromWebRTC,
                computeMuJoCoFinalTransforms: computeMuJoCoFinalTransforms,
                tryAutoMinimize: tryAutoMinimize
            ))
            .modifier(StateChangeModifiers(
                dataManager: dataManager,
                videoStreamManager: videoStreamManager,
                uvcCameraManager: uvcCameraManager,
                mujocoManager: mujocoManager,
                recordingManager: recordingManager,
                imageData: imageData,
                hasFrames: $hasFrames,
                hasAudio: $hasAudio,
                hasSimPoses: $hasSimPoses,
                videoMinimized: $videoMinimized,
                hasAutoMinimized: $hasAutoMinimized,
                fixedWorldTransform: $fixedWorldTransform,
                userInteracted: $userInteracted,
                mujocoUsdzURL: $mujocoUsdzURL,
                mujocoEntity: $mujocoEntity,
                mujocoBodyEntities: $mujocoBodyEntities,
                initialLocalTransforms: $initialLocalTransforms,
                pythonToSwiftNameMap: $pythonToSwiftNameMap,
                pythonToSwiftTargets: $pythonToSwiftTargets,
                entityPathByObjectID: $entityPathByObjectID,
                nameMappingInitialized: $nameMappingInitialized,
                mujocoFinalTransforms: $mujocoFinalTransforms,
                attachToPosition: $attachToPosition,
                attachToRotation: $attachToRotation,
                mujocoPoseUpdateTrigger: $mujocoPoseUpdateTrigger,
                loadMuJoCoModel: loadMuJoCoModel,
                tryAutoMinimize: tryAutoMinimize
            ))
            .task {
                // Hand joint visualization update loop (~180Hz when enabled)
                // Uses a simple counter instead of UUID to reduce allocation overhead
                while !Task.isCancelled {
                    if dataManager.showHandJoints {
                        handJointUpdateTrigger &+= 1  // Trigger view update (overflow-safe)
                    }
                    try? await Task.sleep(nanoseconds: 11_111_111) // ~90Hz
                }
            }
            .upperLimbVisibility(dataManager.upperLimbVisible ? .visible : .hidden)
    }
    
    @ViewBuilder
    private var baseRealityView: some View {
        RealityView { content, attachments in
            setupRealityViewContent(content: content, attachments: attachments)
        } update: { updateContent, attachments in
            let _ = updateTrigger
            let _ = dataManager.videoPlaneZDistance
            let _ = dataManager.videoPlaneYPosition
            let _ = dataManager.videoPlaneScale  // Trigger update when scale changes
            let _ = dataManager.stereoBaselineOffset // Trigger update when baseline offset changes
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
            let _ = dataManager.showHeadBeam  // Trigger update when head beam visibility changes
            let _ = dataManager.showHandJoints  // Trigger update when hand joints visibility changes
            let _ = dataManager.handJointsOpacity  // Trigger update when hand joints opacity changes
            let _ = handJointUpdateTrigger  // Trigger update when hand tracking data changes
            let _ = extrinsicCalibrationManager.verificationMarkerPose // Trigger update when verification pose changes
            
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
                    // Update preview plane size based on current scale (even without video frames)
                    let scale = dataManager.videoPlaneScale
                    let previewHeight: Float = 9.6 * scale
                    let previewWidth = previewHeight * (16.0 / 9.0)  // Default 16:9 aspect ratio for preview
                    let previewMesh = MeshResource.generatePlane(width: previewWidth, height: previewHeight)
                    previewEntity?.components[ModelComponent.self]?.mesh = previewMesh
                } else {
                    previewEntity?.isEnabled = false
                }
                
                // Determine video source and get appropriate frame
                let isUVCMode = dataManager.videoSource == .uvcCamera
                let networkFrameAvailable = imageData.left != nil && imageData.right != nil
                let uvcFrameAvailable = uvcFrame != nil && isUVCMode
                let hasVideoFrame = isUVCMode ? uvcFrameAvailable : networkFrameAvailable
                
                // Hide video plane when calibration wizard is active (video goes to wizard instead)
                let hideForCalibration = dataManager.isCalibrationWizardActive
                
                // Update video texture based on source
                if hasVideoFrame && !hideForCalibration, let skyBox = skyBoxEntity {
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
                    
                    // Calculate aspect ratio - for stereo UVC, the displayed image is half width (side-by-side split)
                    let isUVCStereo = isUVCMode && UVCCameraManager.shared.stereoEnabled
                    let effectiveWidth = isUVCStereo ? imageWidth / 2 : imageWidth
                    let aspectRatio = Float(effectiveWidth / imageHeight)
                    let scale = dataManager.videoPlaneScale
                    
                    if currentAspectRatio == nil || abs(currentAspectRatio! - aspectRatio) > 0.01 {
                        currentAspectRatio = aspectRatio
                    }
                    
                    // Apply scale factor to plane dimensions
                    let planeHeight: Float = 9.6 * scale
                    var planeWidth = planeHeight * aspectRatio
                    
                    // Adjust plane width for stereo baseline cropping (crop not stretch)
                    let baselineOffset = abs(dataManager.stereoBaselineOffset)
                    if baselineOffset > 0.001 {
                        planeWidth *= Float(1.0 - baselineOffset)
                    }
                    
                    let newMesh = MeshResource.generatePlane(width: planeWidth, height: planeHeight)
                    skyBox.components[ModelComponent.self]?.mesh = newMesh
                    previewEntity?.components[ModelComponent.self]?.mesh = newMesh
                    
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
                                dlog("🎬 [CombinedStreamingView] First video frame received (source: \(isUVCMode ? "UVC" : "Network")), hasFrames=\(hasFrames)")
                                // Small delay to ensure state is fully committed
                                try? await Task.sleep(nanoseconds: 100_000_000)  // 100ms
                                dlog("🎬 [CombinedStreamingView] Checking auto-minimize after delay")
                                tryAutoMinimize()
                            }
                        }
                    }
                    
                    // Check stereo mode: UVC uses UVCCameraManager.stereoEnabled, network uses DataManager.stereoEnabled
                    let isStereo = isUVCMode ? UVCCameraManager.shared.stereoEnabled : DataManager.shared.stereoEnabled
                    
                    if isStereo {
                        // Stereo mode: need left and right images
                        let leftImage: CGImage?
                        let rightImage: CGImage?
                        
                        if isUVCMode, let uvc = uvcFrame, let cgImage = uvc.cgImage {
                            // Split UVC side-by-side frame into left and right halves
                            let width = cgImage.width
                            let height = cgImage.height
                            let halfWidth = width / 2
                            
                            let leftRect = CGRect(x: 0, y: 0, width: halfWidth, height: height)
                            let rightRect = CGRect(x: halfWidth, y: 0, width: halfWidth, height: height)
                            
                            leftImage = cgImage.cropping(to: leftRect)
                            rightImage = cgImage.cropping(to: rightRect)
                        } else if let imgLeft = imageData.left, let imgRight = imageData.right {
                            // Network stream already has separate left/right
                            leftImage = imgLeft.cgImage
                            rightImage = imgRight.cgImage
                        } else {
                            leftImage = nil
                            rightImage = nil
                        }
                        
                        if let leftCG = leftImage, let rightCG = rightImage {
                            // Apply baseline offset cropping if needed
                            let offset = CGFloat(dataManager.stereoBaselineOffset)
                            let finalLeftImage: CGImage?
                            let finalRightImage: CGImage?
                            
                            if abs(offset) > 0.001 {
                                let width = CGFloat(leftCG.width)
                                let height = CGFloat(leftCG.height)
                                let cropWidth = width * (1.0 - abs(offset))
                                
                                // Calculate crop rects based on offset direction
                                // Negative offset (Narrower): Crop outer sides (Left: Crop Left, Right: Crop Right) -> Visual shift inward
                                // Positive offset (Wider): Crop inner sides (Left: Crop Right, Right: Crop Left) -> Visual shift outward
                                
                                let leftRect: CGRect
                                let rightRect: CGRect
                                
                                if offset < 0 {
                                    // Narrower: Left eye crops left side (keeps right), Right eye crops right side (keeps left)
                                    leftRect = CGRect(x: abs(offset) * width, y: 0, width: cropWidth, height: height)
                                    rightRect = CGRect(x: 0, y: 0, width: cropWidth, height: height)
                                } else {
                                    // Wider: Left eye crops right side (keeps left), Right eye crops left side (keeps right)
                                    leftRect = CGRect(x: 0, y: 0, width: cropWidth, height: height)
                                    rightRect = CGRect(x: abs(offset) * width, y: 0, width: cropWidth, height: height)
                                }
                                
                                finalLeftImage = leftCG.cropping(to: leftRect)
                                finalRightImage = rightCG.cropping(to: rightRect)
                            } else {
                                finalLeftImage = leftCG
                                finalRightImage = rightCG
                            }
                            
                            do {
                                guard let sphereEntity = stereoMaterialEntity,
                                      var stereoMaterial = sphereEntity.components[ModelComponent.self]?.materials.first as? ShaderGraphMaterial else {
                                    var skyBoxMaterial = UnlitMaterial()
                                    var textureOptions = TextureResource.CreateOptions(semantic: .hdrColor)
                                    textureOptions.mipmapsMode = .none
                                    if let img = finalRightImage {
                                        let texture = try TextureResource.generate(from: img, options: textureOptions)
                                        skyBoxMaterial.color = .init(texture: .init(texture))
                                        skyBox.components[ModelComponent.self]?.materials = [skyBoxMaterial]
                                    }
                                    return
                                }
                                
                                var textureOptions = TextureResource.CreateOptions(semantic: .hdrColor)
                                textureOptions.mipmapsMode = .none
                                
                                if let lImg = finalLeftImage, let rImg = finalRightImage {
                                    let leftTexture = try TextureResource.generate(from: lImg, options: textureOptions)
                                    let rightTexture = try TextureResource.generate(from: rImg, options: textureOptions)
                                    try stereoMaterial.setParameter(name: "left", value: .textureResource(leftTexture))
                                    try stereoMaterial.setParameter(name: "right", value: .textureResource(rightTexture))
                                    skyBox.components[ModelComponent.self]?.materials = [stereoMaterial]
                                }
                            } catch {
                                dlog("❌ ERROR: Failed to load stereo textures: \(error)")
                            }
                        } else {
                            // Fallback to mono if stereo split fails
                            var skyBoxMaterial = UnlitMaterial()
                            do {
                                var textureOptions = TextureResource.CreateOptions(semantic: .hdrColor)
                                textureOptions.mipmapsMode = .none
                                let texture = try TextureResource.generate(from: displayImage.cgImage!, options: textureOptions)
                                skyBoxMaterial.color = .init(texture: .init(texture))
                                skyBox.components[ModelComponent.self]?.materials = [skyBoxMaterial]
                            } catch {
                                dlog("❌ ERROR: Failed to load fallback mono texture: \(error)")
                            }
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
                            dlog("❌ ERROR: Failed to load mono texture: \(error)")
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
                        targetTranslation = SIMD3<Float>(0.0, -0.1, -1.0)
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
                    dlog("🔗 [CombinedStreamingView] Adding MuJoCo model to scene")
                    mujocoRoot.addChild(entity)
                }
            }
            
            // === HEAD BEAM UPDATE ===
            if let headBeamAnchor = findEntity(named: "headBeamAnchor", in: updateContent.entities),
               let headBeam = headBeamAnchor.findEntity(named: "headBeam") {
                headBeam.isEnabled = dataManager.showHeadBeam
            }
            
            // === HAND JOINTS UPDATE (Optimized) ===
            if let handJointsRoot = findEntity(named: "handJointsRoot", in: updateContent.entities) {
                handJointsRoot.isEnabled = dataManager.showHandJoints
                
                if dataManager.showHandJoints {
                    // Use pre-computed static bone connections
                    let allConnections = Self.allBoneConnections
                    
                    // Cache entity references on first access (avoids repeated findEntity calls)
                    // Use local variables to ensure we have the entities on the same frame
                    var leftJoints = cachedLeftJointEntities
                    var rightJoints = cachedRightJointEntities
                    var leftBones = cachedLeftBoneEntities
                    var rightBones = cachedRightBoneEntities
                    
                    if leftJoints.isEmpty {
                        for i in 0..<27 {
                            if let entity = handJointsRoot.findEntity(named: "leftJoint_\(i)") as? ModelEntity {
                                leftJoints.append(entity)
                            }
                        }
                        cachedLeftJointEntities = leftJoints
                    }
                    if rightJoints.isEmpty {
                        for i in 0..<27 {
                            if let entity = handJointsRoot.findEntity(named: "rightJoint_\(i)") as? ModelEntity {
                                rightJoints.append(entity)
                            }
                        }
                        cachedRightJointEntities = rightJoints
                    }
                    if leftBones.isEmpty {
                        for (idx, connection) in allConnections.enumerated() {
                            if let entity = handJointsRoot.findEntity(named: "leftBone_\(idx)_\(connection.0)_\(connection.1)") as? ModelEntity {
                                leftBones.append(entity)
                            }
                        }
                        cachedLeftBoneEntities = leftBones
                    }
                    if rightBones.isEmpty {
                        for (idx, connection) in allConnections.enumerated() {
                            if let entity = handJointsRoot.findEntity(named: "rightBone_\(idx)_\(connection.0)_\(connection.1)") as? ModelEntity {
                                rightBones.append(entity)
                            }
                        }
                        cachedRightBoneEntities = rightBones
                    }
                    
                    let trackingData = DataManager.shared.latestHandTrackingData
                    let leftWrist = trackingData.leftWrist
                    let rightWrist = trackingData.rightWrist
                    let opacity = dataManager.handJointsOpacity
                    
                    // Only recreate materials if opacity changed (expensive operation)
                    let opacityChanged = abs(cachedHandJointsOpacity - opacity) > 0.001
                    if opacityChanged {
                        cachedHandJointsOpacity = opacity
                        let cgOpacity = CGFloat(opacity)
                        
                        if opacity >= 0.99 {
                            // Fully opaque - use UnlitMaterial for brighter, unlit appearance
                            var leftJoint = UnlitMaterial()
                            leftJoint.color = .init(tint: UIColor(red: 0.2, green: 0.8, blue: 1.0, alpha: 1.0))
                            cachedLeftJointMaterial = leftJoint
                            
                            var rightJoint = UnlitMaterial()
                            rightJoint.color = .init(tint: UIColor(red: 1.0, green: 0.6, blue: 0.2, alpha: 1.0))
                            cachedRightJointMaterial = rightJoint
                            
                            var leftBone = UnlitMaterial()
                            leftBone.color = .init(tint: UIColor(red: 0.1, green: 0.6, blue: 0.8, alpha: 1.0))
                            cachedLeftBoneMaterial = leftBone
                            
                            var rightBone = UnlitMaterial()
                            rightBone.color = .init(tint: UIColor(red: 0.9, green: 0.5, blue: 0.1, alpha: 1.0))
                            cachedRightBoneMaterial = rightBone
                        } else {
                            // Transparent - use SimpleMaterial for proper alpha blending
                            var leftJoint = SimpleMaterial()
                            leftJoint.color = .init(tint: UIColor(red: 0.2, green: 0.8, blue: 1.0, alpha: cgOpacity))
                            leftJoint.metallic = 0.0
                            leftJoint.roughness = 1.0
                            cachedLeftJointMaterial = leftJoint
                            
                            var rightJoint = SimpleMaterial()
                            rightJoint.color = .init(tint: UIColor(red: 1.0, green: 0.6, blue: 0.2, alpha: cgOpacity))
                            rightJoint.metallic = 0.0
                            rightJoint.roughness = 1.0
                            cachedRightJointMaterial = rightJoint
                            
                            var leftBone = SimpleMaterial()
                            leftBone.color = .init(tint: UIColor(red: 0.1, green: 0.6, blue: 0.8, alpha: cgOpacity * 0.9))
                            leftBone.metallic = 0.0
                            leftBone.roughness = 1.0
                            cachedLeftBoneMaterial = leftBone
                            
                            var rightBone = SimpleMaterial()
                            rightBone.color = .init(tint: UIColor(red: 0.9, green: 0.5, blue: 0.1, alpha: cgOpacity * 0.9))
                            rightBone.metallic = 0.0
                            rightBone.roughness = 1.0
                            cachedRightBoneMaterial = rightBone
                        }
                        
                        // Apply materials to all entities when opacity changes
                        if let leftMat = cachedLeftJointMaterial {
                            for entity in leftJoints {
                                entity.model?.materials = [leftMat]
                            }
                        }
                        if let rightMat = cachedRightJointMaterial {
                            for entity in rightJoints {
                                entity.model?.materials = [rightMat]
                            }
                        }
                        if let leftBoneMat = cachedLeftBoneMaterial {
                            for entity in leftBones {
                                entity.model?.materials = [leftBoneMat]
                            }
                        }
                        if let rightBoneMat = cachedRightBoneMaterial {
                            for entity in rightBones {
                                entity.model?.materials = [rightBoneMat]
                            }
                        }
                    }
                    
                    // Calculate and update left hand positions (fast path - only position updates)
                    var leftJointPositions: [SIMD3<Float>] = []
                    let leftWristPos = SIMD3<Float>(leftWrist.columns.3.x, leftWrist.columns.3.y, leftWrist.columns.3.z)
                    let leftHandValid = simd_length(leftWristPos) > 0.01
                    
                    for i in 0..<27 {
                        let jointLocal = trackingData.leftSkeleton.joints[i]
                        let jointWorld = simd_mul(leftWrist, jointLocal)
                        let position = SIMD3<Float>(jointWorld.columns.3.x, jointWorld.columns.3.y, jointWorld.columns.3.z)
                        leftJointPositions.append(position)
                        
                        if i < leftJoints.count {
                            let jointEntity = leftJoints[i]
                            if leftHandValid {
                                jointEntity.position = position
                                jointEntity.isEnabled = true
                            } else {
                                jointEntity.isEnabled = false
                            }
                        }
                    }
                    
                    // Update left hand bones
                    for (idx, connection) in allConnections.enumerated() {
                        if idx < leftBones.count {
                            let boneEntity = leftBones[idx]
                            // Skip forearm bones (indices 25, 26) if forearm data is not available
                            let usesForearm = connection.0 >= 25 || connection.1 >= 25
                            let forearmValid = usesForearm ? simd_length(leftJointPositions[25]) > 0.01 : true
                            let startPos = leftJointPositions[connection.0]
                            let endPos = leftJointPositions[connection.1]
                            // Validate both positions are not at origin (prevents rods from origin)
                            let startValid = simd_length(startPos) > 0.01
                            let endValid = simd_length(endPos) > 0.01
                            if leftHandValid && forearmValid && startValid && endValid {
                                updateBoneEntity(boneEntity, from: startPos, to: endPos)
                                boneEntity.isEnabled = true
                            } else {
                                boneEntity.isEnabled = false
                            }
                        }
                    }
                    
                    // Calculate and update right hand positions
                    var rightJointPositions: [SIMD3<Float>] = []
                    let rightWristPos = SIMD3<Float>(rightWrist.columns.3.x, rightWrist.columns.3.y, rightWrist.columns.3.z)
                    let rightHandValid = simd_length(rightWristPos) > 0.01
                    
                    for i in 0..<27 {
                        let jointLocal = trackingData.rightSkeleton.joints[i]
                        let jointWorld = simd_mul(rightWrist, jointLocal)
                        let position = SIMD3<Float>(jointWorld.columns.3.x, jointWorld.columns.3.y, jointWorld.columns.3.z)
                        rightJointPositions.append(position)
                        
                        if i < rightJoints.count {
                            let jointEntity = rightJoints[i]
                            if rightHandValid {
                                jointEntity.position = position
                                jointEntity.isEnabled = true
                            } else {
                                jointEntity.isEnabled = false
                            }
                        }
                    }
                    
                    // Update right hand bones
                    for (idx, connection) in allConnections.enumerated() {
                        if idx < rightBones.count {
                            let boneEntity = rightBones[idx]
                            // Skip forearm bones (indices 25, 26) if forearm data is not available
                            let usesForearm = connection.0 >= 25 || connection.1 >= 25
                            let forearmValid = usesForearm ? simd_length(rightJointPositions[25]) > 0.01 : true
                            let startPos = rightJointPositions[connection.0]
                            let endPos = rightJointPositions[connection.1]
                            // Validate both positions are not at origin (prevents rods from origin)
                            let startValid = simd_length(startPos) > 0.01
                            let endValid = simd_length(endPos) > 0.01
                            if rightHandValid && forearmValid && startValid && endValid {
                                updateBoneEntity(boneEntity, from: startPos, to: endPos)
                                boneEntity.isEnabled = true
                            } else {
                                boneEntity.isEnabled = false
                            }
                        }
                    }
                }
            }
            
            // === MARKER DETECTION UPDATE ===
            if let markerRoot = findEntity(named: "markerRoot", in: updateContent.entities),
               let markerWorldAnchor = findEntity(named: "markerWorldAnchor", in: updateContent.entities) {
                let markerManager = MarkerDetectionManager.shared
                
                // Merge detected and fixed markers - fixed markers override detected ones
                var allMarkers: [Int: DetectedMarker] = markerManager.detectedMarkers
                for (id, marker) in markerManager.fixedMarkers {
                    allMarkers[id] = marker
                }
                
                // Get sorted marker IDs for consistent entity assignment
                let sortedMarkerIds = allMarkers.keys.sorted()
                
                // Track which entity indices are in use
                var usedEntityIndices = Set<Int>()
                
                if markerManager.isEnabled && !allMarkers.isEmpty {
                    var entityIndex = 0
                    for markerId in sortedMarkerIds {
                        guard entityIndex < 20 else { break }
                        guard let marker = allMarkers[markerId] else { continue }
                        
                        let isFixed = markerManager.isMarkerFixed(markerId)
                        
                        if let markerEntity = markerRoot.findEntity(named: "marker_\(entityIndex)") ??
                                              markerWorldAnchor.findEntity(named: "marker_\(entityIndex)") {
                            
                            // Apply full transform from ARKit anchor (or fixed pose)
                            // Then rotate -90° around X and 180° around Z to align correctly
                            var transform = Transform(matrix: marker.poseInWorld)
                            let rotateX = simd_quatf(angle: -.pi / 2, axis: [1, 0, 0])  // -90° around X
                            let rotateZ = simd_quatf(angle: .pi, axis: [0, 0, 1])  // 180° around Z
                            transform.rotation = transform.rotation * rotateX * rotateZ
                            
                            if isFixed {
                                // Fixed marker: parent to world anchor, maintain world position
                                if markerEntity.parent !== markerWorldAnchor {
                                    // Store the transform to apply after re-parenting
                                    fixedMarkerTransforms[markerId] = transform
                                    markerEntity.setParent(markerWorldAnchor, preservingWorldTransform: false)
                                }
                                // Apply the fixed transform
                                if let lockedTransform = fixedMarkerTransforms[markerId] {
                                    markerEntity.move(to: lockedTransform, relativeTo: markerWorldAnchor, duration: 0.1, timingFunction: .linear)
                                }
                            } else {
                                // Live marker: parent to markerRoot, update from tracking
                                if markerEntity.parent !== markerRoot {
                                    markerEntity.setParent(markerRoot, preservingWorldTransform: true)
                                    fixedMarkerTransforms.removeValue(forKey: markerId)
                                }
                                // Use move for smoother transitions (no flashing)
                                markerEntity.move(to: transform, relativeTo: markerRoot, duration: 0.05, timingFunction: .linear)
                            }
                            markerEntity.isEnabled = true
                            usedEntityIndices.insert(entityIndex)
                            
                            // Use estimated size from this specific marker
                            let markerSize = marker.estimatedSizeMeters
                            let halfSize = markerSize / 2
                            
                            // Update boundary edges - in the XY plane (marker surface)
                            // After flip: X=right, Y=down in image, Z=out toward viewer
                            if let boundary0 = markerEntity.findEntity(named: "boundary_0") as? ModelEntity {
                                boundary0.position = [0, halfSize, 0]  // Top edge
                                boundary0.scale = [markerSize / 0.1, 1, 1]
                                boundary0.transform.rotation = .init(angle: 0, axis: [0, 0, 1])
                            }
                            if let boundary1 = markerEntity.findEntity(named: "boundary_1") as? ModelEntity {
                                boundary1.position = [0, -halfSize, 0]  // Bottom edge
                                boundary1.scale = [markerSize / 0.1, 1, 1]
                            }
                            if let boundary2 = markerEntity.findEntity(named: "boundary_2") as? ModelEntity {
                                boundary2.position = [halfSize, 0, 0]  // Right edge
                                boundary2.scale = [markerSize / 0.1, 1, 1]
                                boundary2.transform.rotation = simd_quatf(angle: .pi / 2, axis: [0, 0, 1])
                            }
                            if let boundary3 = markerEntity.findEntity(named: "boundary_3") as? ModelEntity {
                                boundary3.position = [-halfSize, 0, 0]  // Left edge
                                boundary3.scale = [markerSize / 0.1, 1, 1]
                                boundary3.transform.rotation = simd_quatf(angle: .pi / 2, axis: [0, 0, 1])
                            }
                            
                            // Update label position (attachment is already parented from setup)
                            if let labelEntity = markerEntity.findEntity(named: "label") {
                                labelEntity.position = [0, 0, 0.05]  // 5cm out from marker surface along Z
                            }
                        }
                        entityIndex += 1
                    }
                }
                
                // Hide only unused entity slots (instead of all markers)
                for i in 0..<20 where !usedEntityIndices.contains(i) {
                    if let markerEntity = markerRoot.findEntity(named: "marker_\(i)") {
                        markerEntity.isEnabled = false
                    }
                    if let markerEntity = markerWorldAnchor.findEntity(named: "marker_\(i)") {
                        markerEntity.isEnabled = false
                    }
                }
            }

            
            // === CALIBRATION MARKER UPDATE ===
            if let calMarkerRoot = updateContent.entities.first(where: { $0.name == "calibrationMarkerRoot" }) {
                // Only update poses if not frozen - otherwise keep markers at last known positions
                if !extrinsicCalibrationManager.isMarkerVisualizationFrozen {
                    // Update with current calibration markers
                    let calMarkers = extrinsicCalibrationManager.arkitTrackedMarkers
                    let markerCount = calMarkers.count
                    let markerSize = extrinsicCalibrationManager.markerSizeMeters
                    let halfSize = markerSize / 2
                    
                    var entityIndex = 0
                    for (markerId, poseMatrix) in calMarkers.sorted(by: { $0.key < $1.key }) {
                        guard entityIndex < 10 else { break }
                        if let calMarkerEntity = calMarkerRoot.findEntity(named: "calMarker_\(entityIndex)") {
                            // Apply same rotation correction as detection markers
                            var transform = Transform(matrix: poseMatrix)
                            let rotateX = simd_quatf(angle: -.pi / 2, axis: [1, 0, 0])
                            let rotateZ = simd_quatf(angle: .pi, axis: [0, 0, 1])
                            transform.rotation = transform.rotation * rotateX * rotateZ
                            calMarkerEntity.transform = transform
                            calMarkerEntity.isEnabled = true
                            
                            // Update boundary edges
                            if let boundary0 = calMarkerEntity.findEntity(named: "boundary_0") as? ModelEntity {
                                boundary0.position = [0, halfSize, 0]
                                boundary0.scale = [markerSize / 0.1, 1, 1]
                            }
                            if let boundary1 = calMarkerEntity.findEntity(named: "boundary_1") as? ModelEntity {
                                boundary1.position = [0, -halfSize, 0]
                                boundary1.scale = [markerSize / 0.1, 1, 1]
                            }
                            if let boundary2 = calMarkerEntity.findEntity(named: "boundary_2") as? ModelEntity {
                                boundary2.position = [halfSize, 0, 0]
                                boundary2.scale = [markerSize / 0.1, 1, 1]
                                boundary2.transform.rotation = simd_quatf(angle: .pi / 2, axis: [0, 0, 1])
                            }
                            if let boundary3 = calMarkerEntity.findEntity(named: "boundary_3") as? ModelEntity {
                                boundary3.position = [-halfSize, 0, 0]
                                boundary3.scale = [markerSize / 0.1, 1, 1]
                                boundary3.transform.rotation = simd_quatf(angle: .pi / 2, axis: [0, 0, 1])
                            }
                            
                            // Update wall size to match marker width
                            if let wall = calMarkerEntity.findEntity(named: "wall") as? ModelEntity {
                                wall.scale = [markerSize / 0.1, 1, 1]
                            }
                            
                            if let labelEntity = calMarkerEntity.findEntity(named: "label") {
                                labelEntity.position = [0, 0, 0.05]
                            }
                            
                            entityIndex += 1
                        }
                    }
                    
                    // Only disable markers beyond current count (avoid blanket disable which causes flashing)
                    for i in entityIndex..<10 {
                        if let calMarkerEntity = calMarkerRoot.findEntity(named: "calMarker_\(i)") {
                            calMarkerEntity.isEnabled = false
                        }
                    }
                }  // End of if !isMarkerVisualizationFrozen
            }
            
            // === VERIFICATION MARKER UPDATE ===
            if let verMarkerRoot = updateContent.entities.first(where: { $0.name == "verificationMarkerRoot" }) {
                if let verMarker = verMarkerRoot.findEntity(named: "verificationMarker") {
                    
                    if let pose = extrinsicCalibrationManager.verificationMarkerPose {
                        // Apply transform
                        let transform = Transform(matrix: pose)
                        // Note: verificationMarkerPose is already in World coordinate frame with correct orientation
                        // (Computed via T_world_head * inv(T_head_cam) * T_cam_marker)
                        // So no additional rotation correction is needed here.
                        
                        verMarker.transform = transform
                        verMarker.isEnabled = true
                        
                        // Update boundary scale
                        let markerSize = extrinsicCalibrationManager.markerSizeMeters
                        let halfSize = markerSize / 2
                        
                        if let b0 = verMarker.findEntity(named: "boundary_0") as? ModelEntity {
                            b0.position = [0, halfSize, 0]
                            b0.scale = [markerSize / 0.1, 1, 1]
                        }
                        if let b1 = verMarker.findEntity(named: "boundary_1") as? ModelEntity {
                            b1.position = [0, -halfSize, 0]
                            b1.scale = [markerSize / 0.1, 1, 1]
                        }
                        if let b2 = verMarker.findEntity(named: "boundary_2") as? ModelEntity {
                            b2.position = [halfSize, 0, 0]
                            b2.scale = [markerSize / 0.1, 1, 1]
                            b2.transform.rotation = simd_quatf(angle: .pi / 2, axis: [0, 0, 1])
                        }
                        if let b3 = verMarker.findEntity(named: "boundary_3") as? ModelEntity {
                            b3.position = [-halfSize, 0, 0]
                            b3.scale = [markerSize / 0.1, 1, 1]
                            b3.transform.rotation = simd_quatf(angle: .pi / 2, axis: [0, 0, 1])
                        }
                        
                    } else {
                        verMarker.isEnabled = false
                    }
                }
            }
            
            // === ACCESSORY TRACKING ===
            // Update snapshots for streaming (captures current stylus transform)
            if #available(visionOS 26.0, *) {
                AccessoryTrackingManager.shared.updateSnapshots()
            }
            
            // === MUJOCO POSE UPDATE ===
            if !mujocoFinalTransforms.isEmpty {
                applyMuJoCoTransforms(mujocoFinalTransforms)
            }
            
        } attachments: {
            statusAttachments
        }
    }
    
    // MARK: - Attachments
    
    @AttachmentContentBuilder
    private var statusAttachments: some AttachmentContent {
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
        
        // Marker label attachments (20 labels for up to 20 markers)
        markerLabelAttachments
    }
    
    @AttachmentContentBuilder
    private var markerLabelAttachments: some AttachmentContent {
        ForEach(0..<20, id: \.self) { index in
            Attachment(id: "markerLabel_\(index)") {
                MarkerLabelView(markerManager: markerDetectionManager, index: index)
            }
        }
        // Calibration marker labels (10 labels)
        ForEach(0..<10, id: \.self) { index in
            Attachment(id: "calMarkerLabel_\(index)") {
                CalibrationMarkerLabelView(calibrationManager: extrinsicCalibrationManager, index: index)
            }
        }
    }
    
    // MARK: - RealityView Setup
    
    private func setupRealityViewContent(content: RealityViewContent, attachments: RealityViewAttachments) {
        dlog("🟢 [CombinedStreamingView] RealityView content block called")
        
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
        
        // === HEAD BEAM SETUP ===
        let headBeamAnchor = AnchorEntity(.head)
        headBeamAnchor.name = "headBeamAnchor"
        content.add(headBeamAnchor)
        
        let headBeamEntity = createHeadBeam()
        headBeamEntity.name = "headBeam"
        headBeamEntity.isEnabled = dataManager.showHeadBeam
        headBeamEntity.setParent(headBeamAnchor)
        
        // === HAND JOINTS SETUP ===
        let handJointsRoot = Entity()
        handJointsRoot.name = "handJointsRoot"
        handJointsRoot.isEnabled = dataManager.showHandJoints
        content.add(handJointsRoot)
        
        // Create 54 spheres for hand joint visualization (27 per hand)
        // Joint indices: 0=wrist, 1-4=thumb, 5-9=index, 10-14=middle, 15-19=ring, 20-24=little, 25=forearmWrist, 26=forearmArm
        let jointRadius: Float = 0.008  // 8mm radius spheres
        let jointMesh = MeshResource.generateSphere(radius: jointRadius)
        
        // Left hand joints (cyan/blue color)
        var leftHandMaterial = UnlitMaterial()
        leftHandMaterial.color = .init(tint: UIColor(red: 0.2, green: 0.8, blue: 1.0, alpha: 0.9))
        
        for i in 0..<27 {
            let jointEntity = ModelEntity(mesh: jointMesh, materials: [leftHandMaterial])
            jointEntity.name = "leftJoint_\(i)"
            jointEntity.isEnabled = false
            handJointsRoot.addChild(jointEntity)
        }
        
        // Right hand joints (orange color)
        var rightHandMaterial = UnlitMaterial()
        rightHandMaterial.color = .init(tint: UIColor(red: 1.0, green: 0.6, blue: 0.2, alpha: 0.9))
        
        for i in 0..<27 {
            let jointEntity = ModelEntity(mesh: jointMesh, materials: [rightHandMaterial])
            jointEntity.name = "rightJoint_\(i)"
            jointEntity.isEnabled = false
            handJointsRoot.addChild(jointEntity)
        }
        
        // Skeleton bone connections - use static properties to ensure consistency with update loop
        // Joint indices: 0=wrist, 1-4=thumb, 5-9=index, 10-14=middle, 15-19=ring, 20-24=little, 25=forearmWrist, 26=forearmArm
        let allConnections = Self.allBoneConnections
        
        // Create a unit cylinder mesh (height=1) to be scaled per bone
        let boneRadius: Float = 0.003  // 3mm radius
        let unitBoneMesh = MeshResource.generateCylinder(height: 1.0, radius: boneRadius)
        
        // Create bone entities for left hand
        var leftBoneMaterial = UnlitMaterial()
        leftBoneMaterial.color = .init(tint: UIColor(red: 0.1, green: 0.6, blue: 0.8, alpha: 0.8))
        
        for (idx, connection) in allConnections.enumerated() {
            let boneEntity = ModelEntity(mesh: unitBoneMesh, materials: [leftBoneMaterial])
            boneEntity.name = "leftBone_\(idx)_\(connection.0)_\(connection.1)"
            boneEntity.isEnabled = false
            handJointsRoot.addChild(boneEntity)
        }
        
        // Create bone entities for right hand
        var rightBoneMaterial = UnlitMaterial()
        rightBoneMaterial.color = .init(tint: UIColor(red: 0.9, green: 0.5, blue: 0.1, alpha: 0.8))
        
        for (idx, connection) in allConnections.enumerated() {
            let boneEntity = ModelEntity(mesh: unitBoneMesh, materials: [rightBoneMaterial])
            boneEntity.name = "rightBone_\(idx)_\(connection.0)_\(connection.1)"
            boneEntity.isEnabled = false
            handJointsRoot.addChild(boneEntity)
        }
        
        // === MARKER DETECTION VISUALIZATION ===
        // World anchor for fixed markers (same pattern as video plane)
        let markerWorldAnchor = AnchorEntity(world: .zero)
        markerWorldAnchor.name = "markerWorldAnchor"
        content.add(markerWorldAnchor)
        
        let markerRoot = Entity()
        markerRoot.name = "markerRoot"
        content.add(markerRoot)
        
        // Create coordinate frame + boundary for each potential marker (up to 20)
        let axisLength: Float = 0.05  // 5cm axis length
        let axisRadius: Float = 0.003  // 3mm radius
        let boundaryWidth: Float = 0.003  // 3mm line width
        
        // Axis meshes
        let axisMesh = MeshResource.generateCylinder(height: axisLength, radius: axisRadius)
        let originMesh = MeshResource.generateSphere(radius: axisRadius * 2)
        
        // Axis materials (RGB = XYZ)
        var xMaterial = UnlitMaterial()
        xMaterial.color = .init(tint: .red)
        var yMaterial = UnlitMaterial()
        yMaterial.color = .init(tint: .green)
        var zMaterial = UnlitMaterial()
        zMaterial.color = .init(tint: .blue)
        var originMaterial = UnlitMaterial()
        originMaterial.color = .init(tint: .white)
        var boundaryMaterial = UnlitMaterial()
        boundaryMaterial.color = .init(tint: UIColor(red: 1.0, green: 0.6, blue: 0.1, alpha: 0.9))
        
        for i in 0..<20 {
            // Container for the entire marker visualization
            let markerContainer = Entity()
            markerContainer.name = "marker_\(i)"
            markerContainer.isEnabled = false
            markerRoot.addChild(markerContainer)
            
            // Origin sphere (white)
            let origin = ModelEntity(mesh: originMesh, materials: [originMaterial])
            origin.name = "origin"
            markerContainer.addChild(origin)
            
            // ARKit ImageAnchor coordinate system:
            // X = right (red), Y = up in image plane (green), Z = out of image (blue, normal)
            
            // X axis (red) - cylinder along X
            let xAxis = ModelEntity(mesh: axisMesh, materials: [xMaterial])
            xAxis.name = "xAxis"
            xAxis.transform.rotation = simd_quatf(angle: -.pi / 2, axis: [0, 0, 1])  // Rotate to point along +X
            xAxis.position = [axisLength / 2, 0, 0]
            markerContainer.addChild(xAxis)
            
            // Y axis (green) - cylinder along Y (in the marker plane)
            let yAxis = ModelEntity(mesh: axisMesh, materials: [yMaterial])
            yAxis.name = "yAxis"
            // Default cylinder is along Y, no rotation needed
            yAxis.position = [0, axisLength / 2, 0]
            markerContainer.addChild(yAxis)
            
            // Z axis (blue) - cylinder along Z (pointing out of marker, the normal)
            let zAxis = ModelEntity(mesh: axisMesh, materials: [zMaterial])
            zAxis.name = "zAxis"
            zAxis.transform.rotation = simd_quatf(angle: .pi / 2, axis: [1, 0, 0])  // Rotate to point along +Z
            zAxis.position = [0, 0, axisLength / 2]
            markerContainer.addChild(zAxis)
            
            // Boundary edges (in the XY plane at Z=0)
            let boundaryMesh = MeshResource.generateBox(width: 0.1, height: boundaryWidth, depth: boundaryWidth)
            
            for j in 0..<4 {
                let edge = ModelEntity(mesh: boundaryMesh, materials: [boundaryMaterial])
                edge.name = "boundary_\(j)"
                markerContainer.addChild(edge)
            }
            
            // Text label entity - positioned along Z axis (pointing out of marker)
            let labelEntity = Entity()
            labelEntity.name = "label"
            labelEntity.position = [0, 0, 0.05]  // 5cm out from marker surface (along Z)
            markerContainer.addChild(labelEntity)
            
            // Attach the SwiftUI label to the label entity
            if let labelAttachment = attachments.entity(for: "markerLabel_\(i)") {
                labelAttachment.setParent(labelEntity)
                // Add billboard component so label always faces the user
                labelAttachment.components.set(BillboardComponent())
            }
        }
        
        // === CALIBRATION MARKER VISUALIZATION (purple color scheme) ===
        let calibrationMarkerRoot = Entity()
        calibrationMarkerRoot.name = "calibrationMarkerRoot"
        content.add(calibrationMarkerRoot)
        
        // Purple materials for calibration markers
        var calXMaterial = UnlitMaterial()
        calXMaterial.color = .init(tint: .red)
        var calYMaterial = UnlitMaterial()
        calYMaterial.color = .init(tint: .green)
        var calZMaterial = UnlitMaterial()
        calZMaterial.color = .init(tint: .blue)
        var calOriginMaterial = UnlitMaterial()
        calOriginMaterial.color = .init(tint: .white)
        var calBoundaryMaterial = UnlitMaterial()
        calBoundaryMaterial.color = .init(tint: UIColor(red: 0.7, green: 0.3, blue: 0.9, alpha: 0.9))  // Purple
        
        // Wall material - semi-transparent purple for visibility when visionOS hides passthrough
        var calWallMaterial = SimpleMaterial()
        calWallMaterial.color = .init(tint: UIColor(red: 0.5, green: 0.2, blue: 0.7, alpha: 0.4))
        calWallMaterial.metallic = 0.0
        calWallMaterial.roughness = 1.0
        
        let wallHeight: Float = 0.15  // 15cm wall height
        
        for i in 0..<10 {
            let calMarkerContainer = Entity()
            calMarkerContainer.name = "calMarker_\(i)"
            calMarkerContainer.isEnabled = false
            calibrationMarkerRoot.addChild(calMarkerContainer)
            
            // Origin sphere
            let calOrigin = ModelEntity(mesh: originMesh, materials: [calOriginMaterial])
            calOrigin.name = "origin"
            calMarkerContainer.addChild(calOrigin)
            
            // Axes
            let calXAxis = ModelEntity(mesh: axisMesh, materials: [calXMaterial])
            calXAxis.name = "xAxis"
            calXAxis.transform.rotation = simd_quatf(angle: -.pi / 2, axis: [0, 0, 1])
            calXAxis.position = [axisLength / 2, 0, 0]
            calMarkerContainer.addChild(calXAxis)
            
            let calYAxis = ModelEntity(mesh: axisMesh, materials: [calYMaterial])
            calYAxis.name = "yAxis"
            calYAxis.position = [0, axisLength / 2, 0]
            calMarkerContainer.addChild(calYAxis)
            
            let calZAxis = ModelEntity(mesh: axisMesh, materials: [calZMaterial])
            calZAxis.name = "zAxis"
            calZAxis.transform.rotation = simd_quatf(angle: .pi / 2, axis: [1, 0, 0])
            calZAxis.position = [0, 0, axisLength / 2]
            calMarkerContainer.addChild(calZAxis)
            
            // Boundary
            let calBoundaryMesh = MeshResource.generateBox(width: 0.1, height: boundaryWidth, depth: boundaryWidth)
            for j in 0..<4 {
                let calEdge = ModelEntity(mesh: calBoundaryMesh, materials: [calBoundaryMaterial])
                calEdge.name = "boundary_\(j)"
                calMarkerContainer.addChild(calEdge)
            }
            
            // Wall - vertical plane extending upward from marker surface (15cm tall)
            // Uses placeholder size, will be updated with actual marker size at runtime
            // Wall extends in Z direction (normal to marker) which corresponds to "up" in the visualiztion
            let wallMesh = MeshResource.generateBox(width: 0.1, height: 0.002, depth: wallHeight)
            let wall = ModelEntity(mesh: wallMesh, materials: [calWallMaterial])
            wall.name = "wall"
            // Position: centered on marker X, half wall height up in Z direction (marker-local)
            wall.position = [0, 0, wallHeight / 2]
            calMarkerContainer.addChild(wall)
            
            // Label
            let calLabelEntity = Entity()
            calLabelEntity.name = "label"
            calLabelEntity.position = [0, 0, 0.05]
            calMarkerContainer.addChild(calLabelEntity)
            
            if let calLabelAttachment = attachments.entity(for: "calMarkerLabel_\(i)") {
                calLabelAttachment.setParent(calLabelEntity)
                // Add billboard component so label always faces the user
                calLabelAttachment.components.set(BillboardComponent())
            }
        }
                    
        // === STATUS DISPLAY ===
        let statusAnchor = AnchorEntity(.head)
        statusAnchor.name = "statusHeadAnchor"
        content.add(statusAnchor)
        
        let statusContainer = Entity()
        statusContainer.name = "statusContainer"
        statusContainer.setParent(statusAnchor)
        statusContainer.transform.translation = SIMD3<Float>(0.0, 0.0, -1.0)
        
        if let statusAttachment = attachments.entity(for: "status") {
            dlog("🟢 [CombinedStreamingView] Status attachment found and attached")
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
        
        // === VERIFICATION MARKER VISUALIZATION (Green) ===
        let verificationMarkerRoot = Entity()
        verificationMarkerRoot.name = "verificationMarkerRoot"
        content.add(verificationMarkerRoot)
        
        let verificationMarker = Entity()
        verificationMarker.name = "verificationMarker"
        verificationMarker.isEnabled = false
        verificationMarkerRoot.addChild(verificationMarker)
        
        // Green material for verification
        var verMaterial = UnlitMaterial()
        verMaterial.color = .init(tint: .green)
        var verBoundaryMaterial = UnlitMaterial()
        verBoundaryMaterial.color = .init(tint: UIColor(red: 0.0, green: 1.0, blue: 0.0, alpha: 0.8)) // Bright green
        
        // Origin
        let verOrigin = ModelEntity(mesh: originMesh, materials: [verMaterial])
        verOrigin.name = "origin"
        verificationMarker.addChild(verOrigin)
        
        // Axes (same colors as others for consistency)
        let verXAxis = ModelEntity(mesh: axisMesh, materials: [xMaterial])
        verXAxis.transform.rotation = simd_quatf(angle: -.pi / 2, axis: [0, 0, 1])
        verXAxis.position = [axisLength / 2, 0, 0]
        verificationMarker.addChild(verXAxis)
        
        let verYAxis = ModelEntity(mesh: axisMesh, materials: [yMaterial])
        verYAxis.position = [0, axisLength / 2, 0]
        verificationMarker.addChild(verYAxis)
        
        let verZAxis = ModelEntity(mesh: axisMesh, materials: [zMaterial])
        verZAxis.transform.rotation = simd_quatf(angle: .pi / 2, axis: [1, 0, 0])
        verZAxis.position = [0, 0, axisLength / 2]
        verificationMarker.addChild(verZAxis)
        
        // Boundary
        let verBoundaryMesh = MeshResource.generateBox(width: 0.1, height: boundaryWidth, depth: boundaryWidth)
        for j in 0..<4 {
            let edge = ModelEntity(mesh: verBoundaryMesh, materials: [verBoundaryMaterial])
            edge.name = "boundary_\(j)"
            verificationMarker.addChild(edge)
        }
        
        // === ACCESSORY TRACKING (visionOS 26+) ===
        // Set the root entity for AccessoryTrackingManager so it can add anchor entities
        if #available(visionOS 26.0, *) {
            // Create a dedicated root for accessory anchors
            let accessoryRoot = Entity()
            accessoryRoot.name = "accessoryRoot"
            content.add(accessoryRoot)
            
            // Set the root entity on the manager - visualization is created when stylus connects
            AccessoryTrackingManager.shared.rootEntity = accessoryRoot
        }
    }
    
    // MARK: - Auto-Minimize Logic
    
    /// Check if all required streams are ready and auto-minimize if conditions are met
    private func tryAutoMinimize() {
        guard !hasAutoMinimized && !userInteracted else { return }
        
        let dm = DataManager.shared
        
        // Simple logic:
        // - If sim is enabled: need USDZ loaded in RealityKit AND sim poses coming in
        // - If video is enabled: need frames coming in
        // - If audio is enabled: need audio coming in (or frames if video+audio)
        
        let simReady = !dm.simEnabled || (hasSimPoses && dm.usdzSceneLoaded)
        let videoReady = !dm.videoEnabled || hasFrames
        let audioReady = !dm.audioEnabled || hasAudio || (dm.videoEnabled && hasFrames)
        
        let allReady = simReady && videoReady && audioReady
        let somethingEnabled = dm.simEnabled || dm.videoEnabled || dm.audioEnabled
        
        if allReady && somethingEnabled {
            dlog("✅ [AutoMinimize] Ready - minimizing after delay")
            Task { @MainActor in
                try? await Task.sleep(nanoseconds: 1_000_000_000)  // 1 second delay
                // Re-check conditions after delay
                let stillReady = (!dm.simEnabled || (hasSimPoses && dm.usdzSceneLoaded)) &&
                                 (!dm.videoEnabled || hasFrames) &&
                                 (!dm.audioEnabled || hasAudio || (dm.videoEnabled && hasFrames))
                if !hasAutoMinimized && !userInteracted && stillReady {
                    withAnimation(.spring(response: 0.6, dampingFraction: 0.8)) {
                        isMinimized = true
                        hasAutoMinimized = true
                    }
                }
            }
        }
    }
    
    // MARK: - MuJoCo Model Loading
    
    private func loadMuJoCoModel(from url: URL) async {
        do {
            dlog("📦 [MuJoCo] Loading USDZ from \(url.absoluteString)")
            
            // Clear previous state
            mujocoEntity = nil
            mujocoBodyEntities.removeAll()
            initialLocalTransforms.removeAll()
            pythonToSwiftNameMap.removeAll()
            nameMappingInitialized = false
            DataManager.shared.usdzSceneLoaded = false
            
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
            dlog("✅ [MuJoCo] Model loaded with \(mujocoBodyEntities.count) bodies")
            
            // Mark USDZ as fully loaded in RealityKit
            await MainActor.run {
                DataManager.shared.usdzSceneLoaded = true
                tryAutoMinimize()  // Now check if we can auto-minimize
            }
            
        } catch {
            dlog("❌ [MuJoCo] Failed to load USDZ: \(error)")
        }
    }
    
    private func indexMuJoCoBodyEntities(_ rootEntity: Entity) {
        mujocoBodyEntities.removeAll()
        initialLocalTransforms.removeAll()
        entityPathByObjectID.removeAll()
        pythonToSwiftTargets.removeAll()
        
        dlog("🔍 [indexMuJoCoBodyEntities] Starting recursive indexing...")
        
        // Detect Isaac Lab mode for special orphan handling
        let isIsaacLabMode = mujocoUsdzURL?.lowercased().contains("isaac_") ?? false
        
        func indexRec(_ entity: Entity, parentPath: String) {
            if entity.name.isEmpty {
                for child in entity.children {
                    indexRec(child, parentPath: parentPath)
                }
                return
            }
            
            // For Isaac Lab: Hide orphan entities at root level during indexing
            // These are artifacts from USD de-instancing that appear as duplicates
            if isIsaacLabMode && parentPath.isEmpty {
                let entityName = entity.name
                // Skip and hide orphan prototypes (mesh_0, visuals, __Prototype*, etc.)
                if entityName == "mesh_0" || entityName == "visuals" || entityName.hasPrefix("__Prototype") {
                    entity.isEnabled = false
                    dlog("🚫 [indexMuJoCoBodyEntities] Hiding orphan: '\(entityName)'")
                    // Still traverse children in case there are valid entities underneath
                    for child in entity.children {
                        indexRec(child, parentPath: "")  // Continue with empty parent path
                    }
                    return  // Don't add THIS entity to dictionary
                }
            }
            
            if let modelEntity = entity as? ModelEntity {
                let pathKey = parentPath.isEmpty ? modelEntity.name : "\(parentPath)/\(modelEntity.name)"
                mujocoBodyEntities[pathKey] = modelEntity
                initialLocalTransforms[pathKey] = modelEntity.transform
                entityPathByObjectID[ObjectIdentifier(modelEntity)] = pathKey
                
                dlog("📎 Indexed ModelEntity: '\(pathKey)'")
                
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
                
                dlog("📎 Indexed Wrapper: '\(pathKey)'")
                
                for child in originalChildren {
                    indexRec(child, parentPath: pathKey)
                }
            }
        }
        
        indexRec(rootEntity, parentPath: "")
        
        // Hide orphan prototype entities (artifacts from USD de-instancing)
        // These are entities like "visuals" at root level that aren't assigned to any Python body
        let isIsaacLab = mujocoUsdzURL?.lowercased().contains("isaac_") ?? false
        if isIsaacLab {
            // Collect all Python body names that will receive pose updates
            // These are paths with format: env_X/Robot/link_name
            var validBodyPaths: Set<String> = []
            for pathKey in mujocoBodyEntities.keys {
                // Count path depth: env_0=1, env_0/G1=2, env_0/G1/pelvis=3
                let depth = pathKey.components(separatedBy: "/").count
                if depth >= 3 {  // Actual body links have 3+ components
                    validBodyPaths.insert(pathKey)
                }
            }
            
            for (pathKey, entity) in mujocoBodyEntities {
                // Orphan prototypes at root level (no "/" in path)
                if !pathKey.contains("/") && (pathKey == "visuals" || pathKey == "mesh_0" || pathKey.hasPrefix("__Prototype")) {
                    entity.isEnabled = false
                    dlog("🚫 [indexMuJoCoBodyEntities] Hiding orphan prototype: '\(pathKey)'")
                }
                
                // Robot root entities (env_X/RobotName) - these have duplicate geometry that won't move
                // because pose streaming only updates child links (env_X/RobotName/link_name)
                let components = pathKey.components(separatedBy: "/")
                if components.count == 2 && components[0].hasPrefix("env_") {
                    // This is a robot root like "env_0/G1" - hide it if it has geometry
                    // The child links will be visible and receive pose updates
                    if let modelEntity = entity as? ModelEntity, modelEntity.model != nil {
                        entity.isEnabled = false
                        dlog("🚫 [indexMuJoCoBodyEntities] Hiding robot root with geometry: '\(pathKey)'")
                    }
                }
            }
        }
        
        dlog("📝 Indexed \(mujocoBodyEntities.count) entities")
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
        
        // Detect simulator type from filename (Python sends "isaac_scene.usdz" for Isaac Lab)
        let isIsaacLab = mujocoUsdzURL?.lowercased().contains("isaac_") ?? false
        
        var result: [String] = []
        
        if isIsaacLab {
            // === ISAAC LAB: Find ALL visual children with valid transforms ===
            // Try /visuals first (for imported USD assets like DexCube)
            let visualsPrefix = base + "/visuals"
            var visualCandidates: [String] = []
            for swiftName in mujocoBodyEntities.keys {
                if swiftName.hasPrefix(visualsPrefix) && !swiftName.contains("/collisions") && initialLocalTransforms[swiftName] != nil {
                    visualCandidates.append(swiftName)
                }
            }
            // Sort by depth and add the shallowest visual children
            visualCandidates.sort { $0.count < $1.count }
            if !visualCandidates.isEmpty {
                // Only add the shallowest level (all at same depth)
                let shallowestDepth = visualCandidates.first!.components(separatedBy: "/").count
                for candidate in visualCandidates {
                    if candidate.components(separatedBy: "/").count == shallowestDepth {
                        result.append(candidate)
                    }
                }
            }
            
            // If no /visuals found, try direct children (for procedural assets like CuboidCfg)
            // Include ALL direct children, not just the first one (some links have multiple meshes)
            if result.isEmpty {
                let childPrefix = base + "/"
                var childCandidates: [String] = []
                for swiftName in mujocoBodyEntities.keys {
                    if swiftName.hasPrefix(childPrefix) &&
                       !swiftName.contains("/collisions") &&
                       initialLocalTransforms[swiftName] != nil {
                        // Only consider direct children (depth 1)
                        let suffix = String(swiftName.dropFirst(childPrefix.count))
                        let depth = suffix.components(separatedBy: "/").count
                        if depth == 1 && !suffix.contains("/") {  // Direct child only
                            childCandidates.append(swiftName)
                        }
                    }
                }
                // Add ALL direct children (some links have both Cube and mesh_0)
                for child in childCandidates {
                    result.append(child)
                }
                if result.count > 0 {
                    dlog("🔍 [targets] Found \(result.count) visual children for '\(pyName)': \(result)")
                }
            }
        } else {
            // === MUJOCO: Simple hierarchy - use original leaf finding ===
            func findLeaves(_ entity: Entity) {
                if let model = entity as? ModelEntity {
                    let hasModelChildren = entity.children.contains { $0 is ModelEntity }
                    if !hasModelChildren, let key = entityPathByObjectID[ObjectIdentifier(model)] {
                        result.append(key)
                        return
                    }
                }
                for child in entity.children {
                    findLeaves(child)
                }
            }
            
            for child in baseEntity.children {
                findLeaves(child)
            }
        }
        
        // If no leaf targets found, use the base entity itself
        if result.isEmpty {
            result = [base]
            dlog("⚠️ [targets] No children found for '\(pyName)', using base: '\(base)'")
        }
        
        pythonToSwiftTargets[pyName] = result
        if result.count > 1 || result.first != base {
            dlog("🎯 [targets] '\(pyName)' → [\(result.joined(separator: ", "))]")
        }
        return result
    }
    
    /// Compute final transform for a specific target entity
    /// Set skipLocalTransform=true for Isaac Lab entities that shouldn't have local offset applied
    private func computeFinalTransformForTarget(bodyName: String, values: [Float], axisCorrection: simd_quatf, skipLocalTransform: Bool = false) -> simd_float4x4 {
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
        
        // Apply initial local transform from USDZ import (skip for Isaac Lab visual entities)
        if skipLocalTransform {
            return rkWorldTransform
        } else {
            let yLocal = initialLocalTransforms[bodyName]?.matrix ?? matrix_identity_float4x4
            return rkWorldTransform * yLocal
        }
    }
    
    /// Compute final transforms from WebRTC JSON format
    /// Input format: {"body_name": [x, y, z, qx, qy, qz, qw], ...}
    private func computeMuJoCoFinalTransformsFromWebRTC(_ poses: [String: [Float]]) -> [String: simd_float4x4] {
        guard !mujocoBodyEntities.isEmpty, !initialLocalTransforms.isEmpty else {
            return [:]
        }
        
        if !nameMappingInitialized {
            dlog("🔤 [WebRTC poses] Initializing name mapping...")
            dlog("   Python names: \(Array(poses.keys).sorted())")
            dlog("   Swift names: \(Array(mujocoBodyEntities.keys).sorted())")
            initializeMuJoCoNameMapping(pythonNames: Array(poses.keys))
            let validMatches = poses.keys.reduce(0) { acc, py in
                if let swift = pythonToSwiftNameMap[py], mujocoBodyEntities[swift] != nil { return acc + 1 }
                return acc
            }
            if validMatches > 0 {
                nameMappingInitialized = true
                // Cache sorted body names for consistent iteration order
                cachedSortedBodyNames = Array(poses.keys).sorted()
                dlog("✅ Name mapping initialized with \(validMatches) valid matches, cached \(cachedSortedBodyNames.count) sorted names")
            }
        }
        
        // Detect Isaac Lab mode from USDZ filename (cache this check)
        let isIsaacLab = mujocoUsdzURL?.lowercased().contains("isaac_") ?? false
        
        // Pre-compute axis correction (constant)
        let axisCorrection = simd_quatf(angle: -.pi / 2, axis: SIMD3<Float>(1, 0, 0))
        
        // Pre-allocate with estimated capacity to reduce allocations
        var finalTransforms: [String: simd_float4x4] = [:]
        finalTransforms.reserveCapacity(poses.count * 2)  // Typically 1-2 targets per pose
        
        // Use cached sorted order for consistent iteration (O(1) vs O(n log n) per frame)
        // Fall back to unsorted if cache is empty (shouldn't happen after initialization)
        let bodyNamesToProcess = cachedSortedBodyNames.isEmpty ? Array(poses.keys) : cachedSortedBodyNames
        
        for pyName in bodyNamesToProcess {
            guard let values = poses[pyName], values.count >= 7 else { continue }
            
            // Get all target entities for this Python body (cached after first call)
            let targetNames = targets(for: pyName)
            
            for bodyName in targetNames {
                guard initialLocalTransforms[bodyName] != nil else { continue }
                
                // For Isaac Lab: Determine if this is a standalone rigid object or an articulated link
                let pathComponents = pyName.components(separatedBy: "/")
                let isStandaloneRigidObject = isIsaacLab && pathComponents.count == 2
                
                let transform = computeFinalTransformForTarget(
                    bodyName: bodyName, 
                    values: values, 
                    axisCorrection: axisCorrection,
                    skipLocalTransform: isStandaloneRigidObject
                )
                finalTransforms[bodyName] = transform
            }
        }
        
        return finalTransforms
    }
    
    private func applyMuJoCoTransforms(_ transforms: [String: simd_float4x4]) {
        // Direct application without depth sorting - world transforms are independent
        for (name, desiredWorld) in transforms {
            guard let entity = mujocoBodyEntities[name] else { continue }
            
            let prevLocalScale = entity.scale
            entity.setTransformMatrix(desiredWorld, relativeTo: nil)
            entity.scale = prevLocalScale  // Preserve scale
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
    
    /// Updates a bone entity (cylinder) to connect two joint positions
    /// Uses scale on Y-axis to adjust length (mesh is created with height=1)
    /// Optimized version with simplified quaternion calculation
    private func updateBoneEntity(_ entity: ModelEntity, from start: SIMD3<Float>, to end: SIMD3<Float>) {
        let direction = end - start
        let length = simd_length(direction)
        
        // Skip if joints are too close or invalid
        guard length > 0.001 else {
            entity.isEnabled = false
            return
        }
        
        // Position at midpoint
        entity.position = (start + end) * 0.5
        
        // Scale Y to match bone length (mesh has height=1)
        entity.scale = SIMD3<Float>(1.0, length, 1.0)
        
        // Rotate to align with direction
        // Cylinder is created along Y-axis, so we need to rotate it to align with the bone direction
        let normalizedDirection = simd_normalize(direction)
        let yAxis = SIMD3<Float>(0, 1, 0)
        
        // Calculate rotation from Y-axis to the bone direction
        let dot = simd_dot(yAxis, normalizedDirection)
        if dot > 0.9999 {
            // Already aligned
            entity.orientation = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        } else if dot < -0.9999 {
            // Opposite direction, rotate 180° around X
            entity.orientation = simd_quatf(angle: .pi, axis: SIMD3<Float>(1, 0, 0))
        } else {
            // General case: rotate around the cross product axis
            let axis = simd_normalize(simd_cross(yAxis, normalizedDirection))
            let angle = acos(dot)
            entity.orientation = simd_quatf(angle: angle, axis: axis)
        }
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

/// Creates a "light beam" ray that extends from the head anchor towards -Z axis
private func createHeadBeam() -> Entity {
    let beamEntity = Entity()
    
    // Create a very long, thin cylinder to represent the beam
    // The cylinder is oriented along Y-axis by default, so we'll rotate it to point along -Z
    let beamLength: Float = 100.0  // 100 meters long (effectively infinite)
    let beamRadius: Float = 0.002  // 2mm radius for a thin laser-like appearance
    
    let cylinderMesh = MeshResource.generateCylinder(height: beamLength, radius: beamRadius)
    
    // Create a glowing yellow/orange material for the beam
    var beamMaterial = UnlitMaterial()
    beamMaterial.color = .init(tint: UIColor(red: 1.0, green: 0.8, blue: 0.2, alpha: 0.8))  // Golden yellow
    
    let beamModelEntity = ModelEntity(mesh: cylinderMesh, materials: [beamMaterial])
    
    // Rotate the cylinder 90 degrees around X-axis so it points along -Z instead of Y
    beamModelEntity.transform.rotation = simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(1, 0, 0))
    
    // Offset the beam so it starts 0.3m (30cm) away from head and extends towards -Z
    // Also offset by -0.1m on both X and Y axes
    // Since the cylinder is centered, we offset by half its length plus the start distance
    let startOffset: Float = 0.3  // 30cm away from head anchor
    beamModelEntity.transform.translation = SIMD3<Float>(-0.0, -0.05, -(beamLength / 2 + startOffset))
    
    beamEntity.addChild(beamModelEntity)
    
    return beamEntity
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
        dlog("🚀 [CombinedMuJoCoManager] Starting gRPC server on port \(grpcPort)...")
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
            dlog("✅ [CombinedMuJoCoManager] gRPC server started on port \(grpcPort)")
            
            try await server.serve()
            
        } catch {
            dlog("❌ [CombinedMuJoCoManager] Failed to start server: \(error)")
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
            dlog("🛑 [CombinedMuJoCoManager] gRPC server stopped")
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
        dlog("🎯 [CombinedMuJoCoARServiceImpl] Initialized")
    }
    
    func sendUsdzUrl(
        request: MujocoAr_UsdzUrlRequest,
        context: ServerContext
    ) async throws -> MujocoAr_UsdzUrlResponse {
        dlog("📨 [sendUsdzUrl] URL: \(request.usdzURL)")
        
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
        dlog("📨 [sendUsdzData] Received \(request.usdzData.count) bytes")
        
        var response = MujocoAr_UsdzDataResponse()
        
        do {
            let tempDir = FileManager.default.temporaryDirectory
            let fileName = request.filename.isEmpty ? "\(UUID().uuidString).usdz" : request.filename
            let localURL = tempDir.appendingPathComponent(fileName)
            
            try request.usdzData.write(to: localURL)
            dlog("💾 [sendUsdzData] Saved to: \(localURL.path)")
            
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
            dlog("❌ [sendUsdzData] Error: \(error)")
            response.success = false
            response.message = "Failed: \(error.localizedDescription)"
        }
        
        return response
    }
    
    func sendUsdzDataChunked(
        request: RPCAsyncSequence<MujocoAr_UsdzChunkRequest, any Error>,
        context: ServerContext
    ) async throws -> MujocoAr_UsdzDataResponse {
        dlog("📦 [sendUsdzDataChunked] Starting chunked transfer")
        
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
            dlog("💾 [sendUsdzDataChunked] Saved \(chunkData.count) bytes to: \(localURL.path)")
            
            await MainActor.run {
                manager?.onUsdzReceived?(localURL.absoluteString, attachToPosition, attachToRotation)
                manager?.updateConnectionStatus("Client Connected - USDZ Received (\(receivedChunks) chunks)")
            }
            
            response.success = true
            response.message = "Chunked USDZ received (\(receivedChunks) chunks)"
            response.localFilePath = localURL.path
            
        } catch {
            dlog("❌ [sendUsdzDataChunked] Error: \(error)")
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
        dlog("🔄 [streamPoses] Starting pose stream")
        
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
            dlog("❌ [streamPoses] Error: \(error)")
            throw error
        }
        
        dlog("🔄 [streamPoses] Stream ended")
    }
    
    func streamHandTracking(
        request: MujocoAr_HandTrackingRequest,
        response: RPCWriter<MujocoAr_HandTrackingUpdate>,
        context: ServerContext
    ) async throws {
        dlog("🖐️ [streamHandTracking] Client connected")
        
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
            dlog("❌ [streamHandTracking] Error: \(error)")
            throw error
        }
        
        dlog("🖐️ [streamHandTracking] Client disconnected")
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

// MARK: - View Modifiers for CombinedStreamingView

/// Video source related modifiers
private struct VideoSourceModifiers: ViewModifier {
    @ObservedObject var dataManager: DataManager
    @ObservedObject var uvcCameraManager: UVCCameraManager
    @ObservedObject var imageData: ImageData
    @Binding var uvcFrame: UIImage?
    @Binding var updateTrigger: Bool
    @Binding var hasFrames: Bool
    @Binding var currentAspectRatio: Float?
    
    func body(content: Content) -> some View {
        content
            .onReceive(imageData.$left) { _ in
                if dataManager.videoSource == .network {
                    updateTrigger.toggle()
                }
            }
            .onReceive(uvcCameraManager.$currentFrame) { frame in
                if dataManager.videoSource == .uvcCamera {
                    uvcFrame = frame
                    updateTrigger.toggle()
                }
            }
            .onChange(of: dataManager.videoSource) { oldValue, newValue in
                handleVideoSourceChange(oldValue: oldValue, newValue: newValue)
            }
            .onChange(of: uvcCameraManager.availableDevices) { oldDevices, newDevices in
                handleDevicesChange(oldDevices: oldDevices, newDevices: newDevices)
            }
            .onChange(of: uvcCameraManager.selectedDevice) { oldDevice, newDevice in
                handleDeviceSelection(oldDevice: oldDevice, newDevice: newDevice)
            }
    }
    
    private func handleVideoSourceChange(oldValue: VideoSource, newValue: VideoSource) {
        dlog("📹 [CombinedStreamingView] Video source changed from \(oldValue.rawValue) to \(newValue.rawValue)")
        hasFrames = false
        currentAspectRatio = nil
        
        if newValue == .uvcCamera {
            if uvcCameraManager.selectedDevice != nil {
                dlog("📹 [CombinedStreamingView] Starting UVC capture with selected device: \(uvcCameraManager.selectedDevice?.name ?? "unknown")")
                uvcCameraManager.startCapture()
            } else if let firstDevice = uvcCameraManager.availableDevices.first {
                dlog("📹 [CombinedStreamingView] Selecting first available device: \(firstDevice.name)")
                uvcCameraManager.selectDevice(firstDevice)
                Task {
                    try? await Task.sleep(nanoseconds: 200_000_000)
                    await MainActor.run { uvcCameraManager.startCapture() }
                }
            } else {
                dlog("📹 [CombinedStreamingView] No UVC devices available")
            }
        } else {
            uvcCameraManager.stopCapture()
            uvcFrame = nil
        }
        updateTrigger.toggle()
    }
    
    private func handleDevicesChange(oldDevices: [UVCDevice], newDevices: [UVCDevice]) {
        if dataManager.videoSource == .uvcCamera && !newDevices.isEmpty && oldDevices.isEmpty {
            dlog("📹 [CombinedStreamingView] UVC device connected while in UVC mode, auto-starting capture")
            if let firstDevice = newDevices.first {
                uvcCameraManager.selectDevice(firstDevice)
                Task {
                    try? await Task.sleep(nanoseconds: 200_000_000)
                    await MainActor.run { uvcCameraManager.startCapture() }
                }
            }
        }
    }
    
    private func handleDeviceSelection(oldDevice: UVCDevice?, newDevice: UVCDevice?) {
        if dataManager.videoSource == .uvcCamera && newDevice != nil && !uvcCameraManager.isCapturing {
            dlog("📹 [CombinedStreamingView] UVC device selected, starting capture")
            Task {
                try? await Task.sleep(nanoseconds: 100_000_000)
                await MainActor.run { uvcCameraManager.startCapture() }
            }
        }
    }
}

/// Lifecycle modifiers (task, onAppear)
private struct LifecycleModifiers: ViewModifier {
    @ObservedObject var dataManager: DataManager
    @ObservedObject var appModel: 🥽AppModel
    @ObservedObject var videoStreamManager: VideoStreamManager
    @ObservedObject var uvcCameraManager: UVCCameraManager
    @ObservedObject var mujocoManager: CombinedMuJoCoManager
    @ObservedObject var recordingManager: RecordingManager
    @ObservedObject var imageData: ImageData
    @Binding var hasAutoMinimized: Bool
    @Binding var userInteracted: Bool
    @Binding var hasFrames: Bool
    @Binding var hasAudio: Bool
    @Binding var hasSimPoses: Bool
    @Binding var stereoMaterialEntity: Entity?
    @Binding var attachToPosition: SIMD3<Float>?
    @Binding var attachToRotation: simd_quatf?
    @Binding var mujocoUsdzURL: String?
    @Binding var mujocoFinalTransforms: [String: simd_float4x4]
    @Binding var mujocoPoseUpdateTrigger: UUID
    @Binding var mujocoBodyEntities: [String: ModelEntity]
    
    var computeMuJoCoFinalTransformsFromWebRTC: ([String: [Float]]) -> [String: simd_float4x4]
    var computeMuJoCoFinalTransforms: ([String: MujocoAr_BodyPose]) -> [String: simd_float4x4]
    var tryAutoMinimize: () -> Void
    
    func body(content: Content) -> some View {
        content
            .task { appModel.run() }
            .task { await appModel.processDeviceAnchorUpdates() }
            .task(priority: .low) { await appModel.processReconstructionUpdates() }
            .onAppear { handleOnAppear() }
    }
    
    private func handleOnAppear() {
        dlog("🚀 [CombinedStreamingView] View appeared, starting services")
        
        hasAutoMinimized = false
        userInteracted = false
        hasFrames = false
        hasAudio = false
        hasSimPoses = false
        dlog("🔄 [CombinedStreamingView] Reset auto-minimize state: hasAutoMinimized=\(hasAutoMinimized), userInteracted=\(userInteracted)")
        
        setupTeleoperationMode()
        
        loadStereoMaterial()
        setupMuJoCoCallbacks()
    }
    
    private func setupTeleoperationMode() {
        dlog("🤖 [CombinedStreamingView] Teleoperation Mode - Full WebRTC support")
        
        videoStreamManager.onSimPosesReceived = { timestamp, poses, qpos, ctrl in
            // Record simulation data on background thread to avoid blocking main thread
            DispatchQueue.global(qos: .utility).async {
                RecordingManager.shared.recordSimulationData(
                    timestamp: timestamp,
                    poses: poses,
                    qpos: qpos,
                    ctrl: ctrl,
                    trackingData: DataManager.shared.latestHandTrackingData
                )
            }
            
            // Compute and apply transforms directly (already on main thread from WebRTCClient)
            // This ensures all bodies get the same frame's transforms atomically
            let transforms = computeMuJoCoFinalTransformsFromWebRTC(poses)
            
            // Apply transforms in consistent sorted order to prevent visual artifacts
            // The sorted keys are cheap since transforms dict uses String keys which sort fast
            for name in transforms.keys.sorted() {
                guard let desiredWorld = transforms[name], let entity = mujocoBodyEntities[name] else { continue }
                entity.setTransformMatrix(desiredWorld, relativeTo: nil)
            }
            
            // Update state for UI stats (no dispatch needed, already on main)
            if mujocoManager.bodyCount == 0 {
                mujocoPoseUpdateTrigger = UUID()
            }
            
            mujocoManager.recordPoseUpdate(bodyCount: poses.count)
            
            if !mujocoManager.poseStreamingViaWebRTC {
                mujocoManager.poseStreamingViaWebRTC = true
                mujocoManager.simEnabled = true
                mujocoManager.updateConnectionStatus("Streaming via WebRTC")
                dlog("✅ [Callback] First pose received")
            }
            
            if !hasSimPoses {
                hasSimPoses = true
                tryAutoMinimize()
            }
        }
        
        dlog("📝 [setupTeleoperationMode] onSimPosesReceived callback registered")
        
        videoStreamManager.start(imageData: imageData)
        
        Task { await mujocoManager.startServer() }
        
        if dataManager.videoSource == .uvcCamera {
            dlog("📹 [CombinedStreamingView] UVC mode active on appear, initializing camera")
            startUVCCapture()
        }
    }
    
    private func startUVCCapture() {
        Task {
            let granted = await uvcCameraManager.requestCameraAccess()
            if granted {
                await MainActor.run {
                    if uvcCameraManager.selectedDevice != nil {
                        dlog("📹 [CombinedStreamingView] Starting capture with existing device")
                        uvcCameraManager.startCapture()
                    } else if let firstDevice = uvcCameraManager.availableDevices.first {
                        dlog("📹 [CombinedStreamingView] Selecting and starting first device: \(firstDevice.name)")
                        uvcCameraManager.selectDevice(firstDevice)
                    }
                }
                try? await Task.sleep(nanoseconds: 300_000_000)
                await MainActor.run {
                    if !uvcCameraManager.isCapturing && uvcCameraManager.selectedDevice != nil {
                        uvcCameraManager.startCapture()
                    }
                }
            }
        }
    }
    
    private func loadStereoMaterial() {
        Task {
            if let scene = try? await Entity(named: "Immersive", in: realityKitContentBundle) {
                if let sphereEntity = scene.findEntity(named: "Sphere") {
                    sphereEntity.isEnabled = false
                    await MainActor.run {
                        stereoMaterialEntity = sphereEntity
                    }
                    dlog("✅ [CombinedStreamingView] Loaded stereo material")
                }
            }
        }
    }
    
    private func setupMuJoCoCallbacks() {
        mujocoManager.onUsdzReceived = { url, position, rotation in
            Task { @MainActor in
                attachToPosition = position
                attachToRotation = rotation
                mujocoUsdzURL = url
                mujocoManager.simEnabled = true
                
                // Register USDZ for recording
                if !url.isEmpty {
                    let fileURL: URL?
                    if url.hasPrefix("file://") {
                        // Already a file URL string, parse it directly
                        fileURL = URL(string: url)
                    } else {
                        // Plain file path, convert to file URL
                        fileURL = URL(fileURLWithPath: url)
                    }
                    if let validURL = fileURL {
                        RecordingManager.shared.setUsdzUrl(validURL)
                    }
                }
            }
        }
        
        mujocoManager.onPosesReceived = { poses in
            if !mujocoManager.poseStreamingViaWebRTC {
                Task { @MainActor in
                    let transforms = computeMuJoCoFinalTransforms(poses)
                    mujocoFinalTransforms = transforms
                    mujocoPoseUpdateTrigger = UUID()
                }
            }
        }
    }
}

/// State change modifiers (onChange, onDisappear)
private struct StateChangeModifiers: ViewModifier {
    @ObservedObject var dataManager: DataManager
    @ObservedObject var videoStreamManager: VideoStreamManager
    @ObservedObject var uvcCameraManager: UVCCameraManager
    @ObservedObject var mujocoManager: CombinedMuJoCoManager
    @ObservedObject var recordingManager: RecordingManager
    @ObservedObject var imageData: ImageData
    @Binding var hasFrames: Bool
    @Binding var hasAudio: Bool
    @Binding var hasSimPoses: Bool
    @Binding var videoMinimized: Bool
    @Binding var hasAutoMinimized: Bool
    @Binding var fixedWorldTransform: Transform?
    @Binding var userInteracted: Bool
    @Binding var mujocoUsdzURL: String?
    @Binding var mujocoEntity: Entity?
    @Binding var mujocoBodyEntities: [String: ModelEntity]
    @Binding var initialLocalTransforms: [String: Transform]
    @Binding var pythonToSwiftNameMap: [String: String]
    @Binding var pythonToSwiftTargets: [String: [String]]
    @Binding var entityPathByObjectID: [ObjectIdentifier: String]
    @Binding var nameMappingInitialized: Bool
    @Binding var mujocoFinalTransforms: [String: simd_float4x4]
    @Binding var attachToPosition: SIMD3<Float>?
    @Binding var attachToRotation: simd_quatf?
    @Binding var mujocoPoseUpdateTrigger: UUID
    
    var loadMuJoCoModel: (URL) async -> Void
    var tryAutoMinimize: () -> Void
    
    func body(content: Content) -> some View {
        content
            .onChange(of: mujocoUsdzURL) { _, newValue in
                if let urlStr = newValue, let url = URL(string: urlStr) {
                    Task { await loadMuJoCoModel(url) }
                }
            }
            .onChange(of: mujocoPoseUpdateTrigger) { _, _ in
                // Trigger is only used to force RealityView update on first pose
                // No clearing needed - transforms are continuously updated via applyMuJoCoTransforms()
            }
            .onChange(of: dataManager.pythonClientIP) { _, newValue in
                if newValue == nil { handlePythonClientDisconnected() }
            }
            .onChange(of: dataManager.webrtcGeneration) { oldValue, newValue in
                handleWebRTCGenerationChange(oldValue: oldValue, newValue: newValue)
            }
            .onChange(of: hasFrames) { oldValue, newValue in
                if newValue && !oldValue {
                    dlog("🎬 [CombinedStreamingView] Video frames arrived, attempting auto-minimize")
                    tryAutoMinimize()
                }
            }
            .onChange(of: dataManager.videoEnabled) { _, newValue in
                dlog("🔧 [CombinedStreamingView] videoEnabled changed to \(newValue)")
                if newValue { tryAutoMinimize() }
            }
            .onChange(of: dataManager.audioEnabled) { _, newValue in
                dlog("🔧 [CombinedStreamingView] audioEnabled changed to \(newValue)")
                if newValue && hasFrames {
                    hasAudio = true
                    tryAutoMinimize()
                }
            }
            .onChange(of: dataManager.simEnabled) { _, newValue in
                dlog("🔧 [CombinedStreamingView] simEnabled changed to \(newValue)")
                if newValue { tryAutoMinimize() }
            }
            .onChange(of: dataManager.videoPlaneFixedToWorld) { _, isFixed in
                handleVideoPlaneFixedChange(isFixed: isFixed)
            }
            .onChange(of: userInteracted) { oldValue, newValue in
                dlog("⚠️ [CombinedStreamingView] userInteracted changed from \(oldValue) to \(newValue)")
                if newValue {
                    Thread.callStackSymbols.prefix(10).forEach { dlog("  \($0)") }
                }
            }
            // Handle disconnection signals (from WebRTCClient/Signaling)
            .onChange(of: dataManager.connectionStatus) { _, status in
                if status == "Peer disconnected" || status.contains("ICE disconnected") || status.contains("ICE failed") || status.contains("ICE closed") {
                    dlog("🔄 [CombinedStreamingView] Disconnection detected ('\(status)'), restarting VideoStreamManager...")
                    
                    // Stop with preserveForReconnect=true to keep WebRTCClient and its signaling callbacks
                    // This is critical for cross-network mode where the same WebRTCClient handles reconnection
                    videoStreamManager.stop(preserveForReconnect: true)
                    
                    // Reset UI state
                    resetStreamingState()
                    
                    // Restart after brief delay to allow cleanup
                    DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
                        videoStreamManager.start(imageData: imageData)
                    }
                }
            }
            // Handle cross-network USDZ loading via WebRTC data channel
            .onChange(of: dataManager.loadedUsdzPath) { _, newPath in
                guard let path = newPath, !path.isEmpty else { return }
                dlog("📦 [CombinedStreamingView] Cross-network USDZ received: \(path)")
                
                // Convert to file URL and trigger the same loading flow as local mode
                let fileURL = URL(fileURLWithPath: path)
                
                // Convert attach position/rotation
                var position: SIMD3<Float>? = nil
                var rotation: simd_quatf? = nil
                
                if let pos = dataManager.loadedUsdzAttachPosition, pos.count >= 3 {
                    position = SIMD3<Float>(pos[0], pos[1], pos[2])
                }
                if let rot = dataManager.loadedUsdzAttachRotation, rot.count >= 4 {
                    // [x, y, z, w] format
                    rotation = simd_quatf(ix: rot[0], iy: rot[1], iz: rot[2], r: rot[3])
                }
                
                // Set state to trigger the existing loading flow
                attachToPosition = position
                attachToRotation = rotation
                mujocoUsdzURL = fileURL.absoluteString
                mujocoManager.simEnabled = true
                
                // Register for recording
                RecordingManager.shared.setUsdzUrl(fileURL)
                
                // Clear the loaded path so we can receive another
                Task { @MainActor in
                    dataManager.loadedUsdzPath = nil
                }
            }
            .onDisappear { handleOnDisappear() }
    }
    
    private func handlePythonClientDisconnected() {
        dlog("🔌 [CombinedStreamingView] Python client disconnected")
        recordingManager.onVideoSourceDisconnected(reason: "Python client disconnected")
        
        // In local mode, when Python disconnects via gRPC, we need to fully close the WebRTC 
        // connection to stop video frames immediately. Otherwise frames keep flowing until ICE timeout.
        videoStreamManager.stop(preserveForReconnect: false)  // Full cleanup
        
        resetStreamingState()
    }
    
    private func handleWebRTCGenerationChange(oldValue: Int, newValue: Int) {
        if newValue < 0 {
            recordingManager.onVideoSourceDisconnected(reason: "WebRTC disconnected")
            // Full cleanup to stop video frames immediately
            videoStreamManager.stop(preserveForReconnect: false)
            resetStreamingState()
        } else if newValue > 0 && oldValue != newValue {
            mujocoManager.poseStreamingViaWebRTC = false
            hasSimPoses = false
            videoStreamManager.stop(preserveForReconnect: true)
            // Capture the generation to check if it's still valid when the delayed block runs
            let expectedGeneration = newValue
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) { [weak dataManager] in
                // Only start if generation hasn't changed (prevents restart during disconnect)
                guard let dm = dataManager, dm.webrtcGeneration == expectedGeneration else {
                    dlog("⚠️ [CombinedStreamingView] Skipping delayed start - generation changed")
                    return
                }
                videoStreamManager.start(imageData: imageData)
            }
        }
    }
    
    private func resetStreamingState() {
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
        videoStreamManager.stop(preserveForReconnect: true)
        
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
    
    private func handleVideoPlaneFixedChange(isFixed: Bool) {
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
    
    private func handleOnDisappear() {
        dlog("🛑 [CombinedStreamingView] View disappeared, stopping services")
        videoStreamManager.stop(preserveForReconnect: false)  // Full cleanup on disappear
        uvcCameraManager.stopCapture()
        fixedWorldTransform = nil
        Task { await mujocoManager.stopServer() }
    }
}
