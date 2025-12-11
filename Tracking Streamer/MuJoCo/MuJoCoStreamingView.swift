import SwiftUI
import RealityKit
import ARKit
import simd
import QuartzCore
import GRPCCore
import GRPCNIOTransportHTTP2
import GRPCProtobuf

// MARK: - MuJoCo Streaming View
/// A view for visualizing MuJoCo simulations streamed from Python.
/// This handles USDZ model loading, pose streaming, and hand tracking.
struct MuJoCoStreamingView: View {
    @StateObject private var networkManager = MuJoCoNetworkManager()
    @StateObject private var grpcManager = MuJoCoGRPCServerManager()
    @StateObject private var appModel = 🥽AppModel()
    
    @State private var entity: ModelEntity?
    @State private var bodyEntities: [String: ModelEntity] = [:]
    @State private var usdzURL: String? = nil
    @State private var initialLocalTransforms: [String: Transform] = [:]
    @State private var pythonToSwiftNameMap: [String: String] = [:]
    @State private var nameMappingInitialized: Bool = false
    @State private var pythonToSwiftTargets: [String: [String]] = [:]
    @State private var entityPathByObjectID: [ObjectIdentifier: String] = [:]
    
    // Status display
    @State private var statusEntity: Entity?
    @State private var statusContainerEntity: Entity?
    @State private var realityContent: RealityViewContent?
    @State private var inputPort: String = "50051"
    @State private var isMinimized: Bool = false
    @Namespace private var minimizeNS
    @ObservedObject private var dataManager = DataManager.shared
    
    // Attachment offset state (ZUP coordinates)
    @State private var attachToPosition: SIMD3<Float>? = nil
    @State private var attachToRotation: simd_quatf? = nil
    
    // Pose update trigger
    @State private var finalTransforms: [String: simd_float4x4] = [:]
    @State private var poseUpdateTrigger: UUID = UUID()
    
    let spatialGen = SpatialGenHelper()
    private let applyInWorldSpace: Bool = true
    
    var body: some View {
        Group {
            RealityView { content, attachments in
                realityContent = content
                createStatusDisplay(content: content, attachments: attachments)
                
                if let entity = entity {
                    content.add(entity)
                    dlog("✅ Added initial entity to RealityView: \(entity.name)")
                }
            } update: { updateContent, updatedAttachments in
                if let newEntity = entity {
                    let existingEntities = updateContent.entities
                    let entityExists = existingEntities.contains { $0.id == newEntity.id }
                    
                    if !entityExists {
                        for existingEntity in existingEntities {
                            if existingEntity.name != "statusHeadAnchor" {
                                updateContent.remove(existingEntity)
                            }
                        }
                        updateContent.add(newEntity)
                        dlog("✅ [RealityView.update] Added new entity: \(newEntity.name)")
                    }
                }
                
                if !finalTransforms.isEmpty {
                    dlog("🔄 [RealityView.update] Applying \(finalTransforms.count) final transforms")
                    applyFinalTransforms(finalTransforms)
                    dlog("✅ [RealityView.update] Applied transforms successfully")
                }
            } attachments: {
                Attachment(id: "status") {
                    statusAttachmentView
                }
            }
            .onAppear {
                MuJoCoStatusDisplayComponent.registerComponent()
                MuJoCoStatusDisplaySystem.registerSystem()
            }
            .onChange(of: isMinimized) { _, _ in
                updateStatusContainerPosition(animated: true)
            }
        }
        .task {
            dlog("🚀 [MuJoCoStreamingView] Starting initialization...")
            networkManager.updateNetworkInfo()
            appModel.run()
        }
        .task {
            await appModel.processDeviceAnchorUpdates()
        }
        .onChange(of: usdzURL) { _, newValue in
            if let urlStr = newValue, let url = URL(string: urlStr) {
                Task {
                    await loadUsdzModel(from: url)
                }
            }
        }
        .onChange(of: poseUpdateTrigger) { _, _ in
            Task {
                try? await Task.sleep(nanoseconds: 1_000_000)
                finalTransforms = [:]
            }
        }
        .onDisappear {
            Task {
                networkManager.updateConnectionStatus("Stopping Server...")
                await grpcManager.stopServer()
                networkManager.updateConnectionStatus("Disconnected")
            }
        }
        .upperLimbVisibility(dataManager.upperLimbVisible ? .visible : .hidden)
    }
    
    // MARK: - Status Attachment View
    @ViewBuilder
    private var statusAttachmentView: some View {
        ZStack(alignment: .topTrailing) {
            if isMinimized {
                minimizedButton
            } else {
                expandedPanel
            }
        }
        .animation(.spring(response: 0.45, dampingFraction: 0.85), value: isMinimized)
    }
    
    private var minimizedButton: some View {
        Button(action: {
            withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                isMinimized = false
            }
        }) {
            ZStack {
                Circle()
                    .fill(.regularMaterial)
                    .frame(width: 44, height: 44)
                    .shadow(radius: 4)
                Image(systemName: "cube.transparent")
                    .foregroundColor(.orange)
            }
        }
        .buttonStyle(.plain)
        .matchedGeometryEffect(id: "statusMiniDot", in: minimizeNS)
        .padding(6)
    }
    
    private var expandedPanel: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Image(systemName: "cube.transparent")
                    .foregroundColor(.orange)
                Text("MuJoCo Streaming")
                    .font(.headline)
                    .foregroundColor(.primary)
                Spacer()
            }
            
            VStack(alignment: .leading, spacing: 8) {
                Text("IP: \(networkManager.ipAddress)")
                    .font(.system(.body, design: .monospaced))
                    .foregroundColor(.primary)
                
                if networkManager.isServerRunning {
                    Text("Port: \(networkManager.grpcPort, format: .number.grouping(.never))")
                        .font(.system(.body, design: .monospaced))
                        .foregroundColor(.primary)
                } else {
                    HStack {
                        Text("Port:")
                            .font(.system(.body, design: .monospaced))
                            .foregroundColor(.primary)
                        
                        TextField("Port", text: $inputPort)
                            .keyboardType(.numberPad)
                            .textFieldStyle(.roundedBorder)
                            .frame(maxWidth: 100)
                            .font(.system(.body, design: .monospaced))
                            .onChange(of: inputPort) { _, newValue in
                                let filtered = newValue.filter { $0.isNumber }
                                if filtered != newValue {
                                    inputPort = filtered
                                }
                                if let port = Int(filtered), port > 0 && port <= 65535 {
                                    networkManager.grpcPort = port
                                }
                            }
                    }
                }
                
                HStack {
                    Circle()
                        .fill(connectionStatusColor)
                        .frame(width: 8, height: 8)
                    Text("\(networkManager.connectionStatus)")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                
                Divider()
                    .padding(.vertical, 4)
                HStack {
                    Text("Show My Hand")
                        .font(.caption)
                        .foregroundColor(.primary)
                    Spacer()
                    Toggle(isOn: $dataManager.upperLimbVisible) {
                        Text(dataManager.upperLimbVisible ? "Visible" : "Hidden")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    }
                    .labelsHidden()
                }
                
                serverButton
            }
        }
        .overlay(alignment: .topTrailing) {
            HStack(spacing: 8) {
                Button(action: {
                    withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                        isMinimized = true
                    }
                }) {
                    ZStack {
                        Circle()
                            .fill(.thinMaterial)
                            .frame(width: 28, height: 28)
                            .shadow(radius: 2)
                        Image(systemName: "minus")
                            .font(.system(size: 12, weight: .bold))
                            .foregroundColor(.primary)
                    }
                }
                .buttonStyle(.plain)
                .matchedGeometryEffect(id: "statusMiniDot", in: minimizeNS)
                
                Button(action: { exit(0) }) {
                    ZStack {
                        Circle()
                            .fill(.thinMaterial)
                            .frame(width: 28, height: 28)
                            .shadow(radius: 2)
                        Image(systemName: "xmark")
                            .font(.system(size: 12, weight: .bold))
                            .foregroundColor(.red)
                    }
                }
                .buttonStyle(.plain)
            }
        }
        .frame(width: 250)
        .padding(20)
        .background(.regularMaterial, in: RoundedRectangle(cornerRadius: 16))
        .glassBackgroundEffect()
        .transition(.asymmetric(insertion: .opacity.combined(with: .scale),
                                removal: .opacity.combined(with: .move(edge: .top))))
    }
    
    @ViewBuilder
    private var serverButton: some View {
        if !networkManager.isServerRunning {
            Button(action: {
                Task { await startServer() }
            }) {
                HStack {
                    Image(systemName: "play.fill")
                    Text("START SERVER")
                        .font(.caption.weight(.semibold))
                }
                .foregroundColor(.white)
                .padding(.horizontal, 16)
                .padding(.vertical, 8)
                .background(.green, in: RoundedRectangle(cornerRadius: 8))
            }
            .buttonStyle(.plain)
        } else {
            Button(action: {
                Task { await stopServer() }
            }) {
                HStack {
                    Image(systemName: "stop.fill")
                    Text("STOP SERVER")
                        .font(.caption.weight(.semibold))
                }
                .foregroundColor(.white)
                .padding(.horizontal, 16)
                .padding(.vertical, 8)
                .background(.red, in: RoundedRectangle(cornerRadius: 8))
            }
            .buttonStyle(.plain)
        }
    }
    
    private var connectionStatusColor: Color {
        switch networkManager.connectionStatus {
        case let status where status.contains("Streaming"):
            return .green
        case let status where status.contains("Connected"):
            return .blue
        case let status where status.contains("Running"):
            return .green
        case let status where status.contains("Starting") || status.contains("Stopping"):
            return .orange
        case let status where status.contains("Stopped"):
            return .gray
        default:
            return .red
        }
    }
    
    // MARK: - Server Control
    private func startServer() async {
        dlog("🚀 [startServer] Starting MuJoCo gRPC server on port \(networkManager.grpcPort)...")
        networkManager.updateConnectionStatus("Starting Server...")
        networkManager.isServerRunning = true
        await grpcManager.startServer(streamingView: self, port: networkManager.grpcPort)
        networkManager.updateConnectionStatus("Server Running")
        dlog("✅ [startServer] MuJoCo gRPC server started successfully")
    }
    
    private func stopServer() async {
        dlog("🛑 [stopServer] Stopping MuJoCo gRPC server...")
        networkManager.updateConnectionStatus("Stopping Server...")
        await grpcManager.stopServer()
        networkManager.updateConnectionStatus("Server Stopped")
        networkManager.isServerRunning = false
        dlog("✅ [stopServer] MuJoCo gRPC server stopped successfully")
    }
    
    // MARK: - Status Display
    private func createStatusDisplay(content: RealityViewContent, attachments: RealityViewAttachments) {
        guard let statusAttachment = attachments.entity(for: "status") else {
            dlog("❌ Could not find status attachment")
            return
        }
        
        let headAnchor = AnchorEntity(.head)
        headAnchor.name = "statusHeadAnchor"
        
        let statusContainer = Entity()
        statusContainer.name = "statusContainer"
        statusContainer.addChild(statusAttachment)
        statusContainer.setPosition([0.0, 0.1, -0.8], relativeTo: headAnchor)
        headAnchor.addChild(statusContainer)
        content.add(headAnchor)
        
        statusContainer.components.set(MuJoCoStatusDisplayComponent(networkManager: networkManager))
        statusEntity = headAnchor
        statusContainerEntity = statusContainer
        updateStatusContainerPosition(animated: false)
        
        dlog("✅ Created head-following status display")
    }
    
    @MainActor
    private func updateStatusContainerPosition(animated: Bool) {
        guard let head = statusEntity, let container = statusContainerEntity else { return }
        let expandedPos = SIMD3<Float>(0.0, 0.1, -0.8)
        let minimizedPos = SIMD3<Float>(0.3, 0.3, -0.8)
        let target = isMinimized ? minimizedPos : expandedPos
        let transform = Transform(translation: target)
        if animated {
            container.move(to: transform, relativeTo: head, duration: 0.4)
        } else {
            container.setPosition(target, relativeTo: head)
        }
    }
    
    // MARK: - USDZ Model Loading
    private func loadUsdzModel(from url: URL) async {
        do {
            dlog("♻️ Resetting previous model and cached data...")
            
            await MainActor.run {
                if let oldEntity = entity, let content = realityContent {
                    content.remove(oldEntity)
                    dlog("🧹 Removed previous entity from RealityView")
                }
            }
            
            entity = nil
            bodyEntities.removeAll()
            initialLocalTransforms.removeAll()
            pythonToSwiftNameMap.removeAll()
            nameMappingInitialized = false
            pythonToSwiftTargets.removeAll()
            entityPathByObjectID.removeAll()
            
            dlog("📦 Loading USDZ from \(url.absoluteString)")
            let loadedEntity = try await spatialGen.loadEntity(from: url)
            
            let newEntity: ModelEntity
            if let model = loadedEntity as? ModelEntity {
                newEntity = model
            } else {
                let wrapper = ModelEntity()
                wrapper.addChild(loadedEntity)
                newEntity = wrapper
            }
            
            await MainActor.run {
                if let content = realityContent {
                    content.add(newEntity)
                    dlog("✅ Added new entity to RealityView: \(newEntity.name)")
                }
                entity = newEntity
            }
            
            indexBodyEntities(newEntity)
            dlog("✅ Model loaded and indexed successfully")
            
        } catch {
            dlog("❌ Failed to load USDZ: \(error)")
        }
    }
    
    // MARK: - Body Entity Indexing
    private func indexBodyEntities(_ rootEntity: Entity) {
        bodyEntities.removeAll()
        initialLocalTransforms.removeAll()
        entityPathByObjectID.removeAll()
        dlog("🔍 [indexBodyEntities] Starting recursive indexing...")
        
        func indexRec(_ entity: Entity, parentPath: String) {
            if entity.name.isEmpty {
                for child in entity.children {
                    indexRec(child, parentPath: parentPath)
                }
                return
            }
            
            if let modelEntity = entity as? ModelEntity {
                let pathKey = parentPath.isEmpty ? modelEntity.name : "\(parentPath)/\(modelEntity.name)"
                bodyEntities[pathKey] = modelEntity
                initialLocalTransforms[pathKey] = modelEntity.transform
                entityPathByObjectID[ObjectIdentifier(modelEntity)] = pathKey
                
                dlog("✅ Added ModelEntity: '\(modelEntity.name)'")
                
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
                bodyEntities[pathKey] = wrapper
                initialLocalTransforms[pathKey] = wrapper.transform
                entityPathByObjectID[ObjectIdentifier(wrapper)] = pathKey
                
                dlog("✅ Added wrapper: '\(wrapper.name)'")
                
                for child in originalChildren {
                    indexRec(child, parentPath: pathKey)
                }
            }
        }
        
        indexRec(rootEntity, parentPath: "")
        dlog("📝 Indexed \(bodyEntities.count) entities with cached local transforms")
    }
    
    // MARK: - Pose Transform Application
    private func applyFinalTransforms(_ transforms: [String: simd_float4x4]) {
        var depthCache: [String: Int] = [:]
        
        func depth(for name: String) -> Int {
            if let d = depthCache[name] { return d }
            guard let entity = bodyEntities[name] else { depthCache[name] = 0; return 0 }
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
        var appliedWorld: [String: simd_float4x4] = [:]
        
        for name in sortedNames {
            guard let entity = bodyEntities[name], let desiredWorld = transforms[name] else { continue }
            
            if applyInWorldSpace {
                let prevLocalScale = entity.scale
                entity.setTransformMatrix(desiredWorld, relativeTo: nil)
                if let parent = entity.parent {
                    entity.setScale(prevLocalScale, relativeTo: parent)
                } else {
                    entity.setScale(prevLocalScale, relativeTo: nil)
                }
                appliedWorld[name] = desiredWorld
            }
        }
    }
    
    // MARK: - Public API for gRPC Service
    func updateUsdzURL(_ url: String, attachToPosition: SIMD3<Float>? = nil, attachToRotation: simd_quatf? = nil) {
        dlog("🔄 [updateUsdzURL] Called with URL: \(url)")
        
        self.attachToPosition = attachToPosition
        self.attachToRotation = attachToRotation
        
        if let position = attachToPosition, let rotation = attachToRotation {
            dlog("📍 [updateUsdzURL] Attach to position: \(position), rotation: \(rotation)")
        }
        
        networkManager.updateConnectionStatus("Client Connected - USDZ Received")
        
        var finalURLString = url
        let timestamp = Int(Date().timeIntervalSince1970)
        
        if url.hasPrefix("http://") || url.hasPrefix("https://") || url.hasPrefix("file://") {
            if url.contains("?") {
                finalURLString += "&t=\(timestamp)"
            } else {
                finalURLString += "?t=\(timestamp)"
            }
            
            // If it's a file URL, pass it to RecordingManager (stripping query params)
            if let urlObj = URL(string: url), urlObj.isFileURL {
                RecordingManager.shared.setUsdzUrl(urlObj)
            }
        } else {
            let fileURL = URL(fileURLWithPath: url)
            finalURLString = "\(fileURL.absoluteString)?t=\(timestamp)"
            
            // Pass clean file URL to RecordingManager
            RecordingManager.shared.setUsdzUrl(fileURL)
        }
        
        self.usdzURL = finalURLString
    }
    
    func updatePosesWithTransforms(_ transforms: [String: simd_float4x4]) {
        dlog("🔄 [updatePosesWithTransforms] Called with \(transforms.count) final transforms")
        networkManager.updateConnectionStatus("Streaming Poses - \(transforms.count) Bodies")
        finalTransforms = transforms
        poseUpdateTrigger = UUID()
    }
    
    func computeFinalTransforms(_ poses: [String: MujocoAr_BodyPose]) -> [String: simd_float4x4] {
        guard !bodyEntities.isEmpty, !initialLocalTransforms.isEmpty else {
            dlog("⏳ Entities not fully indexed yet; deferring pose application")
            return [:]
        }
        
        // Initialize name mapping if needed
        if !nameMappingInitialized {
            let pythonNames = Array(poses.keys)
            initializeNameMapping(pythonNames: pythonNames)
            let validMatches = pythonNames.reduce(0) { acc, py in
                if let swift = pythonToSwiftNameMap[py], bodyEntities[swift] != nil { return acc + 1 }
                return acc
            }
            if validMatches > 0 { nameMappingInitialized = true }
        }
        
        let axisCorrection = simd_quatf(angle: -.pi / 2, axis: SIMD3<Float>(1, 0, 0))
        var finalTransforms: [String: simd_float4x4] = [:]
        
        for (pyName, pose) in poses {
            let bodyName = pythonToSwiftNameMap[pyName] ?? pyName
            
            guard initialLocalTransforms[bodyName] != nil else {
                continue
            }
            
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
    
    // MARK: - Name Mapping
    private func initializeNameMapping(pythonNames: [String]) {
        let swiftNames = Array(bodyEntities.keys)
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
        dlog("🔤 Built name mapping (Python → Swift): \(newMap.count) entries")
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
    
    // MARK: - Hand Tracking
    func getHandTrackingData() -> MujocoAr_HandTrackingUpdate {
        var update = MujocoAr_HandTrackingUpdate()
        let data = DataManager.shared.latestHandTrackingData
        
        update.timestamp = Date().timeIntervalSince1970
        update.leftHand.wristMatrix = convertToMuJoCoMatrix4x4(from: data.leftWrist)
        update.rightHand.wristMatrix = convertToMuJoCoMatrix4x4(from: data.rightWrist)
        update.leftHand.skeleton.jointMatrices = data.leftSkeleton.joints.map { convertToMuJoCoMatrix4x4(from: $0) }
        update.rightHand.skeleton.jointMatrices = data.rightSkeleton.joints.map { convertToMuJoCoMatrix4x4(from: $0) }
        update.head = convertToMuJoCoMatrix4x4(from: data.Head)
        
        return update
    }
}

// MARK: - Helper Functions
func convertToMuJoCoMatrix4x4(from jointMatrix: simd_float4x4) -> MujocoAr_Matrix4x4 {
    var matrix = MujocoAr_Matrix4x4()
    matrix.m00 = jointMatrix.columns.0.x
    matrix.m01 = jointMatrix.columns.1.x
    matrix.m02 = jointMatrix.columns.2.x
    matrix.m03 = jointMatrix.columns.3.x
    matrix.m10 = jointMatrix.columns.0.y
    matrix.m11 = jointMatrix.columns.1.y
    matrix.m12 = jointMatrix.columns.2.y
    matrix.m13 = jointMatrix.columns.3.y
    matrix.m20 = jointMatrix.columns.0.z
    matrix.m21 = jointMatrix.columns.1.z
    matrix.m22 = jointMatrix.columns.2.z
    matrix.m23 = jointMatrix.columns.3.z
    matrix.m30 = jointMatrix.columns.0.w
    matrix.m31 = jointMatrix.columns.1.w
    matrix.m32 = jointMatrix.columns.2.w
    matrix.m33 = jointMatrix.columns.3.w
    return matrix
}

// MARK: - Status Display Component
struct MuJoCoStatusDisplayComponent: Component {
    var networkManager: MuJoCoNetworkManager
}

public struct MuJoCoStatusDisplaySystem: System {
    static let query = EntityQuery(where: .has(MuJoCoStatusDisplayComponent.self))
    
    public init(scene: RealityKit.Scene) {}
    
    public func update(context: SceneUpdateContext) {
        let entities = context.entities(matching: Self.query, updatingSystemWhen: .rendering)
        for entity in entities {
            guard let statusComponent = entity.components[MuJoCoStatusDisplayComponent.self] else { continue }
            let currentTime = CACurrentMediaTime()
            if Int(currentTime) % 5 == 0 {
                statusComponent.networkManager.updateNetworkInfo()
            }
        }
    }
}

// MARK: - Network Manager
final class MuJoCoNetworkManager: ObservableObject {
    @Published var ipAddress: String = "Getting IP..."
    @Published var connectionStatus: String = "Server Stopped"
    @Published var grpcPort: Int = 50051
    @Published var isServerRunning: Bool = false
    
    init() {
        updateNetworkInfo()
    }
    
    func updateNetworkInfo() {
        DispatchQueue.global(qos: .background).async {
            let ip = self.getWiFiAddress() ?? self.getLocalIPAddress() ?? "No IP Found"
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
    
    private func getLocalIPAddress() -> String? {
        let host = CFHostCreateWithName(nil, "google.com" as CFString).takeRetainedValue()
        CFHostStartInfoResolution(host, .addresses, nil)
        var success: DarwinBoolean = false
        if let addresses = CFHostGetAddressing(host, &success)?.takeUnretainedValue() as NSArray?,
           let theAddress = addresses.firstObject as? NSData {
            var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
            if getnameinfo(theAddress.bytes.assumingMemoryBound(to: sockaddr.self), socklen_t(theAddress.length),
                          &hostname, socklen_t(hostname.count), nil, 0, NI_NUMERICHOST) == 0 {
                return String(cString: hostname)
            }
        }
        return nil
    }
}
