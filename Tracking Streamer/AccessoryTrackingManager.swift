import SwiftUI
import ARKit
import GameController
import RealityKit
import simd

// MARK: - Data Structures

/// Snapshot of stylus state for streaming to Python
@available(visionOS 26.0, *)
struct StylusSnapshot {
    var isTracking: Bool = false
    var position: SIMD3<Float>?
    var orientation: simd_quatf?
    var tipPressed: Bool = false
    var tipPressure: Float = 0.0
    var primaryPressed: Bool = false
    var primaryPressure: Float = 0.0
    var secondaryPressed: Bool = false
    var secondaryPressure: Float = 0.0
    var timestamp: TimeInterval = 0
}

/// Connected stylus info (for UI display)
@available(visionOS 26.0, *)
struct ConnectedStylus: Identifiable {
    let id: Int  // Stylus hash
    let name: String
    var isSelected: Bool
    weak var stylus: GCStylus?
}

/// Manager for ARKit accessory tracking (spatial styluses like Apple Pencil Pro)
/// Uses AccessoryTrackingProvider pattern from Apple sample code
/// Only available on visionOS 26.0 or later
@available(visionOS 26.0, *)
@MainActor
class AccessoryTrackingManager: ObservableObject {
    static let shared = AccessoryTrackingManager()
    
    // MARK: - Published State
    
    /// Whether accessory tracking is enabled
    @Published var isEnabled: Bool {
        didSet {
            UserDefaults.standard.set(isEnabled, forKey: "accessoryTrackingEnabled")
            if isEnabled {
                Task { await startTracking() }
            } else {
                stopTracking()
            }
        }
    }
    
    /// Available styluses (can be selected for tracking)
    @Published var availableStyluses: [ConnectedStylus] = []
    
    /// Status message for UI display
    @Published var statusMessage: String = "No stylus connected"
    
    /// Whether tracking is actively running
    @Published var isTrackingActive: Bool = false
    
    /// The root entity for visualization (set by CombinedStreamingView)
    weak var rootEntity: Entity? {
        didSet {
            // If tracking is active and we just got a root entity, setup visualization
            if rootEntity != nil && isTrackingActive && visualizationEntity == nil {
                setupVisualization()
            }
        }
    }
    
    /// Visualization entities
    var visualizationEntity: Entity?
    var stylusAnchorEntity: AnchorEntity?  // AnchorEntity for auto-tracking
    var sphereEntity: ModelEntity?
    var xAxisEntity: ModelEntity?
    var yAxisEntity: ModelEntity?
    var zAxisEntity: ModelEntity?
    var labelEntity: Entity?  // Billboard text label for button states
    
    // MARK: - ARKit Tracking (from Apple sample pattern)
    
    /// The active stylus being tracked
    private(set) var activeStylus: GCStylus? = nil
    
    /// The accessory tracking provider
    private var accessoryTrackingProvider: AccessoryTrackingProvider? = nil
    
    /// The ARKit session
    private let arKitSession = ARKitSession()
    
    /// Timer for updating button state display
    private var buttonUpdateTimer: Timer?
    
    // MARK: - Thread-Safe Snapshot Properties
    
    private let snapshotLock = NSLock()
    nonisolated(unsafe) private var _snapshot = StylusSnapshot()
    
    /// Thread-safe snapshot for streaming
    nonisolated var snapshot: StylusSnapshot {
        snapshotLock.lock()
        defer { snapshotLock.unlock() }
        return _snapshot
    }
    
    // Legacy compatibility properties
    nonisolated var snapshotEnabled: Bool {
        snapshot.isTracking
    }
    
    nonisolated var snapshotTrackingActive: Bool {
        snapshot.isTracking
    }
    
    nonisolated var snapshotStylusTransform: simd_float4x4? {
        guard let pos = snapshot.position, let rot = snapshot.orientation else { return nil }
        var matrix = simd_float4x4(rot)
        matrix.columns.3 = SIMD4<Float>(pos.x, pos.y, pos.z, 1)
        return matrix
    }
    
    // MARK: - Initialization
    
    private init() {
        self.isEnabled = UserDefaults.standard.bool(forKey: "accessoryTrackingEnabled")
        
        Task {
            await setupStylusNotifications()
        }
    }
    
    deinit {
        NotificationCenter.default.removeObserver(self)
    }
    
    // MARK: - Stylus Notifications (from Apple sample)
    
    private func setupStylusNotifications() async {
        // Check for already connected styluses
        if let stylus = GCStylus.styli.first(where: { $0.productCategory == GCProductCategorySpatialStylus }) {
            await setActiveStylus(stylus: stylus)
        }
        
        // Listen for stylus connections
        NotificationCenter.default.addObserver(
            forName: NSNotification.Name.GCStylusDidConnect,
            object: nil,
            queue: nil
        ) { [weak self] notification in
            if let stylus = notification.object as? GCStylus,
               stylus.productCategory == GCProductCategorySpatialStylus {
                Task { @MainActor in
                    await self?.setActiveStylus(stylus: stylus)
                }
            }
        }
        
        // Listen for stylus disconnections
        NotificationCenter.default.addObserver(
            forName: NSNotification.Name.GCStylusDidDisconnect,
            object: nil,
            queue: nil
        ) { [weak self] notification in
            if let stylus = notification.object as? GCStylus,
               stylus.productCategory == GCProductCategorySpatialStylus {
                Task { @MainActor in
                    if stylus == self?.activeStylus {
                        withAnimation {
                            self?.activeStylus = nil
                        }
                        self?.arKitSession.stop()
                        self?.isTrackingActive = false
                        self?.updateStatusMessage()
                    }
                }
            }
        }
        
        refreshStyluses()
        
        if isEnabled {
            await startTracking()
        }
    }
    
    private func setActiveStylus(stylus: GCStylus) async {
        guard stylus != activeStylus else { return }
        
        withAnimation {
            activeStylus = stylus
        }
        
        // Set input queue depth to buffer inputs (from Apple sample)
        activeStylus?.input?.inputStateQueueDepth = 100
        
        // Add to available list if not present
        let id = stylus.hash
        if !availableStyluses.contains(where: { $0.id == id }) {
            availableStyluses.append(ConnectedStylus(
                id: id,
                name: stylus.vendorName ?? "Spatial Stylus",
                isSelected: true,
                stylus: stylus
            ))
        }
        
        updateStatusMessage()
        dlog("✏️ [AccessoryTracking] Active stylus set: \(stylus.vendorName ?? "Unknown")")
        
        // Start tracking if enabled
        if isEnabled {
            await runAccessoryTracking(for: stylus)
        }
    }
    
    // MARK: - Stylus List Management
    
    func refreshStyluses() {
        let spatialStyluses = GCStylus.styli.filter {
            $0.productCategory == GCProductCategorySpatialStylus
        }
        
        let existingSelection = Dictionary(uniqueKeysWithValues:
            availableStyluses.map { ($0.id, $0.isSelected) }
        )
        
        availableStyluses = spatialStyluses.map { stylus in
            let id = stylus.hash
            let wasSelected = existingSelection[id] ?? true
            return ConnectedStylus(
                id: id,
                name: stylus.vendorName ?? "Spatial Stylus",
                isSelected: wasSelected,
                stylus: stylus
            )
        }
        
        updateStatusMessage()
    }
    
    func toggleStylus(id: Int) {
        if let index = availableStyluses.firstIndex(where: { $0.id == id }) {
            availableStyluses[index].isSelected.toggle()
            
            if isEnabled && isTrackingActive {
                Task { await restartTracking() }
            }
        }
    }
    
    var selectedStylus: GCStylus? {
        availableStyluses
            .first(where: { $0.isSelected && $0.stylus != nil })?
            .stylus
    }
    
    // MARK: - Tracking Control
    
    func startTracking() async {
        guard !isTrackingActive else { return }
        
        if let stylus = activeStylus ?? selectedStylus {
            await runAccessoryTracking(for: stylus)
        } else {
            statusMessage = "No stylus connected"
        }
    }
    
    /// Run accessory tracking for a stylus (from Apple sample)
    private func runAccessoryTracking(for stylus: GCStylus) async {
        // Create Accessory from stylus
        guard let accessory = try? await Accessory(device: stylus) else {
            statusMessage = "Failed to create accessory"
            dlog("❌ [AccessoryTracking] Failed to create Accessory from stylus")
            return
        }
        
        // Create the accessory tracking provider
        accessoryTrackingProvider = AccessoryTrackingProvider(accessories: [accessory])
        
        guard let provider = accessoryTrackingProvider else { return }
        
        do {
            try await arKitSession.run([provider])
            isTrackingActive = true
            activeStylus = stylus
            
            // Setup anchor-based visualization (auto-tracks stylus)
            await setupAnchorVisualization(stylus: stylus)
            
            // Start timer for button state updates
            startButtonUpdateTimer()
            
            updateStatusMessage()
            dlog("✅ [AccessoryTracking] ARKitSession running with AccessoryTrackingProvider")
        } catch {
            statusMessage = "Failed: \(error.localizedDescription)"
            dlog("❌ [AccessoryTracking] ARKitSession.run failed: \(error)")
        }
    }
    
    func stopTracking() {
        arKitSession.stop()
        accessoryTrackingProvider = nil
        isTrackingActive = false
        
        visualizationEntity?.removeFromParent()
        visualizationEntity = nil
        stylusAnchorEntity?.removeFromParent()
        stylusAnchorEntity = nil
        sphereEntity = nil
        xAxisEntity = nil
        yAxisEntity = nil
        zAxisEntity = nil
        labelEntity = nil
        
        updateStatusMessage()
        updateSnapshots()
        stopButtonUpdateTimer()
        dlog("🛑 [AccessoryTracking] Stopped tracking")
    }
    
    private func startButtonUpdateTimer() {
        stopButtonUpdateTimer()
        buttonUpdateTimer = Timer.scheduledTimer(withTimeInterval: 0.05, repeats: true) { [weak self] _ in
            Task { @MainActor in
                self?.updateButtonDisplay()
            }
        }
    }
    
    private func stopButtonUpdateTimer() {
        buttonUpdateTimer?.invalidate()
        buttonUpdateTimer = nil
    }
    
    /// Update the label display with current button states
    private func updateButtonDisplay() {
        guard isTrackingActive else { return }
        
        if let buttons = getLatestButtonInputs() {
            updateLabelText(
                tipPressed: buttons.tipPressed,
                tipPressure: buttons.tipPressure,
                primaryPressed: buttons.primaryPressed,
                primaryPressure: buttons.primaryPressure,
                secondaryPressed: buttons.secondaryPressed,
                secondaryPressure: buttons.secondaryPressure
            )
            
            // Also update snapshot
            snapshotLock.lock()
            _snapshot.tipPressed = buttons.tipPressed
            _snapshot.tipPressure = buttons.tipPressure
            _snapshot.primaryPressed = buttons.primaryPressed
            _snapshot.primaryPressure = buttons.primaryPressure
            _snapshot.secondaryPressed = buttons.secondaryPressed
            _snapshot.secondaryPressure = buttons.secondaryPressure
            snapshotLock.unlock()
        }
    }
    
    private func restartTracking() async {
        stopTracking()
        try? await Task.sleep(nanoseconds: 100_000_000)
        await startTracking()
    }
    
    // MARK: - Anchor Access (from Apple sample)
    
    /// Get the latest stylus anchor
    func getLatestStylusAnchor() -> AccessoryAnchor? {
        guard accessoryTrackingProvider?.state == .running else { return nil }
        return accessoryTrackingProvider?.latestAnchors.first
    }
    
    /// Get predicted stylus anchor at a future timestamp
    func getPredictedStylusAnchor(at timestamp: TimeInterval) -> AccessoryAnchor? {
        guard let provider = accessoryTrackingProvider,
              let latestAnchor = getLatestStylusAnchor() else { return nil }
        return provider.predictAnchor(for: latestAnchor, at: timestamp)
    }
    
    // MARK: - Button Input Capture (from Apple sample)
    
    /// Persistent button state (updated by processing input queue)
    private var _tipPressed = false
    private var _tipPressure: Float = 0.0
    private var _primaryPressed = false
    private var _primaryPressure: Float = 0.0
    private var _secondaryPressed = false
    private var _secondaryPressure: Float = 0.0
    
    /// Get latest button inputs from the stylus
    /// Returns the current button state (updated from input queue)
    func getLatestButtonInputs() -> (tipPressed: Bool, tipPressure: Float,
                                     primaryPressed: Bool, primaryPressure: Float,
                                     secondaryPressed: Bool, secondaryPressure: Float)? {
        guard let stylusInput = activeStylus?.input else { return nil }
        
        // Process all available input states to get the latest
        while let inputState = stylusInput.nextInputState() {
            if let tip = inputState.buttons[.stylusTip]?.pressedInput {
                _tipPressed = tip.isPressed
                _tipPressure = tip.value
            }
            if let primary = inputState.buttons[.stylusPrimaryButton]?.pressedInput {
                _primaryPressed = primary.isPressed
                _primaryPressure = primary.value
            }
            if let secondary = inputState.buttons[.stylusSecondaryButton]?.pressedInput {
                _secondaryPressed = secondary.isPressed
                _secondaryPressure = secondary.value
            }
        }
        
        // Return the persistent state (includes release events)
        return (_tipPressed, _tipPressure, _primaryPressed, _primaryPressure, _secondaryPressed, _secondaryPressure)
    }
    
    // MARK: - Snapshot Update
    
    /// Update snapshot with latest tracking and button data
    /// Called from RealityView update loop
    func updateSnapshots() {
        // Debug: log guard condition occasionally
        if Int.random(in: 0...60) == 0 {
            dlog("🔄 [AccessoryTracking] updateSnapshots: isEnabled=\(isEnabled), isTrackingActive=\(isTrackingActive)")
        }
        
        guard isEnabled && isTrackingActive else {
            snapshotLock.lock()
            _snapshot = StylusSnapshot()
            snapshotLock.unlock()
            return
        }
        
        var newSnapshot = StylusSnapshot()
        newSnapshot.isTracking = true
        
        // Get anchor data
        if let anchor = getLatestStylusAnchor() {
            newSnapshot.timestamp = anchor.timestamp
            
            // Get position and orientation from anchor's transform
            let transform = anchor.originFromAnchorTransform
            let position = SIMD3<Float>(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
            let rotation = simd_quatf(transform)
            newSnapshot.position = position
            newSnapshot.orientation = rotation
            
            // Log first successful anchor retrieval
            if Int.random(in: 0...500) == 0 {
                dlog("📍 [AccessoryTracking] Anchor pos: (\(position.x), \(position.y), \(position.z))")
            }
        } else {
            // Debug: log why no anchor - more frequently now
            let state = accessoryTrackingProvider?.state
            let anchorCount = accessoryTrackingProvider?.latestAnchors.count ?? -1
            if Int.random(in: 0...30) == 0 {
                dlog("⚠️ [AccessoryTracking] No anchor. Provider state: \(String(describing: state)), anchors: \(anchorCount)")
            }
        }
        
        // Get button data
        if let buttons = getLatestButtonInputs() {
            newSnapshot.tipPressed = buttons.tipPressed
            newSnapshot.tipPressure = buttons.tipPressure
            newSnapshot.primaryPressed = buttons.primaryPressed
            newSnapshot.primaryPressure = buttons.primaryPressure
            newSnapshot.secondaryPressed = buttons.secondaryPressed
            newSnapshot.secondaryPressure = buttons.secondaryPressure
        }
        
        // Update visualization position
        if let pos = newSnapshot.position, let rot = newSnapshot.orientation {
            visualizationEntity?.position = pos
            visualizationEntity?.orientation = rot
            
            // Update button state label
            updateLabelText(
                tipPressed: newSnapshot.tipPressed,
                tipPressure: newSnapshot.tipPressure,
                primaryPressed: newSnapshot.primaryPressed,
                primaryPressure: newSnapshot.primaryPressure,
                secondaryPressed: newSnapshot.secondaryPressed,
                secondaryPressure: newSnapshot.secondaryPressure
            )
            
            // Make label face upward (billboard effect - always readable)
            // Reset label rotation to world up so it's always horizontal
            labelEntity?.orientation = simd_quatf(angle: 0, axis: [0, 1, 0])
        }
        
        snapshotLock.lock()
        _snapshot = newSnapshot
        snapshotLock.unlock()
    }
    
    // MARK: - Visualization
    
    func setupVisualization() {
        guard let root = rootEntity else {
            dlog("⚠️ [AccessoryTracking] setupVisualization called but rootEntity is nil")
            return
        }
        guard visualizationEntity == nil else {
            dlog("ℹ️ [AccessoryTracking] Visualization already exists")
            return
        }
        
        // Remove old visualization
        visualizationEntity?.removeFromParent()
        
        let container = Entity()
        container.name = "stylusVisualization"
        root.addChild(container)
        visualizationEntity = container
        
        let sphereRadius: Float = 0.01
        let axisLength: Float = 0.05
        let axisRadius: Float = 0.002
        
        // Cyan sphere
        var sphereMaterial = UnlitMaterial()
        sphereMaterial.color = .init(tint: UIColor(red: 0.0, green: 0.9, blue: 1.0, alpha: 0.8))
        let sphere = ModelEntity(mesh: .generateSphere(radius: sphereRadius), materials: [sphereMaterial])
        container.addChild(sphere)
        sphereEntity = sphere
        
        // X axis (red)
        var xMaterial = UnlitMaterial()
        xMaterial.color = .init(tint: .red)
        let xAxis = ModelEntity(mesh: .generateCylinder(height: axisLength, radius: axisRadius), materials: [xMaterial])
        xAxis.transform.rotation = simd_quatf(angle: -.pi / 2, axis: [0, 0, 1])
        xAxis.position = [axisLength / 2, 0, 0]
        container.addChild(xAxis)
        xAxisEntity = xAxis
        
        // Y axis (green)
        var yMaterial = UnlitMaterial()
        yMaterial.color = .init(tint: .green)
        let yAxis = ModelEntity(mesh: .generateCylinder(height: axisLength, radius: axisRadius), materials: [yMaterial])
        yAxis.position = [0, axisLength / 2, 0]
        container.addChild(yAxis)
        yAxisEntity = yAxis
        
        // Z axis (blue)
        var zMaterial = UnlitMaterial()
        zMaterial.color = .init(tint: .blue)
        let zAxis = ModelEntity(mesh: .generateCylinder(height: axisLength, radius: axisRadius), materials: [zMaterial])
        zAxis.transform.rotation = simd_quatf(angle: .pi / 2, axis: [1, 0, 0])
        zAxis.position = [0, 0, axisLength / 2]
        container.addChild(zAxis)
        zAxisEntity = zAxis
        
        // Billboard label for button states (positioned above stylus)
        let label = Entity()
        label.name = "stylusLabel"
        label.position = [0, 0.08, 0]  // 8cm above stylus tip
        container.addChild(label)
        labelEntity = label
        
        // Add initial text mesh
        updateLabelText(tipPressed: false, tipPressure: 0, primaryPressed: false, primaryPressure: 0, secondaryPressed: false, secondaryPressure: 0)
        
        dlog("✅ [AccessoryTracking] Visualization created with sphere, axes, and label")
    }
    
    /// Setup visualization using AnchorEntity that auto-tracks the stylus
    private func setupAnchorVisualization(stylus: GCStylus) async {
        guard let root = rootEntity else {
            dlog("⚠️ [AccessoryTracking] setupAnchorVisualization called but rootEntity is nil")
            return
        }
        
        // Remove old visualization
        visualizationEntity?.removeFromParent()
        stylusAnchorEntity?.removeFromParent()
        
        do {
            let source = try await AnchoringComponent.AccessoryAnchoringSource(device: stylus)
            
            dlog("📍 [AccessoryTracking] Available locations: \(source.accessoryLocations)")
            
            // Try to get "aim" location first, fallback to "origin"
            guard let location = source.locationName(named: "aim") ?? source.locationName(named: "origin") else {
                dlog("⚠️ [AccessoryTracking] No suitable location found")
                // Fall back to manual visualization
                setupVisualization()
                return
            }
            
            // Create AnchorEntity that auto-tracks the stylus
            let anchorEntity = AnchorEntity(
                .accessory(from: source, location: location),
                trackingMode: .continuous,
                physicsSimulation: .none
            )
            
            root.addChild(anchorEntity)
            stylusAnchorEntity = anchorEntity
            visualizationEntity = anchorEntity  // Use the anchor as the visualization container
            
            // Add visualization to the anchor
            let sphereRadius: Float = 0.01
            let axisLength: Float = 0.05
            let axisRadius: Float = 0.002
            
            // Cyan sphere
            var sphereMaterial = UnlitMaterial()
            sphereMaterial.color = .init(tint: UIColor(red: 0.0, green: 0.9, blue: 1.0, alpha: 0.8))
            let sphere = ModelEntity(mesh: .generateSphere(radius: sphereRadius), materials: [sphereMaterial])
            anchorEntity.addChild(sphere)
            sphereEntity = sphere
            
            // X axis (red)
            var xMaterial = UnlitMaterial()
            xMaterial.color = .init(tint: .red)
            let xAxis = ModelEntity(mesh: .generateCylinder(height: axisLength, radius: axisRadius), materials: [xMaterial])
            xAxis.transform.rotation = simd_quatf(angle: -.pi / 2, axis: [0, 0, 1])
            xAxis.position = [axisLength / 2, 0, 0]
            anchorEntity.addChild(xAxis)
            xAxisEntity = xAxis
            
            // Y axis (green)
            var yMaterial = UnlitMaterial()
            yMaterial.color = .init(tint: .green)
            let yAxis = ModelEntity(mesh: .generateCylinder(height: axisLength, radius: axisRadius), materials: [yMaterial])
            yAxis.position = [0, axisLength / 2, 0]
            anchorEntity.addChild(yAxis)
            yAxisEntity = yAxis
            
            // Z axis (blue)
            var zMaterial = UnlitMaterial()
            zMaterial.color = .init(tint: .blue)
            let zAxis = ModelEntity(mesh: .generateCylinder(height: axisLength, radius: axisRadius), materials: [zMaterial])
            zAxis.transform.rotation = simd_quatf(angle: .pi / 2, axis: [1, 0, 0])
            zAxis.position = [0, 0, axisLength / 2]
            anchorEntity.addChild(zAxis)
            zAxisEntity = zAxis
            
            // Billboard label for button states (positioned above sphere)
            let label = Entity()
            label.name = "stylusLabel"
            label.position = [0, 0.05, 0]  // 5cm above the sphere center
            // Add billboard component so label always faces the user
            label.components.set(BillboardComponent())
            sphere.addChild(label)  // Parent to sphere so it follows
            labelEntity = label
            
            // Add initial text mesh
            updateLabelText(tipPressed: false, tipPressure: 0, primaryPressed: false, primaryPressure: 0, secondaryPressed: false, secondaryPressure: 0)
            
            dlog("✅ [AccessoryTracking] AnchorEntity visualization created - auto-tracking stylus")
            
        } catch {
            dlog("❌ [AccessoryTracking] Failed to create anchor: \(error)")
            // Fall back to manual visualization
            setupVisualization()
        }
    }
    
    /// Cache for last displayed state to avoid unnecessary updates
    private var lastStateHash: String = ""
    
    /// Update the billboard label with button states
    /// Only updates when state actually changes to avoid flashing
    private func updateLabelText(tipPressed: Bool, tipPressure: Float,
                                  primaryPressed: Bool, primaryPressure: Float,
                                  secondaryPressed: Bool, secondaryPressure: Float) {
        guard let label = labelEntity else { return }
        
        // Convert pressure to percentage (rounded for caching)
        let tipPct = Int(tipPressure * 100)
        let secPct = Int(secondaryPressure * 100)
        
        // Build state hash for comparison
        let stateHash = "\(tipPressed)-\(tipPct)-\(primaryPressed)-\(secPct)-\(secondaryPressed)"
        guard stateHash != lastStateHash else { return }
        lastStateHash = stateHash
        
        // Remove ALL old text entities
        for child in label.children {
            child.removeFromParent()
        }
        
        let fontSize: CGFloat = 0.012
        let lineSpacing: Float = 0.018
        
        // Create TIP line (pressure button)
        let tipText = tipPressed ? String(format: "TIP  %3d%%", tipPct) : "TIP   --"
        let tipEntity = createTextLine(tipText, pressed: tipPressed, yOffset: lineSpacing)
        label.addChild(tipEntity)
        
        // Create PRI line (binary button with circle)
        let priText = primaryPressed ? "PRI   ●" : "PRI   ○"
        let priEntity = createTextLine(priText, pressed: primaryPressed, yOffset: 0)
        label.addChild(priEntity)
        
        // Create SEC line (pressure button)
        let secText = secondaryPressed ? String(format: "SEC  %3d%%", secPct) : "SEC   --"
        let secEntity = createTextLine(secText, pressed: secondaryPressed, yOffset: -lineSpacing)
        label.addChild(secEntity)
    }
    
    /// Create a single text line with appropriate color
    private func createTextLine(_ text: String, pressed: Bool, yOffset: Float) -> Entity {
        let textMesh = MeshResource.generateText(
            text,
            extrusionDepth: 0.001,
            font: .monospacedSystemFont(ofSize: 0.012, weight: .medium),
            containerFrame: .zero,
            alignment: .left,
            lineBreakMode: .byClipping
        )
        
        var material = UnlitMaterial()
        // Green when pressed, white when not
        material.color = .init(tint: pressed ? UIColor.green : UIColor.white.withAlphaComponent(0.8))
        
        let entity = ModelEntity(mesh: textMesh, materials: [material])
        // Left align - position at x=0, with y offset for stacking
        let bounds = textMesh.bounds
        entity.position = [-bounds.extents.x / 2, yOffset, 0]
        return entity
    }
    
    // MARK: - Helpers
    
    private func updateStatusMessage() {
        if isTrackingActive {
            if accessoryTrackingProvider?.state == .running {
                statusMessage = "Tracking \(activeStylus?.vendorName ?? "stylus")"
            } else {
                statusMessage = "Searching..."
            }
        } else if availableStyluses.isEmpty {
            statusMessage = "No stylus connected"
        } else {
            let selectedCount = availableStyluses.filter { $0.isSelected }.count
            statusMessage = "\(selectedCount)/\(availableStyluses.count) selected"
        }
    }
    
    var trackedCount: Int {
        isTrackingActive && accessoryTrackingProvider?.state == .running ? 1 : 0
    }
    
    // Legacy compatibility - removed, now using stored property above
}

// MARK: - Availability Wrapper

func isAccessoryTrackingAvailable() -> Bool {
    if #available(visionOS 26.0, *) {
        return true
    }
    return false
}
