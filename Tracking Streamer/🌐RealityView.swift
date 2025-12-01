import SwiftUI
import RealityKit
import ARKit

struct 🌐RealityView: View {
    var model: 🥽AppModel
    @State private var isMinimized = false
    @State private var showViewControls = false
    @State private var previewZDistance: Float? = nil
    @State private var previewActive = false
    @State private var userInteracted = false
    @State private var previewStatusPosition: (x: Float, y: Float)? = nil
    @State private var previewStatusActive = false
    @ObservedObject private var dataManager = DataManager.shared
    
    var body: some View {
        RealityView { content, attachments in
            print("🟢 [🌐RealityView] RealityView content block called")
            
            let resultLabelEntity = attachments.entity(for: Self.resultLabelID)!
            resultLabelEntity.components.set(🧑HeadTrackingComponent())
            resultLabelEntity.name = 🧩Name.resultLabel
            
            // Create status display anchor
            let statusAnchor = AnchorEntity(.head)
            statusAnchor.name = "statusHeadAnchor"
            content.add(statusAnchor)
            
            // Create status container entity
            let statusContainer = Entity()
            statusContainer.name = "statusContainer"
            statusContainer.setParent(statusAnchor)
            
            // Position status in top area (relative to head)
            statusContainer.transform.translation = SIMD3<Float>(0.0, 0.0, -1.0)
            
            // Attach the status UI to the container
            if let statusAttachment = attachments.entity(for: Self.statusAttachmentID) {
                print("🟢 [🌐RealityView] Status attachment found and attached")
                statusAttachment.setParent(statusContainer)
            } else {
                print("🔴 [🌐RealityView] Status attachment NOT found!")
            }
            
            // Create preview status container entity (initially hidden)
            let statusPreviewContainer = Entity()
            statusPreviewContainer.name = "statusPreviewContainer"
            statusPreviewContainer.setParent(statusAnchor)
            
            // Initialize at the correct Z position
            statusPreviewContainer.transform.translation = SIMD3<Float>(
                dataManager.statusMinimizedXPosition,
                dataManager.statusMinimizedYPosition,
                -1.0
            )
            
            // Attach the status preview UI to the preview container
            if let statusPreviewAttachment = attachments.entity(for: "statusPreview") {
                print("🟢 [🌐RealityView] Status preview attachment found and attached")
                statusPreviewAttachment.setParent(statusPreviewContainer)
                statusPreviewContainer.isEnabled = false
            }
        } update: { updateContent, attachments in
            // Explicitly depend on state to trigger updates
            let _ = isMinimized
            let _ = showViewControls
            let _ = previewStatusPosition
            let _ = previewStatusActive
            let _ = dataManager.statusMinimizedXPosition
            let _ = dataManager.statusMinimizedYPosition
            
            // Update status container position based on minimized state
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
                        
                        statusPreviewContainer.isEnabled = true
                        var previewTransform = statusPreviewContainer.transform
                        previewTransform.translation = SIMD3<Float>(xPos, yPos, -1.0)
                        statusPreviewContainer.move(to: previewTransform, relativeTo: statusPreviewContainer.parent, duration: 0.1, timingFunction: .linear)
                    } else {
                        statusPreviewContainer.isEnabled = false
                    }
                }
            }
        } attachments: {
            Attachment(id: Self.resultLabelID) {
            }
            Attachment(id: Self.statusAttachmentID) {
                print("🟡 [🌐RealityView] Status attachment builder called")
                return StatusOverlay(
                    showVideoStatus: false, 
                    isMinimized: $isMinimized,
                    showViewControls: $showViewControls,
                    previewZDistance: $previewZDistance,
                    previewActive: $previewActive,
                    userInteracted: $userInteracted,
                    videoFixed: .constant(false),
                    previewStatusPosition: $previewStatusPosition,
                    previewStatusActive: $previewStatusActive
                )
            }
            
            Attachment(id: "statusPreview") {
                StatusPreviewView(
                    showVideoStatus: false,
                    videoFixed: false
                )
            }
        }
        .gesture(
            TapGesture()
                .targetedToAnyEntity()
        )
        .task { self.model.run() }
        .task { await self.model.processDeviceAnchorUpdates() }
        // .task { self.model.startserver() }
        .task(priority: .low) { await self.model.processReconstructionUpdates() }
        .upperLimbVisibility(dataManager.upperLimbVisible ? .visible : .hidden)
    }
    static let resultLabelID: String = "resultLabel"
    static let statusAttachmentID: String = "status"
}



