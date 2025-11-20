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
        } update: { updateContent, attachments in
            // Explicitly depend on isMinimized to trigger updates
            let _ = isMinimized
            let _ = showViewControls
            
            // Update status container position based on minimized state
            if let statusAnchor = updateContent.entities.first(where: { $0.name == "statusHeadAnchor" }) as? AnchorEntity {
                if let statusContainer = statusAnchor.children.first(where: { $0.name == "statusContainer" }) {
                    // Move to y=0.5 when minimized
                    let yPosition: Float = isMinimized ? 0.3 : 0.0
                    let targetTranslation = SIMD3<Float>(0.0, yPosition, -1.0)
                    
                    // Animate the position change
                    var transform = statusContainer.transform
                    transform.translation = targetTranslation
                    statusContainer.move(to: transform, relativeTo: statusContainer.parent, duration: 0.5, timingFunction: .easeInOut)
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
                    userInteracted: $userInteracted
                )
                .frame(maxWidth: 300)
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
    }
    static let resultLabelID: String = "resultLabel"
    static let statusAttachmentID: String = "status"
}



