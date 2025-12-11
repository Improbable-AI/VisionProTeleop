import SwiftUI
import RealityKit

/// View control settings panel for adjusting video plane position
struct ViewControlSettings: View {
    @ObservedObject var dataManager = DataManager.shared
    @Binding var isPresented: Bool
    @Binding var previewZDistance: Float?
    @State private var hidePreviewTask: Task<Void, Never>?
    
    var body: some View {
        VStack(alignment: .leading, spacing: 16) {
            HStack {
                Text("View Controls")
                    .font(.headline)
                    .foregroundColor(.white)
                Spacer()
                Button {
                    dlog("❌ [ViewControlSettings] Close button pressed")
                    withAnimation(.spring(response: 0.45, dampingFraction: 0.85)) {
                        isPresented = false
                    }
                    previewZDistance = nil  // Hide preview when closing
                    hidePreviewTask?.cancel()
                } label: {
                    ZStack {
                        Circle()
                            .fill(Color.white.opacity(0.3))
                            .frame(width: 30, height: 30)
                        Image(systemName: "xmark")
                            .font(.system(size: 12, weight: .bold))
                            .foregroundColor(.white)
                    }
                }
                .buttonStyle(.plain)
            }
            
            Divider()
                .background(Color.white.opacity(0.3))
            
            VStack(alignment: .leading, spacing: 12) {
                Text("Video Distance")
                    .font(.subheadline)
                    .foregroundColor(.white.opacity(0.8))
                
                HStack {
                    Text("\(String(format: "%.1f", -dataManager.videoPlaneZDistance))m")
                        .font(.system(size: 14, weight: .medium, design: .monospaced))
                        .foregroundColor(.white)
                        .frame(width: 50, alignment: .leading)
                    
                    Slider(value: Binding(
                        get: { -dataManager.videoPlaneZDistance },
                        set: { positiveValue in
                            let negativeValue = -positiveValue
                            dlog("🎚️ [ViewControlSettings] Slider value changing to: \(positiveValue)m (internal: \(negativeValue))")
                            dataManager.videoPlaneZDistance = negativeValue
                            // Update preview when dragging
                            previewZDistance = negativeValue
                            dlog("🎚️ [ViewControlSettings] Preview z-distance set to: \(String(describing: previewZDistance))")
                            
                            // Cancel any existing hide task
                            hidePreviewTask?.cancel()
                            
                            // Schedule hiding the preview after 3 seconds of inactivity
                            hidePreviewTask = Task { @MainActor in
                                try? await Task.sleep(nanoseconds: 3_000_000_000)
                                if !Task.isCancelled {
                                    dlog("🎚️ [ViewControlSettings] Hiding preview after inactivity")
                                    previewZDistance = nil
                                }
                            }
                        }
                    ), in: 2.0...20.0, step: 0.5)
                    .tint(.blue)
                }
                
                HStack {
                    Text("Near (2m)")
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.6))
                    Spacer()
                    Text("Far (20m)")
                        .font(.caption2)
                        .foregroundColor(.white.opacity(0.6))
                }
            }
            
            Divider()
                .background(Color.white.opacity(0.3))
            
            HStack {
                Spacer()
                Button {
                    dlog("🎚️ [ViewControlSettings] Reset button pressed")
                    dataManager.videoPlaneZDistance = -10.0
                    previewZDistance = -10.0
                    
                    // Hide preview after showing reset position
                    hidePreviewTask?.cancel()
                    hidePreviewTask = Task { @MainActor in
                        try? await Task.sleep(nanoseconds: 3_000_000_000)
                        if !Task.isCancelled {
                            previewZDistance = nil
                        }
                    }
                } label: {
                    Text("Reset to Default")
                        .font(.subheadline)
                        .foregroundColor(.blue)
                        .padding(.horizontal, 12)
                        .padding(.vertical, 6)
                        .background(Color.white.opacity(0.2))
                        .cornerRadius(8)
                }
                .buttonStyle(.plain)
                Spacer()
            }
            
            Text("💡 A gray preview will appear showing the video position")
                .font(.caption)
                .foregroundColor(.white.opacity(0.6))
                .multilineTextAlignment(.leading)
        }
        .padding(20)
        .background(Color.black.opacity(0.8))
        .cornerRadius(16)
        .frame(width: 320)
    }
}
