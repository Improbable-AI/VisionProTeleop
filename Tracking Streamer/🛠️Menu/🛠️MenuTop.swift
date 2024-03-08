import SwiftUI

struct 🛠️MenuTop: View {
//    @EnvironmentObject var model: 🥽AppModel
    @Environment(\.dismissImmersiveSpace) var dismissImmersiveSpace
    var body: some View {
        VStack(spacing: 24) {
            HStack(spacing: 28) {
                Button {
                    Task { await self.dismissImmersiveSpace() }
                } label: {
                    HStack(spacing: 8) {
                        Image(systemName: "escape")
                            .imageScale(.small)
                        Text("Exit")
                    }
                    .font(.title.weight(.regular))
                    .padding(.vertical, 12)
                    .padding(.horizontal, 20)
                }
                .buttonStyle(.plain)
                .glassBackgroundEffect()}
        }
    }
}

