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
//                Button {
//                    self.model.presentPanel = .setting
//                } label: {
//                    HStack(spacing: 8) {
//                        Image(systemName: "gearshape")
//                            .imageScale(.small)
//                        Text("Setting")
//                    }
//                    .font(.title.weight(.regular))
//                    .padding(.vertical, 12)
//                    .padding(.horizontal, 20)
//                }
//                .buttonStyle(.plain)
//                .disabled(self.model.presentPanel == .setting)
//                .glassBackgroundEffect()
//                Button {
//                    self.model.presentPanel = .about
//                } label: {
//                    HStack(spacing: 8) {
//                        Image(systemName: "questionmark")
//                            .imageScale(.small)
//                        Text("About")
//                    }
//                    .font(.title.weight(.regular))
//                    .padding(.vertical, 12)
//                    .padding(.horizontal, 20)
//                }
//                .buttonStyle(.plain)
//                .disabled(self.model.presentPanel == .about)
//                .glassBackgroundEffect()
//            }
//            ZStack(alignment: .top) {
//                🛠️SettingPanel()
//                    .overlay(alignment: .topTrailing) { self.hideButton() }
//                    .padding(32)
//                    .padding(.horizontal)
//                    .fixedSize()
//                    .glassBackgroundEffect()
//                    .opacity(self.model.presentPanel == .setting ? 1 : 0)
//                🛠️AboutPanel()
//                    .overlay(alignment: .topTrailing) { self.hideButton() }
//                    .padding(24)
//                    .padding(.horizontal)
//                    .fixedSize()
//                    .glassBackgroundEffect()
//                    .opacity(self.model.presentPanel == .about ? 1 : 0)
//            }
        }
//        .animation(.default, value: self.model.presentPanel)
//        .offset(y: -2000)
//        .offset(z: -700)
    }
}

//private extension 🛠️MenuTop {
//    private func hideButton() -> some View {
//        Button {
////            self.model.presentPanel = nil
//        } label: {
//            Image(systemName: "arrow.down.right.and.arrow.up.left")
//                .padding()
//        }
//        .buttonBorderShape(.circle)
//        .buttonStyle(.plain)
//        .frame(width: 60, height: 60)
//    }
//}
