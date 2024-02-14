import SwiftUI

struct 🛠️AboutPanel: View {
//    @EnvironmentObject var model: 🥽AppModel
    var body: some View {
        VStack(spacing: 24) {
            HStack {
                Spacer()
                Text("HandsWidth")
                    .font(.largeTitle.weight(.semibold))
                Spacer()
            }
            .frame(height: 60)
            HStack(spacing: 32) {
                VStack(spacing: 12) {
                    Image(.graph1)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .frame(width: 300)
                        .clipShape(.rect(cornerRadius: 24))
                    Text("Measurement of the distance between the fingers.")
                        .font(.caption)
                        .multilineTextAlignment(.center)
                }
                VStack(spacing: 12) {
                    Image(.graph2)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .frame(width: 300)
                        .clipShape(.rect(cornerRadius: 24))
                    Text("Fix / Unfix a pointer by indirect tap.")
                        .font(.caption)
                        .multilineTextAlignment(.center)
                }
            }
            .padding(.horizontal)
//            switch self.model.authorizationStatus {
//                case .notDetermined, .denied:
//                    HStack(spacing: 24) {
//                        Text("Hand tracking authorization:")
//                            .fontWeight(.semibold)
//                        Text(self.model.authorizationStatus?.description ?? "nil")
//                    }
//                    .font(.caption)
//                    .foregroundStyle(.secondary)
//                default:
//                    EmptyView()
//            }
        }
    }
}
