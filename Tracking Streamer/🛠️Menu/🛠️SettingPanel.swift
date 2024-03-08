import SwiftUI

struct 🛠️SettingPanel: View {
//    @EnvironmentObject var model: 🥽AppModel
    var body: some View {
        VStack(spacing: 24) {
            HStack {
                Spacer()
                Text("Unit")
                    .font(.largeTitle.weight(.semibold))
                Spacer()
            }
            .frame(height: 60)
        }
    }
}
