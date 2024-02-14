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
//            Picker("Unit", selection: self.$model.unit) {
//                ForEach(📏Unit.allCases) {
//                    Text($0.value.symbol)
//                }
//            }
//            .pickerStyle(.segmented)
//            .frame(height: 60)
//            .frame(width: 360)
        }
    }
}
