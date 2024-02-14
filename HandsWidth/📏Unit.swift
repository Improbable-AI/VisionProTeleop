import Foundation

enum 📏Unit: String {
    case centiMeters, meters, inches, feet, yards
}

extension 📏Unit: CaseIterable, Identifiable {
    var id: Self { self }
    var value: UnitLength {
        switch self {
            case .centiMeters: .centimeters
            case .meters: .meters
            case .inches: .inches
            case .feet: .feet
            case .yards: .yards
        }
    }
}
