/*
See the LICENSE.txt file for this sample’s licensing information.

Abstract:
Data structures for multiplayer.
*/

import SwiftUI

/// Data about a player in a multiplayer session.
class Player {
    let name: String
    var score: Int
    let color: Color
    var isReady = false
    
    init(name: String, score: Int, color: Color) {
        self.name = name
        self.score = score
        self.color = color
    }
    
    /// The local player, "me".
    static var localName: String = ""
}

/// A set of fake players for use in previews and testing.
var initialPlayers: [Player] = [
    .init(name: "Altus", score: 7, color: .green),
    .init(name: "Nimbus", score: 2, color: .yellow),
    .init(name: "Cumulus", score: 4, color: .purple),
    .init(name: "Cirrus", score: 5, color: .red)
]

/// Maps player names to friendly cloud names.
///
/// Player names are UUIDs during multiplayer; these themed names improve the presentation.
func fantasyName(for player: Player, in players: [Player]) -> String {
    let fantasyNames = ["Altus", "Nimbus", "Cumulus", "Cirrus"]
    
    let uuids = players.map { $0.name }.sorted()
    return fantasyNames[uuids.firstIndex(of: player.name)!]
}

// A utility to randomly assign players a theme color for some UI presentations.
extension Color {
    static func random() -> Self {
        [.red, .blue, .green, .pink, .purple].randomElement()!
    }
}
