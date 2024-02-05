/*
See the LICENSE.txt file for this sample’s licensing information.

Abstract:
The score screen for multiplayer.
*/

import SwiftUI

struct MultiScore: View {
    @Environment(GameModel.self) var gameModel
    @Environment(\.dismissImmersiveSpace) var dismissImmersiveSpace
    
    var body: some View {
        VStack(spacing: 10) {
            Image("greatJob")
                .resizable()
                .aspectRatio(contentMode: .fit)
                .frame(width: 450, height: 200, alignment: .center)
                .accessibilityHidden(true)
            
            if youWon {
                Text("You won!")
                    .font(.system(size: 30, weight: .bold))
                    .offset(y: -13)
            } else {
                Text("\(fantasyName(for: sortedByScore.first!, in: gameModel.players)) won!")
                    .font(.system(size: 30, weight: .bold))
                    .offset(y: -13)
            }

            ForEach(sortedByScore, id: \.name) { player in
                HStack {
                    Text("\(sortedByScore.firstIndex(where: { $0.name == player.name })! + 1). " + fantasyName(for: player, in: gameModel.players)
                         + (player.name == you.name
                         ? "(you) \(player.score)"
                         : " \(player.score)"))

                }
                .font(.headline)
            }
            .frame(minWidth: 0, maxWidth: 135, alignment: .leading)
            .offset(y: -13)
            
            Group {
                Button {
                    playAgain()
                } label: {
                    Text("Play Again")
                        .frame(maxWidth: .infinity)
                }
                Button {
                    Task {
                        await goBackToStart()
                    }
                } label: {
                    Text("Back to Main Menu")
                        .frame(maxWidth: .infinity)
                }
            }
            .frame(width: 220)
        }
        .padding(15)
        .frame(width: 634, height: 550)
    }
    
    var you: Player {
        #if targetEnvironment(simulator)
        gameModel.players.first!
        #else
        gameModel.players.first(where: { $0.name == Player.localName })!
        #endif
    }
    
    var youWon: Bool {
        sortedByScore.first!.name == you.name
    }
    
    var sortedByScore: [Player] {
        gameModel.players.sorted(using: KeyPathComparator(\.score, order: .reverse))
    }
    
    func playAgain() {
        let inputChoice = gameModel.inputKind
        gameModel.reset()
        
        if beamIntermediate.parent == nil {
            spaceOrigin.addChild(beamIntermediate)
        }
        
        gameModel.isPlaying = true
        gameModel.isInputSelected = true
        gameModel.isCountDownReady = true
        gameModel.inputKind = inputChoice
    }
    
    func goBackToStart() async {
        await dismissImmersiveSpace()
        gameModel.reset()
    }
}

#Preview {
    MultiScore()
        .environment(GameModel())
        .glassBackgroundEffect(
            in: RoundedRectangle(
                cornerRadius: 32,
                style: .continuous
            )
        )
}
