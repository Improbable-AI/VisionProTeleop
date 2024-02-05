/*
See the LICENSE.txt file for this sample’s licensing information.

Abstract:
The play screen for multiplayer.
*/

import SwiftUI

struct MultiPlay: View {
    @Environment(GameModel.self) var gameModel
    @Environment(\.dismissImmersiveSpace) var dismissImmersiveSpace
    
    var body: some View {
        HStack(alignment: .top) {
            VStack(spacing: 0) {
                let progress = Float(gameModel.timeLeft) / Float(GameModel.gameTime)
                HStack(alignment: .top) {
                    Button {
                        Task {
                            await dismissImmersiveSpace()
                        }
                        gameModel.reset()
                    } label: {
                        Label("Back", systemImage: "chevron.backward")
                            .labelStyle(.iconOnly)
                    }
                    .offset(x: -10)

                    VStack {
                        Text(verbatim: "\(String(format: "%02d", gameModel.score))")
                            .font(.system(size: 60))
                            .bold()
                            .accessibilityLabel(Text("Score"))
                            .accessibilityValue(Text("\(gameModel.score)"))
                        Text("\(fantasyName(for: you, in: sortedByScore)) (you)")
                            .font(.system(size: 20))
                            .bold()
                            .accessibilityHidden(true)
                            .offset(y: -5)
                    }
                    .padding(.leading, 0)
                    .padding(.trailing, 60)
                }
                ForEach(gameModel.players.filter { $0.name != you.name }, id: \.name) { player in
                    HStack {
                        Text(fantasyName(for: player, in: gameModel.players))
                        Text("\(player.score)")
                    }
                    .opacity(0.5)
                }
                .font(.headline)
                HStack {
                    Button {
                        gameModel.isMuted.toggle()
                    } label: {
                        Label(
                            gameModel.isMuted ? "Play music" : "Stop music",
                            systemImage: gameModel.isMuted ? "speaker.slash.fill" : "speaker.wave.3.fill"
                        )
                            .labelStyle(.iconOnly)
                    }
                    .padding(.leading, 12)
                    .padding(.trailing, 10)
                    ProgressView(value: (progress > 1.0 || progress < 0.0) ? 1.0 : progress)
                        .contentShape(.accessibility, Capsule().offset(y: -3))
                        .accessibilityLabel("")
                        .accessibilityValue(Text("\(gameModel.timeLeft) seconds remaining"))
                        .tint(Color(uiColor: UIColor(red: 242 / 255, green: 68 / 255, blue: 206 / 255, alpha: 1.0)))
                        .padding(.vertical, 30)
                    Button {
                        gameModel.isPaused.toggle()
                        gameModel.isMuted.toggle()
                    } label: {
                        if gameModel.isPaused {
                            Label("Play", systemImage: "play.fill")
                                .labelStyle(.iconOnly)
                        } else {
                            Label("Pause", systemImage: "pause.fill")
                                .labelStyle(.iconOnly)
                        }
                    }
                    .padding(.trailing, 12)
                    .padding(.leading, 0)
                    .hidden()
                }
                .background(
                    .regularMaterial,
                    in: .rect(
                        topLeadingRadius: 0,
                        bottomLeadingRadius: 12,
                        bottomTrailingRadius: 12,
                        topTrailingRadius: 0,
                        style: .continuous
                    )
                )
                .frame(width: 260, height: 70)
                .offset(y: 15)
            }
            .padding(.vertical, 12)
        }
        .frame(width: 260)
        .task {
            do {
                #if targetEnvironment(simulator)
                let shouldAddProjector = true
                #else
                let shouldAddProjector = gameModel.inputKind == .alternative
                #endif

                if shouldAddProjector, heart != nil {
                    try await addFloorBeamMaterials()
                }
            } catch {
                print(error)
            }
        }
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
}

#Preview {
    MultiPlay()
        .environment(GameModel())
        .glassBackgroundEffect(
            in: RoundedRectangle(
                cornerRadius: 32,
                style: .continuous
            )
        )
}
