/*
See the LICENSE.txt file for this sample’s licensing information.

Abstract:
The input selection and waiting screen before the game starts.
*/

import SwiftUI

struct Lobby: View {
    @Environment(GameModel.self) var gameModel
    
    var progressValue: Float {
        min(1, max(0, Float(gameModel.countDown) / 3.0 + 0.01))
    }
    
    var body: some View {
        if !gameModel.isInputSelected {
            inputSelection
                .frame(width: 634, height: 499)
        } else {
            if gameModel.isSharePlaying {
                multiWaiting
                    .frame(width: 634, height: 499)
            } else { // Solo
                Gauge(value: progressValue) {
                    EmptyView()
                }
                .labelsHidden()
                .animation(.default, value: progressValue)
                .gaugeStyle(.accessoryCircularCapacity)
                .scaleEffect(x: 3, y: 3, z: 1)
                .frame(width: 150, height: 150)
                .padding(75)
                .overlay {
                    Text(verbatim: "\(gameModel.countDown)")
                        .animation(.none, value: progressValue)
                        .font(.system(size: 64))
                        .bold()
                }
                .frame(width: 634, height: 499)
                .accessibilityHidden(true)
            }
        }
    }
    
    var inputSelection: some View {
        VStack {
            Text("Choose how you’ll cheer up grumpy clouds.")
                .font(.title)
                .padding(.top, 40)
                .padding(.bottom, 30)
            HStack(alignment: .top, spacing: 30) {
                VStack {
                    Button {
                        chooseInputAndReady(.hands)
                    } label: {
                        Label {
                            Text("Make a heart with two hands.")
                        } icon: {
                            Image("gesture_hand")
                                .resizable()
                                .aspectRatio(contentMode: .fit)
                                .frame(width: 216, height: 216)
                                .scaleEffect(x: 1.18, anchor: .center)
                                .offset(y: 30)
                        }
                        .labelStyle(.iconOnly)
                    }
                    .buttonBorderShape(.roundedRectangle(radius: 28))
                    .padding(.bottom, 10)
                    
                    Text("Make a heart with two hands.")
                        .font(.headline)
                        .frame(width: 216)
                        .accessibilityHidden(true)
                }

                VStack {
                    Button {
                        chooseInputAndReady(.alternative)
                    } label: {
                        Label {
                            Text("Use a pinch gesture or a compatible device.")
                        } icon: {
                            Image("keyboardGameController")
                                .resizable()
                                .aspectRatio(contentMode: .fit)
                                .frame(width: 216, height: 216)
                        }
                        .labelStyle(.iconOnly)
                    }
                    .buttonBorderShape(.roundedRectangle(radius: 28))
                    .padding(.bottom, 10)
                    
                    Text("Use a pinch gesture or a compatible device.")
                        .font(.headline)
                        .frame(width: 216)
                        .accessibilityHidden(true)
                }
            }
            .multilineTextAlignment(.center)
            .padding(.horizontal, 20)
        }
    }
    
    func chooseInputAndReady(_ kind: InputKind) {
        gameModel.isInputSelected = true
        gameModel.inputKind = kind
        
        if gameModel.isSharePlaying {
            multiReady()
        } else {
            // Delay three seconds, then...
            gameModel.isCountDownReady = true
        }
    }
    
    func multiReady() {
        print("Sending local ready message for: ", sessionInfo?.session?.localParticipant.id.asPlayerName as Any)
        
        guard let localPlayer = gameModel.players.first(where: { $0.name == Player.localName }) else {
            print("Local Player isn't set")
            return
        }
        
        localPlayer.isReady = true
        gameModel.players = gameModel.players.filter { _ in true }
        sessionInfo?.reliableMessenger?.send(ReadyStateMessage(ready: true)) { error in
            if error != nil {
                print("Send score error:", error!)
            }
        }
    }
    
    var multiWaiting: some View {
        VStack(spacing: 20) {
            Image("shareplayGraphic")
            Text("Waiting for all players to choose.")
                .font(.title)
            HStack(spacing: 10) {
                ForEach(gameModel.players, id: \.name) { player in
                    if player.isReady {
                        Image(systemName: "checkmark.circle")
                            .foregroundColor(.green)
                    } else {
                        ProgressView()
                    }
                }
            }
        }
    }
}

#Preview {
    Lobby()
        .environment(GameModel())
        .glassBackgroundEffect(
            in: RoundedRectangle(
                cornerRadius: 32,
                style: .continuous
            )
        )
}
