//
//  CustomVideoPlayer.swift
//  Tracking Viewer
//
//  Created on 12/2/25.
//

import SwiftUI
import AVKit

/// A custom video player view that supports resizeAspectFill and has no controls
struct CustomVideoPlayer: UIViewRepresentable {
    let player: AVPlayer
    
    func makeUIView(context: Context) -> PlayerUIView {
        return PlayerUIView(player: player)
    }
    
    func updateUIView(_ uiView: PlayerUIView, context: Context) {
        uiView.playerLayer.player = player
    }
    
    class PlayerUIView: UIView {
        let playerLayer = AVPlayerLayer()
        
        init(player: AVPlayer) {
            super.init(frame: .zero)
            playerLayer.player = player
            playerLayer.videoGravity = .resizeAspectFill // This makes the video fill the frame
            layer.addSublayer(playerLayer)
        }
        
        required init?(coder: NSCoder) {
            fatalError("init(coder:) has not been implemented")
        }
        
        override func layoutSubviews() {
            super.layoutSubviews()
            playerLayer.frame = bounds
        }
    }
}
