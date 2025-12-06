//
//  Tracking_ViewerApp.swift
//  Tracking Viewer
//
//  Created by younghyopark on 11/29/25.
//

import SwiftUI

@main
struct Tracking_ViewerApp: App {
    @State private var isOnboardingComplete = hasCompletedOnboarding()
    
    var body: some Scene {
        WindowGroup {
            Group {
                if isOnboardingComplete {
                    ContentView()
                } else {
                    WelcomeOnboardingView(isOnboardingComplete: $isOnboardingComplete)
                }
            }
            .onOpenURL { url in
                // Handle Dropbox OAuth callback
                if url.scheme == "visionproteleop" && url.host == "dropbox" {
                    Task {
                        await DropboxManager.shared.handleCallback(url: url)
                    }
                }
            }
        }
    }
}
