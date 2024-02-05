/*
See the LICENSE.txt file for this sample’s licensing information.

Abstract:
Group activities integration.
*/

import Spatial
import GroupActivities
import Foundation

/// Starts a Happy Beam group activity.
func startSession() async throws {
    let activity = HeartProjection()
    let activationSuccess = try await activity.activate()
    print("Group Activities session activation: ", activationSuccess)
}

/// Metadata about the Happy Beam group activity.
struct HeartProjection: GroupActivity {
    var metadata: GroupActivityMetadata {
        var data = GroupActivityMetadata()
        data.title = "Happy Beam"
        data.subtitle = "Project happy hearts to cheer up grumpy clouds."
        data.supportsContinuationOnTV = false
        return data
    }
    static var activityIdentifier = "com.example.apple-samplecode.happybeam"
}

/// A message that contains current information about the position of another player's beam.
struct BeamMessage: Codable {
    let pose: Pose3D
}

/// A message that indicates another player cheered up a particular cloud.
struct ScoreMessage: Codable {
    let cloudID: Int
}

/// A message that indicates another player's current ready state before the game starts.
struct ReadyStateMessage: Codable {
    let ready: Bool
}

/// State information about the current group activity.
var sessionInfo: ProjectionSessionInfo? = nil

/// A container for group activity session information.
class ProjectionSessionInfo: ObservableObject {
    @Published var session: GroupSession<HeartProjection>?
    var messenger: GroupSessionMessenger?
    var reliableMessenger: GroupSessionMessenger?
}
