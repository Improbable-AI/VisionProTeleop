/*
See the LICENSE.txt file for this sample’s licensing information.

Abstract:
RealityKit entities used throughout the app.
*/

import RealityKit

/// The root entity for entities placed during the game.
let spaceOrigin = Entity()

/// An anchor that helps calculate the position of clouds relative to the player.
let cameraAnchor = AnchorEntity(.head)

/// A container for asset names that the app loads from a bundle at runtime.
struct BundleAssets {
    static let
        heartBlasterScene = "HeartBlaster.usda",
        heartBlasterEntity = "HeartBeam",
        
        heartTurretScene = "HeartTurret.usda",
        heartTurretEntity = "heartTurret",
        
        heartLightScene = "Heart.usda",
        heartLightEntity = "Heart",
        
        cloudScene = "Cloud.usda",
        cloudEntity = "UpdatedGrumpyScene2",
        cloudParameterName = "happiness",
        
        cloud = "Cloud.usdz",
        beamName = "Beam Intermediate"
}

/// The beam entity that's displayed when the player makes a heart gesture.
var beam = Entity()

/// An entity that sits in between the beam and its parent; used to finely adjust the beam position.
let beamIntermediate = Entity()

/// The version of the beam that's attached to the purple holder.
var floorBeam = Entity()
var isFloorBeamShowing = false

// Containers for assets used throughout the game.
var globalFireworks: Entity? = nil
var globalHeart: Entity? = nil
var turret: Entity? = nil
var heart: Entity? = nil
var cloud: Entity? = nil

/// The clouds in the current round of the game.
var cloudEntities: [Entity] = []

/// A map from beam names to entities in multiplayer.
var multiBeamMap: [String: Entity] = [:]

/// Creates a beam for each player in multiplayer as they join the game and play spatially.
@MainActor
func initialBeam(for player: Player) async -> Entity {
    guard let playerBeam = await loadFromRealityComposerPro(
        named: BundleAssets.heartBlasterEntity,
        fromSceneNamed: BundleAssets.heartBlasterScene
    ) else {
        fatalError("Unable to load beam from Reality Composer Pro project.")
    }
    
    let handOrigin = Entity()
    let multiBeamIntermediate = Entity()
    handOrigin.addChild(multiBeamIntermediate)
    multiBeamIntermediate.addChild(playerBeam)
    
    spaceOrigin.addChild(handOrigin)
    
    playerBeam.generateCollisionShapes(recursive: true)
    playerBeam.name = "multibeam-\(player.name)"
    
    return handOrigin
}
