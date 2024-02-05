/*
See the LICENSE.txt file for this sample’s licensing information.

Abstract:
Declarations and parameters for clouds and their movement.
*/

import Accessibility
import Spatial
import RealityKit

/// A data source to sync cloud information among multiple players.
class Cloud: Identifiable {
    var id: Int
    var isHappy: Bool
    
    init(id: Int, isHappy: Bool) {
        self.id = id
        self.isHappy = isHappy
    }
}

/// The main cloud model; it's cloned when a new cloud spawns.
var cloudTemplate: Entity? = nil
var cloudNumber = 0

/// Creates a cloud and places it in the space.
@MainActor
func spawnCloud() async throws -> Entity {
    let start = Point3D(
        x: cloudPaths[cloudPathsIndex].0,
        y: cloudPaths[cloudPathsIndex].1,
        z: cloudPaths[cloudPathsIndex].2
    )
    
    let cloud = try await spawnCloudExact(
        start: start,
        end: .init(
            x: start.x + CloudSpawnParameters.deltaX,
            y: start.y + CloudSpawnParameters.deltaY,
            z: start.z + CloudSpawnParameters.deltaZ
        ),
        speed: CloudSpawnParameters.speed
    )
    
    // Needs to increment *after* spawnCloudExact()
    cloudPathsIndex += 1
    cloudPathsIndex %= cloudPaths.count
    
    cloudEntities.append(cloud)
    return cloud
}

/// Storage for each of the linear cloud movement animations.
var cloudMovementAnimations: [AnimationResource] = []

/// Places a cloud in the scene and sets it on a set journey.
@MainActor
func spawnCloudExact(start: Point3D, end: Point3D, speed: Double) async throws -> Entity {
    if cloudTemplate == nil {
        guard let cloud = await loadFromRealityComposerPro(
            named: BundleAssets.cloudEntity,
            fromSceneNamed: BundleAssets.cloudScene
        ) else {
            fatalError("Error loading cloud from Reality Composer Pro project.")
        }
        cloudTemplate = cloud
    }
    guard let cloudTemplate = cloudTemplate else {
        fatalError("Cloud template is nil.")
    }
    
    let cloud = cloudTemplate.clone(recursive: true)
    cloud.generateCollisionShapes(recursive: true)
    cloud.name = "CCloud\(cloudNumber)"
    cloudNumber += 1
    
    cloud.components[PhysicsBodyComponent.self] = PhysicsBodyComponent()
    cloud.scale = .init(repeating: 0.001)
    
    cloud.position = simd_float(start.vector + .init(x: 0, y: 0, z: -0.7))
    
    var accessibilityComponent = AccessibilityComponent()
    accessibilityComponent.label = "Cloud"
    accessibilityComponent.value = "Grumpy"
    accessibilityComponent.isAccessibilityElement = true
    accessibilityComponent.traits = [.button, .playsSound]
    accessibilityComponent.systemActions = [.activate]
    cloud.components[AccessibilityComponent.self] = accessibilityComponent
    
    let animation = cloudMovementAnimations[cloudPathsIndex]
    
    cloud.playAnimation(animation, transitionDuration: 1.0, startsPaused: false)
    cloud.setMaterialParameterValues(parameter: "saturation", value: .float(0.0))
    cloud.setMaterialParameterValues(parameter: "animate_texture", value: .bool(false))
    
    cloudAnimate(cloud, kind: .sadBlink, shouldRepeat: false)
    
    spaceOrigin.addChild(cloud)
    
    return cloud
}

/// Describes the 3D scene relative to the player.
func postCloudOverviewAnnouncement(gameModel: GameModel) {
    guard !cloudEntities.isEmpty else {
        return
    }
    var averageCameraPositionFront: SIMD3<Float> = [0, 0, 0]
    var averageCameraPositionBehind: SIMD3<Float> = [0, 0, 0]
    var cloudsFront = 0
    var cloudsBehind = 0
    for cloud in cloudEntities {
        let cloudInstance = gameModel.clouds.first(where: { cloudInstance in
            if ("CCloud" + String(cloudInstance.id)) == cloud.name {
                return true
            }
            return false
        })
        if cloudInstance?.isHappy ?? false {
            continue
        }
        let cloudPosition = cloud.position(relativeTo: cameraAnchor)
        if cloudPosition.z > 0 {
            averageCameraPositionBehind += cloudPosition
            cloudsBehind += 1
        } else {
            averageCameraPositionFront += cloudPosition
            cloudsFront += 1
        }
    }
    averageCameraPositionFront /= [Float(cloudsFront), Float(cloudsFront), Float(cloudsFront)]
    var cloudPositioningAnnouncementFront = String(format: "%d \(cloudsFront > 1 ? "clouds" : "cloud")", cloudsFront)
    if averageCameraPositionFront.y > 0.5 {
        cloudPositioningAnnouncementFront += " above and in front of you "
    } else if averageCameraPositionFront.y < -0.5 {
        cloudPositioningAnnouncementFront += " below and in front of you "
    } else {
        cloudPositioningAnnouncementFront += " in front of you "
    }
    
    if averageCameraPositionFront.x > 0.5 {
        cloudPositioningAnnouncementFront += "to the right"
    } else if averageCameraPositionFront.x < -0.5 {
        cloudPositioningAnnouncementFront += "to the left"
    }
    
    averageCameraPositionBehind /= [Float(cloudsBehind), Float(cloudsBehind), Float(cloudsBehind)]
    var cloudPositioningAnnouncementBehind = String(format: "%d \(cloudsBehind > 1 ? "clouds" : "cloud")", cloudsBehind)
    if averageCameraPositionBehind.y > 0.5 {
        cloudPositioningAnnouncementBehind += " above and behind you "
    } else if averageCameraPositionBehind.y < -0.5 {
        cloudPositioningAnnouncementBehind += " below and behind you "
    } else {
        cloudPositioningAnnouncementBehind += " behind you "
    }
    
    if averageCameraPositionBehind.x > 0.5 {
        cloudPositioningAnnouncementBehind += "to the right"
    } else if averageCameraPositionBehind.x < -0.5 {
        cloudPositioningAnnouncementBehind += "to the left"
    }
    
    var cloudPositioningAnnouncement = ""
    if cloudsFront > 0 && cloudsBehind == 0 {
        cloudPositioningAnnouncement = cloudPositioningAnnouncementFront
    } else if cloudsBehind > 0 && cloudsFront == 0 {
        cloudPositioningAnnouncement = cloudPositioningAnnouncementBehind
    } else {
        cloudPositioningAnnouncement = cloudPositioningAnnouncementFront + " " + cloudPositioningAnnouncementBehind
    }
    
    AccessibilityNotification.Announcement(cloudPositioningAnnouncement).post()
}

/// Plays one of the cloud animations on the cloud you specify.
@MainActor func cloudAnimate(_ cloud: Entity, kind: CloudAnimations, shouldRepeat: Bool) {
    guard let animation = cloudAnimations[kind] else {
        fatalError("Tried to load an animation that doesn't exist: \(kind)")
    }
    
    if shouldRepeat {
        cloud.playAnimation(animation.repeat(count: 100))
    } else {
        cloud.playAnimation(animation)
    }
}

/// A map from a kind of animation to the animation resource that contains that animation.
var cloudAnimations: [CloudAnimations: AnimationResource] = [:]

/// The available animations inside the cloud asset.
enum CloudAnimations {
    case sadBlink
    case smile
    case happyBlink
}

/// Cloud spawn parameters (in meters).
struct CloudSpawnParameters {
    static var deltaX = 0.02
    static var deltaY = -0.12
    static var deltaZ = 12.0
    
    static var speed = 11.73
}

/// A counter that advances to the next cloud path.
var cloudPathsIndex = 0

/// A hand-picked selection of random starting parameters for the motion of the clouds.
let cloudPaths: [(Double, Double, Double)] = [
    (x: 1.757_231_498_429_01, y: 1.911_673_694_896_59, z: -8.094_368_331_589_704),
    (x: -0.179_269_237_592_594_17, y: 1.549_268_306_906_908_4, z: -7.254_713_426_424_875),
    (x: -0.013_296_800_013_828_491, y: 2.147_766_026_068_617_8, z: -8.601_541_438_900_849),
    (x: 2.228_704_746_539_703, y: 0.963_797_733_336_365_2, z: -7.183_621_312_117_454),
    (x: -0.163_925_123_812_864_4, y: 1.821_619_897_406_197, z: -8.010_893_563_433_282),
    (x: 0.261_716_575_589_896_03, y: 1.371_932_443_334_715, z: -7.680_206_361_333_17),
    (x: 1.385_410_631_256_254_6, y: 1.797_698_998_556_775_5, z: -7.383_548_882_448_866),
    (x: -0.462_798_470_454_367_4, y: 1.431_650_092_907_264_4, z: -7.169_154_476_151_876),
    (x: 1.112_766_805_791_563, y: 0.859_548_406_627_492_2, z: -7.147_229_496_720_969),
    (x: 1.210_194_536_657_374, y: 0.880_254_638_358_228_8, z: -8.051_132_737_691_349),
    (x: 0.063_637_772_899_141_52, y: 1.973_172_635_040_014_7, z: -8.503_837_407_474_947),
    (x: 0.883_082_630_134_997_2, y: 1.255_268_496_843_653_4, z: -7.760_994_300_660_705),
    (x: 0.891_719_821_716_725_7, y: 2.085_000_111_104_786_7, z: -8.908_048_018_555_112),
    (x: 0.422_260_067_132_894_2, y: 1.370_335_319_771_187, z: -7.525_853_388_894_509),
    (x: 0.473_470_811_107_753_46, y: 1.864_930_149_962_240_6, z: -8.164_641_191_459_626)
]
