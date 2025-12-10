import numpy as np
import torch
import os
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="This script demonstrates adding a Kuka Allegro robot to an Isaac Lab environment and streaming it."
)
parser.add_argument("--num-envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--task", type=str, default="nonprehensile", help="Task to run.")
parser.add_argument("--ip", type=str, default=None, help="IP address of the Vision Pro device.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
args = args_cli 
args.torch_deterministic = True
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import omni
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR, ISAAC_NUCLEUS_DIR
from isaaclab.sim.spawners.materials import RigidBodyMaterialCfg
from isaaclab.utils.math import quat_from_matrix

# -----------------------------------------------------------------------------
# Asset Configuration (Kuka Allegro)
# -----------------------------------------------------------------------------

KUKA_ALLEGRO_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/KukaAllegro/kuka.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            retain_accelerations=True,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1000.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=1,
            sleep_threshold=0.005,
            stabilization_threshold=0.0005,
        ),
        joint_drive_props=sim_utils.JointDrivePropertiesCfg(drive_type="force"),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(1.0, 0.0, 0.0, 0.0),
        joint_pos={
            "iiwa7_joint_(1|2|7)": 0.0,
            "iiwa7_joint_3": 0.7854,
            "iiwa7_joint_4": 1.5708,
            "iiwa7_joint_(5|6)": -1.5708,
            "(index|middle|ring)_joint_0": 0.0,
            "(index|middle|ring)_joint_1": 0.3,
            "(index|middle|ring)_joint_2": 0.3,
            "(index|middle|ring)_joint_3": 0.3,
            "thumb_joint_0": 1.5,
            "thumb_joint_1": 0.60147215,
            "thumb_joint_2": 0.33795027,
            "thumb_joint_3": 0.60845138,
        },
    ),
    actuators={
        "kuka_allegro_actuators": ImplicitActuatorCfg(
            joint_names_expr=[
                "iiwa7_joint_(1|2|3|4|5|6|7)",
                "index_joint_(0|1|2|3)",
                "middle_joint_(0|1|2|3)",
                "ring_joint_(0|1|2|3)",
                "thumb_joint_(0|1|2|3)",
            ],
            effort_limit_sim={
                "iiwa7_joint_(1|2|3|4|5|6|7)": 300.0,
                "index_joint_(0|1|2|3)": 0.5,
                "middle_joint_(0|1|2|3)": 0.5,
                "ring_joint_(0|1|2|3)": 0.5,
                "thumb_joint_(0|1|2|3)": 0.5,
            },
            stiffness={
                "iiwa7_joint_(1|2|3|4)": 300.0,
                "iiwa7_joint_5": 100.0,
                "iiwa7_joint_6": 50.0,
                "iiwa7_joint_7": 25.0,
                "index_joint_(0|1|2|3)": 3.0,
                "middle_joint_(0|1|2|3)": 3.0,
                "ring_joint_(0|1|2|3)": 3.0,
                "thumb_joint_(0|1|2|3)": 3.0,
            },
            damping={
                "iiwa7_joint_(1|2|3|4)": 45.0,
                "iiwa7_joint_5": 20.0,
                "iiwa7_joint_6": 15.0,
                "iiwa7_joint_7": 15.0,
                "index_joint_(0|1|2|3)": 0.1,
                "middle_joint_(0|1|2|3)": 0.1,
                "ring_joint_(0|1|2|3)": 0.1,
                "thumb_joint_(0|1|2|3)": 0.1,
            },
            friction={
                "iiwa7_joint_(1|2|3|4|5|6|7)": 1.0,
                "index_joint_(0|1|2|3)": 0.01,
                "middle_joint_(0|1|2|3)": 0.01,
                "ring_joint_(0|1|2|3)": 0.01,
                "thumb_joint_(0|1|2|3)": 0.01,
            },
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)

# -----------------------------------------------------------------------------
# Scene Configuration
# -----------------------------------------------------------------------------

class KukaAllegroSceneCfg(InteractiveSceneCfg):
    """Designs the scene."""

    # Ground-plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg(size = (10.0, 10.0), 
    physics_material = RigidBodyMaterialCfg(static_friction = 1.0, dynamic_friction = 1.0)))

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # Robot
    Robot = KUKA_ALLEGRO_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")\
                            .replace(init_state = ArticulationCfg.InitialStateCfg(
                                            pos=(0.4, -0.15, 0.3),  # Bump up z slightly
                                            rot=(0.7071068, 0, 0, 0.7071068), # Rotate to face forward
                                            # Keep the default joint pos from the config for now
                                            joint_pos={
                                                "iiwa7_joint_(1|2|7)": 0.0,
                                                "iiwa7_joint_3": 0.7854,
                                                "iiwa7_joint_4": 1.5708,
                                                "iiwa7_joint_(5|6)": -1.5708,
                                                "(index|middle|ring)_joint_0": 0.0,
                                                "(index|middle|ring)_joint_1": 0.3,
                                                "(index|middle|ring)_joint_2": 0.3,
                                                "(index|middle|ring)_joint_3": 0.3,
                                                "thumb_joint_0": 1.5,
                                                "thumb_joint_1": 0.60147215,
                                                "thumb_joint_2": 0.33795027,
                                                "thumb_joint_3": 0.60845138,
                                            },
                            ))

    # Reference Object (Optional, kept from example)
    Object = RigidObjectCfg(
        prim_path="/World/envs/env_.*/object",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=False,
                disable_gravity=False,
                enable_gyroscopic_forces=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
                sleep_threshold=0.005,
                stabilization_threshold=0.0025,
                max_depenetration_velocity=1000.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(density=400.0),
            scale=(1.2, 1.2, 1.2),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.17, 0.1), rot=(1.0, 0.0, 0.0, 0.0)),
    )

# -----------------------------------------------------------------------------
# Environment
# -----------------------------------------------------------------------------

class MyIsaacEnv: 
    def __init__(self, args_cli): 
        self.args_cli = args_cli
        gravity_direction = np.array([0, 0, -1])
        magnitude = 9.81
        gravity = magnitude * gravity_direction / np.linalg.norm(gravity_direction)

        self.sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt = 0.01, gravity=gravity)
        self.sim_cfg.physx.gpu_max_rigid_patch_count = 4096 * 4096
        self.sim = sim_utils.SimulationContext(self.sim_cfg)
        self.sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
        self.decimation = 2
        
        # Design scene
        self.scene_cfg = KukaAllegroSceneCfg(self.args_cli.num_envs, env_spacing=2.0, replicate_physics = True)
        self.scene = InteractiveScene(self.scene_cfg)

        self.sim.reset()

        # Kuka Allegro has 23 joints (7 arm joints + 16 hand joints)
        # We'll use a placeholder init_pos for control logic.
        # The robot spawns in the config's init state, but we need a tensor for the 'step' noise.
        # Order: iiwa (7), index (4), middle (4), ring (4), thumb (4) assuming alphabetical/config order?
        # Isaac Lab sorts joints. We will just use zeros mostly as a baseline for noise.
        
        self.num_dof = 23
        self.init_pos = torch.zeros(self.num_dof).to(args_cli.device)
        
        # Populate with some reasonable defaults if we wanted, but sticking to 0 for simplicity/relative mvt.
        # Actually, let's read the current position after reset to be safe.
        self.scene.update(0.0)
        self.init_pos = self.scene["Robot"].data.joint_pos[0].clone()

        self.scene.reset()

    def step(self, action):
        """
        action: (num_envs, num_dof)
        """
        # Apply action directly for now (position control)
        # We assume the action input is the full joint state target
        
        for _ in range(self.decimation): 
            self.scene["Robot"].set_joint_position_target(action)
            self.scene.write_data_to_sim()
            self.sim.step()
            self.scene.update(self.sim.get_physics_dt())

if __name__ == "__main__":
    from avp_stream import VisionProStreamer

    my_env = MyIsaacEnv(args_cli)

    if args.ip is not None:
        # Configure Isaac Lab streaming with the scene
        streamer = VisionProStreamer(ip=args.ip)
        streamer.configure_isaac(
            scene=my_env.scene,           # Scene is required for live articulation poses
            relative_to=[0, 0, 0.7, 90],  # Position the robot in front of the user
            include_ground=False,         # Don't include ground plane
            env_indices=[0],              # Only stream the first environment
        )
        streamer.start_webrtc()
        
    while True: 
        # Create a dummy action that just holds the initial pose with slight noise to show aliveness
        action = my_env.init_pos.unsqueeze(0).repeat(args_cli.num_envs, 1)
        
        # Add small noise
        noise = 0.02 * (torch.rand_like(action) - 0.5)
        action = action + noise
        
        my_env.step(action)
                
        if args.ip is not None:
            streamer.update_sim()
