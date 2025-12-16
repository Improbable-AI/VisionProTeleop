"""
Example script demonstrating a Boston Dynamics Spot robot with random leg actuation.

This example shows:
- Setting up the Spot robot in Isaac Lab
- Randomly actuating leg joints to create dynamic movements
- Streaming to Vision Pro

Usage:
    python examples/18_spot_demo.py --num-envs 1
    python examples/18_spot_demo.py --ip <vision_pro_ip>  # Stream to Vision Pro
"""

import numpy as np
import torch
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="Boston Dynamics Spot robot with random leg actuation."
)
parser.add_argument("--num-envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--ip", type=str, default=None, help="IP address of the Vision Pro device.")
parser.add_argument("--random-seed", type=int, default=None, help="Random seed for reproducibility.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Now import Isaac Lab modules
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim.spawners.materials import RigidBodyMaterialCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

# Set random seed for reproducibility
if args_cli.random_seed is not None:
    np.random.seed(args_cli.random_seed)
    torch.manual_seed(args_cli.random_seed)


# =============================================================================
# Spot Robot Configuration (simplified from IsaacLab's spot.py)
# Using ImplicitActuatorCfg for simplicity (the original uses DelayedPDActuatorCfg)
# =============================================================================
SPOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/BostonDynamics/spot/spot.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.6),  # Start slightly above ground
        joint_pos={
            "[fh]l_hx": 0.1,   # all left hip_x
            "[fh]r_hx": -0.1,  # all right hip_x
            "f[rl]_hy": 0.9,   # front hip_y
            "h[rl]_hy": 1.1,   # hind hip_y
            ".*_kn": -1.5,     # all knees
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            effort_limit=45.0,
            stiffness=60.0,
            damping=1.5,
        ),
    },
)


# =============================================================================
# Scene Configuration
# =============================================================================
class SpotDemoSceneCfg(InteractiveSceneCfg):
    """Scene configuration for Spot demonstration."""

    # Ground plane with friction
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            size=(20.0, 20.0),
            physics_material=RigidBodyMaterialCfg(
                static_friction=0.8,
                dynamic_friction=0.6,
                restitution=0.0,
            ),
        ),
    )

    # Dome light for visibility
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    # Spot Robot
    Spot_Robot = SPOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Spot")


# =============================================================================
# Environment Class
# =============================================================================
class SpotDemoEnv:
    """Environment for Spot demonstration with random leg actuation."""

    def __init__(self, args_cli):
        self.args_cli = args_cli

        # Setup simulation
        gravity = np.array([0, 0, -9.81])
        self.sim_cfg = sim_utils.SimulationCfg(
            device=args_cli.device,
            dt=0.005,  # 5ms timestep
            gravity=gravity,
        )
        self.sim = sim_utils.SimulationContext(self.sim_cfg)
        self.sim.set_camera_view([3.0, 3.0, 2.0], [0.0, 0.0, 0.3])

        # Create scene
        self.scene_cfg = SpotDemoSceneCfg(
            args_cli.num_envs, env_spacing=3.0, replicate_physics=True
        )
        self.scene = InteractiveScene(self.scene_cfg)

        # Reset simulation
        self.sim.reset()

        # Get robot reference
        self.robot = self.scene["Spot_Robot"]

        # Get joint info
        self.joint_pos_limits = self.robot.data.soft_joint_pos_limits[0]
        self.num_joints = self.joint_pos_limits.shape[0]
        self.default_joint_pos = self.robot.data.default_joint_pos.clone()

        # Movement parameters
        self.phase = 0.0
        self.phase_speed = 0.1
        self.movement_amplitude = 0.3

        print(f"[Spot Demo] Robot has {self.num_joints} joints")
        print(f"[Spot Demo] Joint names: {self.robot.data.joint_names[:12]}")
        print(f"[Spot Demo] Number of environments: {args_cli.num_envs}")

    def generate_walking_motion(self):
        """Generate pseudo-walking motion by oscillating leg joints."""
        self.phase += self.phase_speed
        
        # Get default positions
        target_pos = self.default_joint_pos.clone()
        
        # Create walking-like pattern (diagonal pairs move together)
        # Front-left and hind-right move together, front-right and hind-left move together
        
        for env_idx in range(self.args_cli.num_envs):
            # Simple sinusoidal motion for hip_y and knee joints
            phase_offset = env_idx * 0.5  # Offset for multiple envs
            
            for joint_idx in range(self.num_joints):
                joint_name = self.robot.data.joint_names[joint_idx]
                
                # Hip Y joints (leg swing forward/back)
                if "_hy" in joint_name:
                    if "fl_" in joint_name or "hr_" in joint_name:
                        # Diagonal pair 1
                        offset = self.movement_amplitude * np.sin(self.phase + phase_offset)
                    else:
                        # Diagonal pair 2
                        offset = self.movement_amplitude * np.sin(self.phase + phase_offset + np.pi)
                    target_pos[env_idx, joint_idx] += offset
                
                # Knee joints (leg lift)
                elif "_kn" in joint_name:
                    if "fl_" in joint_name or "hr_" in joint_name:
                        # Diagonal pair 1
                        offset = self.movement_amplitude * 0.5 * np.sin(self.phase + phase_offset)
                    else:
                        # Diagonal pair 2
                        offset = self.movement_amplitude * 0.5 * np.sin(self.phase + phase_offset + np.pi)
                    target_pos[env_idx, joint_idx] += offset
                
                # Add small random noise for more natural motion
                target_pos[env_idx, joint_idx] += (torch.rand(1, device=self.args_cli.device) - 0.5).item() * 0.02
        
        # Clamp to joint limits
        target_pos = torch.clamp(target_pos, self.joint_pos_limits[:, 0], self.joint_pos_limits[:, 1])
        
        return target_pos

    def step(self):
        """Perform one simulation step with walking motion."""
        # Generate target positions
        target_pos = self.generate_walking_motion()
        
        # Set joint targets
        self.robot.set_joint_position_target(target_pos)
        
        # Step simulation
        self.scene.write_data_to_sim()
        self.sim.step()
        self.scene.update(self.sim.get_physics_dt())

    def get_robot_state(self):
        """Get current robot state for monitoring."""
        root_pos = self.robot.data.root_pos_w
        root_vel = self.robot.data.root_lin_vel_w
        return {
            "root_pos": root_pos,
            "height": root_pos[:, 2].mean().item(),
            "forward_vel": root_vel[:, 0].mean().item(),
        }


# =============================================================================
# Main
# =============================================================================
if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("Boston Dynamics Spot Robot Demo")
    print("=" * 60 + "\n")

    # Create environment
    env = SpotDemoEnv(args_cli)

    # Setup Vision Pro streaming if IP provided
    streamer = None
    if args_cli.ip is not None:
        from avp_stream import VisionProStreamer

        streamer = VisionProStreamer(ip=args_cli.ip)
        streamer.configure_isaac(
            scene=env.scene,
            relative_to=[0, 0, 0, 0],
            include_ground=False,
            env_indices=[0],
        )
        streamer.start_webrtc()
        print(f"[Spot Demo] Streaming to Vision Pro at {args_cli.ip}")

    # Simulation loop
    step_count = 0

    print("\n[Spot Demo] Starting simulation. Press Ctrl+C to exit.\n")

    try:
        while True:
            # Step simulation
            env.step()
            step_count += 1

            # Get and print state periodically
            if step_count % 200 == 0:
                state = env.get_robot_state()
                print(
                    f"Step {step_count:5d} | Height: {state['height']:6.3f}m | "
                    f"Phase: {env.phase:.2f}"
                )

            # Update streamer if active
            if streamer is not None:
                streamer.update_sim()

    except KeyboardInterrupt:
        print("\n[Spot Demo] Simulation ended by user.")

    # Cleanup
    simulation_app.close()
