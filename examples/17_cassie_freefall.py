"""
Example script demonstrating an Agility Cassie bipedal robot free falling from height
with random joint positions.

This example shows:
- Setting up the Cassie robot in Isaac Lab
- Spawning the robot at a height above the ground
- Initializing with random joint positions
- Letting it free fall and interact with the ground plane

Usage:
    python examples/17_cassie_freefall.py --num-envs 1
    python examples/17_cassie_freefall.py --num-envs 4  # Multiple robots falling
    python examples/17_cassie_freefall.py --ip <vision_pro_ip>  # Stream to Vision Pro
"""

import numpy as np
import torch
import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(
    description="Cassie bipedal robot free falling from height with random joint positions."
)
parser.add_argument("--num-envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--drop-height", type=float, default=2.0, help="Height from which the robot drops (meters).")
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
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

# Set random seed for reproducibility
if args_cli.random_seed is not None:
    np.random.seed(args_cli.random_seed)
    torch.manual_seed(args_cli.random_seed)


# =============================================================================
# Cassie Robot Configuration (based on IsaacLab's cassie.py)
# =============================================================================
CASSIE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Agility/Cassie/cassie.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,  # Enable gravity for free fall
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,  # Enable self-collisions for realistic falling
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, args_cli.drop_height),  # Start at drop height
        joint_pos={
            "hip_abduction_left": 0.1,
            "hip_rotation_left": 0.0,
            "hip_flexion_left": 1.0,
            "thigh_joint_left": -1.8,
            "ankle_joint_left": 1.57,
            "toe_joint_left": -1.57,
            "hip_abduction_right": -0.1,
            "hip_rotation_right": 0.0,
            "hip_flexion_right": 1.0,
            "thigh_joint_right": -1.8,
            "ankle_joint_right": 1.57,
            "toe_joint_right": -1.57,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=["hip_.*", "thigh_.*", "ankle_.*"],
            effort_limit_sim=200.0,
            stiffness={
                "hip_abduction.*": 100.0,
                "hip_rotation.*": 100.0,
                "hip_flexion.*": 200.0,
                "thigh_joint.*": 200.0,
                "ankle_joint.*": 200.0,
            },
            damping={
                "hip_abduction.*": 3.0,
                "hip_rotation.*": 3.0,
                "hip_flexion.*": 6.0,
                "thigh_joint.*": 6.0,
                "ankle_joint.*": 6.0,
            },
        ),
        "toes": ImplicitActuatorCfg(
            joint_names_expr=["toe_.*"],
            effort_limit_sim=20.0,
            stiffness={
                "toe_joint.*": 20.0,
            },
            damping={
                "toe_joint.*": 1.0,
            },
        ),
    },
)


# =============================================================================
# Scene Configuration
# =============================================================================
class CassieFreefallSceneCfg(InteractiveSceneCfg):
    """Scene configuration for Cassie freefall demonstration."""

    # Ground plane with some friction for realistic landing
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            size=(20.0, 20.0),
            physics_material=RigidBodyMaterialCfg(
                static_friction=0.8,
                dynamic_friction=0.6,
                restitution=0.1,  # Slight bounce on impact
            ),
        ),
    )

    # Dome light for visibility
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    # Cassie Robot
    Cassie_Robot = CASSIE_CFG.replace(prim_path="{ENV_REGEX_NS}/Cassie")


# =============================================================================
# Environment Class
# =============================================================================
class CassieFreefallEnv:
    """Environment for Cassie freefall simulation."""

    def __init__(self, args_cli):
        self.args_cli = args_cli

        # Setup simulation with gravity
        gravity = np.array([0, 0, -9.81])
        self.sim_cfg = sim_utils.SimulationCfg(
            device=args_cli.device,
            dt=0.005,  # 5ms timestep for stable falling simulation
            gravity=gravity,
        )
        self.sim = sim_utils.SimulationContext(self.sim_cfg)
        self.sim.set_camera_view([5.0, 5.0, 4.0], [0.0, 0.0, 1.0])

        # Create scene
        self.scene_cfg = CassieFreefallSceneCfg(
            args_cli.num_envs, env_spacing=3.0, replicate_physics=True
        )
        self.scene = InteractiveScene(self.scene_cfg)

        # Reset simulation
        self.sim.reset()

        # Get robot reference
        self.robot = self.scene["Cassie_Robot"]

        # Get joint limits for random initialization
        self.joint_pos_limits = self.robot.data.soft_joint_pos_limits[0]  # (num_joints, 2)
        self.num_joints = self.joint_pos_limits.shape[0]

        print(f"[Cassie Freefall] Robot has {self.num_joints} joints")
        print(f"[Cassie Freefall] Drop height: {args_cli.drop_height} meters")
        print(f"[Cassie Freefall] Number of environments: {args_cli.num_envs}")

    def reset_with_random_joints(self):
        """Reset the robot with random joint positions within limits."""
        # Generate random joint positions within limits
        joint_pos_low = self.joint_pos_limits[:, 0]
        joint_pos_high = self.joint_pos_limits[:, 1]
        
        # Random positions within 50% of the joint range (to avoid extreme poses)
        joint_range = joint_pos_high - joint_pos_low
        joint_mid = (joint_pos_high + joint_pos_low) / 2
        
        random_scale = 0.5  # Use 50% of the joint range
        random_offsets = (torch.rand(self.args_cli.num_envs, self.num_joints, device=self.args_cli.device) - 0.5) * 2
        random_joint_pos = joint_mid + random_offsets * (joint_range * random_scale / 2)
        
        # Clamp to be safe
        random_joint_pos = torch.clamp(random_joint_pos, joint_pos_low, joint_pos_high)

        # Zero velocity
        joint_vel = torch.zeros_like(random_joint_pos)

        # Write to simulation
        self.robot.write_joint_state_to_sim(random_joint_pos, joint_vel)

        # Reset root state (position and orientation)
        root_state = self.robot.data.default_root_state.clone()
        root_state[:, 2] = self.args_cli.drop_height  # Set height
        
        # Add small random rotation for variety
        if self.args_cli.num_envs > 1:
            # Add random yaw rotation
            random_yaw = (torch.rand(self.args_cli.num_envs, device=self.args_cli.device) - 0.5) * np.pi / 4  # +/- 22.5 degrees
            root_state[:, 3:7] = self._quat_from_yaw(random_yaw)
        
        self.robot.write_root_state_to_sim(root_state)

        # Reset scene
        self.scene.reset()

        print(f"[Cassie Freefall] Reset with random joint positions")

    def _quat_from_yaw(self, yaw):
        """Convert yaw angle to quaternion (wxyz format)."""
        half_yaw = yaw / 2
        quat = torch.zeros(self.args_cli.num_envs, 4, device=self.args_cli.device)
        quat[:, 0] = torch.cos(half_yaw)  # w
        quat[:, 3] = torch.sin(half_yaw)  # z
        return quat

    def step(self):
        """Perform one simulation step."""
        self.scene.write_data_to_sim()
        self.sim.step()
        self.scene.update(self.sim.get_physics_dt())

    def get_robot_state(self):
        """Get current robot state for monitoring."""
        root_pos = self.robot.data.root_pos_w
        root_vel = self.robot.data.root_lin_vel_w
        return {
            "root_pos": root_pos,
            "root_vel": root_vel,
            "height": root_pos[:, 2].mean().item(),
            "vertical_vel": root_vel[:, 2].mean().item(),
        }


# =============================================================================
# Main
# =============================================================================
if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("Cassie Bipedal Robot Free Fall Demonstration")
    print("=" * 60 + "\n")

    # Create environment
    env = CassieFreefallEnv(args_cli)

    # Reset with random joint positions  
    env.reset_with_random_joints()

    # Setup Vision Pro streaming if IP provided
    streamer = None
    if args_cli.ip is not None:
        from avp_stream import VisionProStreamer

        streamer = VisionProStreamer(ip=args_cli.ip)
        streamer.configure_isaac(
            scene=env.scene,
            relative_to=[0, 0, 0, 0],  # Viewing position
            include_ground=False,
            env_indices=[0],
        )
        streamer.start_webrtc()
        print(f"[Cassie Freefall] Streaming to Vision Pro at {args_cli.ip}")

    # Simulation loop
    step_count = 0
    reset_interval = 500  # Reset every 500 steps (~2.5 seconds at 5ms timestep)

    print("\n[Cassie Freefall] Starting simulation. Press Ctrl+C to exit.\n")

    try:
        while True:
            # Step simulation
            env.step()
            step_count += 1

            # Get and print state periodically
            if step_count % 100 == 0:
                state = env.get_robot_state()
                print(
                    f"Step {step_count:5d} | Height: {state['height']:6.3f}m | "
                    f"Vertical Vel: {state['vertical_vel']:7.3f} m/s"
                )

            # Reset periodically for continuous demonstration
            if step_count % reset_interval == 0:
                print("\n[Cassie Freefall] Resetting with new random pose...\n")
                env.reset_with_random_joints()

            # Update streamer if active
            if streamer is not None:
                streamer.update_sim()

    except KeyboardInterrupt:
        print("\n[Cassie Freefall] Simulation ended by user.")

    # Cleanup
    simulation_app.close()
