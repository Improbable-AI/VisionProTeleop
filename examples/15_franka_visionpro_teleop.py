"""
Vision Pro teleoperation demo for Franka Panda robot.

This script demonstrates how to use Vision Pro hand tracking to
teleoperate a Franka Panda robot arm in Isaac Lab:
- Position tracking from the right hand wrist
- Pinch gesture controls gripper open/close
- Real-time visualization streamed back to Vision Pro

Adapted from Isaac Lab's haply_teleoperation.py demo.

Usage:
    # Keyboard mode (for testing):
    python examples/18_franka_visionpro_teleop.py
    
    # Vision Pro mode:
    python examples/18_franka_visionpro_teleop.py --ip <vision_pro_ip>
    python examples/18_franka_visionpro_teleop.py --ip <vision_pro_ip> --pos-sensitivity 2.0

Keyboard controls (when --ip not provided):
    i/k: Move end-effector +/- Z (up/down)
    j/l: Move end-effector +/- Y (left/right)
    w/s: Move end-effector +/- X (forward/backward)
    space: Toggle gripper open/close
"""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Vision Pro teleoperation with Franka Panda.")
parser.add_argument("--num-envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--ip", type=str, default=None, help="Vision Pro IP address. If not provided, uses keyboard control.")
parser.add_argument(
    "--pos-sensitivity",
    type=float,
    default=1.0,
    help="Position sensitivity scaling factor.",
)

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch
import sys
import threading
import select
from scipy.spatial.transform import Rotation as R

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, AssetBaseCfg, RigidObject, RigidObjectCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_assets import FRANKA_PANDA_HIGH_PD_CFG  # isort: skip

# =============================================================================
# Workspace mapping constants
# =============================================================================
WORKSPACE_LIMITS = {
    "x": (0.1, 0.9),
    "y": (-0.50, 0.50),
    "z": (1.05, 1.85),
}


# def apply_hand_to_robot_mapping(
#     hand_pos: np.ndarray,
#     hand_initial_pos: np.ndarray,
#     robot_initial_pos: np.ndarray,
#     sensitivity: float = 1.0,
# ) -> np.ndarray:
#     """Apply coordinate mapping from Vision Pro hand position to Franka end-effector.

#     Uses relative position control: robot_pos = robot_initial_pos + (hand_pos - hand_initial_pos) * sensitivity

#     Args:
#         hand_pos: Current hand position [x, y, z] in meters (Z-up Vision Pro coords)
#         hand_initial_pos: Hand's reference position when teleoperation started
#         robot_initial_pos: Initial end-effector position in robot workspace
#         sensitivity: Position scaling factor

#     Returns:
#         robot_pos: Target position for robot EE in world frame [x, y, z]
#     """
#     # Compute relative movement from initial position
#     hand_delta = (hand_pos - hand_initial_pos) * sensitivity

#     # Vision Pro uses Z-up coordinate system, same as Isaac Lab
#     # Direct mapping: X forward, Y left, Z up
#     robot_pos = robot_initial_pos + hand_delta

#     # Apply workspace limits for safety
#     robot_pos[0] = np.clip(robot_pos[0], WORKSPACE_LIMITS["x"][0], WORKSPACE_LIMITS["x"][1])
#     robot_pos[1] = np.clip(robot_pos[1], WORKSPACE_LIMITS["y"][0], WORKSPACE_LIMITS["y"][1])
#     robot_pos[2] = np.clip(robot_pos[2], WORKSPACE_LIMITS["z"][0], WORKSPACE_LIMITS["z"][1])

#     return robot_pos


def hand2pose(hand, side="right"):
    """
    Convert hand tracking data to a 4x4 pose matrix suitable for robot control.
    
    The pose is computed from thumb and index finger positions to create
    a natural grasping orientation:
    - Z-axis points from finger base to fingertip (grasp direction)
    - X-axis points from thumb to index (lateral direction)
    - Y-axis is the cross product
    
    Args:
        hand: Dictionary containing hand tracking data from VisionProStreamer
        side: "left" or "right" hand (default: "right")
    
    Returns:
        4x4 numpy array representing the hand pose in world coordinates
    """
    wrist = hand[f"{side}_wrist"]
    finger = wrist @ hand[f"{side}_fingers"]

    thumb_tip = finger[4, :3, -1]
    thumb_base = finger[2, :3, -1]
    index_tip = finger[9, :3, -1]
    index_base = finger[7, :3, -1]

    base_middle = (thumb_base + index_base) * 0.5
    tip_middle = (thumb_tip + index_tip) * 0.5

    z_axis = tip_middle - base_middle
    z_axis /= np.linalg.norm(z_axis)

    # Use thumb→index direction as x
    x_axis = index_base - thumb_base
    x_axis -= np.dot(x_axis, z_axis) * z_axis  # Make x ⟂ z
    x_axis /= np.linalg.norm(x_axis)

    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    rot = np.column_stack((x_axis, y_axis, z_axis))
    rot /= np.cbrt(np.linalg.det(rot))  # Ensure det = 1

    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = tip_middle

    return mat




@configclass
class FrankaVisionProSceneCfg(InteractiveSceneCfg):
    """Configuration for Franka scene with Vision Pro teleoperation."""

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd",
            scale=(1.0, 1.0, 1.0),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.50, 0.0, 1.05), rot=(0.707, 0, 0, 0.707)),
    )

    robot: Articulation = FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    robot.init_state.pos = (-0.02, 0.0, 1.05)
    robot.spawn.activate_contact_sensors = True

    cube = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.06, 0.06, 0.06),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.5),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.5),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.8, 0.2), metallic=0.2),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.60, 0.00, 1.15)),
    )

    # Target position marker - small red cuboid (kinematic so it won't fall)
    target_marker = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/TargetMarker",
        spawn=sim_utils.CuboidCfg(
            size=(0.1, 0.1, 0.1),  # 3cm cube
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=False, disable_gravity=True),  # Kinematic = can set pose freely
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),  # No collisions
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.2, 0.2), metallic=0.5),  # Red
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.45, 0.00, 1.35)),
    )

    left_finger_contact_sensor = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/panda_leftfinger",
        update_period=0.0,
        history_length=3,
        debug_vis=False,
        track_pose=True,
    )

    right_finger_contact_sensor = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/panda_rightfinger",
        update_period=0.0,
        history_length=3,
        debug_vis=False,
        track_pose=True,
    )


# =============================================================================
# Keyboard Controller (Terminal-based, works over SSH)
# =============================================================================
class KeyboardController:
    """Terminal-based keyboard controller for testing without Vision Pro.
    
    Works over SSH by reading from stdin in a background thread.
    Uses single-character input mode.
    """
    
    def __init__(self):
        import tty
        import termios
        
        self.gripper_open = True
        self._current_key = None
        self._lock = threading.Lock()
        self._running = True
        
        # Save terminal settings
        self._old_settings = termios.tcgetattr(sys.stdin)
        
        # Set terminal to raw mode for single-char input
        tty.setcbreak(sys.stdin.fileno())
        
        # Start background thread for keyboard reading
        self._thread = threading.Thread(target=self._read_keys, daemon=True)
        self._thread.start()
        
        print("[KEYBOARD] Terminal keyboard control initialized")
    
    def _read_keys(self):
        """Background thread to read keyboard input."""
        while self._running:
            try:
                # Check if input is available (non-blocking)
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    key = sys.stdin.read(1)
                    
                    with self._lock:
                        self._current_key = key
                        
                        # Toggle gripper on space
                        if key == ' ':
                            self.gripper_open = not self.gripper_open
                            print(f"\r[KEYBOARD] Gripper: {'OPEN' if self.gripper_open else 'CLOSED'}        ")
            except Exception:
                pass
    
    def cleanup(self):
        """Restore terminal settings."""
        import termios
        self._running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
    
    def get_delta(self, step_size: float = 0.01) -> np.ndarray:
        """Get position delta based on current key press.
        
        Controls:
            i/k: +/- Z (up/down)
            j/l: +/- Y (left/right)  
            w/s: +/- X (forward/backward)
            space: toggle gripper
        """
        delta = np.array([0.0, 0.0, 0.0])
        
        with self._lock:
            key = self._current_key
            self._current_key = None  # Consume the key
        
        if key is None:
            return delta
        
        key = key.lower()
        
        # i/k for Z (up/down)
        if key == 'i':
            delta[2] += step_size  # +Z (up)
        elif key == 'k':
            delta[2] -= step_size  # -Z (down)
        
        # j/l for Y (left/right)
        elif key == 'j':
            delta[1] += step_size  # +Y (left)
        elif key == 'l':
            delta[1] -= step_size  # -Y (right)
        
        # w/s for X (forward/backward)
        elif key == 'w':
            delta[0] += step_size  # +X (forward)
        elif key == 's':
            delta[0] -= step_size  # -X (backward)
        
        return delta
    
    def get_gripper_target(self) -> float:
        """Get gripper target position."""
        return 0.04 if self.gripper_open else 0.0


def run_simulator(
    sim: sim_utils.SimulationContext,
    scene: InteractiveScene,
    streamer,
    use_keyboard: bool = False,
):
    """Runs the simulation loop with Vision Pro or keyboard teleoperation."""
    sim_dt = sim.get_physics_dt()
    count = 1

    robot: Articulation = scene["robot"]
    cube: RigidObject = scene["cube"]
    target_marker_obj: RigidObject = scene["target_marker"]

    ee_body_name = "panda_hand"
    ee_body_idx = robot.body_names.index(ee_body_name)

    # Initialize robot to a good starting pose
    joint_pos = robot.data.default_joint_pos.clone()
    joint_pos[0, :7] = torch.tensor([0.0, -0.569, 0.0, -2.81, 0.0, 3.037, 0.741], device=robot.device)
    joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(joint_pos, joint_vel)

    # Run a few steps to settle
    for _ in range(10):
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

    # Get initial end-effector position and orientation
    robot_initial_pos = robot.data.body_pos_w[0, ee_body_idx].cpu().numpy()
    robot_initial_quat = robot.data.body_quat_w[0, ee_body_idx].cpu().numpy()  # (w, x, y, z)
    target_pos = robot_initial_pos.copy()  # Current target position
    target_quat = robot_initial_quat.copy()  # Current target orientation (w, x, y, z)
    hand_initial_pos = None  # Will be set on first valid hand tracking

    # Setup IK controller - now using "pose" for position + orientation tracking
    ik_controller_cfg = DifferentialIKControllerCfg(
        command_type="pose",
        use_relative_mode=False,
        ik_method="dls",
        ik_params={"lambda_val": 0.05},
    )

    arm_joint_names = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
    ]
    arm_joint_indices = [robot.joint_names.index(name) for name in arm_joint_names]

    ik_controller = DifferentialIKController(cfg=ik_controller_cfg, num_envs=scene.num_envs, device=sim.device)
    initial_ee_quat = robot.data.body_quat_w[:, ee_body_idx]
    # For pose command, we need 7 values: pos (3) + quat (4) in (w, x, y, z) format
    initial_command = torch.zeros(scene.num_envs, 7, device=sim.device)
    ik_controller.set_command(command=initial_command, ee_quat=initial_ee_quat)

    # Gripper state
    gripper_target = 0.04  # Open
    
    # Create target position visualization marker (axis frame)
    marker_cfg = FRAME_MARKER_CFG.copy()
    marker_cfg.prim_path = "/Visuals/TargetPositionMarker"
    marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)  # 10cm axis size
    target_marker = VisualizationMarkers(marker_cfg)
    
    # Create keyboard controller if in keyboard mode
    keyboard_controller = KeyboardController() if use_keyboard else None

    if use_keyboard:
        print("\n[INFO] Keyboard Teleoperation ready!")
        print("  i/k: Move end-effector +/- Z (up/down)")
        print("  j/l: Move end-effector +/- Y (left/right)")
        print("  w/s: Move end-effector +/- X (forward/backward)")
        print("  space: Toggle gripper open/close\n")
    else:
        print("\n[INFO] Vision Pro Teleoperation ready!")
        print("  Move right hand: Control robot end-effector position")
        print("  Pinch gesture: Close gripper")
        print("  Release pinch: Open gripper\n")
        print("  Waiting for first hand tracking data...\n")

    while simulation_app.is_running():
        # Reset periodically
        if count % 10000 == 0:
            count = 1
            root_state = robot.data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins
            robot.write_root_pose_to_sim(root_state[:, :7])
            robot.write_root_velocity_to_sim(root_state[:, 7:])

            joint_pos = robot.data.default_joint_pos.clone()
            joint_pos[0, :7] = torch.tensor([0.0, -0.569, 0.0, -2.81, 0.0, 3.037, 0.741], device=robot.device)
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)

            cube_state = cube.data.default_root_state.clone()
            cube_state[:, :3] += scene.env_origins
            cube.write_root_pose_to_sim(cube_state[:, :7])
            cube.write_root_velocity_to_sim(cube_state[:, 7:])

            scene.reset()
            ik_controller.reset()
            hand_initial_pos = None  # Reset hand reference
            target_pos = robot_initial_pos.copy()  # Reset target position
            target_quat = robot_initial_quat.copy()  # Reset target orientation
            print("[INFO]: Resetting robot state...")

        if use_keyboard:
            # Keyboard control mode
            delta = keyboard_controller.get_delta(step_size=0.005)
            target_pos = target_pos + delta
            
            # Apply workspace limits
            target_pos[0] = np.clip(target_pos[0], WORKSPACE_LIMITS["x"][0], WORKSPACE_LIMITS["x"][1])
            target_pos[1] = np.clip(target_pos[1], WORKSPACE_LIMITS["y"][0], WORKSPACE_LIMITS["y"][1])
            target_pos[2] = np.clip(target_pos[2], WORKSPACE_LIMITS["z"][0], WORKSPACE_LIMITS["z"][1])
            
            gripper_target = keyboard_controller.get_gripper_target()
            
            # Print position periodically
            if count % 100 == 0:
                print(f"[POS] X: {target_pos[0]:.3f}, Y: {target_pos[1]:.3f}, Z: {target_pos[2]:.3f}")
            
        else:
            # Vision Pro control mode
            hand = streamer.get_latest()
            
            # Debug: Print hand tracking status periodically (PROMINENT)
            if count % 100 == 0:
                print(f"\n{'='*60}")
                print(f"[TELEOP DEBUG] Frame {count}")
                if hand is None:
                    print(f"  PROBLEM: streamer.get_latest() returned None!")
                    print(f"  Hand tracking data stopped flowing after initialization")
                elif hand.get("right_wrist") is None:
                    print(f"  PROBLEM: Hand data exists but right_wrist is None")
                else:
                    hand_pos = hand["right_wrist"][0, :3, 3]
                    print(f"  Hand tracking: ACTIVE")
                    print(f"  Current hand pos: [{hand_pos[0]:.3f}, {hand_pos[1]:.3f}, {hand_pos[2]:.3f}]")
                    print(f"  Initial hand pos: [{hand_initial_pos[0]:.3f}, {hand_initial_pos[1]:.3f}, {hand_initial_pos[2]:.3f}]" if hand_initial_pos is not None else "  Initial hand pos: Not set")
                    print(f"  Target robot pos: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                print(f"{'='*60}\n")

            if hand is not None and hand.get("right_wrist") is not None:
                # Extract right hand pose (position + rotation)
                hand_pose = hand2pose(hand)
                hand_pos = hand_pose[:3, 3]
                hand_rot_matrix = hand_pose[:3, :3]
                
                # Convert rotation matrix to quaternion (w, x, y, z) format
                # scipy uses (x, y, z, w) so we need to reorder
                rot = R.from_matrix(hand_rot_matrix)
                quat_xyzw = rot.as_quat()  # Returns (x, y, z, w)
                hand_quat = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])  # Convert to (w, x, y, z)

                # Get pinch distance for gripper control
                pinch_distance = hand.get("right_pinch_distance", 0.05)

                # Map pinch to gripper: closed when pinching (small distance)
                if pinch_distance < 0.02:  # Pinching threshold
                    gripper_target = 0.0  # Close
                else:
                    gripper_target = 0.04  # Open

                target_pos = hand_pos
                target_quat = hand_quat

        # Convert target to tensors
        target_pos_tensor = torch.tensor(target_pos, dtype=torch.float32, device=sim.device).unsqueeze(0)
        target_quat_tensor = torch.tensor(target_quat, dtype=torch.float32, device=sim.device).unsqueeze(0)
        
        # Update the kinematic target marker cuboid position and orientation (this syncs to Vision Pro)
        marker_root_state = torch.zeros((1, 7), dtype=torch.float32, device=sim.device)
        marker_root_state[:, :3] = target_pos_tensor  # Set position
        marker_root_state[:, 3:7] = target_quat_tensor  # Set orientation
        target_marker_obj.write_root_pose_to_sim(marker_root_state)

        # Get current robot state
        current_joint_pos = robot.data.joint_pos[:, arm_joint_indices]
        ee_pos_w = robot.data.body_pos_w[:, ee_body_idx]
        ee_quat_w = robot.data.body_quat_w[:, ee_body_idx]

        # Compute IK with pose command (position + orientation)
        jacobian = robot.root_physx_view.get_jacobians()[:, ee_body_idx, :, arm_joint_indices]
        # Pose command: [pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z]
        pose_command = torch.cat([target_pos_tensor, target_quat_tensor], dim=1)
        ik_controller.set_command(command=pose_command, ee_quat=ee_quat_w)
        joint_pos_des = ik_controller.compute(ee_pos_w, ee_quat_w, jacobian, current_joint_pos)

        # Update joint targets
        joint_pos_target = robot.data.joint_pos[0].clone()
        joint_pos_target[arm_joint_indices] = joint_pos_des[0]
        joint_pos_target[[-2, -1]] = gripper_target  # Gripper fingers

        robot.set_joint_position_target(joint_pos_target.unsqueeze(0))

        # Step simulation
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        # Stream updated poses back to Vision Pro
        if streamer is not None:
            streamer.update_sim()


        count += 1



def main():
    """Main function to set up and run the teleoperation demo."""
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=1 / 200)
    sim = sim_utils.SimulationContext(sim_cfg)

    # Set camera view
    sim.set_camera_view([1.6, 1.0, 1.70], [0.4, 0.0, 1.0])

    scene_cfg = FrankaVisionProSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    # Determine mode based on --ip argument
    use_keyboard = False 
    streamer = None
    
    sim.reset()
    
    if args_cli.ip is not None:
        # Create Vision Pro streamer
        from avp_stream import VisionProStreamer
        
        streamer = VisionProStreamer(ip=args_cli.ip)
        
        # Configure Isaac Lab streaming
        streamer.configure_isaac(
            scene=scene,
            relative_to=[0, 0.5, -0.3, 0],  # Position scene at comfortable viewing height
            include_ground=False,
            env_indices=[0],
            streaming_hz = 60
        )
        streamer.start_webrtc()
        
        print(f"[INFO] Vision Pro connected: {args_cli.ip}")
    else:
        print("[INFO] Running in keyboard mode (no --ip provided)")


    run_simulator(sim, scene, streamer, use_keyboard=use_keyboard)


if __name__ == "__main__":
    main()
    simulation_app.close()

