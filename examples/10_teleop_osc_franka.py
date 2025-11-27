"""
Example: Teleoperation of Franka Panda using Operational Space Control

This example demonstrates real-time teleoperation of a Franka Panda robot
using hand tracking from the Vision Pro. The robot follows the user's hand
pose using an Operational Space Controller (OSC).

The hand pose is computed from thumb and index finger positions to create
a natural grasping pose that the robot end-effector follows.

Features:
- Real-time hand tracking to robot control
- Operational Space Controller with impedance control
- MuJoCo simulation streaming to Vision Pro
- Block pushing task with success detection

Example:
    python examples/10_teleop_osc_franka.py --ip 192.168.86.21

Requirements:
    - MuJoCo
    - scipy
    - Vision Pro with Tracking Streamer app running
"""

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from copy import deepcopy
import time
import argparse
from pathlib import Path

from avp_stream import VisionProStreamer

# Get the directory containing demo assets
ASSETS_DIR = Path(__file__).resolve().parent.parent / "assets" / "mujoco_demos"


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


class OperationalSpaceController:
    """
    Operational Space Controller (OSC) for Franka Panda robot.
    
    This controller implements Cartesian impedance control with null-space
    joint posture regulation. The end-effector follows a target pose while
    the null-space motion keeps joints near a default configuration.
    
    Attributes:
        impedance_pos: Position stiffness [N/m]
        impedance_ori: Orientation stiffness [Nm/rad]
        Kp_null: Null-space joint stiffness
        damping_ratio: Damping ratio for both Cartesian and null-space control
    """

    impedance_pos = np.asarray([5.0, 5.0, 5.0])  # [N/m]
    impedance_ori = np.asarray([1.0, 1.0, 1.0])  # [Nm/rad]

    # Joint impedance control gains
    Kp_null = np.asarray([0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.1]) * 10.0

    # Damping ratio for both Cartesian and joint impedance control
    damping_ratio = 2.0

    # Gains for the twist computation (0-1 range)
    Kpos: float = 10
    Kori: float = 10

    # Integration timestep
    integration_dt: float = 1.0

    # Simulation timestep
    dt: float = 0.002

    # Compute damping and stiffness matrices
    damping_pos = damping_ratio * 2 * np.sqrt(impedance_pos)
    damping_ori = damping_ratio * 2 * np.sqrt(impedance_ori)
    Kp = np.concatenate([impedance_pos, impedance_ori], axis=0)
    Kd = np.concatenate([damping_pos, damping_ori], axis=0)
    Kd_null = damping_ratio * 2 * np.sqrt(Kp_null)

    def __init__(self, model):
        """
        Initialize the controller.
        
        Args:
            model: MuJoCo model object
        """
        # End-effector site we wish to control
        site_name = "attachment_site"
        self.site_id = model.site(site_name).id

        # Get the dof and actuator ids for the joints we wish to control
        joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        self.dof_ids = np.array([model.joint(name).id for name in joint_names])
        self.actuator_ids = np.array([model.actuator(name).id for name in joint_names])

        # Initial joint configuration saved as a keyframe in the XML file
        key_name = "home"
        self.key_id = model.key(key_name).id
        self.q0 = model.key(key_name).qpos[:7]

        # Mocap body we will control
        mocap_name = "target"
        self.mocap_id = model.body(mocap_name).mocapid[0]

        # Pre-allocate numpy arrays
        self.jac = np.zeros((6, model.nv))
        self.twist = np.zeros(6)
        self.site_quat = np.zeros(4)
        self.site_quat_conj = np.zeros(4)
        self.error_quat = np.zeros(4)
        self.M_inv = np.zeros((model.nv, model.nv))
        self.Mx = np.zeros((6, 6))

        # Logging buffers
        self.qpos_log = []
        self.qvel_log = []
        self.ctrl_log = []
        self.mocap_log = []

    def log(self, data):
        """Log current state for trajectory recording."""
        self.qpos_log.append(deepcopy(data.qpos))
        self.qvel_log.append(deepcopy(data.qvel))
        self.ctrl_log.append(deepcopy(data.ctrl))
        
        target_mocap_id = data.model.body("target_pos").mocapid[0]
        mocap_pose = np.concatenate([
            data.mocap_pos[target_mocap_id],
            data.mocap_quat[target_mocap_id]
        ])
        self.mocap_log.append(deepcopy(mocap_pose))

    def save(self, ep_idx, save_dir):
        """Save recorded trajectory to file."""
        fp = save_dir / f"ep{ep_idx}.npz"

        qpos_log = np.stack(self.qpos_log, axis=0)
        qvel_log = np.stack(self.qvel_log, axis=0)
        ctrl_log = np.stack(self.ctrl_log, axis=0)
        mocap_log = np.stack(self.mocap_log, axis=0)

        np.savez(fp, qpos=qpos_log, qvel=qvel_log, ctrl=ctrl_log, mocap=mocap_log)
        print(f"💾 Saved episode to {fp}")

        self.mocap_log = []
        self.qvel_log = []
        self.qpos_log = []
        self.ctrl_log = []

    def pose2torque(self, model, data):
        """
        Compute joint torques to move end-effector to target pose.
        
        Uses Cartesian impedance control with null-space posture regulation.
        
        Args:
            model: MuJoCo model
            data: MuJoCo data
        
        Returns:
            Computed joint torques
        """
        # Spatial velocity (aka twist)
        dx = data.mocap_pos[self.mocap_id] - data.site(self.site_id).xpos
        self.twist[:3] = self.Kpos * dx / self.integration_dt
        
        mujoco.mju_mat2Quat(self.site_quat, data.site(self.site_id).xmat)
        mujoco.mju_negQuat(self.site_quat_conj, self.site_quat)
        mujoco.mju_mulQuat(self.error_quat, data.mocap_quat[self.mocap_id], self.site_quat_conj)
        mujoco.mju_quat2Vel(self.twist[3:], self.error_quat, 1.0)
        self.twist[3:] *= self.Kori / self.integration_dt

        # Jacobian
        mujoco.mj_jacSite(model, data, self.jac[:3], self.jac[3:], self.site_id)

        # Compute the task-space inertia matrix
        mujoco.mj_solveM(model, data, self.M_inv, np.eye(model.nv))
        new_jac = self.jac[:, :7]
        new_M_inv = self.M_inv[:7, :7]

        Mx_inv = new_jac @ new_M_inv @ new_jac.T
        if abs(np.linalg.det(Mx_inv)) >= 1e-2:
            Mx = np.linalg.inv(Mx_inv)
        else:
            Mx = np.linalg.pinv(Mx_inv, rcond=1e-2)

        # Compute generalized forces
        tau = new_jac.T @ Mx @ (
            self.Kp * self.twist - self.Kd * (new_jac @ data.qvel[self.dof_ids])
        )

        # Add joint task in nullspace
        Jbar = new_M_inv @ new_jac.T @ Mx
        ddq = (
            self.Kp_null * (self.q0 - data.qpos[self.dof_ids])
            - self.Kd_null * data.qvel[self.dof_ids]
        )
        tau += (np.eye(7) - new_jac.T @ Jbar.T) @ ddq

        data.ctrl[self.actuator_ids] = tau
        data.ctrl[-1] = 0.0

        self.log(data)
        mujoco.mj_step(model, data)

        return tau


def reset_simulation(model, data, controller, streamer):
    """
    Reset the simulation to a random initial state.
    
    This function:
    1. Resets robot to home configuration
    2. Randomizes target position and orientation
    3. Randomizes block position and orientation
    4. Waits for user's hand to be tracked
    5. Smoothly interpolates robot to user's hand position
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
        controller: OperationalSpaceController instance
        streamer: VisionProStreamer instance
    """
    key_name = "home"
    key_id = model.key(key_name).id
    mujoco.mj_resetDataKeyframe(model, data, key_id)

    mocap_target_id = model.body("target_pos").mocapid[0]

    # Random 90-degree rotation for target
    angles = [np.deg2rad(np.random.choice([-90, 0, 90])) for _ in range(3)]
    rot = R.from_euler('xyz', angles).as_quat()

    data.mocap_pos[mocap_target_id][0:2] = np.array([0.5, 0.1]) + np.random.uniform(-0.1, 0.1, 2)
    data.mocap_pos[mocap_target_id][2] = 0.05
    data.mocap_quat[mocap_target_id] = rot

    # Randomize block position
    data.qpos[9:11] = np.array([0.5, -0.1]) + np.random.uniform(-0.1, 0.1, 2)
    data.qpos[11] = 0.05

    # Reset mocap target
    data.mocap_pos[controller.mocap_id] = np.array([0.5, 0.0, 0.5])
    data.mocap_quat[controller.mocap_id] = np.array([0, 1, 0, 0])

    data.qvel[:] = 0.0

    # Random rotation for block
    angles = [np.deg2rad(np.random.choice([-90, 0, 90])) for _ in range(3)]
    rot = R.from_euler('xyz', angles).as_quat()
    data.qpos[12:16] = rot

    data.qacc_warmstart[:] = 0.0
    mujoco.mj_forward(model, data)

    print("🖐️ Move your hand to start position...")

    # Wait for hand tracking and follow hand
    for i in range(1000):
        hand = streamer.get_latest()
        if hand is None:
            time.sleep(1 / 60.0)
            continue

        frame = hand2pose(hand, side="right")
        data.mocap_pos[controller.mocap_id] = frame[:3, 3]
        data.mocap_quat[controller.mocap_id] = R.from_matrix(frame[:3, :3]).as_quat(scalar_first=True)
        
        mujoco.mj_forward(model, data)
        streamer.update_sim()
        time.sleep(1 / 600.0)

    print("🤖 Aligning robot to hand...")

    # Smoothly interpolate robot to hand position
    for i in range(1000):
        hand = streamer.get_latest()
        if hand is None:
            time.sleep(1 / 60.0)
            continue

        frame = hand2pose(hand, side="right")
        mujoco.mj_forward(model, data)

        target_pos = frame[:3, 3]
        target_quat = R.from_matrix(frame[:3, :3]).as_quat(scalar_first=True)

        cur_pos = data.site(controller.site_id).xpos
        cur_quat = np.zeros(4)
        mujoco.mju_mat2Quat(cur_quat, data.site(controller.site_id).xmat)

        slerp = Slerp([0, 1], R.from_quat([cur_quat, target_quat]))
        alpha = i / 1000

        data.mocap_pos[controller.mocap_id] = cur_pos * (1 - alpha) + target_pos * alpha
        data.mocap_quat[controller.mocap_id] = slerp(alpha).as_quat()
        
        controller.pose2torque(model, data)
        streamer.update_sim()
        time.sleep(1 / 600.0)

    print("✅ Ready for teleoperation!")


def check_success(model, data):
    """
    Check if the block pushing task is successful.
    
    Success is defined as:
    - Block is within 1cm of target position
    - Block orientation is within ~6 degrees of target
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
    
    Returns:
        True if task is successful, False otherwise
    """
    block_pos = data.body("cube").xpos
    target_pos = data.mocap_pos[model.body("target_pos").mocapid[0]]

    block_quat = data.body("cube").xquat
    target_quat = data.mocap_quat[model.body("target_pos").mocapid[0]]

    dist = np.linalg.norm(block_pos - target_pos)
    quat_diff = R.from_quat(block_quat).inv() * R.from_quat(target_quat)
    quat_diff = quat_diff.magnitude()

    return dist < 0.01 and quat_diff < 0.1


def main(args):
    """Main teleoperation loop."""
    
    # Load the scene
    xml_path = str(ASSETS_DIR / "scenes" / "franka_emika_panda" / "scene_blockpush.xml")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Create logs directory
    logs_dir = ASSETS_DIR / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)

    # Initialize streamer
    streamer = VisionProStreamer(ip=args.ip, record=False, verbose=args.verbose)

    # Configure simulation streaming
    # attach_to format: [x, y, z, yaw_degrees]
    attach_to = [0.2, 1.0, 0.7, -90]

    streamer.configure_sim(
        xml_path=xml_path,
        model=model,
        data=data,
        relative_to=attach_to,
        grpc_port=args.port,
    )

    # Start WebRTC streaming
    streamer.start_webrtc()

    # Initialize controller
    controller = OperationalSpaceController(model)

    # Reset to initial state
    reset_simulation(model, data, controller, streamer)

    ep_idx = args.start_episode

    print(f"\n🎮 Teleoperation started! Push the block to the target.")
    print(f"   Episodes will be saved starting from ep{ep_idx}")
    print(f"   Press Ctrl+C to stop\n")

    try:
        while True:
            hand = streamer.get_latest()
            if hand is None:
                time.sleep(1 / 60.0)
                continue

            frame = hand2pose(hand, side="right")

            data.mocap_pos[controller.mocap_id] = frame[:3, 3]
            data.mocap_quat[controller.mocap_id] = R.from_matrix(frame[:3, :3]).as_quat(scalar_first=True)

            controller.pose2torque(model, data)
            streamer.update_sim()
            time.sleep(1 / 600.0)

            if check_success(model, data):
                print(f"🎉 Success! Saving episode {ep_idx}...")
                controller.save(ep_idx, logs_dir)
                ep_idx += 1
                reset_simulation(model, data, controller, streamer)

    except KeyboardInterrupt:
        print(f"\n\n🛑 Stopped by user")
        if len(controller.qpos_log) > 0:
            print(f"💾 Saving partial episode {ep_idx}...")
            controller.save(ep_idx, logs_dir)

    print(f"\n✨ Teleoperation complete!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Teleoperate Franka Panda robot using Vision Pro hand tracking"
    )
    parser.add_argument(
        "--ip",
        default="192.168.86.21",
        help="Vision Pro IP address",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=50051,
        help="MuJoCo gRPC port for USDZ transfer (default: 50051)",
    )
    parser.add_argument(
        "--start-episode",
        type=int,
        default=0,
        help="Starting episode index for saving trajectories (default: 0)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose output",
    )
    args = parser.parse_args()

    main(args)
