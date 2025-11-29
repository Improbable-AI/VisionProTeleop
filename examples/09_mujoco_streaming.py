"""Stream MuJoCo simulation to Vision Pro, replaying pre-recorded robot trajectories."""

import numpy as np
from tqdm import trange
import time
from pathlib import Path
import argparse

# Get the directory containing demo assets
ASSETS_DIR = Path(__file__).resolve().parent.parent / "assets" / "mujoco_demos"


def main(args):
    import mujoco

    # Load the scene
    xml_path = str(ASSETS_DIR / "scenes" / "franka_emika_panda" / "scene_blockpush.xml")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    from avp_stream import VisionProStreamer
    streamer = VisionProStreamer(ip=args.ip, record=False)

    # attach_to format: [x, y, z, yaw_degrees]
    attach_to = [0.2, 1.0, 0.7, -90]

    streamer.configure_sim(
        xml_path=xml_path,
        model=model,
        data=data,
        relative_to=attach_to,
        grpc_port=args.port,
    )

    streamer.start_webrtc()


    logs_dir = ASSETS_DIR / "logs"
    episodes = sorted(logs_dir.glob("ep*.npz"))

    try:
        for ep_idx in range(1, min(5, len(episodes) + 1)):
            traj_path = logs_dir / f"ep{ep_idx}.npz"
            if not traj_path.exists():
                continue

            print(f"📼 Playing episode {ep_idx}...")
            traj = np.load(traj_path)

            qpos_log = traj["qpos"]
            qvel_log = traj["qvel"]
            ctrl_log = traj["ctrl"]
            mocap_log = traj["mocap"]

            T = qpos_log.shape[0]

            # Initialize state
            data.qpos[:] = qpos_log[0]
            data.qvel[:] = qvel_log[0]

            if mocap_log.shape[0] > 0:
                data.mocap_pos[1] = mocap_log[0, :3]
                data.mocap_quat[1] = mocap_log[0, 3:]

            data.qacc_warmstart[:] = 0.0
            mujoco.mj_forward(model, data)

            # Replay trajectory
            for t in trange(T, desc=f"  Episode {ep_idx}", leave=False):
                data.ctrl = ctrl_log[t]
                mujoco.mj_step(model, data)
                streamer.update_sim()
                time.sleep(1 / 1000.0)

            print(f"  ✅ Episode {ep_idx} complete ({T} steps)")

    except KeyboardInterrupt:
        print(f"\n\n🛑 Stopped by user")

    print(f"\n✨ Demo complete!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Stream MuJoCo simulation replay to Vision Pro"
    )
    parser.add_argument(
        "--viewer",
        default="mujoco",
        choices=["mujoco", "ar"],
        help="Viewer type: 'mujoco' for local preview, 'ar' for Vision Pro streaming",
    )
    parser.add_argument(
        "--ip",
        default="10.29.239.70",
        help="Vision Pro IP address (only used with --viewer ar)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=50051,
        help="MuJoCo gRPC port for USDZ transfer (default: 50051)",
    )
    args = parser.parse_args()

    main(args)
