"""Teleoperate Shadow Hand using differential IK with hand tracking.

Requires: pip install 'mink[examples]'
Adapted from https://github.com/kevinzakka/mink
"""


import os
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    import mink
    from loop_rate_limiters import RateLimiter
except ImportError:
    print("This example requires mink and its dependencies.")
    print("Install with: pip install 'mink[examples]'")
    sys.exit(1)

_HERE = Path(__file__).parent
_XML = os.path.join(_HERE, "..", "assets", "mujoco_demos", "shadow_hand", "scene_left.xml")


def main(args): 
    model = mujoco.MjModel.from_xml_path(_XML)
    configuration = mink.Configuration(model)
    posture_task = mink.PostureTask(model, cost=1e-2)

    fingers = ["thumb", "first", "middle", "ring", "little"]
    finger_tasks = []
    for finger in fingers:
        task = mink.FrameTask(
            frame_name=finger,
            frame_type="site",
            position_cost=1.0,
            orientation_cost=0.0,
            lm_damping=1.0,
        )
        finger_tasks.append(task)

    tasks = [
        posture_task,
        *finger_tasks,
    ]

    model = configuration.model
    data = configuration.data
    solver = "daqp"

    from avp_stream import VisionProStreamer
    streamer = VisionProStreamer(ip=args.ip)

    streamer.configure_sim(
        xml_path=str(_XML),
        model=model,
        data=data,
        relative_to=[-0.3, 0.1, 0.8, 90],
    )

    streamer.start_webrtc()


    # configuration.update_from_keyframe("grasp hard")

    # Initialize mocap bodies at their respective sites.
    posture_task.set_target_from_configuration(configuration)
    for finger in fingers:
        mink.move_mocap_to_frame(model, data, f"{finger}_target", finger, "site")

    rate = RateLimiter(frequency=500.0, warn=False)
    dt = rate.dt
    t = 0

    while True: 
        hand = streamer.get_latest()
        
        rot = np.eye(4)
        rot[:3, :3] = R.from_euler("xz", [0, 180], degrees=True).as_matrix()
        rot = rot[np.newaxis, :, :]

        smaller_hand = hand["left_fingers"].copy() 
        # smaller_hand[..., :3, -1] *= 0.5
        # smaller_hand[...,]

        finger = hand["left_wrist"] @ (smaller_hand )

        # finger = finger



        offset = np.array([0, 0.0, 0]) 

        data.mocap_pos[model.body("thumb_target").mocapid[0], :] = finger[4][:3, 3]  + offset 
        data.mocap_pos[model.body("first_target").mocapid[0], :] = finger[9][:3, 3] + offset 
        data.mocap_pos[model.body("middle_target").mocapid[0], :] = finger[14][:3, 3] + offset 
        data.mocap_pos[model.body("ring_target").mocapid[0], :] = finger[19][:3, 3] + offset 
        data.mocap_pos[model.body("little_target").mocapid[0], :] = finger[24][:3, 3] + offset 

        # Update task target.
        for finger, task in zip(fingers, finger_tasks):
            task.set_target(
                mink.SE3.from_mocap_name(model, data, f"{finger}_target")
            )

        vel = mink.solve_ik(configuration, tasks, rate.dt, solver, 1e-5)
        configuration.integrate_inplace(vel, rate.dt)

        mujoco.mj_forward(model, data)

        streamer.update_sim()
        rate.sleep()
        t += dt


if __name__ == "__main__":
    import argparse 
    parser = argparse.ArgumentParser(description="Shadow Hand DiffIK for VisionPro")
    parser.add_argument("--ip", type=str, required=True)
    args = parser.parse_args()
    main(args)