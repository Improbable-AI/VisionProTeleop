"""Teleoperate ALOHA bimanual robot using differential IK with hand tracking.

Requires: pip install 'mink[examples]'
Adapted from https://github.com/kevinzakka/mink
"""

import os
import sys
from pathlib import Path
from typing import List, Optional, Sequence

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
_XML = os.path.join(_HERE, "..", "assets", "mujoco_demos", "aloha", "scene.xml")

# Single arm joint names.
_JOINT_NAMES = [
    "waist",
    "shoulder",
    "elbow",
    "forearm_roll",
    "wrist_angle",
    "wrist_rotate",
]

# Single arm velocity limits, taken from:
# https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf/vx300s.urdf.xacro
_VELOCITY_LIMITS = {k: np.pi for k in _JOINT_NAMES}


def hand2pose(hand, side = "right", euler = [0, 0, 0] ): 

    wrist = hand[f"{side}_wrist"]
    finger = wrist @ hand[f"{side}_fingers"]

    pinch_distance = hand["right_pinch_distance"]

    thumb_tip = finger[4, :3, -1]
    thumb_base = finger[2, :3, -1]
    index_tip = finger[9, :3, -1]
    index_base = finger[7, :3, -1]

    base_middle = (thumb_base + index_base) * 0.5
    tip_middle = (thumb_tip + index_tip) * 0.5

    z_axis = tip_middle - base_middle
    z_axis /= np.linalg.norm(z_axis)

    
    # use index→thumb direction as x
    x_axis = thumb_base - index_base
    x_axis -= np.dot(x_axis, z_axis) * z_axis   # make y ⟂ z
    x_axis /= np.linalg.norm(x_axis)

    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    rot = np.column_stack((x_axis, y_axis, z_axis))
    rot /= np.cbrt(np.linalg.det(rot))  # ensure det = 1

    rot = rot @ R.from_euler("xyz", euler, degrees=True).as_matrix() 

    mat = np.eye(4) 
    mat[:3, :3] = rot
    mat[:3, 3] = tip_middle

    return mat, pinch_distance





def compensate_gravity(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    subtree_ids: Sequence[int],
    qfrc_applied: Optional[np.ndarray] = None,
) -> None:
    """Compute forces to counteract gravity for the given subtrees.

    Args:
        model: Mujoco model.
        data: Mujoco data.
        subtree_ids: List of subtree ids. A subtree is defined as the kinematic tree
            starting at the body and including all its descendants. Gravity
            compensation forces will be applied to all bodies in the subtree.
        qfrc_applied: Optional array to store the computed forces. If not provided,
            the applied forces in `data` are used.
    """
    qfrc_applied = data.qfrc_applied if qfrc_applied is None else qfrc_applied
    qfrc_applied[:] = 0.0  # Don't accumulate from previous calls.
    jac = np.empty((3, model.nv))
    for subtree_id in subtree_ids:
        total_mass = model.body_subtreemass[subtree_id]
        mujoco.mj_jacSubtreeCom(model, data, jac, subtree_id)
        qfrc_applied[:] -= model.opt.gravity * total_mass @ jac


def main(args): 


    model = mujoco.MjModel.from_xml_path(str(_XML))
    data = mujoco.MjData(model)

    from avp_stream import VisionProStreamer

    streamer = VisionProStreamer(ip=args.ip) 

    streamer.configure_sim(
        xml_path=str(_XML),
        model=model,
        data=data,
        relative_to=[0, 0.1, 0.6, 0],
        force_reload=False
    )
    streamer.start_webrtc()


    # Bodies for which to apply gravity compensation.
    left_subtree_id = model.body("left/base_link").id
    right_subtree_id = model.body("right/base_link").id

    # Get the dof and actuator ids for the joints we wish to control.
    joint_names: List[str] = []
    velocity_limits: dict[str, float] = {}
    for prefix in ["left", "right"]:
        for n in _JOINT_NAMES:
            name = f"{prefix}/{n}"
            joint_names.append(name)
            velocity_limits[name] = _VELOCITY_LIMITS[n]
    dof_ids = np.array([model.joint(name).id for name in joint_names])
    actuator_ids = np.array([model.actuator(name).id for name in joint_names])
    print(actuator_ids)

    configuration = mink.Configuration(model)

    tasks = [
        l_ee_task := mink.FrameTask(
            frame_name="left/gripper",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=0.3,
            lm_damping=1.0,
        ),
        r_ee_task := mink.FrameTask(
            frame_name="right/gripper",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=0.3,
            lm_damping=1.0,
        ),
        posture_task := mink.PostureTask(model, cost=1e-4),
    ]

    # Enable collision avoidance between the following geoms.
    l_wrist_geoms = mink.get_subtree_geom_ids(model, model.body("left/wrist_link").id)
    r_wrist_geoms = mink.get_subtree_geom_ids(model, model.body("right/wrist_link").id)
    l_geoms = mink.get_subtree_geom_ids(model, model.body("left/upper_arm_link").id)
    r_geoms = mink.get_subtree_geom_ids(model, model.body("right/upper_arm_link").id)
    frame_geoms = mink.get_body_geom_ids(model, model.body("metal_frame").id)
    collision_pairs = [
        (l_wrist_geoms, r_wrist_geoms),
        (l_geoms + r_geoms, frame_geoms + ["table"]),
    ]
    collision_avoidance_limit = mink.CollisionAvoidanceLimit(
        model=model,
        geom_pairs=collision_pairs,  # type: ignore
        minimum_distance_from_collisions=0.05,
        collision_detection_distance=0.1,
    )

    limits = [
        mink.ConfigurationLimit(model=model),
        mink.VelocityLimit(model, velocity_limits),
        collision_avoidance_limit,
    ]

    l_mid = model.body("left/target").mocapid[0]
    r_mid = model.body("right/target").mocapid[0]
    solver = "daqp"
    pos_threshold = 5e-3
    ori_threshold = 5e-3
    max_iters = 5



    # Initialize to the home keyframe.
    mujoco.mj_resetDataKeyframe(model, data, model.key("neutral_pose").id)
    configuration.update(data.qpos)
    mujoco.mj_forward(model, data)
    posture_task.set_target_from_configuration(configuration)

    # Initialize mocap targets at the end-effector site.
    mink.move_mocap_to_frame(model, data, "left/target", "left/gripper", "site")
    mink.move_mocap_to_frame(model, data, "right/target", "right/gripper", "site")

    rate = RateLimiter(frequency=200.0, warn=False)

    while True: 
        # Update task targets.
        l_ee_task.set_target(mink.SE3.from_mocap_name(model, data, "left/target"))
        r_ee_task.set_target(mink.SE3.from_mocap_name(model, data, "right/target"))


        mocap_ids = [l_mid, r_mid]
        hand = streamer.get_latest()

        hand_right_pose, right_pinch = hand2pose(hand, side="right", euler=[0, -90, -90])
        hand_left_pose, left_pinch = hand2pose(hand, side="left", euler=[0, -90, 90])

        data.mocap_pos[l_mid, :] = hand_left_pose[:3, 3]
        data.mocap_quat[l_mid, :] = R.from_matrix(hand_left_pose[:3, :3]).as_quat(scalar_first=True)

        data.mocap_pos[r_mid, :] = hand_right_pose[:3, 3]
        data.mocap_quat[r_mid, :] = R.from_matrix(hand_right_pose[:3, :3]).as_quat(scalar_first=True)




        vel = mink.solve_ik(
            configuration,
            tasks,
            rate.dt,
            solver,
            limits=limits,
            damping=1e-5,
        )
        configuration.integrate_inplace(vel, rate.dt)


        data.qpos[dof_ids] = configuration.q[dof_ids]
        data.qpos[6:8] = np.clip(left_pinch , 0.01, 0.04)
        data.qpos[14:16] = np.clip(right_pinch , 0.01, 0.04)
        mujoco.mj_forward(model, data)

        streamer.update_sim()

        # rate.sleep()


if __name__ == "__main__":

    import argparse 
    parser = argparse.ArgumentParser(
        description="Mujoco Aloha DiffIK with VisionPro Hand Tracking"
    )
    parser.add_argument("--ip", type=str, required=True)
    args = parser.parse_args()

    main(args)
