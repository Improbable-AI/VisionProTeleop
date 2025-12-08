import numpy as np
import torch
import os
from isaaclab.app import AppLauncher
import gymnasium as gym
import numpy as np 

import argparse
# add argparse arguments
parser = argparse.ArgumentParser(
    description="This script demonstrates adding a custom robot to an Isaac Lab environment."
)
parser.add_argument("--num-envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--task", type=str, default="nonprehensile", help="Task to run.")
parser.add_argument("--ip", type=str, default="127.0.0.1", help="IP address of the Vision Pro device.")

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
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.sensors import CameraCfg, OffsetCfg, ContactSensorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR
from isaaclab.actuators import ImplicitActuatorCfg, IdealPDActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.math import matrix_from_quat, quat_from_matrix  
from isaaclab.sim.spawners.materials import RigidBodyMaterialCfg
import matplotlib.pyplot as plt
from tqdm import trange
from isaaclab.utils.math import apply_delta_pose, compute_pose_error, axis_angle_from_quat, quat_from_angle_axis, euler_xyz_from_quat, quat_from_euler_xyz, quat_box_plus, quat_error_magnitude, quat_apply
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
import isaaclab.sim as sim_utils

import torch
import tyro
import gymnasium as gym
from scipy.spatial.transform import Rotation as R
import os 
from collections import deque
import pathlib

CUR_PATH = pathlib.Path(__file__).parent.resolve()
import os


np.random.seed(0)

download_path = f"{ISAAC_NUCLEUS_DIR}/Robots/Franka/FR3/fr3.usd".replace("5.1", "4.5")
print(download_path)

FRANKA_FR3_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=download_path,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=12, solver_velocity_iteration_count=1
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "fr3_joint1": 0.0,
            "fr3_joint2": 0.0,
            "fr3_joint3": 0.0,
            "fr3_joint4": -1.5707899999999999,
            "fr3_joint5": 0.0,
            "fr3_joint6": 1.5707899999999999,
            "fr3_joint7": -0.7853,
            "fr3_finger_joint.*": 0.04,
        },
        # pos=(0.0, -1.0, 0.0), 

    ), 
	actuators={
    "fr3_joint1": ImplicitActuatorCfg(
        joint_names_expr=["fr3_joint1"],
        effort_limit_sim=87.0,
        velocity_limit_sim=1000.0,
        stiffness=315.6198926951318, 
        damping=40.37177601314333,
        armature=0.32639890937508514,
        # friction=0.1980493933180444,
        # dynamic_friction=0.2226116283694231, 
        # viscous_friction=0.11542705310763021,
    ),
    "fr3_joint2": ImplicitActuatorCfg(
        joint_names_expr=["fr3_joint2"],
        effort_limit_sim=87.0,
        velocity_limit_sim=1000.0,
        stiffness=215.30227057923668, 
        damping=17.411410568828444, 
        armature=0.4997860511637881, 
        # friction=0.26773675570806105, 
        # dynamic_friction=0.14435867015706552,
        # viscous_friction=0.26782766387192347,
    ),
    "fr3_joint3": ImplicitActuatorCfg(
        joint_names_expr=["fr3_joint3"],
        effort_limit_sim=87.0,
        velocity_limit_sim=1000.0, 
        stiffness=316.82162179150953, #joints[2][1],
        damping=39.53866400471302, #joints[2][2],
        armature=0.45932007034254824, #0.195,
        # friction=0.22740109363637268, #1.137,
        # dynamic_friction=0.365030031370259,
        # viscous_friction=0.3020249440909233,
    ),
    "fr3_joint4": ImplicitActuatorCfg(
        joint_names_expr=["fr3_joint4"],
        effort_limit_sim=87.0,
        velocity_limit_sim=1000.0,
        stiffness=344.59722994952625, #joints[3][1],
        damping=28.197981157703317, #joints[3][2],
        armature=0.49379348707571874, #0.195,
        # friction=0.15042498580023086, #1.137,
        # dynamic_friction=0.2765144827094045,
        # viscous_friction=0.3134558941952869,
    ),
    "fr3_joint5": ImplicitActuatorCfg(
        joint_names_expr=["fr3_joint5"],
        effort_limit_sim=12.0,
        velocity_limit_sim=1000.0,
        stiffness=194.95359554487283, #joints[4][1],
        damping=14.55069680027277, #joints[4][2],
        armature=0.47895310622724685, #0.074,
        # friction=0.265526083347621, #0.763,
        # dynamic_friction=0.11975422661312576,
        # viscous_friction=0.30937850143651086,
    ),
    "fr3_joint6": ImplicitActuatorCfg(
        joint_names_expr=["fr3_joint6"],
        effort_limit_sim=12.0,
        velocity_limit_sim=1000.0,
        stiffness=210.12208357052535, #joints[5][1],
        damping=14.89840090146294, #joints[5][2],
        armature=0.4809130149251913, #0.074,
        # friction=0.3004278459031679, #0.44,
        # dynamic_friction=0.1889105579712283,
        # viscous_friction=0.15543581043789634
    ),
    "fr3_joint7": ImplicitActuatorCfg(
        joint_names_expr=["fr3_joint7"],
        effort_limit_sim=12.0,
        velocity_limit_sim=1000.0,
        stiffness=116.48132566212718, #joints[6][1],
        damping=8.95965596447379, #joints[6][2],
        armature=0.2250284214884057, #0.074,
        # friction=0.07339105988761851, #0.248,
        # dynamic_friction=0.3511542689173363,
        # viscous_friction=0.1705076549611242,
    ),
    "fr3_hand": ImplicitActuatorCfg(
            joint_names_expr=["fr3_finger_joint.*"],
            effort_limit_sim=200.0,
            velocity_limit_sim=1000.0,
            stiffness=2e3,
            damping=1e2,
        ),
    },
soft_joint_pos_limit_factor=1.0,
)

# download_path = f"{ISAAC_NUCLEUS_DIR}/Robots/FrankaRobotics/FrankaPanda/franka.usd"
# print(download_path)
# save_path = f"{CUR_PATH}/../assets/fr3.usd"

# if not os.path.exists(save_path):
#     os.makedirs(os.path.dirname(save_path), exist_ok=True)
#     if not os.path.exists(download_path):
#         print("Downloading FR3 robot from Isaac Nucleus...")
#         os.system(f"wget -O {save_path} {download_path}")
#     print("Copying FR3 robot to local assets...")
#     # os.system(f"cp {download_path} {save_path}")
#     print("FR3 robot saved to:", save_path)



# FRANKA_FR3_CFG.spawn.usd_path = download_path
# FRANKA_FR3_CFG.spawn.variants = {"Gripper": "Robotiq_2F_85"}
# FRANKA_FR3_CFG.spawn.rigid_props.disable_gravity = True
# FRANKA_FR3_CFG.init_state.joint_pos = {
#     "panda_joint1": 0.0,
#     "panda_joint2": -0.569,
#     "panda_joint3": 0.0,
#     "panda_joint4": -2.810,
#     "panda_joint5": 0.0,
#     "panda_joint6": 3.037,
#     "panda_joint7": 0.741,
#     "finger_joint": 0.0,
#     ".*_inner_finger_joint": 0.0,
#     ".*_inner_finger_knuckle_joint": 0.0,
#     ".*_outer_.*_joint": 0.0,
# }
# FRANKA_FR3_CFG.init_state.pos = (-0.85, 0, 0.76)
# FRANKA_FR3_CFG.actuators = {
#     "panda_shoulder": ImplicitActuatorCfg(
#         joint_names_expr=["panda_joint[1-4]"],
#         effort_limit_sim=5200.0,
#         velocity_limit_sim=2.175,
#         stiffness=1100.0,
#         damping=80.0,
#     ),
#     "panda_forearm": ImplicitActuatorCfg(
#         joint_names_expr=["panda_joint[5-7]"],
#         effort_limit_sim=720.0,
#         velocity_limit_sim=2.61,
#         stiffness=1000.0,
#         damping=80.0,
#     ),
#     "gripper_drive": ImplicitActuatorCfg(
#         joint_names_expr=["finger_joint"],  # "right_outer_knuckle_joint" is its mimic joint
#         effort_limit_sim=1650,
#         velocity_limit_sim=10.0,
#         stiffness=17,
#         damping=0.02,
#     ),
#     # enable the gripper to grasp in a parallel manner
#     "gripper_finger": ImplicitActuatorCfg(
#         joint_names_expr=[".*_inner_finger_joint"],
#         effort_limit_sim=50,
#         velocity_limit_sim=10.0,
#         stiffness=0.2,
#         damping=0.001,
#     ),
#     # set PD to zero for passive joints in close-loop gripper
#     "gripper_passive": ImplicitActuatorCfg(
#         joint_names_expr=[".*_inner_finger_knuckle_joint", "right_outer_knuckle_joint"],
#         effort_limit_sim=1.0,
#         velocity_limit_sim=10.0,
#         stiffness=0.0,
#         damping=0.0,
#     ),
# }


class NewRobotsSceneCfg(InteractiveSceneCfg):
    """Designs the scene."""

    # Ground-plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg(size = (10.0, 10.0), 
    physics_material = RigidBodyMaterialCfg(static_friction = 1.0, dynamic_friction = 1.0)))

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )


    Franka = FRANKA_FR3_CFG.replace(prim_path="{ENV_REGEX_NS}/Franka")\
                                .replace(init_state = ArticulationCfg.InitialStateCfg(
                                                joint_pos={
                                                    "fr3_joint1": 0.0,
                                                    "fr3_joint2": 0.0,
                                                    "fr3_joint3": 0.0,
                                                    "fr3_joint4": -1.5707899999999999,
                                                    "fr3_joint5": 0.0,
                                                    "fr3_joint6": 1.5707899999999999,
                                                    "fr3_joint7": -0.7853,
                                                    "fr3_finger_joint.*": 0.04,
                                                },
                                                pos=(0.4, -0.15, 0.0), 
                                                rot=(0.7071068 , 0, 0, 0.7071068),
                                ))


    Object = RigidObjectCfg(
        prim_path="/World/envs/env_.*/object",
        spawn=sim_utils.CuboidCfg(
                size = (0.05, 0.05, 0.05),
                mass_props=sim_utils.MassPropertiesCfg(mass = 0.5),
                collision_props = sim_utils.CollisionPropertiesCfg(
                    collision_enabled = True),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=4,
                    # max_angular_velocity=1000.0,
                    # max_linear_velocity=1000.0,
                    # max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
                activate_contact_sensors=True,
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.1, 0.39, 0.05), rot=(1.0, 0.0, 0.0, 0.0)),
    )

    Target_Pose = RigidObjectCfg(
        prim_path="/World/envs/env_.*/target_pose",
        spawn=sim_utils.CuboidCfg(
                size = (0.05, 0.05, 0.05),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    rigid_body_enabled = True, 
                    disable_gravity=True,
                ),
                collision_props = sim_utils.CollisionPropertiesCfg(
                    collision_enabled = False),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)), #, opacity = 0.9),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.1, 0.39, 0.3), rot=(1.0, 0.0, 0.0, 0.0)),
    )






class MyIsaacEnv: 
    def __init__(self, args_cli): 

        self.args_cli = args_cli

        gravity_direction = np.array([0, 0, -1])
        magnitude = 9.81
        gravity = magnitude * gravity_direction / np.linalg.norm(gravity_direction)

        self.sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt = 0.01, gravity=gravity)
        self.sim_cfg.physx.gpu_max_rigid_patch_count = 4096 * 4096
        self.sim = sim_utils.SimulationContext(self.sim_cfg)
        # self.sim.physx_cfg.gpu_max_rigid_patch_count = 4096 * 4096        
        self.sim.set_camera_view([3.5, 0.0, 3.2], [0.0, 0.0, 0.5])
        self.decimation = 2
        # Design scene
        self.scene_cfg = NewRobotsSceneCfg(self.args_cli.num_envs, env_spacing=2.0, replicate_physics = True)
        self.scene = InteractiveScene(self.scene_cfg)

        self.sim.reset()


        # self.init_pos = torch.tensor([0.0, 0.0, 0.0, -1.5707899999999999, 0.0, 1.5707899999999999, -0.7853]).to(args_cli.device).unsqueeze(0)
        self.init_pos = torch.tensor([0.0, 0.4, -0.32, -2.0, 0.15, 2.5, 0.145]).to(args_cli.device).unsqueeze(0)

        joint_pos = torch.zeros(9) 
        joint_pos[:7] = self.init_pos.clone()
        joint_vel = torch.zeros_like(joint_pos) 

        self.scene["Franka"].write_joint_state_to_sim(joint_pos, joint_vel)

        self.target_ee_mat = torch.eye(3).to(args_cli.device).unsqueeze(0).repeat(self.scene_cfg.num_envs, 1, 1)
        # [ 0, 1, 0]
        # [ 1, 0, 0]
        # [ 0, 0, -1]
        self.target_ee_mat[:, :3, 0] = torch.tensor([1.0, 0.0, 0.0]).to(args_cli.device)
        self.target_ee_mat[:, :3, 1] = torch.tensor([0.0, -1.0, 0.0]).to(args_cli.device)
        self.target_ee_mat[:, :3, 2] = torch.tensor([0.0, 0.0, -1.0]).to(args_cli.device)\

        self.target_ee_quat = quat_from_matrix(self.target_ee_mat)
        self.scene.reset()

    def step(self, action):
        """
        action: (num_envs, 8)

        """

        def compute_action(per_action, robot_name = "Franka"): 


            new_action = torch.zeros(self.scene_cfg.num_envs, 9).to(self.args_cli.device)
            # new_action[:, :7] = self._process_actions(action[:, :7])
            new_action[:, :7] = per_action[:, :7]
            # new_action[:, :7] = dq + cur_qpos[:, :7]  #- self.init_pos[:, :7]
            binary_action = torch.where(per_action[:, 7] < 0.0, torch.zeros_like(per_action[:, 7]), 0.04 * torch.ones_like(per_action[:, 7]))
            new_action[:, 7] = binary_action
            new_action[:, 8] = binary_action

            return new_action 


        right_action = compute_action(action, robot_name = "Franka")


        for _ in range(self.decimation): 
            self.scene["Franka"].set_joint_position_target(right_action)
            # self.scene["Left_Franka"].set_joint_position_target(left_action)
            self.scene.write_data_to_sim()
            self.sim.step()
            self.scene.update(self.sim.get_physics_dt())


if __name__ == "__main__":

    from avp_stream import VisionProStreamer


    my_env = MyIsaacEnv(args_cli)

    # Get stage and configure Isaac Lab streaming
    stage = omni.usd.get_context().get_stage()
    streamer = VisionProStreamer(ip=args.ip)
    streamer.configure_isaac(
        stage=stage,
        relative_to=[0, 0, 0.7, 90],  # Position the robot in front of the user
        include_ground=False,         # Don't include ground plane
        env_indices=[0],              # Only stream the first environment
    )
    streamer.start_webrtc()
    


    while True: 
        action = torch.zeros(args_cli.num_envs, 8).to(args_cli.device)
        action[:, :7] = my_env.init_pos + 0.1 * (torch.rand_like(my_env.init_pos) - 0.5)
        action[:, 7] = 1.0
        my_env.step(action)
        streamer.update_sim()