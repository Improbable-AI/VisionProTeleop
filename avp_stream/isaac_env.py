import isaacgym
import torch 
from isaacgym import gymapi
from isaacgym import gymutil
from isaacgym import gymtorch

import numpy as np
import torch
import time
from pathlib import Path

from avp_stream import VisionProStreamer
from avp_stream.utils.isaac_utils import * 
from avp_stream.utils.se3_utils import * 
from avp_stream.utils.trn_constants import * 
from copy import deepcopy
from typing import * 

CUR_PATH = Path(__file__).parent.resolve()

class IsaacVisualizerEnv: 

    def __init__(self, args):

        self.args = args 

        # acquire gym interface
        self.gym = gymapi.acquire_gym()
 
        # set torch device
        self.device = 'cpu'  # i'll just fix this to CUDA 

        # configure sim
        self.sim_params = default_sim_params(use_gpu = True if self.device == 'cuda:0' else False) 

        # create sim
        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, self.sim_params)
        if self.sim is None:
            raise Exception("Failed to create sim")
        
        # load assets
        self.num_envs = 1

        # create viewer
        self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())
        if self.viewer is None:
            raise Exception("Failed to create viewer")

        # create env 
        self._load_asset()
        self.create_env() 

        # setup viewer camera
        middle_env = self.num_envs // 2
        setup_viewer_camera(self.gym, self.envs[middle_env], self.viewer)

        # ==== prepare tensors =====
        # from now on, we will use the tensor API that can run on CPU or GPU
        self.gym.prepare_sim(self.sim)
        self.initialize_tensors()



    def _load_asset(self):

        self.axis = load_axis(self.gym, self.sim, self.device, 'normal', f'{CUR_PATH}/assets')
        self.small_axis = load_axis(self.gym, self.sim, self.device, 'small', f'{CUR_PATH}/assets')
        self.huge_axis = load_axis(self.gym, self.sim, self.device, 'huge', f'{CUR_PATH}/assets')

        asset_options = gymapi.AssetOptions()
        asset_options.disable_gravity = True
        asset_options.fix_base_link = True
        self.sphere = self.gym.create_sphere(self.sim, 0.008, asset_options)


    def create_env(self):
        spacing = 1.0
        env_lower = gymapi.Vec3(-spacing, -spacing, 0.0)
        env_upper = gymapi.Vec3(spacing, spacing, spacing)

        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0, 0, 1)
        self.gym.add_ground(self.sim, plane_params)

        # create env
        self.envs = []
        self.robot_actor_idxs_over_sim = [] 
        self.env_side_actor_idxs_over_sim = []

        for env_idx in range(self.num_envs):
            env = self.gym.create_env(self.sim, env_lower, env_upper, 1)
            self.envs.append(env)

            self.head_axis = self.gym.create_actor(env, self.axis, gymapi.Transform(), 'head', 0)

            self.right_wrist_axis = self.gym.create_actor(env, self.axis, gymapi.Transform(), 'right_wrist', 1)
            self.left_wrist_axis = self.gym.create_actor(env, self.axis, gymapi.Transform(), 'left_wrist', 2)


            # SPHERE
            for i in range(25): 
                
                finger_1 = self.gym.create_actor(env, self.sphere, gymapi.Transform(), f'right_finger_{i}', 3 + i )
                if i in [0, 4, 9, 14, 19, 24]:
                    self.gym.set_rigid_body_color(env, finger_1, 0, gymapi.MESH_VISUAL_AND_COLLISION, gymapi.Vec3(1, 1, 0))
                else:
                    self.gym.set_rigid_body_color(env, finger_1, 0, gymapi.MESH_VISUAL_AND_COLLISION, gymapi.Vec3(1, 1, 1))

            for i in range(25):
                finger_2 = self.gym.create_actor(env, self.sphere, gymapi.Transform(), f'left_finger_{i}', 28 + i )

                if i in [0, 4, 9, 14, 19, 24]:
                    self.gym.set_rigid_body_color(env, finger_2, 0, gymapi.MESH_VISUAL_AND_COLLISION, gymapi.Vec3(1, 1, 0))
                else:
                    self.gym.set_rigid_body_color(env, finger_2, 0, gymapi.MESH_VISUAL_AND_COLLISION, gymapi.Vec3(1, 1, 1))

            # SMALL AXIS
            for i in range(25): 
                finger_1 = self.gym.create_actor(env, self.small_axis, gymapi.Transform(), f'right_finger_{i}', 53 + i  )

            for i in range(25):
                finger_2 = self.gym.create_actor(env, self.small_axis, gymapi.Transform(), f'left_finger_{i}', 78 + i )

            self.env_axis = self.gym.create_actor(env, self.huge_axis, gymapi.Transform(), 'env_axis', 103 )


    def initialize_tensors(self): 
        
        refresh_tensors(self.gym, self.sim)
        # get jacobian tensor
        # get rigid body state tensor
        _rb_states = self.gym.acquire_rigid_body_state_tensor(self.sim)
        self.rb_states = gymtorch.wrap_tensor(_rb_states).view(self.num_envs, -1, 13)

        # get actor root state tensor
        _root_states = self.gym.acquire_actor_root_state_tensor(self.sim)
        root_states = gymtorch.wrap_tensor(_root_states).view(self.num_envs, -1, 13)
        self.root_state = root_states

        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)
        self.gym.step_graphics(self.sim)
        self.gym.draw_viewer(self.viewer, self.sim, False)
        self.gym.sync_frame_time(self.sim)

    # will be overloaded
    def step(self, transformation: Dict[str, torch.Tensor], sync_frame_time = False): 

        self.simulate()

        new_root_state = self.modify_root_state(transformation)
        env_side_actor_idxs = torch.arange(0, 103, dtype = torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(self.sim, gymtorch.unwrap_tensor(new_root_state), gymtorch.unwrap_tensor(env_side_actor_idxs), len(env_side_actor_idxs))

        # update viewer
        self.render(sync_frame_time)

    def move_camera(self):

        head_xyz = self.visionos_head[:, :3, 3]
        head_ydir = self.visionos_head[:, :3, 1]

        cam_pos = head_xyz - head_ydir * 0.5
        cam_target = head_xyz + head_ydir * 0.5
        cam_target[..., -1] -= 0.2

        cam_pos = gymapi.Vec3(*cam_pos[0])
        cam_target = gymapi.Vec3(*cam_target[0])

        self.gym.viewer_camera_look_at(self.viewer, self.envs[0], cam_pos, cam_target)

    def simulate(self): 
        # step the physics
        self.gym.simulate(self.sim)

        # refresh tensors
        refresh_tensors(self.gym, self.sim)


    def render(self, sync_frame_time = True): 

        # update viewer
        if self.args.follow:
            self.move_camera()
        self.gym.step_graphics(self.sim)
        self.gym.draw_viewer(self.viewer, self.sim, False)
        if sync_frame_time:
            self.gym.sync_frame_time(self.sim)

    def modify_root_state(self, transformations): 

        new_root_state = self.root_state

        self.visionos_head = transformations['head'] 
        
        self.sim_right_wrist = transformations['right_wrist'] #@ VISIONOS_RIGHT_HAND_TO_LEAP 
        self.sim_left_wrist = transformations['left_wrist'] # @ VISIONOS_LEFT_HAND_TO_LEAP

        sim_right_fingers = torch.cat([self.sim_right_wrist @ finger for finger in transformations['right_fingers']], dim = 0)
        sim_left_fingers = torch.cat([self.sim_left_wrist @ finger for finger in transformations['left_fingers']], dim = 0)

        self.sim_right_fingers = sim_right_fingers 
        self.sim_left_fingers = sim_left_fingers 

        new_root_state = deepcopy(self.root_state)
        new_root_state[:, 0, :7] = mat2posquat(self.visionos_head )
        new_root_state[:, 1, :7] = mat2posquat(self.sim_right_wrist )
        new_root_state[:, 2, :7] = mat2posquat(self.sim_left_wrist )
        new_root_state[:, 3:28, :7] = mat2posquat(self.sim_right_fingers )#  
        new_root_state[:, 28:53, :7] = mat2posquat(self.sim_left_fingers )# 
        new_root_state[:, 53:78, :7] = mat2posquat(self.sim_right_fingers)#
        new_root_state[:, 78:103, :7] = mat2posquat(self.sim_left_fingers )
        # new_root_state[:, 103, :7] = mat2posquat(transformed_wrist_right)
        new_root_state = new_root_state.view(-1, 13)

        return new_root_state


def np2tensor(data: Dict[str, np.ndarray], device) -> Dict[str, torch.Tensor]:  
    for key in data.keys():
        data[key] = torch.tensor(data[key], dtype = torch.float32, device = device)
    return data


if __name__=="__main__": 

    import argparse 
    import os 

    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', type = str, required = True)
    parser.add_argument('--record', action = 'store_true')
    parser.add_argument('--follow', action = 'store_true', help = "The viewpoint follows the users head")
    args = parser.parse_args()

    s = VisionProStreamer(args.ip, args.record)

    env = IsaacVisualizerEnv(args)
    while True: 
        t0 = time.time()
        latest = s.latest
        env.step(np2tensor(latest, env.device)) 
        print(time.time() - t0)


